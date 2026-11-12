#!/usr/bin/env python3
"""Timing bench logger: ZED-F9P + LIS3DH on the ROCK Pi X, every event on one clock.

Wiring (README section 5):
  ZED-F9P PPS  -> header pin 7   (gpiochip2 line 21)
  LIS3DH INT1  -> header pin 29  (gpiochip2 line 24)  and  ZED-F9P INT (EXTINT)
  tap switch   -> header pin 31  (gpiochip2 line 15), other side to 3.3 V   (T5 only)
  ZED-F9P I2C  -> pins 3/5       LIS3DH I2C -> pins 27/28

The kernel time-stamps every pin edge in its interrupt handler (CLOCK_MONOTONIC).
The PPS edges map that clock to GPS time afterwards (bench_analyze.py). The
ZED-F9P also time-stamps the LIS3DH data-ready edges itself (UBX-TIM-TM2), a
reference that does not depend on Linux. While logging, a PM QoS request keeps
the CPUs out of the deep C-states whose wake-up delays the interrupts by up to
~1 ms (needs the udev rule from setup_i2c_gpio.sh).

  python3 bench_logger.py check          # 15 s wiring and timing check
  python3 bench_logger.py T1             # run a test (T1..T5), see --help
  python3 bench_analyze.py ~/bench/T1-20261103-160000

Each run writes a directory under --out (default ~/bench):
  meta.json      settings, phases, counters
  accel.csv      seqno,drdy_ns,read_ns,status,x,y,z   (LIS3DH, raw left-justified counts)
                 FIFO mode (default): seqno = sample number, drdy_ns = 0 (bench_analyze.py
                 interpolates the sample times from edges.csv), read_ns = end of the batch read,
                 status = 0x08, or 0x88 when the FIFO was full.
                 --no-fifo: seqno/drdy_ns = the data-ready edge; both -1 = rescue read, INT1 was
                 stuck high without an edge
  edges.csv      (FIFO mode) mono_ns,seqno,popped,fss,read_ns   one row per FIFO watermark edge:
                 kernel time stamp, kernel line seqno, samples read before it, FIFO fill seen by
                 the read that followed, time of that read
  pps.csv        mono_ns,seqno
  switch.csv     mono_ns,seqno,rising                 (T5)
  temp.csv       mono_ns,cpu_mc,lis3dh_raw
  markers.csv    mono_ns,elapsed_s,qos,load,label     (phase changes)
  gnss.ubx       raw receiver stream (RINEX-convertible with RTKLIB)
  gnss.csv       arrival_ns of RAWX / TIM-TM2 / TIM-TP / GGA with their fields
"""

import argparse
import json
import multiprocessing as mp
import os
import platform
import select
import signal
import struct
import subprocess
import sys
import time

import lis3dh
import zedf9p

GPIO_CHIP = "/dev/gpiochip2"  # INT33FF:02
# Header pin -> line on gpiochip2. The Radxa wiki numbers are from an older kernel
# where this chip started at 314: pin 7 = 335, pin 29 = 338, pin 31 = 329.
LINE_PPS, LINE_DRDY, LINE_SWITCH = 21, 24, 15
ACCEL_BUS = "/dev/i2c-header-27-28"
GNSS_BUS = "/dev/i2c-header-3-5"
QOS_DEVICE = "/dev/cpu_dma_latency"
QOS_LATENCY_US = 10  # Allows C1 (1 us wake-up); excludes C6N/C6S/C7/C7S (80 us .. 10 ms).
NS_PER_S = 1_000_000_000

LIS3DH_CTRL_REG3 = 0x22
LIS3DH_I1_ZYXDA = 0x10      # Data-ready on INT1.
LIS3DH_I1_WTM = 0x04        # FIFO watermark on INT1: high while the FIFO holds >= threshold samples.
LIS3DH_CTRL_REG5 = 0x24
LIS3DH_FIFO_EN = 0x40
LIS3DH_FIFO_CTRL = 0x2E
LIS3DH_FIFO_SRC = 0x2F      # bit 6 OVRN (FIFO full), bit 5 EMPTY, bits 4..0 unread samples
LIS3DH_FIFO_STREAM = 0x80   # Stream mode: the oldest sample is dropped when the FIFO is full.
FIFO_DEPTH = 32
DEFAULT_FIFO_THRESHOLD = 8
LIS3DH_TEMP_CFG = 0x1F
LIS3DH_ADC_TEMP_EN = 0xC0
LIS3DH_OUT_ADC3_L = 0x0C
LIS3DH_STATUS_AND_DATA = lis3dh.STATUS_REG | lis3dh.AUTO_INCREMENT  # 0x27..0x2D, 7 bytes

CFG_MSGOUT_TIM_TM2_I2C = 0x20910178
CFG_MSGOUT_TIM_TP_I2C = 0x2091017D
CFG_TP_TIMEGRID_TP1 = 0x2005000C  # 0 = UTC, 1 = GPS
UBX_TIM_TM2 = (0x0D, 0x03)
UBX_TIM_TP = (0x0D, 0x01)
TM2 = struct.Struct("<BBHHHIIIII")
TP = struct.Struct("<IIiHBB")

# drdy = data-ready edges, or FIFO watermark edges in FIFO mode; overruns = FIFO full in FIFO mode
COUNTERS = ("pps", "drdy", "overruns", "lost_events", "rescued", "switch", "rawx", "tm2",
            "gnss_bad_frames")
C = {name: i for i, name in enumerate(COUNTERS)}

# phases: (start s, PM QoS on, CPU load on, label)
STEADY = [(0, True, False, "PM QoS on")]
PRESETS = {
    "check": dict(duration=15, odr=100, gnss_rate=1, phases=STEADY,
                  about="wiring and timing check"),
    "T1": dict(duration=3600, odr=100, gnss_rate=1,
               phases=[(0, False, False, "idle, deep C-states allowed"),
                       (1200, True, False, "idle, PM QoS on"),
                       (2400, True, True, "PM QoS on, CPU load on all cores")],
               about="PPS vs Radxa clock: interrupt timestamp jitter without/with PM QoS, under load"),
    "T2": dict(duration=7200, odr=100, gnss_rate=1,
               phases=STEADY + [(6300, True, False,
                                 "WARM THE LIS3DH NOW (hand or hair dryer) until the end")],
               about="LIS3DH sample clock vs GPS time, warm-up in the last 15 min"),
    "T3": dict(duration=1800, odr=100, gnss_rate=10, phases=STEADY,
               about="timestamp paths (Python read vs kernel edge vs F9P time mark), GNSS 10 Hz"),
    "T4": dict(duration=3600, odr=400, gnss_rate=10, phases=STEADY,
               about="sample integrity at 400 Hz with GNSS 10 Hz"),
    "T5": dict(duration=600, odr=400, gnss_rate=1, phases=STEADY, switch=True,
               about="tap test: tap the microswitch ~30 times, 5-10 s apart"),
}


def log(message):
    print(time.strftime("%H:%M:%S ") + message, file=sys.stderr, flush=True)


def cpu_temp_zone():
    zones = "/sys/class/thermal"
    for name in sorted(os.listdir(zones)):
        try:
            with open(os.path.join(zones, name, "type")) as f:
                if f.read().strip() == "soc_dts0":
                    return os.path.join(zones, name, "temp")
        except OSError:
            pass
    return None


def read_int(path):
    try:
        with open(path) as f:
            return int(f.read())
    except (OSError, TypeError, ValueError):
        return -1


# --- worker: LIS3DH + GPIO edges --------------------------------------------------------------

def accel_worker(cfg, run_dir, stop, counters):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    import gpiod
    from gpiod.line import Bias, Direction, Edge, Value
    from smbus2 import SMBus, i2c_msg

    bus = SMBus(ACCEL_BUS)
    addr = lis3dh.DEFAULT_ADDRESS
    files, req = [], None

    def open_csv(name, header):
        f = open(os.path.join(run_dir, name), "w")
        f.write(header + "\n")
        files.append(f)
        return f

    try:
        lis3dh.LIS3DH(bus, addr, cfg["odr"], cfg["range"])  # BDU, high resolution, range, ODR
        bus.write_byte_data(addr, LIS3DH_TEMP_CFG, LIS3DH_ADC_TEMP_EN)
        fifo, fth = cfg["fifo"], cfg["fifo_threshold"]
        if fifo:
            # Stream-mode FIFO: the sensor buffers 32 samples (80 ms at 400 Hz), so a late read
            # loses nothing. INT1 = watermark: it rises when the FIFO reaches `fth` samples, an
            # edge the kernel time-stamps; the samples in between get interpolated times.
            bus.write_byte_data(addr, LIS3DH_FIFO_CTRL, 0)               # bypass: empties the FIFO
            bus.write_byte_data(addr, LIS3DH_CTRL_REG5, LIS3DH_FIFO_EN)
            bus.write_byte_data(addr, LIS3DH_FIFO_CTRL, LIS3DH_FIFO_STREAM | fth)
            bus.write_byte_data(addr, LIS3DH_CTRL_REG3, LIS3DH_I1_WTM)
        else:
            bus.write_byte_data(addr, LIS3DH_CTRL_REG3, LIS3DH_I1_ZYXDA)
        rising = gpiod.LineSettings(direction=Direction.INPUT, edge_detection=Edge.RISING)
        config = {(LINE_PPS, LINE_DRDY): rising}
        if cfg["switch"]:
            config[LINE_SWITCH] = gpiod.LineSettings(direction=Direction.INPUT,
                                                     edge_detection=Edge.BOTH, bias=Bias.PULL_DOWN)
        req = gpiod.request_lines(GPIO_CHIP, consumer="bench_logger", config=config,
                                  event_buffer_size=1024)
        accel = open_csv("accel.csv", "seqno,drdy_ns,read_ns,status,x,y,z")
        edges = open_csv("edges.csv", "mono_ns,seqno,popped,fss,read_ns") if fifo else None
        pps = open_csv("pps.csv", "mono_ns,seqno")
        switch = open_csv("switch.csv", "mono_ns,seqno,rising") if cfg["switch"] else None
        temp = open_csv("temp.csv", "mono_ns,cpu_mc,lis3dh_raw")
        zone = cpu_temp_zone()
        rising_type = gpiod.EdgeEvent.Type.RISING_EDGE

        def read_sample(seqno, drdy_ns):
            d = bus.read_i2c_block_data(addr, LIS3DH_STATUS_AND_DATA, 7)
            read_ns = time.monotonic_ns()
            x, y, z = struct.unpack("<hhh", bytes(d[1:7]))
            accel.write("%d,%d,%d,%d,%d,%d,%d\n" % (seqno, drdy_ns, read_ns, d[0], x, y, z))
            return d[0], read_ns

        if fifo:
            popped = 0

            def drain():
                """Read the FIFO until a FIFO_SRC read says EMPTY.

                Returns (fill at the first look, time of that look).
                """
                nonlocal popped
                first = None
                while True:
                    src = bus.read_byte_data(addr, LIS3DH_FIFO_SRC)
                    now = time.monotonic_ns()
                    n = src & 0x1F
                    full = bool(src & 0x40)
                    if full:
                        counters[C["overruns"]] += 1
                        n = n or FIFO_DEPTH   # 32 does not fit the 5-bit count
                    if first is None:
                        first = (n, now)
                    if n == 0:
                        return first
                    msg = i2c_msg.write(addr, [lis3dh.OUT_X_L | lis3dh.AUTO_INCREMENT])
                    rd = i2c_msg.read(addr, 6 * n)
                    bus.i2c_rdwr(msg, rd)
                    read_ns = time.monotonic_ns()
                    data = bytes(rd)
                    status = 0x88 if full else 0x08
                    for i in range(n):
                        x, y, z = struct.unpack_from("<hhh", data, 6 * i)
                        accel.write("%d,0,%d,%d,%d,%d,%d\n" % (popped, read_ns, status, x, y, z))
                        popped += 1

            drain()                          # Sample 0 is the first one after this look.
            ignore_before = time.monotonic_ns()
            wake_ms = max(1, 2 * fth * 1000 // cfg["odr"])
            poller = select.poll()
            poller.register(req.fd, select.POLLIN)
            last_seq = None
            next_slow = time.monotonic()
            while not stop.is_set():
                got_wtm = []
                if not poller.poll(wake_ms):
                    # No watermark edge for two thresholds' worth of samples: look anyway.
                    n, _ = drain()
                    if n >= fth:
                        counters[C["rescued"]] += 1
                else:
                    for ev in req.read_edge_events():
                        line = ev.line_offset
                        if line == LINE_DRDY:
                            if ev.timestamp_ns < ignore_before:
                                continue
                            got_wtm.append((ev.timestamp_ns, ev.line_seqno, popped))
                            counters[C["drdy"]] += 1
                            if last_seq is not None and ev.line_seqno > last_seq + 1:
                                counters[C["lost_events"]] += ev.line_seqno - last_seq - 1
                            last_seq = ev.line_seqno
                        elif line == LINE_PPS:
                            pps.write("%d,%d\n" % (ev.timestamp_ns, ev.line_seqno))
                            counters[C["pps"]] += 1
                        elif line == LINE_SWITCH and switch:
                            switch.write("%d,%d,%d\n" % (ev.timestamp_ns, ev.line_seqno,
                                                         ev.event_type == rising_type))
                            counters[C["switch"]] += 1
                    if got_wtm:
                        fss, look_ns = drain()
                        for ts, seq_no, before in got_wtm:
                            edges.write("%d,%d,%d,%d,%d\n" % (ts, seq_no, before, fss, look_ns))
                if time.monotonic() >= next_slow:
                    raw = bus.read_i2c_block_data(addr, LIS3DH_OUT_ADC3_L | lis3dh.AUTO_INCREMENT, 2)
                    temp.write("%d,%d,%d\n" % (time.monotonic_ns(), read_int(zone),
                                               struct.unpack("<h", bytes(raw))[0]))
                    for f in files:
                        f.flush()
                    next_slow += 1
            return

        # INT1 is a level: it drops when the sample is read and rises with the next one. If
        # the next sample lands while the previous is being read, INT1 stays high, no edge
        # follows and edge-driven reading would stop for good. Two periods without an edge
        # while INT1 is high means that happened: read once to re-arm it.
        stuck_ns = 2 * NS_PER_S // cfg["odr"]
        poll_ms = max(1, stuck_ns // 1_000_000)
        bus.read_i2c_block_data(addr, LIS3DH_STATUS_AND_DATA, 7)  # Clear INT1 so it re-arms.
        last_read = time.monotonic_ns()
        poller = select.poll()
        poller.register(req.fd, select.POLLIN)
        last_seq = None
        next_slow = time.monotonic()
        while not stop.is_set():
            if not poller.poll(poll_ms):
                if (req.get_value(LINE_DRDY) == Value.ACTIVE
                        and time.monotonic_ns() - last_read > stuck_ns):
                    _, last_read = read_sample(-1, -1)
                    counters[C["rescued"]] += 1
            else:
                for ev in req.read_edge_events():
                    line = ev.line_offset
                    if line == LINE_DRDY:
                        status, last_read = read_sample(ev.line_seqno, ev.timestamp_ns)
                        counters[C["drdy"]] += 1
                        if status & lis3dh.STATUS_ZYXOR:
                            counters[C["overruns"]] += 1
                        if last_seq is not None and ev.line_seqno > last_seq + 1:
                            counters[C["lost_events"]] += ev.line_seqno - last_seq - 1
                        last_seq = ev.line_seqno
                    elif line == LINE_PPS:
                        pps.write("%d,%d\n" % (ev.timestamp_ns, ev.line_seqno))
                        counters[C["pps"]] += 1
                    elif line == LINE_SWITCH and switch:
                        switch.write("%d,%d,%d\n" % (ev.timestamp_ns, ev.line_seqno,
                                                     ev.event_type == rising_type))
                        counters[C["switch"]] += 1
            if time.monotonic() >= next_slow:
                raw = bus.read_i2c_block_data(addr, LIS3DH_OUT_ADC3_L | lis3dh.AUTO_INCREMENT, 2)
                temp.write("%d,%d,%d\n" % (time.monotonic_ns(), read_int(zone),
                                           struct.unpack("<h", bytes(raw))[0]))
                for f in files:
                    f.flush()
                next_slow += 1
    finally:
        try:
            bus.write_byte_data(addr, LIS3DH_CTRL_REG3, 0)
            bus.write_byte_data(addr, LIS3DH_TEMP_CFG, 0)
            bus.write_byte_data(addr, lis3dh.CTRL_REG1, lis3dh.CTRL_REG1_XYZ_ON)  # Power down.
        except OSError:
            pass
        if req:
            req.release()
        for f in files:
            f.close()
        bus.close()


# --- worker: ZED-F9P ------------------------------------------------------------------------

def gnss_worker(cfg, run_dir, stop, counters):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    from smbus2 import SMBus

    bus = SMBus(GNSS_BUS)
    rx = zedf9p.Receiver(zedf9p.DDC(bus))
    saved = None
    raw = out = None
    try:
        version = rx.version()
        settings = {
            zedf9p.CFG_RATE_MEAS: round(1000 / cfg["gnss_rate"]),
            zedf9p.CFG_RATE_NAV: max(1, round(cfg["gnss_rate"])),  # Position fix ~1 Hz.
            zedf9p.CFG_MSGOUT_RXM_RAWX_I2C: 1,
            zedf9p.CFG_MSGOUT_RXM_SFRBX_I2C: 1,
            CFG_MSGOUT_TIM_TM2_I2C: 1,
            CFG_MSGOUT_TIM_TP_I2C: 1,
            CFG_TP_TIMEGRID_TP1: 1,  # PPS and time marks on the GPS time grid.
        }
        saved = rx.get(settings)
        rx.set(settings)
        with open(os.path.join(run_dir, "gnss_info.json"), "w") as f:
            json.dump({"version": version, "settings": {hex(k): v for k, v in settings.items()},
                       "saved": {hex(k): v for k, v in saved.items()}}, f, indent=1)
        raw = open(os.path.join(run_dir, "gnss.ubx"), "wb")
        out = open(os.path.join(run_dir, "gnss.csv"), "w")
        out.write("# rawx,arrival_ns,week,tow,leap_s,rec_stat,num_meas\n"
                  "# tm2,arrival_ns,ch,flags,count,wnR,wnF,towMsR,towSubMsR,towMsF,towSubMsF,accEst\n"
                  "# tp,arrival_ns,towMS,towSubMS,qErr,week,flags,refInfo\n"
                  "# gga,arrival_ns,quality,num_sv,hdop\n")
        next_flush = time.monotonic() + 1
        while not stop.is_set():
            data, messages = rx.pump()
            now = time.monotonic_ns()
            if data:
                raw.write(data)
            for m in messages:
                if m.kind == "NMEA":
                    if m.ident == "GGA":
                        g = m.data.decode("ascii", "replace").split(",")
                        if len(g) > 8:
                            out.write("gga,%d,%s,%s,%s\n" % (now, g[6], g[7], g[8]))
                elif m.ident == zedf9p.UBX_RXM_RAWX:
                    tow, week, leap, n, rec_stat, _ = zedf9p.RAWX_HEADER.unpack_from(m.data)
                    out.write("rawx,%d,%d,%r,%d,%d,%d\n" % (now, week, tow, leap, rec_stat, n))
                    counters[C["rawx"]] += 1
                elif m.ident == UBX_TIM_TM2 and len(m.data) >= TM2.size:
                    out.write("tm2,%d,%s\n" % (now, ",".join(map(str, TM2.unpack_from(m.data)))))
                    counters[C["tm2"]] += 1
                elif m.ident == UBX_TIM_TP and len(m.data) >= TP.size:
                    out.write("tp,%d,%s\n" % (now, ",".join(map(str, TP.unpack_from(m.data)))))
            counters[C["gnss_bad_frames"]] = rx.parser.bad_frames
            if time.monotonic() >= next_flush:
                raw.flush()
                out.flush()
                next_flush += 1
            if not data:
                time.sleep(0.01)
    finally:
        if saved:
            try:
                rx.set(saved)
            except (OSError, RuntimeError) as error:
                print("WARNING: could not restore receiver settings: %s" % error, file=sys.stderr)
        for f in (raw, out):
            if f:
                f.close()
        bus.close()


# --- main process -----------------------------------------------------------------------------

def interrupt(*_):
    raise KeyboardInterrupt


def spin():
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    while True:
        pass


class Controls:
    """PM QoS request and CPU load generators for the test phases."""

    def __init__(self):
        self.qos = None
        self.qos_error = None
        self.load = []

    def set(self, qos, load):
        if qos and not self.qos:
            try:
                self.qos = open(QOS_DEVICE, "wb", buffering=0)
                self.qos.write(struct.pack("<i", QOS_LATENCY_US))
            except OSError as error:
                self.qos = None
                if not self.qos_error:
                    self.qos_error = str(error)
                    log("WARNING: PM QoS unavailable (%s); rerun 'sudo sh setup_i2c_gpio.sh'"
                        % error)
        elif not qos and self.qos:
            self.qos.close()
            self.qos = None
        if load and not self.load:
            self.load = [mp.Process(target=spin, daemon=True) for _ in range(os.cpu_count() or 1)]
            for p in self.load:
                p.start()
        elif not load and self.load:
            for p in self.load:
                p.terminate()
                p.join()
            self.load = []

    def close(self):
        self.set(False, False)


def ntp_status():
    try:
        return subprocess.run(["timedatectl", "show", "-p", "NTP", "-p", "NTPSynchronized"],
                              capture_output=True, text=True, timeout=5).stdout.split()
    except (OSError, subprocess.SubprocessError):
        return []


def status_line(counters, elapsed, cfg, controls):
    c = {name: counters[i] for name, i in C.items()}
    return ("%5.0f/%d s | PPS %d | DRDY %d (%.2f/s) overruns %d lost %d rescued %d | RAWX %d "
            "TM2 %d%s | QoS %s, load %s" % (
                elapsed, cfg["duration"], c["pps"], c["drdy"], c["drdy"] / max(elapsed, 1e-9),
                c["overruns"], c["lost_events"], c["rescued"], c["rawx"], c["tm2"],
                " | switch %d" % c["switch"] if cfg["switch"] else "",
                "on" if controls.qos else "off", "on" if controls.load else "off"))


def record(name, cfg, out_dir):
    run_dir = os.path.join(os.path.expanduser(out_dir), time.strftime(name + "-%Y%m%d-%H%M%S"))
    os.makedirs(run_dir)
    meta = {
        "test": name, "about": cfg["about"], "config": cfg,
        "start_local": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "start_mono_ns": time.monotonic_ns(),
        "host": platform.node(), "kernel": platform.release(), "python": platform.python_version(),
        "ntp": ntp_status(), "gpio_chip": GPIO_CHIP,
        "lines": {"pps_pin7": LINE_PPS, "drdy_pin29": LINE_DRDY, "switch_pin31": LINE_SWITCH},
        "lis3dh_mg_per_digit": lis3dh.RANGES[cfg["range"]][1],
        "qos_latency_us": QOS_LATENCY_US,
        "mode": "fifo" if cfg["fifo"] else "drdy",
        "fifo_threshold": cfg["fifo_threshold"] if cfg["fifo"] else None,
    }
    meta_path = os.path.join(run_dir, "meta.json")
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=1)
    log("%s: %s -> %s" % (name, cfg["about"], run_dir))

    stop = mp.Event()
    counters = mp.Array("q", len(COUNTERS), lock=False)
    workers = [mp.Process(target=accel_worker, args=(cfg, run_dir, stop, counters), name="accel"),
               mp.Process(target=gnss_worker, args=(cfg, run_dir, stop, counters), name="gnss")]
    controls = Controls()
    markers = open(os.path.join(run_dir, "markers.csv"), "w")
    markers.write("mono_ns,elapsed_s,qos,load,label\n")
    phases = cfg["phases"]
    phase = -1
    ended = "duration"
    for w in workers:
        w.start()
    t0 = time.monotonic()
    next_status = t0 + 10
    try:
        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= cfg["duration"]:
                break
            while phase + 1 < len(phases) and elapsed >= phases[phase + 1][0]:
                phase += 1
                _, qos, load, label = phases[phase]
                controls.set(qos and not cfg["no_qos"], load)
                markers.write("%d,%.1f,%d,%d,%s\n" % (time.monotonic_ns(), elapsed,
                                                      bool(controls.qos), bool(controls.load), label))
                markers.flush()
                log("phase: " + label)
            dead = [w for w in workers if not w.is_alive()]
            if dead:
                raise RuntimeError("%s worker stopped (exit code %s)" % (dead[0].name,
                                                                        dead[0].exitcode))
            if time.monotonic() >= next_status:
                log(status_line(counters, elapsed, cfg, controls))
                next_status += 10
            time.sleep(0.2)
    except KeyboardInterrupt:
        ended = "interrupted"
    finally:
        stop.set()
        for w in workers:
            w.join(15)
            if w.is_alive():
                w.terminate()
        controls.close()
        markers.close()
        meta.update(ended=ended, duration_s=round(time.monotonic() - t0, 1),
                    counters={n: counters[i] for n, i in C.items()},
                    qos_error=controls.qos_error)
        with open(meta_path, "w") as f:
            json.dump(meta, f, indent=1)
    log("%s after %.0f s: %s" % (ended, time.monotonic() - t0,
                                 status_line(counters, time.monotonic() - t0, cfg, controls)))
    return run_dir


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="tests:\n" + "\n".join("  %-6s %5d s  ODR %3d Hz  GNSS %2d Hz  %s" % (
            k, v["duration"], v["odr"], v["gnss_rate"], v["about"]) for k, v in PRESETS.items()))
    parser.add_argument("test", choices=sorted(PRESETS), help="test preset")
    parser.add_argument("--duration", type=float, help="override the preset duration (s)")
    parser.add_argument("--odr", type=int, choices=sorted(lis3dh.RATES), help="LIS3DH rate (Hz)")
    parser.add_argument("--gnss-rate", type=float, help="GNSS measurement rate (Hz)")
    parser.add_argument("--range", type=int, choices=sorted(lis3dh.RANGES), default=8,
                        help="LIS3DH full scale in g (default 8, so taps do not clip)")
    parser.add_argument("--switch", action="store_true", help="log the tap switch on pin 31")
    parser.add_argument("--no-fifo", action="store_true",
                        help="old mode: one data-ready edge and one read per sample (loses samples "
                             "when a read is late)")
    parser.add_argument("--fifo-threshold", type=int, default=DEFAULT_FIFO_THRESHOLD,
                        help="FIFO watermark in samples, 1..%d (default %d): one time-stamped edge "
                             "per this many samples; the FIFO holds %d, so a read may be up to "
                             "(%d - threshold) / ODR late" % (FIFO_DEPTH - 1, DEFAULT_FIFO_THRESHOLD,
                                                              FIFO_DEPTH, FIFO_DEPTH))
    parser.add_argument("--no-qos", action="store_true",
                        help="never request PM QoS (measure with deep C-states)")
    parser.add_argument("--out", default="~/bench", help="parent directory (default ~/bench)")
    args = parser.parse_args(argv)

    cfg = dict(PRESETS[args.test])
    cfg.setdefault("switch", False)
    for key in ("duration", "odr", "gnss_rate"):
        if getattr(args, key) is not None:
            cfg[key] = getattr(args, key)
    cfg["switch"] = cfg["switch"] or args.switch
    cfg["range"] = args.range
    cfg["no_qos"] = args.no_qos
    cfg["fifo"] = not args.no_fifo
    if not 1 <= args.fifo_threshold < FIFO_DEPTH:
        parser.error("--fifo-threshold must be 1..%d" % (FIFO_DEPTH - 1))
    cfg["fifo_threshold"] = args.fifo_threshold
    signal.signal(signal.SIGTERM, interrupt)  # Service stop: clean up like Ctrl-C.
    try:
        run_dir = record(args.test, cfg, args.out)
    except (OSError, RuntimeError, ValueError) as error:
        sys.exit("ERROR: %s" % error)
    import bench_analyze
    if args.test == "check":
        sys.exit(0 if bench_analyze.print_check(run_dir) else 1)
    print("\nAnalyse with:  python3 bench_analyze.py %s" % run_dir)


if __name__ == "__main__":
    main()
