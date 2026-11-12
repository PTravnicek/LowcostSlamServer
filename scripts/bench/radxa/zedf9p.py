#!/usr/bin/env python3
"""Raw GNSS measurement logger for a u-blox ZED-F9P on the ROCK Pi X I2C header.

Wiring (SparkFun GPS-RTK-SMA breakout, I2C address 0x42):
  3V3 -> pin 1   GND -> pin 6   SDA -> pin 3   SCL -> pin 5
  Active multi-band antenna with open sky view on the SMA connector.

At startup the receiver's RAM configuration (never BBR or flash) is set to
output UBX-RXM-RAWX (pseudorange, carrier phase, Doppler and C/N0 of every
tracked signal) and UBX-RXM-SFRBX (broadcast navigation data) on I2C at
--rate. The receiver's own position fix (and NMEA) stays at about 1 Hz: with
four constellations, a 10 Hz fix overloads it and it drops a third of the RAWX
epochs. The previous values are restored on exit; a power cycle restores them too.

The receiver's byte stream (UBX + NMEA) is written to --output unchanged.
RTKLIB converts it to RINEX for PPK/RTK post-processing:
  convbin -r ubx -od -os -o gnss.obs -n gnss.nav gnss.ubx
--csv prints every RAWX measurement on stdout. Status lines go to stderr.

  python3 zedf9p.py --output ~/gnss/gnss-%Y%m%d-%H%M%S.ubx
  python3 zedf9p.py --rate 5 --csv > rawx.csv
Needs python3-smbus2 and group i2c (setup_i2c_gpio.sh).
"""

import argparse
import collections
import os
import signal
import struct
import sys
import time

DEFAULT_BUS = "/dev/i2c-header-3-5"
DEFAULT_ADDRESS = 0x42
REG_BYTES_AVAILABLE = 0xFD  # 0xFD/0xFE: big-endian count of bytes waiting.
REG_STREAM = 0xFF           # Output stream (reads 0xFF when empty).
READ_CHUNK = 1024
POLL_S = 0.02
MAX_PAYLOAD = 8192
MAX_NMEA = 128
PRINTABLE = bytes(range(0x20, 0x7F))

# Configuration keys: u-blox F9 HPG interface description (SparkFun u-blox_config_keys.h).
CFG_RATE_MEAS = 0x30210001             # U2: ms between measurements
CFG_RATE_NAV = 0x30210002              # U2: measurements per navigation solution
CFG_I2COUTPROT_NMEA = 0x10720002       # L: NMEA output on I2C
CFG_MSGOUT_RXM_RAWX_I2C = 0x209102A4   # U1: output RXM-RAWX every n-th epoch on I2C
CFG_MSGOUT_RXM_SFRBX_I2C = 0x20910231  # U1: output RXM-SFRBX on I2C
VALUE_SIZE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}  # Key bits 28..30 -> value bytes.
LAYER_RAM = 0x01

UBX_ACK = 0x05
UBX_ACK_NAK, UBX_ACK_ACK = 0x00, 0x01
UBX_CFG_VALSET = (0x06, 0x8A)
UBX_CFG_VALGET = (0x06, 0x8B)
UBX_MON_VER = (0x0A, 0x04)
UBX_RXM_RAWX = (0x02, 0x15)
UBX_RXM_SFRBX = (0x02, 0x13)

GNSS = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 5: "QZSS", 6: "GLO", 7: "NAVIC"}
# (gnssId, sigId) -> signal, as in the interface description and RTKLIB demo5 ubx_sig().
SIGNALS = {
    (0, 0): "L1CA", (0, 3): "L2CL", (0, 4): "L2CM", (0, 6): "L5I", (0, 7): "L5Q",
    (1, 0): "L1CA",
    (2, 0): "E1C", (2, 1): "E1B", (2, 3): "E5aI", (2, 4): "E5aQ", (2, 5): "E5bI", (2, 6): "E5bQ",
    (3, 0): "B1I_D1", (3, 1): "B1I_D2", (3, 2): "B2I_D1", (3, 3): "B2I_D2",
    (3, 5): "B1Cp", (3, 6): "B1Cd", (3, 7): "B2ap", (3, 8): "B2ad",
    (5, 0): "L1CA", (5, 1): "L1S", (5, 4): "L2CM", (5, 5): "L2CL", (5, 8): "L5I", (5, 9): "L5Q",
    (6, 0): "L1OF", (6, 2): "L2OF",
    (7, 0): "L5A",
}
FIX_QUALITY = {"0": "no fix", "1": "GNSS", "2": "DGNSS", "4": "RTK fixed", "5": "RTK float",
               "6": "dead reckoning"}

RAWX_HEADER = struct.Struct("<dHbBBB2x")        # rcvTow week leapS numMeas recStat version
RAWX_MEAS = struct.Struct("<ddfBBBBHBBBBBx")    # prMes cpMes doMes gnssId svId sigId freqId
#                                                 locktime cno prStdev cpStdev doStdev trkStat

Message = collections.namedtuple("Message", "kind ident data")
Epoch = collections.namedtuple("Epoch", "week tow leap_s measurements")
Measurement = collections.namedtuple(
    "Measurement", "gnss sv signal glo_k pr_m cp_cycles doppler_hz cno_dbhz lock_ms "
                   "pr_std_m cp_std_cycles do_std_hz half_cyc_valid")

CSV_HEADER = ("week,tow_s,gnss,sv,signal,glo_k,pr_m,cp_cycles,doppler_hz,cno_dbhz,lock_ms,"
              "pr_std_m,cp_std_cycles,do_std_hz,half_cyc_valid")


def checksum(body):
    a = b = 0
    for c in body:
        a = (a + c) & 0xFF
        b = (b + a) & 0xFF
    return bytes((a, b))


def ubx_frame(msg, payload=b""):
    body = bytes(msg) + struct.pack("<H", len(payload)) + payload
    return b"\xb5\x62" + body + checksum(body)


def encode_values(values):
    out = b""
    for key, value in values.items():
        out += struct.pack("<I", key) + int(value).to_bytes(VALUE_SIZE[key >> 28 & 7], "little")
    return out


def decode_values(data):
    values, i = {}, 0
    while i + 4 <= len(data):
        key = struct.unpack_from("<I", data, i)[0]
        size = VALUE_SIZE[key >> 28 & 7]
        values[key] = int.from_bytes(data[i + 4:i + 4 + size], "little")
        i += 4 + size
    return values


def decode_rawx(payload):
    tow, week, leap_s, count, rec_stat, _ = RAWX_HEADER.unpack_from(payload)
    if len(payload) < RAWX_HEADER.size + count * RAWX_MEAS.size:
        raise ValueError("short RXM-RAWX")
    measurements = []
    for i in range(count):
        (pr, cp, doppler, gnss, sv, sig, freq, lock, cno, pr_std, cp_std, do_std,
         trk) = RAWX_MEAS.unpack_from(payload, RAWX_HEADER.size + i * RAWX_MEAS.size)
        measurements.append(Measurement(
            GNSS.get(gnss, str(gnss)), sv, SIGNALS.get((gnss, sig), "sig%d" % sig),
            freq - 7 if gnss == 6 else None,
            pr if trk & 1 else None,
            cp if trk & 2 else None,
            doppler, cno, lock,
            0.01 * 2 ** (pr_std & 15),
            0.004 * (cp_std & 15) if cp_std & 15 != 15 else None,
            0.002 * 2 ** (do_std & 15),
            bool(trk & 4)))
    return Epoch(week, tow, leap_s if rec_stat & 1 else None, measurements)


def csv_rows(epoch):
    def f(value, spec):
        return "" if value is None else format(value, spec)
    for m in epoch.measurements:
        yield ",".join((
            str(epoch.week), "%.9f" % epoch.tow, m.gnss, str(m.sv), m.signal, f(m.glo_k, "d"),
            f(m.pr_m, ".4f"), f(m.cp_cycles, ".4f"), "%.3f" % m.doppler_hz, str(m.cno_dbhz),
            str(m.lock_ms), "%.2f" % m.pr_std_m, f(m.cp_std_cycles, ".3f"), "%.3f" % m.do_std_hz,
            str(int(m.half_cyc_valid))))


class StreamParser:
    """Splits the receiver byte stream into UBX frames and NMEA sentences."""

    def __init__(self):
        self._buf = bytearray()
        self.bad_frames = 0
        self.skipped_bytes = 0

    def feed(self, data):
        buf = self._buf
        buf += data
        out = []
        while buf:
            if buf[0] == 0xB5:
                if len(buf) >= 2 and buf[1] != 0x62:
                    self._skip(1)
                    continue
                if len(buf) < 6:
                    break
                n = buf[4] | buf[5] << 8
                if n > MAX_PAYLOAD:
                    self.bad_frames += 1
                    self._skip(1)
                    continue
                if len(buf) < n + 8:
                    break
                if checksum(buf[2:n + 6]) != buf[n + 6:n + 8]:
                    self.bad_frames += 1
                    self._skip(1)
                    continue
                out.append(Message("UBX", (buf[2], buf[3]), bytes(buf[6:n + 6])))
                del buf[:n + 8]
            elif buf[0] == 0x24:  # '$'
                end = buf.find(b"\n", 0, MAX_NMEA)
                text = bytes(buf[:end] if end >= 0 else buf[:MAX_NMEA]).rstrip(b"\r")
                if text.translate(None, PRINTABLE) or (end < 0 and len(buf) >= MAX_NMEA):
                    self._skip(1)  # Not a sentence: binary data or no end in sight.
                    continue
                if end < 0:
                    break
                line = bytes(buf[:end + 1]).rstrip(b"\r\n")
                del buf[:end + 1]
                out.append(Message("NMEA", line[3:6].decode("ascii", "replace"), line))
            else:
                starts = [i for i in (buf.find(b"\xb5", 1), buf.find(b"$", 1)) if i > 0]
                self._skip(min(starts) if starts else len(buf))
        return out

    def _skip(self, n):
        del self._buf[:n]
        self.skipped_bytes += n


class DDC:
    """u-blox I2C transport (DDC): byte count at 0xFD/0xFE, stream at 0xFF."""

    def __init__(self, bus, address=DEFAULT_ADDRESS):
        from smbus2 import i2c_msg  # Imported here so the tests run without smbus2.
        self._msg = i2c_msg
        self.bus = bus
        self.address = address

    def available(self):
        w = self._msg.write(self.address, [REG_BYTES_AVAILABLE])
        r = self._msg.read(self.address, 2)
        self.bus.i2c_rdwr(w, r)
        hi, lo = list(r)
        return hi << 8 | lo

    def read(self, n):
        out = bytearray()
        while len(out) < n:
            r = self._msg.read(self.address, min(n - len(out), READ_CHUNK))
            self.bus.i2c_rdwr(self._msg.write(self.address, [REG_STREAM]), r)
            out += bytes(r)
        return bytes(out)

    def write(self, data):
        self.bus.i2c_rdwr(self._msg.write(self.address, list(data)))


class Receiver:
    def __init__(self, ddc):
        self.ddc = ddc
        self.parser = StreamParser()

    def pump(self):
        """Read everything waiting. Returns (raw bytes, parsed messages)."""
        n = self.ddc.available()
        if not n:
            return b"", []
        data = self.ddc.read(n)
        return data, self.parser.feed(data)

    def request(self, msg, payload=b"", reply=None, timeout=3.0):
        """Send a UBX message; wait for the `reply` message, or for its ACK if reply is None."""
        self.ddc.write(ubx_frame(msg, payload))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for m in self.pump()[1]:
                if m.kind != "UBX":
                    continue
                if reply and m.ident == reply:
                    return m.data
                if m.ident[0] == UBX_ACK and m.data[:2] == bytes(msg):
                    if m.ident[1] == UBX_ACK_NAK:
                        raise RuntimeError("receiver rejected UBX %02X-%02X" % msg)
                    if not reply:
                        return b""
            time.sleep(POLL_S)
        raise TimeoutError("no reply to UBX %02X-%02X" % msg)

    def version(self):
        p = self.request(UBX_MON_VER, reply=UBX_MON_VER)
        return [p[i:i + 30].split(b"\0")[0].decode("ascii", "replace")
                for i in range(40, len(p), 30)]

    def get(self, keys):
        query = struct.pack("<BBH", 0, 0, 0) + b"".join(struct.pack("<I", k) for k in keys)
        return decode_values(self.request(UBX_CFG_VALGET, query, reply=UBX_CFG_VALGET)[4:])

    def set(self, values, layers=LAYER_RAM):
        self.request(UBX_CFG_VALSET, struct.pack("<BBH", 0, layers, 0) + encode_values(values))


class Stats:
    def __init__(self, interval_s):
        self.interval_s = interval_s
        self.epochs = self.missed = self.sfrbx = self.bytes = 0
        self.last_epoch = None
        self.last_gga = None

    def update(self, message):
        if message.kind == "NMEA" and message.ident == "GGA":
            self.last_gga = message.data.split(b",")
        elif message.kind == "UBX" and message.ident == UBX_RXM_SFRBX:
            self.sfrbx += 1

    def epoch(self, epoch):
        if self.last_epoch:
            dt = (epoch.week - self.last_epoch.week) * 604800 + epoch.tow - self.last_epoch.tow
            if dt > 1.5 * self.interval_s:
                self.missed += round(dt / self.interval_s) - 1
        self.last_epoch = epoch
        self.epochs += 1

    def line(self, seconds):
        e = self.last_epoch
        if e:
            per = collections.Counter(m.gnss for m in e.measurements)
            meas = "%d meas (%s)" % (len(e.measurements),
                                     ", ".join("%s %d" % kv for kv in sorted(per.items())))
        else:
            meas = "no RAWX yet"
        fix = ""
        g = self.last_gga
        if g and len(g) > 8:
            fix = ", fix %s, %s SV, HDOP %s" % (FIX_QUALITY.get(g[6].decode(), g[6].decode()),
                                                g[7].decode(), g[8].decode())
        return "%d epochs (%d missed), last %s, %d SFRBX%s, %.1f kB/s" % (
            self.epochs, self.missed, meas, self.sfrbx, fix, self.bytes / seconds / 1000)


def log(message):
    print(time.strftime("%H:%M:%S ") + message, file=sys.stderr, flush=True)


def record(rx, args):
    out = None
    if args.output:
        name = os.path.expanduser(time.strftime(args.output))
        if os.path.dirname(name):
            os.makedirs(os.path.dirname(name), exist_ok=True)
        out = open(name, "ab")
        log("writing " + name)
    if args.csv:
        print(CSV_HEADER, flush=True)
    stats = Stats(1 / args.rate)
    status_start = time.monotonic()
    try:
        while True:
            data, messages = rx.pump()
            if data:
                stats.bytes += len(data)
                if out:
                    out.write(data)
                    out.flush()
            for m in messages:
                stats.update(m)
                if m.kind == "UBX" and m.ident == UBX_RXM_RAWX:
                    epoch = decode_rawx(m.data)
                    stats.epoch(epoch)
                    if args.csv:
                        sys.stdout.write("".join(row + "\n" for row in csv_rows(epoch)))
                        sys.stdout.flush()
            now = time.monotonic()
            if args.status and now - status_start >= args.status:
                log(stats.line(now - status_start))
                stats.bytes = 0
                status_start = now
            if not data:
                time.sleep(POLL_S)
    finally:
        if out:
            out.close()
        if rx.parser.bad_frames:
            log("%d frames with bad checksums" % rx.parser.bad_frames)


def run(args):
    from smbus2 import SMBus
    bus = SMBus(args.bus)
    try:
        rx = Receiver(DDC(bus, args.address))
        log("receiver: " + ", ".join(rx.version()))
        settings = {CFG_RATE_MEAS: round(1000 / args.rate),
                    CFG_RATE_NAV: max(1, round(args.rate)),  # Position fix at ~1 Hz.
                    CFG_MSGOUT_RXM_RAWX_I2C: 1, CFG_MSGOUT_RXM_SFRBX_I2C: 1}
        if args.no_nmea:
            settings[CFG_I2COUTPROT_NMEA] = 0
        saved = rx.get(settings)
        rx.set(settings)
        log("RAWX + SFRBX at %g Hz, position fix at %g Hz (RAM only)"
            % (args.rate, args.rate / settings[CFG_RATE_NAV]))
        try:
            record(rx, args)
        finally:
            rx.set(saved)
            log("receiver configuration restored")
    finally:
        bus.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bus", default=DEFAULT_BUS, help="I2C device (default %(default)s)")
    parser.add_argument("--address", type=lambda s: int(s, 0), default=DEFAULT_ADDRESS,
                        help="receiver I2C address (default 0x42)")
    parser.add_argument("--rate", type=float, default=1.0,
                        help="measurement rate in Hz (default 1; up to 10 is reliable, "
                             "20 loses epochs with four constellations)")
    parser.add_argument("--output", metavar="PATH",
                        help="write the raw UBX/NMEA stream to PATH; strftime codes allowed")
    parser.add_argument("--csv", action="store_true", help="print RAWX measurements as CSV")
    parser.add_argument("--no-nmea", action="store_true",
                        help="turn NMEA off on I2C while logging (saves bandwidth at high rates)")
    parser.add_argument("--status", type=float, default=10, metavar="S",
                        help="status line on stderr every S seconds (0 = off, default 10)")
    args = parser.parse_args(argv)
    if args.rate <= 0:
        parser.error("--rate must be positive")
    # A service stop (SIGTERM) unwinds like Ctrl-C, so the configuration is restored.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        run(args)
    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
    except (OSError, RuntimeError, ValueError) as error:
        sys.exit("ERROR: %s" % error)


if __name__ == "__main__":
    main()
