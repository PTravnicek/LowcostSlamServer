#!/usr/bin/env python3
"""Analyse a bench_logger.py run: one GPS timeline for the LIS3DH and the ZED-F9P.

  python3 bench_analyze.py ~/bench/T2-20261103-153000

Sections, computed from whatever the run contains:
  1. Clock map     PPS edges -> GPS time; Radxa clock rate; interrupt timestamp jitter per phase
  2. LIS3DH clock  sample rate in GPS time from the F9P edge counter and from kernel edges,
                   drift with temperature, and how well sample times can be predicted/interpolated
  3. Timestamps    kernel edge and Python read time vs the F9P time mark (truth)
  4. Integrity     lost or missed samples, FIFO overruns, read latency, missed GNSS epochs

FIFO runs (meta.json "mode": "fifo", edges.csv present): the kernel time-stamps one FIFO watermark
edge per `fifo_threshold` samples. Each edge is the arrival of a known sample, so the sample times
are interpolated between edges; the F9P time marks and edge counter are compared with those edges.
  5. Taps          switch closure vs acceleration onset (T5)

Writes report.txt and CSV tables into the run directory. Standard library only, Python 3.10+.
GPS times are kept as nanoseconds since the first PPS second of the run (S0).
"""

import argparse
import array
import bisect
import collections
import json
import math
import os
import statistics

NS = 1_000_000_000
FIFO_DEPTH = 32             # LIS3DH FIFO size
WEEK_S = 604800
PPS_P99_LIMIT_US = 100      # T1: interval error, phases with PM QoS
PATH_P99_LIMIT_US = 1000    # T3: kernel timestamp error vs F9P time mark
PREDICT_MAX_LIMIT_US = 1000  # T2: worst prediction error within 1 s of a PPS anchor
PPS_GATE_NS = 20_000_000    # PPS edges must lie this close to a whole second after the last good one
TAP_QUIET_NS = 200_000_000   # Switch bounce: a closure needs 200 ms of quiet before it.

# gnss.csv "tm2" rows: arrival time, then UBX-TIM-TM2 in u-blox field order (wnF before towMsR).
TM2Row = collections.namedtuple("TM2Row", "arrival ch flags count wn_r wn_f ms_r sub_r ms_f sub_f acc")


def pct(values, q):
    """q-th percentile (nearest rank)."""
    if not values:
        return float("nan")
    s = sorted(values)
    return s[min(len(s) - 1, max(0, math.ceil(q / 100 * len(s)) - 1))]


def spread(values_ns):
    """(median, p99 |x|, max |x|) in microseconds."""
    a = [abs(v) for v in values_ns]
    return (statistics.median(values_ns) / 1e3, pct(a, 99) / 1e3, max(a) / 1e3)


def unwrap16(counts):
    out, total, last = [], 0, None
    for c in counts:
        if last is not None:
            total += (c - last) % 65536
        out.append(total)
        last = c
    return out


def crossing_level(edge_ns, edge_fss, edge_read, odr, threshold):
    """FIFO fill at which the watermark edge rises, measured.

    The edge rises with the arrival of a sample, so when the FIFO is read `lat` later it holds
    level + floor(lat / period) samples. The LIS3DH raises it at threshold + 1 (measured on the
    ROCK Pi X: 12954 of 12956 edges; the datasheet says "equal or higher"), so the level is taken
    from the data and only falls back to that when there are no edges.
    """
    if not len(edge_ns):
        return threshold + 1
    period = NS / odr
    levels = collections.Counter(f - int((r - e) // period)
                                 for e, f, r in zip(edge_ns, edge_fss, edge_read))
    return levels.most_common(1)[0][0]


def anchor_numbers(edge_ns, popped_before, odr, level):
    """1-based number of the sample whose arrival raised each watermark edge.

    The first edge follows a FIFO that was read empty, so it is sample popped + level. The next
    ones add the time since the previous edge in sample periods: the sensor clock is within ~1 %
    of nominal, so even edges 50 samples apart are unambiguous (the edges lie on the sample grid
    to 0.003 samples). A missed edge (INT1 stayed high because the FIFO was not read empty in
    time) just makes the step longer.
    """
    if not len(edge_ns):
        return []
    nums = [popped_before[0] + level]
    period = NS / odr
    for i in range(1, len(edge_ns)):
        step = max(1, round((edge_ns[i] - edge_ns[i - 1]) / period))
        nums.append(nums[-1] + step)
        if nums[-1] - nums[0] >= 200:
            period = (edge_ns[i] - edge_ns[0]) / (nums[-1] - nums[0])
    return nums


def interpolate_sample_times(count, index, ns):
    """Time of samples 0..count-1 by linear interpolation between anchors (index[k] -> ns[k]);
    outside the anchors the nearest segment is extended."""
    out = array.array("q")
    if len(index) < 2:
        out.extend([0] * count)
        return out
    k, last = 0, len(index) - 2
    for i in range(count):
        while k < last and index[k + 1] <= i:
            k += 1
        i0, i1, t0, t1 = index[k], index[k + 1], ns[k], ns[k + 1]
        out.append(round(t0 + (i - i0) * (t1 - t0) / (i1 - i0)))
    return out


class Run:
    def __init__(self, path):
        self.dir = path
        with open(self.file("meta.json")) as f:
            self.meta = json.load(f)
        cfg = self.meta["config"]
        self.odr, self.gnss_rate = cfg["odr"], cfg["gnss_rate"]
        self.mg_per_digit = self.meta["lis3dh_mg_per_digit"]
        self.pps_ns, _ = self.columns("pps.csv", "qq")
        cols = self.columns("accel.csv", "qqqBhhh")
        timed = [i for i, t in enumerate(cols[1]) if t >= 0]  # drdy_ns -1: rescue read
        self.rescued = len(cols[1]) - len(timed)
        if self.rescued:
            cols = [array.array(c.typecode, (c[i] for i in timed)) for c in cols]
        self.seq, self.drdy_ns, self.read_ns, self.status, self.x, self.y, self.z = cols
        self.fifo = self.meta.get("mode") == "fifo"
        self.threshold = self.meta.get("fifo_threshold") or 1
        self.edge_ns = self.edge_seq = self.edge_popped = self.edge_fss = self.edge_read = ()
        self.anchor_num = self.anchor_idx = []
        self.level = self.threshold + 1
        if self.fifo:
            self.rescued = self.meta.get("counters", {}).get("rescued", 0)  # edges that never came
            (self.edge_ns, self.edge_seq, self.edge_popped, self.edge_fss,
             self.edge_read) = self.columns("edges.csv", "qqqqq")
            self.level = crossing_level(self.edge_ns, self.edge_fss, self.edge_read, self.odr,
                                        self.threshold)
            self.anchor_num = anchor_numbers(self.edge_ns, self.edge_popped, self.odr, self.level)
            self.anchor_idx = [n - 1 for n in self.anchor_num]  # 0-based index of that sample
            self.drdy_ns = interpolate_sample_times(len(self.seq), self.anchor_idx, self.edge_ns)
        self.sw_ns, _, self.sw_rising = self.columns("switch.csv", "qqb")
        self.temp_ns, self.cpu_mc, self.lis_temp = self.columns("temp.csv", "qqh")
        self.markers = []
        for line in self.lines("markers.csv"):
            mono, elapsed, qos, load, label = line.split(",", 4)
            self.markers.append((int(mono), float(elapsed), qos == "1", load == "1", label))
        self.rawx, self.tm2, self.gga = [], [], []
        for line in self.lines("gnss.csv", header=False):
            p = line.split(",")
            if p[0] == "rawx":
                self.rawx.append((int(p[1]), int(p[2]), float(p[3]), int(p[4]), int(p[5])))
            elif p[0] == "tm2":
                self.tm2.append(TM2Row(*(int(v) for v in p[1:12])))
            elif p[0] == "gga":
                self.gga.append((int(p[1]), p[2], p[3], p[4]))

    def file(self, name):
        return os.path.join(self.dir, name)

    def lines(self, name, header=True):
        """Complete lines only (a run killed mid-write may end with a partial one)."""
        if not os.path.exists(self.file(name)):
            return
        with open(self.file(name)) as f:
            if header:
                next(f, None)
            for line in f:
                if line.endswith("\n") and not line.startswith("#"):
                    yield line.rstrip("\n")

    def columns(self, name, types):
        cols = [array.array(t) for t in types]
        for line in self.lines(name):
            try:
                values = [int(v) for v in line.split(",")]
            except ValueError:
                continue
            if len(values) == len(types):
                for c, v in zip(cols, values):
                    c.append(v)
        return cols

    def phase_at(self, mono_ns):
        """Index of the phase active at mono_ns (0 if there are no markers)."""
        if not self.markers:
            return 0
        return max(0, bisect.bisect_right([m[0] for m in self.markers], mono_ns) - 1)

    def phase_label(self, i):
        if not self.markers:
            return "all"
        m = self.markers[i]
        return "%s [QoS %s]" % (m[4], "on" if m[2] else "off")

    def leap_seconds(self):
        valid = [r[3] for r in self.rawx if r[4] & 1]
        return collections.Counter(valid).most_common(1)[0][0] if valid else 18


class ClockMap:
    """CLOCK_MONOTONIC ns -> GPS ns since S0, interpolated between consecutive PPS edges."""

    def __init__(self, pps_ns, rawx):
        t = []
        pps_ns = list(pps_ns)
        # Start at the first edge that is followed by one a second later (a leading glitch is not).
        start = next((i for i in range(len(pps_ns) - 1)
                      if abs(pps_ns[i + 1] - pps_ns[i] - NS) < PPS_GATE_NS), 0)
        for x in pps_ns[start:]:
            # Keep an edge only if it lies within 20 ms of a whole number of seconds after the last
            # good one; a spurious edge (T4 had one 89 ms before a real PPS) is dropped, not the
            # real edge after it.
            if not t:
                t.append(x)
                continue
            k = round((x - t[-1]) / NS)
            if k >= 1 and abs(x - t[-1] - k * NS) < PPS_GATE_NS:
                t.append(x)
        if len(t) < 3:
            raise ValueError("only %d PPS edges" % len(t))
        n = [0]
        for a, b in zip(t, t[1:]):
            n.append(n[-1] + round((b - a) / NS))
        votes = collections.Counter()
        for arrival, week, tow, _, _ in rawx:
            s = round(tow)
            if abs(tow - s) > 0.05:
                continue
            k = bisect.bisect_right(t, arrival) - 1
            if k >= 0 and arrival - t[k] < 0.95 * NS:
                votes[week * WEEK_S + s - n[k]] += 1
        if not votes:
            raise ValueError("no RAWX epoch could be matched to a PPS edge")
        offset, count = votes.most_common(1)[0]
        self.agreement = count / sum(votes.values())
        self.t = t
        self.S = [k + offset for k in n]
        self.S0 = self.S[0]
        fit = statistics.linear_regression([s - self.S0 for s in self.S], [x - t[0] for x in t])
        self.slope, self.intercept = fit.slope, fit.intercept  # mono ns per GPS second

    def gps_ns(self, mono):
        t, S = self.t, self.S
        k = bisect.bisect_right(t, mono) - 1
        if 0 <= k < len(t) - 1 and 1 <= S[k + 1] - S[k] <= 5:
            return (S[k] - self.S0) * NS + (mono - t[k]) * (S[k + 1] - S[k]) * NS / (t[k + 1] - t[k])
        k = min(max(k, 0), len(t) - 1)
        return (S[k] - self.S0) * NS + (mono - t[k]) * NS / self.slope

    def gps_sorted(self, monos):
        """gps_ns() for an ascending sequence, walking the PPS edges once."""
        t, S, S0 = self.t, self.S, self.S0
        out = array.array("d")
        k, last = 0, len(t) - 1
        for m in monos:
            while k < last and t[k + 1] <= m:
                k += 1
            if t[k] <= m and k < last and 1 <= S[k + 1] - S[k] <= 5:
                out.append((S[k] - S0) * NS + (m - t[k]) * (S[k + 1] - S[k]) * NS / (t[k + 1] - t[k]))
            else:
                out.append(self.gps_ns(m))
        return out


def tm2_edges(run, S0):
    """F9P time marks of LIS3DH rising edges: [(gps ns since S0, edge count, accEst ns)]."""
    leap = run.leap_seconds()
    out = []
    for r in run.tm2:
        base = r.flags >> 3 & 3
        if not (r.flags & 0x80 and r.flags & 0x40) or base not in (1, 2):
            continue  # Need a new rising edge, valid time, GNSS or UTC time base.
        g = (r.wn_r * WEEK_S - S0) * NS + r.ms_r * 1_000_000 + r.sub_r
        if base == 2:
            g += leap * NS  # UTC -> GPS
        out.append((g, r.count, r.acc))
    return out


def fit_rate(times_s, counts, max_points=20000):
    step = max(1, len(times_s) // max_points)
    return statistics.linear_regression(times_s[::step], counts[::step]).slope


class Report:
    def __init__(self):
        self.lines = []
        self.verdicts = []

    def __call__(self, text=""):
        self.lines.append(text)

    def verdict(self, ok, text):
        self.verdicts.append((ok, text))
        self("  -> %s: %s" % ("PASS" if ok else "FAIL", text))


def write_csv(run, name, header, rows):
    with open(run.file(name), "w") as f:
        f.write(header + "\n")
        for r in rows:
            f.write(",".join(r) + "\n")


# --- sections ------------------------------------------------------------------------------------

def section_clock(run, cmap, out):
    out("1. Clock map: PPS -> GPS time")
    t, S = cmap.t, cmap.S
    missing = sum(b - a - 1 for a, b in zip(S, S[1:]) if b - a > 1)
    out("  PPS edges %d, GPS seconds %d..%d, missing %d, RAWX labels agree %.0f%%"
        % (len(t), S[0], S[-1], missing, 100 * cmap.agreement))
    ppm = (cmap.slope / NS - 1) * 1e6
    resid = [(x - t[0]) - (cmap.slope * (s - cmap.S0) + cmap.intercept) for x, s in zip(t, S)]
    out("  Radxa CLOCK_MONOTONIC vs GPS: %+.2f ppm, wander around that rate %.1f us peak-peak%s"
        % (ppm, (max(resid) - min(resid)) / 1e3,
           "  (NTP %s)" % " ".join(run.meta.get("ntp", [])) if run.meta.get("ntp") else ""))
    per_phase = collections.defaultdict(list)
    rows = []
    for k in range(len(t) - 1):
        if S[k + 1] - S[k] == 1:
            e = (t[k + 1] - t[k]) - cmap.slope
            ph = run.phase_at(t[k + 1])
            per_phase[ph].append(e)
            rows.append(("%d" % (S[k + 1] - cmap.S0), "%.3f" % (e / 1e3), str(ph)))
    write_csv(run, "pps_intervals.csv", "gps_s,interval_error_us,phase", rows)
    out("  PPS interval error (timestamp jitter of two edges), per phase:")
    qos_p99 = []
    for ph in sorted(per_phase):
        e = per_phase[ph]
        _, p99, mx = spread(e)
        out("    %-52s n %5d  std %7.1f us  p99 %7.1f us  max %7.1f us"
            % (run.phase_label(ph), len(e), statistics.pstdev(e) / 1e3, p99, mx))
        if not run.markers or run.markers[ph][2]:
            qos_p99.append(p99)
    if qos_p99:
        out.verdict(max(qos_p99) < PPS_P99_LIMIT_US,
                    "PPS timestamp p99 %.1f us with PM QoS (limit %d us)"
                    % (max(qos_p99), PPS_P99_LIMIT_US))
    else:
        out("  (no phase with PM QoS: run setup_i2c_gpio.sh, see meta.json qos_error)")
    out()


def missed_samples(run, gt):
    """Samples with no edge at all: gaps longer than 1.5 nominal-ish periods between edges."""
    if len(gt) < 3:
        return 0
    period = statistics.median(b - a for a, b in zip(gt[:2000], gt[1:2001]))
    return sum(round((gt[i + 1] - gt[i]) / period) - 1 for i in range(len(gt) - 1)
               if run.seq[i + 1] - run.seq[i] == 1 and gt[i + 1] - gt[i] > 1.5 * period)


def interpolation_check(run, cmap, out):
    """Leave-one-out: predict each interior watermark edge from its two neighbours by linear
    interpolation over the sample index, as sample times between edges are. The edge times are
    kernel time stamps (13 us against the F9P with PM QoS), so the difference is the interpolation
    error. Judged only in phases with PM QoS, like the other timestamp checks: without it the
    kernel time stamps themselves jitter by up to ~1 ms (T1 phase 1)."""
    idx, g = run.anchor_idx, cmap.gps_sorted(run.edge_ns)
    per_phase = collections.defaultdict(list)
    for k in range(1, len(idx) - 1):
        a, b, c = idx[k - 1], idx[k], idx[k + 1]
        if c == a:
            continue
        pred = g[k - 1] + (b - a) * (g[k + 1] - g[k - 1]) / (c - a)
        per_phase[run.phase_at(run.edge_ns[k])].append(pred - g[k])
    if not per_phase:
        return
    steps = [b - a for a, b in zip(idx, idx[1:])]
    out("  sample times are interpolated between %d watermark edges (one per %d..%d samples)"
        % (len(idx), min(steps), max(steps)))
    out("  interpolation error (each edge predicted from its two neighbours), per phase:")
    qos_max = []
    for ph in sorted(per_phase):
        med, p99, mx = spread(per_phase[ph])
        out("    %-52s n %5d  median %+7.1f us  p99 %7.1f us  max %7.1f us"
            % (run.phase_label(ph), len(per_phase[ph]), med, p99, mx))
        if not run.markers or run.markers[ph][2]:
            qos_max.append(mx)
    if qos_max:
        out.verdict(max(qos_max) < PREDICT_MAX_LIMIT_US,
                    "sample times interpolated within %.0f us with PM QoS (limit %d us)"
                    % (max(qos_max), PREDICT_MAX_LIMIT_US))


def mark_samples(run, cmap, edges):
    """[(GPS ns, cumulative samples)] at the F9P time marks. Non-FIFO: the F9P counts every
    data-ready edge, one per sample. FIFO: it counts watermark edges, which are not a fixed number
    of samples apart, so each mark takes the sample number of the kernel edge it matches."""
    if not run.fifo:
        return [(g, c) for (g, _, _), c in zip(edges, unwrap16([c for _, c, _ in edges]))]
    ge = cmap.gps_sorted(run.edge_ns)
    out = []
    for g, _, _ in edges:
        j = bisect.bisect_left(ge, g)
        j = min((i for i in (j - 1, j) if 0 <= i < len(ge)), key=lambda i: abs(ge[i] - g), default=None)
        if j is not None and abs(ge[j] - g) < 0.5 * NS * run.level / run.odr:
            out.append((g, run.anchor_num[j]))
    return out


def section_lis_clock(run, cmap, gt, edges, out):
    out("2. LIS3DH sample clock (nominal %d Hz)" % run.odr)
    lossy = 0 if run.fifo else missed_samples(run, gt) + run.rescued
    if lossy:
        out("  WARNING: %d samples had no edge (section 4), so edge counts undercount samples and the"
            % lossy)
        out("  rates below are too low; measure the clock in a loss-free run (e.g. T2 at 100 Hz).")
    odr_k = fit_rate([g / NS for g in gt], list(run.seq)) if len(gt) > 10 else float("nan")
    odr_t = float("nan")
    marks = mark_samples(run, cmap, edges)   # [(gps ns, cumulative samples)]
    if len(marks) > 2:
        odr_t = fit_rate([g / NS for g, _ in marks], [c for _, c in marks])
        out("  from F9P time marks (hardware):  %.5f Hz  (%+.0f ppm vs nominal)"
            % (odr_t, (odr_t / run.odr - 1) * 1e6))
    out("  from kernel edges via PPS map:   %.5f Hz  (%+.0f ppm vs nominal)%s"
        % (odr_k, (odr_k / run.odr - 1) * 1e6,
           "; agree within %.1f ppm" % abs((odr_k / odr_t - 1) * 1e6) if len(marks) > 2 else ""))

    # Per minute: rate (time marks, else kernel) and temperatures.
    temp_g = cmap.gps_sorted(run.temp_ns) if len(run.temp_ns) else []
    minutes = collections.defaultdict(dict)
    for g, s in ((gt[i], run.seq[i]) for i in range(len(gt))):
        m = minutes[int(g // (60 * NS))]
        m.setdefault("k0", (g, s))
        m["k1"] = (g, s)
    if len(marks) > 2:
        for g, c in marks:
            m = minutes[int(g // (60 * NS))]
            m.setdefault("t0", (g, c))
            m["t1"] = (g, c)
    for g, cpu, lis in zip(temp_g, run.cpu_mc, run.lis_temp):
        m = minutes[int(g // (60 * NS))]
        m.setdefault("lis", []).append(lis / 256)
        m.setdefault("cpu", []).append(cpu / 1000)
    rows, series = [], []
    for minute in sorted(minutes):
        m = minutes[minute]
        rate = {}
        for key in ("k", "t"):
            if key + "0" in m and m[key + "1"][0] - m[key + "0"][0] > 30 * NS:
                (g0, c0), (g1, c1) = m[key + "0"], m[key + "1"]
                rate[key] = (c1 - c0) / ((g1 - g0) / NS)
        lis = statistics.mean(m["lis"]) if m.get("lis") else float("nan")
        cpu = statistics.mean(m["cpu"]) if m.get("cpu") else float("nan")
        best = rate.get("t", rate.get("k"))
        if best:
            series.append(((best / run.odr - 1) * 1e6, lis, cpu))
        rows.append((str(minute), "%.5f" % rate.get("t", math.nan), "%.5f" % rate.get("k", math.nan),
                     "%.1f" % (((best or math.nan) / run.odr - 1) * 1e6), "%.2f" % lis, "%.1f" % cpu))
    write_csv(run, "odr_minutes.csv", "minute,odr_tm2_hz,odr_kernel_hz,ppm,lis3dh_temp_rel_c,cpu_temp_c",
              rows)
    if len(series) >= 2:
        ppm = [p for p, _, _ in series]
        lis = [x for _, x, _ in series if not math.isnan(x)]
        out("  per-minute rate: %+.0f .. %+.0f ppm (range %.0f ppm) over %d min"
            % (min(ppm), max(ppm), max(ppm) - min(ppm), len(series)))
        if len(lis) == len(series) and max(lis) - min(lis) >= 2:
            slope = statistics.linear_regression(lis, ppm).slope
            out("  LIS3DH temperature changed %.1f C (relative sensor): %+.0f ppm/C"
                % (max(lis) - min(lis), slope))

    span_s = (gt[-1] - gt[0]) / NS if len(gt) > 1 else 0
    if run.fifo:
        interpolation_check(run, cmap, out)
    # Re-anchor at a PPS second, predict the following samples from the trailing 60 s rate.
    for horizon in (1, 10) if not run.fifo else ():
        worst = []
        first = math.ceil(gt[0] / NS) + 5 if len(gt) else 0
        for s in range(first, int(gt[-1] // NS) - horizon if len(gt) else 0, horizon):
            a = bisect.bisect_left(gt, s * NS)
            b = bisect.bisect_left(gt, max(gt[0], (s - 60) * NS))
            end = bisect.bisect_left(gt, (s + horizon) * NS)
            if a - b < 50 or end <= a:
                continue
            rate = (run.seq[a] - run.seq[b]) / ((gt[a] - gt[b]) / NS)
            worst.append(max(abs(gt[n] - gt[a] - (run.seq[n] - run.seq[a]) * NS / rate)
                             for n in range(a, end)))
        if worst:
            out("  predict samples %2d s after a PPS anchor (rate from last 60 s): worst per window "
                "median %.1f us, p99 %.1f us, max %.1f us" % (horizon, statistics.median(worst) / 1e3,
                                                             pct(worst, 99) / 1e3, max(worst) / 1e3))
            if horizon == 1:
                out.verdict(max(worst) < PREDICT_MAX_LIMIT_US * 1e3,
                            "sample times predicted within %.0f us for 1 s after each PPS (limit %d us)"
                            % (max(worst) / 1e3, PREDICT_MAX_LIMIT_US))
    if span_s:
        out("  counting samples at the nominal %d Hz would be off by %.1f ms after 1 s, %.0f ms after 10 s"
            % (run.odr, abs(odr_k - run.odr) / run.odr * 1e3, abs(odr_k - run.odr) / run.odr * 1e4))
    out()
    return odr_k


def section_paths(run, cmap, gt, edges, odr, out):
    out("3. Timestamp paths vs F9P time marks")
    if not edges or not len(gt):
        out("  no F9P time marks or no samples")
        out()
        return
    if run.fifo:   # The edges are the watermark edges: compare those.
        gt, seqs, reads, monos = cmap.gps_sorted(run.edge_ns), run.edge_seq, run.edge_read, run.edge_ns
        half = 0.5 * NS * run.level / odr
    else:
        seqs, reads, monos = run.seq, run.read_ns, run.drdy_ns
        half = 0.5 * NS / odr
    per_phase = collections.defaultdict(lambda: ([], []))
    offsets = collections.Counter()
    rows, unmatched = [], 0
    for g, count, acc in edges:
        j = bisect.bisect_left(gt, g)
        j = min((i for i in (j - 1, j) if 0 <= i < len(gt)), key=lambda i: abs(gt[i] - g))
        if abs(gt[j] - g) > half:
            unmatched += 1
            continue
        ek = gt[j] - g
        er = cmap.gps_ns(reads[j]) - g
        offsets[(count - seqs[j]) % 65536] += 1
        ph = run.phase_at(monos[j])
        per_phase[ph][0].append(ek)
        per_phase[ph][1].append(er)
        rows.append(("%.6f" % (g / NS), "%.2f" % (ek / 1e3), "%.2f" % (er / 1e3), str(ph)))
    write_csv(run, "path_errors.csv", "gps_s,kernel_err_us,read_err_us,phase", rows)
    out("  %d time marks matched to a kernel edge, %d unmatched, F9P accEst median %d ns"
        % (len(rows), unmatched, statistics.median([a for _, _, a in edges])))
    qos_p99 = []
    for ph in sorted(per_phase):
        ek, er = per_phase[ph]
        k, r = spread(ek), spread(er)
        out("    %s" % run.phase_label(ph))
        out("      kernel edge + PPS map: median %+8.1f us  p99 %8.1f us  max %8.1f us  (n %d)"
            % (k + (len(ek),)))
        out("      Python read time:      median %+8.1f us  p99 %8.1f us  max %8.1f us"
            % r)
        if not run.markers or run.markers[ph][2]:
            qos_p99.append(k[1])
    consistent = len(offsets) == 1
    out("  edge counters (F9P count - kernel seqno) %s"
        % ("constant: no edge seen by one and missed by the other" if consistent
           else "changed %d times: %s" % (len(offsets) - 1, dict(offsets.most_common(4)))))
    if qos_p99:
        out.verdict(max(qos_p99) < PATH_P99_LIMIT_US and consistent,
                    "kernel timestamps p99 %.1f us vs GNSS truth (limit %d us)%s"
                    % (max(qos_p99), PATH_P99_LIMIT_US, "" if consistent else ", counters differ"))
    out()


def fifo_lost_samples(run):
    """Samples the FIFO dropped, from the numbering. The anchor number counts every sample the
    sensor made; `popped` counts those that were read. For an edge raised right after the FIFO was
    read empty, number - level == popped, so a positive difference is lost samples. (An edge
    raised while a read was running gives a negative one: pops had not caught up yet.)"""
    diffs = [n - run.level - p for n, p in zip(run.anchor_num, run.edge_popped)]
    return max([0] + diffs), sum(1 for d in diffs if d > 0)


def section_integrity(run, gt, odr, out):
    out("4. Integrity")
    seq = run.seq
    period = NS / odr
    counters = run.meta.get("counters", {})
    if run.fifo:
        lost_edges = sum(b - a - 1 for a, b in zip(run.edge_seq, run.edge_seq[1:]) if b - a > 1)
        gaps = sum(1 for a, b in zip(run.anchor_num, run.anchor_num[1:]) if b - a > 2 * run.level)
        lost_n, lost_edges_n = fifo_lost_samples(run)
        full = counters.get("overruns", 0)
        out("  LIS3DH samples %d read in batches (FIFO watermark %d, edge rises at %d of %d samples): "
            "lost samples %d (edges where sample number > read count: %d), FIFO-full events %d" % (
                len(seq), run.threshold, run.level, FIFO_DEPTH, lost_n, lost_edges_n, full))
        out("  watermark edges %d: kernel lost %d, gaps in the edge series %d, reads without an edge %d"
            % (len(run.edge_ns), lost_edges, gaps, run.rescued))
        lat = [r - e for r, e in zip(run.edge_read, run.edge_ns)]
        if lat:
            fill = list(run.edge_fss)
            out("  watermark edge -> first FIFO read: median %.2f ms, p99 %.2f ms, max %.2f ms "
                "(FIFO full after %.1f ms)" % (statistics.median(lat) / 1e6, pct(lat, 99) / 1e6,
                                                max(lat) / 1e6, (FIFO_DEPTH - run.level) * period / 1e6))
            out("  FIFO fill at that read: median %d, p99 %d, max %d of %d samples (edge level %d)" % (
                statistics.median(fill), pct(fill, 99), max(fill), FIFO_DEPTH, run.level))
        missed, overruns, lost, rescued = 0, full, lost_n, 0
    else:
        lost = sum(b - a - 1 for a, b in zip(seq, seq[1:]) if b - a > 1)
        missed = missed_samples(run, gt)
        overruns = sum(1 for s in run.status if s & 0x80)
        rescued = run.rescued
        stale = sum(1 for s in run.status if not s & 0x08)
        lat = [r - d for r, d in zip(run.read_ns, run.drdy_ns)]
        out("  LIS3DH samples %d: kernel lost events %d, missed samples (no edge) %d, overrun flags %d, "
            "stale reads %d, rescue reads (INT1 stuck high) %d"
            % (len(seq), lost, missed, overruns, stale, run.rescued))
        if lat:
            out("  read latency after data-ready: median %.2f ms, p99 %.2f ms, max %.2f ms "
                "(sample period %.2f ms)" % (statistics.median(lat) / 1e6, pct(lat, 99) / 1e6,
                                             max(lat) / 1e6, period / 1e6))
    gnss_missed = 0
    start = run.rawx[0][0] + 3 * NS if run.rawx else 0
    for (a0, w0, t0, _, _), (a1, w1, t1, _, _) in zip(run.rawx, run.rawx[1:]):
        dt = (w1 - w0) * WEEK_S + t1 - t0
        if a1 > start and dt > 1.5 / run.gnss_rate:
            gnss_missed += round(dt * run.gnss_rate) - 1
    fixes = [q for _, q, _, _ in run.gga]
    out("  GNSS: RAWX epochs %d (%g Hz), missed %d (after the first 3 s), TIM-TM2 %d, "
        "bad frames %d, fix in %d of %d GGA" % (
            len(run.rawx), run.gnss_rate, gnss_missed, len(run.tm2),
            counters.get("gnss_bad_frames", 0), sum(q not in ("", "0") for q in fixes), len(fixes)))
    clean = lost == missed == overruns == rescued == gnss_missed == 0
    if run.fifo:
        clean = lost == overruns == gnss_missed == 0   # A late edge costs no samples in FIFO mode.
    out.verdict(clean, "no samples or epochs lost" if clean else
                "lost %d / missed %d / FIFO-full or overrun %d / rescued %d samples, %d GNSS epochs"
                % (lost, missed, overruns, rescued, gnss_missed))
    out()


def section_taps(run, cmap, gt, out):
    if not len(run.sw_ns):
        return
    out("5. Tap test: switch closure vs acceleration onset")
    scale = run.mg_per_digit / 1000 / 16
    taps, last = [], None
    for t, rising in zip(run.sw_ns, run.sw_rising):
        if rising and (last is None or t - last > TAP_QUIET_NS):
            taps.append(cmap.gps_ns(t))
        last = t

    def acc(i):
        return (run.x[i] * scale, run.y[i] * scale, run.z[i] * scale)

    rows, offsets = [], []
    for g in taps:
        i0, i1 = bisect.bisect_left(gt, g - 500e6), bisect.bisect_left(gt, g - 50e6)
        if i1 - i0 < 10:
            continue
        pre = [acc(i) for i in range(i0, i1)]
        base = [statistics.mean(c) for c in zip(*pre)]

        def dev(i):
            return math.dist(acc(i), base)
        noise = statistics.pstdev([math.dist(a, base) for a in pre])
        thr = max(0.05, 8 * noise)
        j0, j1 = bisect.bisect_left(gt, g - 20e6), bisect.bisect_left(gt, g + 200e6)
        hit = next((j for j in range(max(j0, 1), j1) if dev(j) > thr), None)
        if hit is None:
            continue
        d0, d1 = dev(hit - 1), dev(hit)
        onset = gt[hit - 1] + (thr - d0) / (d1 - d0) * (gt[hit] - gt[hit - 1]) if d1 > d0 else gt[hit]
        peak = max(dev(j) for j in range(hit, j1))
        offsets.append(onset - g)
        rows.append(("%.6f" % (g / NS), "%.3f" % ((onset - g) / 1e6), "%.3f" % peak))
    write_csv(run, "taps.csv", "gps_s,offset_ms,peak_g", rows)
    out("  switch closures %d, taps with a detected onset %d" % (len(taps), len(offsets)))
    if len(offsets) >= 2:
        o = [x / 1e6 for x in offsets]
        period_ms = 1e3 / run.odr
        std = statistics.stdev(o)
        out("  onset - closure: mean %+.3f ms, std %.3f ms, min %+.3f, max %+.3f ms "
            "(sample period %.2f ms)" % (statistics.mean(o), std, min(o), max(o), period_ms))
        out.verdict(std < period_ms, "tap offset std %.3f ms (limit one sample period %.2f ms); use "
                    "the mean %+.3f ms as the accelerometer delay" % (std, period_ms, statistics.mean(o)))
    out()


# --- entry points --------------------------------------------------------------------------------

def analyze(path):
    run = Run(path)
    out = Report()
    cfg = run.meta["config"]
    out("Run %s: %s" % (os.path.basename(os.path.normpath(path)), run.meta.get("about", "")))
    out("  %s, %.0f s (%s), LIS3DH %d Hz +-%d g, GNSS %g Hz, %s"
        % (run.meta.get("start_local", "?"), run.meta.get("duration_s", cfg["duration"]),
           run.meta.get("ended", "?"), run.odr, cfg["range"], run.gnss_rate, run.meta.get("kernel", "")))
    for m in run.markers:
        out("  phase at %6.0f s: %s" % (m[1], run.phase_label(run.markers.index(m))))
    out()
    try:
        cmap = ClockMap(run.pps_ns, run.rawx)
    except ValueError as error:
        out("No GPS clock map (%s): check the PPS wire to pin 7 and the GNSS fix." % error)
        return run, out
    gt = cmap.gps_sorted(run.drdy_ns)
    edges = tm2_edges(run, cmap.S0)
    section_clock(run, cmap, out)
    odr = section_lis_clock(run, cmap, gt, edges, out) if len(gt) > 10 else run.odr
    section_paths(run, cmap, gt, edges, odr, out)
    section_integrity(run, gt, odr, out)
    section_taps(run, cmap, gt, out)
    failed = [text for ok, text in out.verdicts if not ok]
    out("Summary: %d PASS, %d FAIL%s" % (len(out.verdicts) - len(failed), len(failed),
                                        "".join("\n  FAIL " + f for f in failed)))
    return run, out


def check(path):
    """Wiring/timing checks for a short run: [(level, text)]."""
    run = Run(path)
    items = []
    duration = run.meta.get("duration_s", run.meta["config"]["duration"])
    n_pps = len(run.pps_ns)
    gaps = [(b - a) / NS for a, b in zip(run.pps_ns, run.pps_ns[1:])]
    if n_pps >= duration - 3 and gaps and all(0.99 < g < 1.01 for g in gaps):
        items.append(("OK", "PPS on pin 7: %d edges, 1 Hz" % n_pps))
    else:
        items.append(("FAIL", "PPS on pin 7: %d edges in %.0f s; check the wire from the breakout PPS "
                              "pin and that the PPS LED blinks (needs a GNSS fix)" % (n_pps, duration)))
    per_edge = run.level if run.fifo else 1   # Samples per kernel edge (about; the check allows 5 %).
    edge_ns = run.edge_ns if run.fifo else run.drdy_ns
    span = (edge_ns[-1] - edge_ns[0]) / NS if len(edge_ns) > 1 else 0
    rate = (len(edge_ns) - 1) / span if span else 0
    expected = run.odr / per_edge
    ok = abs(rate / expected - 1) < 0.15 if run.fifo else abs(rate / expected - 1) < 0.05
    items.append(("OK" if ok else "FAIL", "LIS3DH %s on pin 29: %.1f edges/s (expected %.1f)%s"
                  % ("FIFO watermark" if run.fifo else "data-ready", rate, expected,
                     "" if ok else "; check LIS3DH INT -> pin 29")))
    marks = [r for r in run.tm2 if r.flags & 0x80]
    if len(marks) >= 3:
        cum = unwrap16([r.count for r in marks])
        f9p_rate = cum[-1] / ((marks[-1].arrival - marks[0].arrival) / NS)
        ok = abs(f9p_rate / expected - 1) < (0.15 if run.fifo else 0.05)
        items.append(("OK" if ok else "FAIL", "F9P INT counts LIS3DH edges: %.1f /s%s"
                      % (f9p_rate, "" if ok else "; check LIS3DH INT -> breakout INT")))
    else:
        items.append(("FAIL", "F9P INT: %d time marks; check LIS3DH INT -> breakout INT pin" % len(marks)))
    try:
        cmap = ClockMap(run.pps_ns, run.rawx)
        items.append(("OK" if cmap.agreement > 0.9 else "WARN",
                      "PPS edges labelled with GPS seconds (%.0f%% of RAWX epochs agree)"
                      % (100 * cmap.agreement)))
        gt = cmap.gps_sorted(run.edge_ns if run.fifo else run.drdy_ns)
        errs = []
        for g, _, _ in tm2_edges(run, cmap.S0):
            j = bisect.bisect_left(gt, g)
            cand = [i for i in (j - 1, j) if 0 <= i < len(gt)]
            if cand:
                errs.append(min((gt[i] - g for i in cand), key=abs))
        if errs:
            med, p99, mx = spread(errs)
            ok = abs(med) < 1000 and mx < 5000
            items.append(("OK" if ok else "FAIL", "kernel timestamps vs F9P time marks: median %+.1f us, "
                          "max %.1f us (%d marks)" % (med, mx, len(errs))))
    except ValueError as error:
        items.append(("FAIL", "no GPS clock map: %s" % error))
    if run.gga:
        q, nsv = run.gga[-1][1], run.gga[-1][2]
        items.append(("OK" if q not in ("", "0") else "WARN", "GNSS fix quality %s, %s satellites" % (q, nsv)))
    if run.meta.get("qos_error"):
        items.append(("WARN", "PM QoS unavailable (%s): run 'sudo sh setup_i2c_gpio.sh' once; without "
                              "it interrupt timestamps jitter by up to ~1 ms" % run.meta["qos_error"]))
    else:
        items.append(("OK", "PM QoS active (CPUs kept out of C6/C7)"))
    c = run.meta.get("counters", {})
    bad = c.get("lost_events", 0) + c.get("overruns", 0) + c.get("rescued", 0)
    items.append(("OK" if not bad else "WARN", "lost events %d, overruns %d, rescue reads %d"
                  % (c.get("lost_events", 0), c.get("overruns", 0), c.get("rescued", 0))))
    if run.meta["config"].get("switch"):
        items.append(("OK", "tap switch on pin 31: %d edges" % len(run.sw_ns)))
    return items


def print_check(path):
    items = check(path)
    print()
    for level, text in items:
        print("[%-4s] %s" % (level, text))
    return not any(level == "FAIL" for level, _ in items)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="run directory written by bench_logger.py")
    args = parser.parse_args(argv)
    run, out = analyze(args.run)
    text = "\n".join(out.lines) + "\n"
    with open(run.file("report.txt"), "w") as f:
        f.write(text)
    print(text, end="")
    print("(report.txt and CSV tables written to %s)" % args.run)


if __name__ == "__main__":
    main()
