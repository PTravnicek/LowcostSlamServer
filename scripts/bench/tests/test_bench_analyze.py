"""Run using python3 -m unittest discover tests  (works on macOS and the ROCK Pi X).

bench_analyze.py on a synthetic run with known clock errors, jitter and delays.
"""

import json
import os
import random
import sys
import tempfile
import unittest

RADXA = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "radxa")
sys.path.insert(0, RADXA)

import bench_analyze as ba  # noqa: E402
import bench_logger  # noqa: E402

NS = ba.NS
WEEK = ba.WEEK_S
S0 = 2443 * WEEK + 222000      # GPS second of the first PPS edge
M0 = 5_000_000_000_000         # CLOCK_MONOTONIC at S0
LATENCY_NS = 20_000            # Same interrupt latency for PPS and data-ready edges.


def make_run(d, seconds=90, odr=100, odr_true=100.82, radxa_ppm=40.0, jitter_ns=3000,
             drop=(), lose=(), taps=(), tap_delay_ms=3.0, leap=18, tm2_base=2, gnss_rate=1,
             fifo_level=0, fifo_lose=None):
    """Write a run directory in bench_logger.py format; returns the true edge GPS times."""
    rng = random.Random(7)

    def mono(g_ns):
        return int(M0 + g_ns * (1 + radxa_ppm * 1e-6))

    def stamp(g_ns):
        return mono(g_ns) + LATENCY_NS + int(rng.gauss(0, jitter_ns))

    def w(name, header, rows):
        with open(os.path.join(d, name), "w") as f:
            f.write(header + "\n" + "".join(",".join(map(str, r)) + "\n" for r in rows))

    w("pps.csv", "mono_ns,seqno", [(stamp(k * NS), k + 1) for k in range(seconds)])
    rawx = []
    for i in range(int(seconds * gnss_rate)):
        g = i * NS / gnss_rate
        rawx.append(("rawx", mono(g) + 120_000_000, S0 // WEEK, repr(S0 % WEEK + g / NS - 0.008),
                     leap, 1, 30))

    mg = 4  # +-8 g
    one_g = round(1000 / mg) * 16
    period = NS / odr_true
    accel, edges, seq, n = [], [], 0, 0

    def tap_x(g):
        x = 0
        for t in taps:
            dt = g - (t * NS + tap_delay_ms * 1e6)
            if 0 <= dt < 20e6:
                x = int(0.5 * one_g * min(1, dt / 2e6))
        return x

    if fifo_level:
        # FIFO mode: the watermark edge rises with sample number `popped + fifo_level` (the FIFO
        # was read empty), the read follows ~0.4 ms later and takes everything that arrived.
        # fifo_lose = (drain number, samples dropped): the FIFO overflowed and lost the oldest.
        arrival = lambda i: 300_000_000 + i * period      # Arrival of 1-based sample number i.
        made = delivered = k = 0
        fifo_rows, edge_rows = [], []
        while True:
            number = made + fifo_level
            g = arrival(number)
            if g >= (seconds - 1) * NS:
                break
            read_g = g + 400_000 + rng.randrange(300_000)
            produced = int((read_g - 300_000_000) // period)
            first = made
            if fifo_lose and fifo_lose[0] == k:
                first += fifo_lose[1]
            popped_before = delivered
            for i in range(first, produced):
                fifo_rows.append((delivered, 0, int(mono(read_g)), 0x08, tap_x(arrival(i + 1)), 0, one_g))
                delivered += 1
            k += 1
            edges.append((g, k))
            edge_rows.append((stamp(g), k, popped_before, produced - made, int(mono(read_g))))
            made = produced
        w("accel.csv", "seqno,drdy_ns,read_ns,status,x,y,z", fifo_rows)
        w("edges.csv", "mono_ns,seqno,popped,fss,read_ns", edge_rows)
        n = -1
    while n >= 0:
        g = 300_000_000 + n * period
        if g >= (seconds - 1) * NS:
            break
        n += 1
        if n in drop:           # Sensor never re-armed: no edge anywhere.
            continue
        seq += 1
        edges.append((g, seq))
        if n in lose:           # Edge seen by the kernel but its event was dropped.
            continue
        x = tap_x(g)
        drdy = stamp(g)
        accel.append((seq, drdy, drdy + 400_000 + rng.randrange(300_000), 0x0F, x, 0, one_g))
    if not fifo_level:
        w("accel.csv", "seqno,drdy_ns,read_ns,status,x,y,z", accel)

    tm2 = []
    for k in range(1, seconds - 1):
        g, s = max((e for e in edges if e[0] < k * NS), default=(None, None))
        if g is None:
            continue
        t = S0 * NS + int(g) - (leap * NS if tm2_base == 2 else 0)
        wn, rem = divmod(t, WEEK * NS)
        flags = 0x80 | 0x40 | tm2_base << 3
        # UBX-TIM-TM2 payload: ch flags count wnR wnF towMsR towSubMsR towMsF towSubMsF accEst
        payload = bench_logger.TM2.pack(0, flags, (s + 1000) % 65536, wn, wn + 1,
                                        rem // 1_000_000, rem % 1_000_000, 1, 2, 30)
        tm2.append(("tm2", mono(k * NS) + 150_000_000) + bench_logger.TM2.unpack(payload))
    gga = [("gga", mono(k * NS) + 200_000_000, 2, 12, 1.1) for k in range(seconds)]
    with open(os.path.join(d, "gnss.csv"), "w") as f:
        f.write("# comment line\n")
        for r in rawx + tm2 + gga:
            f.write(",".join(map(str, r)) + "\n")

    switch = []
    for t in taps:
        m = mono(t * NS) + LATENCY_NS
        switch += [(m, 0, 1), (m + 1_000_000, 0, 0), (m + 2_000_000, 0, 1), (m + 300_000_000, 0, 0)]
    w("switch.csv", "mono_ns,seqno,rising", switch)
    w("temp.csv", "mono_ns,cpu_mc,lis3dh_raw", [(mono(k * NS), 45000, 25 * 256) for k in range(seconds)])
    w("markers.csv", "mono_ns,elapsed_s,qos,load,label", [(M0 - NS, 0.0, 1, 0, "PM QoS on, steady")])
    meta = {"about": "synthetic", "duration_s": seconds, "ended": "duration",
            "lis3dh_mg_per_digit": mg, "counters": {"gnss_bad_frames": 0},
            "config": {"duration": seconds, "odr": odr, "gnss_rate": gnss_rate, "range": 8,
                       "switch": bool(taps)}}
    if fifo_level:
        meta.update(mode="fifo", fifo_threshold=fifo_level - 1,
                    counters={"gnss_bad_frames": 0, "rescued": 0, "overruns": 0})
    with open(os.path.join(d, "meta.json"), "w") as f:
        json.dump(meta, f)
    return edges


class BenchAnalyzeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def report(self):
        run, out = ba.analyze(self.dir)
        return run, "\n".join(out.lines), out.verdicts

    def test_clock_map_recovers_rate_and_labels(self):
        make_run(self.dir)
        run = ba.Run(self.dir)
        cmap = ba.ClockMap(run.pps_ns, run.rawx)
        self.assertEqual(cmap.S0, S0)
        self.assertEqual(cmap.agreement, 1.0)
        self.assertAlmostEqual((cmap.slope / NS - 1) * 1e6, 40.0, delta=0.5)
        mono = list(range(M0 + NS, M0 + 80 * NS, 777_777_777))
        self.assertEqual(list(cmap.gps_sorted(mono)), [cmap.gps_ns(m) for m in mono])

    def test_clean_run_passes_and_finds_true_rate(self):
        make_run(self.dir)
        run, text, verdicts = self.report()
        self.assertTrue(all(ok for ok, _ in verdicts), text)
        cmap = ba.ClockMap(run.pps_ns, run.rawx)
        edges = ba.tm2_edges(run, cmap.S0)  # UTC time marks + 18 leap seconds
        rate = ba.fit_rate([g / NS for g, _, _ in edges], ba.unwrap16([c for _, c, _ in edges]))
        self.assertAlmostEqual(rate, 100.82, delta=100.82e-7)
        self.assertIn("(+8200 ppm vs nominal)", text)
        self.assertIn("edge counters (F9P count - kernel seqno) constant", text)

    def test_kernel_path_error_is_jitter_sized(self):
        make_run(self.dir)
        self.report()
        with open(os.path.join(self.dir, "path_errors.csv")) as f:
            errors = [abs(float(line.split(",")[1])) for line in list(f)[1:]]
        self.assertGreater(len(errors), 80)
        self.assertLess(max(errors), 20)  # us; 3 us jitter on each of three edges

    def test_gps_time_base_marks(self):
        make_run(self.dir, tm2_base=1)
        _, text, verdicts = self.report()
        self.assertTrue(all(ok for ok, _ in verdicts), text)

    def test_missed_and_lost_samples_fail_integrity(self):
        make_run(self.dir, drop={1000, 3000, 3001}, lose={5000})
        _, text, verdicts = self.report()
        self.assertIn("kernel lost events 1, missed samples (no edge) 3", text)
        self.assertIn("rescue reads (INT1 stuck high) 0", text)
        self.assertIn("FAIL: lost 1 / missed 3", text)
        self.assertIn("constant: no edge seen by one and missed by the other", text)
        self.assertIn("WARNING: 3 samples had no edge", text)

    def test_rescue_reads_are_excluded_and_reported(self):
        make_run(self.dir)
        with open(os.path.join(self.dir, "accel.csv"), "a") as f:
            f.write("-1,-1,5000000000000,15,0,0,4000\n")
        run, text, _ = self.report()
        self.assertEqual(run.rescued, 1)
        self.assertNotIn(-1, run.drdy_ns)
        self.assertIn("rescue reads (INT1 stuck high) 1", text)
        self.assertIn("rescued 1 samples", text)

    def test_taps_measure_delay_and_ignore_bounce(self):
        make_run(self.dir, odr=400, odr_true=401.0, taps=(20, 30, 40, 50, 60), tap_delay_ms=3.0)
        _, text, verdicts = self.report()
        self.assertIn("switch closures 5, taps with a detected onset 5", text)
        with open(os.path.join(self.dir, "taps.csv")) as f:
            offsets = [float(line.split(",")[1]) for line in list(f)[1:]]
        self.assertEqual(len(offsets), 5)
        for o in offsets:
            self.assertAlmostEqual(o, 3.0, delta=1000 / 401)  # within one sample period
        self.assertTrue(all(ok for ok, _ in verdicts), text)

    def test_check_items(self):
        make_run(self.dir, seconds=15)
        levels = {text.split(":")[0]: level for level, text in ba.check(self.dir)}
        self.assertEqual(levels["PPS on pin 7"], "OK")
        self.assertEqual(levels["LIS3DH data-ready on pin 29"], "OK")
        self.assertEqual(levels["F9P INT counts LIS3DH edges"], "OK")
        self.assertEqual(levels["kernel timestamps vs F9P time marks"], "OK")

    def test_missing_pps_is_reported(self):
        make_run(self.dir, seconds=15)
        with open(os.path.join(self.dir, "pps.csv"), "w") as f:
            f.write("mono_ns,seqno\n")
        _, text, _ = self.report()
        self.assertIn("No GPS clock map", text)
        levels = [level for level, text in ba.check(self.dir) if text.startswith("PPS")]
        self.assertEqual(levels, ["FAIL"])

class FifoRunTest(unittest.TestCase):
    """FIFO runs: kernel time stamps on the watermark edges, sample times interpolated."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def test_sample_times_follow_the_true_arrival_times(self):
        make_run(self.dir, odr=400, odr_true=397.5, fifo_level=9)
        run, out = ba.analyze(self.dir)
        text = "\n".join(out.lines)
        self.assertTrue(all(ok for ok, _ in out.verdicts), text)
        self.assertEqual(run.level, 9)
        period = NS / 397.5
        worst = 0
        for i in range(0, len(run.drdy_ns), 37):
            true = mono_of(300_000_000 + (i + 1) * period) + LATENCY_NS
            worst = max(worst, abs(run.drdy_ns[i] - true))
        self.assertLess(worst, 30_000)   # ns: 3 us jitter on the edges, interpolated
        cmap = ba.ClockMap(run.pps_ns, run.rawx)
        rate = ba.fit_rate([g / NS for g in cmap.gps_sorted(run.drdy_ns)], list(run.seq))
        self.assertAlmostEqual(rate, 397.5, delta=0.004)   # 10 ppm
        self.assertIn("constant: no edge seen by one and missed by the other", text)
        self.assertIn("lost samples 0", text)

    def test_overflowed_fifo_is_reported_as_lost_samples(self):
        make_run(self.dir, odr=400, odr_true=397.5, fifo_level=9, fifo_lose=(300, 3))
        _, out = ba.analyze(self.dir)
        text = "\n".join(out.lines)
        self.assertIn("lost samples 3", text)
        self.assertTrue(any(not ok and "lost 3" in t for ok, t in out.verdicts), text)

    def test_spurious_pps_edge_does_not_replace_the_real_one(self):
        make_run(self.dir, odr=400, odr_true=397.5, fifo_level=9)
        path = os.path.join(self.dir, "pps.csv")
        with open(path) as f:
            lines = f.read().splitlines()
        real = int(lines[30].split(",")[0])                 # Real edge; the glitch comes 89 ms before it.
        lines.insert(30, "%d,0" % (real - 89_000_000))
        with open(path, "w") as f:
            f.write("\n".join(lines) + "\n")
        run = ba.Run(self.dir)
        cmap = ba.ClockMap(run.pps_ns, run.rawx)
        self.assertEqual(len(cmap.t), 90)
        self.assertIn(real, cmap.t)

    def test_interpolation_is_only_judged_in_phases_with_pm_qos(self):
        make_run(self.dir, odr=100, odr_true=100.8, fifo_level=9, jitter_ns=400_000)  # 0.4 ms jitter
        _, out = ba.analyze(self.dir)
        self.assertTrue(any(not ok and "interpolated" in t for ok, t in out.verdicts),
                        "\n".join(out.lines))
        with open(os.path.join(self.dir, "markers.csv"), "w") as f:      # Same run, QoS off.
            f.write("mono_ns,elapsed_s,qos,load,label\n%d,0.0,0,0,idle, deep C-states allowed\n"
                    % (M0 - NS))
        _, out = ba.analyze(self.dir)
        text = "\n".join(out.lines)
        self.assertFalse(any("interpolated" in t for _, t in out.verdicts), text)
        self.assertIn("interpolation error", text)


def mono_of(g_ns):
    return int(M0 + g_ns * (1 + 40.0 * 1e-6))


if __name__ == "__main__":
    unittest.main()
