# LIS3DH FIFO timing fix: handoff notes

Written 2026-11-08 for the next Claude instance (or person) that picks this bench up. Read this before touching
`radxa/bench_logger.py` or `radxa/bench_analyze.py`.

## What the bench is

A ROCK Pi X with a ZED-F9P (GNSS) and a LIS3DH accelerometer. The goal is a GPS-time stamp with at most 1 ms error
on every LIS3DH sample, so the accelerometer can fill the gaps between GNSS fixes. The kernel time-stamps pin edges
(`CLOCK_MONOTONIC`); the F9P PPS maps that clock to GPS time; the F9P also time-stamps the LIS3DH INT wire itself
(UBX-TIM-TM2) as an independent check. Wiring (unchanged, nothing was rewired for this fix):

| Signal | Header pin | Linux |
|---|---|---|
| F9P PPS | 7 | gpiochip2 line 21 |
| LIS3DH INT1 (also wired to the F9P INT/EXTINT pin) | 29 | gpiochip2 line 24 |
| tap switch (T5, not fitted yet) | 31 | gpiochip2 line 15 |
| F9P I2C | 3 / 5 | `/dev/i2c-header-3-5`, 0x42 |
| LIS3DH I2C | 27 / 28 | `/dev/i2c-header-27-28`, 0x18 |

Both I2C buses run at 400 kHz (about 21 us per byte, measured). The design document is a Claude Doc artifact
("ROCK Pi X timing bench", 3 Nov 2026); it was not readable by Claude through the artifact tools (not shared), so this
file and the docstrings in the two scripts are the written record.

## What T1-T4 showed on 2026-11-03 (old per-sample reading, runs in `~/bench/T*-20261103-*`)

PASS in every run: PPS timestamp jitter with PM QoS (p99 13-39 us) and kernel edge vs F9P time mark (p99 13-22 us).
PM QoS matters: T1 without it had PPS jitter std 275 us, p99 651 us; with it 8.6 us, p99 17.5 us.

FAIL in every run: sample integrity (0.10 % lost in T1, 0.15 % T2, 0.43 % T3, 0.42 % T4, gaps up to 11 samples) and
"predict sample times 1 s after a PPS" (errors 0.5-4 ms). T4 also had a PPS glitch (an extra edge 89 ms before a real one)
and F9P/kernel edge counters that disagreed.

Root causes, from the raw data:

1. **Per-sample reading cannot be made lossless.** DRDY on INT1 is a level. If a read is late, the next sample arrives
   while INT1 is still high, there is no new edge, and that sample has no timestamp (and may be overwritten). Late
   reads come from rare scheduling stalls of 5-45 ms. It is not bus bandwidth: each transaction costs 0.43-0.7 ms of
   which the wire time is only about 0.25 ms. Things that did NOT fix it (180 s A/B runs at 400 Hz, 2 runs each, loss
   varied 0.01 % to 1.1 % between identical runs): moving file writes to a thread, removing the GNSS worker, pinning
   to one core (worse). SCHED_FIFO was not tried (radxa has no passwordless sudo).
2. **The LIS3DH clock is not nominal and not constant.** 100 Hz mode runs at 100.78-100.84 Hz, 400 Hz mode at 397.5 Hz.
   Gap-filled per-minute rate still wandered 400-900 ppm, residual against a 60 s line fit 350-800 us. So sample times
   must NOT be counted or extrapolated; every sample needs a real edge near it. The old "predict after PPS" test cannot pass.
3. **The old analyzer's per-minute rate swings (up to 8770 ppm) were loss artifacts**, not real drift.
4. **The old PPS gate kept a spurious edge and dropped the real one.**

## The fix: FIFO watermark edges as time anchors

`bench_logger.py` (default mode, `--no-fifo` restores the old behaviour):

- LIS3DH FIFO in stream mode (FIFO_CTRL 0x2E = 0x80 | threshold, CTRL_REG5 0x24 = 0x40), INT1 routed to the
  **watermark** (CTRL_REG3 0x22 = 0x04) instead of data-ready. Default threshold 8 (`--fifo-threshold`).
- The kernel time-stamps each rising watermark edge. On each edge the logger drains the FIFO (FIFO_SRC 0x2F read, one
  burst read of n*6 bytes from OUT_X_L|0x80, repeat until FIFO_SRC says empty). A multi-sample burst from OUT_X_L pops
  consecutive samples (verified). The FIFO holds 32 samples = 80 ms at 400 Hz, so a read may be about 58 ms late.
- Output: `accel.csv` rows are written per sample with `seqno` = sample number and `drdy_ns` = 0 (placeholder);
  `edges.csv` (new) has one row per watermark edge: `mono_ns,seqno,popped,fss,read_ns`. `meta.json` gets
  `"mode": "fifo"` and `"fifo_threshold"`.

`bench_analyze.py`:

- **The watermark edge rises at FIFO fill = threshold + 1, not threshold** (datasheet says "equal or higher"). Measured:
  fill at read minus floor(latency / period) = 9 for 12954 of 12956 edges at threshold 8. The analyzer measures this
  level from the data (`crossing_level`); a wrong level shifts every timestamp by one sample (2.5 ms at 400 Hz).
- Edge to sample number: first edge = popped_before + level; each next edge adds round(dt / sample period). Edges lie on
  the sample grid to 0.003 samples, so this is unambiguous even for missed edges. Sample times are linear
  interpolations between edges (`interpolate_sample_times`); `run.drdy_ns` is filled with them so sections 2-5 and the
  tap test work unchanged.
- Section 2 in FIFO mode: interpolation check by leave-one-out (each edge predicted from its two neighbours, limit
  1 ms) replaces the "predict 1 s after PPS" test. F9P time marks take their sample number from the matched kernel edge
  (the F9P counts watermark edges, which are 9-14 samples apart).
- Section 4 in FIFO mode: lost samples = anchor number - level - popped_before > 0 (positive means the FIFO dropped
  samples); FIFO-full events (FIFO_SRC bit 6) are counted as overruns; late or missed edges cost no samples and are
  only reported. A late edge is therefore no longer a failure.
- `ClockMap` PPS gate: keeps an edge only if it lies within 20 ms of a whole number of seconds after the last good one.
  The logger itself does not gate PPS live.

Also learned (do not re-test): with the FIFO enabled and INT1 on data-ready, only about 23 % of samples produce an
edge (DRDY stays high while the FIFO is unread), so DRDY and FIFO cannot be combined. OVRN in FIFO_SRC means "FIFO full".
FSS is 5 bits, so a full FIFO (32) reads as 0 with OVRN set; the logger handles that.

## Files

On the radxa, `~/ola-receiver/`:

- `radxa/bench_logger.py`, `radxa/bench_analyze.py`, `tests/test_bench_analyze.py`: changed.
- `backup-before-fifo-20261108/`: the originals of those three files.
- `FIFO_TIMING_HANDOFF.md`: this file.

Other copies: the user's Windows machine has `LowcostSlamServer/scripts/bench/` with the same files (untracked in git
at the time of writing). The user's canonical project lives on a Mac (`RADXA_RXTX`, `deploy.sh` copies the board-side
files to `~/ola-receiver/`), so **copy these changes back to the Mac, or the next deploy.sh run overwrites them.**
Scratch material from the investigation (A/B runs, probes) is in `~/bench_exp/`; it can be deleted.

## How to run

```
cd ~/ola-receiver/radxa
python3 bench_logger.py check            # 15 s, should be all OK
python3 bench_logger.py T4               # 60 min, 400 Hz FIFO mode (default)
python3 bench_analyze.py ~/bench/<run dir>
python3 bench_logger.py T4 --no-fifo     # old per-sample mode, for comparison
python3 bench_logger.py T4 --fifo-threshold 16   # fewer reads, less margin before the FIFO fills
cd ~/ola-receiver && python3 -m unittest discover -s tests -v    # 36 tests
```

## Status and results

Installed on the radxa on 2026-11-08 and validated there. Unit tests: 37 pass
(`cd ~/ola-receiver && python3 -m unittest discover -s tests -v`). `bench_logger.py check`: all OK, including
"lost events 0, overruns 0, rescue reads 0".

Hardware runs in FIFO mode (threshold 8), all with the analyzer's summary 4 PASS / 0 FAIL. Run directories and
`SUMMARY.txt` are in `~/bench/fifo-validation/`. Durations are shortened from the presets (T1 1500 s instead of 3600 s,
so its CPU-load phase never ran; T2 600 s instead of 7200 s, so its warm-up phase never ran; T3 600 s instead of 1800 s;
T4 900 s instead of 3600 s).

| Run | Samples | Lost samples | FIFO full | Interpolation error p99 / max | Kernel edge vs F9P p99 | Worst edge-to-read delay |
|---|---|---|---|---|---|---|
| T4, 400 Hz, GNSS 10 Hz, 900 s | 357,645 | 0 | 0 | 16 / 126 us | 16.4 us | 16.5 ms (FIFO full after 58 ms) |
| T3, 100 Hz, GNSS 10 Hz, 600 s | 60,455 | 0 | 0 | 32 / 90 us | 17.9 us | 15.8 ms (FIFO full after 228 ms) |
| T2, 100 Hz, GNSS 1 Hz, 600 s | 60,458 | 0 | 0 | 32 / 49 us | 18.9 us | 16.2 ms |
| T1, 100 Hz, 1500 s | 151,121 | 0 | 0 | 33 / 84 us (QoS phase) | 20.8 us (QoS phase) | 16.8 ms |

For comparison the old per-sample mode on 2026-11-03 lost 0.10-0.43 % of samples in every run.

T1 phase 1 (PM QoS off, deep C-states allowed) is informative and is NOT judged: PPS interval error std 286 us,
p99 672 us; kernel edge vs F9P p99 626 us; interpolation error p99 574 us, max 1122 us. So PM QoS is required for the
sample times too, not just for the PPS. (The first version of the interpolation check judged this phase and failed T1;
it now judges only QoS-on phases like the other checks.)

Measured sample clock (gap-free, so these are the real numbers): 100 Hz mode runs at 100.74-100.75 Hz (+7450 ppm),
400 Hz mode at 397.33 Hz (-6660 ppm). Per-minute variation is 200-400 ppm, so it still wanders; interpolating between
edges handles that, counting would not.

Does buffering reduce I2C load? (90 s at 400 Hz, GNSS 10 Hz, same code, `--no-fifo` vs default; `cmp_load.sh` in
`~/bench/fifo-validation/`):

| | per-sample (`--no-fifo`) | FIFO |
|---|---|---|
| Wake-ups of the reader | about 397/s | about 44/s |
| I2C transactions (from the code) | about 400/s | about 130/s (status, burst, status per batch) |
| Accelerometer-bus interrupts | 3159/s | 2581/s (the DesignWare controller interrupts about once per byte, so this barely drops) |
| CPU time of the whole logger | 12.2 s user + 3.3 s sys | 6.0 s user + 1.7 s sys |
| Tolerated read delay before a sample is lost | none (one sample period) | 58 ms at 400 Hz |

So the buffer does not reduce bytes on the wire much; it removes the per-sample wake-up and Python work (CPU about
halved) and, more importantly, makes a late read harmless.

## Open items

1. T5 (tap test) needs the microswitch on pin 31. It is the only way to measure a constant offset between the
   watermark assertion and the true sample instant, and the sensor's filter delay. Until then, a few us of constant
   offset are unmeasured.
2. The production server (`LowcostSlamServer/app/core/imu_i2c.py` on the user's machine) still polls the LIS3DH and stamps
   with `time.time_ns()`. It should get the same FIFO + watermark-edge + PM QoS approach (hold `/dev/cpu_dma_latency` at
   10 us while recording) and write per-sample GPS-mappable stamps.
3. SCHED_FIFO for the sampling process (needs root; the server already runs under sudo) was never tried. With the FIFO
   it may not be needed: the worst stall seen was 16.5 ms against 58 ms of FIFO margin.
4. The logger does not reject spurious PPS edges live; only the analyzer does. A gpiod debounce on the PPS line would
   fix that for a live system.
5. The T2 warm-up phase (sensor temperature coefficient of the sample clock) only runs in the full 120 min T2.
6. The artifact text is outdated: T2 criterion, FIFO mode and options, `i2cdetect` hint (use the
   `/dev/i2c-header-*` symlinks instead of `-r 4`, the bus number changes between boots).
