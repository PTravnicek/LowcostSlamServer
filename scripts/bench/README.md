# ROCK Pi X GNSS / accelerometer timing bench

Standalone timing and data-integrity experiments for ROCK Pi X v1.41, LIS3DH and
u-blox ZED-F9P. This is **not integrated into the server's acquisition code**.
It records FIFO batches, kernel GPIO edge timestamps and GNSS time references
for offline analysis. No camera SDK or server dependencies are needed.

## Quick start: no hardware required

From the repository root, using Python 3.10+:

```sh
python3 -B -m unittest discover -s scripts/bench/tests -p 'test_*.py' -v
python3 scripts/bench/radxa/bench_logger.py --help
```

The 30 imported tests use synthetic data/fake hardware and the Python standard
library. Actual collection needs Linux, libgpiod v2 Python bindings and smbus2.

## Board setup and wiring

Tested on Debian 13 / ROCK Pi X, not generic Radxa boards. On the board only:

```sh
sudo sh scripts/bench/radxa/setup_i2c_gpio.sh
```

This installs packages, configures i2c-dev, assigns GPIO/I2C group membership and
writes `/etc/udev/rules.d/60-rockpix-header.rules`. Log out and back in afterward.
It grants access to `/dev/cpu_dma_latency` for the logger's 10-microsecond PM QoS
request. It does **not** configure I2C speed; historical validation used 400 kHz.

All pin numbers below are **physical header pins**. Use common ground, suitable
3.3-V power/signals, and verify breakout power requirements; do not apply 5-V
signals to the header.

| Connection | Header / interface |
| --- | --- |
| F9P SDA / SCL | Pins 3 / 5, `/dev/i2c-header-3-5`, address `0x42` |
| LIS3DH SDA / SCL | Pins 27 / 28, `/dev/i2c-header-27-28`, address `0x18` |
| F9P PPS | Pin 7, gpiochip2 line 21 |
| LIS3DH INT1 | Pin 29, gpiochip2 line 24 **and** F9P INT/EXTINT |
| T5 microswitch | Pin 31, gpiochip2 line 15; other contact to 3.3 V |
| Common ground | E.g. pin 6 |

The bench's LIS3DH wiring differs from the standalone `lis3dh.py` default.
LIS3DH SDO is low; CS must permit I2C operation. GPIO/bus paths are fixed in the
bench logger. Use appropriate I2C pull-ups and a GNSS antenna with open sky;
wait for a valid GNSS time reference/PPS. The switch is needed only for T5.

## Collect and analyze

Only one application may own the sensors: deliberately stop the server and
other sensor loggers before collecting. Even `check` configures hardware and
writes a recording; it is not a read-only probe. Run normally without sudo:

```sh
python3 scripts/bench/radxa/bench_logger.py check
# Short smoke test, not a substitute for the full preset:
python3 scripts/bench/radxa/bench_logger.py T4 --duration 60 --out "$HOME/bench"
# Full preset:
python3 scripts/bench/radxa/bench_logger.py T4
# Substitute the actual directory printed by the logger:
python3 scripts/bench/radxa/bench_analyze.py "$HOME/bench/T4-YYYYMMDD-HHMMSS"
```

Analysis writes `report.txt` and derived CSVs into the run directory. Inspect
PASS/FAIL/WARN messages: the analyzer's exit status alone is not a test verdict,
and missing evidence may prevent some checks from running.

| Preset | Duration | Accelerometer | GNSS RAWX | Purpose |
| --- | --- | --- | --- | --- |
| check | 15 s | 100 Hz | 1 Hz | Wiring and timing sanity check |
| T1 | 1 h | 100 Hz | 1 Hz | 20 min QoS off, 20 min on, 20 min on + CPU load |
| T2 | 2 h | 100 Hz | 1 Hz | Long-run drift; manual warm-up prompt in final 15 min |
| T3 | 30 min | 100 Hz | 10 Hz | GNSS throughput |
| T4 | 1 h | 400 Hz | 10 Hz | High-rate accelerometer / GNSS |
| T5 | 10 min | 400 Hz | 1 Hz | Switch-correlated taps, about 30 spaced 5–10 s apart |

T1 loads all CPU cores; supervise temperature and power. Follow the T2 prompt
without overheating hardware. GNSS RAWX rate is not navigation-fix rate:
navigation remains about 1 Hz. Rates above 10 Hz are not validated as lossless.

FIFO stream mode is the default. The default threshold register is 8; the
measured watermark crossing occurs at 9 samples. Default acceleration range is
±8 g. See `--help` for overrides; use positive durations and GNSS rates. Keep
PM QoS enabled for precision testing except the intentional T1 comparison.
Missing QoS permissions only warn and do not necessarily stop collection.

Ctrl-C/SIGTERM requests cleanup and restoration of receiver settings. Power
loss or hard termination may prevent restoration. `--no-fifo` is a legacy
comparison mode, not a recommended fallback: the current code does not clear
FIFO configuration when switching modes, so reset/power-cycle the sensor before
comparing against that mode.

## Data interpretation and remaining validation

- Keep the **whole run directory**. In FIFO mode `accel.csv` has placeholder
  `drdy_ns=0`; it is not a valid sample time. `read_ns` is host read completion,
  not sampling time. Raw axes are signed left-justified sensor counts, not g.
- `edges.csv`, `pps.csv`, `gnss.csv` and `meta.json` are needed to reconstruct
  sample times. The analyzer interpolates between watermark edges and maps
  monotonic time to GPS time offline. It does not rewrite `accel.csv` or export
  a complete GPS-timestamped sample stream.
- Do not calculate sample times from sample index / nominal rate: the LIS3DH
  oscillator differs from nominal and drifts. FIFO capacity is finite; the
  historical default at 400 Hz provided about 58 ms of service-delay margin.
- PPS glitch rejection is offline only. Timing results do not establish
  acceleration calibration, position accuracy or inertial dead-reckoning quality.
- Historical shortened FIFO runs reported zero lost samples, but full T1 CPU
  load, T2 warm-up and T5 delay characterization still need validation. T5 does
  not establish a universally constant filter delay for every signal/configuration.
- Preserve raw measurements outside Git: GNSS recordings can disclose location
  and time, and metadata can disclose host/configuration details.

## Import provenance

Retrieved on 2026-11-12 from the board's `~/ola-receiver/` into a branch based on
upstream commit `581e232ab4c0a3cb43a13ac482f35ca32ee2dd32`.

`radxa/` contains the five original bench/driver/setup files. `tests/` contains
three relevant test modules. Runtime sources are unchanged from the retrieved
board copy. The NMEA parser test fixture was replaced with a synthetic 0 N,
0 E position and a valid checksum to avoid publishing a possible bench location.

[FIFO_TIMING_HANDOFF.md](FIFO_TIMING_HANDOFF.md) is the historical 2026-11-08
handoff, preserved as provenance. Its paths, full-suite test counts and deploy
instructions describe the original project, not this import. Follow this README
for the current layout. Unrelated receiver/service scripts, board server edits,
credentials, bytecode and raw recordings are intentionally not included.

This import does not deploy anything or update the older `RADXA_RXTX` directory;
its old deploy script must not be used to overwrite the board's updated sources.
