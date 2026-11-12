#!/usr/bin/env python3
"""LIS3DH 3-axis accelerometer on the ROCK Pi X I2C header (userspace, smbus2).

Wiring, module -> ROCK Pi X 40-pin header (v1.41+ header is 3.3 V):
  VCC -> pin 1 (3.3 V)   GND -> pin 6   SDA -> pin 3   SCL -> pin 5
  SDO low (module default) = address 0x18, SDO high = 0x19.
  CS high or open = I2C mode.

The sensor runs in high-resolution (12-bit) mode with block data update, so the
three axes of one sample always belong together. Output is CSV on stdout:
wall-clock time and acceleration in g. Needs python3-smbus2 and membership in
group i2c; setup_i2c_gpio.sh installs both.

  python3 lis3dh.py                                  # 10 Hz, +-2 g, until Ctrl-C
  python3 lis3dh.py --rate 100 --range 8 --count 500 > shake.csv
"""

import argparse
import sys
import time

DEFAULT_BUS = "/dev/i2c-header-3-5"
DEFAULT_ADDRESS = 0x18

WHO_AM_I = 0x0F
WHO_AM_I_VALUE = 0x33
CTRL_REG1 = 0x20
CTRL_REG4 = 0x23
STATUS_REG = 0x27
OUT_X_L = 0x28
AUTO_INCREMENT = 0x80  # Subaddress MSB: read OUT_X_L..OUT_Z_H in one transfer.

STATUS_ZYXDA = 0x08  # New X, Y and Z sample available.
STATUS_ZYXOR = 0x80  # A sample was overwritten before it was read.
TURN_ON_SAMPLES = 7  # High-resolution mode settles after 7/ODR (datasheet table 12).

# CTRL_REG1 ODR field, normal/high-resolution mode (datasheet table 31).
RATES = {1: 0x1, 10: 0x2, 25: 0x3, 50: 0x4, 100: 0x5, 200: 0x6, 400: 0x7, 1344: 0x9}
# Full scale in g -> (CTRL_REG4 FS field, high-resolution sensitivity in mg/digit, table 4).
RANGES = {2: (0x0, 1), 4: (0x1, 2), 8: (0x2, 4), 16: (0x3, 12)}

CTRL_REG1_XYZ_ON = 0x07   # Xen, Yen, Zen; ODR 0 = power-down (reset value).
CTRL_REG4_BDU = 0x80
CTRL_REG4_HR = 0x08


def to_g(lo, hi, mg_per_digit):
    """Convert one left-justified 12-bit output register pair to g."""
    raw = hi << 8 | lo
    if raw & 0x8000:
        raw -= 0x10000
    return (raw >> 4) * mg_per_digit / 1000.0


class LIS3DH:
    def __init__(self, bus, address=DEFAULT_ADDRESS, rate=10, range_g=2):
        """bus: an smbus2.SMBus (or anything with the same byte/block methods)."""
        if rate not in RATES:
            raise ValueError(f"rate must be one of {sorted(RATES)} Hz")
        if range_g not in RANGES:
            raise ValueError(f"range must be one of {sorted(RANGES)} g")
        self.bus = bus
        self.address = address
        who = bus.read_byte_data(address, WHO_AM_I)
        if who != WHO_AM_I_VALUE:
            raise RuntimeError(f"no LIS3DH at 0x{address:02x}: WHO_AM_I=0x{who:02x}, "
                               f"expected 0x{WHO_AM_I_VALUE:02x}")
        fs, self.mg_per_digit = RANGES[range_g]
        bus.write_byte_data(address, CTRL_REG4, CTRL_REG4_BDU | fs << 4 | CTRL_REG4_HR)
        bus.write_byte_data(address, CTRL_REG1, RATES[rate] << 4 | CTRL_REG1_XYZ_ON)

    def status(self):
        return self.bus.read_byte_data(self.address, STATUS_REG)

    def read(self):
        """Latest sample as (x, y, z) in g."""
        d = self.bus.read_i2c_block_data(self.address, OUT_X_L | AUTO_INCREMENT, 6)
        return tuple(to_g(d[i], d[i + 1], self.mg_per_digit) for i in (0, 2, 4))

    def power_down(self):
        self.bus.write_byte_data(self.address, CTRL_REG1, CTRL_REG1_XYZ_ON)


def open_bus(path):
    from smbus2 import SMBus  # Imported here so the tests run without smbus2.
    return SMBus(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bus", default=DEFAULT_BUS, help=f"I2C device (default {DEFAULT_BUS})")
    parser.add_argument("--address", type=lambda s: int(s, 0), default=DEFAULT_ADDRESS,
                        help="0x18 (SDO low, default) or 0x19 (SDO high)")
    parser.add_argument("--rate", type=int, choices=sorted(RATES), default=10, help="Hz")
    parser.add_argument("--range", type=int, choices=sorted(RANGES), default=2, help="+- g")
    parser.add_argument("--count", type=int, default=0, help="samples to take (0 = until Ctrl-C)")
    args = parser.parse_args(argv)

    bus = open_bus(args.bus)
    sensor = None
    samples = overruns = 0
    settling = TURN_ON_SAMPLES
    poll = 0.1 / args.rate
    try:
        sensor = LIS3DH(bus, args.address, args.rate, args.range)
        print("time_s,x_g,y_g,z_g")
        while not args.count or samples < args.count:
            status = sensor.status()
            if not status & STATUS_ZYXDA:
                time.sleep(poll)
                continue
            x, y, z = sensor.read()
            if settling:
                settling -= 1
                continue
            if status & STATUS_ZYXOR:
                overruns += 1
            print(f"{time.time():.3f},{x:.4f},{y:.4f},{z:.4f}")
            samples += 1
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        if sensor:
            sensor.power_down()
        bus.close()
    if overruns:
        print(f"warning: {overruns} of {samples} samples followed a missed one "
              f"(rate too high to poll)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
