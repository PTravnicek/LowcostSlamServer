"""Run using python3 -m unittest discover tests  (works on macOS and the ROCK Pi X)."""

import os
import sys
import unittest

RADXA = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "radxa")
sys.path.insert(0, RADXA)

import lis3dh  # noqa: E402


class FakeBus:
    """Register file standing in for smbus2.SMBus."""

    def __init__(self, who_am_i=0x33):
        self.regs = {lis3dh.WHO_AM_I: who_am_i}
        self.block_reads = []

    def read_byte_data(self, address, reg):
        return self.regs.get(reg, 0)

    def write_byte_data(self, address, reg, value):
        self.regs[reg] = value

    def read_i2c_block_data(self, address, reg, length):
        self.block_reads.append(reg)
        start = reg & 0x7F
        return [self.regs.get(start + i, 0) for i in range(length)]


class ConversionTest(unittest.TestCase):
    def test_one_g_at_2g(self):
        # +-2 g high resolution: 1 mg/digit, 1 g = 1000 digits = 16000 left-justified.
        self.assertEqual(lis3dh.to_g(0x80, 0x3E, 1), 1.0)
        self.assertEqual(lis3dh.to_g(0x80, 0xC1, 1), -1.0)

    def test_full_scale_extremes(self):
        self.assertEqual(lis3dh.to_g(0xF0, 0x7F, 1), 2.047)
        self.assertEqual(lis3dh.to_g(0x00, 0x80, 1), -2.048)
        self.assertEqual(lis3dh.to_g(0x10, 0x00, 12), 0.012)


class SensorTest(unittest.TestCase):
    def test_configures_high_resolution_bdu(self):
        bus = FakeBus()
        lis3dh.LIS3DH(bus, rate=100, range_g=8)
        self.assertEqual(bus.regs[lis3dh.CTRL_REG4], 0xA8)  # BDU | FS=8 g | HR
        self.assertEqual(bus.regs[lis3dh.CTRL_REG1], 0x57)  # 100 Hz | X/Y/Z on

    def test_read_uses_auto_increment(self):
        bus = FakeBus()
        sensor = lis3dh.LIS3DH(bus)
        bus.regs.update({0x28: 0x80, 0x29: 0x3E, 0x2A: 0, 0x2B: 0, 0x2C: 0x80, 0x2D: 0xC1})
        self.assertEqual(sensor.read(), (1.0, 0.0, -1.0))
        self.assertEqual(bus.block_reads, [0xA8])

    def test_power_down(self):
        bus = FakeBus()
        lis3dh.LIS3DH(bus).power_down()
        self.assertEqual(bus.regs[lis3dh.CTRL_REG1], 0x07)

    def test_rejects_wrong_chip(self):
        with self.assertRaises(RuntimeError):
            lis3dh.LIS3DH(FakeBus(who_am_i=0x32))

    def test_rejects_unsupported_rate(self):
        with self.assertRaises(ValueError):
            lis3dh.LIS3DH(FakeBus(), rate=60)


if __name__ == "__main__":
    unittest.main()
