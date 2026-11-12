"""Run using python3 -m unittest discover tests  (works on macOS and the ROCK Pi X)."""

import os
import struct
import sys
import unittest

RADXA = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "radxa")
sys.path.insert(0, RADXA)

import zedf9p  # noqa: E402

# Synthetic position at 0 N, 0 E; do not publish recorded bench coordinates.
GGA = b"$GNGGA,120000.00,0000.00000,N,00000.00000,E,1,12,1.81,0.0,M,0.0,M,,*7A\r\n"


def rawx_payload(measurements, tow=395000.5, week=2390, leap=18, rec_stat=1):
    p = zedf9p.RAWX_HEADER.pack(tow, week, leap, len(measurements), rec_stat, 1)
    for m in measurements:
        p += zedf9p.RAWX_MEAS.pack(*m)
    return p


class FakeDDC:
    """Answers UBX requests the way the receiver does: reply message(s), then ACK."""

    def __init__(self, nak=False, values=None):
        self.out = b""
        self.written = []
        self.nak = nak
        self.values = values or {}

    def available(self):
        return len(self.out)

    def read(self, n):
        data, self.out = self.out[:n], self.out[n:]
        return data

    def write(self, frame):
        self.written.append(frame)
        msg, payload = (frame[2], frame[3]), frame[6:-2]
        if msg == zedf9p.UBX_CFG_VALGET:
            keys = struct.unpack("<%dI" % ((len(payload) - 4) // 4), payload[4:])
            body = b"\x01\x00\x00\x00" + zedf9p.encode_values({k: self.values[k] for k in keys})
            self.out += GGA[:20] + zedf9p.ubx_frame(msg, body)  # Unrelated data first.
        self.out += zedf9p.ubx_frame((0x05, 0x00 if self.nak else 0x01), bytes(msg))


class FrameTest(unittest.TestCase):
    def test_mon_ver_poll(self):
        # The well-known UBX-MON-VER poll: B5 62 0A 04 00 00 0E 34.
        self.assertEqual(zedf9p.ubx_frame(zedf9p.UBX_MON_VER).hex(), "b5620a0400000e34")

    def test_values_round_trip_with_key_sizes(self):
        values = {zedf9p.CFG_RATE_MEAS: 200, zedf9p.CFG_I2COUTPROT_NMEA: 0,
                  zedf9p.CFG_MSGOUT_RXM_RAWX_I2C: 1}
        encoded = zedf9p.encode_values(values)
        self.assertEqual(len(encoded), 3 * 4 + 2 + 1 + 1)
        self.assertEqual(zedf9p.decode_values(encoded), values)


class ParserTest(unittest.TestCase):
    def stream(self):
        rawx = zedf9p.ubx_frame(zedf9p.UBX_RXM_RAWX, rawx_payload([]))
        return b"\x00\xffjunk" + GGA + rawx + b"\xb5\x00" + zedf9p.ubx_frame((0x02, 0x13), b"ab")

    def check(self, chunks):
        p = zedf9p.StreamParser()
        messages = [m for chunk in chunks for m in p.feed(chunk)]
        self.assertEqual([(m.kind, m.ident) for m in messages],
                         [("NMEA", "GGA"), ("UBX", (0x02, 0x15)), ("UBX", (0x02, 0x13))])
        self.assertEqual(messages[0].data, GGA.rstrip())
        self.assertEqual(messages[2].data, b"ab")
        self.assertEqual(p.bad_frames, 0)

    def test_whole(self):
        self.check([self.stream()])

    def test_byte_by_byte(self):
        s = self.stream()
        self.check([s[i:i + 1] for i in range(len(s))])

    def test_bad_checksum_is_skipped(self):
        p = zedf9p.StreamParser()
        bad = bytearray(zedf9p.ubx_frame((0x02, 0x13), b"abcd"))
        bad[7] ^= 1
        good = zedf9p.ubx_frame((0x01, 0x07), b"xy")
        messages = p.feed(bytes(bad) + good)
        self.assertEqual([m.ident for m in messages], [(0x01, 0x07)])
        self.assertEqual(p.bad_frames, 1)


class RawxTest(unittest.TestCase):
    def test_decode(self):
        gps = (2.1e7, 1.1e8, -1234.5, 0, 12, 0, 0, 64000, 45, 3, 2, 4, 0x07)
        glo = (2.2e7, 0.0, 500.25, 6, 3, 2, 5, 0, 30, 5, 15, 6, 0x01)
        epoch = zedf9p.decode_rawx(rawx_payload([gps, glo]))
        self.assertEqual((epoch.week, epoch.tow, epoch.leap_s), (2390, 395000.5, 18))
        a, b = epoch.measurements
        self.assertEqual((a.gnss, a.sv, a.signal, a.glo_k), ("GPS", 12, "L1CA", None))
        self.assertEqual((a.pr_m, a.cp_cycles, a.doppler_hz), (2.1e7, 1.1e8, -1234.5))
        self.assertEqual((a.cno_dbhz, a.lock_ms, a.half_cyc_valid), (45, 64000, True))
        self.assertAlmostEqual(a.pr_std_m, 0.08)
        self.assertAlmostEqual(a.cp_std_cycles, 0.008)
        self.assertAlmostEqual(a.do_std_hz, 0.032)
        # GLONASS L2OF, channel k = freqId - 7, carrier phase not valid (trkStat bit 1 clear).
        self.assertEqual((b.gnss, b.signal, b.glo_k, b.cp_cycles, b.cp_std_cycles),
                         ("GLO", "L2OF", -2, None, None))
        rows = list(zedf9p.csv_rows(epoch))
        self.assertEqual(len(rows[0].split(",")), len(zedf9p.CSV_HEADER.split(",")))
        self.assertIn(",GLO,3,L2OF,-2,22000000.0000,,500.250,", rows[1])

    def test_leap_seconds_unknown(self):
        self.assertIsNone(zedf9p.decode_rawx(rawx_payload([], rec_stat=0)).leap_s)

    def test_short_payload(self):
        with self.assertRaises(ValueError):
            zedf9p.decode_rawx(rawx_payload([(0.0,) * 3 + (0,) * 10])[:-1])


class ReceiverTest(unittest.TestCase):
    def test_get_and_set(self):
        ddc = FakeDDC(values={zedf9p.CFG_RATE_MEAS: 1000, zedf9p.CFG_MSGOUT_RXM_RAWX_I2C: 0})
        rx = zedf9p.Receiver(ddc)
        saved = rx.get([zedf9p.CFG_RATE_MEAS, zedf9p.CFG_MSGOUT_RXM_RAWX_I2C])
        self.assertEqual(saved, {zedf9p.CFG_RATE_MEAS: 1000, zedf9p.CFG_MSGOUT_RXM_RAWX_I2C: 0})
        rx.set({zedf9p.CFG_RATE_MEAS: 200})
        valset = ddc.written[-1]
        self.assertEqual(valset[2:4], bytes(zedf9p.UBX_CFG_VALSET))
        self.assertEqual(valset[6:10], b"\x00\x01\x00\x00")  # version 0, RAM layer only
        self.assertEqual(valset[10:16], struct.pack("<IH", zedf9p.CFG_RATE_MEAS, 200))

    def test_nak_raises(self):
        with self.assertRaises(RuntimeError):
            zedf9p.Receiver(FakeDDC(nak=True)).set({zedf9p.CFG_RATE_MEAS: 10})


if __name__ == "__main__":
    unittest.main()
