import math
import unittest

from proxies import packet_format as pf
from tests.packets import build_packet, eeg_raw_block


class SplitPayloadTest(unittest.TestCase):

    def test_new_firmware_packet_has_ppg(self):
        eeg = eeg_raw_block(1)
        payload = build_packet(5, eeg, [1234, -1])[1:]
        self.assertEqual(len(payload), 244)

        eeg_bytes, ppg_bytes, leftover = pf.split_payload(payload)

        self.assertEqual(len(eeg_bytes), 240)
        self.assertEqual(ppg_bytes, bytes([0x04, 0xD2, 0xFF, 0xFF]))
        self.assertEqual(leftover, 0)
        self.assertEqual(pf.decode_eeg_raw(eeg_bytes), eeg)

    def test_old_firmware_packet_has_no_ppg(self):
        eeg = eeg_raw_block(2)
        payload = build_packet(5, eeg)[1:]
        self.assertEqual(len(payload), 240)

        eeg_bytes, ppg_bytes, leftover = pf.split_payload(payload)

        self.assertIsNone(ppg_bytes)
        self.assertEqual(leftover, 0)
        self.assertEqual(pf.decode_eeg_raw(eeg_bytes), eeg)

    def test_unexpected_trailing_bytes_are_reported(self):
        payload = build_packet(0, eeg_raw_block(3))[1:] + b"\x01\x02\x03"
        eeg_bytes, ppg_bytes, leftover = pf.split_payload(payload)
        self.assertIsNone(ppg_bytes)
        self.assertEqual(leftover, 3)
        self.assertEqual(len(eeg_bytes), 240)

    def test_short_payloads(self):
        self.assertEqual(pf.split_payload(b""), (b"", None, 0))
        # 4 bytes alone are not a PPG trailer: there is no EEG sample in front of them
        self.assertEqual(pf.split_payload(b"\x00\x01\x02\x03"), (b"", None, 4))


class PpgDecodingTest(unittest.TestCase):

    def test_raw_codes_are_signed_big_endian(self):
        self.assertEqual(pf.decode_ppg_raw(bytes([0x7F, 0xFF, 0x00, 0x00])), [32767, 0])
        self.assertEqual(pf.decode_ppg_raw(bytes([0xFF, 0xFF, 0x80, 0x01])), [-1, -32767])

    def test_conversion_to_volts(self):
        volts = pf.decode_ppg_volts(bytes([0x04, 0xD2, 0x7F, 0xFF]))
        self.assertAlmostEqual(volts[0], 1234 * 125e-6, places=9)
        self.assertAlmostEqual(volts[1], 4.096 - 125e-6, places=9)
        self.assertAlmostEqual(pf.convert_ads1118_to_volts(-1), -125e-6, places=9)

    def test_invalid_marker_becomes_nan(self):
        volts = pf.decode_ppg_volts(bytes([0x80, 0x00, 0x00, 0x08]))
        self.assertTrue(math.isnan(volts[0]))
        self.assertAlmostEqual(volts[1], 8 * 125e-6, places=9)

    def test_rate_matches_firmware(self):
        self.assertEqual(pf.PPG_SAMPLING_RATE, 250 // (pf.EEG_SAMPLES_PER_PACKET // pf.PPG_SAMPLES_PER_PACKET))
        self.assertEqual(pf.PPG_TRAILER_SIZE, 4)


if __name__ == "__main__":
    unittest.main()
