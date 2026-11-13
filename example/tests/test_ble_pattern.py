"""
BLE transfer test pattern (firmware built with BLE_TEST_PATTERN=1): the notification bytes
are built here exactly as p2p_server_app.c sends them, go through AlchemiacProxy
(packet_handler / motion_handler -> workers -> callbacks) and are checked by
utils/ble_test_pattern.py. The checkers must accept a correct stream (wrap-arounds, losses
signalled by the packet index) and catch every decoding error.

Cross-check with the real firmware output (optional): build the dumps with the firmware host
tests (Alchmiac-fw/tests/host, BLE_PATTERN_EEG_DUMP / BLE_PATTERN_IMU_DUMP), then set the
same variables when running these tests.
"""
import asyncio
import contextlib
import io
import os
import queue
import unittest

from proxies.AlchemiacProxy import AlchemiacProxy
from tests.test_proxy import run
from utils import ble_test_pattern as btp

EEG_PACKET_LEN = 245
IMU_PACKET_LEN = 18


# ---- What the firmware sends ----------------------------------------------------------

def eeg_packet(index, n0, eeg_order='big', ppg_order='big'):
    """[index][10 samples x 8 channels x 3 bytes][2 x int16]: sample s carries n0 + s."""
    out = bytearray([index & 0x7F])
    for s in range(10):
        n = (n0 + s) & 0xFFFF
        for c in range(8):
            channel = bytes([0xFF if c == 7 else c, n >> 8, n & 0xFF])
            out += channel if eeg_order == 'big' else channel[::-1]
    for s in (4, 9):
        out += ((n0 + s) & 0xFFFF).to_bytes(2, byteorder=ppg_order)
    return bytes(out)


def eeg_stream(nb_packets, n0=btp.PATTERN_START):
    return [eeg_packet(p, n0 + 10 * p) for p in range(nb_packets)]


def imu_packet(m, order='little'):
    return b''.join(((m + i) & 0xFFFF).to_bytes(2, byteorder=order) for i in range(9))


# ---- Through the SDK ------------------------------------------------------------------

def check_eeg(packets):
    eeg_calls, ppg_calls, out = run(packets)
    checker = btp.EegPatternChecker()
    for k, samples in enumerate(eeg_calls):
        checker.check_packet(samples, ppg_calls[k] if k < len(ppg_calls) else None)
    return checker, out


def check_imu(packets):
    proxy = object.__new__(AlchemiacProxy)
    proxy.motion_queue = queue.Queue()
    for pkt in packets:
        asyncio.run(proxy.motion_handler(None, bytearray(pkt)))
    proxy.motion_queue.put(None)
    calls = []
    AlchemiacProxy.motion_process(proxy.motion_queue, calls.append)
    checker = btp.ImuPatternChecker()
    for sample in calls:
        checker.check_packet(sample)
    return checker


def chunks(data, size):
    return [data[i:i + size] for i in range(0, len(data) - size + 1, size)]


class EegPatternTest(unittest.TestCase):

    def test_packet_layout(self):
        pkt = eeg_packet(3, 0x1234)
        self.assertEqual(len(pkt), EEG_PACKET_LEN)
        self.assertEqual(pkt[:7], bytes([3, 0x00, 0x12, 0x34, 0x01, 0x12, 0x34]))
        self.assertEqual(pkt[22:25], bytes([0xFF, 0x12, 0x34]))     # channel 7
        self.assertEqual(pkt[-4:], bytes([0x12, 0x38, 0x12, 0x3D]))  # n0 + 4, n0 + 9

    def test_correct_stream_with_wraps(self):
        """300 packets: counter wrap at sample 256, packet index wraps at 128 and 256."""
        checker, _ = check_eeg(eeg_stream(300))
        self.assertEqual(checker.error_count, 0, checker.errors)
        self.assertEqual(checker.packets_ok, 300)
        self.assertEqual(checker.samples_checked, 3000)
        self.assertEqual(checker.ppg_checked, 600)
        self.assertEqual(checker.wraps, 1)
        self.assertEqual(checker.packets_lost, 0)

    def test_ppg_sign_change(self):
        """+32767 -> -32768; 0x8000 (-32768) is also the invalid marker, decoded as NaN."""
        checker, _ = check_eeg(eeg_stream(20, n0=0x7FC0))
        self.assertEqual(checker.error_count, 0, checker.errors)
        self.assertEqual(checker.ppg_sign_changes, 1)

    def test_losses_signalled_by_the_index(self):
        # The proxy holds the packets behind a loss until 10 more have arrived: keep some after.
        packets = eeg_stream(40)
        del packets[20:23]
        del packets[5]
        checker, out = check_eeg(packets)
        self.assertIn("Dropped packet 5", out)
        self.assertEqual(checker.error_count, 0, checker.errors)
        self.assertEqual(checker.packets_lost, 4)
        self.assertEqual(checker.packets_ok, 36)

    def test_loss_hidden_by_the_index_is_caught(self):
        """128 packets lost: the 7-bit index looks continuous, the 16-bit counter does not."""
        packets = eeg_stream(300)
        del packets[10:138]
        checker, out = check_eeg(packets)
        self.assertNotIn("Dropped", out)
        self.assertEqual(checker.error_count, 1, checker.errors)
        self.assertIn("saut de 1280", checker.errors[0])

    def test_swapped_packets_are_caught(self):
        """The late packet makes the proxy insert 126 NaN packets, then the counter jumps back."""
        packets = eeg_stream(200)
        packets[7], packets[8] = packets[8], packets[7]
        checker, out = check_eeg(packets)
        self.assertIn("Dropped packet 127", out)
        self.assertGreater(checker.error_count, 0)
        self.assertIn("saut de", checker.errors[0])

    def test_duplicate_packet_is_caught(self):
        """A packet received twice: the proxy takes it for 127 lost packets."""
        packets = eeg_stream(200)
        packets.insert(8, packets[7])
        checker, out = check_eeg(packets)
        self.assertGreater(checker.error_count, 0)

    def test_little_endian_eeg_is_caught(self):
        packets = eeg_stream(5)
        packets[2] = eeg_packet(2, btp.PATTERN_START + 20, eeg_order='little')
        checker, _ = check_eeg(packets)
        self.assertGreater(checker.error_count, 0)
        self.assertIn("ordre des octets inversé", checker.errors[0])

    def test_little_endian_ppg_is_caught(self):
        packets = eeg_stream(5)
        packets[1] = eeg_packet(1, btp.PATTERN_START + 10, ppg_order='little')
        checker, _ = check_eeg(packets)
        self.assertEqual(checker.error_count, 2, checker.errors)
        self.assertIn("PPG0", checker.errors[0])
        self.assertIn("ordre des octets inversé", checker.errors[0])

    def test_missing_sign_extension_is_named(self):
        n = 0x1234
        raw = btp.expected_eeg_raw(n)
        raw[7] &= 0xFFFFFF                      # decoded as unsigned
        samples = [[v * btp.EEG_LSB_UV for v in btp.expected_eeg_raw((n + s) & 0xFFFF)] for s in range(10)]
        samples[0] = [v * btp.EEG_LSB_UV for v in raw]
        checker = btp.EegPatternChecker()
        checker.check_packet(samples)
        self.assertEqual(checker.error_count, 1, checker.errors)
        self.assertIn("extension du signe manquante", checker.errors[0])

    def test_real_data_is_rejected(self):
        from tests.packets import build_packet, eeg_raw_block
        packets = [build_packet(i, eeg_raw_block(i), [1, 2]) for i in range(3)]
        checker, _ = check_eeg(packets)
        text, passed = btp.report(checker, btp.ImuPatternChecker())
        self.assertFalse(passed)
        self.assertIn("BLE_TEST_PATTERN=1", text)


class ImuPatternTest(unittest.TestCase):

    def test_correct_stream_with_wrap(self):
        checker = check_imu([imu_packet(btp.PATTERN_START + k) for k in range(300)])
        self.assertEqual(checker.error_count, 0, checker.errors)
        self.assertEqual(checker.packets_ok, 300)
        self.assertEqual(checker.wraps, 1)

    def test_lost_packets_are_counted(self):
        packets = [imu_packet(btp.PATTERN_START + k) for k in range(50)]
        del packets[10:13]
        checker = check_imu(packets)
        self.assertEqual(checker.error_count, 0, checker.errors)
        self.assertEqual(checker.packets_lost, 3)

    def test_big_endian_imu_is_caught(self):
        packets = [imu_packet(0x1234 + k) for k in range(5)]
        packets[2] = imu_packet(0x1236, order='big')
        checker = check_imu(packets)
        self.assertGreater(checker.error_count, 0)
        self.assertIn("ordre des octets inversé", checker.errors[0])


class ReportTest(unittest.TestCase):

    def test_passed(self):
        eeg, _ = check_eeg(eeg_stream(40))
        imu = check_imu([imu_packet(btp.PATTERN_START + k) for k in range(10)])
        text, passed = btp.report(eeg, imu)
        self.assertTrue(passed, text)
        self.assertIn("RÉUSSI", text)

    def test_capture_without_wrap_fails(self):
        eeg, _ = check_eeg(eeg_stream(10, n0=0x1000))
        imu = check_imu([imu_packet(0x1000 + k) for k in range(10)])
        text, passed = btp.report(eeg, imu)
        self.assertFalse(passed)
        self.assertIn("jamais revenu à 0", text)


class SessionRoutingTest(unittest.TestCase):
    """ble_pattern_check.PatternSession pairs each EEG packet with its PPG values."""

    def test_eeg_and_ppg_callbacks_are_paired(self):
        import ble_pattern_check
        eeg_calls, ppg_calls, _ = run(eeg_stream(30))
        session = ble_pattern_check.PatternSession()
        for samples, ppg in zip(eeg_calls, ppg_calls):
            session.eeg_callback(samples)
            session.ppg_callback(ppg)
        session.finish()
        self.assertEqual(session.eeg.error_count, 0, session.eeg.errors)
        self.assertEqual(session.eeg.packets_with_ppg, 30)


@unittest.skipUnless(os.environ.get("BLE_PATTERN_EEG_DUMP") or os.environ.get("BLE_PATTERN_IMU_DUMP"),
                     "BLE_PATTERN_EEG_DUMP / BLE_PATTERN_IMU_DUMP non définis")
class FirmwareDumpTest(unittest.TestCase):
    """Packets written by the real firmware code (Alchmiac-fw host tests)."""

    def test_eeg_dump(self):
        path = os.environ.get("BLE_PATTERN_EEG_DUMP")
        if not path:
            self.skipTest("BLE_PATTERN_EEG_DUMP non défini")
        with open(path, "rb") as f:
            packets = chunks(f.read(), EEG_PACKET_LEN)
        self.assertGreater(len(packets), 26)
        self.assertEqual(packets[:3], eeg_stream(3), "le firmware et ce test doivent produire les mêmes octets")
        checker, _ = check_eeg(packets)
        self.assertEqual(checker.error_count, 0, checker.errors)
        self.assertEqual(checker.wraps, 1)

    def test_imu_dump(self):
        path = os.environ.get("BLE_PATTERN_IMU_DUMP")
        if not path:
            self.skipTest("BLE_PATTERN_IMU_DUMP non défini")
        with open(path, "rb") as f:
            packets = chunks(f.read(), IMU_PACKET_LEN)
        self.assertEqual(packets[0], imu_packet(btp.PATTERN_START))
        checker = check_imu(packets)
        self.assertEqual(checker.error_count, 0, checker.errors)
        self.assertEqual(checker.wraps, 1)


if __name__ == "__main__":
    unittest.main()
