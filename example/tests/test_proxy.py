"""
End-to-end decoding through AlchemiacProxy, from the raw BLE notification bytes to the
callbacks: packet_handler (index, drop detection) -> queue -> worker_process (decoding).
No BLE connection is opened.
"""
import asyncio
import contextlib
import io
import math
import queue
import unittest
from unittest import mock

from proxies import AlchemiacProxy as proxy_module
from proxies.AlchemiacProxy import AlchemiacProxy
from tests.packets import build_packet, eeg_raw_block


def make_proxy():
    """AlchemiacProxy without its BLE/worker threads (state needed by packet_handler only)."""
    proxy = object.__new__(AlchemiacProxy)
    proxy.last_packet = None
    proxy.packets = []
    proxy.samples_per_packets = []
    proxy.packet_received = []
    proxy.packet_times = []
    proxy.eeg_queue = queue.Queue()
    return proxy


def run(packets, with_ppg_callback=True):
    """Feeds the notifications, then runs the worker until the queue is drained."""
    proxy = make_proxy()
    eeg_calls, ppg_calls = [], []
    stdout = io.StringIO()
    with contextlib.redirect_stdout(stdout):
        for pkt in packets:
            asyncio.run(proxy.packet_handler(None, None, bytearray(pkt)))
        proxy.eeg_queue.put(None)
        if with_ppg_callback:
            AlchemiacProxy.worker_process(proxy.eeg_queue, eeg_calls.append, ppg_calls.append)
        else:
            AlchemiacProxy.worker_process(proxy.eeg_queue, eeg_calls.append)  # former 2-argument call
    return eeg_calls, ppg_calls, stdout.getvalue()


def run_with_timestamps(packets, arrival_times):
    """Same as run(), with with_timestamps=True and a fake PC clock (one value per notification)."""
    proxy = make_proxy()
    eeg_calls, ppg_calls = [], []
    clock = iter(arrival_times)
    with contextlib.redirect_stdout(io.StringIO()),             mock.patch.object(proxy_module, "time", lambda: next(clock)):
        for pkt in packets:
            asyncio.run(proxy.packet_handler(None, None, bytearray(pkt)))
    proxy.eeg_queue.put(None)
    with contextlib.redirect_stdout(io.StringIO()):
        AlchemiacProxy.worker_process(proxy.eeg_queue,
                                      lambda s, t: eeg_calls.append((s, t)),
                                      lambda s, t: ppg_calls.append((s, t)),
                                      with_timestamps=True)
    return eeg_calls, ppg_calls


def expected_uv(eeg_raw):
    return [AlchemiacProxy.convert_ads1299_to_microvolts(sample) for sample in eeg_raw]


class NewFirmwareTest(unittest.TestCase):

    def test_eeg_and_ppg_are_decoded(self):
        blocks = [eeg_raw_block(i) for i in range(3)]
        ppg = [[1000, 1010], [1020, -32768], [32767, -5]]
        packets = [build_packet(i, blocks[i], ppg[i]) for i in range(3)]
        self.assertTrue(all(len(p) == 245 for p in packets))

        eeg_calls, ppg_calls, out = run(packets)

        self.assertEqual(eeg_calls, [expected_uv(b) for b in blocks])
        self.assertEqual(len(ppg_calls), 3)
        self.assertAlmostEqual(ppg_calls[0][0], 1000 * 125e-6)
        self.assertAlmostEqual(ppg_calls[0][1], 1010 * 125e-6)
        self.assertTrue(math.isnan(ppg_calls[1][1]))
        self.assertAlmostEqual(ppg_calls[2][1], -5 * 125e-6)
        self.assertNotIn("Warning", out, "a 245-byte packet must not raise the trailing-bytes warning")

    def test_dropped_packet_gives_nan_for_eeg_and_ppg(self):
        indices = [0, 1] + list(range(3, 13))   # packet 2 lost
        packets = [build_packet(i, eeg_raw_block(i), [i, i]) for i in indices]

        eeg_calls, ppg_calls, out = run(packets)

        self.assertIn("Dropped packet 2", out)
        self.assertEqual(len(eeg_calls), len(ppg_calls))
        self.assertTrue(all(math.isnan(v) for sample in eeg_calls[2] for v in sample))
        self.assertTrue(all(math.isnan(v) for v in ppg_calls[2]))
        self.assertAlmostEqual(ppg_calls[3][0], 3 * 125e-6)

    def test_without_ppg_callback(self):
        blocks = [eeg_raw_block(10 + i) for i in range(2)]
        packets = [build_packet(i, blocks[i], [1, 2]) for i in range(2)]

        eeg_calls, ppg_calls, out = run(packets, with_ppg_callback=False)

        self.assertEqual(eeg_calls, [expected_uv(b) for b in blocks])
        self.assertEqual(ppg_calls, [])
        self.assertNotIn("Warning", out)


class TimestampTest(unittest.TestCase):
    """with_timestamps=True: Unix time per sample on a regular 250 Hz grid."""

    T0 = 1763065432.5   # arrival of the first packet

    def test_eeg_and_ppg_on_a_250_hz_grid_across_a_drop(self):
        indices = [0, 1] + list(range(3, 13))   # packet 2 lost
        packets = [build_packet(i, eeg_raw_block(i), [i, i]) for i in indices]
        # Arrival times jitter: the grid must only depend on the first one
        arrivals = [self.T0 + 0.04 * k + (0.007 if k % 2 else -0.003) for k in range(len(indices))]
        arrivals[0] = self.T0

        eeg_calls, ppg_calls = run_with_timestamps(packets, arrivals)

        self.assertEqual(len(eeg_calls), 13)
        times = [t for _, ts in eeg_calls for t in ts]
        self.assertEqual(len(times), 130)
        self.assertAlmostEqual(eeg_calls[0][1][-1], self.T0, places=6,
                               msg="last sample of the first packet = its arrival time")
        for n, t in enumerate(times):
            self.assertAlmostEqual(t, self.T0 + (n - 9) / 250, places=6)
        self.assertTrue(all(math.isnan(v) for sample in eeg_calls[2][0] for v in sample),
                        "the dropped packet keeps its place on the grid, as NaN")

        self.assertEqual(len(ppg_calls), 13)
        for k, (values, ts) in enumerate(ppg_calls):
            self.assertEqual(len(ts), 2)
            self.assertAlmostEqual(ts[0], eeg_calls[k][1][4], places=6)
            self.assertAlmostEqual(ts[1], eeg_calls[k][1][9], places=6)
        self.assertAlmostEqual(ppg_calls[1][1][1] - ppg_calls[1][1][0], 1 / 50, places=6)

    def test_old_firmware_without_ppg(self):
        packets = [build_packet(i, eeg_raw_block(40 + i)) for i in range(2)]
        eeg_calls, ppg_calls = run_with_timestamps(packets, [self.T0, self.T0 + 0.05])
        self.assertEqual([s for s, _ in eeg_calls], [expected_uv(eeg_raw_block(40 + i)) for i in range(2)])
        self.assertAlmostEqual(eeg_calls[1][1][0], self.T0 + 1 / 250, places=6)
        self.assertEqual(ppg_calls, [])

    def test_motion_timestamp(self):
        q = queue.Queue()
        q.put((self.T0, 1000, 0, 0, 0, 0, 0, 0, 0, 0))
        q.put(None)
        calls = []
        AlchemiacProxy.motion_process(q, lambda s, t: calls.append((s, t)), with_timestamps=True)
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0][1], self.T0)
        self.assertAlmostEqual(calls[0][0][0], 1000 * 0.061 / 1000)

    def test_motion_without_timestamp(self):
        q = queue.Queue()
        q.put((self.T0, 1000, 0, 0, 0, 0, 0, 0, 0, 0))
        q.put(None)
        calls = []
        AlchemiacProxy.motion_process(q, calls.append)
        self.assertEqual(len(calls[0]), 9, "former call: values only")


class OldFirmwareTest(unittest.TestCase):
    """241-byte packets (firmware without PPG): behaviour unchanged."""

    def test_eeg_unchanged_and_no_ppg(self):
        blocks = [eeg_raw_block(20 + i) for i in range(3)]
        packets = [build_packet(i, blocks[i]) for i in range(3)]
        self.assertTrue(all(len(p) == 241 for p in packets))

        eeg_calls, ppg_calls, out = run(packets)

        self.assertEqual(eeg_calls, [expected_uv(b) for b in blocks])
        self.assertEqual(ppg_calls, [], "no PPG callback with a firmware that sends no PPG")
        self.assertNotIn("Warning", out)

    def test_dropped_packet_gives_nan_eeg_only(self):
        indices = [0, 1] + list(range(3, 13))
        packets = [build_packet(i, eeg_raw_block(i)) for i in indices]

        eeg_calls, ppg_calls, _ = run(packets)

        self.assertTrue(all(math.isnan(v) for sample in eeg_calls[2] for v in sample))
        self.assertEqual(ppg_calls, [])

    def test_malformed_packet_still_warns(self):
        packet = build_packet(0, eeg_raw_block(30)) + b"\x00\x00\x00"
        eeg_calls, ppg_calls, out = run([packet])
        self.assertIn("Dropping 3 trailing byte(s)", out)
        self.assertEqual(eeg_calls, [expected_uv(eeg_raw_block(30))])
        self.assertEqual(ppg_calls, [])


if __name__ == "__main__":
    unittest.main()
