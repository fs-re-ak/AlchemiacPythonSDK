"""
Runs the real-time visualizer off-screen and checks that the PPG plot receives the samples,
and that the former call (without PPG) still works.
"""
import math
import multiprocessing
import os
import queue
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import numpy as np
    import pyqtgraph as pg
    from utils.visualization import visualizer
    GUI_AVAILABLE = True
except ImportError:  # PyQt5 / pyqtgraph not installed
    GUI_AVAILABLE = False


def _run_legacy_visualizer(eeg_queue, shutdown_event):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from utils.visualization import visualizer as vis
    vis(eeg_queue, shutdown_event)


@unittest.skipUnless(GUI_AVAILABLE, "PyQt5/pyqtgraph not installed")
class VisualizerTest(unittest.TestCase):

    def test_ppg_plot_receives_samples(self):
        eeg_q, ppg_q = queue.Queue(), queue.Queue()
        eeg_q.put(np.zeros((10, 8)))
        eeg_q.put(None)                       # sentinel: closes the window on the first refresh
        for values in ([1.0, 1.1], [math.nan, 1.3], [1.4, 1.5]):
            ppg_q.put(values)

        recorded = []
        original = pg.PlotDataItem.setData

        def spy(item, *args, **kwargs):
            if kwargs.get("connect") == "finite":
                recorded.append(np.array(args[0], dtype=float))
            return original(item, *args, **kwargs)

        pg.PlotDataItem.setData = spy
        try:
            visualizer(eeg_q, multiprocessing.Event(), ppg_queue=ppg_q)
        finally:
            pg.PlotDataItem.setData = original

        self.assertEqual(len(recorded), 1, "the PPG curve must be updated once")
        data = recorded[0]
        self.assertEqual(data.size, 500, "10 s window at 50 Hz")
        tail = data[-6:]
        np.testing.assert_allclose(tail[[0, 1, 3, 4, 5]], [1.0, 1.1, 1.3, 1.4, 1.5])
        self.assertTrue(np.isnan(tail[2]))
        self.assertTrue(np.isnan(data[:-6]).all(), "the rest of the window stays empty")

    def test_legacy_call_without_ppg(self):
        ctx = multiprocessing.get_context("spawn")
        eeg_q, shutdown = ctx.Queue(), ctx.Event()
        eeg_q.put(np.ones((10, 8)))
        eeg_q.put(None)
        proc = ctx.Process(target=_run_legacy_visualizer, args=(eeg_q, shutdown))
        proc.start()
        proc.join(timeout=60)
        if proc.is_alive():
            proc.terminate()
            self.fail("the visualizer did not close")
        self.assertEqual(proc.exitcode, 0)


if __name__ == "__main__":
    unittest.main()
