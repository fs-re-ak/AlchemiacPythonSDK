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


def _run_scripted_visualizer(result_q):
    """Drives the window like a user: pause, Space key, wheel zoom, reset (own process: one QApplication each)."""
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    import numpy as np
    import pyqtgraph as pg
    from utils.visualization import visualizer
    from PyQt5 import QtCore, QtWidgets
    from PyQt5.QtTest import QTest

    eeg_q, ppg_q = queue.Queue(), queue.Queue()
    eeg_updates = []                       # EEG curve redraws (ch. 0 offset is 0)
    original_set_data = pg.PlotDataItem.setData
    original_exec = QtWidgets.QApplication.exec_
    results = {}

    def spy(item, *args, **kwargs):
        if args and kwargs.get("connect") is None:
            eeg_updates.append(np.array(args[0], dtype=float))
        return original_set_data(item, *args, **kwargs)

    class Wheel:                           # minimal stand-in for a wheel event
        def __init__(self, pos):
            self._pos = pos
        def delta(self):
            return 120
        def pos(self):
            return self._pos
        def accept(self):
            pass

    def scripted_exec(app):
        window = next(w for w in app.topLevelWidgets()
                      if w.findChild(pg.GraphicsLayoutWidget) is not None)
        window.activateWindow()
        buttons = {b.text(): b for b in window.findChildren(QtWidgets.QPushButton)}
        pause, reset = buttons["Pause (Espace)"], buttons["Réinitialiser le zoom"]
        vb = window.findChild(pg.GraphicsLayoutWidget).getItem(0, 0).getViewBox()

        eeg_q.put(np.full((10, 8), 1.0))
        QTest.qWait(150)
        results["live"] = len(eeg_updates)

        pause.click()
        n_before = len(eeg_updates)
        eeg_q.put(np.full((10, 8), 2.0))
        QTest.qWait(150)
        results["while_paused"] = len(eeg_updates) - n_before
        results["paused_text"] = pause.text()

        pause.click()                      # resume: the data received while paused shows up
        results["resumed_tail"] = eeg_updates[-8][-1] if len(eeg_updates) >= 8 else None

        QTest.qWait(50)                    # let the lazy auto-range settle first
        x0, y0 = vb.viewRange()
        vb.wheelEvent(Wheel(QtCore.QPointF(vb.width() / 2, vb.height() / 2)))
        QTest.qWait(50)
        x1, y1 = vb.viewRange()
        results["x_span"] = (x0[1] - x0[0], x1[1] - x1[0])
        results["y"] = (y0, y1)
        results["auto"] = vb.autoRangeEnabled()

        reset.click()
        results["auto_after_reset"] = vb.autoRangeEnabled()

        QTest.keyClick(window, QtCore.Qt.Key_Space)
        results["space_pauses"] = pause.isChecked()
        return 0

    pg.PlotDataItem.setData = spy
    QtWidgets.QApplication.exec_ = scripted_exec
    try:
        visualizer(eeg_q, multiprocessing.Event(), apply_filter=False, ppg_queue=ppg_q)
    finally:
        pg.PlotDataItem.setData = original_set_data
        QtWidgets.QApplication.exec_ = original_exec
    result_q.put({k: (np.asarray(v).tolist() if not isinstance(v, (str, int, float, bool, type(None))) else v)
                  for k, v in results.items()})


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

    def test_pause_and_horizontal_zoom(self):
        ctx = multiprocessing.get_context("spawn")
        result_q = ctx.Queue()
        proc = ctx.Process(target=_run_scripted_visualizer, args=(result_q,))
        proc.start()
        try:
            results = result_q.get(timeout=60)
        finally:
            proc.join(timeout=30)
            if proc.is_alive():
                proc.terminate()

        self.assertGreater(results["live"], 0, "curves are drawn while running")
        self.assertEqual(results["while_paused"], 0, "no redraw while paused")
        self.assertEqual(results["paused_text"], "Reprendre (Espace)")
        self.assertEqual(results["resumed_tail"], 2.0, "resuming shows the latest data")
        self.assertLess(results["x_span"][1], results["x_span"][0], "the wheel zooms the time axis")
        np.testing.assert_allclose(results["y"][1], results["y"][0], err_msg="Y range unchanged")
        self.assertEqual(results["auto"][1], True, "Y stays in auto-range")
        self.assertEqual(results["auto_after_reset"][0], True, "reset restores the full time window")
        self.assertTrue(results["space_pauses"], "Space toggles the pause")

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
