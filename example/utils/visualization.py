from multiprocessing import Queue, Event
from queue import Empty
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
from scipy.signal import butter, lfilter, lfilter_zi


def butter_bandpass(lowcut, highcut, fs, order=4):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    return butter(order, [low, high], btype='band')


def visualizer(queue: Queue, shutdown_event: Event, nbChannels=8, samplingRate=250,
               apply_filter=True, lowcut=15, highcut=30.0, order=2,
               ppg_queue: Queue = None, ppgSamplingRate=50):
    """
    Real-time EEG plotter using PyQtGraph with optional bandpass filtering.

    Parameters
    ----------
    queue : multiprocessing.Queue
        Queue through which raw EEG samples (shape: [samples, nbChannels]) are received.

    shutdown_event : multiprocessing.Event
        Event used to signal that the visualizer should terminate cleanly.

    nbChannels : int
        Number of EEG channels.

    samplingRate : float
        Sampling rate of EEG data (Hz).

    apply_filter : bool
        If True, applies a bandpass filter (default: True).

    lowcut, highcut : float
        Bandpass filter cutoff frequencies (Hz).

    order : int
        Filter order.

    ppg_queue : multiprocessing.Queue, optional
        Queue through which PPG samples (list of floats, volts) are received.
        When given, a second plot shows the raw PPG signal (10 s window).

    ppgSamplingRate : float
        Sampling rate of the PPG data (Hz).

    Display controls
    ----------------
    - "Pause" button or Space key: freezes the display. Data keep being received and
      filtered in the background; resuming jumps back to the live signal.
    - Mouse wheel / right-drag: zooms the time axis only; left-drag pans it.
      The vertical scale is never changed by the mouse, so amplitudes stay readable.
    - "Réinitialiser le zoom" button: shows the whole window again.
    """

    app = QtWidgets.QApplication([])
    window = QtWidgets.QWidget()
    window.setWindowTitle("Real-time EEG")
    layout = QtWidgets.QVBoxLayout(window)

    toolbar = QtWidgets.QHBoxLayout()
    pause_button = QtWidgets.QPushButton("Pause (Espace)")
    pause_button.setCheckable(True)
    reset_zoom_button = QtWidgets.QPushButton("Réinitialiser le zoom")
    for button in (pause_button, reset_zoom_button):
        button.setFocusPolicy(QtCore.Qt.NoFocus)  # Space must toggle the pause, not "click" a button
        toolbar.addWidget(button)
    toolbar.addWidget(QtWidgets.QLabel("Molette : zoom horizontal · Glisser : déplacer"))
    toolbar.addStretch(1)
    layout.addLayout(toolbar)

    win = pg.GraphicsLayoutWidget()
    layout.addWidget(win)
    window.resize(800, 440)
    window.show()

    plot = win.addPlot(title="EEG Channels")
    plot.showGrid(x=True, y=True)
    plot.setLabel('left', 'Amplitude (uV)')
    plot.setLabel('bottom', 'Time (samples)')
    plot.setMouseEnabled(x=True, y=False)  # horizontal zoom only, Y keeps auto-range
    plots = [plot]

    curves = [plot.plot(pen=pg.intColor(i)) for i in range(nbChannels)]

    buffer_size = samplingRate * 5  # 5 seconds
    data_buffers = [np.zeros(buffer_size) for _ in range(nbChannels)]

    channel_offsets = [100 * i for i in range(nbChannels)]

    # --- Optional PPG plot (raw volts, no filtering) ---
    ppg_curve = None
    if ppg_queue is not None:
        window.resize(800, 690)
        win.nextRow()
        ppg_plot = win.addPlot(title="PPG (ADS1118 AIN0)")
        ppg_plot.showGrid(x=True, y=True)
        ppg_plot.setLabel('left', 'Tension (V)')
        ppg_plot.setLabel('bottom', 'Time (samples)')
        ppg_plot.setMouseEnabled(x=True, y=False)
        plots.append(ppg_plot)
        ppg_curve = ppg_plot.plot(pen=pg.mkPen('r', width=2))
        ppg_buffer = np.full(int(ppgSamplingRate * 10), np.nan)  # 10 seconds

    # --- Display state: buffers are always updated, curves only when not paused ---
    paused = False
    eeg_dirty = False
    ppg_dirty = False

    def refresh_display():
        nonlocal eeg_dirty, ppg_dirty
        if eeg_dirty:
            for j in range(nbChannels):
                curves[j].setData(data_buffers[j] + channel_offsets[j])
            eeg_dirty = False
        if ppg_dirty:
            ppg_curve.setData(ppg_buffer, connect='finite')
            ppg_dirty = False

    def set_paused(checked):
        nonlocal paused
        paused = checked
        pause_button.setText("Reprendre (Espace)" if paused else "Pause (Espace)")
        plot.setTitle("EEG Channels (pause)" if paused else "EEG Channels")
        if not paused:
            refresh_display()

    def reset_zoom():
        for p in plots:
            p.enableAutoRange()

    pause_button.toggled.connect(set_paused)
    reset_zoom_button.clicked.connect(reset_zoom)
    pause_shortcut = QtWidgets.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Space), window)
    pause_shortcut.activated.connect(pause_button.toggle)

    def update_ppg():
        nonlocal ppg_buffer, ppg_dirty
        new_values = []
        while True:
            try:
                values = ppg_queue.get_nowait()
            except Empty:
                break
            if values is None:
                continue
            new_values.extend(float(v) for v in values)
        if new_values:
            n = min(len(new_values), ppg_buffer.size)
            ppg_buffer = np.roll(ppg_buffer, -n)
            ppg_buffer[-n:] = new_values[-n:]
            ppg_dirty = True

    # --- Initialize filter if enabled ---
    if apply_filter:
        b, a = butter_bandpass(lowcut, highcut, fs=samplingRate, order=order)
        filter_states = [lfilter_zi(b, a) * 0 for _ in range(nbChannels)]
    else:
        b, a, filter_states = None, None, None

    def update():
        nonlocal filter_states, eeg_dirty
        if ppg_curve is not None:
            update_ppg()
        quit_requested = False
        while True:
            try:
                samples = queue.get_nowait()
            except Empty:
                break

            # Shutdown sentinel
            if samples is None:
                quit_requested = True
                break

            # Guard: must be a 2-D float array with the right number of columns
            if not isinstance(samples, np.ndarray) or samples.ndim != 2 or samples.shape[1] != nbChannels:
                print(f"[VISUALIZER] Unexpected sample shape {getattr(samples, 'shape', type(samples))}, skipping.")
                continue

            for i in range(samples.shape[0]):
                sample = samples[i, :]

                if apply_filter:
                    filtered_sample = np.zeros(nbChannels)
                    for j in range(nbChannels):
                        # lfilter returns (y, zf).  y has shape (1,) — extract the
                        # scalar explicitly so this works across all NumPy versions.
                        y, filter_states[j] = lfilter(
                            b, a, [float(sample[j])], zi=filter_states[j]
                        )
                        filtered_sample[j] = float(y[0])
                else:
                    filtered_sample = sample.astype(float)

                # Update buffers (curves are redrawn once per refresh, below)
                for j in range(nbChannels):
                    data_buffers[j] = np.roll(data_buffers[j], -1)
                    data_buffers[j][-1] = filtered_sample[j]
                eeg_dirty = True

        if not paused:
            refresh_display()

        if quit_requested:
            app.quit()
            return

        if shutdown_event.is_set():
            print("[VISUALIZER] Shutdown signal received. Closing app...")
            app.quit()

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(30)

    app.exec_()
