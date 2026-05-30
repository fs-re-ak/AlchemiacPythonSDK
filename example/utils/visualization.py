from multiprocessing import Queue, Event
from queue import Empty
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from scipy.signal import butter, lfilter, lfilter_zi


def butter_bandpass(lowcut, highcut, fs, order=4):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    return butter(order, [low, high], btype='band')


def butter_bandstop(lowcut, highcut, fs, order=4):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    return butter(order, [low, high], btype='bandstop')


def visualizer(queue: Queue, shutdown_event: Event, nbChannels=8, samplingRate=250, 
               apply_filter=True, lowcut=15, highcut=30.0, order=2,
               apply_bandstop=False, bandstop_low=45.0, bandstop_high=65.0, bandstop_order=6):
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

    apply_bandstop : bool
        If True, applies an aggressive band-stop filter before the bandpass (default: False).

    bandstop_low, bandstop_high : float
        Band-stop cutoff frequencies (Hz).

    bandstop_order : int
        Band-stop filter order (higher = sharper cutoff).
    """

    app = QtWidgets.QApplication([])
    win = pg.GraphicsLayoutWidget(title="Real-time EEG")
    win.resize(800, 400)
    win.show()

    plot = win.addPlot(title="EEG Channels")
    plot.showGrid(x=True, y=True)
    plot.setLabel('left', 'Amplitude (uV)')
    plot.setLabel('bottom', 'Time (samples)')
    
    curves = [plot.plot(pen=pg.intColor(i)) for i in range(nbChannels)]

    buffer_size = samplingRate * 5  # 5 seconds
    data_buffers = [np.zeros(buffer_size) for _ in range(nbChannels)]

    channel_offsets = [100 * i for i in range(nbChannels)]

    # --- Initialize filters if enabled ---
    if apply_filter:
        b, a = butter_bandpass(lowcut, highcut, fs=samplingRate, order=order)
        filter_states = [lfilter_zi(b, a) * 0 for _ in range(nbChannels)]
    else:
        b, a, filter_states = None, None, None

    if apply_bandstop:
        b_stop, a_stop = butter_bandstop(
            bandstop_low, bandstop_high, fs=samplingRate, order=bandstop_order
        )
        bandstop_states = [lfilter_zi(b_stop, a_stop) * 0 for _ in range(nbChannels)]
    else:
        b_stop, a_stop, bandstop_states = None, None, None

    def update():
        nonlocal filter_states, bandstop_states
        while True:
            try:
                samples = queue.get_nowait()
            except Empty:
                break

            # Shutdown sentinel
            if samples is None:
                app.quit()
                return

            # Guard: must be a 2-D float array with the right number of columns
            if not isinstance(samples, np.ndarray) or samples.ndim != 2 or samples.shape[1] != nbChannels:
                print(f"[VISUALIZER] Unexpected sample shape {getattr(samples, 'shape', type(samples))}, skipping.")
                continue

            for i in range(samples.shape[0]):
                sample = samples[i, :]

                if apply_filter or apply_bandstop:
                    filtered_sample = np.zeros(nbChannels)
                    for j in range(nbChannels):
                        x = float(sample[j])
                        if apply_bandstop:
                            y_stop, bandstop_states[j] = lfilter(
                                b_stop, a_stop, [x], zi=bandstop_states[j]
                            )
                            x = float(y_stop[0])
                        if apply_filter:
                            y, filter_states[j] = lfilter(
                                b, a, [x], zi=filter_states[j]
                            )
                            filtered_sample[j] = float(y[0])
                        else:
                            filtered_sample[j] = x
                else:
                    filtered_sample = sample.astype(float)

                # Update buffers and curves
                for j in range(nbChannels):
                    data_buffers[j] = np.roll(data_buffers[j], -1)
                    data_buffers[j][-1] = filtered_sample[j]
                    curves[j].setData(data_buffers[j] + channel_offsets[j])

        if shutdown_event.is_set():
            print("[VISUALIZER] Shutdown signal received. Closing app...")
            app.quit()

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(30)

    app.exec_()
