"""
Real-Time EEG Visualizer for Muse Headset (Lesson 1)

Author: Fred Simard, RE-AK Technologies, 2025
Website: https://www.re-ak.com
Discord: https://discord.gg/XzeDHJf6ne

Description:
------------
This module implements a real-time EEG visualizer using PyQtGraph and multiprocessing.

It receives EEG samples from a queue and plots the four Muse channels (TP9, AF7, AF8, TP10)
with vertical offsets for clarity. It also applies a live bandpass filter to reduce noise.

This script is intended for educational/demo purposes and is part of Lesson 1 in the Muse
EEG YouTube tutorial series, demonstrating EEG artifacts such as eye blinks, jaw clenches,
and movement-induced noise.

Main Components:
----------------
- `visualizer(queue: Queue, shutdown_event: Event)`:
    Launches a PyQtGraph-based GUI window in a separate process. Handles real-time EEG data
    visualization and graceful shutdown triggered via multiprocessing event.
"""

import numpy as np
from queue import Empty 
from multiprocessing import Queue, Event
from utils.filters import apply_bandpass_filter

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtWidgets, QtCore


def visualizer(queue: Queue, shutdown_event: Event, nbChannels=8, samplingRate=250):
    """
    Real-time EEG plotter using PyQtGraph. Each of the four Muse EEG channels is displayed
    in a scrolling time window, with real-time bandpass filtering and vertical offsets 
    for easier visual separation.

    Parameters:
    -----------
    queue : multiprocessing.Queue
        Queue through which raw EEG samples (shape: [samples, 4]) are received.

    shutdown_event : multiprocessing.Event
        Event used to signal that the visualizer should terminate cleanly.
    """

    app = QtWidgets.QApplication([])
    win = pg.GraphicsLayoutWidget(title="Real-time EEG")
    win.resize(800, 400)
    win.show()

    plot = win.addPlot(title="EEG Channels (TP9, AF7, AF8, TP10)")
    plot.showGrid(x=True, y=True)
    plot.setLabel('left', 'Amplitude (uV)')
    plot.setLabel('bottom', 'Time (arbitrary units)')
    
    curves = [plot.plot(pen=pg.intColor(i)) for i in range(nbChannels)]

    buffer_size = samplingRate * 5  # 5 seconds of data at 256Hz
    data_buffers = [np.zeros(buffer_size) for _ in range(nbChannels)]

    channel_offsets = [100 * i for i in range(nbChannels)]  # Vertical offsets per channel

    def update():
        while True:
            try:
                samples = queue.get_nowait()
            except Empty:
                break

            for i in range(samples.shape[0]):
                sample = samples[i, :]

                # Apply bandpass filter to each channel using a rolling buffer
                
                if False:
                    filtered_sample = np.array([
                        apply_bandpass_filter(np.append(data_buffers[j][1:], sample[j]))
                        for j in range(nbChannels)
                    ])

                    # Keep only the latest filtered value
                    filtered_sample = np.array([ch[-1] for ch in filtered_sample])
                    
                filtered_sample = sample

                for j in range(nbChannels):
                    data_buffers[j] = np.roll(data_buffers[j], -1)
                    data_buffers[j][-1] = filtered_sample[j]
                    # Apply vertical offset to separate traces visually
                    curves[j].setData(data_buffers[j] - channel_offsets[j])

        if shutdown_event.is_set():
            print("[VISUALIZER] Shutdown signal received. Closing app...")
            app.quit()

    print("check B")
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(30)

    app.exec_()
    
