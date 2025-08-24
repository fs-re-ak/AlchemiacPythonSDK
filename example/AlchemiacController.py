
import sys
import csv
import atexit
import asyncio
import numpy as np
from time import time, sleep
import signal
from datetime import datetime
from multiprocessing import Process, Queue, Event, Manager, freeze_support


from utils.visualization import visualizer
from proxies.AlchemiacProxy import AlchemiacProxy


freeze_support()

now = datetime.now()
timestamp_str = now.strftime("%Y-%m-%d_%H-%M-%S")
eeg_filename = f"alchemiac_eeg_{timestamp_str}.txt"
motion_filename = f"alchemiac_motion_{timestamp_str}.txt"

eeg_file = open(f"data/{eeg_filename}.csv", "w", newline="")
eeg_writer = csv.writer(eeg_file)
eeg_writer.writerow(["AF8","AF7","CHEEK_R","CHEEK_L","EAR_R","AFz","BROW_L","NOSE"])

motion_file = open(f"data/{motion_filename}.csv", mode="w", newline="")
motion_writer = csv.writer(motion_file)
motion_writer.writerow(["x(g)","y(g)","z(g)","x(deg/s)","y(deg/s)","z(deg/s)","x(G)","y(G)","z(G)"])


def close_file():
    eeg_file.close()
    motion_file.close()

atexit.register(close_file)

 
def motion_callback(sample):
    
    global motion_writer
    
    ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, cx_raw, cy_raw, cz_raw = sample
    
    motion_writer.writerow([
        f"{ax_raw:.5f}", f"{ay_raw:.5f}", f"{az_raw:.5f}",
        f"{gx_raw:.5f}", f"{gy_raw:.5f}", f"{gz_raw:.5f}",
        f"{cx_raw:.5f}", f"{cy_raw:.5f}", f"{cz_raw:.5f}",
    ])
    
    pass
          
    
# ------------------------------
# Graceful Shutdown Handling
# ------------------------------

# Event object to coordinate shutdown across processes
shutdown_event = Event()

def signal_handler(sig, frame):
    """
    Handles Ctrl+C (SIGINT) to trigger a clean shutdown.
    """
    print("\n[INFO] Ctrl+C received. Initiating shutdown...")
    shutdown_event.set()

# Register signal handler
signal.signal(signal.SIGINT, signal_handler)


q = None
    
def eeg_callback(samples):
    global q
    
    """
    Called when new EEG samples are received from the Muse.
    Pushes the data to the visualization queue for plotting.
    """
    
    #global eeg_writer
    if q is not None:
        q.put(np.array(samples))  # send to visualizer
    
    eeg_writer.writerows(samples)
    pass

if __name__ == "__main__":
    import sys
    import multiprocessing
    multiprocessing.set_start_method("spawn", force=True)  # for Windows compatibility

    q = Queue()  # Not Manager.Queue!
    
    
    # Start visualization in a separate process
    vis_process = Process(target=visualizer, args=(q, shutdown_event))
    vis_process.start()

    try:
        # Initialize Muse connection
        proxy = AlchemiacProxy(eeg_callback=eeg_callback, motion_callback=motion_callback)
        proxy.waitForConnected()
        

        # Optional buffer period to stabilize signal
        sleep(1)
        print("Initial padding, to stabilize signals...")
        sleep(10)

        # Start of live demo sequence
        print("Starting experience")
        #proxy.set_async_config()

        # Phase 1: Just let the EEG stream for 6 minutes (adjust to your demo needs)
        sleep(240)

    except KeyboardInterrupt:
        print("[MAIN] KeyboardInterrupt caught in try-block.")

    finally:
        # Ensure clean shutdown
        print("[MAIN] Cleaning up...")
        shutdown_event.set()        # Signal visualizer to stop
        proxy.disconnect()          # Disconnect Muse
        q.put(None)
        vis_process.terminate()     # Force-stop the visualizer (in case it lags)
        vis_process.join()          # Wait for visualizer to exit
        print("[MAIN] Shutdown complete. Goodbye!")