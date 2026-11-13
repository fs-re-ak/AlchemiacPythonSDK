
import csv
import atexit
import numpy as np
from time import sleep
import signal
from datetime import datetime
from multiprocessing import Process, Queue, Event, freeze_support

from utils.visualization import visualizer
from utils.ble_discovery import scan_and_select
from proxies.AlchemiacProxy import AlchemiacProxy


freeze_support()

# ─── Data files ──────────────────────────────────────────────────────────────
_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
eeg_file    = open(f"data/alchemiac_eeg_{_timestamp}.csv",    "w", newline="")
motion_file = open(f"data/alchemiac_motion_{_timestamp}.csv", "w", newline="")
ppg_file    = open(f"data/alchemiac_ppg_{_timestamp}.csv",    "w", newline="")

eeg_writer    = csv.writer(eeg_file)
motion_writer = csv.writer(motion_file)
ppg_writer    = csv.writer(ppg_file)
eeg_writer.writerow(["AF8", "AF7", "CHEEK_R", "CHEEK_L", "EAR_R", "AFz", "BROW_L", "NOSE"])
motion_writer.writerow(["x(g)", "y(g)", "z(g)", "x(deg/s)", "y(deg/s)", "z(deg/s)", "x(G)", "y(G)", "z(G)"])
ppg_writer.writerow(["PPG(V)"])  # 50 Hz, empty if the firmware sends no PPG


def _close_files():
    eeg_file.close()
    motion_file.close()
    ppg_file.close()

atexit.register(_close_files)


# ─── Callbacks ───────────────────────────────────────────────────────────────
q = None
ppg_q = None


def eeg_callback(samples):
    global q
    if not samples:           # skip empty packet (e.g. 0-byte data after header)
        return
    arr = np.array(samples, dtype=np.float64)
    # Only forward well-formed 2-D arrays; log anything unexpected
    if arr.ndim != 2 or arr.shape[1] != 8:
        print(f"[EEG] Unexpected sample array shape {arr.shape}, skipping visualizer update.")
    elif q is not None:
        q.put(arr)
    eeg_writer.writerows(samples)


def motion_callback(sample):
    ax, ay, az, gx, gy, gz, cx, cy, cz = sample
    motion_writer.writerow([
        f"{ax:.5f}", f"{ay:.5f}", f"{az:.5f}",
        f"{gx:.5f}", f"{gy:.5f}", f"{gz:.5f}",
        f"{cx:.5f}", f"{cy:.5f}", f"{cz:.5f}",
    ])


def ppg_callback(samples):
    if ppg_q is not None:
        ppg_q.put(list(samples))
    ppg_writer.writerows([[f"{v:.6f}"] for v in samples])


# ─── Graceful shutdown ───────────────────────────────────────────────────────
shutdown_event = Event()


def _signal_handler(sig, frame):
    print("\n[INFO] Ctrl+C received. Initiating shutdown...")
    shutdown_event.set()


signal.signal(signal.SIGINT, _signal_handler)


# ─── Entry point ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import multiprocessing
    # "spawn" is already the default on macOS (Python ≥ 3.8) and Windows.
    # Forced here so behaviour is identical on all platforms.
    multiprocessing.set_start_method("spawn", force=True)

    # 1. Discover and select the device
    device_address = scan_and_select()

    # 2. Start the EEG visualizer in a separate process
    q = Queue()
    ppg_q = Queue()
    vis_process = Process(target=visualizer, args=(q, shutdown_event), kwargs={"ppg_queue": ppg_q})
    vis_process.start()

    proxy = None
    try:
        # 3. Connect
        proxy = AlchemiacProxy(device_address,
                               eeg_callback=eeg_callback,
                               motion_callback=motion_callback,
                               ppg_callback=ppg_callback)
        proxy.waitForConnected()

        # Brief stabilisation pause before the live demo
        print("Stabilising signal…")
        sleep(1)
        sleep(10)

        # 4. Stream for the demo duration
        print("Starting experience")
        sleep(240)

    except KeyboardInterrupt:
        print("[MAIN] KeyboardInterrupt caught.")

    finally:
        print("[MAIN] Cleaning up…")
        shutdown_event.set()
        if proxy is not None:
            proxy.disconnect()
        q.put(None)
        vis_process.terminate()
        vis_process.join()
        print("[MAIN] Shutdown complete. Goodbye!")
