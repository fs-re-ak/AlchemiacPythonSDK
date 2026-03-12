
import sys
import csv
import atexit
import asyncio
import platform
import numpy as np
from time import time, sleep
import signal
from datetime import datetime
from multiprocessing import Process, Queue, Event, freeze_support
from bleak import BleakScanner

from utils.visualization import visualizer
from proxies.AlchemiacProxy import AlchemiacProxy


freeze_support()

# ─── Platform ────────────────────────────────────────────────────────────────
# On macOS (Darwin), CoreBluetooth exposes UUIDs instead of MAC addresses.
# bleak.BleakScanner returns device.address in the correct format per platform,
# so no manual conversion is needed — just always use device.address.
OS = platform.system()          # "Windows" | "Darwin" | "Linux"
DEVICE_NAME = "Hermes V1"
SCAN_TIMEOUT = 8.0              # seconds

# ─── Data files ──────────────────────────────────────────────────────────────
_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
eeg_file    = open(f"data/alchemiac_eeg_{_timestamp}.csv",    "w", newline="")
motion_file = open(f"data/alchemiac_motion_{_timestamp}.csv", "w", newline="")

eeg_writer    = csv.writer(eeg_file)
motion_writer = csv.writer(motion_file)
eeg_writer.writerow(["AF8", "AF7", "CHEEK_R", "CHEEK_L", "EAR_R", "AFz", "BROW_L", "NOSE"])
motion_writer.writerow(["x(g)", "y(g)", "z(g)", "x(deg/s)", "y(deg/s)", "z(deg/s)", "x(G)", "y(G)", "z(G)"])


def _close_files():
    eeg_file.close()
    motion_file.close()

atexit.register(_close_files)


# ─── Callbacks ───────────────────────────────────────────────────────────────
q = None


def eeg_callback(samples):
    global q
    if q is not None:
        q.put(np.array(samples))
    eeg_writer.writerows(samples)


def motion_callback(sample):
    ax, ay, az, gx, gy, gz, cx, cy, cz = sample
    motion_writer.writerow([
        f"{ax:.5f}", f"{ay:.5f}", f"{az:.5f}",
        f"{gx:.5f}", f"{gy:.5f}", f"{gz:.5f}",
        f"{cx:.5f}", f"{cy:.5f}", f"{cz:.5f}",
    ])


# ─── Graceful shutdown ───────────────────────────────────────────────────────
shutdown_event = Event()


def _signal_handler(sig, frame):
    print("\n[INFO] Ctrl+C received. Initiating shutdown...")
    shutdown_event.set()


signal.signal(signal.SIGINT, _signal_handler)


# ─── BLE scanning & device selection ─────────────────────────────────────────
async def _scan(timeout: float = SCAN_TIMEOUT) -> list:
    """Return a list of BLE devices whose name contains DEVICE_NAME."""
    addr_kind = "UUID" if OS == "Darwin" else "MAC address"
    print(f"\n[SCAN] Scanning for '{DEVICE_NAME}' ({timeout:.0f} s)  "
          f"[{OS} — using {addr_kind}]\n")
    all_devices = await BleakScanner.discover(timeout=timeout)
    return [d for d in all_devices if d.name and DEVICE_NAME in d.name]


def _select_device(devices: list):
    """Print a numbered menu and return the chosen BleakDevice."""
    addr_label = "UUID" if OS == "Darwin" else "Address"
    width = 54
    print("─" * width)
    print(f"  Found {len(devices)} Hermes device(s):")
    print("─" * width)
    for i, dev in enumerate(devices):
        print(f"  [{i}]  {dev.name:<20}  {addr_label}: {dev.address}")
    print("─" * width)

    while True:
        try:
            raw = input(f"\n  Select device [0–{len(devices) - 1}]: ").strip()
            idx = int(raw)
            if 0 <= idx < len(devices):
                return devices[idx]
            print(f"  Enter a number between 0 and {len(devices) - 1}.")
        except ValueError:
            print("  Invalid input — enter a number.")


def scan_and_select() -> str:
    """
    Scan for Hermes devices, loop until at least one is found,
    let the user choose, and return the address ready for BleakClient.
    """
    while True:
        devices = asyncio.run(_scan())
        if devices:
            break
        print(f"[SCAN] No '{DEVICE_NAME}' devices found.")
        ans = input("  Press Enter to scan again, or 'q' to quit: ").strip().lower()
        if ans == "q":
            print("[INFO] Exiting.")
            sys.exit(0)

    chosen = _select_device(devices)
    print(f"\n[INFO] Selected: {chosen.name}  ({chosen.address})\n")
    return chosen.address


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
    vis_process = Process(target=visualizer, args=(q, shutdown_event))
    vis_process.start()

    proxy = None
    try:
        # 3. Connect
        proxy = AlchemiacProxy(device_address,
                               eeg_callback=eeg_callback,
                               motion_callback=motion_callback)
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
