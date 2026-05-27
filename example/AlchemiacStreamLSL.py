
import signal
from time import sleep
from multiprocessing import freeze_support

from pylsl import StreamInfo, StreamOutlet

from utils.ble_discovery import scan_and_select
from proxies.AlchemiacProxy import AlchemiacProxy


freeze_support()

# ─── LSL stream configuration ────────────────────────────────────────────────
EEG_CHANNELS = ["AF8", "AF7", "CHEEK_R", "CHEEK_L", "EAR_R", "AFz", "BROW_L", "NOSE"]
EEG_SAMPLING_RATE = 250

MOTION_CHANNELS = ["ax(g)", "ay(g)", "az(g)", "gx(deg/s)", "gy(deg/s)", "gz(deg/s)", "cx(G)", "cy(G)", "cz(G)"]
MOTION_SAMPLING_RATE = 100


# ─── Graceful shutdown ───────────────────────────────────────────────────────
shutdown_flag = False


def _signal_handler(sig, frame):
    global shutdown_flag
    print("\n[INFO] Ctrl+C received. Initiating shutdown...")
    shutdown_flag = True


signal.signal(signal.SIGINT, _signal_handler)


# ─── Entry point ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import multiprocessing
    multiprocessing.set_start_method("spawn", force=True)

    # 1. Discover and select the device
    device_address = scan_and_select()

    # 2. Set up LSL outlets
    lsl_eeg_info = StreamInfo(
        'Reak_EEG', 'EEG', len(EEG_CHANNELS), EEG_SAMPLING_RATE, 'float32', 'reak_eeg_001'
    )
    for label in EEG_CHANNELS:
        lsl_eeg_info.desc().append_child("channels").append_child("channel").append_child_value("label", label)
    lsl_eeg_outlet = StreamOutlet(lsl_eeg_info)

    lsl_motion_info = StreamInfo(
        'Reak_Motion', 'Motion', len(MOTION_CHANNELS), MOTION_SAMPLING_RATE, 'float32', 'reak_motion_001'
    )
    for label in MOTION_CHANNELS:
        lsl_motion_info.desc().append_child("channels").append_child("channel").append_child_value("label", label)
    lsl_motion_outlet = StreamOutlet(lsl_motion_info)

    print("[INFO] LSL streams initialized: Reak_EEG + Reak_Motion")

    # 3. Callbacks — forward decoded data directly to LSL outlets
    def eeg_callback(samples):
        for sample in samples:
            lsl_eeg_outlet.push_sample(sample)

    def motion_callback(sample):
        lsl_motion_outlet.push_sample(list(sample))

    proxy = None
    try:
        # 4. Connect
        proxy = AlchemiacProxy(device_address, eeg_callback=eeg_callback, motion_callback=motion_callback)
        proxy.waitForConnected()

        print("Stabilising signal…")
        sleep(1)

        print("[INFO] Streaming EEG + Motion via LSL. Press Ctrl+C to stop.")

        # 5. Run indefinitely until Ctrl+C
        while not shutdown_flag:
            sleep(0.1)

    except KeyboardInterrupt:
        print("[MAIN] KeyboardInterrupt caught.")

    finally:
        print("[MAIN] Cleaning up…")
        if proxy is not None:
            proxy.disconnect()
        print("[MAIN] Shutdown complete. Goodbye!")
