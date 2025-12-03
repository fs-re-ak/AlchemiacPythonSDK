from pylsl import StreamInfo, StreamOutlet
from time import sleep
import signal

from proxies.AlchemiacProxy import AlchemiacProxy


# configuration
EEG_CHANNELS = ["AF8","AF7","CHEEK_R","CHEEK_L","EAR_R","AFz","BROW_L","NOSE"]
EEG_SAMPLING_RATE = 256       

MOTION_CHANNELS = ["ax(g)","ay(g)","az(g)","gx(deg/s)","gy(deg/s)","gz(deg/s)","cx(G)","cy(G)","cz(G)"]
MOTION_SAMPLING_RATE = 100     


# setup LSL streams
lsl_eeg_info = StreamInfo('Reak_EEG', 'EEG', len(EEG_CHANNELS), EEG_SAMPLING_RATE, 'float32', 'reak_eeg_001')
for label in EEG_CHANNELS:
    lsl_eeg_info.desc().append_child("channels").append_child("channel").append_child_value("label", label)
lsl_eeg_outlet = StreamOutlet(lsl_eeg_info)

lsl_motion_info = StreamInfo('Reak_Motion', 'Motion', len(MOTION_CHANNELS), MOTION_SAMPLING_RATE, 'float32', 'reak_motion_001')
for label in MOTION_CHANNELS:
    lsl_motion_info.desc().append_child("channels").append_child("channel").append_child_value("label", label)
lsl_motion_outlet = StreamOutlet(lsl_motion_info)

print("[INFO] LSL streams initialized: Reak_EEG + Reak_Motion")


# graceful shutdown
shutdown_flag = False
def signal_handler(sig, frame):
    global shutdown_flag
    print("\n[INFO] Ctrl+C received. Shutting down...")
    shutdown_flag = True
signal.signal(signal.SIGINT, signal_handler)

# callbacks
def eeg_callback(samples):
    for sample in samples:
        lsl_eeg_outlet.push_sample(sample)

def motion_callback(sample):
    lsl_motion_outlet.push_sample(sample)


# main
if __name__ == "__main__":
    proxy = AlchemiacProxy("00:80:E1:27:4C:28", eeg_callback=eeg_callback, motion_callback=motion_callback)
    proxy.waitForConnected()
    print("[INFO] Connected. Stabilizing signals...")
    sleep(10)
    print("[INFO] Streaming EEG + Motion via LSL. Press Ctrl+C to stop.")

    try:
        while not shutdown_flag:
            sleep(0.1)  # keep the script alive
    finally:
        proxy.disconnect()
        print("[INFO] Shutdown complete.")
