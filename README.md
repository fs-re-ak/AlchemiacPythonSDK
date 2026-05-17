# AlchemiacPythonSDK

Open-source Python SDK for the **Hermes V1** EEG headset by Alchemiac. It provides a BLE proxy that streams real-time EEG and motion data to your application via callbacks, along with ready-to-run tools for recording, visualisation, and LSL streaming.

---

## Table of Contents

- [Cloning the repository](#cloning-the-repository)
- [Running AlchemiacController](#running-alchemiaccontroller)
- [Streaming via LSL](#streaming-via-lsl)
- [SDK Integration Guide](#sdk-integration-guide)
  - [AlchemiacProxy](#alchemiacproxy)
  - [Callbacks](#callbacks)
  - [Minimal integration example](#minimal-integration-example)
  - [BLE characteristics reference](#ble-characteristics-reference)

---

## Cloning the repository

```bash
git clone https://github.com/your-org/AlchemiacPythonSDK.git
cd AlchemiacPythonSDK
```

Install Python dependencies (Python 3.9+ recommended):

```bash
cd example
pip install -r requirements.txt
```

Create the data output directory expected by the controller:

```bash
mkdir data
```

> **Platform note:** Bluetooth access on macOS requires granting Terminal (or your IDE) the *Bluetooth* permission in System Preferences → Privacy & Security. On Windows, no extra setup is needed beyond a working Bluetooth adapter.

---

## Running AlchemiacController

`AlchemiacController.py` is the reference application. It scans for nearby Hermes V1 devices over BLE, streams EEG and motion data, saves timestamped CSV files, and opens a real-time EEG visualizer.

Run it from inside the `example/` directory:

```bash
cd example
python AlchemiacController.py
```

### What to expect

1. **BLE scan** — The controller scans for up to 8 seconds looking for any BLE device whose name contains `"Hermes V1"`:

   ```
   [SCAN] Scanning for 'Hermes V1' (8 s)  [Windows — using MAC address]
   ```

   If no device is found, you are prompted to scan again or quit. The scan repeats until at least one device is discovered.

2. **Device selection** — When one or more headsets are found, a numbered menu is shown:

   ```
   ──────────────────────────────────────────────────────
     Found 1 Hermes device(s):
   ──────────────────────────────────────────────────────
     [0]  Hermes V1             Address: 00:80:E1:27:47:BC
   ──────────────────────────────────────────────────────

     Select device [0–0]:
   ```

3. **Connection & subscription** — The proxy connects via BLE and subscribes to EEG data, motion (IMU), button events, and the EEG config characteristic. Confirmation lines appear for each:

   ```
   Connected to Hermes V1 [00:80:E1:27:47:BC]
   Subscribed to button notifications.
   Subscribed to accelerometer notifications.
   Subscribed to EEG data characteristic notifications.
   ```

4. **Live visualizer** — A PyQtGraph window opens showing all 8 EEG channels in real time with a 15–30 Hz bandpass filter applied. The window plots a rolling 5-second buffer.

5. **CSV recording** — Two timestamped files are written to `example/data/`:
   - `alchemiac_eeg_<timestamp>.csv` — columns: `AF8, AF7, CHEEK_R, CHEEK_L, EAR_R, AFz, BROW_L, NOSE` (values in µV)
   - `alchemiac_motion_<timestamp>.csv` — columns: `x(g), y(g), z(g), x(deg/s), y(deg/s), z(deg/s), x(G), y(G), z(G)`

6. **Streaming duration** — After a 1-second stabilisation pause, the session streams for 240 seconds (4 minutes) then shuts down automatically. You can also press **Ctrl+C** at any time for a clean shutdown.

7. **Shutdown** — The proxy unsubscribes from all characteristics, disconnects from the device, flushes the CSV files, and terminates the visualizer process:

   ```
   [MAIN] Cleaning up…
   Disconnected from device.
   [MAIN] Shutdown complete. Goodbye!
   ```

---

## Streaming via LSL

`AlchemiacStreamLSL.py` publishes EEG and motion data as two [Lab Streaming Layer](https://labstreaminglayer.org/) (LSL) streams, making the headset compatible with any LSL-aware recording software (e.g. BrainVision Recorder, OpenViBE, BIDS-LSL, MNE-LSL).

Install the additional dependency before running:

```bash
pip install pylsl
```

Then run from inside the `example/` directory:

```bash
cd example
python AlchemiacStreamLSL.py
```

### What to expect

The startup sequence is identical to `AlchemiacController` — BLE scan, device selection menu, connection confirmations — followed by LSL-specific output:

```
[INFO] LSL streams initialized: Reak_EEG + Reak_Motion
Stabilising signal…
[INFO] Streaming EEG + Motion via LSL. Press Ctrl+C to stop.
```

The script then runs **indefinitely** (no fixed duration), forwarding data to LSL consumers in real time. Press **Ctrl+C** to trigger a clean shutdown:

```
[INFO] Ctrl+C received. Initiating shutdown...
[MAIN] Cleaning up…
[MAIN] Shutdown complete. Goodbye!
```

### LSL stream details

| Stream name | Type | Channels | Rate | Format |
|---|---|---|---|---|
| `Reak_EEG` | `EEG` | 8 — `AF8, AF7, CHEEK_R, CHEEK_L, EAR_R, AFz, BROW_L, NOSE` | 250 Hz | float32 (µV) |
| `Reak_Motion` | `Motion` | 9 — `ax(g), ay(g), az(g), gx(deg/s), gy(deg/s), gz(deg/s), cx(G), cy(G), cz(G)` | 100 Hz | float32 |

Each stream carries full channel metadata (label per channel) readable by any LSL resolver. Stream UIDs are `reak_eeg_001` and `reak_motion_001`.

---

## SDK Integration Guide

The files relevant to integration are:

| File | Purpose |
|---|---|
| `example/proxies/AlchemiacProxy.py` | BLE transport layer — handles connection, packet reassembly, and data decoding |
| `example/AlchemiacController.py` | Reference application — CSV recording + live EEG visualiser |
| `example/AlchemiacStreamLSL.py` | Reference application — forwards data to LSL for compatible recording software |

---

### AlchemiacProxy

`AlchemiacProxy` manages the entire BLE lifecycle on a background thread so your application code stays synchronous and callback-driven.

```python
from proxies.AlchemiacProxy import AlchemiacProxy

proxy = AlchemiacProxy(
    mac_address,          # str — MAC address (Windows/Linux) or UUID (macOS)
    eeg_callback=my_eeg,  # callable(samples: list[list[float]]) | None
    motion_callback=my_motion,  # callable(sample: tuple[float, ...]) | None
    event_callback=None,  # reserved for future button-event callbacks
)
```

**Key methods:**

| Method | Description |
|---|---|
| `waitForConnected()` | Blocks the calling thread until the BLE connection is fully established and all characteristics are subscribed. |
| `disconnect()` | Triggers a clean shutdown: unsubscribes from all characteristics, disconnects BLE, and joins all background threads. Always call this before exiting. |

---

### Callbacks

#### EEG callback

Called once per decoded BLE packet (roughly every 40 ms at 250 Hz). Each call delivers a batch of samples from that packet.

```python
def my_eeg_callback(samples):
    """
    samples: list of lists, shape [n_samples, 8]
             Each inner list is one time point across 8 channels, in microvolts (µV).
             Channels: AF8, AF7, CHEEK_R, CHEEK_L, EAR_R, AFz, BROW_L, NOSE
             Dropped packets are filled with float('nan') across all channels.
    """
    for sample in samples:
        af8, af7, cheek_r, cheek_l, ear_r, afz, brow_l, nose = sample
        # process or buffer the sample
```

The proxy decodes 24-bit ADS1299 raw values to microvolts using:

```
LSB (µV) = (2 × Vref × 10⁶) / (gain × 2²⁴)
```

with default `gain=12` and `Vref=4.5 V`.

#### Motion callback

Called for each IMU packet with physical-unit values already applied:

```python
def my_motion_callback(sample):
    """
    sample: tuple of 9 floats
        ax, ay, az   — accelerometer  (g)
        gx, gy, gz   — gyroscope      (degrees/s)
        cx, cy, cz   — magnetometer   (gauss)
    """
    ax, ay, az, gx, gy, gz, cx, cy, cz = sample
```

---

### Minimal integration example

```python
import asyncio
from time import sleep
from bleak import BleakScanner
from proxies.AlchemiacProxy import AlchemiacProxy

# --- Discover device ---
devices = asyncio.run(BleakScanner.discover(timeout=8.0))
hermes = next((d for d in devices if d.name and "Hermes V1" in d.name), None)
if hermes is None:
    raise RuntimeError("No Hermes V1 found")

# --- Define your callbacks ---
def on_eeg(samples):
    for sample in samples:
        print(f"EEG µV: {[f'{v:.1f}' for v in sample]}")

def on_motion(sample):
    ax, ay, az, *_ = sample
    print(f"Accel: {ax:.3f}g  {ay:.3f}g  {az:.3f}g")

# --- Connect and stream ---
proxy = AlchemiacProxy(hermes.address, eeg_callback=on_eeg, motion_callback=on_motion)
proxy.waitForConnected()

try:
    sleep(30)          # stream for 30 seconds
finally:
    proxy.disconnect()
```

---

### BLE characteristics reference

| Characteristic | UUID | Description |
|---|---|---|
| EEG Service | `9fa480e0-4967-11e5-a151-0002a5d5c51b` | Parent service for EEG |
| EEG Data | `9fa480e1-4967-11e5-a151-0002a5d5c51b` | Notify — raw 24-bit EEG packets |
| EEG Config | `9fa480e2-4967-11e5-a151-0002a5d5c51b` | Write/Notify — device configuration |
| Event Service | `9fa48300-4967-11e5-a151-0002a5d5c51b` | Parent service for button events |
| Button Event | `9fa48301-4967-11e5-a151-0002a5d5c51b` | Notify — button press/release |
| Motion (IMU) | `9fa48201-4967-11e5-a151-0002a5d5c51b` | Notify — accelerometer + gyro + magnetometer |

Packet format for EEG Data: a 1-byte rolling sequence number (0–127) followed by N × 24-byte samples, where each 24-byte block encodes 8 channels × 3 bytes (big-endian, signed). The proxy handles packet-drop detection and gap-filling automatically.

---

## License

See [LICENSE](LICENSE).
