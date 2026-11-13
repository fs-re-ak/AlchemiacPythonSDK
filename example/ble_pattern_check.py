"""
BLE transfer test: connects to a headset whose firmware is built with BLE_TEST_PATTERN=1
(Alchmiac-fw), receives the data through AlchemiacProxy and checks every EEG, PPG and IMU
value against the expected 16-bit counters (see utils/ble_test_pattern.py).

    python ble_pattern_check.py                     # scan, then 300 s capture
    python ble_pattern_check.py --duration 600 --address 00:80:E1:27:47:BC

Exit code 0 when the test passes, 1 otherwise. Never run it with a headset used for
measurements: the test firmware sends no real data.
"""

import argparse
import sys
import threading
from time import sleep, time

from proxies.AlchemiacProxy import AlchemiacProxy
from utils.ble_discovery import scan_and_select
from utils.ble_test_pattern import EegPatternChecker, ImuPatternChecker, report

PROGRESS_PERIOD_S = 10


class PatternSession:
    """Routes AlchemiacProxy's callbacks to the checkers."""

    def __init__(self):
        self.eeg = EegPatternChecker()
        self.imu = ImuPatternChecker()
        self._lock = threading.Lock()
        self._pending_eeg = None    # EEG packet waiting for its PPG callback

    # The worker calls eeg_callback, then ppg_callback for the same packet (if it has PPG).
    def eeg_callback(self, samples):
        with self._lock:
            self._flush()
            self._pending_eeg = samples

    def ppg_callback(self, samples):
        with self._lock:
            if self._pending_eeg is not None:
                self.eeg.check_packet(self._pending_eeg, samples)
                self._pending_eeg = None

    def motion_callback(self, sample):
        with self._lock:
            self.imu.check_packet(sample)

    def finish(self):
        with self._lock:
            self._flush()

    def _flush(self):
        if self._pending_eeg is not None:    # packet without PPG (older firmware)
            self.eeg.check_packet(self._pending_eeg, None)
            self._pending_eeg = None

    def progress(self, elapsed):
        with self._lock:
            return (f"[{elapsed:5.0f} s] EEG {self.eeg.packets_ok} ok / {self.eeg.packets_lost} perdus, "
                    f"IMU {self.imu.packets_ok} ok / {self.imu.packets_lost} perdus, "
                    f"retours à 0 : {self.eeg.wraps}, "
                    f"erreurs : {self.eeg.error_count + self.imu.error_count}")


def main():
    parser = argparse.ArgumentParser(description="Test du transfert BLE avec le firmware BLE_TEST_PATTERN=1")
    parser.add_argument("--duration", type=float, default=300,
                        help="durée de la capture en secondes (défaut 300 : au moins un cycle 16 bits de 262 s)")
    parser.add_argument("--address", help="adresse BLE du casque (sinon : recherche et choix)")
    args = parser.parse_args()

    address = args.address or scan_and_select()
    session = PatternSession()
    proxy = AlchemiacProxy(address,
                           eeg_callback=session.eeg_callback,
                           motion_callback=session.motion_callback,
                           ppg_callback=session.ppg_callback)
    try:
        proxy.waitForConnected()
        print(f"Capture de {args.duration:.0f} s (Ctrl+C pour arrêter plus tôt)")
        start = time()
        while (elapsed := time() - start) < args.duration:
            sleep(min(PROGRESS_PERIOD_S, args.duration - elapsed))
            print(session.progress(time() - start))
    except KeyboardInterrupt:
        print("Arrêt demandé.")
    finally:
        proxy.disconnect()
        session.finish()

    text, passed = report(session.eeg, session.imu)
    print()
    print(text)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
