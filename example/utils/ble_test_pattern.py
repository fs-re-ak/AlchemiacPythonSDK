"""
Checks the BLE transfer test pattern sent by the firmware built with BLE_TEST_PATTERN=1
(Alchmiac-fw, p2p_server_app.c), on the values delivered by AlchemiacProxy's callbacks.

The firmware replaces the data with 16-bit counters just before sending them:

    n : +1 per EEG sample (250 Hz), m : +1 per IMU packet (50 Hz), both start at 0xFF00
    EEG channel c = 0..6 (int24 big-endian)   bytes [c, n >> 8, n & 0xFF]    -> c * 65536 + n
    EEG channel 7                             bytes [0xFF, n >> 8, n & 0xFF] -> n - 65536
    PPG, 2 per packet (int16 big-endian)      (int16) n of EEG samples 4 and 9
    IMU field i = 0..8 (int16 little-endian)  (int16) (m + i)

Any error in the byte order, the sign extension, the sample order, the packet reassembly
or the loss detection breaks these relations. The checkers convert the callback values
(microvolts, volts, physical units) back to the raw codes, which is exact.

No dependency (no BLE, no numpy): the checkers can be tested on their own.
"""

import math

from proxies import packet_format

PATTERN_START = 0xFF00
EEG_CHANNELS = packet_format.EEG_CHANNELS
EEG_SAMPLES_PER_PACKET = packet_format.EEG_SAMPLES_PER_PACKET
PPG_EEG_INDICES = (4, 9)        # EEG samples whose counter the PPG values carry
LAST_CHANNEL_OFFSET = -65536    # channel 7 tag 0xFF -> negative value
IMU_FIELDS = 9

# Same conversions as AlchemiacProxy (convert_ads1299_to_microvolts, motion_process)
EEG_LSB_UV = (2 * 4.5 * 1e6) / (12 * 2 ** 24)
IMU_SENS = (0.061 / 1000,) * 3 + (8.75 / 1000,) * 3 + (0.14 / 1000,) * 3

MAX_STORED_ERRORS = 20
MAX_IMU_GAP = 1000              # larger jumps of the IMU counter are errors, not losses


def int16(value):
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def int24(value):
    value &= 0xFFFFFF
    return value - 0x1000000 if value & 0x800000 else value


def expected_eeg_raw(n):
    """The 8 raw channel values sent for counter n."""
    return [c * 65536 + n for c in range(EEG_CHANNELS - 1)] + [n + LAST_CHANNEL_OFFSET]


def eeg_bytes(raw):
    return int24(raw).to_bytes(3, byteorder='big', signed=True)


def diagnose_eeg(got, expected):
    """Names the likely cause of a wrong 24-bit value."""
    sent = eeg_bytes(expected)
    if got == int.from_bytes(sent, byteorder='little', signed=True):
        return "ordre des octets inversé (lu en little-endian)"
    if got == int.from_bytes(sent, byteorder='big', signed=False):
        return "extension du signe manquante"
    return "valeur inattendue"


def diagnose_int16(got, expected, sent_order):
    """Names the likely cause of a wrong 16-bit value."""
    swapped = int16(((expected & 0xFF) << 8) | ((expected >> 8) & 0xFF))
    if got == swapped:
        return f"ordre des octets inversé (envoyé en {sent_order})"
    if got == expected & 0xFFFF:
        return "extension du signe manquante"
    return "valeur inattendue"


def _is_nan(value):
    return value is None or (isinstance(value, float) and math.isnan(value))


class _Checker:
    def __init__(self):
        self.errors = []            # first MAX_STORED_ERRORS messages
        self.error_count = 0

    def _error(self, message):
        self.error_count += 1
        if len(self.errors) < MAX_STORED_ERRORS:
            self.errors.append(message)


class EegPatternChecker(_Checker):
    """
    Feed it the eeg_callback and ppg_callback values of each packet, in order:
    check_packet(samples_uv, ppg_volts). A packet the proxy reports as lost (NaN) moves
    the expected counter by 10 samples: if the proxy missed a loss, or reordered or
    corrupted packets, the counter no longer follows and an error is reported.
    """

    def __init__(self):
        super().__init__()
        self.next_n = None          # counter expected for the next EEG sample
        self.packets_ok = 0
        self.packets_lost = 0       # reported lost by the proxy (NaN), after sync
        self.samples_checked = 0
        self.ppg_checked = 0
        self.packets_with_ppg = 0
        self.wraps = 0              # counter 0xFFFF -> 0x0000 seen in consecutive samples
        self.ppg_sign_changes = 0   # PPG +32767 -> -32768 seen
        self._last_ppg = None
        self._packet = 0            # callback number, for the messages

    @property
    def synced(self):
        return self.next_n is not None

    def check_packet(self, samples_uv, ppg_volts=None):
        self._packet += 1
        flat = [v for sample in samples_uv for v in sample]
        if flat and all(_is_nan(v) for v in flat):
            if self.synced:
                self.packets_lost += 1
                self.next_n = (self.next_n + len(samples_uv)) & 0xFFFF
            self._last_ppg = None
            return
        if len(samples_uv) != EEG_SAMPLES_PER_PACKET:
            self._error(f"paquet {self._packet} : {len(samples_uv)} échantillons EEG au lieu de "
                        f"{EEG_SAMPLES_PER_PACKET}")

        packet_ok = True
        sample_counters = []
        for s, sample in enumerate(samples_uv):
            raw = [self._to_raw(v) for v in sample]
            if None in raw or len(raw) != EEG_CHANNELS:
                self._error(f"paquet {self._packet} éch. {s} : valeurs non entières ou manquantes {sample}")
                packet_ok = False
                sample_counters.append(None)
                continue

            # Channel 0 carries n itself. A well-formed sample with an unexpected counter is a
            # jump (resync on it); a malformed one is compared with the expected counter.
            n = raw[0] & 0xFFFF
            if self.synced and n != self.next_n:
                if raw == expected_eeg_raw(n):
                    self._error(f"paquet {self._packet} éch. {s} : compteur {n:#06x}, attendu {self.next_n:#06x} "
                                f"(saut de {(n - self.next_n) & 0xFFFF} échantillons non signalé par "
                                f"l'index de paquet)")
                    packet_ok = False
                else:
                    n = self.next_n
            elif self.synced and n == 0:
                self.wraps += 1
            sample_counters.append(n)

            for c, (got, expected) in enumerate(zip(raw, expected_eeg_raw(n))):
                if got != expected:
                    self._error(f"paquet {self._packet} éch. {s} canal {c} : {got}, attendu {expected} "
                                f"({diagnose_eeg(got, expected)})")
                    packet_ok = False
            self.next_n = (n + 1) & 0xFFFF
            self.samples_checked += 1

        if ppg_volts is not None:
            self.packets_with_ppg += 1
            packet_ok &= self._check_ppg(ppg_volts, sample_counters)
        if packet_ok:
            self.packets_ok += 1

    def _check_ppg(self, ppg_volts, sample_counters):
        ok = True
        if len(ppg_volts) != len(PPG_EEG_INDICES):
            self._error(f"paquet {self._packet} : {len(ppg_volts)} échantillons PPG au lieu de "
                        f"{len(PPG_EEG_INDICES)}")
            return False
        for k, (value, s) in enumerate(zip(ppg_volts, PPG_EEG_INDICES)):
            n = sample_counters[s] if s < len(sample_counters) else None
            if n is None:
                continue
            expected = int16(n)
            if _is_nan(value):
                got = packet_format.PPG_INVALID_RAW     # the SDK turns -32768 into NaN
            else:
                got = round(value / packet_format.ADS1118_LSB_V)
            if got != expected:
                self._error(f"paquet {self._packet} PPG{k} : {got}, attendu {expected} "
                            f"({diagnose_int16(got, expected, 'big-endian')})")
                ok = False
            if self._last_ppg is not None and self._last_ppg > 32000 and got < -32000:
                self.ppg_sign_changes += 1
            self._last_ppg = got
            self.ppg_checked += 1
        return ok

    @staticmethod
    def _to_raw(uv):
        if _is_nan(uv):
            return None
        raw = round(uv / EEG_LSB_UV)
        return raw if abs(raw * EEG_LSB_UV - uv) < EEG_LSB_UV * 1e-3 else None


class ImuPatternChecker(_Checker):
    """
    Feed it the motion_callback values of each IMU packet: check_packet(sample).
    The IMU characteristic has no packet index: a forward jump of the counter is counted
    as lost packets, any other mismatch is an error.
    """

    def __init__(self):
        super().__init__()
        self.next_m = None
        self.packets_ok = 0
        self.packets_lost = 0
        self.wraps = 0
        self._packet = 0

    def check_packet(self, sample):
        self._packet += 1
        if len(sample) != IMU_FIELDS:
            self._error(f"paquet IMU {self._packet} : {len(sample)} champs au lieu de {IMU_FIELDS}")
            return
        raw = [round(v / sens) for v, sens in zip(sample, IMU_SENS)]
        m = raw[0] & 0xFFFF
        well_formed = raw == [int16(m + i) for i in range(IMU_FIELDS)]
        ok = True

        if self.next_m is not None:
            gap = (m - self.next_m) & 0xFFFF
            if not well_formed:
                m = self.next_m         # compare the fields with the expected counter
            elif 0 < gap <= MAX_IMU_GAP:
                self.packets_lost += gap
            elif gap != 0:
                self._error(f"paquet IMU {self._packet} : compteur {m:#06x}, attendu {self.next_m:#06x}")
                ok = False
            if m == 0:
                self.wraps += 1

        for i, got in enumerate(raw):
            expected = int16(m + i)
            if got != expected:
                self._error(f"paquet IMU {self._packet} champ {i} : {got}, attendu {expected} "
                            f"({diagnose_int16(got, expected, 'little-endian')})")
                ok = False
        self.next_m = (m + 1) & 0xFFFF
        if ok:
            self.packets_ok += 1


def report(eeg, imu):
    """Text report and verdict (True = test passed)."""
    lines = [
        "=== Test du transfert BLE (motif 16 bits) ===",
        f"EEG : {eeg.packets_ok} paquets corrects, {eeg.packets_lost} perdus (signalés par l'index), "
        f"{eeg.samples_checked} échantillons vérifiés, {eeg.wraps} retour(s) à 0 du compteur 16 bits",
        f"PPG : {eeg.ppg_checked} échantillons vérifiés dans {eeg.packets_with_ppg} paquets, "
        f"{eeg.ppg_sign_changes} passage(s) de +32767 à -32768",
        f"IMU : {imu.packets_ok} paquets corrects, {imu.packets_lost} perdus, "
        f"{imu.wraps} retour(s) à 0 du compteur",
    ]
    received = eeg.packets_ok + eeg.packets_lost
    if received:
        lines.append(f"Taux de perte EEG : {100 * eeg.packets_lost / received:.2f} %")

    problems = []
    if eeg.error_count:
        problems.append(f"{eeg.error_count} erreur(s) de décodage EEG/PPG")
    if imu.error_count:
        problems.append(f"{imu.error_count} erreur(s) de décodage IMU")
    if eeg.samples_checked == 0:
        problems.append("aucun échantillon EEG reçu")
    if eeg.ppg_checked == 0:
        problems.append("aucun échantillon PPG reçu")
    if imu.packets_ok + imu.error_count == 0:
        problems.append("aucun paquet IMU reçu")
    if eeg.wraps == 0 and eeg.samples_checked:
        problems.append("le compteur EEG n'est jamais revenu à 0 (capture trop courte : 262 s par cycle)")

    for name, checker in (("EEG/PPG", eeg), ("IMU", imu)):
        if checker.errors:
            lines.append(f"Premières erreurs {name} :")
            lines += [f"  - {e}" for e in checker.errors]
    if eeg.error_count and eeg.packets_ok == 0:
        lines.append("Aucun paquet conforme : le firmware est-il compilé avec BLE_TEST_PATTERN=1 ?")

    passed = not problems
    lines.append("RÉSULTAT : RÉUSSI" if passed else "RÉSULTAT : ÉCHEC (" + "; ".join(problems) + ")")
    return "\n".join(lines), passed
