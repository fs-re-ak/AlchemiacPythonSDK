"""
Decoding of the EEG data characteristic (9fa480e1-...) sent by the Hermes V1.

Packet layout (after the 1-byte packet index, removed by AlchemiacProxy):

    10 EEG samples x 8 channels x 3 bytes (24-bit signed, big-endian)  = 240 bytes
    2 PPG samples x 2 bytes (16-bit signed, big-endian)                =   4 bytes  (firmware with PPG clip)

Firmware without PPG sends only the 240 EEG bytes: the PPG part is then absent.
This module has no dependency so it can be tested on its own.
"""

import math

EEG_CHANNELS = 8
EEG_SAMPLE_SIZE = 24                # 8 channels x 3 bytes
EEG_SAMPLES_PER_PACKET = 10

PPG_SAMPLE_SIZE = 2                 # int16, big-endian
PPG_SAMPLES_PER_PACKET = 2
PPG_TRAILER_SIZE = PPG_SAMPLE_SIZE * PPG_SAMPLES_PER_PACKET
PPG_SAMPLING_RATE = 50              # Hz: one PPG sample every 5 EEG samples (250 SPS)

# ADS1118, FSR +/-4.096 V: 1 LSB = 4.096 / 32768 = 125 uV
ADS1118_LSB_V = 4.096 / 32768
# Sent by the firmware when no valid sample is available (ADS1118 absent, SPI error)
PPG_INVALID_RAW = -32768


def split_payload(data):
    """
    Splits a packet payload (packet index already removed) into its EEG and PPG parts.

    Returns (eeg_bytes, ppg_bytes, leftover):
      - ppg_bytes is None when the packet carries no PPG (older firmware);
      - leftover is the number of trailing bytes that belong to neither part.
    """
    remainder = len(data) % EEG_SAMPLE_SIZE
    if remainder == PPG_TRAILER_SIZE and len(data) > PPG_TRAILER_SIZE:
        return data[:-PPG_TRAILER_SIZE], data[-PPG_TRAILER_SIZE:], 0
    return data[:len(data) - remainder], None, remainder


def decode_eeg_raw(eeg_bytes):
    """Returns the raw 24-bit signed values, one list of 8 channels per sample."""
    samples = []
    for i in range(0, len(eeg_bytes) - EEG_SAMPLE_SIZE + 1, EEG_SAMPLE_SIZE):
        samples.append([
            int.from_bytes(eeg_bytes[i + j:i + j + 3], byteorder='big', signed=True)
            for j in range(0, EEG_SAMPLE_SIZE, 3)
        ])
    return samples


def decode_ppg_raw(ppg_bytes):
    """Returns the raw ADS1118 codes (int16)."""
    return [
        int.from_bytes(ppg_bytes[i:i + PPG_SAMPLE_SIZE], byteorder='big', signed=True)
        for i in range(0, len(ppg_bytes) - PPG_SAMPLE_SIZE + 1, PPG_SAMPLE_SIZE)
    ]


def convert_ads1118_to_volts(raw):
    """ADS1118 code -> volts (NaN for the invalid marker)."""
    if raw == PPG_INVALID_RAW:
        return math.nan
    return raw * ADS1118_LSB_V


def decode_ppg_volts(ppg_bytes):
    """Returns the PPG samples of a packet in volts (NaN when invalid)."""
    return [convert_ads1118_to_volts(raw) for raw in decode_ppg_raw(ppg_bytes)]
