"""Builds EEG data characteristic packets exactly as the firmware does (p2p_server_app.c)."""

import random


def eeg_raw_block(seed, n_samples=10):
    """n_samples x 8 random 24-bit signed values, reproducible."""
    rng = random.Random(seed)
    return [[rng.randint(-(1 << 23), (1 << 23) - 1) for _ in range(8)] for _ in range(n_samples)]


def build_packet(index, eeg_raw, ppg_raw=None):
    """[index][10 x 8 x int24 BE][2 x int16 BE (optional)]"""
    out = bytearray([index & 0x7F])
    for sample in eeg_raw:
        for value in sample:
            out += value.to_bytes(3, byteorder='big', signed=True)
    if ppg_raw is not None:
        for value in ppg_raw:
            out += value.to_bytes(2, byteorder='big', signed=True)
    return bytes(out)
