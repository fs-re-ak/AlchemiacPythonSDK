"""Shared BLE discovery helpers for Hermes V1 headsets."""

from __future__ import annotations

import asyncio
import platform
import sys
from dataclasses import dataclass

from bleak import BleakScanner
from bleak.backends.device import BLEDevice

DEVICE_NAME = "Hermes V1"
EEG_SERVICE_UUID = "9fa480e0-4967-11e5-a151-0002a5d5c51b"
OS = platform.system()
SCAN_TIMEOUT = 15.0 if OS == "Darwin" else 8.0


@dataclass(frozen=True)
class DiscoveredDevice:
    device: BLEDevice
    display_name: str


def _normalize_uuid(uuid: str) -> str:
    return uuid.replace("-", "").lower()


def _advertises_eeg_service(adv) -> bool:
    if not adv or not adv.service_uuids:
        return False
    target = _normalize_uuid(EEG_SERVICE_UUID)
    return any(_normalize_uuid(uuid) == target for uuid in adv.service_uuids)


def _resolve_name(device: BLEDevice, adv=None) -> str | None:
    if device.name:
        return device.name
    if adv and adv.local_name:
        return adv.local_name
    return None


def _is_hermes(device: BLEDevice, adv=None) -> bool:
    # macOS replaces the advertised local name with the BLE module vendor name,
    # so matching on "Hermes V1" is unreliable — rely on the EEG service UUID instead.
    if OS == "Darwin":
        return _advertises_eeg_service(adv)

    name = _resolve_name(device, adv)
    if name and DEVICE_NAME in name:
        return True
    return _advertises_eeg_service(adv)


def _display_name(device: BLEDevice, adv=None) -> str:
    name = _resolve_name(device, adv)
    if name:
        return name
    if _advertises_eeg_service(adv):
        return "Hermes V1 (service match)"
    return f"Unknown device [{device.address}]"


def _entry_from_scan(device: BLEDevice, adv) -> DiscoveredDevice:
    return DiscoveredDevice(device=device, display_name=_display_name(device, adv))


def _merge_scan_results(
    target: dict[str, DiscoveredDevice],
    scan_results: dict,
) -> None:
    for device, adv in scan_results.values():
        target[device.address] = _entry_from_scan(device, adv)


async def scan_devices(timeout: float = SCAN_TIMEOUT) -> tuple[list[DiscoveredDevice], list[DiscoveredDevice]]:
    """
    Scan for BLE devices.

    Returns:
        (hermes_devices, all_devices) — Hermes matches and the full nearby list.
    """
    if OS == "Darwin":
        print(
            f"\n[SCAN] Scanning for Hermes headsets via EEG service UUID "
            f"({timeout:.0f} s)  [macOS — name filter disabled]\n"
        )
    else:
        print(
            f"\n[SCAN] Scanning for '{DEVICE_NAME}' ({timeout:.0f} s)  "
            f"[{OS} — using MAC address]\n"
        )

    all_devices: dict[str, DiscoveredDevice] = {}
    hermes_devices: dict[str, DiscoveredDevice] = {}

    scan_results = await BleakScanner.discover(timeout=timeout, return_adv=True)
    _merge_scan_results(all_devices, scan_results)

    for device, adv in scan_results.values():
        if _is_hermes(device, adv):
            hermes_devices[device.address] = _entry_from_scan(device, adv)

    if OS == "Darwin":
        service_scan = await BleakScanner.discover(
            timeout=timeout,
            return_adv=True,
            service_uuids=[EEG_SERVICE_UUID],
        )
        _merge_scan_results(all_devices, service_scan)
        for device, adv in service_scan.values():
            hermes_devices[device.address] = _entry_from_scan(device, adv)

    if not hermes_devices and OS != "Darwin":
        fallback = await BleakScanner.find_device_by_name(
            DEVICE_NAME,
            timeout=timeout,
        )
        if fallback and _is_hermes(fallback):
            entry = DiscoveredDevice(
                device=fallback,
                display_name=_display_name(fallback),
            )
            all_devices.setdefault(fallback.address, entry)
            hermes_devices[fallback.address] = entry

    return list(hermes_devices.values()), list(all_devices.values())


async def scan_hermes_devices(timeout: float = SCAN_TIMEOUT) -> list[DiscoveredDevice]:
    """Return Hermes matches only (used by ble_scanner.py)."""
    hermes_devices, _ = await scan_devices(timeout)
    return hermes_devices


async def log_nearby_devices(timeout: float = SCAN_TIMEOUT) -> None:
    _, all_devices = await scan_devices(timeout)
    if not all_devices:
        print("[SCAN] No BLE devices seen at all during the scan window.")
        return

    print(f"[SCAN] Saw {len(all_devices)} BLE device(s) nearby:")
    for entry in all_devices:
        print(f"  - {entry.display_name}  [{entry.device.address}]")


def _log_nearby_devices(timeout: float = SCAN_TIMEOUT) -> None:
    asyncio.run(log_nearby_devices(timeout))


def select_device(devices: list[DiscoveredDevice], list_label: str) -> DiscoveredDevice:
    """Print a numbered menu and return the chosen device."""
    addr_label = "UUID" if OS == "Darwin" else "Address"
    width = 54
    print("─" * width)
    print(f"  {list_label}:")
    print("─" * width)
    for i, entry in enumerate(devices):
        print(f"  [{i}]  {entry.display_name:<28}  {addr_label}: {entry.device.address}")
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
    Scan for devices and return the address chosen by the user.

    If one or more Hermes headsets match, the user picks from that list only.
    Otherwise the user picks from every device seen during the scan.
    """
    while True:
        hermes_devices, all_devices = asyncio.run(scan_devices())

        if hermes_devices:
            chosen = select_device(
                hermes_devices,
                f"Found {len(hermes_devices)} Hermes device(s)",
            )
            break

        if all_devices:
            print("[SCAN] No Hermes devices matched automatically.")
            if OS == "Darwin":
                print(
                    "[SCAN] macOS: select your headset from the list below "
                    "(names may show the BLE chip vendor instead of 'Hermes V1')."
                )
            chosen = select_device(
                all_devices,
                f"Found {len(all_devices)} nearby BLE device(s)",
            )
            break

        print("[SCAN] No BLE devices found at all.")
        if OS == "Darwin":
            print(
                "[SCAN] macOS tips: power-cycle the headset, toggle Bluetooth, "
                "close other BLE apps, and confirm this terminal has Bluetooth permission."
            )
        ans = input("  Press Enter to scan again, or 'q' to quit: ").strip().lower()
        if ans == "q":
            print("[INFO] Exiting.")
            sys.exit(0)

    print(f"\n[INFO] Selected: {chosen.display_name}  ({chosen.device.address})\n")
    return chosen.device.address
