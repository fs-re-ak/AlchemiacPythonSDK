import asyncio

from utils.ble_discovery import DEVICE_NAME, SCAN_TIMEOUT, scan_devices


async def main():
    print(f"Scanning for '{DEVICE_NAME}' ({SCAN_TIMEOUT:.0f} s)...")
    hermes_devices, all_devices = await scan_devices()

    if hermes_devices:
        print(f"\nMatched {len(hermes_devices)} Hermes device(s):")
        for i, entry in enumerate(hermes_devices):
            print(f"  {i}: {entry.display_name} [{entry.device.address}]")
        return

    print(f"No '{DEVICE_NAME}' devices matched automatically.")
    print(f"\nSaw {len(all_devices)} nearby BLE device(s):")
    for i, entry in enumerate(all_devices):
        print(f"  {i}: {entry.display_name} [{entry.device.address}]")


if __name__ == "__main__":
    asyncio.run(main())
