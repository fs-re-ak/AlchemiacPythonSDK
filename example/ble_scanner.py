import asyncio
from bleak import BleakScanner, BleakClient

async def scan_and_list_services():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    
    if not devices:
        print("No devices found. Ensure your BLE device is powered on and in range.")
        return
    
    for i, device in enumerate(devices):
        print(f"{i}: {device.name} [{device.address}]")
    

if __name__ == "__main__":
    asyncio.run(scan_and_list_services())
