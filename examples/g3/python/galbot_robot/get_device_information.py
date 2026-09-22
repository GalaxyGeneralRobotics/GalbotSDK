import time
from galbot_sdk.g3 import GalbotRobot
from typing import Dict

def print_device_info(device_info: dict):
    """
    device_info: dict, including the following fields:
        - 'model': Device model (str)
        - 'serial_number': Serial number (str)
        - 'firmware_version': Firmware version (str)
        - 'hardware_version': Hardware version (str)
        - 'manufacturer': Manufacturer (str)
    """
    if not device_info:
        print("Device information is empty")
        return

    print("Device information:")
    print(f"  Model: {device_info.get('model', 'N/A')}")
    print(f"  Serial number: {device_info.get('serial_number', 'N/A')}")
    print(f"  Firmware version: {device_info.get('firmware_version', 'N/A')}")
    print(f"  Hardware version: {device_info.get('hardware_version', 'N/A')}")
    print(f"  : {device_info.get('manufacturer', 'N/A')}")

# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()

# Program started, waiting for data
time.sleep(1)
print("Initialization succeeded")

device_info = robot.get_device_information()
if not device_info:
    print("Failed to get device information!")
else:
    print("Device information retrieved successfully!")
    print_device_info(device_info)

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')
