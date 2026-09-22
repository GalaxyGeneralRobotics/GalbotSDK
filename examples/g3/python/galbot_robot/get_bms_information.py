import time
from galbot_sdk.g3 import GalbotRobot


def print_bms_information(bms_info: dict):
    """
    bms_info: dict, includes:
        - 'voltage'            : float (V)
        - 'current'            : float (A)
        - 'battery_level'      : float (0-100)
        - 'temperature'        : float (°C)
        - 'charging_status'    : str or int
        - 'health_status'      : str or int
        - 'capacity'           : float (Ah)
    """
    if not bms_info:
        print("BMS information is empty")
        return

    print(f"Voltage (V): {bms_info.get('voltage')}")
    print(f"Current (A): {bms_info.get('current')}")
    print(f"Battery level (%): {bms_info.get('battery_level')}")
    print(f"Temperature (C): {bms_info.get('temperature')}")
    print(f"Charging status: {bms_info.get('charging_status')}")
    print(f"Health status: {bms_info.get('health_status')}")
    print(f"Capacity (Ah): {bms_info.get('capacity')}")


# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()

# Program started, waiting for data
time.sleep(3)
print("Initialization succeeded")

bms_info = robot.get_bms_information()
print_bms_information(bms_info)

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print("Resources released successfully")
