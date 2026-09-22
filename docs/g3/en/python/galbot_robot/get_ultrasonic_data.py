import time
from galbot_sdk.g3 import GalbotRobot, SensorType, UltrasonicType


def ultrasonic_type_to_string(ultrasonic_type: UltrasonicType) -> str:
    """Convert ultrasonic sensor type to string"""
    type_map = {
        UltrasonicType.FRONT_LEFT: "FRONT_LEFT",
        UltrasonicType.FRONT_RIGHT: "FRONT_RIGHT",
        UltrasonicType.RIGHT_LEFT: "RIGHT_LEFT",
        UltrasonicType.RIGHT_RIGHT: "RIGHT_RIGHT",
        UltrasonicType.BACK_LEFT: "BACK_LEFT",
        UltrasonicType.BACK_RIGHT: "BACK_RIGHT",
        UltrasonicType.LEFT_LEFT: "LEFT_LEFT",
        UltrasonicType.LEFT_RIGHT: "LEFT_RIGHT",
    }
    return type_map.get(ultrasonic_type, "UNKNOWN_ULTRASONIC")


def print_ultrasonic_data(ultrasonic_type: UltrasonicType, ultrasonic_data: dict):
    """
    Print ultrasonic sensor data
    
    ultrasonic_data: dict, including the following fields:
        - 'timestamp_ns': Timestamp (ns)
        - 'distance': Distance (m)
    """
    print(f"--- {ultrasonic_type_to_string(ultrasonic_type)} ---")
    if not ultrasonic_data:
        print("  Ultrasonic data is empty")
        return

    print(f"  Timestamp (ns): {ultrasonic_data.get('timestamp_ns')}")
    print(f"  Distance (m): {ultrasonic_data.get('distance')}")


# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()

# Initialize sensors; only sensors passed during initialization can retrieve data to save resources
enable_sensor_set = {SensorType.BASE_ULTRASONIC}
robot.init(enable_sensor_set)

# Program started, waiting for data
time.sleep(1)
print("Initialization succeeded")

# Get single ultrasonic sensor data
print("\n===== Get single ultrasonic sensor data =====")
ultrasonic_data = robot.get_ultrasonic_data(UltrasonicType.FRONT_LEFT)
print_ultrasonic_data(UltrasonicType.FRONT_LEFT, ultrasonic_data)

# Iterate to get all 8 ultrasonic sensor data
print("\n===== Get all ultrasonic sensor data =====")
ultrasonic_types = [
    UltrasonicType.FRONT_LEFT,
    UltrasonicType.FRONT_RIGHT,
    UltrasonicType.RIGHT_LEFT,
    UltrasonicType.RIGHT_RIGHT,
    UltrasonicType.BACK_LEFT,
    UltrasonicType.BACK_RIGHT,
    UltrasonicType.LEFT_LEFT,
    UltrasonicType.LEFT_RIGHT,
]

for ultrasonic_type in ultrasonic_types:
    data = robot.get_ultrasonic_data(ultrasonic_type)
    print_ultrasonic_data(ultrasonic_type, data)

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')
