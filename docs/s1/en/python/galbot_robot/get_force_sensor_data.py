import time
from galbot_sdk.s1 import GalbotRobot, GalbotOneFoxtrotSensor


def force_sensor_type_to_string(sensor_type: GalbotOneFoxtrotSensor) -> str:
    """Convert force sensor type to string"""
    type_map = {
        GalbotOneFoxtrotSensor.LEFT_WRIST_FORCE: "LEFT_WRIST_FORCE",
        GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE: "RIGHT_WRIST_FORCE",
    }
    return type_map.get(sensor_type, "UNKNOWN_FORCE_SENSOR")


def print_force_data(sensor_type: GalbotOneFoxtrotSensor, force_data: dict):
    """
    Print force sensor data.

    force_data: dict, including the following fields:
        - 'timestamp_ns': Timestamp (ns)
        - 'force': {'x', 'y', 'z'} Force (N)
        - 'torque': {'x', 'y', 'z'} Torque (N.m)
    """
    print(f"--- {force_sensor_type_to_string(sensor_type)} ---")
    if not force_data:
        print("  Force data is empty")
        print("  Note: S1 force sensor data is supported only on hardware version 2.x; S1 1.x is not supported.")
        return

    print(f"  Timestamp (ns): {force_data.get('timestamp_ns')}")

    force = force_data.get("force", {})
    print(f"  Force (N):  fx={force.get('x')}, fy={force.get('y')}, fz={force.get('z')}")

    torque = force_data.get("torque", {})
    print(f"  Torque (Nm): tx={torque.get('x')}, ty={torque.get('y')}, tz={torque.get('z')}")


robot = GalbotRobot()
if robot.init():
    print("System initialized successfully!")
else:
    print("System initialization failed!")
    exit(1)

print("Note: S1 force sensor data is supported only on hardware version 2.x; S1 1.x is not supported.")

time.sleep(1)

print("\n===== Get left wrist force sensor data =====")
left_force_data = robot.get_force_sensor_data(GalbotOneFoxtrotSensor.LEFT_WRIST_FORCE)
print_force_data(GalbotOneFoxtrotSensor.LEFT_WRIST_FORCE, left_force_data)

print("\n===== Get calibrated left wrist force data in base_link =====")
calibrated_left_force_data = robot.get_force_sensor_data(
    GalbotOneFoxtrotSensor.LEFT_WRIST_FORCE, calibrated=True, ref_frame="base_link"
)
print_force_data(GalbotOneFoxtrotSensor.LEFT_WRIST_FORCE, calibrated_left_force_data)

print("\n===== Get right wrist force sensor data =====")
right_force_data = robot.get_force_sensor_data(GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE)
print_force_data(GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE, right_force_data)

print("\n===== Get calibrated right wrist force data in mount frame =====")
calibrated_right_force_data = robot.get_force_sensor_data(
    GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE,
    calibrated=True,
    ref_frame="right_arm_end_effector_mount_link",
)
print_force_data(GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE, calibrated_right_force_data)

print("\n===== Get all force sensor data =====")
force_sensor_types = [
    GalbotOneFoxtrotSensor.LEFT_WRIST_FORCE,
    GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE,
]

for sensor_type in force_sensor_types:
    force_data = robot.get_force_sensor_data(sensor_type)
    print_force_data(sensor_type, force_data)

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
print("Resources released successfully")
