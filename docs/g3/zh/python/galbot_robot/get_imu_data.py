import time
from galbot_sdk.g3 import GalbotRobot, SensorType
from typing import Dict

def print_imu_data(imu_data: dict):
    """
    imu_data: dict, includes:
        - 'timestamp_ns'
        - 'accel'   : {'x', 'y', 'z'}
        - 'gyro'    : {'x', 'y', 'z'}
        - 'magnet'  : {'x', 'y', 'z'}
    """
    if not imu_data:
        print("IMU data is empty")
        return

    print(f"Timestamp (ns): {imu_data.get('timestamp_ns')}")

    accel = imu_data.get("accel", {})
    gyro = imu_data.get("gyro", {})
    magnet = imu_data.get("magnet", {})

    print(
        f"Accelerometer: x={accel.get('x')}, "
        f"y={accel.get('y')}, "
        f"z={accel.get('z')}"
    )
    print(
        f"Gyroscope:     x={gyro.get('x')}, "
        f"y={gyro.get('y')}, "
        f"z={gyro.get('z')}"
    )
    print(
        f"Magnetometer:  x={magnet.get('x')}, "
        f"y={magnet.get('y')}, "
        f"z={magnet.get('z')}"
    )

# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
# - SensorType::LIDAR_IMU: lidar IMU
# - SensorType::CHASSIS_IMU: Chassis lidar IMU
robot.init({SensorType.CHASSIS_IMU})

# Program started, waiting for data
time.sleep(1)
print("Initialization succeeded")

imu_data = robot.get_imu_data(SensorType.CHASSIS_IMU)
if not imu_data:
    print("No imu data!")
else:
    print("IMU data:")
    print_imu_data(imu_data)

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')