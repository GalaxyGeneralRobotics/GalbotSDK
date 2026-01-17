import time
from galbot_sdk.g1 import GalbotRobot, SensorType
from typing import Dict

def print_imu_data(imu_data: dict):
    """
    imu_data: dict，包含:
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

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init({SensorType.TORSO_IMU})

# 程序立即启动，稍等数据就绪时间
time.sleep(1)
print("初始化成功")

imu_data = robot.get_imu_data(SensorType.TORSO_IMU)
if not imu_data:
    print("No imu data!")
else:
    print("imu数据：")
    print_imu_data(imu_data)

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')