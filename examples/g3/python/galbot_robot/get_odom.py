import time
from galbot_sdk.g3 import GalbotRobot


def print_odom_data(odom_data: dict):
    """
    Print odometry data
    
    odom_data: dict, includes:
        - 'timestamp_ns': Timestamp (ns)
        - 'position': [x, y, z] Position (m)
        - 'orientation': [qx, qy, qz, qw] Quaternion orientation
        - 'linear_velocity': [vx, vy, vz] Linear velocity (m/s)
        - 'angular_velocity': [wx, wy, wz] Angular velocity (rad/s)
    """
    if not odom_data:
        print("Odom data is empty")
        return

    print(f"Timestamp (ns): {odom_data.get('timestamp_ns')}")
    
    position = odom_data.get("position", [])
    if position:
        print(f"Position (m): x={position[0]}, y={position[1]}, z={position[2]}")
    
    orientation = odom_data.get("orientation", [])
    if orientation:
        print(f"Orientation (quaternion): qx={orientation[0]}, qy={orientation[1]}, "
              f"qz={orientation[2]}, qw={orientation[3]}")

    linear_velocity = odom_data.get("linear_velocity", [])
    if linear_velocity:
        print(f"Linear velocity (m/s): vx={linear_velocity[0]}, vy={linear_velocity[1]}, "
              f"vz={linear_velocity[2]}")

    angular_velocity = odom_data.get("angular_velocity", [])
    if angular_velocity:
        print(f"Angular velocity (rad/s): wx={angular_velocity[0]}, wy={angular_velocity[1]}, "
              f"wz={angular_velocity[2]}")


# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()

# Program started, waiting for data
time.sleep(1)
print("Initialization succeeded")

# Get odometry data
odom_data = robot.get_odom()
if not odom_data:
    print("No odom data!")
else:
    print("Odometry data:")
    print_odom_data(odom_data)

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')
