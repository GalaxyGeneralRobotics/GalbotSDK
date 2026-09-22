from galbot_sdk.g1 import GalbotRobot, ControlStatus
import time


def print_base_velocity(base_velocity_info: dict):
    """
    base_velocity_info: dict, includes:
        - 'linear_velocity': [vx, vy, vz] Linear velocity (m/s)
        - 'angular_velocity': [wx, wy, wz] Angular velocity (rad/s)
    """
    if not base_velocity_info:
        print("Base velocity data is empty")
        return

    # Print linear and angular velocity in the same format as the standalone example.
    linear_velocity = base_velocity_info.get("linear_velocity", [])
    if linear_velocity:
        print(
            f"Linear velocity (m/s): vx={linear_velocity[0]}, vy={linear_velocity[1]}, "
            f"vz={linear_velocity[2]}"
        )

    angular_velocity = base_velocity_info.get("angular_velocity", [])
    if angular_velocity:
        print(
            f"Angular velocity (rad/s): wx={angular_velocity[0]}, wy={angular_velocity[1]}, "
            f"wz={angular_velocity[2]}"
        )


# Get GalbotRobot
robot = GalbotRobot()
robot.init()
time.sleep(1)
print("Initialization succeeded")

# Read base velocity before issuing the motion command.
base_velocity_before = robot.get_base_velocity()
if base_velocity_before:
    print("Base velocity before command:")
    print_base_velocity(base_velocity_before)
else:
    print("Failed to get base velocity before command.")

# Set chassis speed
linear_velocity = [0.2, 0.0, 0.0]  # 0.2 m/s
angular_velocity = [0.0, 0.0, 0.0]  # 0.0 rad/s

duration_s = 3.0  # Block while publishing at 10 Hz for 2 seconds; no stop is sent.
status = robot.set_base_velocity(linear_velocity, angular_velocity, duration_s)

if status == ControlStatus.SUCCESS:
    print(f"Velocity publishing completed after {duration_s} seconds; stopping depends on the watchdog.")
else:
    print("Set chassis speed failed.")

# Observe immediately; publishing completion does not imply a stopped base.

# Read base velocity again after the command has finished.
base_velocity_after = robot.get_base_velocity()
if base_velocity_after:
    print("Base velocity after command:")
    print_base_velocity(base_velocity_after)
else:
    print("Failed to get base velocity after command.")

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')
