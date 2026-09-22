import time
import math
from galbot_sdk.g3 import GalbotRobot
from galbot_sdk.g3 import JointCommand

def head_high_frequency_control():
    """
    Head high-frequency control example
    """

    control_frequency = 100.0  # Hz
    dt = 1.0 / control_frequency
    duration = 4.0  # Control for 4 seconds

    amplitude = 0.3  # Maximum oscillation amplitude (rad)
    frequency = 0.5  # Sine frequency (Hz)
    # Joint group name to control
    joint_groups = ["head"]
    # Fill this field to control specific joints, which will override joint_groups. If empty, controls all joints in joint_groups by default
    joint_names = []

    print("Start high-frequency head control")

    joint_commands = [JointCommand(), JointCommand()]

    start_time = time.time()

    while True:
        current_time = time.time() - start_time
        if current_time > duration:
            break

        # Generate sine trajectory
        target_position = amplitude * math.sin(
            2 * math.pi * frequency * current_time
        )

        # Set head joint angles
        joint_commands[0].position = target_position
        joint_commands[1].position = target_position
        print(f"current: {current_time:.2f}s, target: {target_position:.3f} rad")

        # Expected arrival time
        time_from_start_sec = 0.0

        execution_status = GalbotRobot().set_joint_commands(
            joint_commands,
            joint_groups,
            joint_names,
            time_from_start_sec
        )

        # Sleep at a fixed interval
        time.sleep(dt)

    print("end")

def main():
    # Get and initialize the GalbotRobot singleton
    robot = GalbotRobot()

    if robot.init():
        print("System initialized successfully!")
    else:
        print("System initialization failed!")
        return

    # Program started, waiting for data
    time.sleep(2)
    
    head_high_frequency_control()

    # Exit system and release SDK resources
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    print("Program exited")


if __name__ == "__main__":
    main()
