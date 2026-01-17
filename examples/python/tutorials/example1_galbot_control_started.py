"""
Note: When running this example, please ensure the robot's `emergency stop button` is released;
"""
import time

try:
    from galbot_sdk.g1  import GalbotRobot, ControlStatus
except ImportError:
    print("Failed to import galbot_sdk, please install it first or check if it is in PYTHONPATH")
    exit(1)


def demo_heart_pose(robot: GalbotRobot,
                    joint_group_names: list,
                    position_seq: list,
                    is_blocking: bool,
                    max_speed: float,
                    timeout_s: float,
                    retry_count: int = 3
                    ):
    """
    Robot heart gesture demonstration function

    Parameters:
        robot (GalbotRobot): GalbotRobot instance
        joint_group_names (list): List of joint group names to control
        position_seq (list): List of joint group angle sequences to set
        is_blocking (bool): Whether to set angles in blocking mode
        max_speed (float): Maximum speed
        timeout_s (float): Timeout time (seconds)
        retry_count (int, optional): Number of retries, default is 3

    Returns:
        None
    """

    # Get current joint group angles for subsequent restoration
    original_pos = robot.get_joint_positions(joint_group_names, [])
    print(f"Current angles of joint group {joint_group_names}: {original_pos}")

    # Start heart gesture
    pos_idx = 0
    print("Starting heart gesture...")
    while True:
        time.sleep(1)
        pos = position_seq[pos_idx]
        control_status = robot.set_joint_positions(
            pos, joint_group_names, [], is_blocking, max_speed, timeout_s
        )

        # If setting fails, retry 3 times
        retry_cnt = retry_count
        while control_status != ControlStatus.SUCCESS and retry_cnt > 0:
            print(f"Setting angles for joint group {joint_group_names} failed, retrying {retry_cnt}...")
            retry_cnt = retry_cnt - 1
            time.sleep(1)
            control_status = robot.set_joint_positions(
                pos, joint_group_names, [], is_blocking, max_speed, timeout_s
            )
        # If successful, switch to next pose sequence
        if control_status == ControlStatus.SUCCESS:
            print(f"Setting angles for joint group {joint_group_names} successful")
            pos_idx = pos_idx + 1
        # If failed, break the loop
        else:
            print(f"Setting angles for joint group {joint_group_names} failed")
            break

        # If all pose sequences are completed, break the loop
        if pos_idx > len(position_seq) - 1:
            break

    # Get current joint group angles
    print("Showing heart gesture for 15 seconds, then restoring original pose...")
    time.sleep(5)

    # Restore original pose joint group angles
    control_status = robot.set_joint_positions(
        original_pos, joint_group_names, [], is_blocking, max_speed, timeout_s
    )
    # If setting fails, retry 5 times
    retry_cnt = retry_count
    while control_status != ControlStatus.SUCCESS and retry_cnt > 0:
        print(f"Setting angles for joint group {joint_group_names} failed, retrying {retry_cnt}...")
        retry_cnt = retry_cnt - 1
        time.sleep(2)
        control_status = robot.set_joint_positions(
            original_pos, joint_group_names, [], is_blocking, max_speed, timeout_s
        )
    # If successful, restore original pose
    if control_status == ControlStatus.SUCCESS:
        print(f"Restoring angles for joint group {joint_group_names} successful")
    else:
        print(f"Restoring angles for joint group {joint_group_names} failed")

def check_robot_safety():
    """Check if the robot is safe"""
    # Prompt for precautions
    print("⚠️  Note: 1. Please ensure the robot's emergency stop button is released; 2. Please ensure there are no obstacles in front, back, left, and right of the robot to avoid unexpected situations.")
    while True:
        key = input("Please confirm that the robot's emergency stop button is released and there are no obstacles. Continue? (y/n)...")
        if key == 'y':
            print("User confirmed, continuing execution...")
            break
        elif key == 'n':
            print("User not confirmed, program exiting...")
            exit(1)
        else:
            print("Input error, please enter 'y' or 'n'")

def main():
    check_robot_safety()

    try:
        # Get robot instance
        robot = GalbotRobot.get_instance()
        
        # Initialize robot
        state = robot.init()
        if not state:
            print(f"Initialization failed")
            exit(1)
        else:
            print(f"Initialization successful")
            print(f"Is robot running: {robot.is_running()}")

        # Wait for data preparation
        time.sleep(3)

        # Get list of joint names
        joint_names = robot.get_joint_names()
        if len(joint_names) > 0:
            print(f"List of joint names: {joint_names}")
        else:
            print(f"Failed to get list of joint names")

        # Get joint positions using joint group names, empty returns all joints by default
        joint_group_names = ["left_arm", "right_arm"]
        # Left and right arm heart gesture sequence
        position_seq = [
            [1.53, 0.36, -2.54, -1.80, 0.12, -0.82, 0.09, # left_arm
             -1.53, -0.36, 2.54, 1.80, -0.12, 0.82, -0.09] # right_arm
        ]
        # Whether to block and wait for joints to reach position
        is_blocking = True
        # Limit maximum joint speed to 0.1rad/s
        max_speed = 0.1
        # Maximum blocking wait time
        timeout_s = 20

        # Perform heartbeat gesture
        demo_heart_pose(robot, joint_group_names, position_seq,
                        is_blocking, max_speed, timeout_s)

    except Exception as e:
        print(f"An exception occurred: {e}")
    finally:
        # Actively send SIGINT shutdown signal
        robot.request_shutdown()
        # Wait to enter shutdown state
        robot.wait_for_shutdown()
        # Release SDK resources
        robot.destroy()
        print('Resource release successful')


if __name__ == "__main__":
    main()