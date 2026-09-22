"""
Note: When running this example, please ensure the robot's `emergency stop button` is released;
"""
import time
import os
from typing import List, Dict, Any

try:
    import galbot_sdk.s1 as gm
    from galbot_sdk.s1 import GalbotRobot
    from galbot_sdk.s1 import GalbotMotion
    from galbot_sdk.s1 import GalbotNavigation
    from galbot_sdk.s1 import (
        SensorType, RgbOutputFormat, S1JointGroup, ControlStatus,
        Trajectory, TrajectoryPoint, JointCommand
    )
except ImportError:
    print("Failed to import galbot_sdk, please install it first or check if it is in PYTHONPATH")
    exit(1)

try:
    import cv2
except ImportError:
    os.system("pip install opencv-python")
    import cv2

try:
    import numpy as np
except ImportError:
    os.system("pip install numpy")
    import numpy as np


def decode_compressed_image(compressed_image: Dict[str, Any]) -> np.ndarray:
    """
    Decode CompressedImage image.

    Parameters:
        compressed_image (dict): image dict, keys: [header, format, data, "depth_scale"]

    Returns:
        numpy.ndarray: decoded image
    """
    image_data = compressed_image["data"]
    if compressed_image["format"] == "jpeg":
        return decode_rgb_image(image_data)
    elif compressed_image["format"] == "16UC1":
        return decode_depth_image(image_data)
    else:
        raise ValueError(f"Unsupported data format: {compressed_image['format']}")


def decode_rgb_image(image_data: bytes) -> np.ndarray:
    """
    Decode RGB image.

    Parameters:
        image_data (bytes): Raw image data

    Returns:
        numpy.ndarray: Decoded RGB image
    """
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Failed to decode RGB Image")
    return img


def decode_depth_image(image_data: Dict[str, Any]) -> np.ndarray:
    """
    Decode depth image.

    Parameters:
        image_data (dict): Depth image data with metadata

    Returns:
        numpy.ndarray: Decoded depth image
    """
    depth_img = np.frombuffer(image_data["data"], dtype=np.uint16).copy()

    # Check if height and width exist
    if "height" not in image_data or "width" not in image_data:
        raise ValueError("Missing 'height' or 'width' in depth image metadata.")
    if image_data["height"] == 0 or image_data["width"] == 0:
        raise ValueError(
            f"Invalid 'height' ({image_data['height']}) or 'width' ({image_data['width']}) in depth image metadata."
        )

    # Parse depth image
    depth_img = depth_img.reshape((image_data["height"], image_data["width"]))
    depth_img = depth_img.astype(np.float32) / image_data["depth_scale"]

    return depth_img


def fake_vla(rgb_data_dict: dict) -> Dict[str, np.ndarray]:
    """
    Fake VLA (Vision-Language-Action) model implementation.
    
    This function is a placeholder. In a real-world scenario, you would implement
    target detection using computer vision techniques. For this example, we assume
    a default pose.

    Parameters:
        rgb_data_dict (dict): Dictionary containing RGB image data from various sensors

    Returns:
        dict: Trajectories for different robot components (legs, head, arms)
    """
    print("Fake VLA executing...")
    
    num_points = 200

    # S1 zero pose (aligned with C++ examples and GalbotMotionS1::move_whole_body_joint_zero)
    torso_zero = np.array([1.1], dtype=np.float64)
    head_zero = np.array([0.0, -0.26], dtype=np.float64)
    left_arm_zero = np.array([-0.47, -0.94, -0.54, -1.92, 0.2, 0.0, 0.0], dtype=np.float64)
    right_arm_zero = np.array([0.47, 0.94, 0.54, 1.92, -0.2, 0.0, 0.0], dtype=np.float64)

    # Torso: Move linearly from 1.1 to 0.7
    torso_traj = np.linspace([1.1], [0.7], num=num_points, dtype=np.float64)

    # Head: Apply small sinusoidal motion on the pitch joint
    phase = np.linspace(0.0, np.pi, num=num_points, dtype=np.float64)
    head_traj = np.repeat(head_zero.reshape(1, -1), num_points, axis=0)
    head_traj[:, 1] = head_zero[1] + 0.1 * np.sin(phase)

    # Dual arms: Start from zero pose and apply small linear changes only on joint 4
    left_delta = np.zeros_like(left_arm_zero)
    right_delta = np.zeros_like(right_arm_zero)
    left_delta[3] = 0.2
    right_delta[3] = -0.2

    s = np.linspace(0.0, 1.0, num=num_points, dtype=np.float64).reshape(-1, 1)
    left_arm_traj = left_arm_zero.reshape(1, -1) + s * left_delta.reshape(1, -1)
    right_arm_traj = right_arm_zero.reshape(1, -1) + s * right_delta.reshape(1, -1)

    return {
        "torso": torso_traj,
        "head": head_traj,
        "left_arm": left_arm_traj,
        "right_arm": right_arm_traj,
    }


def estimate_vla(robot: 'GalbotRobot', enable_sensor_set: set) -> Dict[str, np.ndarray]:
    """
    Estimate VLA (Vision-Language-Action) model outputs based on sensor data.

    Parameters:
        robot (GalbotRobot): GalbotRobot instance
        enable_sensor_set (set): Set of enabled sensor types

    Returns:
        dict: Joint positions for each joint group
    """
    # Get RGB images from enabled cameras
    rgb_data_dict = {}
    for sensor_type in enable_sensor_set:
        if sensor_type in [
            SensorType.LEFT_ARM_CAMERA,
            SensorType.RIGHT_ARM_CAMERA
        ]:
            # Get RGB data of sensor type
            rgb_data = robot.get_rgb_data(sensor_type, RgbOutputFormat.JPEG, True)
            time.sleep(1)
            
            # Decode RGB image
            if rgb_data:
                rgb_image = decode_compressed_image(rgb_data)
                rgb_data_dict[sensor_type] = rgb_image
            else:
                print(f"Failed to get RGB data from {sensor_type}")
        else:
            print(f"Unsupported sensor type: {sensor_type}")
    
    print("RGB data dictionary keys:", rgb_data_dict.keys())

    # Get joint position list
    joint_positions = fake_vla(rgb_data_dict)
    return joint_positions


def generate_target_point(q: List[float], target_time: float = 10.0) -> 'TrajectoryPoint':
    """
    Generate target points for trajectory execution.

    Parameters:
        q (List[float]): Joint positions
        target_time (float): Time from start in seconds (default: 10.0)

    Returns:
        TrajectoryPoint: Target trajectory point with specified joint commands
    """
    joint_position = TrajectoryPoint()
    joint_position.time_from_start_second = target_time
    joint_command_vec = []
    for joint in q:
        joint_cmd = JointCommand()
        joint_cmd.position = joint
        joint_command_vec.append(joint_cmd)
    joint_position.joint_command_vec = joint_command_vec
    return joint_position


def generate_target_trajectory(
    trajectory: np.ndarray, 
    joint_groups: List[str] = None, 
    joint_names: List[str] = None, 
    dt: float = 0.008
) -> 'Trajectory':
    """
    Generate trajectory for joints.

    Parameters:
        trajectory (np.ndarray): 2D array of joint positions over time
        joint_groups (List[str]): List of joint groups
        joint_names (List[str]): List of joint names
        dt (float): Time step between trajectory points (default: 0.008)

    Returns:
        Trajectory: Generated trajectory for execution
    """
    if joint_groups is None:
        joint_groups = []
    if joint_names is None:
        joint_names = []

    if trajectory is None or np.ndim(trajectory) != 2 or len(trajectory) == 0:
        return None

    # Create Trajectory
    traj = Trajectory()
    traj.joint_groups = joint_groups
    traj.joint_names = joint_names

    current_time = 0.0
    points = []
    for state in trajectory:
        current_time += dt
        # Generate target point for each joint
        traj_point = generate_target_point(state, current_time)
        points.append(traj_point)

    traj.points = points
    return traj


def print_joint_states(joint_states: List['JointState']) -> None:
    """
    Print joint states in a readable format.

    Parameters:
        joint_states (List[JointState]): List of joint states
    """
    for js in joint_states:
        print(
            f" : position = {js.position} , velocity = {js.velocity} "
            f", acceleration = {js.acceleration} , effort = {js.effort} , current = {js.current}"
        )


def check_robot_safety() -> None:
    """
    Check if the robot is safe to operate.
    Prompts the user to confirm safety conditions before proceeding.
    """
    # Prompt for precautions
    print(
        "⚠️  Note: 1. Please ensure the robot's emergency stop button is released; "
        "2. Please ensure there are no obstacles in front, back, left, and right "
        "of the robot to avoid unexpected situations."
    )
    
    while True:
        key = input(
            "Please confirm that the robot's emergency stop button is released "
            "and there are no obstacles. Continue? (y/n)..."
        ).lower()
        
        if key == 'y':
            print("User confirmed, continuing execution...")
            break
        elif key == 'n':
            print("User not confirmed, program exiting...")
            exit(1)
        else:
            print("Input error, please enter 'y' or 'n'")


def main() -> None:
    """
    Main function to execute the VLA (Vision-Language-Action) example.
    Initializes the robot, estimates actions based on sensor data, and executes trajectories.
    """
    check_robot_safety()

    robot = None  # Initialize robot variable for cleanup in finally block

    try:
        # Get robot instances
        robot = GalbotRobot()
        motion = GalbotMotion()
        navi = GalbotNavigation()

        # Enable required sensors - S1 only has arm cameras, not head cameras
        enable_sensor_set = {
            SensorType.LEFT_ARM_CAMERA,
            SensorType.RIGHT_ARM_CAMERA
        }
                             
        # Initialize robot components
        if not robot.init(enable_sensor_set):
            print("Robot initialization failed")
            exit(1)
        else:
            print("Robot initialization successful")
            
        if not motion.init():
            print("Motion initialization failed")
            exit(1)
        else:
            print("Motion initialization successful")
            
        if not navi.init():
            print("Navigation initialization failed")
            exit(1)
        else:
            print("Navigation initialization successful")
        
        # Wait for data preparation
        time.sleep(3)

        # Estimate VLA actions
        joint_positions = estimate_vla(robot, enable_sensor_set)
        
        # Generate target trajectory with fixed S1 order
        joint_groups = ["torso", "head", "left_arm", "right_arm"]
        joint_traj = np.concatenate([joint_positions[group] for group in joint_groups], axis=-1)
        
        # Final joint position check state
        whole_body_joint = joint_traj[-1]
        base_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        check_state = gm.RobotStates()
        check_state.whole_body_joint = whole_body_joint
        check_state.base_state = base_state
        print(f"✅ Final joint position check state: {whole_body_joint}")
        
        # Check collision
        status, collision_res = motion.check_collision(
            start=[check_state],
            enable_collision_check=True
        )
        time.sleep(1)

        if status == gm.MotionStatus.SUCCESS:
            print(f"✅ OK: collision check finished: {collision_res} (False=no collision)")

            # Get joint states before execution
            before_joint_states = robot.get_joint_states(joint_groups, [])
            print(f"Joint groups to execute: {joint_groups}")
            print(f"Joint states size before execution: {len(before_joint_states)}")
            print_joint_states(before_joint_states)

            # Execute trajectory
            trajectory = generate_target_trajectory(joint_traj, joint_groups, [])
            if trajectory is not None:
                status = robot.execute_joint_trajectory(trajectory, True)
                time.sleep(1)

                if status == ControlStatus.SUCCESS:
                    print("✅ Joint trajectory execution successful.")
                else:
                    print("❌ Joint trajectory execution failed.")
                    print(f"Diagnostic: target points = {len(trajectory.points)}, joint dimension = {len(whole_body_joint)}")

                # Check joint state
                joint_states = robot.get_joint_states(joint_groups, [])
                print_joint_states(joint_states)

                # Compute position error after execution
                error_norm_after = sum(abs(whole_body_joint[i] - joint_states[i].position) for i in range(len(whole_body_joint)))
                print(f"Absolute position error norm after execution: {error_norm_after}")
                print(f"✅ Final joint position check state after execution")
            else:
                print("❌ Generated trajectory is invalid, cannot execute.")
        else:
            print("❌ Collision check failed, will not execute the joint trajectory.")

    except Exception as e:
        print(f"An exception occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure proper cleanup
        if robot is not None:
            # Actively send SIGINT shutdown signal
            robot.request_shutdown()
            # Wait to enter shutdown state
            robot.wait_for_shutdown()
            # Release SDK resources
            robot.destroy()
        print('Resource release successful')


if __name__ == "__main__":
    main()