"""
Note: When running this example, please confirm that the robot's navigation function `/data/galbot/bin/service_navigation_plan` has been loaded;
"""
try:
    from galbot_sdk.g1  import GalbotNavigation, GalbotRobot
except ImportError:
    print("Failed to import galbot_sdk, please install it first or check if it's in PYTHONPATH")
    exit(1)

import os

try:
    import numpy as np
except ImportError:
    os.system("pip install numpy")
    import numpy as np

try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    os.system("pip install scipy")
    from scipy.spatial.transform import Rotation as R

import time
from typing import Sequence, Dict

def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.array(q, dtype=np.float64)
    return q / np.linalg.norm(q)

def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """
    Calculate the conjugate of a quaternion

    Parameters:
        q (np.ndarray): Input quaternion [x, y, z, w]

    Returns:
        np.ndarray: Conjugate of the quaternion [x, y, z, w]
    """
    qx, qy, qz, qw = q
    return np.array([-qx, -qy, -qz, qw])

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Calculate the product of two quaternions

    Parameters:
        q1 (np.ndarray): First quaternion [x, y, z, w]
        q2 (np.ndarray): Second quaternion [x, y, z, w]

    Returns:
        np.ndarray: Product of the two quaternions [x, y, z, w]
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def orientation_error_angle(A: np.ndarray, B: np.ndarray) -> float:
    """
    Calculate the rotation angle error between two quaternions (in radians)

    Parameters:
        A (np.ndarray): First quaternion [x, y, z, w]
        B (np.ndarray): Second quaternion [x, y, z, w]

    Returns:
        float: Rotation angle error (in radians)
    """
    qA = quat_normalize(A[3:7])
    qB = quat_normalize(B[3:7])

    q_err = quat_multiply(qB, quat_conjugate(qA))
    q_err = quat_normalize(q_err)

    # Numerically stable
    qw = np.clip(q_err[3], -1.0, 1.0)

    angle = 2 * np.arccos(qw)
    return angle  # Unit: radians


def calculate_error(pose1: Sequence[float], pose2: Sequence[float]) -> Dict[str, float]:
    """
    Calculate the position error and rotation error between two poses (in radians)

    Parameters:
        pose1 (Sequence[float]): First pose, [x, y, z, qx, qy, qz, qw]
        pose2 (Sequence[float]): Second pose, [x, y, z, qx, qy, qz, qw]

    Returns:
        dict: Dictionary containing position error (in meters) and rotation error (in radians)
    """
    A, B = np.array(pose1), np.array(pose2)
    pos_err = np.linalg.norm(A[:3] - B[:3])
    rot_err = orientation_error_angle(A, B)

    return {
        "position_error_norm": pos_err,
        "orientation_error_rad": rot_err,
        "orientation_error_deg": np.degrees(rot_err)
    }

def local_pose_to_global(start_pose: Sequence[float], local_pose: Sequence[float]):
    """
    Convert local pose to global pose

    Parameters:
        start_pose (Sequence[float]): Start pose, [x, y, z, qx, qy, qz, qw]
        local_pose (Sequence[float]): Local pose, [x, y, z, qx, qy, qz, qw]
    
    Returns:
        Sequence[float]: Global pose, [x, y, z, qx, qy, qz, qw]
    """
    start_mat = np.eye(4)
    start_mat[:3, :3] = R.from_quat([start_pose[3], start_pose[4], start_pose[5], start_pose[6]]).as_matrix()
    start_mat[:3, 3] = [start_pose[0], start_pose[1], start_pose[2]]

    local_mat = np.eye(4)
    local_mat[:3, :3] = R.from_quat([local_pose[3], local_pose[4], local_pose[5], local_pose[6]]).as_matrix()
    local_mat[:3, 3] = [local_pose[0], local_pose[1], local_pose[2]]

    global_mat = start_mat @ local_mat

    return global_mat[:3, 3].tolist() + R.from_matrix(global_mat[:3, :3]).as_quat().tolist()

def demo_square_move(robot: GalbotRobot, nav: GalbotNavigation):
    """
    Demonstrate robot moving in a square pattern in navigation environment

    Parameters:
        robot (GalbotRobot): Robot instance
        nav (GalbotNavigation): Navigation instance
    """
    try:
        start_pose = nav.get_current_pose()
    except Exception as e:
        print(f"Failed to get current pose: {e}")
        return
    
    # Move forward 0.5m, turn left 90 degrees
    local_pose = [0.5, 0.0, 0.0, 0.0, 0.0, 0.707, 0.707] 
    
    try:
        # Move forward 0.5m each time, turn left 90 degrees, repeat 4 times to form a square
        for _ in range(4):
            # Calculate target pose
            cur_pose = nav.get_current_pose()
            goal_pose = local_pose_to_global(cur_pose, local_pose)
            
            # Check if path is reachable
            if nav.check_path_reachability(goal_pose, cur_pose):
                # Navigate to target pose
                retry_cnt = 3
                while True:
                    status = nav.navigate_to_goal(goal_pose, enable_collision_check=True, is_blocking=True, timeout=30)
                    time.sleep(0.5)
                    retry_cnt -= 1
                    if nav.check_goal_arrival() or retry_cnt < 0:
                        break
                    else:
                        print(f"Navigation failed, retrying...{retry_cnt}")
                print("navigate_to_goal return status:", status)
                print("Has arrived:", nav.check_goal_arrival())
            else:
                print("Path unreachable or unsafe")

        cur_pose = nav.get_current_pose()
        print(f"Current pose: {cur_pose}, Error from start pose: {calculate_error(cur_pose, start_pose)}")
    except Exception as e:
        print(f"Exception occurred during navigation: {e}")

def move_to_original(robot: GalbotRobot, nav: GalbotNavigation):
    """
    Demonstrate robot returning to start pose in navigation environment

    Parameters:
        robot (GalbotRobot): Robot instance
        nav (GalbotNavigation): Navigation instance
    """
    cur_pose = nav.get_current_pose()
    goal_pose = [0, 0, 0, 0, 0, 0, 1]
    
    try:
        if nav.check_path_reachability(goal_pose, cur_pose):
            retry_cnt = 3
            while True:
                status = nav.navigate_to_goal(goal_pose, enable_collision_check=True, is_blocking=True, timeout=30)
                time.sleep(0.5)
                retry_cnt -= 1
                if nav.check_goal_arrival() or retry_cnt < 0:
                    break
                else:
                    print(f"Navigation failed, retrying...{retry_cnt}")
            print("navigate_to_goal return status:", status)
            print("Has arrived:", nav.check_goal_arrival())
        else:
            print("Path unreachable or unsafe")
    except Exception as e:
        print(f"Exception occurred during navigation: {e}")

def check_robot_safety():
    """Check if robot is safe"""
    # Prompt important notes
    print("⚠️  Note: 1. Please ensure the emergency stop button of the robot is released; 2. Please ensure there are no obstructions around the robot to avoid unexpected situations; 3. Please ensure the area around the robot is clear of obstacles.")
    while True:
        key = input("Please confirm that the robot's emergency stop button is released and there are no obstructions, continue? (y/n)...")
        if key == 'y':
            print("User confirmed, continuing...")
            break
        elif key == 'n':
            print("User did not confirm, exiting program...")
            exit(1)
        else:
            print("Invalid input, please enter 'y' or 'n'")

def main():
    check_robot_safety()
    try:
        # Get robot instance
        robot = GalbotRobot.get_instance()
        # Get navigation instance
        nav = GalbotNavigation.get_instance()
        
        # Initialize robot
        if robot.init():
            print("Robot initialization successful")
        else:
            print("Robot initialization failed")
        # Initialize navigation
        if nav.init():  
            print("Navigation initialization successful")
        else:
            print("Navigation initialization failed")
        
        # Wait for data preparation
        time.sleep(1)
        
        # Check initial localization status
        is_localized = nav.is_localized()
        if not is_localized:
            print("Localization failed, attempting to re-localize: Please move the robot to the origin of the map!")
        time.sleep(3)
        while not is_localized:
            nav.relocalize([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            time.sleep(0.5)
            is_localized = nav.is_localized()

        # square_move
        demo_square_move(robot, nav)

    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()

if __name__ == "__main__":
    main()