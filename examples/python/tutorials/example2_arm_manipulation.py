"""
Note: When running this example, please ensure the robot's motion control service `/data/galbot/bin/service_motion_plan`,
    robot state publishing service `/data/galbot/bin/robot_state_publish`,
    and hand-eye calibration publishing service `/data/galbot/bin/eyehand_calib_publish` are loaded;
"""
try:
    import galbot_sdk.g1 as gm
    from galbot_sdk.g1 import GalbotMotion, GalbotRobot, ControlStatus
except ImportError:
    print("Failed to import galbot_sdk, please install it first or check if it is in PYTHONPATH")
    exit(1)

import os

try:
    import numpy as np
except ImportError:
    os.system("pip install numpy")
    import numpy as np

import time
from typing import Sequence, Dict

def printStatus(status):
    if(status == gm.MotionStatus.SUCCESS):
        print("Execution result: SUCCESS, Execution successful")
    elif(status == gm.MotionStatus.TIMEOUT):
        print("Execution result: TIMEOUT, Execution timeout")
    elif(status == gm.MotionStatus.FAULT):
        print("Execution result: FAULT, Fault occurred, unable to continue execution")
    elif(status == gm.MotionStatus.INVALID_INPUT):
        print("Execution result: INVALID_INPUT, Input parameters do not meet requirements")
    elif(status == gm.MotionStatus.INIT_FAILED):
        print("Execution result: INIT_FAILED, Internal communication component creation failed")
    elif(status == gm.MotionStatus.IN_PROGRESS):
        print("Execution result: IN_PROGRESS, Moving but not in position")
    elif(status == gm.MotionStatus.STOPPED_UNREACHED):
        print("Execution result: STOPPED_UNREACHED, Stopped but did not reach target")
    elif(status == gm.MotionStatus.DATA_FETCH_FAILED):
        print("Execution result: DATA_FETCH_FAILED, Data acquisition failed")
    elif(status == gm.MotionStatus.PUBLISH_FAIL):
        print("Execution result: PUBLISH_FAIL, Data sending failed")
    elif(status == gm.MotionStatus.COMM_DISCONNECTED):
        print("Execution result: COMM_DISCONNECTED, Connection failed")

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
    Calculate the rotation angle error between two quaternions (radians)

    Parameters:
        A (np.ndarray): First quaternion [x, y, z, w]
        B (np.ndarray): Second quaternion [x, y, z, w]

    Returns:
        float: Rotation angle error (radians)
    """
    qA = quat_normalize(A[3:7])
    qB = quat_normalize(B[3:7])

    q_err = quat_multiply(qB, quat_conjugate(qA))
    q_err = quat_normalize(q_err)

    # Numerical stability
    qw = np.clip(q_err[3], -1.0, 1.0)

    angle = 2 * np.arccos(qw)
    return angle  # Unit: radians


def calculate_error(pose1: Sequence[float], pose2: Sequence[float]) -> Dict[str, float]:
    """
    Calculate position error and rotation error between two poses (radians)

    Parameters:
        pose1 (Sequence[float]): First pose [x, y, z, qx, qy, qz, qw]
        pose2 (Sequence[float]): Second pose [x, y, z, qx, qy, qz, qw]

    Returns:
        dict: Dictionary containing position error (meters) and rotation error (radians)
    """
    A, B = np.array(pose1), np.array(pose2)
    pos_err = np.linalg.norm(A[:3] - B[:3])
    rot_err = orientation_error_angle(A, B)

    return {
        "position_error_norm": pos_err,
        "orientation_error_rad": rot_err,
        "orientation_error_deg": np.degrees(rot_err)
    }

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
        # Get GalbotMotion singleton and initialize
        motion = GalbotMotion.get_instance()
        robot = GalbotRobot.get_instance()

        if motion.init():
            print("GalbotMotion initialization successful")
        else:
            print("GalbotMotion initialization failed")
        if robot.init():
            print("GalbotRobot initialization successful")
        else:
            print("GalbotRobot initialization failed")

        # Program starts immediately, wait for data readiness time
        time.sleep(1)

        # Define target pose
        chain_pose_baselink = {
            "leg": [0.0596,-0.0000,1.0327,0.5000,0.5003,0.4997,0.5000],
            "head": [0.0599,0.0002,1.4098,-0.7072,0.0037,0.0037,0.7069],
            "left_arm": [0.1267,0.2342,0.7356,0.0220,0.0127,0.0343,0.9991],
            "right_arm": [0.1267,-0.2345,0.7358,-0.0225,0.0126,-0.0343,0.9991]
        }
        # Define target chain name, target pose, reference pose, end link
        target_frame = "EndEffector"
        reference_frame = "base_link"
        target_chain = "left_arm"
        end_link = "left_arm_end_effector_mount_link"

        # 1. Get current left_arm end-effector pose
        try:
            status, original_pose = motion.get_end_effector_pose_on_chain(
                chain_name=target_chain,
                frame_id=target_frame,
                reference_frame=reference_frame
            )
            assert status == gm.MotionStatus.SUCCESS, "Failed to get end-effector pose"
            print(f"✅ Current {target_chain} end-effector pose: {original_pose}")
            time.sleep(0.8)
        except Exception as e:
            print(f"❌ Exception getting end-effector pose by chain name: {e}")

        # 2. Solve joint angles based on target pose IK and verify the solution
        # 2.1 Solve joint angles joint_angles_ik for target pose through IK
        try:
            status, joint_angles_ik = motion.inverse_kinematics(
                target_pose=chain_pose_baselink[target_chain],
                chain_names=[target_chain],
                target_frame=target_frame,
                reference_frame=reference_frame,
                enable_collision_check=False # Disable collision detection
            )
            assert status == gm.MotionStatus.SUCCESS, "IK solving failed"
            print(f"✅ Target {target_chain} IK solving successful joint_angles_ik: {joint_angles_ik}")
            time.sleep(1)
        except Exception as e:
            print(f"❌ IK solving exception: {e}")

        # 2.2 Set end-effector pose to target pose tgt_pose_ik by setting joint group angles joint_angles_ik
        try:
            status = robot.set_joint_positions(
                joint_angles_ik[target_chain], 
                [target_chain], 
                [], 
                True,
                0.1,
                20.0,
            )
            assert status == ControlStatus.SUCCESS, "Setting joint group angles failed"
            print(f"✅ Setting {target_chain} joint group angles successful.")
            time.sleep(1)
        except Exception as e:
            print(f"❌ Setting {target_chain} joint group angles exception: {e}")

        # 2.3 Verify whether the set joint group angles are consistent with the solved angles
        try:
            status, tgt_pose_ik = motion.get_end_effector_pose_on_chain(
                chain_name=target_chain,
                frame_id=target_frame,
                reference_frame=reference_frame
            )
            assert status == gm.MotionStatus.SUCCESS, "Failed to get end-effector pose"
            print(f"✅ Getting {target_chain} end-effector pose successful: {tgt_pose_ik}")
            time.sleep(1)

            error = calculate_error(tgt_pose_ik, chain_pose_baselink[target_chain])
            print(f"End-effector pose error: {error}")
        except Exception as e:
            print(f"❌ Getting {target_chain} end-effector pose exception: {e}")

        # 2.4 Verify whether the end-effector pose tgt_pose_fk corresponding to joint group angles joint_angles_ik solved by FK is consistent with target pose tgt_pose_ik
        try:
            status, tgt_pose_fk = motion.forward_kinematics(
                target_frame=end_link,
                reference_frame=reference_frame,
                joint_state=joint_angles_ik,
                params=gm.Parameter()
            )
            assert status == gm.MotionStatus.SUCCESS, "FK solving failed"
            print(f"✅ Target {target_chain} FK solving successful: {tgt_pose_fk}")
            time.sleep(1)

            error = calculate_error(tgt_pose_fk, chain_pose_baselink[target_chain])
            print(f"FK solving error: {error}")
        except Exception as e:
            print(f"❌ FK solving exception: {e}")

        time.sleep(3)
        print()

        # 3. Restore to original pose by setting end-effector pose
        # 3.1 Set end-effector pose to restore to original pose
        try:
            status = motion.set_end_effector_pose(
                target_pose=original_pose,
                end_effector_frame=target_chain,
                reference_frame=reference_frame,
                enable_collision_check=False,
                is_blocking=True,
                timeout=5.0,
                params=gm.Parameter()
            )
            assert status == gm.MotionStatus.SUCCESS, "Setting end-effector pose failed"
            print(f"✅ Setting end-effector pose successful: status={status}")
            time.sleep(1)
        except Exception as e:
            print(f"❌ Setting {target_chain} end-effector pose exception: {e}")

        # 3.2 Get end-effector pose and verify whether it has been restored to original pose
        try:
            status, original_pose_rec = motion.get_end_effector_pose_on_chain(
                chain_name=target_chain,
                frame_id=target_frame,
                reference_frame=reference_frame
            )
            assert status == gm.MotionStatus.SUCCESS, "Failed to get end-effector pose"
            print(f"✅ Getting {target_chain} end-effector pose successful: {original_pose_rec}")
            time.sleep(0.8)
            
            error = calculate_error(original_pose_rec, original_pose)
            print(f"Restore end-effector pose error: {error}")
        except Exception as e:
            print(f"❌ Setting end-effector pose exception: {e}")
    
    except Exception as e:
        print(f"❌ Main program exception: {e}")
    finally:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()

if __name__=="__main__":
    main()