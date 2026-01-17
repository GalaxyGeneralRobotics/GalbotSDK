"""
Note: When running this example, please confirm that the robot's motion control service `/data/galbot/bin/service_motion_plan`,
    robot state publishing service `/data/galbot/bin/robot_state_publish`,
    navigation service `/data/galbot/bin/service_navigation_plan`
    and hand-eye calibration publishing service `/data/galbot/bin/eyehand_calib_publish` have been loaded;
"""
try:
    import galbot_sdk.g1 as gm
    from galbot_sdk.g1  import GalbotNavigation, GalbotRobot, GalbotMotion, JointGroup, SensorType
except ImportError:
    print("Import galbot_sdk failed, please install it first or check if it is in the PYTHONPATH")
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

try:
    import cv2
except ImportError:
    os.system("pip install opencv-python")
    import cv2

import time
from typing import Sequence

def decode_compressed_image(compressed_image, camera_info={}):
    """
    decode CompressedImage image

    Parameters:
        compressed_image (dict): image dict, keys:[header, format, data, "depth_scale"]

    Returns:
        numpy.ndarray: decoded image
    """
    image_data = compressed_image["data"]
    if compressed_image["format"] == "rgb8":
        return decode_rgb_image(image_data)
    elif compressed_image["format"] == "16UC1":
        return decode_depth_image(image_data, compressed_image["depth_scale"], camera_info)
    else:
        raise ValueError(f"Unsupport data format: {compressed_image['format']}")

def decode_rgb_image(image_data):
    """decode rgb image"""
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Fail to Decode RGB Image")
    return img

def decode_depth_image(image_data, depth_scale, camera_info):
    """decode depth image"""
    depth_img = np.frombuffer(image_data, dtype=np.uint16).copy()

    if not camera_info:
        depth_img = depth_img.reshape((720, 1280))
    else:
        depth_img = depth_img.reshape((camera_info["height"], camera_info["width"]))
    depth_img = depth_img.astype(np.float32) / depth_scale

    return depth_img

def print_gripper_state(joint_group, gripper_state):
    """
    Print gripper state

    Parameters:
        joint_group (JointGroup): JointGroup enum
        gripper_state (object): Contains timestamp_ns, width, velocity, effort, is_moving
    """
    print(f"Timestamp (ns): {gripper_state.timestamp_ns}")
    print(
        f"width {gripper_state.width} "
        f"velocity {gripper_state.velocity} "
        f"effort {gripper_state.effort} "
        f"is moving {gripper_state.is_moving}"
    )

def get_navigation_pose(object_goal_pose: Sequence[float], motion: GalbotMotion, arm: str = "left_arm"):
    """
    Get navigation target pose

    Parameters:
        object_goal_pose (Sequence[float]): Target pose [x, y, z, qx, qy, qz, qw]
        motion (GalbotMotion): Motion control instance
        arm (str, optional): End effector name. Defaults to "left_arm".

    Returns:
        Sequence[float]: Navigation target pose [x, y, z, qx, qy, qz, qw]
    """
    assert arm in ["left_arm", "right_arm"], "arm must be left_arm or right_arm"

    try:
        status, ee_pose_in_base = motion.get_end_effector_pose(
            end_effector_frame=f"{arm}_end_effector_mount_link",
            reference_frame="base_link"
        )
        if status != gm.MotionStatus.SUCCESS:
            print(f"Failed to get end effector pose: status={status}")
            offset_y = ee_pose_in_base[1]
        else:
            print(f"Successfully got end effector pose: pose={ee_pose_in_base}")
            offset_y = 0.3

        # Set chassis pose to the same z coordinate as the target pose
        base_goal_pose_mat = np.eye(4)
        base_goal_pose_mat[:3, :3] = R.from_quat(object_goal_pose[3:]).as_matrix()
        base_goal_pose_mat[:3, 3] = np.array([object_goal_pose[0], object_goal_pose[1], 0])

        # According to the relative position of the chassis and camera, move the camera navigation target 0.6m backward in the local coordinate to leave observation space for the camera
        base_goal_pose_mat = base_goal_pose_mat @ np.array([[1,0,0,-0.6],[0,1,0,-offset_y],[0,0,1,0],[0,0,0,1]])
        print(base_goal_pose_mat)

        base_goal_pose_quat = R.from_matrix(base_goal_pose_mat[:3, :3]).as_quat()
        base_goal_pose_pos = base_goal_pose_mat[:3, 3]

        return base_goal_pose_pos.tolist() + base_goal_pose_quat.tolist()
    
    except Exception as e:
        print("Failed to get navigation target pose:", e)
    

def navigation_to_goal(nav: GalbotNavigation, goal_pose: Sequence[float], retry_cnt: int = 3):
    """
    Navigate to target pose

    Parameters:
        nav (GalbotNavigation): Navigation instance
        goal_pose (Sequence[float]): Target pose [x, y, z, qx, qy, qz, qw]
        retry_cnt (int, optional): Number of retries. Defaults to 3.
    """
    try:
        cur_pose = nav.get_current_pose()
        print(f"Current pose: {cur_pose}")
        if nav.check_path_reachability(goal_pose, cur_pose):
            retry_cnt = 3
            while True:
                status = nav.navigate_to_goal(goal_pose, enable_collision_check=True, is_blocking=True, timeout=20)
                time.sleep(0.5)
                retry_cnt -= 1
                if nav.check_goal_arrival() or retry_cnt < 0:
                    break
                else:
                    print(f"Navigation failed: status={status}, retrying: {retry_cnt}")
            print("navigate_to_goal return status:", status)
            print("Has arrived:", nav.check_goal_arrival())
        else:
            print("Path unreachable or unsafe")
    except Exception as e:
        print(f"Exception occurred during navigation: {e}")

def lift_camera_up(motion: GalbotMotion, target_pose: Sequence[float], target_chain: str, reference_frame: str):
    """
    Lift camera to target height

    Parameters:
        motion (GalbotMotion): Motion control instance
        target_pose (Sequence[float]): Target pose [x, y, z, qx, qy, qz, qw]
        target_chain (str): End effector name
        reference_frame (str): Reference frame
    """
    try:
        retry_cnt = 3
        while True:
            status, cur_ee_pose = motion.get_end_effector_pose_on_chain(
                chain_name=target_chain,
                frame_id="EndEffector",
                reference_frame=reference_frame
            )
            time.sleep(0.5)
            retry_cnt -= 1

            if status == gm.MotionStatus.SUCCESS or retry_cnt < 0:
                print(f"Current end effector pose: {cur_ee_pose}")
                break
            else:
                print(f"Failed to get end effector pose: status={status}, retrying: {retry_cnt}")
        
        tgt_ee_pose = cur_ee_pose.copy()
        tgt_ee_pose[2] = target_pose[2] - 0.1
        print(f"Target end effector pose: {tgt_ee_pose}")

        retry_cnt = 3
        while True:
            status = motion.set_end_effector_pose(
                target_pose=tgt_ee_pose,
                end_effector_frame=target_chain,
                reference_frame=reference_frame,
                enable_collision_check=False,
                is_blocking=True,
                timeout=5.0,
                params=gm.Parameter()
            )
            time.sleep(0.5)
            retry_cnt -= 1
            if status == gm.MotionStatus.SUCCESS or retry_cnt < 0:
                print(f"✅ Successfully set end effector pose: status={status}")
                break
            else:
                print(f"Failed to set end effector pose: status={status}, retrying: {retry_cnt}")
    except Exception as e:
        print(f"❌ Failed to lift camera: {e}")

def detect_target(img: np.ndarray, depth_img: np.ndarray) -> Sequence[float]:
    """
    Detection target function. Input RGB image and depth image, output target pose.

    Parameters:
        img (np.ndarray): RGB image
        depth_img (np.ndarray): Depth image

    Returns:
        Sequence[float]: Target pose [x, y, z, qx, qy, qz, qw]
    """
    try:
        
        ############### NOTE ###############
        # This function is a placeholder. In a real-world scenario, you would implement
        # target detection using computer vision techniques. For this example, we assume
        # a default pose.
        ####################################

        # Assume detected target pose is [-0.05, -0.1, 0.12, 0.0, 0.0, 0.0, 1.0]
        # Indicates the target is 0.12m in front of the camera, 0.05m to the left, 0.1m in height, facing the default camera direction
        default_pose = [-0.05, -0.1, 0.12, 0.0, 0.0, 0.0, 1.0]
        
        return default_pose
    except Exception as e:
        print(f"Target detection exception: {e}")
        return None

def pose_camera_to_base(robot: GalbotRobot, pose_camera: Sequence[float]) -> Sequence[float]:
    """
    Transform camera pose to chassis coordinate system

    Parameters:
        robot (GalbotRobot): Robot instance
        pose_camera (Sequence[float]): Camera pose [x, y, z, qx, qy, qz, qw]

    Returns:
        Sequence[float]: Chassis pose [x, y, z, qx, qy, qz, qw]
    """
    source_frame="left_arm_camera_color_optical_frame"
    target_frame="base_link"
    base_to_cam = robot.get_transform(target_frame, source_frame)[0]
    if base_to_cam is None:
        print("Failed to get transform from camera to chassis")
        return None
    else:
        print("base_to_cam: ", base_to_cam)

    base_to_cam_mat = np.eye(4)
    base_to_cam_mat[:3, :3] = R.from_quat(base_to_cam[3:]).as_matrix()
    base_to_cam_mat[:3, 3] = np.array(base_to_cam[:3])
    
    pose_camera_mat = np.eye(4)
    pose_camera_mat[:3, :3] = R.from_quat(pose_camera[3:]).as_matrix()
    pose_camera_mat[:3, 3] = np.array(pose_camera[:3])
    
    pose_base_mat = base_to_cam_mat @ pose_camera_mat[:, 3:]
    print("pose_base_mat: ", pose_base_mat)
    
    return pose_base_mat.flatten()[:3].tolist() + [0, 0, 0, 1]

def detect_object(robot: GalbotRobot, arm: str = "left_arm"):
    try:
        # Get camera image data
        if arm == "left_arm":
            rgb_image_data = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA)
            depth_data = robot.get_depth_data(SensorType.LEFT_ARM_DEPTH_CAMERA)
        elif arm == "right_arm":
            rgb_image_data = robot.get_rgb_data(SensorType.RIGHT_ARM_CAMERA)
            depth_data = robot.get_depth_data(SensorType.RIGHT_ARM_DEPTH_CAMERA)
        else:
            raise ValueError("arm must be left_arm or right_arm")

        # Decode image data
        if not rgb_image_data:
            print("No rgb image data!")
        else:
            print("get rgb image suceess")
            img = decode_compressed_image(rgb_image_data)
        
        if not depth_data:
            print("No depth_data!")
        else:
            depth_img = decode_compressed_image(depth_data)
            print("get depth data suceess")

        # Detect target
        object_pose_camera = detect_target(img, depth_img)
        if object_pose_camera is None:
            print("Target detection failed")
            return None
        else:
            print(f"object_pose_camera: {object_pose_camera}")
        
        # Calculate target pose in chassis coordinate system
        object_pose_base = pose_camera_to_base(robot, object_pose_camera)
        print(f"Target pose in chassis coordinate system: {object_pose_base}")
    
    except Exception as e:
        print(f"Target detection exception: {e}")

    return object_pose_base

def check_robot_safety():
    """Check if robot is safe"""
    # Prompt important notes
    print("⚠️  Note: 1. Please ensure the emergency stop button of the robot is released; 2. Please ensure there are no obstructions around the robot to avoid unexpected situations. 3. Please ensure the area around the robot is clear of obstacles.")
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

def pick_and_place(robot: GalbotRobot, 
                   nav: GalbotNavigation, 
                   motion: GalbotMotion, 
                   object_pose_base: Sequence[float], 
                   target_chain: str, 
                   reference_frame: str):
    try:
        # Open left gripper
        # Set left gripper width to 0.1m, speed to 0.05m, force to 10N, will block until gripper reaches position
        status = robot.set_gripper_command(
            JointGroup.LEFT_GRIPPER, 0.1, 0.05, 10, True
        )
        time.sleep(0.5)

        print("object_pose_base: ", object_pose_base)
        
        # Reach to target position
        retry_cnt = 3
        while True:
            status = motion.set_end_effector_pose(
                target_pose=object_pose_base,
                end_effector_frame=target_chain,
                reference_frame=reference_frame,
                enable_collision_check=False,
                is_blocking=True,
                timeout=5.0,
                params=gm.Parameter()
            )
            time.sleep(1)
            retry_cnt -= 1
            if status == gm.MotionStatus.SUCCESS or retry_cnt < 0:
                break
            else:
                print(f"Failed to set end effector pose: status={status}, retry count: {retry_cnt}")
        
        assert status == gm.MotionStatus.SUCCESS, "Failed to set end effector pose"
        print(f"✅ Successfully set end effector pose: status={status}")

        # Close gripper to grasp object
        status = robot.set_gripper_command(
            JointGroup.LEFT_GRIPPER, 0.02, 0.05, 10, True
        )
        time.sleep(0.5)

        # Return to initial position
        navigation_to_goal(nav, [0, 0, 0, 0, 0, 0, 1])
        time.sleep(2)

        # Release target
        status = robot.set_gripper_command(
            JointGroup.LEFT_GRIPPER, 0.1, 0.05, 10, True
        )
        time.sleep(0.5)
    except Exception as e:
        print(f"Exception occurred during pick_and_place: {e}")
        return None


def main():
    check_robot_safety()
    try:
        # Get robot instance
        robot = GalbotRobot.get_instance()
        # Get GalbotMotion instance
        motion = GalbotMotion.get_instance()
        # Get navigation instance
        nav = GalbotNavigation.get_instance()

        # Get RGB and depth images from left arm, depth images from right arm, 
        # base LiDAR data, and torso IMU data
        enable_sensor_set = {SensorType.LEFT_ARM_CAMERA, # Left arm depth camera
                            SensorType.LEFT_ARM_DEPTH_CAMERA, # Left arm RGB camera
                            SensorType.BASE_LIDAR, # Base LiDAR
                            SensorType.TORSO_IMU} # Torso IMU sensor

        # Initialize robot
        if robot.init(enable_sensor_set):
            print("GalbotRobot initialization successful")
        else:
            print("GalbotRobot initialization failed")
        if motion.init():
            print("GalbotMotion initialization successful")
        else:
            print("GalbotMotion initialization failed")
        if nav.init():
            print("GalbotNavigation initialization successful")
        else:
            print("GalbotNavigation initialization failed")

        # Program starts immediately, wait for data readiness
        time.sleep(1)

        # Calculate navigation target pose
        object_goal_pose = [-1, 0.33, 0.90, 0, 0, 1, 0]
        base_goal_pose = [0, 0, 0, 0, 0, 0, 1]
        base_goal_pose = get_navigation_pose(object_goal_pose, motion)
        
        # Navigate to target pose
        navigation_to_goal(nav, base_goal_pose)
        
        target_chain = "left_arm"
        reference_frame = "base_link"

        # Get current end effector pose for subsequent restoration
        try:
            status, original_pose = motion.get_end_effector_pose_on_chain(
                chain_name=target_chain,
                frame_id="EndEffector",
                reference_frame=reference_frame
            )
            assert status == gm.MotionStatus.SUCCESS, "Failed to get end effector pose"
            print(f"✅ Successfully got {target_chain} end effector pose: {original_pose}")
            time.sleep(1)
        except Exception as e:
            print(f"❌ {target_chain} end effector pose exception: {e}")

        # Lift camera
        lift_camera_up(motion, object_goal_pose, "left_arm", "base_link")

        # Detect target
        object_pose_base = detect_object(robot, arm="left_arm")
        if object_pose_base is None:
            print("Target detection failed")
            return None
        else:
            print(f"Detected target pose: {object_pose_base}")
        
        # Grasp and return to initial position
        pick_and_place(robot, nav, motion, object_pose_base, "left_arm", "base_link")

        # After grasping target, restore posture
        try:
            time.sleep(2)
            status = motion.set_end_effector_pose(
                target_pose=original_pose,
                end_effector_frame=target_chain,
                reference_frame=reference_frame,
                enable_collision_check=False,
                is_blocking=True,
                timeout=5.0,
                params=gm.Parameter()
            )
            assert status == gm.MotionStatus.SUCCESS, "Failed to set end effector pose"
            print(f"✅ Successfully set end effector pose: status={status}")
            time.sleep(1)
        except Exception as e:
            print(f"❌ {target_chain} end effector pose exception: {e}")
            
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        # Actively send SIGINT exit signal
        robot.request_shutdown()
        # Wait to enter shutdown state
        robot.wait_for_shutdown()
        # Release SDK resources
        robot.destroy()
        print('Resource release successful')

if __name__ == "__main__":
    main()