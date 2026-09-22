"""
This tutorial demonstrates a complete pick-and-place task for a standard application scenario.
It integrates navigation, perception, motion planning, and robot control, and can be used as a
reference implementation for standard applications.
Before the task starts, the robot moves to the known 21-joint initial pose defined by this example.
After picking, the robot navigates to its current map pose with the X coordinate offset by 0.1 m,
then performs placement.
The final grasp approach and place descent use `move_line`; other end-effector motions use
`set_end_effector_pose`.
"""

try:
    import galbot_sdk.g1 as gm
    from galbot_sdk.g1 import GalbotNavigation
    from galbot_sdk.g1 import GalbotRobot
    from galbot_sdk.g1 import GalbotMotion
    from galbot_sdk.g1 import G1JointGroup, SensorType
except ImportError:
    print(
        "Import galbot_sdk failed, please install it first or check if it is in the PYTHONPATH"
    )
    exit(1)

import os

try:
    import numpy as np
except ImportError:
    os.system("pip install numpy")
    import numpy as np

try:
    import cv2
except ImportError:
    os.system("pip install opencv-python")
    import cv2

import time
from typing import Sequence

PLACE_NAVIGATION_X_OFFSET_M = 0.1
GRASP_TRANSITION_OFFSET_M = 0.05
PLACE_POSE_BASE = [0.4, 0.3, 0.7, 0.0, 0.0, 0.0, 1.0]
# Placeholder grasp pose for this runnable tutorial. Replace it with the
# base-frame pose produced by the application's perception module.
GRASP_POSE_BASE_PLACEHOLDER = [0.4, 0.3, 0.65, 0.0, 0.0, 0.0, 1.0]
INITIAL_LEG_JOINT_NAMES = [
    "leg_joint1",
    "leg_joint2",
    "leg_joint3",
    "leg_joint4",
    "leg_joint5",
]
INITIAL_LEG_JOINT_POSITIONS = [0.4, 1.2, 0.8, 0.0, 0.0]

INITIAL_UPPER_BODY_JOINT_NAMES = [
    "head_joint1",
    "head_joint2",
    "left_arm_joint1",
    "left_arm_joint2",
    "left_arm_joint3",
    "left_arm_joint4",
    "left_arm_joint5",
    "left_arm_joint6",
    "left_arm_joint7",
    "right_arm_joint1",
    "right_arm_joint2",
    "right_arm_joint3",
    "right_arm_joint4",
    "right_arm_joint5",
    "right_arm_joint6",
    "right_arm_joint7",
]
INITIAL_UPPER_BODY_JOINT_POSITIONS = [
    0.0,
    0.0,
    1.90,
    -1.46,
    -0.54,
    -1.96,
    0.0,
    -0.4,
    0.0,
    -1.90,
    1.46,
    0.54,
    1.96,
    0.0,
    0.4,
    0.0,
]


def move_to_initial_pose(robot: GalbotRobot) -> bool:
    """Move the legs first, then move the head and both arms together."""
    leg_status = robot.set_joint_positions(
        joint_positions=INITIAL_LEG_JOINT_POSITIONS,
        joint_names=INITIAL_LEG_JOINT_NAMES,
        is_blocking=True,
        speed_rad_s=0.2,
        timeout_s=20.0,
    )
    if leg_status != gm.ControlStatus.SUCCESS:
        print(f"❌ Failed to move the legs to the initial pose: status={leg_status}")
        return False
    print(f"✅ Successfully moved the legs to the initial pose: status={leg_status}")

    upper_body_status = robot.set_joint_positions(
        joint_positions=INITIAL_UPPER_BODY_JOINT_POSITIONS,
        joint_names=INITIAL_UPPER_BODY_JOINT_NAMES,
        is_blocking=True,
        speed_rad_s=0.2,
        timeout_s=20.0,
    )
    if upper_body_status != gm.ControlStatus.SUCCESS:
        print(
            "❌ Failed to move the head and arms to the initial pose: "
            f"status={upper_body_status}"
        )
        return False

    print(
        "✅ Successfully moved the head and arms to the initial pose: "
        f"status={upper_body_status}"
    )
    return True


def decode_compressed_image(compressed_image, camera_info={}):
    """
    decode CompressedImage image

    Parameters:
        compressed_image (dict): image dict, keys:[header, format, data, "depth_scale"]

    Returns:
        numpy.ndarray: decoded image
    """
    image_data = compressed_image["data"]
    if compressed_image["format"] == "jpeg":
        return decode_rgb_image(image_data)
    elif compressed_image["format"] == "16UC1":
        return decode_depth_image(
            image_data, compressed_image["depth_scale"], camera_info
        )
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


def navigation_to_goal(
    nav: GalbotNavigation, goal_pose: Sequence[float], retry_cnt: int = 3
):
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
                status = nav.navigate_to_goal(
                    goal_pose, enable_collision_check=True, is_blocking=True, timeout=20
                )
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


def quaternion_to_rotation_matrix(quaternion: Sequence[float]) -> np.ndarray:
    """Convert an [x, y, z, w] quaternion to a 3x3 rotation matrix."""
    quaternion_array = np.asarray(quaternion, dtype=np.float64)
    if quaternion_array.shape != (4,):
        raise ValueError("Quaternion must contain exactly four values")

    norm = np.linalg.norm(quaternion_array)
    if norm < np.finfo(np.float64).eps:
        raise ValueError("Quaternion norm must be greater than zero")

    x, y, z, w = quaternion_array / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def pose_camera_to_base(
    robot: GalbotRobot, pose_camera: Sequence[float]
) -> Sequence[float]:
    """
    Transform camera pose to chassis coordinate system

    Parameters:
        robot (GalbotRobot): Robot instance
        pose_camera (Sequence[float]): Camera pose [x, y, z, qx, qy, qz, qw]

    Returns:
        Sequence[float]: Chassis pose [x, y, z, qx, qy, qz, qw]
    """
    source_frame = "left_arm_camera_color_optical_frame"
    target_frame = "base_link"
    base_to_cam = robot.get_transform(target_frame, source_frame)[0]
    if base_to_cam is None:
        print("Failed to get transform from camera to chassis")
        return None
    else:
        print("base_to_cam: ", base_to_cam)

    base_to_cam_mat = np.eye(4)
    base_to_cam_mat[:3, :3] = quaternion_to_rotation_matrix(base_to_cam[3:])
    base_to_cam_mat[:3, 3] = np.array(base_to_cam[:3])

    pose_base_mat = (
        base_to_cam_mat[:3, :3] @ np.array(pose_camera[:3]).reshape(3, 1)
        + base_to_cam_mat[:3, 3:]
    )

    return pose_base_mat.flatten()[:3].tolist() + [0, 0, 0, 1]


def detect_object(robot: GalbotRobot):
    try:
        # This example uses the left-arm RGB and depth cameras.
        rgb_image_data = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA)
        depth_data = robot.get_depth_data(SensorType.LEFT_ARM_DEPTH_CAMERA)

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

        # Placeholder perception result in the OpenCV camera frame (x right, y down, z forward).
        # Replace this pose with the output of an application-specific RGB-D detector.
        object_pose_camera = [0.0, 0.20, 0.29, 0.0, 0.71, 0.0, 0.71]
        print(f"object_pose_camera: {object_pose_camera}")

        # Calculate target pose in chassis coordinate system
        object_pose_base = pose_camera_to_base(robot, object_pose_camera)
        print(f"Target pose in chassis coordinate system: {object_pose_base}")

    except Exception as e:
        object_pose_base = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        print(f"Target detection exception: {e}")

    return object_pose_base


def check_robot_safety():
    """Check if robot is safe"""
    # Prompt important notes
    print(
        "⚠️  Note: 1. Please ensure the emergency stop button of the robot is released; 2. Please ensure there are no obstructions around the robot to avoid unexpected situations. 3. Please ensure the area around the robot is clear of obstacles."
    )
    while True:
        key = input(
            "Please confirm that the robot's emergency stop button is released and there are no obstructions, continue? (y/n)..."
        )
        if key == "y":
            print("User confirmed, continuing...")
            break
        elif key == "n":
            print("User did not confirm, exiting program...")
            exit(1)
        else:
            print("Invalid input, please enter 'y' or 'n'")


def pick_and_place(
    robot: GalbotRobot,
    nav: GalbotNavigation,
    motion: GalbotMotion,
    object_pose_base: Sequence[float],
    navigation_enabled: bool,
):
    try:
        # Attach tool to end effector
        status = motion.attach_tool(chain="left_arm", tool="galbot_gripper")
        time.sleep(0.5)
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to attach tool: status={status}")
            return False
        else:
            print(f"✅ Successfully attached tool: status={status}")

        # The target poses represent the attached gripper TCP, not the bare arm flange.
        # Tool pose mode makes set_end_effector_pose control the attached tool TCP.
        motion_params = gm.Parameter()
        motion_params.set_direct_execute(True)
        motion_params.set_blocking(True)
        motion_params.set_timeout(20.0)
        motion_params.set_tool_pose(True)
        motion_params.set_check_collision(True)
        motion_params.set_reference_frame("base_link")
        motion_params.set_actuate("with_chain_only")

        # Open left gripper
        # Set left gripper width to 0.1m, speed to 0.05m, force to 10N, will block until gripper reaches position
        status = robot.set_gripper_command(
            G1JointGroup.left_gripper, 0.1, 0.05, 10, False
        )
        time.sleep(0.5)
        print(f"✅ Successfully set left gripper width to 0.1m: status={status}")

        # This fixed grasp pose is only a placeholder for the runnable tutorial.
        # A real application should use object_pose_base from perception instead.
        print(f"Perception pose: {object_pose_base}")
        grasp_pose = list(GRASP_POSE_BASE_PLACEHOLDER)
        pre_grasp_pose = list(grasp_pose)
        pre_grasp_pose[0] -= GRASP_TRANSITION_OFFSET_M
        lift_grasp_pose = list(grasp_pose)
        lift_grasp_pose[2] += GRASP_TRANSITION_OFFSET_M
        retreat_pose = list(lift_grasp_pose)
        retreat_pose[0] -= GRASP_TRANSITION_OFFSET_M

        print(f"pre_grasp_pose: {pre_grasp_pose}")
        status = motion.set_end_effector_pose(
            target_pose=pre_grasp_pose,
            end_effector_frame="left_arm",
            reference_frame="base_link",
            enable_collision_check=True,
            is_blocking=True,
            timeout=20.0,
            params=motion_params,
        )
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to execute pre_grasp pose command: status={status}")
            return False
        print(f"✅ Successfully executed pre_grasp pose command: {pre_grasp_pose}")

        # Use move_line for the final 5 cm straight-line approach to the object.
        grasp_target = gm.MotionPlanChainTarget()
        grasp_target.chain_name = "left_arm"
        grasp_target.mode = gm.MotionPlanTargetMode.kCartesian
        grasp_target.cart.chain_name = "left_arm"
        grasp_target.cart.frame_id = "EndEffector"
        grasp_target.cart.reference_frame = "base_link"
        grasp_target.cart.pose = gm.Pose(grasp_pose)

        print(f"grasp_pose: {grasp_pose}")
        status, _ = motion.move_line([[grasp_target]], motion_params)
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to execute grasp move_line: status={status}")
            return False
        print(f"✅ Successfully executed grasp move_line: {grasp_pose}")

        # Close gripper to grasp object
        status = robot.set_gripper_command(
            G1JointGroup.left_gripper, 0.02, 0.05, 10, False
        )
        time.sleep(0.5)
        print(f"✅ Successfully closed left gripper: status={status}")

        print(f"lift_grasp_pose: {lift_grasp_pose}")
        status = motion.set_end_effector_pose(
            target_pose=lift_grasp_pose,
            end_effector_frame="left_arm",
            reference_frame="base_link",
            enable_collision_check=True,
            is_blocking=True,
            timeout=20.0,
            params=motion_params,
        )
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to execute lift_grasp pose command: status={status}")
            return False
        print(f"✅ Successfully executed lift_grasp pose command: {lift_grasp_pose}")

        print(f"retreat_pose: {retreat_pose}")
        status = motion.set_end_effector_pose(
            target_pose=retreat_pose,
            end_effector_frame="left_arm",
            reference_frame="base_link",
            enable_collision_check=True,
            is_blocking=True,
            timeout=20.0,
            params=motion_params,
        )
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to execute retreat pose command: status={status}")
            return False
        print(f"✅ Successfully executed retreat pose command: {retreat_pose}")

        # Return to the initial left-arm joint pose before navigation.
        left_arm_status = robot.set_joint_positions(
            joint_positions=INITIAL_UPPER_BODY_JOINT_POSITIONS[2:9],
            joint_names=INITIAL_UPPER_BODY_JOINT_NAMES[2:9],
            is_blocking=True,
            speed_rad_s=0.2,
            timeout_s=20.0,
        )
        if left_arm_status != gm.ControlStatus.SUCCESS:
            print(
                f"❌ Failed to restore the left-arm initial pose: status={left_arm_status}"
            )
            return False
        print(
            f"✅ Successfully restored the left-arm initial pose: status={left_arm_status}"
        )

        if navigation_enabled:
            place_navigation_goal = list(nav.get_current_pose())
            place_navigation_goal[0] += PLACE_NAVIGATION_X_OFFSET_M
            print(f"Place navigation target pose: {place_navigation_goal}")
            navigation_to_goal(nav, place_navigation_goal)
            time.sleep(2)
            print("✅ Navigation stage completed; starting Place")
        else:
            print(
                "⚠️ Navigation is unavailable; skipping navigation and continuing "
                "with Place at the current position."
            )

        # Move above the place point, then descend 5 cm in a straight line.
        place_pose = list(PLACE_POSE_BASE)
        pre_place_pose = list(place_pose)
        pre_place_pose[2] += GRASP_TRANSITION_OFFSET_M
        print(f"pre_place_pose: {pre_place_pose}")
        status = motion.set_end_effector_pose(
            target_pose=pre_place_pose,
            end_effector_frame="left_arm",
            reference_frame="base_link",
            enable_collision_check=True,
            is_blocking=True,
            timeout=20.0,
        )
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to execute pre_place pose command: status={status}")
            return False
        print(f"✅ Successfully executed pre_place pose command: {pre_place_pose}")

        place_target = gm.MotionPlanChainTarget()
        place_target.chain_name = "left_arm"
        place_target.mode = gm.MotionPlanTargetMode.kCartesian
        place_target.cart.chain_name = "left_arm"
        place_target.cart.frame_id = "EndEffector"
        place_target.cart.reference_frame = "base_link"
        place_target.cart.pose = gm.Pose(place_pose)

        print(f"place_pose: {place_pose}")
        status, _ = motion.move_line([[place_target]], motion_params)
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to execute place move_line: status={status}")
            return False
        print(f"✅ Successfully executed place move_line: {place_pose}")

        # Release target
        status = robot.set_gripper_command(
            G1JointGroup.left_gripper, 0.1, 0.05, 10, False
        )
        time.sleep(0.5)
        print(f"✅ Successfully released left gripper: status={status}")

        # Return the left arm to its initial joint pose after placing the object.
        left_arm_status = robot.set_joint_positions(
            joint_positions=INITIAL_UPPER_BODY_JOINT_POSITIONS[2:9],
            joint_names=INITIAL_UPPER_BODY_JOINT_NAMES[2:9],
            is_blocking=True,
            speed_rad_s=0.2,
            timeout_s=20.0,
        )
        if left_arm_status != gm.ControlStatus.SUCCESS:
            print(
                "❌ Failed to restore the left-arm pose after placing: "
                f"status={left_arm_status}"
            )
            return False
        print(
            "✅ Successfully restored the left-arm pose after placing: "
            f"status={left_arm_status}"
        )

        # Detach tool from end effector
        status = motion.detach_tool(chain="left_arm")
        time.sleep(0.5)
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to detach tool: status={status}")
        else:
            print(f"✅ Successfully detached tool: status={status}")
        return True
    except Exception as e:
        print(f"Exception occurred during pick_and_place: {e}")
        return False


def main():
    check_robot_safety()
    try:
        # Get robot instance
        robot = GalbotRobot()
        # Get GalbotMotion instance
        motion = GalbotMotion()
        # Get navigation instance
        nav = GalbotNavigation()

        # Enable only the RGB and depth cameras used for target detection.
        enable_sensor_set = {
            SensorType.LEFT_ARM_CAMERA,
            SensorType.LEFT_ARM_DEPTH_CAMERA,
        }

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

        if not move_to_initial_pose(robot):
            raise RuntimeError("Failed to move to the task initial pose")

        # Check localization once. If unavailable, Pick still runs and navigation
        # between Pick and Place is skipped.
        try:
            navigation_enabled = nav.is_localized()
        except Exception as e:
            navigation_enabled = False
            print(f"⚠️ Navigation status check failed: {e}")
        if navigation_enabled:
            print("✅ Robot is localized; Place navigation is enabled.")
        else:
            print(
                "⚠️ Robot is not localized; navigation will be skipped. "
                "This run demonstrates Pick and Place at the current position."
            )
        print()

        # Detect the target before Pick.
        object_pose_base = detect_object(robot)
        print()

        # Execute Pick, navigate, and then Place.
        if not pick_and_place(robot, nav, motion, object_pose_base, navigation_enabled):
            raise RuntimeError("Pick-and-place execution failed")
        print()

    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        # Actively send SIGINT exit signal
        robot.request_shutdown()
        # Wait to enter shutdown state
        robot.wait_for_shutdown()
        # Release SDK resources
        robot.destroy()
        print("Resource release successful")


if __name__ == "__main__":
    main()
