"""
This tutorial demonstrates a complete pick-and-place task for a standard application scenario.
It integrates navigation, perception, motion planning, and robot control, and can be used as a
reference implementation for standard applications.
Before the task starts, the robot moves to the known 21-joint initial pose defined by this example.
After picking, the robot navigates to its current map pose with the X coordinate offset by 0.1 m,
then performs placement.
The taught grasp approach and place descent use `move_line`; other end-effector motions use
`set_end_effector_pose`. All Cartesian poses below were taught for the bare left-arm
`EndEffector` frame relative to `base_link` on G3.
"""

try:
    import galbot_sdk.g3 as gm
    from galbot_sdk.g3 import GalbotNavigation
    from galbot_sdk.g3 import GalbotRobot
    from galbot_sdk.g3 import GalbotMotion
    from galbot_sdk.g3 import G3JointGroup, SensorType
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
# Keep each planned arm segment slow enough to observe, and pause at task
# waypoints so that the pick-and-place sequence is easy to follow.
ARM_MIN_MOVE_TIME_S = 3.0
ARM_MOVE_TIMEOUT_S = 30.0
ARM_JOINT_SPEED_RAD_S = 0.1
WAYPOINT_HOLD_S = 1.0
GRIPPER_SETTLE_S = 0.5
PRE_GRASP_POSE = [
    0.43645796, 0.18778193, 0.92196164, -0.01950938,
    0.02598107, -0.05602237, 0.99790073,
]
GRASP_POSE = [
    0.51412043, 0.18056627, 0.96286294, -0.02264866,
    0.00051349, -0.05636788, 0.99815301,
]
LIFT_GRASP_POSE = [
    0.53047695, 0.15313211, 1.09918264, -0.00801662,
    -0.19539735, -0.09980387, 0.97559971,
]
RETREAT_POSE = [
    0.43381901, 0.16343561, 1.0412931, -0.02825337,
    -0.11813149, -0.09585053, 0.98795717,
]
PRE_PLACE_POSE = [
    0.50956077, 0.20189686, 1.05753193, -0.02514585,
    -0.11590543, -0.04433032, 0.99195183,
]
PLACE_POSE = [
    0.49092053, 0.19268637, 0.97338715, -0.02437315,
    -0.01105621, -0.05520225, 0.99811644,
]
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
    0.00001198, 0.0,
    1.50008953, -1.3599937, -0.45010296, 1.53000093,
    -0.09994802, -0.41997507, 0.00004712,
    -1.49992013, 1.3599937, 0.44991097, -1.5300498,
    0.10004402, 0.4200955, 0.00004887,
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
        speed_rad_s=ARM_JOINT_SPEED_RAD_S,
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


def decode_compressed_image(compressed_image):
    """
    decode CompressedImage image

    Parameters:
        compressed_image (dict): image dict, keys: [header, format, data]

    Returns:
        numpy.ndarray: decoded image
    """
    image_data = compressed_image["data"]
    if compressed_image["format"].lower() in ("jpeg", "jpg", "rgb8"):
        return decode_rgb_image(image_data)
    raise ValueError(f"Unsupported data format: {compressed_image['format']}")


def decode_rgb_image(image_data):
    """decode rgb image"""
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Fail to Decode RGB Image")
    return img


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
    # The G3 TF tree publishes the arm RGB camera under this frame name.
    source_frame = "left_arm_camera"
    target_frame = "base_link"
    base_to_cam = robot.get_transform(target_frame, source_frame)[0]
    if not base_to_cam or len(base_to_cam) != 7:
        raise RuntimeError(
            f"Failed to get a valid transform from {source_frame} to {target_frame}"
        )
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
        # G3 uses the left-arm RGB camera. The pose below is a placeholder for
        # an application-specific perception result.
        rgb_image_data = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA)

        # Decode image data
        if not rgb_image_data:
            print("No rgb image data!")
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        else:
            print("get rgb image suceess")
            decode_compressed_image(rgb_image_data)

        # Placeholder perception result in the OpenCV camera frame (x right, y down, z forward).
        # Replace this pose with the output of an application-specific detector.
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
        # attach_tool() only loads a planning model; it does not detect hardware.
        # Do not add a virtual gripper when no physical gripper feedback exists.
        left_gripper_state = robot.get_gripper_state(G3JointGroup.left_gripper)
        has_left_gripper = left_gripper_state is not None
        if has_left_gripper:
            # A gripper-less G3 can still publish placeholder feedback. Confirm
            # hardware availability with the blocking open command that the
            # pick task needs anyway, before changing the planning model.
            gripper_status = robot.set_gripper_command(
                G3JointGroup.left_gripper, 0.1, 0.05, 10, True
            )
            if gripper_status in {
                gm.ControlStatus.DATA_FETCH_FAILED,
                gm.ControlStatus.TIMEOUT,
            }:
                has_left_gripper = False
                print(
                    "⚠️ Left gripper feedback is only a placeholder "
                    f"({gripper_status})."
                )
            elif gripper_status != gm.ControlStatus.SUCCESS:
                print(f"❌ Failed to open left gripper: status={gripper_status}")
                return False
        if has_left_gripper:
            status = motion.attach_tool(chain="left_arm", tool="galbot_gripper")
            time.sleep(0.5)
            if status != gm.MotionStatus.SUCCESS:
                print(f"❌ Failed to attach tool: status={status}")
                return False
            print(f"✅ Successfully attached tool: status={status}")
        else:
            print(
                "⚠️ Left gripper was not detected; skipping its tool model "
                "and gripper commands."
            )
            # Clear a model that may have been left attached by an earlier
            # interrupted/failed run of this example.
            stale_tool_status = motion.detach_tool(chain="left_arm")
            if stale_tool_status == gm.MotionStatus.SUCCESS:
                print("Cleared a previously attached left-arm tool model.")

        # These poses were taught for the bare arm EndEffector frame, not the tool TCP.
        motion_params = gm.Parameter()
        motion_params.set_direct_execute(True)
        motion_params.set_blocking(True)
        motion_params.set_timeout(ARM_MOVE_TIMEOUT_S)
        motion_params.set_tool_pose(False)
        motion_params.set_check_collision(True)
        motion_params.set_reference_frame("base_link")
        motion_params.set_actuate("with_chain_only")
        if has_left_gripper:
            print(
                "✅ Successfully set left gripper width to 0.1m: "
                f"status={gripper_status}"
            )

        # These G3 poses were recorded by drag teaching in the base_link frame.
        # Perception remains informational until an application-specific grasp-pose
        # generator replaces the taught trajectory.
        print(f"Perception pose: {object_pose_base}")
        pre_grasp_pose = list(PRE_GRASP_POSE)
        grasp_pose = list(GRASP_POSE)
        lift_grasp_pose = list(LIFT_GRASP_POSE)
        retreat_pose = list(RETREAT_POSE)

        print(f"pre_grasp_pose: {pre_grasp_pose}")
        status = motion.set_end_effector_pose(
            target_pose=pre_grasp_pose,
            end_effector_frame="left_arm",
            reference_frame="base_link",
            enable_collision_check=True,
            is_blocking=True,
            timeout=ARM_MOVE_TIMEOUT_S,
            params=motion_params,
        )
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to execute pre_grasp pose command: status={status}")
            return False
        print(f"✅ Successfully executed pre_grasp pose command: {pre_grasp_pose}")
        print(f"Holding at pre_grasp for {WAYPOINT_HOLD_S:.1f}s...")
        time.sleep(WAYPOINT_HOLD_S)

        # Use move_line for the taught final approach to the object.
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
        print(f"Holding at grasp for {WAYPOINT_HOLD_S:.1f}s...")
        time.sleep(WAYPOINT_HOLD_S)

        if has_left_gripper:
            gripper_status = robot.set_gripper_command(
                G3JointGroup.left_gripper, 0.02, 0.05, 10, True
            )
            if gripper_status != gm.ControlStatus.SUCCESS:
                print(f"❌ Failed to close left gripper: status={gripper_status}")
                return False
            print(f"✅ Successfully closed left gripper: status={gripper_status}")
            time.sleep(GRIPPER_SETTLE_S)

        print(f"lift_grasp_pose: {lift_grasp_pose}")
        status = motion.set_end_effector_pose(
            target_pose=lift_grasp_pose,
            end_effector_frame="left_arm",
            reference_frame="base_link",
            enable_collision_check=True,
            is_blocking=True,
            timeout=ARM_MOVE_TIMEOUT_S,
            params=motion_params,
        )
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to execute lift_grasp pose command: status={status}")
            return False
        print(f"✅ Successfully executed lift_grasp pose command: {lift_grasp_pose}")
        print(f"Holding at lift_grasp for {WAYPOINT_HOLD_S:.1f}s...")
        time.sleep(WAYPOINT_HOLD_S)

        print(f"retreat_pose: {retreat_pose}")
        status = motion.set_end_effector_pose(
            target_pose=retreat_pose,
            end_effector_frame="left_arm",
            reference_frame="base_link",
            enable_collision_check=True,
            is_blocking=True,
            timeout=ARM_MOVE_TIMEOUT_S,
            params=motion_params,
        )
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to execute retreat pose command: status={status}")
            return False
        print(f"✅ Successfully executed retreat pose command: {retreat_pose}")
        print(f"Holding at retreat for {WAYPOINT_HOLD_S:.1f}s...")
        time.sleep(WAYPOINT_HOLD_S)

        # Return to the initial left-arm joint pose before navigation.
        left_arm_status = robot.set_joint_positions(
            joint_positions=INITIAL_UPPER_BODY_JOINT_POSITIONS[2:9],
            joint_names=INITIAL_UPPER_BODY_JOINT_NAMES[2:9],
            is_blocking=True,
            speed_rad_s=ARM_JOINT_SPEED_RAD_S,
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

        # Move to the taught pre-place pose, then follow the taught placement approach.
        pre_place_pose = list(PRE_PLACE_POSE)
        place_pose = list(PLACE_POSE)
        print(f"pre_place_pose: {pre_place_pose}")
        status = motion.set_end_effector_pose(
            target_pose=pre_place_pose,
            end_effector_frame="left_arm",
            reference_frame="base_link",
            enable_collision_check=True,
            is_blocking=True,
            timeout=ARM_MOVE_TIMEOUT_S,
            params=motion_params,
        )
        if status != gm.MotionStatus.SUCCESS:
            print(f"❌ Failed to execute pre_place pose command: status={status}")
            return False
        print(f"✅ Successfully executed pre_place pose command: {pre_place_pose}")
        print(f"Holding at pre_place for {WAYPOINT_HOLD_S:.1f}s...")
        time.sleep(WAYPOINT_HOLD_S)

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
        print(f"Holding at place for {WAYPOINT_HOLD_S:.1f}s...")
        time.sleep(WAYPOINT_HOLD_S)

        if has_left_gripper:
            gripper_status = robot.set_gripper_command(
                G3JointGroup.left_gripper, 0.1, 0.05, 10, True
            )
            if gripper_status != gm.ControlStatus.SUCCESS:
                print(f"❌ Failed to release left gripper: status={gripper_status}")
                return False
            print(f"✅ Successfully released left gripper: status={gripper_status}")
            time.sleep(GRIPPER_SETTLE_S)

        # Return the left arm to its initial joint pose after placing the object.
        left_arm_status = robot.set_joint_positions(
            joint_positions=INITIAL_UPPER_BODY_JOINT_POSITIONS[2:9],
            joint_names=INITIAL_UPPER_BODY_JOINT_NAMES[2:9],
            is_blocking=True,
            speed_rad_s=ARM_JOINT_SPEED_RAD_S,
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

        if has_left_gripper:
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
    original_motion_config = None
    slow_motion_config_applied = False
    try:
        # Get robot instance
        robot = GalbotRobot()
        # Get GalbotMotion instance
        motion = GalbotMotion()
        # Get navigation instance
        nav = GalbotNavigation()

        # G3 does not have arm-mounted depth cameras.
        enable_sensor_set = {SensorType.LEFT_ARM_CAMERA}

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

        # Cartesian motion duration is configured globally by MPS rather than
        # per Parameter in this SDK version. Save and restore the original
        # configuration so this tutorial does not affect later applications.
        config_status, original_motion_config = motion.get_motion_plan_config()
        if config_status != gm.MotionStatus.SUCCESS:
            raise RuntimeError(
                f"Failed to read motion-plan configuration: {config_status}"
            )
        slow_motion_config = gm.MotionPlanConfig()
        trajectory_config = slow_motion_config.create_trajectory_plan_config()
        trajectory_config.set_min_move_time(ARM_MIN_MOVE_TIME_S)
        trajectory_config.set_way_point_plan_expected_time(ARM_MIN_MOVE_TIME_S)
        config_status = motion.set_motion_plan_config(slow_motion_config)
        if config_status != gm.MotionStatus.SUCCESS:
            raise RuntimeError(
                f"Failed to apply slow motion-plan configuration: {config_status}"
            )
        slow_motion_config_applied = True

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
        if slow_motion_config_applied and original_motion_config is not None:
            restore_status = motion.set_motion_plan_config(original_motion_config)
            if restore_status != gm.MotionStatus.SUCCESS:
                print(
                    "⚠️ Failed to restore the original motion-plan configuration: "
                    f"status={restore_status}"
                )
        # Actively send SIGINT exit signal
        robot.request_shutdown()
        # Wait to enter shutdown state
        robot.wait_for_shutdown()
        # Release SDK resources
        robot.destroy()
        print("Resource release successful")


if __name__ == "__main__":
    main()
