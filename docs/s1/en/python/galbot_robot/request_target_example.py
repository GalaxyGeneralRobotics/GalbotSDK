"""
S1 RequestTarget menu example for SDK mirror SingoriXTarget.

This example shows how to construct `SingoriXTarget` directly at the Python SDK
layer and send it to the low-level WBCS through `request_target()`.

1. Joint-space commands are written into `target_group_trajectory_map`
2. Chassis pose / twist style task-space commands are written into `target_task_trajectory_map`
3. One `SingoriXTarget` can contain both joint trajectory and task trajectory
4. The current SDK does not automatically switch the chassis controller, so
   pose / twist / mixed scenes explicitly call `switch_controller(...)`
5. The base twist scene automatically sends a zero-twist target after the
   configured duration to stop the chassis
"""

import math
import time

from galbot_sdk.s1 import (
    ControlStatus,
    ErrorInfo,
    FrameTriad,
    GalbotRobot,
    GroupCommand,
    JointCommand,
    Pose,
    S1ControllerName,
    S1JointGroup,
    SingoriXTarget,
    TargetConfig,
    TargetGroupTrajectory,
    TargetSampling,
    TargetTaskTrajectory,
    TaskCommand,
    Twist,
    Vector3,
    TARGET_DATA_DEFAULT,
    TARGET_DATA_FRAME_POSE,
    TARGET_DATA_FRAME_TWIST,
    TARGET_TYPE_OVERRIDE,
    TARGET_TYPE_PROVERRIDE,
)


CHASSIS_TASK_NAME = S1JointGroup.swerve_chassis
CHASSIS_SUBTASK_POSE = "swerve_chassis_pose"
CHASSIS_SUBTASK_TWIST = "swerve_chassis_twist"


def now_ns():
    return time.time_ns()


def yaw_to_quaternion(yaw):
    return [0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)]


def make_empty_target():
    target = SingoriXTarget()
    target.header.timestamp_ns = now_ns()
    target.header.frame_id = "base_link"
    return target


def make_group_target_config():
    config = TargetConfig()
    config.target_data = TARGET_DATA_DEFAULT
    config.target_type = TARGET_TYPE_PROVERRIDE
    config.target_sampling = TargetSampling.TARGET_SAMPLING_DEFAULT
    config.target_priority = 1
    return config


def make_pose_target_config():
    config = TargetConfig()
    config.target_data = TARGET_DATA_FRAME_POSE
    config.target_type = TARGET_TYPE_OVERRIDE
    config.target_sampling = TargetSampling.TARGET_SAMPLING_LINEAR_INTERPOLATE
    config.target_priority = 1
    return config


def make_twist_target_config():
    config = TargetConfig()
    config.target_data = TARGET_DATA_FRAME_TWIST
    config.target_type = TARGET_TYPE_OVERRIDE
    config.target_sampling = TargetSampling.TARGET_SAMPLING_DIRECT_PASS
    config.target_priority = 1
    return config


def make_joint_command(position):
    command = JointCommand()
    command.position = position
    command.velocity = 0.0
    command.acceleration = 0.0
    command.effort = 0.0
    return command


def make_vector3(x, y, z):
    vec = Vector3()
    vec.x = x
    vec.y = y
    vec.z = z
    return vec


def build_joint_target(group_name, joint_names, positions, time_from_start_s):
    target = make_empty_target()

    group_traj = TargetGroupTrajectory()
    group_traj.target_config = make_group_target_config()
    group_traj.joint_names = list(joint_names)

    command = GroupCommand()
    command.time_from_start_s = time_from_start_s
    command.joint_commands = [make_joint_command(position) for position in positions]
    group_traj.group_commands = [command]

    target.target_group_trajectory_map = {group_name: group_traj}
    return target


def build_swerve_chassis_pose_target(x, y, yaw, time_from_start_s, frame_id="rel(0)", reference_frame_id="odom"):
    target = make_empty_target()

    task_traj = TargetTaskTrajectory()
    task_traj.target_config = make_pose_target_config()
    task_traj.group_names = [S1JointGroup.swerve_chassis]
    task_traj.subtask_names = [f"{CHASSIS_SUBTASK_POSE}_{now_ns()}"]

    triad = FrameTriad()
    triad.header.timestamp_ns = now_ns()
    triad.header.frame_id = frame_id
    triad.body_frame_id = "base_link"
    triad.reference_frame_id = reference_frame_id
    triad.pose = Pose([x, y, 0.0], yaw_to_quaternion(yaw))

    command = TaskCommand()
    command.time_from_start_s = time_from_start_s
    command.subtask_commands = [triad]
    task_traj.task_commands = [command]

    target.target_task_trajectory_map = {CHASSIS_TASK_NAME: task_traj}
    return target


def build_swerve_chassis_twist_target(vx, vy, wz, time_from_start_s):
    target = make_empty_target()

    task_traj = TargetTaskTrajectory()
    task_traj.target_config = make_twist_target_config()
    task_traj.group_names = [S1JointGroup.swerve_chassis]
    task_traj.subtask_names = [f"{CHASSIS_SUBTASK_TWIST}_{now_ns()}"]

    twist = Twist()
    twist.linear = make_vector3(vx, vy, 0.0)
    twist.angular = make_vector3(0.0, 0.0, wz)

    triad = FrameTriad()
    triad.header.timestamp_ns = now_ns()
    triad.header.frame_id = "base_link"
    triad.body_frame_id = "base_link"
    triad.reference_frame_id = "base_link"
    triad.twist = twist

    command = TaskCommand()
    command.time_from_start_s = time_from_start_s
    command.subtask_commands = [triad]
    task_traj.task_commands = [command]

    target.target_task_trajectory_map = {CHASSIS_TASK_NAME: task_traj}
    return target


def build_stop_twist_target():
    return build_swerve_chassis_twist_target(0.0, 0.0, 0.0, 0.1)


def merge_targets(targets):
    merged = make_empty_target()
    group_map = {}
    task_map = {}
    for target in targets:
        group_map.update(target.target_group_trajectory_map)
        task_map.update(target.target_task_trajectory_map)
    merged.target_group_trajectory_map = group_map
    merged.target_task_trajectory_map = task_map
    return merged


def print_error_info(scene_name, error_info):
    if error_info is None:
        print(f"{scene_name}: request_target returned None")
        return
    if not isinstance(error_info, ErrorInfo):
        print(f"{scene_name}: unexpected response type {type(error_info)}")
        return
    if not error_info.error_vec:
        print(f"{scene_name}: request_target success, service returned no errors")
        return

    print(f"{scene_name}: request_target returned {len(error_info.error_vec)} error entries:")
    for error in error_info.error_vec:
        print(
            f"  component={error.commpent}, code={error.error_code}, description={error.description}"
        )


def ensure_controller(robot, controller_name):
    status = robot.switch_controller(controller_name)
    print(f"switch_controller({controller_name}): {getattr(status, 'name', status)}")
    return status


def run_twist_scene(robot, scene_name, target, twist_duration_s):
    print(f"{scene_name}: start moving for {twist_duration_s} seconds")
    print_error_info(scene_name, robot.request_target(target))
    time.sleep(twist_duration_s)
    print(f"{scene_name}: send stop twist target")
    print_error_info(f"{scene_name}_stop", robot.request_target(build_stop_twist_target()))


def print_menu():
    print(
        "\nAvailable commands:\n"
        "  joint        - request a joint-only torso target\n"
        "  base_pose    - request a swerve chassis pose target\n"
        "  base_twist   - request a swerve chassis twist target with auto stop\n"
        "  mixed_pose   - request torso + swerve chassis pose in one target\n"
        "  mixed_twist  - request torso + swerve chassis twist in one target\n"
        "  quit         - exit example\n"
    )


def main():
    robot = GalbotRobot()
    if not robot.init():
        print("robot init failed")
        return

    time.sleep(2)

    torso_joint_names = robot.get_joint_names(True, [S1JointGroup.torso])
    if not torso_joint_names:
        print("failed to fetch active torso joints")
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return

    torso_single_joint = [torso_joint_names[0]]
    joint_time_s = 3.0
    pose_time_s = 4.0
    twist_command_time_s = 0.2
    twist_duration_s = 2.0
    torso_height_m = 0.60  # torso target height in meters

    print_menu()

    while True:
        command = input("Enter command: ").strip()
        if command == "quit":
            break

        if command == "joint":
            target = build_joint_target(S1JointGroup.torso, torso_single_joint, [torso_height_m], joint_time_s)
            print_error_info("joint", robot.request_target(target))
            continue

        if command == "base_pose":
            if ensure_controller(robot, S1ControllerName.SWERVE_CHASSIS_POSE_CTRL) != ControlStatus.SUCCESS:
                continue
            target = build_swerve_chassis_pose_target(0.2, 0.0, 0.0, pose_time_s)
            print_error_info("base_pose", robot.request_target(target))
            continue

        if command == "base_twist":
            if ensure_controller(robot, S1ControllerName.SWERVE_CHASSIS_TWIST_CTRL) != ControlStatus.SUCCESS:
                continue
            target = build_swerve_chassis_twist_target(0.05, 0.0, 0.0, twist_command_time_s)
            run_twist_scene(robot, "base_twist", target, twist_duration_s)
            continue

        if command == "mixed_pose":
            if ensure_controller(robot, S1ControllerName.SWERVE_CHASSIS_POSE_CTRL) != ControlStatus.SUCCESS:
                continue
            target = merge_targets(
                [
                    build_joint_target(S1JointGroup.torso, torso_single_joint, [torso_height_m], joint_time_s),
                    build_swerve_chassis_pose_target(0.1, 0.0, 0.0, pose_time_s),
                ]
            )
            print_error_info("mixed_pose", robot.request_target(target))
            continue

        if command == "mixed_twist":
            if ensure_controller(robot, S1ControllerName.SWERVE_CHASSIS_TWIST_CTRL) != ControlStatus.SUCCESS:
                continue
            target = merge_targets(
                [
                    build_joint_target(S1JointGroup.torso, torso_single_joint, [torso_height_m], joint_time_s),
                    build_swerve_chassis_twist_target(0.05, 0.0, 0.0, twist_command_time_s),
                ]
            )
            run_twist_scene(robot, "mixed_twist", target, twist_duration_s)
            continue

        print(f"Unknown command: {command}")
        print_menu()

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()


if __name__ == "__main__":
    main()
