#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import sys
import time
from typing import List, Sequence

import cv2
import numpy as np

from galbot_sdk.s1 import (
    ControlStatus,
    GalbotRobot,
    JointCommand,
    SensorType,
    Trajectory,
    TrajectoryPoint,
)

_K_SPEED_RAD_S = 0.12
_K_TIMEOUT_S = 35.0

_K_PRESET_TORSO: List[float] = [0.58]
_K_PRESET_HEAD: List[float] = [0.0, 0.0]
_K_PRESET_LEFT_ARM: List[float] = [2.0, -1.5, -0.6, -1.7, 0.0, -0.7, 0.0]
_K_PRESET_RIGHT_ARM: List[float] = [-2.0, 1.5, 0.6, 1.7, 0.0, 0.7, 0.0]

_K_HEAD_ARMS_DEMO_GROUPS: List[str] = ["head", "left_arm", "right_arm"]
_K_HEAD_ARMS_DEMO_DT_S = 0.06
_K_HEAD_ARMS_DEMO_SEG1 = 28
_K_HEAD_ARMS_DEMO_SEG2 = 32
_K_HEAD_ARMS_DEMO_SEG3 = 28
_K_HEAD_SWAY_YAW_POS_RAD = 0.22
_K_HEAD_SWAY_YAW_NEG_RAD = -0.22


def _print_summary(step_description: str, step_passed: bool) -> None:
    print(("[PASS] " if step_passed else "[FAIL] ") + step_description)


def make_point(joint_positions_rad: Sequence[float], time_from_start_sec: float) -> TrajectoryPoint:
    point = TrajectoryPoint()
    point.time_from_start_second = time_from_start_sec
    cmds: List[JointCommand] = []
    for q in joint_positions_rad:
        jc = JointCommand()
        jc.position = float(q)
        cmds.append(jc)
    point.joint_command_vec = cmds
    return point


def build_trajectory(
    joint_waypoints_rad: List[List[float]], joint_groups: List[str], time_step_sec: float
) -> Trajectory:
    trajectory = Trajectory()
    trajectory.joint_groups = joint_groups
    trajectory.joint_names = []
    time_from_start_sec = 0.0
    points: List[TrajectoryPoint] = []
    for waypoint in joint_waypoints_rad:
        time_from_start_sec += time_step_sec
        points.append(make_point(waypoint, time_from_start_sec))
    trajectory.points = points
    return trajectory


def linspace_rows(
    start_positions_rad: Sequence[float], end_positions_rad: Sequence[float], num_samples: int
) -> List[List[float]]:
    if num_samples < 2:
        return [list(start_positions_rad)]
    out: List[List[float]] = []
    n = num_samples - 1
    for sample_idx in range(num_samples):
        blend = sample_idx / n
        row = [
            float(start_positions_rad[j]) + blend * (float(end_positions_rad[j]) - float(start_positions_rad[j]))
            for j in range(len(start_positions_rad))
        ]
        out.append(row)
    return out


def append_rows(dest: List[List[float]], src: List[List[float]]) -> None:
    dest.extend(src)


def upper_body_preset_pose_rad() -> List[float]:
    return list(_K_PRESET_HEAD) + list(_K_PRESET_LEFT_ARM) + list(_K_PRESET_RIGHT_ARM)


def build_slow_head_arms_demo_waypoints_rad() -> List[List[float]]:
    """Slow head yaw left-right (joint2 fixed); arms shift slightly with the sway. Torso not in trajectory."""
    home = upper_body_preset_pose_rad()
    pose_yaw_pos = (
        [_K_HEAD_SWAY_YAW_POS_RAD, 0.0]
        + [2.0 + 0.07, -1.5 + 0.04, -0.6, -1.7, 0.0, -0.8 + 0.04, 0.0]
        + [-2.0 - 0.07, 1.5 - 0.04, 0.6, 1.7, 0.0, 0.8 - 0.04, 0.0]
    )
    pose_yaw_neg = (
        [_K_HEAD_SWAY_YAW_NEG_RAD, 0.0]
        + [2.0 - 0.05, -1.5 - 0.03, -0.6, -1.7, 0.0, -0.8 - 0.03, 0.0]
        + [-2.0 + 0.05, 1.5 + 0.03, 0.6, 1.7, 0.0, 0.8 + 0.03, 0.0]
    )
    rows: List[List[float]] = []
    append_rows(rows, linspace_rows(home, pose_yaw_pos, _K_HEAD_ARMS_DEMO_SEG1))
    append_rows(rows, linspace_rows(pose_yaw_pos, pose_yaw_neg, _K_HEAD_ARMS_DEMO_SEG2))
    append_rows(rows, linspace_rows(pose_yaw_neg, home, _K_HEAD_ARMS_DEMO_SEG3))
    return rows


def _decode_rgb_image_bytes(image_data: bytes) -> np.ndarray | None:
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    return img


def decode_head_rgb(rgb_dict: dict) -> np.ndarray | None:
    if not rgb_dict or "data" not in rgb_dict:
        return None
    data = rgb_dict["data"]
    if not data:
        return None
    return _decode_rgb_image_bytes(bytes(data))


def main() -> int:
    body_preset_step_passed = False
    head_camera_capture_ok = False

    robot = GalbotRobot()
    required_sensors = {SensorType.HEAD_LEFT_CAMERA}
    if not robot.init(required_sensors):
        print("GalbotRobot::init failed", file=sys.stderr)
        return 1
    print("Robot initialized (S1, HEAD_LEFT_CAMERA enabled)")
    time.sleep(5.0)

    torso_step_ok = False
    upper_step_ok = False
    demo_step_ok = False

    torso_set_status = robot.set_joint_positions(
        list(_K_PRESET_TORSO), ["torso"], [], True, _K_SPEED_RAD_S, _K_TIMEOUT_S
    )
    torso_step_ok = torso_set_status == ControlStatus.SUCCESS
    if not torso_step_ok:
        print("[FAIL] Torso set_joint_positions not SUCCESS (blocking)", file=sys.stderr)
    _print_summary("Torso set_joint_positions (blocking)", torso_step_ok)

    if torso_step_ok:
        upper_body_joint_targets_rad: List[float] = []
        upper_body_joint_targets_rad.extend(_K_PRESET_HEAD)
        upper_body_joint_targets_rad.extend(_K_PRESET_LEFT_ARM)
        upper_body_joint_targets_rad.extend(_K_PRESET_RIGHT_ARM)

        upper_body_set_status = robot.set_joint_positions(
            upper_body_joint_targets_rad,
            _K_HEAD_ARMS_DEMO_GROUPS,
            [],
            True,
            _K_SPEED_RAD_S,
            _K_TIMEOUT_S,
        )
        upper_step_ok = upper_body_set_status == ControlStatus.SUCCESS
        if not upper_step_ok:
            print(
                "[FAIL] head/left_arm/right_arm set_joint_positions not SUCCESS (blocking)",
                file=sys.stderr,
            )

        _print_summary("Upper set_joint_positions head+arms (blocking)", upper_step_ok)

        if upper_step_ok:
            demo_waypoints_rad = build_slow_head_arms_demo_waypoints_rad()
            demo_trajectory = build_trajectory(
                demo_waypoints_rad, _K_HEAD_ARMS_DEMO_GROUPS, _K_HEAD_ARMS_DEMO_DT_S
            )
            demo_exec_status = robot.execute_joint_trajectory(demo_trajectory, True)
            demo_step_ok = demo_exec_status == ControlStatus.SUCCESS
            if not demo_step_ok:
                print("[FAIL] Head/arms execute_joint_trajectory not SUCCESS (blocking)", file=sys.stderr)
            _print_summary("Head/arms demo trajectory (blocking)", demo_step_ok)
        else:
            print(
                "[INFO] Skipping head/arms demo: upper set_joint_positions did not return SUCCESS.",
                file=sys.stderr,
            )
            _print_summary("Head/arms demo trajectory (blocking)", False)
    else:
        print(
            "[INFO] Skipping upper-body preset and head/arms demo: torso set_joint_positions not SUCCESS.",
            file=sys.stderr,
        )
        _print_summary("Upper set_joint_positions head+arms (blocking)", False)
        _print_summary("Head/arms demo trajectory (blocking)", False)

    body_preset_step_passed = torso_step_ok and upper_step_ok and demo_step_ok

    head_rgb_data = robot.get_rgb_data(SensorType.HEAD_LEFT_CAMERA)
    if not head_rgb_data:
        print("[FAIL] get_rgb_data returned null", file=sys.stderr)
        head_camera_capture_ok = False
    elif isinstance(head_rgb_data, dict):
        raw = head_rgb_data.get("data")
        raw_len = len(raw) if raw is not None and hasattr(raw, "__len__") else 0
        if raw_len == 0:
            print("[FAIL] head camera data missing or empty", file=sys.stderr)
            head_camera_capture_ok = False
        else:
            head_camera_capture_ok = True
            fmt = head_rgb_data.get("format", "")
            print(f"Head camera: {raw_len} bytes format={fmt}")
            head_image_mat = decode_head_rgb(head_rgb_data)
            if head_image_mat is not None and head_image_mat.size > 0:
                print(f"  decoded size {head_image_mat.shape[1]}x{head_image_mat.shape[0]}")
    else:
        head_camera_capture_ok = True
        print("Head camera: get_rgb_data returned non-empty object")
    _print_summary("Head camera data", head_camera_capture_ok)

    print("\n======== Summary ========")
    _print_summary("Motion verify (blocking)", body_preset_step_passed)
    _print_summary("Head camera", head_camera_capture_ok)

    installation_all_checks_passed = body_preset_step_passed and head_camera_capture_ok
    print("\nOverall: PASS\n" if installation_all_checks_passed else "\nOverall: FAIL\n")

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()

    return 0 if installation_all_checks_passed else 1


if __name__ == "__main__":
    sys.exit(main())
