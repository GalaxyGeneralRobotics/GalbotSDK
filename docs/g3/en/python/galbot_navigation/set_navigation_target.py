from galbot_sdk.g3 import ControlStatus, G3ControllerName, GalbotNavigation, GalbotRobot
from galbot_sdk.g3 import NavigationTaskStatus, Pose

import numpy as np
import sys
import time


kLocalizationRetryLimit = 20
kLocalizationRetrySleepMs = 500
kPollIntervalMs = 500
kMonitorDurationMs = 2000


def navigation_target_status_to_string(status):
    if status == NavigationTaskStatus.UNKNOWN:
        return "UNKNOWN"
    if status == NavigationTaskStatus.RUNNING:
        return "RUNNING"
    if status == NavigationTaskStatus.SUCCESS:
        return "SUCCESS"
    if status == NavigationTaskStatus.FAILED:
        return "FAILED"
    if status == NavigationTaskStatus.INTERRUPTED:
        return "INTERRUPTED"
    if status == NavigationTaskStatus.OCCUPIED:
        return "OCCUPIED"
    if status == NavigationTaskStatus.COLLISION:
        return "COLLISION"
    if status == NavigationTaskStatus.CLOSE_TO_OBSTACLE:
        return "CLOSE_TO_OBSTACLE"
    return "UNKNOWN"


def format_pose(pose):
    values = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    return "[" + ", ".join(f"{value:g}" for value in values) + "]"


def print_current_pose(navigation, indent="  "):
    current_pose = navigation.get_current_pose()
    print(
        f"{indent}current_pose: ["
        f"{current_pose[0]:g}, {current_pose[1]:g}, {current_pose[2]:g}, "
        f"{current_pose[3]:g}, {current_pose[4]:g}, {current_pose[5]:g}, {current_pose[6]:g}]"
    )


def print_task_snapshot(navigation, handle, snapshot, include_pose=True):
    print(
        f"  task_id: {handle.task_id}, request_sent: {'true' if handle.request_sent else 'false'}, "
        f"status: {navigation_target_status_to_string(snapshot.status)}"
    )
    if include_pose:
        print_current_pose(navigation, indent="    ")


def wait_for_localization(navigation, init_pose):
    retry = 0
    while (not navigation.is_localized()) and retry < kLocalizationRetryLimit:
        navigation.relocalize(init_pose)
        time.sleep(kLocalizationRetrySleepMs / 1000.0)
        retry += 1
    return navigation.is_localized()


def print_task_summary(navigation, handles):
    print("\nfinal task summary")
    for handle in handles:
        snapshot = navigation.get_navigation_target_status(handle.task_id)
        print_task_snapshot(navigation, handle, snapshot, include_pose=False)


def main():
    navigation = GalbotNavigation()
    robot = GalbotRobot()

    if robot.init():
        print("Base instance initialized successfully!")
    else:
        print("Base instance initialization failed!")
        return -1

    if navigation.init():
        print("Navigation instance initialized successfully!")
    else:
        print("Navigation instance initialization failed!")
        return -1

    res = robot.switch_controller(G3ControllerName.CHASSIS_POSE_CTRL)
    if res != ControlStatus.SUCCESS:
        print("Failed to switch controller!")
        return -1

    init_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    time.sleep(3)

    if not wait_for_localization(navigation, init_pose):
        print("localization failed", file=sys.stderr)
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return -1

    frame_id = "map"
    speed_ratio = 0.9
    enable_collision_check = True
    target_points = [
        Pose([1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00]),
        Pose([0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00]),
        Pose([0.30, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00]),
    ]

    handles = []

    for i, target in enumerate(target_points):
        print(f"set target: dynamic_goal_{i}, target_pose: {format_pose(target)}")

        handle = navigation.set_navigation_target(target, frame_id, speed_ratio, enable_collision_check)
        handles.append(handle)

        start_time = time.time()
        while True:
            snapshot = navigation.get_navigation_target_status(handle.task_id)
            print_task_snapshot(navigation, handle, snapshot)

            now = time.time()
            elapsed_ms = (now - start_time) * 1000.0
            if elapsed_ms >= kMonitorDurationMs:
                break

            time.sleep(kPollIntervalMs / 1000.0)

    print_task_summary(navigation, handles)

    print("\nstop navigation and shutdown")
    navigation.stop_navigation()

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()

    return 0


if __name__ == "__main__":
    sys.exit(main())
