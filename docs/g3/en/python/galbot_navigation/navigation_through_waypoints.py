from galbot_sdk.g3 import ControlStatus, G3ControllerName, GalbotNavigation, GalbotRobot
from galbot_sdk.g3 import NavigationTaskStatus, Pose, Waypoint, WaypointParams

import numpy as np
import sys
import time


kLocalizationRetryLimit = 20
kLocalizationRetrySleepMs = 500
kTaskTimeoutSec = 100.0


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


def wait_for_localization(navigation, init_pose):
    retry = 0
    while (not navigation.is_localized()) and retry < kLocalizationRetryLimit:
        navigation.relocalize(init_pose)
        time.sleep(kLocalizationRetrySleepMs / 1000.0)
        retry += 1
    return navigation.is_localized()


def print_task_snapshot(navigation, handle, snapshot):
    print(
        f"  task_id: {handle.task_id}, request_sent: {'true' if handle.request_sent else 'false'}, "
        f"status: {navigation_target_status_to_string(snapshot.status)}"
    )
    print_current_pose(navigation, indent="    ")


def monitor_task_until_terminal(navigation, handle, timeout_s):
    start_time = time.time()
    while True:
        snapshot = navigation.get_navigation_target_status(handle.task_id)
        print_task_snapshot(navigation, handle, snapshot)

        now = time.time()
        if snapshot.status in {
            NavigationTaskStatus.SUCCESS,
            NavigationTaskStatus.FAILED,
            NavigationTaskStatus.INTERRUPTED,
            NavigationTaskStatus.OCCUPIED,
            NavigationTaskStatus.COLLISION,
            NavigationTaskStatus.CLOSE_TO_OBSTACLE,
        }:
            return

        if (now - start_time) >= timeout_s:
            print("  timeout reached while waiting for task completion.")
            return

        time.sleep(1.0)


def make_waypoint(pose, params=None):
    if params is None:
        params = WaypointParams()
    return Waypoint(pose, params)


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

    waypoint_2_params = WaypointParams()
    waypoint_2_params.arrival_position_threshold_x = 0.02
    waypoint_2_params.arrival_position_threshold_y = 0.02
    waypoint_2_params.arrival_orientation_threshold = 0.08
    waypoint_2_params.velocity_scale = 0.2

    waypoint_3_params = WaypointParams()
    waypoint_3_params.arrival_position_threshold_x = 0.04
    waypoint_3_params.arrival_position_threshold_y = 0.04
    waypoint_3_params.arrival_orientation_threshold = 0.05
    waypoint_3_params.velocity_scale = 0.8
    waypoint_3_params.acceleration_scale = 0.8
    waypoint_3_params.jerk_scale = 0.8

    waypoints = [
        make_waypoint(Pose([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])),
        make_waypoint(Pose([0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0]), waypoint_2_params),
        make_waypoint(Pose([1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0]), waypoint_3_params),
    ]

    time.sleep(3)

    if not wait_for_localization(navigation, init_pose):
        print("localization failed", file=sys.stderr)
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return -1

    time.sleep(2)

    print("set target: waypoints")
    for i, waypoint in enumerate(waypoints):
        print(f"  waypoint_{i}: {format_pose(waypoint.pose)}")

    frame_id = "map"
    enable_collision_check = True

    handle = navigation.navigate_through_waypoints(waypoints, frame_id, enable_collision_check)

    if handle.request_sent:
        monitor_task_until_terminal(navigation, handle, kTaskTimeoutSec)
    else:
        print("navigation failed to submit task")

    navigation.stop_navigation()

    print("\nfinal task summary")
    snapshot = navigation.get_navigation_target_status(handle.task_id)
    print(
        f"  task_id: {handle.task_id}, request_sent: {'true' if handle.request_sent else 'false'}, "
        f"status: {navigation_target_status_to_string(snapshot.status)}"
    )

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()

    return 0


if __name__ == "__main__":
    sys.exit(main())
