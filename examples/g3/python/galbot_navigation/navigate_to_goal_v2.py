from galbot_sdk.g3 import ControlStatus, G3ControllerName, GalbotNavigation, GalbotRobot

import numpy as np
import sys
import time


kLocalizationRetryLimit = 20
kLocalizationRetrySleepMs = 500


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


def main():
    navigation = GalbotNavigation()
    robot = GalbotRobot()

    if robot.init():
        print("Base instance initialized successfully!")
    else:
        print("Base instance initialization failed!", file=sys.stderr)
        return -1

    if navigation.init():
        print("Navigation instance initialized successfully!")
    else:
        print("Navigation instance initialization failed!", file=sys.stderr)
        return -1

    res = robot.switch_controller(G3ControllerName.CHASSIS_POSE_CTRL)
    if res != ControlStatus.SUCCESS:
        print("Failed to switch controller!", file=sys.stderr)
        return -1

    init_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    time.sleep(3)

    if not wait_for_localization(navigation, init_pose):
        print("localization failed", file=sys.stderr)
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return -1

    print_current_pose(navigation)

    relative_goal = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    max_vel = np.array([0.5, 0.5, 0.5], dtype=np.float64)
    pose_frame = "base_link"
    enable_collision_check = True
    is_blocking = True
    timeout_s = 20.0
    omni_plan = False

    print("navigate_to_goal_v2 relative target in base_link frame")
    success, status = navigation.navigate_to_goal_v2(
        relative_goal,
        max_vel,
        pose_frame=pose_frame,
        enable_collision_check=enable_collision_check,
        is_blocking=is_blocking,
        timeout=timeout_s,
        omni_plan=omni_plan,
    )
    print(f"navigate_to_goal_v2 result: success={success}, status={status}")

    print_current_pose(navigation)

    navigation.stop_navigation()
    print("navigation stopped")

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()

    print("example completed ...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
