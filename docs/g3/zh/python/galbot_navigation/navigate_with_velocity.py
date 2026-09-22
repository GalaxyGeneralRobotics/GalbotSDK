from galbot_sdk.g3 import ControlStatus, G3ControllerName, GalbotNavigation, GalbotRobot
from galbot_sdk.g3 import NavigationTaskStatus

import sys
import time


def print_current_pose(navigation, indent="  "):
    current_pose = navigation.get_current_pose()
    print(
        f"{indent}current_pose: ["
        f"{current_pose[0]:g}, {current_pose[1]:g}, {current_pose[2]:g}, "
        f"{current_pose[3]:g}, {current_pose[4]:g}, {current_pose[5]:g}, {current_pose[6]:g}]"
    )


def init_sdk():
    navigation = GalbotNavigation()
    robot = GalbotRobot()

    if robot.init():
        print("Base instance initialized successfully!")
    else:
        print("Base instance initialization failed!", file=sys.stderr)
        return None, None

    if navigation.init():
        print("Navigation instance initialized successfully!")
    else:
        print("Navigation instance initialization failed!", file=sys.stderr)
        return None, None

    res = robot.switch_controller(G3ControllerName.CHASSIS_POSE_CTRL)
    if res != ControlStatus.SUCCESS:
        print("Failed to switch controller!", file=sys.stderr)
        return None, None

    time.sleep(3)
    print_current_pose(navigation)
    return navigation, robot


def shutdown_sdk(navigation, robot):
    navigation.stop_navigation()
    print_current_pose(navigation)
    print("navigation stopped")

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    print("example completed ...")


def monitor_navigation(navigation, timeout_s, poll_interval_s=0.5):
    start = time.time()
    while True:
        status = navigation.get_navigation_status()
        elapsed = time.time() - start
        print(f"  status: {status.name}, elapsed: {elapsed:.1f}s")
        print_current_pose(navigation, indent="    ")

        if status in (
            NavigationTaskStatus.SUCCESS,
            NavigationTaskStatus.FAILED,
            NavigationTaskStatus.INTERRUPTED,
            NavigationTaskStatus.OCCUPIED,
            NavigationTaskStatus.COLLISION,
            NavigationTaskStatus.CLOSE_TO_OBSTACLE,
        ):
            break

        if elapsed >= timeout_s:
            print("  monitor timeout reached")
            break

        time.sleep(poll_interval_s)


def test_base_vel_cmd():
    navigation, robot = init_sdk()
    if navigation is None or robot is None:
        return -1

    vx = 0.3
    vy = 0.0
    vyaw = 0.0
    duration_s = 6.0
    enable_collision_check = True

    print(
        f"navigate_with_velocity: vx={vx}, vy={vy}, "
        f"vyaw={vyaw}, duration_s={duration_s}"
    )
    success, status = navigation.navigate_with_velocity(
        vx, vy, vyaw, duration_s, enable_collision_check
    )
    print(f"navigate_with_velocity result: success={success}, status={status}")

    monitor_navigation(navigation, timeout_s=duration_s + 2.0)

    shutdown_sdk(navigation, robot)
    return 0


def test_change_vel_cmd():
    navigation, robot = init_sdk()
    if navigation is None or robot is None:
        return -1

    print("send first velocity command")
    success, status = navigation.navigate_with_velocity(0.3, 0.0, 0.0, 6.0, True)
    print(f"navigate_with_velocity result 1: success={success}, status={status}")
    monitor_navigation(navigation, timeout_s=4.0)

    print("send second velocity command with y direction")
    success, status = navigation.navigate_with_velocity(0.1, 0.2, 0.0, 6.0, True)
    print(f"navigate_with_velocity result 2: success={success}, status={status}")
    monitor_navigation(navigation, timeout_s=8.0)

    shutdown_sdk(navigation, robot)
    return 0


def main():
    return test_base_vel_cmd()

    # return test_change_vel_cmd()


if __name__ == "__main__":
    sys.exit(main())
