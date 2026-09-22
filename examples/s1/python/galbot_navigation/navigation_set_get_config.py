from galbot_sdk.s1 import GalbotNavigation, GalbotRobot

import numpy as np
import sys


def print_result(name, result):
    success, status = result
    print(f"{name}: success={success}, status={status}")


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

    print("dump navigation configs before setting parameters")
    print_result("dump_navigation_configs", navigation.dump_navigation_configs())

    # valid range: [0.05, 1.5]
    vel_limit = np.array([0.5, 0.5, 0.5], dtype=np.float64)
    # valid range: [0.05, 6.0]
    acc_limit = np.array([0.5, 0.5, 0.5], dtype=np.float64)
    # valid range: [0.05, 12.0]
    jerk_limit = np.array([5.0, 5.0, 5.0], dtype=np.float64)
    # valid range: [0.03, 2.0]
    arrival_threshold = np.array([0.05, 0.05, 0.05], dtype=np.float64)
    timeout_s = 30.0

    print_result(
        "set_navigation_velocity_limit",
        navigation.set_navigation_velocity_limit(vel_limit),
    )
    print_result(
        "set_navigation_kinematics_limits",
        navigation.set_navigation_kinematics_limits(vel_limit, acc_limit, jerk_limit),
    )
    print_result("set_navigation_timeout", navigation.set_navigation_timeout(timeout_s))
    print_result(
        "set_navigation_arrival_threshold",
        navigation.set_navigation_arrival_threshold(arrival_threshold),
    )

    print("dump navigation configs after setting parameters")
    print_result("dump_navigation_configs", navigation.dump_navigation_configs())

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()

    return 0


if __name__ == "__main__":
    sys.exit(main())
