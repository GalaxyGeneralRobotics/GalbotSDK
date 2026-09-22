import time

import galbot_sdk.g3 as gm
from galbot_sdk.g3 import GalbotMotion, GalbotRobot


def run_example():
    motion = GalbotMotion()
    robot = GalbotRobot()
    # Initialize SDK interfaces directly in the example flow.
    if not motion.init():
        print("GalbotMotion init FAILED")
        return 1
    if not robot.init():
        print("GalbotRobot init FAILED")
        return 1
    print("GalbotMotion/GalbotRobot init OK")
    time.sleep(1)

    # These are common motion-planning service config categories.
    config_types = [
        "sampler",
        "ik_solver",
        "trajectory_planner",
        "traj_validity_checker",
    ]

    all_success = True
    for config_type in config_types:
        # This is the API being demonstrated: query one config payload by type.
        status, config = motion.get_config_by_type(config_type)
        print(
            f"get_config_by_type({config_type}) status: {motion.status_to_string(status)}, "
            f"payload bytes: {len(config.common_str)}"
        )
        all_success = all_success and status == gm.MotionStatus.SUCCESS

    # Clean shutdown releases SDK resources before the process exits.
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    return 0 if all_success else 1


if __name__ == "__main__":
    raise SystemExit(run_example())
