import time

import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotMotion, GalbotRobot


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

    # Read the current config first. This keeps the example non-destructive.
    get_status, config = motion.get_config_by_type("sampler")
    print(f"get_config_by_type(sampler) status: {motion.status_to_string(get_status)}")
    if get_status != gm.MotionStatus.SUCCESS:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return 1

    print(f"Current sampler config payload bytes: {len(config.common_str)}")
    # Send the unchanged payload back to cover the API call without changing behavior.
    set_status = motion.set_config_by_type(config)
    print(f"set_config_by_type(sampler, unchanged payload) status: {motion.status_to_string(set_status)}")

    # Clean shutdown releases SDK resources before the process exits.
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    return 0 if set_status == gm.MotionStatus.SUCCESS else 1


if __name__ == "__main__":
    raise SystemExit(run_example())
