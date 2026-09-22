import time

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

    # Include common G1 chains and one invalid name to show the empty-result behavior.
    for chain in ["left_arm", "right_arm", "leg", "head", "invalid_chain"]:
        # This is the API being demonstrated: query joint names by chain name.
        joint_names = list(motion.get_chain_joint_names(chain))
        print(f"get_chain_joint_names({chain}): {joint_names}")

    # Clean shutdown releases SDK resources before the process exits.
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    return 0


if __name__ == "__main__":
    raise SystemExit(run_example())
