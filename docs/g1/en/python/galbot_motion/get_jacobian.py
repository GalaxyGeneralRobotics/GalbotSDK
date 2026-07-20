import time

import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotMotion, GalbotRobot


def print_status(status):
    labels = {
        gm.MotionStatus.SUCCESS: "SUCCESS",
        gm.MotionStatus.TIMEOUT: "TIMEOUT",
        gm.MotionStatus.FAULT: "FAULT",
        gm.MotionStatus.INVALID_INPUT: "INVALID_INPUT",
        gm.MotionStatus.INIT_FAILED: "INIT_FAILED",
        gm.MotionStatus.IN_PROGRESS: "IN_PROGRESS",
        gm.MotionStatus.STOPPED_UNREACHED: "STOPPED_UNREACHED",
        gm.MotionStatus.DATA_FETCH_FAILED: "DATA_FETCH_FAILED",
        gm.MotionStatus.PUBLISH_FAIL: "PUBLISH_FAIL",
        gm.MotionStatus.COMM_DISCONNECTED: "COMM_DISCONNECTED",
    }
    return labels.get(status, "UNKNOWN")


def print_vector(values):
    print("[", end="")
    for index, value in enumerate(values):
        if index > 0:
            print(", ", end="")
        print(value, end="")
    print("]", end="")


def print_joint_state(joint_state):
    print("{")
    for chain_name, joints in joint_state.items():
        print(f"    {chain_name}: ", end="")
        print_vector(joints)
        print()
    print("  }")


def print_get_jacobian_params(
    chain_name,
    target_frame,
    reference_frame,
    joint_state,
):
    print("get_jacobian parameters:")
    print(f"  chain_name: {chain_name}")
    print(f"  target_frame: {target_frame}")
    print(f"  reference_frame: {reference_frame}")
    print("  joint_state: ", end="")
    print_joint_state(joint_state)


def print_jacobian(label, result):
    status, matrix = result
    print(f"[{label}] Status: {print_status(status)}")
    if status != gm.MotionStatus.SUCCESS or not matrix:
        print("Jacobian computation failed or returned an empty matrix.")
        return

    rows = len(matrix)
    cols = len(matrix[0])
    print(f"Jacobian matrix ({rows}x{cols}):")
    for row_index, row in enumerate(matrix):
        row_str = f"  row {row_index}:"
        for value in row:
            row_str += f" {value:10.5f}"
        print(row_str)


def make_fixed_joint_values(dof):
    fixed_values = [-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2]
    return [fixed_values[index % len(fixed_values)] for index in range(dof)]


def run_example():
    motion = GalbotMotion()
    robot = GalbotRobot()
    robot_initialized = False

    if not motion.init():
        print("GalbotMotion initialization failed")
        return 1
    print("GalbotMotion initialized successfully")

    if not robot.init():
        print("GalbotRobot initialization failed")
        return 1
    robot_initialized = True
    print("GalbotRobot initialized successfully")

    try:
        time.sleep(3)

        support_chains = set(motion.get_supported_chains())
        print(f"support_chains ({len(support_chains)}): {sorted(support_chains)}")
        # support_chains.discard("torso")
        if not support_chains:
            support_chains = {"head", "left_arm", "right_arm", "torso", "leg"}

        try:
            current_chain_joint_state = motion.get_chain_joint_state()
        except Exception as e:
            print(f"get_chain_joint_state exception: {e}")
            current_chain_joint_state = {}

        target_frame = "EndEffector"
        reference_frame = "base_link"

        # Known chain DOF used only when the runtime joint state is unavailable.
        fallback_dof = {"head": 2, "left_arm": 7, "right_arm": 7, "leg": 5}

        for chain_name in sorted(support_chains):
            # Prefer the runtime joint count; fall back to a known DOF when it is unavailable.
            current_joints = current_chain_joint_state.get(chain_name, [])
            if current_joints:
                dof = len(current_joints)
            else:
                dof = fallback_dof.get(chain_name, 0)
            if dof == 0:
                print(f"Skip chain '{chain_name}': unable to determine chain DOF.")
                continue

            joint_state = {chain_name: make_fixed_joint_values(dof)}

            try:
                print(f"=== get_jacobian: {chain_name} ===")
                print_get_jacobian_params(
                    chain_name,
                    target_frame,
                    reference_frame,
                    joint_state,
                )
                result = motion.get_jacobian(
                    chain_name,
                    target_frame,
                    reference_frame,
                    joint_state,
                )
                print_jacobian(chain_name, result)
            except Exception as e:
                print(f"{chain_name} exception: {e}")

        return 0
    finally:
        if robot_initialized:
            robot.request_shutdown()
            robot.wait_for_shutdown()
            robot.destroy()


if __name__ == "__main__":
    raise SystemExit(run_example())
