import argparse
import time
from galbot_sdk.g1 import GalbotRobot, G1JointGroup, DexHandType

DEXHAND_TYPE_MAP = {
    "inspire": DexHandType.INSPIRE,
    "brainco": DexHandType.BRAINCO,
    "sharpa": DexHandType.SHARPA,
    "linker_l20": DexHandType.LINKER_L20,
}


def parse_dexhand_type(type_name: str) -> DexHandType:
    key = type_name.lower()
    if key not in DEXHAND_TYPE_MAP:
        raise ValueError(
            f"Unknown dexhand type '{type_name}', choose from: inspire, brainco, sharpa, linker_l20"
        )
    return DEXHAND_TYPE_MAP[key]


def print_dexhand_state(hand_name: str, dexhand_state, dexhand_type: DexHandType) -> None:
    type_label = "sharpa" if dexhand_type == DexHandType.SHARPA else "dexterous"
    print(f"{hand_name} {type_label} hand state:")
    print(f"Timestamp (ns): {dexhand_state.timestamp_ns}")

    joint_state_vec = dexhand_state.joint_state.joint_state_vec
    print(f"  Joint states ({len(joint_state_vec)} joints):")
    for i, js in enumerate(joint_state_vec):
        if dexhand_type == DexHandType.SHARPA:
            print(
                f"    joint{i + 1}: position={js.position:.4f}, velocity={js.velocity:.4f}, "
                f"effort={js.effort:.4f}, current={js.current:.4f}"
            )
        else:
            # joint_name is filled by the SDK (e.g. left_dexhand_joint1 / left_dexhand_thumb_roll for L20)
            joint_label = js.joint_name if js.joint_name else f"{hand_name.lower()}_dexhand_joint{i + 1}"
            print(
                f"    {joint_label}: "
                f"position={js.position:.4f}, velocity={js.velocity:.4f}, "
                f"acceleration={js.acceleration:.4f}, "
                f"effort={js.effort:.4f}, current={js.current:.4f}"
            )

    if dexhand_type != DexHandType.SHARPA:
        return

    force_map = dexhand_state.force_sensor_map
    if not force_map:
        print("  (no force sensor data)")
    else:
        print(f"  Force sensors ({len(force_map)} sensors):")
        for sensor_name, effort in force_map.items():
            print(
                f"    {sensor_name} @ {effort.timestamp_ns}: "
                f"Fx={effort.force.x:.4f}, Fy={effort.force.y:.4f}, Fz={effort.force.z:.4f}, "
                f"Mx={effort.torque.x:.4f}, My={effort.torque.y:.4f}, Mz={effort.torque.z:.4f}"
            )


def main():
    parser = argparse.ArgumentParser(description="Get dexhand state for left and right hands.")
    parser.add_argument(
        "--type",
        default="inspire",
        choices=DEXHAND_TYPE_MAP.keys(),
        help="Dexhand model: inspire, brainco, sharpa, or linker_l20",
    )
    args = parser.parse_args()
    dexhand_type = parse_dexhand_type(args.type)

    robot = GalbotRobot()
    robot.init()

    time.sleep(1)
    print("Initialization succeeded")

    left_state = robot.get_dexhand_state(G1JointGroup.left_dexhand, dexhand_type)
    if left_state is None:
        print("get left dexterous hand state error")
    else:
        print_dexhand_state("Left", left_state, dexhand_type)

    right_state = robot.get_dexhand_state(G1JointGroup.right_dexhand, dexhand_type)
    if right_state is None:
        print("get right dexterous hand state error")
    else:
        print_dexhand_state("Right", right_state, dexhand_type)

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    print("Resources released successfully")


if __name__ == "__main__":
    main()
