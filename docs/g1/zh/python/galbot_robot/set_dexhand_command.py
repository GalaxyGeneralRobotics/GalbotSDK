from __future__ import annotations
import argparse
import time
from galbot_sdk.g1 import GalbotRobot, G1JointGroup, ControlStatus, JointCommand, DexHandType

LINKER_L20_POSITION_MAX = [
    1.3543754995475996,     # thumb_roll
    1.562069680534925,      # thumb_yaw
    0.23561944901923448,    # index_yaw
    0.23561944901923448,    # middle_yaw
    0.23561944901923448,    # ring_yaw
    0.23561944901923448,    # little_yaw
    0.82030474843733492,    # thumb_root
    1.2217304763960306,     # index_root
    1.2217304763960306,     # middle_root
    1.2217304763960306,     # ring_root
    1.2217304763960306,     # little_root
    1.2042771838760873,     # thumb_tip
    1.7453292519943295,     # index_tip
    1.7453292519943295,     # middle_tip
    1.7453292519943295,     # ring_tip
    1.7453292519943295,     # little_tip
]

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


def dexhand_joint_count(dexhand_type: DexHandType) -> int:
    if dexhand_type == DexHandType.SHARPA:
        return 22
    if dexhand_type == DexHandType.LINKER_L20:
        return 16
    return 6


def make_dexhand_command(joint_count: int, positions) -> list:
    commands = [JointCommand() for _ in range(joint_count)]
    if isinstance(positions, (int, float)):
        for cmd in commands:
            cmd.position = positions
    else:
        for cmd, pos in zip(commands, positions):
            cmd.position = pos
    return commands


def default_positions(dexhand_type: DexHandType) -> tuple:
    """Return (left_positions, right_positions) for the given dexhand model."""
    if dexhand_type == DexHandType.SHARPA:
        return 0.0, 0.0
    if dexhand_type == DexHandType.BRAINCO:
        return 50.0, 80.0
    if dexhand_type == DexHandType.LINKER_L20:
        return LINKER_L20_POSITION_MAX, LINKER_L20_POSITION_MAX
    return 500.0, 800.0


def main():
    parser = argparse.ArgumentParser(description="Set dexhand joint commands for left and right hands.")
    parser.add_argument(
        "--type",
        default="inspire",
        choices=DEXHAND_TYPE_MAP.keys(),
        help="Dexhand model: inspire (6 joints, 0-1000), brainco (6 joints, 0-100), "
        "sharpa (22 joints, radians), linker_l20 (16 joints, radians)",
    )
    args = parser.parse_args()
    dexhand_type = parse_dexhand_type(args.type)
    type_label = args.type.lower()

    robot = GalbotRobot()
    robot.init()
    print("Initialization succeeded")

    time.sleep(2)

    joint_count = dexhand_joint_count(dexhand_type)
    left_pos, right_pos = default_positions(dexhand_type)

    left_command = make_dexhand_command(joint_count, left_pos)
    status = robot.set_dexhand_command(
        G1JointGroup.left_dexhand, left_command, dexhand_type, False
    )
    if status != ControlStatus.SUCCESS:
        print(f"Set left {type_label} dexhand failed")
    else:
        print(f"Set left {type_label} dexhand success ({joint_count} joints, position={left_pos})")

    time.sleep(1 if dexhand_type == DexHandType.SHARPA else 2)

    right_command = make_dexhand_command(joint_count, right_pos)
    status = robot.set_dexhand_command(
        G1JointGroup.right_dexhand, right_command, dexhand_type, False
    )
    if status != ControlStatus.SUCCESS:
        print(f"Set right {type_label} dexhand failed")
    else:
        print(f"Set right {type_label} dexhand success ({joint_count} joints, position={right_pos})")

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    print("Resources released successfully")


if __name__ == "__main__":
    main()
