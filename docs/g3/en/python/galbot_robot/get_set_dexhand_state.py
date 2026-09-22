import time
from galbot_sdk.g3 import GalbotRobot, G3JointGroup, ControlStatus, JointCommand, DexHandType

CYCLE_COUNT = 3
COMMAND_INTERVAL_SECONDS = 1.0
INIT_WAIT_SECONDS = 2.0

# The following demo positions are copied from the standalone set_dexhand_command example.
# Different dexterous hand models expose different joint counts and units, so each model
# needs its own open/close target list.
INSPIRE_RH56DFX_POSITION_OPEN = [
    1000.0,     # finger1
    1000.0,     # finger2
    1000.0,     # finger3
    1000.0,     # finger4
    1000.0,     # finger5
    1000.0,     # finger6
]

INSPIRE_RH56DFX_POSITION_CLOSE = [
    850.0,     # finger1
    350.0,     # finger2
    60.0,     # finger3
    60.0,     # finger4
    60.0,     # finger5
    60.0,     # finger6
]

INSPIRE_RH56F2_POSITION_OPEN = [
    1750.0,     # thumb_bend
    1550.0,     # thumb_rotate
    1740.0,     # index
    1740.0,     # middle
    1740.0,     # ring
    1740.0,     # little
]

INSPIRE_RH56F2_POSITION_CLOSE = [
    1600.0,     # thumb_bend
    1250.0,     # thumb_rotate
    950.0,     # index
    950.0,     # middle
    950.0,     # ring
    950.0,     # little
]

BRAINCO_POSITION_OPEN = [
    100.0,      # finger1
    100.0,      # finger2
    100.0,      # finger3
    100.0,      # finger4
    100.0,      # finger5
    100.0,      # finger6
]

BRAINCO_POSITION_CLOSE = [
    33.3,      # finger1
    100.0,      # finger2
    86.7,      # finger3
    86.7,      # finger4
    86.7,      # finger5
    86.7,      # finger6
]

LINKER_L20_POSITION_OPEN = [
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

LINKER_L20_POSITION_CLOSE = [
    0.85,     # thumb_roll
    1.55,     # thumb_yaw
    0.00,    # index_yaw
    0.00,    # middle_yaw
    0.00,    # ring_yaw
    0.00,    # little_yaw
    0.40,    # thumb_root
    0.00,     # index_root
    0.00,     # middle_root
    0.00,     # ring_root
    0.00,     # little_root
    0.35,     # thumb_tip
    0.00,     # index_tip
    0.00,     # middle_tip
    0.00,     # ring_tip
    0.00,     # little_tip
]

SHARPA_POSITION_OPEN = [
    0.0,     # thumb CMC_FE
    0.0,     # thumb CMC_AA
    0.0,     # thumb MCP_FE
    0.0,     # thumb MCP_AA
    0.0,     # thumb IP
    0.0,     # index MCP_FE
    0.0,     # index MCP_AA
    0.0,     # index PIP
    0.0,     # index DIP
    0.0,     # middle MCP_FE
    0.0,     # middle MCP_AA
    0.0,     # middle PIP
    0.0,     # middle DIP
    0.0,     # ring MCP_FE
    0.0,     # ring MCP_AA
    0.0,     # ring PIP
    0.0,     # ring DIP
    0.0,     # pinky CMC
    0.0,     # pinky MCP_FE
    0.0,     # pinky MCP_AA
    0.0,     # pinky PIP
    0.0,     # pinky DIP
]

SHARPA_POSITION_CLOSE = [
    0.87,    # thumb CMC_FE
    0.0,     # thumb CMC_AA
    0.44,    # thumb MCP_FE
    0.0,     # thumb MCP_AA
    0.87,    # thumb IP
    0.78,    # index MCP_FE
    0.0,     # index MCP_AA
    0.87,    # index PIP
    0.70,    # index DIP
    0.78,    # middle MCP_FE
    0.0,     # middle MCP_AA
    0.87,    # middle PIP
    0.70,    # middle DIP
    0.78,    # ring MCP_FE
    0.0,     # ring MCP_AA
    0.87,    # ring PIP
    0.70,    # ring DIP
    0.13,    # pinky CMC
    0.78,    # pinky MCP_FE
    0.0,     # pinky MCP_AA
    0.87,    # pinky PIP
    0.70,    # pinky DIP
]

DEXHAND_TYPE_MAP = {
    "inspire": DexHandType.INSPIRE,
    "inspire_rh56dfx": DexHandType.INSPIRE_RH56DFX,
    "inspire_rh56f2": DexHandType.INSPIRE_RH56F2,
    "brainco": DexHandType.BRAINCO,
    "revo": DexHandType.BRAINCO,
    "sharpa": DexHandType.SHARPA,
    "linker_l20": DexHandType.LINKER_L20,
}

EXIT_INPUTS = {"q", "quit", "exit"}
SUPPORTED_TYPE_TEXT = ", ".join(DEXHAND_TYPE_MAP.keys())


def print_supported_types() -> None:
    """Print all dexhand model names accepted by this combined get/set example."""
    print("Supported dexterous hand types:")
    print("  inspire          Compatibility alias for inspire_rh56f2")
    print("  inspire_rh56dfx  Inspire RH56DFX dexterous hand")
    print("  inspire_rh56f2   Inspire RH56F2 dexterous hand")
    print("  brainco          BrainCo dexterous hand")
    print("  revo             Alias for brainco")
    print("  sharpa           Sharpa dexterous hand")
    print("  linker_l20       Linker Hand L20 dexterous hand")


def read_dexhand_type(side_name: str):
    """Read one hand type before robot initialization so invalid input fails early."""
    while True:
        raw_value = input(f"Enter {side_name} dexterous hand type (or q to quit): ").strip()
        key = raw_value.lower()
        if key in EXIT_INPUTS:
            return None, None
        if key in DEXHAND_TYPE_MAP:
            return DEXHAND_TYPE_MAP[key], key

        print(f"Unsupported dexterous hand type: {raw_value}")
        print(f"Please choose from: {SUPPORTED_TYPE_TEXT}")


def make_dexhand_command(positions) -> list:
    """Convert a list of target positions into SDK JointCommand objects."""
    commands = []
    for position in positions:
        cmd = JointCommand()
        cmd.position = position
        commands.append(cmd)
    return commands


def dexhand_motion_positions(dexhand_type: DexHandType, action: str) -> list:
    """Return the demo open or close position list for the selected hand model."""
    is_open = action == "open"
    if dexhand_type == DexHandType.INSPIRE_RH56DFX:
        return INSPIRE_RH56DFX_POSITION_OPEN if is_open else INSPIRE_RH56DFX_POSITION_CLOSE
    if dexhand_type in (DexHandType.INSPIRE, DexHandType.INSPIRE_RH56F2):
        return INSPIRE_RH56F2_POSITION_OPEN if is_open else INSPIRE_RH56F2_POSITION_CLOSE
    if dexhand_type == DexHandType.BRAINCO:
        return BRAINCO_POSITION_OPEN if is_open else BRAINCO_POSITION_CLOSE
    if dexhand_type == DexHandType.SHARPA:
        return SHARPA_POSITION_OPEN if is_open else SHARPA_POSITION_CLOSE
    if dexhand_type == DexHandType.LINKER_L20:
        return LINKER_L20_POSITION_OPEN if is_open else LINKER_L20_POSITION_CLOSE
    raise ValueError(f"Unsupported dexhand type: {dexhand_type}")


def print_dexhand_state(hand_name: str, dexhand_state, dexhand_type: DexHandType) -> None:
    """Print joint states, and print force sensors for Sharpa hands when available."""
    type_label = "sharpa" if dexhand_type == DexHandType.SHARPA else "dexterous"
    print(f"{hand_name} {type_label} hand state:")
    print(f"Timestamp (ns): {dexhand_state.timestamp_ns}")

    joint_state_vec = dexhand_state.joint_state.joint_state_vec
    print(f"  Joint states ({len(joint_state_vec)} joints):")
    for i, js in enumerate(joint_state_vec):
        if dexhand_type == DexHandType.SHARPA:
            # Sharpa state does not use the generic acceleration field in the same way.
            print(
                f"    joint{i + 1}: position={js.position:.4f}, velocity={js.velocity:.4f}, "
                f"effort={js.effort:.4f}, current={js.current:.4f}"
            )
        else:
            # Some SDK backends fill joint_name; otherwise keep a stable fallback label.
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
    print("Starting get_set_dexhand_state example")
    print_supported_types()

    # Ask the user to select the actual dexterous hand model installed on each side.
    left_type, left_label = read_dexhand_type("left")
    if left_type is None:
        print("Example canceled before robot initialization")
        return

    right_type, right_label = read_dexhand_type("right")
    if right_type is None:
        print("Example canceled before robot initialization")
        return

    robot_initialized = False
    robot = GalbotRobot()
    try:
        # Initialize the robot SDK before sending commands or reading states.
        robot.init()
        robot_initialized = True
        print("Initialization succeeded")
        time.sleep(INIT_WAIT_SECONDS)

        # Run one initial open command, then run three close/open demo cycles.
        completed = True
        for action_index, action in enumerate(["open"] + ["close", "open"] * CYCLE_COUNT):
            if action_index > 0 and action == "close":
                cycle_index = (action_index + 1) // 2
                print(f"Running dexterous hand cycle {cycle_index}/{CYCLE_COUNT}")

            # Send the target position command to the left dexterous hand.
            left_positions = dexhand_motion_positions(left_type, action)
            left_command = make_dexhand_command(left_positions)
            left_status = robot.set_dexhand_command(
                G3JointGroup.left_dexhand,
                left_command,
                left_type,
                False,
            )
            if left_status != ControlStatus.SUCCESS:
                print(
                    f"Failed to {action} left {left_label} dexterous hand "
                    f"({len(left_command)} joints), status={left_status}"
                )
                completed = False
            else:
                print(f"Left {left_label} dexterous hand {action} command sent ({len(left_command)} joints)")

            # Send the same action to the right dexterous hand with its own model type.
            right_positions = dexhand_motion_positions(right_type, action)
            right_command = make_dexhand_command(right_positions)
            right_status = robot.set_dexhand_command(
                G3JointGroup.right_dexhand,
                right_command,
                right_type,
                False,
            )
            if right_status != ControlStatus.SUCCESS:
                print(
                    f"Failed to {action} right {right_label} dexterous hand "
                    f"({len(right_command)} joints), status={right_status}"
                )
                completed = False
            else:
                print(f"Right {right_label} dexterous hand {action} command sent ({len(right_command)} joints)")

            # Give the hardware or simulator time to execute the command before reading state.
            time.sleep(COMMAND_INTERVAL_SECONDS)

            # Read back the left hand state after the command has been applied.
            left_state = robot.get_dexhand_state(G3JointGroup.left_dexhand, left_type)
            if left_state is None:
                print("get left dexterous hand state error")
                completed = False
            else:
                print_dexhand_state("Left", left_state, left_type)

            # Read back the right hand state after the command has been applied.
            right_state = robot.get_dexhand_state(G3JointGroup.right_dexhand, right_type)
            if right_state is None:
                print("get right dexterous hand state error")
                completed = False
            else:
                print_dexhand_state("Right", right_state, right_type)

        if completed:
            print("get_set_dexhand_state example completed")
        else:
            print("get_set_dexhand_state example completed with one or more errors")
    finally:
        # Release SDK resources only after the robot has been initialized.
        if robot_initialized:
            robot.request_shutdown()
            robot.wait_for_shutdown()
            robot.destroy()
            print("Resources released successfully")


if __name__ == "__main__":
    main()
