"""Raw-passthrough example for the self-developed CAN gripper.

The device protocol in this file is an example adapter, not part of the SDK's
generic end-tool transport contract.
"""

import argparse
import math
import struct
import sys
import threading
import time

import galbot_sdk.s1 as sdk


TX_CAN_ID = 0x70A
RX_CAN_ID = 0x10A
MIN_WIDTH_MM = 6.0
MAX_WIDTH_MM = 120.0
MIN_VELOCITY_MM_S = 1.0
MAX_VELOCITY_MM_S = 100.0
MIN_TORQUE_NM = 1.0
MAX_TORQUE_NM = 50.0
STATUS_STALE_SECONDS = 1.0
INITIAL_STATUS_WAIT_SECONDS = 1.0
STATUS_PRINT_INTERVAL_SECONDS = 0.5
STOPPED_VELOCITY_TOLERANCE_MM_S = 1.0
DEFAULT_INIT_TIMEOUT_SECONDS = 20.0
DEFAULT_MONITOR_SECONDS = 10.0
DEFAULT_POSITION_TOLERANCE_MM = 2.0

ERROR_TEXT = {
    0x00: "no error",
    0x01: "not enabled",
    0x08: "overvoltage",
    0x09: "undervoltage",
    0x0A: "overcurrent",
    0x0B: "MOS overheating",
    0x0C: "motor coil overheating",
    0x0D: "communication lost",
    0x0E: "motor overload",
    0x10: "locked rotor",
    0x11: "locked rotor and not enabled",
    0x40: "power loss after calibration",
    0x80: "communication error",
}


def encode_can(command: int, function: int, payload: bytes = b"") -> bytes:
    frame = bytearray(64)
    struct.pack_into("<II", frame, 0, 1, TX_CAN_ID)
    frame[8] = len(payload) + 2
    frame[12] = command
    frame[13] = function
    frame[14 : 14 + len(payload)] = payload
    return bytes(frame)


def encode_init() -> bytes:
    return encode_can(0x21, 0x04)


def encode_clear_error() -> bytes:
    return encode_can(0x21, 0x0A)


def encode_move(width_mm: float, velocity_mm_s: float, torque_nm: float) -> bytes:
    return encode_can(0x21, 0x05, struct.pack("<fff", width_mm, velocity_mm_s, torque_nm))


def decode_status(frame: bytes):
    if len(frame) != 64:
        return "size", None
    frame_type, can_id = struct.unpack_from("<II", frame, 0)
    if frame_type != 1:
        return "frame_type", None
    # The 250 Hz receive buffer can contain an empty CAN slot.  Treat it as an
    # idle transport snapshot instead of reporting a device/protocol error.
    if can_id == 0:
        return "idle_snapshot", None
    if can_id != RX_CAN_ID:
        return "can_id", None

    # Do not require frame[12] == 1 here.  On the current S1 EtherCAT/TIB
    # passthrough implementation, valid 0x10A gripper snapshots have byte 12
    # set to zero.  The device frame itself starts at byte 16 on RX, so use its
    # CAN ID and cmd/function fields to identify a valid status report.
    command, function = frame[16], frame[17]
    if command not in (0x22, 0x23) or function != 0x05:
        return "command", None

    position, velocity, torque = struct.unpack_from("<fff", frame, 22)
    motion_state = frame[34]
    if (
        not all(math.isfinite(value) for value in (position, velocity, torque))
        # An uninitialized gripper legitimately reports position 0 mm even
        # though 0 mm is not a legal Move command target.
        or not 0.0 <= position <= MAX_WIDTH_MM
        or abs(velocity) > MAX_VELOCITY_MM_S
        or abs(torque) > MAX_TORQUE_NM
        or motion_state > 4
    ):
        return "payload", None
    return None, (position, velocity, torque, motion_state, frame[21], command)


def format_status(generation: int, status) -> str:
    position, velocity, torque, motion_state, error_code, command = status
    report_kind = "ANSWER" if command == 0x22 else "REPORT"
    return (
        f"generation={generation} source={report_kind} position={position:.3f}mm "
        f"velocity={velocity:.3f}mm/s torque={torque:.3f}Nm "
        f"motion_state={motion_state} error=0x{error_code:02x} "
        f"({ERROR_TEXT.get(error_code, 'unknown error')})"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--width-mm", type=float, default=50.0)
    parser.add_argument("--velocity-mm-s", type=float, default=50.0)
    parser.add_argument("--torque-nm", type=float, default=20.0)
    parser.add_argument("--init-timeout", type=float, default=DEFAULT_INIT_TIMEOUT_SECONDS)
    parser.add_argument("--monitor-seconds", type=float, default=DEFAULT_MONITOR_SECONDS)
    parser.add_argument(
        "--position-tolerance-mm",
        type=float,
        default=DEFAULT_POSITION_TOLERANCE_MM,
    )
    parser.add_argument(
        "--init",
        action="store_true",
        help="explicitly home/calibrate before Move; do not use on every invocation",
    )
    parser.add_argument(
        "--skip-init",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--clear-error",
        action="store_true",
        help="send RESET before optional Init/Move",
    )
    parser.add_argument(
        "--stop-standard-controller",
        action="store_true",
        help="best-effort stop of the standard controller; this is not an exclusive raw lease",
    )
    parser.add_argument(
        "--restore-controller",
        action="store_true",
        help="restart the standard gripper controller on exit (normally keep it disabled in raw mode)",
    )
    args = parser.parse_args()
    values = (
        args.width_mm,
        args.velocity_mm_s,
        args.torque_nm,
        args.init_timeout,
        args.monitor_seconds,
        args.position_tolerance_mm,
    )
    if not all(math.isfinite(value) for value in values):
        parser.error("motion parameters must be finite")
    if args.init_timeout <= 0 or args.monitor_seconds <= 0 or args.position_tolerance_mm <= 0:
        parser.error("timeouts and position tolerance must be positive")
    if not MIN_WIDTH_MM <= args.width_mm <= MAX_WIDTH_MM:
        parser.error(f"--width-mm must be in [{MIN_WIDTH_MM}, {MAX_WIDTH_MM}]")
    if not MIN_VELOCITY_MM_S <= args.velocity_mm_s <= MAX_VELOCITY_MM_S:
        parser.error(f"--velocity-mm-s must be in [{MIN_VELOCITY_MM_S}, {MAX_VELOCITY_MM_S}]")
    if not MIN_TORQUE_NM <= args.torque_nm <= MAX_TORQUE_NM:
        parser.error(f"--torque-nm must be in [{MIN_TORQUE_NM}, {MAX_TORQUE_NM}]")

    side = sdk.EndToolSide.LEFT if args.side == "left" else sdk.EndToolSide.RIGHT
    joint_group = (
        sdk.S1JointGroup.left_gripper
        if args.side == "left"
        else sdk.S1JointGroup.right_gripper
    )
    controller_group = "left_gripper" if args.side == "left" else "right_gripper"
    robot = sdk.GalbotRobot()
    if not robot.init():
        raise RuntimeError("failed to initialize the S1 SDK")

    print(
        "WARNING: same-process raw TX is serialized, but there is no "
        "cross-client resource lease."
    )
    print("WBCS must run with endtool_raw_enabled=true.")
    print(
        f"For exclusive raw control, configure {args.side}_none instead of "
        f"{controller_group} in WBCS group_lists and restart WBCS. "
        "stop_controller() only stops a trajectory; it is not an exclusive raw lease."
    )
    if args.stop_standard_controller:
        stop_status = robot.stop_controller(controller_group)
        print(f"Best-effort stop of {controller_group}: {stop_status}")

    state_changed = threading.Condition()
    state = {
        "status": None,
        "last_generation": 0,
        "last_update": 0.0,
        "latest_raw_generation": 0,
        "idle_snapshots": 0,
        "reject_reasons": {},
        "non_increasing_generations": 0,
    }

    def on_raw(data: sdk.EndToolRawData) -> None:
        if data.side != side or data.kind != sdk.EndToolRxKind.BUFFER_250HZ:
            return
        reject_reason, status = decode_status(data.frame)
        with state_changed:
            state["latest_raw_generation"] = data.generation
            if reject_reason == "idle_snapshot":
                state["idle_snapshots"] += 1
                state_changed.notify_all()
                return
            if reject_reason is not None:
                reasons = state["reject_reasons"]
                reasons[reject_reason] = reasons.get(reject_reason, 0) + 1
                state_changed.notify_all()
                return
            previous_generation = state["last_generation"]
            if data.generation and previous_generation and data.generation <= previous_generation:
                state["non_increasing_generations"] += 1
                state_changed.notify_all()
                return
            state["status"] = status
            state["last_generation"] = data.generation
            state["last_update"] = time.monotonic()
            state_changed.notify_all()

    def get_snapshot():
        with state_changed:
            snapshot = dict(state)
            snapshot["reject_reasons"] = dict(state["reject_reasons"])
            return snapshot

    def wait_for_command_ready_after(baseline_generation: int):
        deadline = time.monotonic() + args.init_timeout
        last_print_at = 0.0
        last_printed_state = None
        while time.monotonic() < deadline:
            with state_changed:
                remaining = deadline - time.monotonic()
                state_changed.wait(timeout=max(0.0, min(0.5, remaining)))
                generation = state["last_generation"]
                status = state["status"]
                if status is None or generation <= baseline_generation:
                    continue
                now = time.monotonic()
                state_key = (status[3], status[4])
                if (
                    state_key != last_printed_state
                    or now - last_print_at >= STATUS_PRINT_INTERVAL_SECONDS
                ):
                    print("Init status:", format_status(generation, status))
                    last_printed_state = state_key
                    last_print_at = now
                # The device state machine defines 2 as Ready. State 4 is the
                # completion state of a previous Move report and must not be
                # used as proof that this Init command has completed.
                if status[3] == 2:
                    return True
        return False

    def wait_for_first_status(timeout: float):
        deadline = time.monotonic() + timeout
        with state_changed:
            while state["status"] is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                state_changed.wait(timeout=remaining)
            return state["status"]

    handle = robot.register_endtool_raw_callback(on_raw)
    if handle == 0:
        if args.restore_controller:
            robot.start_controller(controller_group)
        robot.destroy()
        raise RuntimeError("failed to register raw callback")

    try:
        if args.clear_error:
            result = robot.send_endtool_raw_frame(side, encode_clear_error())
            if result != sdk.ControlStatus.SUCCESS:
                raise RuntimeError("failed to publish the gripper RESET frame")
            print("RESET sent; waiting 0.5s before the next command...")
            time.sleep(0.5)

        if args.init and not args.skip_init:
            init_baseline = get_snapshot()["last_generation"]
            if robot.send_endtool_raw_frame(side, encode_init()) != sdk.ControlStatus.SUCCESS:
                raise RuntimeError("failed to publish the gripper init frame")
            print(
                "Waiting for a fresh Ready status (2) after Init "
                f"(timeout={args.init_timeout:.1f}s)..."
            )
            if not wait_for_command_ready_after(init_baseline):
                snapshot = get_snapshot()
                raise RuntimeError(
                    "gripper did not report a fresh command-ready status after Init; "
                    f"latest_valid_generation={snapshot['last_generation']} "
                    f"latest_raw_generation={snapshot['latest_raw_generation']} "
                    f"idle_snapshots={snapshot['idle_snapshots']} "
                    f"malformed={snapshot['reject_reasons']}"
                )

        current_status = wait_for_first_status(INITIAL_STATUS_WAIT_SECONDS)
        if current_status is not None and (
            current_status[4] == 0x01 or current_status[3] == 0
        ):
            raise RuntimeError(
                "gripper is not initialized (device error 0x01/state 0); "
                "rerun this example with --init before sending Move"
            )

        move_baseline = get_snapshot()["last_generation"]
        result = robot.send_endtool_raw_frame(
            side, encode_move(args.width_mm, args.velocity_mm_s, args.torque_nm)
        )
        if result != sdk.ControlStatus.SUCCESS:
            raise RuntimeError("failed to publish the gripper move frame")

        print(
            "Move sent. Monitoring only fresh passive status reports for "
            f"{args.monitor_seconds:.1f}s..."
        )
        deadline = time.monotonic() + args.monitor_seconds
        printed_generation = move_baseline
        received_after_move = False
        latest_move_status = None
        raw_target_reached = False
        last_raw_print = 0.0
        last_raw_state_error = None
        last_sdk_print = 0.0
        last_sdk_position_mm = None
        while time.monotonic() < deadline:
            with state_changed:
                remaining = deadline - time.monotonic()
                state_changed.wait(timeout=max(0.0, min(0.5, remaining)))
                generation = state["last_generation"]
                status = state["status"]
            if (
                status is not None
                and generation > move_baseline
                and generation != printed_generation
            ):
                received_after_move = True
                printed_generation = generation
                latest_move_status = status
                now = time.monotonic()
                state_error = (status[3], status[4])
                if (
                    state_error != last_raw_state_error
                    or now - last_raw_print >= STATUS_PRINT_INTERVAL_SECONDS
                ):
                    print("Raw status snapshot:", format_status(generation, status))
                    last_raw_print = now
                    last_raw_state_error = state_error
                if (
                    status[4] == 0
                    and abs(status[0] - args.width_mm) <= args.position_tolerance_mm
                    and abs(status[1]) <= STOPPED_VELOCITY_TOLERANCE_MM_S
                ):
                    print(
                        f"Move completed within tolerance: requested={args.width_mm:.3f}mm "
                        f"actual={status[0]:.3f}mm (verified by Raw status)"
                    )
                    raw_target_reached = True
                    break

            sdk_state = robot.get_gripper_state(joint_group)
            if sdk_state is None:
                continue
            position_mm = sdk_state.width * 1000.0
            velocity_mm_s = sdk_state.velocity * 1000.0
            now = time.monotonic()
            if (
                last_sdk_position_mm is None
                or abs(position_mm - last_sdk_position_mm) >= 0.1
                or now - last_sdk_print >= 0.5
            ):
                print(
                    f"SDK state: position={position_mm:.3f}mm "
                    f"velocity={velocity_mm_s:.3f}mm/s effort={sdk_state.effort:.3f} "
                    f"is_moving={sdk_state.is_moving}",
                )
                last_sdk_position_mm = position_mm
                last_sdk_print = now
            if (
                abs(position_mm - args.width_mm) <= args.position_tolerance_mm
                and not sdk_state.is_moving
            ):
                print(
                    f"Move completed within tolerance: requested={args.width_mm:.3f}mm "
                    f"actual={position_mm:.3f}mm (verified by get_gripper_state)"
                )
                break

        snapshot = get_snapshot()
        if not received_after_move:
            print(
                "WARNING: no valid gripper status snapshot was received after Move. "
                "Publication success alone does not prove that the device executed the command."
            )
        elif time.monotonic() - snapshot["last_update"] > STATUS_STALE_SECONDS:
            print(
                "WARNING: the valid gripper status stream stopped; subsequent receive "
                "buffers were idle or rejected."
            )
        print(
            "RX summary: "
            f"latest_raw_generation={snapshot['latest_raw_generation']} "
            f"latest_valid_generation={snapshot['last_generation']} "
            f"idle_snapshots={snapshot['idle_snapshots']} "
            f"malformed={snapshot['reject_reasons']} "
            f"non_increasing_generations={snapshot['non_increasing_generations']}"
        )

        if raw_target_reached:
            return

        sdk_state = robot.get_gripper_state(joint_group)
        sdk_target_reached = False
        if sdk_state is not None:
            position_mm = sdk_state.width * 1000.0
            position_error = abs(position_mm - args.width_mm)
            sdk_target_reached = (
                position_error <= args.position_tolerance_mm and not sdk_state.is_moving
            )
        if not sdk_target_reached:
            raw_summary = (
                "none"
                if latest_move_status is None
                else format_status(printed_generation, latest_move_status)
            )
            sdk_summary = (
                "unavailable"
                if sdk_state is None
                else (
                    f"position={position_mm:.3f}mm error={position_error:.3f}mm "
                    f"is_moving={sdk_state.is_moving}"
                )
            )
            raise RuntimeError(
                "self-developed gripper verification failed: Move did not reach the requested "
                f"width: requested={args.width_mm:.3f}mm "
                f"tolerance={args.position_tolerance_mm:.3f}mm; "
                f"SDK state={sdk_summary}; last_raw_status={raw_summary}"
            )
    finally:
        robot.unregister_endtool_raw_callback(handle)
        if args.restore_controller:
            restore_status = robot.start_controller(controller_group)
            print(f"Best-effort restore of {controller_group}: {restore_status}")
        robot.destroy()


def run_cli() -> int:
    try:
        main()
    except RuntimeError as error:
        print(f"FAILED: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(run_cli())