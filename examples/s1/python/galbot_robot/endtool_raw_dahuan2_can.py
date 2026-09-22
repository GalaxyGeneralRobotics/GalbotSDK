"""Raw-passthrough example for a Dahuan two-finger CAN FD gripper."""

import argparse
import struct
import sys
import threading
import time

import galbot_sdk.s1 as sdk


TX_CAN_ID = 0x601
RX_CAN_ID = 0x201
WRITE_SINGLE_REGISTER = 0x06
READ_REGISTERS = 0x03
REGISTER_INIT = 0x0100
REGISTER_FORCE = 0x0101
REGISTER_TARGET_POSITION = 0x0103
REGISTER_SPEED = 0x0104
REGISTER_STATUS_BASE = 0x0200

ERROR_TEXT = {
    0: "no error",
    1: "under voltage",
    2: "over voltage",
    3: "over current",
    4: "over heat",
    5: "motor disconnected",
    8: "overload",
    11: "over speed",
    15: "startup error",
    32: "encoder error",
}


def encode_request(function: int, address: int, value: int) -> bytes:
    frame = bytearray(64)
    struct.pack_into("<II", frame, 0, 1, TX_CAN_ID)
    frame[8] = 7
    struct.pack_into(">BHH", frame, 12, function, address, value)
    return bytes(frame)


def decode_status(frame: bytes):
    if len(frame) != 64:
        return "size", None
    if struct.unpack_from("<I", frame, 0)[0] != 1:
        return "frame_type", None
    if frame[12] != 0x01:
        return "idle_snapshot", None
    if struct.unpack_from("<I", frame, 4)[0] != RX_CAN_ID:
        return "can_id", None
    if frame[16] != READ_REGISTERS or frame[17] < 12:
        return "function", None
    values = struct.unpack_from(">6H", frame, 18)
    current_ma = values[4] - 65536 if values[4] >= 32768 else values[4]
    return None, (*values[:4], current_ma, values[5])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--position", type=int, default=500, help="target position in [0, 1000] permille")
    parser.add_argument("--speed", type=int, default=50, help="speed in [1, 100] percent")
    parser.add_argument("--force", type=int, default=50, help="force in [20, 100] percent")
    parser.add_argument("--init", action="store_true", help="home before setting motion parameters")
    parser.add_argument("--calibrate", action="store_true")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--init-wait", type=float, default=4.0)
    parser.add_argument("--position-tolerance", type=int, default=20)
    parser.add_argument("--stop-standard-controller", action="store_true")
    parser.add_argument("--restore-controller", action="store_true")
    args = parser.parse_args()
    if not 0 <= args.position <= 1000:
        parser.error("--position must be in [0, 1000]")
    if not 1 <= args.speed <= 100:
        parser.error("--speed must be in [1, 100]")
    if not 20 <= args.force <= 100:
        parser.error("--force must be in [20, 100]")
    if args.duration < 0 or args.init_wait < 0 or args.position_tolerance < 0:
        parser.error("--duration and --init-wait must be non-negative")

    side = sdk.EndToolSide.LEFT if args.side == "left" else sdk.EndToolSide.RIGHT
    controller_group = "left_gripper" if args.side == "left" else "right_gripper"
    robot = sdk.GalbotRobot()
    if not robot.init():
        raise RuntimeError("failed to initialize the S1 SDK")

    print("WARNING: raw TX has no server-side exclusive writer lock.")
    print("WBCS must run with endtool_raw_enabled=true in [robot_info.custom_params].")
    if args.stop_standard_controller:
        print(f"Best-effort stop of {controller_group}: {robot.stop_controller(controller_group)}")
    changed = threading.Condition()
    state = {
        "status": None,
        "status_generation": 0,
        "status_kind": None,
        "generation_by_kind": [0, 0],
        "latest_raw_generation_by_kind": [0, 0],
        "valid": 0,
        "idle": 0,
        "non_increasing_generation": 0,
        "rejected": {},
    }

    def on_raw(data: sdk.EndToolRawData) -> None:
        # Dahuan responses may appear in either receive buffer.
        if data.side != side:
            return
        kind_index = 0 if data.kind == sdk.EndToolRxKind.BUFFER_1KHZ else 1
        reason, status = decode_status(data.frame)
        with changed:
            state["latest_raw_generation_by_kind"][kind_index] = data.generation
            if reason == "idle_snapshot":
                state["idle"] += 1
            elif reason is not None:
                state["rejected"][reason] = state["rejected"].get(reason, 0) + 1
            else:
                previous_generation = state["generation_by_kind"][kind_index]
                if (
                    data.generation
                    and previous_generation
                    and data.generation <= previous_generation
                ):
                    state["non_increasing_generation"] += 1
                    changed.notify_all()
                    return
                state["generation_by_kind"][kind_index] = data.generation
                state["status"] = status
                state["status_generation"] = data.generation
                state["status_kind"] = data.kind
                state["valid"] += 1
                init, grip, position, speed, current, error = status
                print(
                    f"Fresh status: kind={data.kind} generation={data.generation} "
                    f"init={init} grip={grip} position={position}permille "
                    f"speed={speed} current={current}mA error=0x{error:04x} "
                    f"({ERROR_TEXT.get(error, 'unknown error')})"
                )
            changed.notify_all()

    handle = robot.register_endtool_raw_callback(on_raw)
    if handle == 0:
        if args.restore_controller:
            robot.start_controller(controller_group)
        robot.destroy()
        raise RuntimeError("failed to register raw callback")

    def publish(frame: bytes, description: str) -> None:
        result = robot.send_endtool_raw_frame(side, frame)
        if result != sdk.ControlStatus.SUCCESS:
            raise RuntimeError(f"failed to publish {description}: {result}")

    def poll(seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            publish(encode_request(READ_REGISTERS, REGISTER_STATUS_BASE, 6), "status request")
            with changed:
                changed.wait(timeout=min(0.5, max(0.0, deadline - time.monotonic())))

    try:
        if args.init or args.calibrate:
            init_value = 0x00A5 if args.calibrate else 0x0001
            publish(encode_request(WRITE_SINGLE_REGISTER, REGISTER_INIT, init_value), "init command")
            poll(args.init_wait)
        publish(encode_request(WRITE_SINGLE_REGISTER, REGISTER_SPEED, args.speed), "speed command")
        publish(encode_request(WRITE_SINGLE_REGISTER, REGISTER_FORCE, args.force), "force command")
        publish(
            encode_request(WRITE_SINGLE_REGISTER, REGISTER_TARGET_POSITION, args.position),
            "position command",
        )
        # Discard any status left by initialization polling. Only a response to
        # the active queries below may verify this motion command.
        with changed:
            state["status"] = None
            motion_status_baseline = state["valid"]
        poll(args.duration)
        with changed:
            status = state["status"]
            snapshot = dict(state)
            snapshot["rejected"] = dict(state["rejected"])
        if status is None or snapshot["valid"] <= motion_status_baseline:
            raise RuntimeError(
                "Dahuan2 device verification failed: no fresh Dahuan2 status response "
                "was received after the position command. The installed end tool may not "
                "be a Dahuan2 gripper, or its power/wiring/baud rate may be incorrect; "
                "latest_raw_generation_by_kind="
                f"{snapshot['latest_raw_generation_by_kind']} "
                f"idle_snapshots={snapshot['idle']} rejected={snapshot['rejected']}"
            )
        _, _, position, _, _, error = status
        if error != 0:
            raise RuntimeError(
                f"gripper status error=0x{error:04x} "
                f"({ERROR_TEXT.get(error, 'unknown error')})"
            )
        if abs(position - args.position) > args.position_tolerance:
            raise RuntimeError(
                "position did not reach target: "
                f"requested={args.position} actual={position} "
                f"tolerance={args.position_tolerance}permille"
            )
        print(
            f"Position reached: requested={args.position} actual={position}permille "
            f"fresh_status_count={snapshot['valid']} "
            f"non_increasing_generations={snapshot['non_increasing_generation']}"
        )
    finally:
        robot.unregister_endtool_raw_callback(handle)
        if args.restore_controller:
            robot.start_controller(controller_group)
        robot.destroy()


def run_cli() -> int:
    try:
        main()
    except RuntimeError as error:
        # Device absence, protocol mismatch and transport failures are expected
        # test outcomes. Report them without a Python traceback while keeping a
        # non-zero exit status for scripts and CI.
        print(f"FAILED: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(run_cli())
