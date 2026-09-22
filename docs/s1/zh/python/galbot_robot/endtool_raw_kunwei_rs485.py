"""Raw-passthrough active-query example for a Kunwei six-axis RS485 force sensor."""

import argparse
import math
import struct
import threading
import time

import galbot_sdk.s1 as sdk


MAX_ABS_WRENCH_VALUE = 1.0e6
STALE_SECONDS = 1.0
FREEZE_SECONDS = 2.0
WARNING_INTERVAL_SECONDS = 1.0
STATS_INTERVAL_SECONDS = 5.0


def encode_query() -> bytes:
    frame = bytearray(64)
    struct.pack_into("<III", frame, 0, 2, 233, 4)
    frame[12:16] = b"\x49\xaa\x0d\x0a"
    return bytes(frame)


def decode_force(frame: bytes):
    if len(frame) != 64:
        return "size", None
    if frame[20] not in (0x48, 0x49) or frame[21] != 0xAA:
        return "prefix", None
    if frame[46] != 0x0D or frame[47] != 0x0A:
        return "terminator", None

    reading = struct.unpack_from("<6f", frame, 22)
    if any(not math.isfinite(value) or abs(value) > MAX_ABS_WRENCH_VALUE for value in reading):
        return "numeric", None
    if all(value == 0.0 for value in reading):
        return "zero", None
    return None, reading


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--query-interval-ms", type=float, default=10.0)
    parser.add_argument("--print-interval-ms", type=float, default=100.0)
    args = parser.parse_args()
    values = (args.duration, args.query_interval_ms, args.print_interval_ms)
    if not all(math.isfinite(value) and value > 0 for value in values):
        parser.error("duration and intervals must be finite and positive")

    side = sdk.EndToolSide.LEFT if args.side == "left" else sdk.EndToolSide.RIGHT
    robot = sdk.GalbotRobot()
    if not robot.init():
        raise RuntimeError("failed to initialize the S1 SDK")
    print(
        "WARNING: same-process raw TX is serialized, but there is no "
        "cross-client resource lease."
    )
    print("WBCS must run with endtool_raw_enabled=true.")

    state_lock = threading.Lock()
    state = {
        "query_ok": 0,
        "query_fail": 0,
        "rx_1khz": 0,
        "rejected": 0,
        "reject_size": 0,
        "reject_prefix": 0,
        "reject_terminator": 0,
        "reject_numeric": 0,
        "reject_zero": 0,
        "non_increasing_generation": 0,
        "duplicate_snapshots": 0,
        "valid": 0,
        "last_generation": 0,
        "last_valid": 0.0,
        "last_print": 0.0,
        "last_payload": None,
        "last_payload_change": 0.0,
        "last_freeze_warning": 0.0,
        "last_stale_warning": 0.0,
    }

    def on_raw(data: sdk.EndToolRawData) -> None:
        if data.side != side or data.kind != sdk.EndToolRxKind.BUFFER_1KHZ:
            return
        reject_reason, reading = decode_force(data.frame)
        now = time.monotonic()
        freeze_warning = None
        print_reading = False
        with state_lock:
            state["rx_1khz"] += 1
            if reject_reason is not None:
                state["rejected"] += 1
                state[f"reject_{reject_reason}"] += 1
                return

            previous_generation = state["last_generation"]
            if previous_generation and data.generation <= previous_generation:
                state["non_increasing_generation"] += 1
                return
            state["last_generation"] = data.generation

            payload = bytes(data.frame[22:46])
            if state["last_payload"] is not None and payload == state["last_payload"]:
                # The 1 kHz channel is a transport snapshot. Its generation can
                # advance while the TIB keeps replaying the last sensor payload.
                # A duplicate therefore is not a fresh force sample.
                state["duplicate_snapshots"] += 1
                if (
                    now - state["last_payload_change"] >= FREEZE_SECONDS
                    and now - state["last_freeze_warning"] >= WARNING_INTERVAL_SECONDS
                ):
                    state["last_freeze_warning"] = now
                    freeze_warning = now - state["last_payload_change"]
                return_after_lock = True
            else:
                state["last_payload"] = payload
                state["last_payload_change"] = now
                return_after_lock = False

            if not return_after_lock:
                state["last_valid"] = now
                state["valid"] += 1
                if now - state["last_print"] >= args.print_interval_ms / 1000.0:
                    state["last_print"] = now
                    print_reading = True

        if freeze_warning is not None:
            print(
                f"WARNING: force payload unchanged for {freeze_warning:.3f}s while "
                "generation advances. This may be a held TIB snapshot; an exactly "
                "steady sensor can also produce identical bytes."
            )
        if return_after_lock:
            return
        if print_reading:
            fx, fy, fz, mx, my, mz = reading
            print(
                f"generation={data.generation} Fx={fx:.4f} Fy={fy:.4f} Fz={fz:.4f}kg "
                f"Mx={mx:.4f} My={my:.4f} Mz={mz:.4f}kg*m"
            )

    handle = robot.register_endtool_raw_callback(on_raw)
    if handle == 0:
        robot.destroy()
        raise RuntimeError("failed to register raw callback")

    query = encode_query()
    query_interval = args.query_interval_ms / 1000.0
    started_at = time.monotonic()
    deadline = started_at + args.duration
    next_query_at = started_at
    next_stats_at = started_at + STATS_INTERVAL_SECONDS
    last_stats_at = started_at
    last_stats_query_count = 0
    last_stats_rx_count = 0
    try:
        while True:
            now = time.monotonic()
            if now < next_query_at:
                time.sleep(next_query_at - now)
            if time.monotonic() >= deadline:
                break

            result = robot.send_endtool_raw_frame(side, query)
            after_query = time.monotonic()
            with state_lock:
                if result == sdk.ControlStatus.SUCCESS:
                    state["query_ok"] += 1
                else:
                    state["query_fail"] += 1
                    failures = state["query_fail"]

                last_valid = state["last_valid"]
                stale_for = after_query - (last_valid if last_valid else started_at)
                stale_warning = None
                if (
                    stale_for >= STALE_SECONDS
                    and after_query - state["last_stale_warning"] >= WARNING_INTERVAL_SECONDS
                ):
                    state["last_stale_warning"] = after_query
                    stale_warning = stale_for
                snapshot = dict(state)

            if result != sdk.ControlStatus.SUCCESS and (failures == 1 or failures % 100 == 0):
                print(f"Query publish failed: {result}, failures={failures}")
            if stale_warning is not None:
                print(
                    f"WARNING: no fresh force response for {stale_warning:.3f}s: "
                    f"query_ok={snapshot['query_ok']} query_fail={snapshot['query_fail']} "
                    f"rx_1khz={snapshot['rx_1khz']} valid={snapshot['valid']}. "
                    "Check TIB, sensor power/wiring/baud rate, "
                    "endtool_raw_enabled and WBCS logs."
                )

            next_query_at += query_interval
            # Do not catch up with a burst after Publish blocks.
            if next_query_at <= after_query:
                next_query_at = after_query + query_interval

            if after_query >= next_stats_at:
                elapsed = after_query - last_stats_at
                print(
                    f"Rate: query_hz="
                    f"{(snapshot['query_ok'] - last_stats_query_count) / elapsed:.1f} "
                    f"rx_hz={(snapshot['rx_1khz'] - last_stats_rx_count) / elapsed:.1f} "
                    f"fresh={snapshot['valid']} duplicates={snapshot['duplicate_snapshots']} "
                    f"rejects(size={snapshot['reject_size']} "
                    f"prefix={snapshot['reject_prefix']} "
                    f"terminator={snapshot['reject_terminator']} "
                    f"numeric={snapshot['reject_numeric']} zero={snapshot['reject_zero']})"
                )
                last_stats_query_count = snapshot["query_ok"]
                last_stats_rx_count = snapshot["rx_1khz"]
                last_stats_at = after_query
                next_stats_at = after_query + STATS_INTERVAL_SECONDS
    finally:
        robot.unregister_endtool_raw_callback(handle)
        with state_lock:
            snapshot = dict(state)
        print(
            f"Stopped: query_ok={snapshot['query_ok']} query_fail={snapshot['query_fail']} "
            f"rx_1khz={snapshot['rx_1khz']} rejected={snapshot['rejected']} "
            f"(size={snapshot['reject_size']} prefix={snapshot['reject_prefix']} "
            f"terminator={snapshot['reject_terminator']} numeric={snapshot['reject_numeric']} "
            f"zero={snapshot['reject_zero']} "
            "non_increasing_generation="
            f"{snapshot['non_increasing_generation']}) fresh={snapshot['valid']} "
            f"duplicates={snapshot['duplicate_snapshots']}"
        )
        robot.destroy()


if __name__ == "__main__":
    main()
