import time

from galbot_sdk.g1 import (
    GalbotRobot,
    JointState,
    JointStateMessage,
    RgbData,
    SensorType,
    SyncedObservation,
)

NS_TO_MS = 1e-6


def main():
    # 1) Initialize robot with sync mode enabled.
    #    The 2nd init arg must be True, otherwise get_synced_observation() is unavailable.
    robot = GalbotRobot()
    ok = robot.init(
        {
            SensorType.HEAD_LEFT_CAMERA,
            SensorType.LEFT_ARM_CAMERA,
            SensorType.RIGHT_ARM_CAMERA,
        },
        True,  # enable_sync_mode
    )
    if not ok:
        print("init failed")
        robot.destroy()
        return

    time.sleep(2)

    # 2) Build camera list for synchronization.
    #    cameras[0] is the anchor camera; other cameras are matched by nearest timestamp.
    cameras = [
        SensorType.LEFT_ARM_CAMERA,  # anchor
        SensorType.RIGHT_ARM_CAMERA,  # nearest-neighbor
        SensorType.HEAD_LEFT_CAMERA,  # nearest-neighbor
    ]

    # 3) Call API (typed return).
    obs: SyncedObservation | None = robot.get_synced_observation(cameras, True)
    if not obs:
        print("get_synced_observation failed")
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return

    # 4) Read synced RGB frames from typed field: obs.rgb_data_map.
    #    Sync-mode RGB payloads are tightly packed CPU-owned NV12, not JPEG.
    print("[Synced RGB]")
    rgb_map: dict[SensorType, RgbData] = obs.rgb_data_map
    anchor_camera = cameras[0]
    anchor_frame: RgbData | None = rgb_map.get(anchor_camera, None)
    anchor_ts_ns = (
        anchor_frame.header.timestamp_ns if anchor_frame is not None else None
    )
    print(f"anchor_camera={anchor_camera}, anchor_timestamp_ns={anchor_ts_ns}")

    for cam in cameras:
        frame: RgbData | None = rgb_map.get(cam, None)
        cam_ts_ns = frame.header.timestamp_ns if frame is not None else None
        frame_format = frame.format if frame is not None else "N/A"
        dimensions = f"{frame.width}x{frame.height}" if frame is not None else "N/A"
        payload_bytes = len(frame.data) if frame is not None else 0
        delta_ms_str = "N/A"
        if anchor_ts_ns is not None and cam_ts_ns is not None:
            delta_ms_str = f"{(cam_ts_ns - anchor_ts_ns) * NS_TO_MS:.3f}"
        print(
            f"camera={cam}, timestamp_ns={cam_ts_ns if cam_ts_ns is not None else 'N/A'}, "
            f"delta_to_anchor_ms={delta_ms_str}, format={frame_format}, "
            f"dimensions={dimensions}, bytes={payload_bytes}"
        )

    # 5) Read optional joint snapshot from typed field: obs.joint_state
    #    joint_state is JointStateMessage | None.
    joint_state: JointStateMessage | None = obs.joint_state
    print("[Joint]")
    joint_ts_ns = joint_state.timestamp_ns if joint_state is not None else None
    joint_delta_ms_str = "N/A"
    if anchor_ts_ns is not None and joint_ts_ns is not None:
        joint_delta_ms_str = f"{(joint_ts_ns - anchor_ts_ns) * NS_TO_MS:.3f}"
    print(
        "joint_timestamp_ns:",
        joint_ts_ns if joint_ts_ns is not None else "N/A",
        "delta_to_anchor_ms:",
        joint_delta_ms_str,
    )
    # 6) Each element in joint_state_vec is typed JointState and contains joint_name.
    joint_state_vec: list[JointState] = (
        joint_state.joint_state_vec if joint_state is not None else []
    )
    print("joint_count:", len(joint_state_vec))
    for i, js in enumerate(joint_state_vec[:5]):
        print(
            f"  [{i}] joint_name={js.joint_name}, "
            f"position={js.position}, "
            f"velocity={js.velocity}, "
            f"effort={js.effort}"
        )

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()


if __name__ == "__main__":
    main()
