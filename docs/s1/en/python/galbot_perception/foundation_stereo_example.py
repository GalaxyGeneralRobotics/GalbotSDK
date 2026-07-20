"""Foundation stereo depth example (S1): single run_once, save a pseudo-color depth image."""

import time

import cv2
import numpy as np

try:
    from galbot_sdk import (
        GalbotPerception,
        GalbotRobot,
        MachineType,
        PerceptionModule,
    )
except ImportError:
    print("Failed to import galbot_sdk, please install it first or check if it is in PYTHONPATH")
    raise

OUTPUT_IMAGE_PATH = "foundation_stereo_depth.png"


def main():
    robot = GalbotRobot.get_instance(MachineType.S1)
    try:
        if not robot.init():
            print("Robot init failed")
            return
        print("Robot init OK")

        perception = GalbotPerception.get_instance(MachineType.S1)
        if not perception.init({PerceptionModule.FOUNDATION_STEREO, PerceptionModule.LIGHT_STEREO}):
            print("Perception init failed")
            return
        print("Perception init OK")

        time.sleep(12)  # Wait for perception models to load
        print("Triggering single inference...")

        if not perception.run_once(PerceptionModule.FOUNDATION_STEREO):
            print("run_once failed to send command")
            return

        print("Waiting for inference result...")
        if not perception.wait_for_new_result(
            PerceptionModule.FOUNDATION_STEREO, timeout_s=6.0
        ):
            print("Timed out waiting for inference result")
            return

        ok, result = perception.get_latest_result(PerceptionModule.FOUNDATION_STEREO)
        if not ok:
            print("get_latest_result failed")
            return

        print(result.get_result_info())

        depth_map = result.instance_mask
        if depth_map is None:
            print("No depth map (instance_mask is empty)")
            return

        print(f"Depth map shape: {depth_map.shape}, dtype: {depth_map.dtype}")
        print(f"Depth value range: [{np.nanmin(depth_map)}, {np.nanmax(depth_map)}]")

        depth_f = depth_map.astype(np.float32)
        valid = depth_f[depth_f > 0]
        if valid.size > 0:
            vmin, vmax = np.percentile(valid, [1, 99])
            normalized = np.clip((depth_f - vmin) / (vmax - vmin + 1e-6), 0, 1)
        else:
            normalized = np.zeros_like(depth_f)
        colored = cv2.applyColorMap(
            (normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO
        )

        if cv2.imwrite(OUTPUT_IMAGE_PATH, colored):
            print(f"Saved: {OUTPUT_IMAGE_PATH}")
        else:
            print(f"Failed to save: {OUTPUT_IMAGE_PATH}")
    finally:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()


if __name__ == "__main__":
    main()
