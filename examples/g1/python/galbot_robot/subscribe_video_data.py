import threading
import time

try:
    from galbot_sdk.g1 import GalbotRobot, SensorStatus, SensorType
except ImportError:
    print("import galbot_sdk failed, please install it first or check if it is in the PYTHONPATH")
    exit(1)


OUTPUT_PATH = "head_left.h264"
DURATION = 10

output_file = None
frame_count = 0
frame_count_lock = threading.Lock()


# Important:
# The video callback is executed by the SDK internal dispatch thread. 
# Do not perform time-consuming operations in this callback; keep it 
# short and non-blocking. Long-running work may delay video frame delivery 
# for this subscription.
def video_data_callback(video_data: dict):
    global frame_count

    header = video_data.get("header", {})
    data = video_data.get("data", b"")
    if output_file is not None and data:
        # This sample callback only writes the raw H.264 stream to file without expensive transcoding here.
        output_file.write(data)

    with frame_count_lock:
        frame_count += 1
        current_count = frame_count

    if current_count == 1 or current_count % 30 == 0:
        print(
            "frame_count={}, timestamp_ns={}, format={}, bytes={}".format(
                current_count,
                header.get("timestamp_ns"),
                video_data.get("format"),
                len(data),
            )
        )


def main():
    global output_file

    robot = GalbotRobot()

    # Enable only the camera used by this example.
    if not robot.init({SensorType.HEAD_LEFT_CAMERA}):
        print("GalbotRobot initialization failed")
        return
    print("Initialization succeeded")

    output_file = open(OUTPUT_PATH, "wb")
    print(f"writing H.264 stream to {OUTPUT_PATH}")

    status = robot.subscribe_video_data(SensorType.HEAD_LEFT_CAMERA, video_data_callback)
    if status != SensorStatus.SUCCESS:
        print(f"subscribe_video_data failed, status={status}")
        output_file.close()
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return
    print("subscribe_video_data success")

    try:
        time.sleep(DURATION)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        # Unsubscribe clears the user callback without recreating SDK reader resources.
        status = robot.unsubscribe_video_data(SensorType.HEAD_LEFT_CAMERA)
        print(f"unsubscribe_video_data status={status}")

        if output_file is not None:
            output_file.close()

        with frame_count_lock:
            total_frames = frame_count
        print(f"received {total_frames} frames")
        print(f"saved H.264 stream to {OUTPUT_PATH}")

        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        print("Resources released successfully")

if __name__ == "__main__":
    main()
