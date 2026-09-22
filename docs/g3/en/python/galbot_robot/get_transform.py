import time
from galbot_sdk.g3 import GalbotRobot

def print_pose(pose_vec):
    """
    pose_vec: list of floats
    """
    print("pose_vec = [" + ", ".join(str(p) for p in pose_vec) + "]")

# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()

# Program started, waiting for data
time.sleep(1)
print("Initialization succeeded")

# Set target frame and source frame
target_frame = "base_link"
source_frame = "left_arm_link1"
timestamp_ns = 0    # 0 means fetch the latest TF transform value

# Get coordinate transform
ret_val = robot.get_transform(target_frame, source_frame)

if not ret_val[0]:
    print("get_transform error")
else:
    print("tf_timestamp_ns:", ret_val[1])
    print_pose(ret_val[0])

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')