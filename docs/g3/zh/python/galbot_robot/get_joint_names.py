import time
from galbot_sdk.g3 import GalbotRobot

# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()
# Program started, waiting for data
time.sleep(1)
print("Initialization succeeded")

# Get specified joint names; joint groups include ["leg", "head", "left_arm", "right_arm"]
joint_group_names = ["head"]
only_active_joint = True  # Get active joints
head_joint_names = robot.get_joint_names(only_active_joint, joint_group_names)
print("Head joint names:")
for i, name in enumerate(head_joint_names):
    print(f"{i}: {name}")

# Passing an empty list returns all joint group information by default
all_joint_names = robot.get_joint_names(only_active_joint, [])
print("All joint names:")
for i, name in enumerate(all_joint_names):
    print(f"{i}: {name}")

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')
