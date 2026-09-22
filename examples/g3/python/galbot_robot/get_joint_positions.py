import time
from galbot_sdk.g3 import GalbotRobot

def print_joint_positions(joint_positions):
    print(f"pos count is {len(joint_positions)}")
    for pos in joint_positions:
        print(pos)

# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()
# Program started, waiting for data
time.sleep(1)
print("Initialization succeeded")

# Get joint positions by joint group names; returns all joints if empty
joint_group_names = ["left_arm"]
ret = robot.get_joint_positions(joint_group_names, [])
print("Left arm joint positions:")
print_joint_positions(ret)
# Get specified joint positions; if provided, overrides joint group input
joint_names = ["left_arm_joint1", "left_arm_joint2"]
state_ret = robot.get_joint_positions([], joint_names)
print("Left arm joints 1 and 2 positions:")
print_joint_positions(state_ret)

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')