import time
from galbot_sdk.g1 import GalbotRobot

def print_joint_states(joint_states):
    """
    joint_state_vec: List of JointState; each object has
    joint_name, position, velocity, acceleration, effort, current
    """
    for js in joint_states:
        print(f" : joint_name = {js.joint_name} , position = {js.position} , velocity = {js.velocity} "
            f", acceleration = {js.acceleration} , effort = {js.effort} , current = {js.current}"
            f", timestamp_ns = {js.timestamp_ns}")

# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()
# Program started, waiting for data
time.sleep(1)
print("Initialization succeeded")
# Get joint states by joint group names; returns all joints if empty
joint_group_names = ["left_arm"]
ret = robot.get_joint_states(joint_group_names, [])
if not ret:
    print(f"No joint states returned for joint groups {joint_group_names}")
else:
    print_joint_states(ret)

# Get specified joint states; if provided, overrides joint group input
joint_names = ["left_arm_joint1", "left_arm_joint2"]
state_ret = robot.get_joint_states([], joint_names)
if not state_ret:
    print(f"No joint states returned for joint names {joint_names}")
else:
    print_joint_states(state_ret)

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')
