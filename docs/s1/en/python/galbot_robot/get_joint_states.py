import time
from galbot_sdk.s1 import GalbotRobot

def print_joint_states(joint_states):
    """
    joint_state_vec: List of JointState; each object has position, velocity, acceleration, effort, current
    """
    for js in joint_states:
        print(f" : position = {js.position} , velocity = {js.velocity} "
            f", acceleration = {js.acceleration} , effort = {js.effort} , current = {js.current}")

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