import time
from galbot_sdk.g3 import GalbotRobot, G3JointGroup

def print_gripper_state(joint_group, gripper_state):
    """
    joint_group: G3JointGroup enum value
    gripper_state: object including timestamp_ns, width, velocity, effort, is_moving
    """
    print(f"Timestamp (ns): {gripper_state.timestamp_ns}")
    print(
        f"width {gripper_state.width} "
        f"velocity {gripper_state.velocity} "
        f"effort {gripper_state.effort} "
        f"is moving {gripper_state.is_moving}"
    )

# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()

# Program started, waiting for data
time.sleep(1)
print("Initialization succeeded")

# Set gripper joint group (left gripper)
joint_group = G3JointGroup.left_gripper

# Get gripper state
gripper_state = robot.get_gripper_state(joint_group)

if gripper_state is None:
    print("get gripper state error")
else:
    print("Left gripper state is as follows:")
    print_gripper_state(joint_group, gripper_state)

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')