import time
from galbot_sdk.g3 import GalbotRobot, G3JointGroup

def print_suction_cup_state(suction_cup_state):
    """
    suction_cup_state: object including timestamp_ns, pressure, activation, action_state
    """
    group_name = joint_group.name
    print(f"Timestamp (ns): {suction_cup_state.timestamp_ns}")
    print(
        f"pressure {suction_cup_state.pressure} "
        f"activation {suction_cup_state.activation} "
        f"action state {int(suction_cup_state.action_state)}"
    )

# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()

# Program started, waiting for data
time.sleep(1)
print("Initialization succeeded")

# Set suction cup joint group (right suction cup)
joint_group = G3JointGroup.right_suction_cup

# Get suction cup state
suction_cup_state = robot.get_suction_cup_state(joint_group)

if suction_cup_state is None:
    print("get suction cup error")
else:
    print("Right suction cup status:")
    print_suction_cup_state(suction_cup_state)

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')