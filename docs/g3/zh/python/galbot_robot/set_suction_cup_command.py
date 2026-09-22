from galbot_sdk.g3 import GalbotRobot
from galbot_sdk.g3 import G3JointGroup, ControlStatus
import time

# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()
time.sleep(1)
print('Initialization succeeded')

# Set suction cup joint group (right suction cup)
joint_group = G3JointGroup.right_suction_cup

# Whether to activate suction cup
activate = True  # True: activate suction cup, False: deactivate suction cup

# Send suction cup control command
status = robot.set_suction_cup_command(
    joint_group,
    activate
)

# Check execution results
if status != ControlStatus.SUCCESS:
    print("Set suction cup failed")
else:
    print("Set suction cup succeeded")

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')