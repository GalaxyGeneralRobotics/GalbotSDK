import time
from galbot_sdk.g3 import GalbotRobot, G3JointGroup, ControlStatus

# Get and initialize the GalbotRobot singleton
robot = GalbotRobot()
robot.init()
print('Initialization succeeded')
# Program started, waiting for data
time.sleep(2)

# Set left gripper width to 0.02m, speed 0.05m, torque 10N, block until gripper reaches position
status = robot.set_gripper_command(
    G3JointGroup.left_gripper, 0.02, 0.05, 10, True
)
if status != ControlStatus.SUCCESS:
    print("Set gripper failed")
else:
    print('Set gripper success')

# Set left gripper width to 0.1m, speed 0.05m, torque 10N, block until gripper reaches position
status = robot.set_gripper_command(
    G3JointGroup.left_gripper, 0.1, 0.05, 10, True
)

if status != ControlStatus.SUCCESS:
    print("Set gripper failed")
else:
    print('Set gripper success')

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')