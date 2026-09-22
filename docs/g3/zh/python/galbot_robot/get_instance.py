from galbot_sdk.g3 import GalbotRobot
import time

# Get GalbotRobot
robot = GalbotRobot()

state = robot.init()
if not state:
    print("Initialization failed")
else:
    print("Initialization succeeded")

while robot.is_running():
    # business logic
    time.sleep(1)
    break

# Send exit signal to exit the program
robot.request_shutdown()
# Wait for exit state
robot.wait_for_shutdown()
# Release SDK-related resources
robot.destroy()