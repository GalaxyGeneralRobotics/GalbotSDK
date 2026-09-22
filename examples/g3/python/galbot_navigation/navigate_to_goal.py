from galbot_sdk.g3 import GalbotNavigation
from galbot_sdk.g3 import GalbotRobot
from galbot_sdk.g3 import ControlStatus, G3ControllerName
import numpy as np
import time
import sys

nav = GalbotNavigation()
nav.init()
robot = GalbotRobot()
robot.init()

goal = np.array([0.5, 0.0, 0.0, 0, 0, 0.0, 1.0])

res = robot.switch_controller(G3ControllerName.CHASSIS_POSE_CTRL)
if res != ControlStatus.SUCCESS:
    print("Failed to switch controller!")
    sys.exit(1)
else:
    print("Controller switched successfully!")

nav.navigate_to_goal(goal, enable_collision_check=True, is_blocking=False, timeout=20)

start_time = time.time()
reached = False

while True:
    if nav.check_goal_arrival():
        reached = True
        break
    if time.time() - start_time > 20:
        print("Navigation timed out; target not reached within 20s")
        break
    print("Navigating...")
    time.sleep(0.5)

if reached:
    print("Target reached")

nav.stop_navigation()
# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print("Resources released successfully")
