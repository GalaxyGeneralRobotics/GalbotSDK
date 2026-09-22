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

target = np.array([0.2, 0.0, 0.0, 0, 0, 0.0, 1.0])

res = robot.switch_controller(G3ControllerName.CHASSIS_POSE_CTRL)
if res != ControlStatus.SUCCESS:
    print("Failed to switch controller!")
    sys.exit(1)
else:
    print("Controller switched successfully!")

nav.move_straight_to(target, is_blocking=False, timeout=10)
time.sleep(1.0)
nav.stop_navigation()

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print("Resources released successfully")
