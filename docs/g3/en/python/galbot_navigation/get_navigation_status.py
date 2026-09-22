"""
example: navigation get_navigation_status, SUCCESS/FAILED timeout exit,
Avoid deadlock and execute error logic.
"""

from galbot_sdk.g3 import GalbotNavigation
from galbot_sdk.g3 import GalbotRobot
from galbot_sdk.g3 import NavigationTaskStatus, ControlStatus, G3ControllerName
import numpy as np
import time
import sys

nav = GalbotNavigation()
nav.init()
robot = GalbotRobot()
robot.init()

goal = np.array([0.5, 0.0, 0.0, 0, 0, 0.0, 1.0])
timeout_s = 20.0
poll_interval_s = 0.5

res = robot.switch_controller(G3ControllerName.CHASSIS_POSE_CTRL)
if res != ControlStatus.SUCCESS:
    print("Failed to switch controller!")
    sys.exit(1)
else:
    print("Controller switched successfully!")
# Non-blocking navigation
nav.navigate_to_goal(
    goal, enable_collision_check=True, is_blocking=False, timeout=timeout_s
)
start = time.time()

while True:
    status = nav.get_navigation_status()
    elapsed = time.time() - start

    if status == NavigationTaskStatus.SUCCESS:
        print("Target reached")
        break
    if status == NavigationTaskStatus.FAILED:
        print("Navigation failed; exit error-handling logic promptly")
        break
    if elapsed >= timeout_s:
        print("navigationtimeout, exit")
        break

    if status == NavigationTaskStatus.RUNNING:
        print(f"Navigating... Status: {status.name}, elapsed: {elapsed:.1f}s")
    else:
        print(f"Status: {status.name}, elapsed: {elapsed:.1f}s")

    time.sleep(poll_interval_s)

nav.stop_navigation()
robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
print("Resources released successfully")
