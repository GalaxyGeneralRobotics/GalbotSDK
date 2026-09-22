from galbot_sdk.g3 import GalbotRobot, GalbotNavigation
from galbot_sdk.g3 import ControlStatus, G3ControllerName
import numpy as np
import time
import sys

robot = GalbotRobot()
robot.init()
nav = GalbotNavigation()
nav.init()

init_pose = np.array([0.0, 0.0, 0.0, 0, 0, 0.0, 1.0])
goal_pose = np.array([1.0, 0.0, 0.0, 0, 0, 0.0, 1.0])

res = robot.switch_controller(G3ControllerName.CHASSIS_POSE_CTRL)
if res != ControlStatus.SUCCESS:
    print("Failed to switch controller!")
    sys.exit(1)
else:
    print("Controller switched successfully!")

while not nav.is_localized():
    nav.relocalize(init_pose)
    time.sleep(0.5)

if nav.check_path_reachability(goal_pose, nav.get_current_pose()):
    nav.navigate_to_goal(
        goal_pose, enable_collision_check=True, is_blocking=True, timeout=30
    )
    print("Whether reached:", nav.check_goal_arrival())

nav.stop_navigation()
# Shutdown system
robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
