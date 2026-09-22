from galbot_sdk.g3 import GalbotNavigation
from galbot_sdk.g3 import GalbotRobot
import numpy as np
import time

nav = GalbotNavigation()
nav.init()
robot = GalbotRobot()
robot.init()

init_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

# success
while not nav.is_localized():
    nav.relocalize(init_pose)
    time.sleep(0.5)

print("Current pose:", nav.get_current_pose())

nav.stop_navigation()
# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')