from galbot_sdk.g3 import GalbotNavigation
from galbot_sdk.g3 import GalbotRobot
import numpy as np

# Initialize system and navigation module
robot = GalbotRobot()
robot.init()

nav = GalbotNavigation()
nav.init()

print("GalbotNavigation has been initialized:", nav is not None)

# send SIGINT shutdown signal
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print('Resources released successfully')