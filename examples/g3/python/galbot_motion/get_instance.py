from galbot_sdk.g3 import GalbotMotion
from galbot_sdk.g3 import GalbotRobot

# Get and initialize the GalbotMotion singleton
motion = GalbotMotion()
robot = GalbotRobot()

if motion.init():
    print("GalbotMotion initialized successfully")
else:
    print("GalbotMotion initialization failed")

if robot.init():
    print("GalbotRobot initialized successfully")
else:
    print("GalbotRobot initialization failed")

# You can still manage the robot lifecycle through GalbotRobot
robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()