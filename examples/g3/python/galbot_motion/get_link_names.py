import time
import galbot_sdk.g3 as gm
from galbot_sdk.g3 import GalbotMotion, GalbotRobot

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

# Program started, waiting for data
time.sleep(2)

try:
    # Get all link names
    all_link_names = motion.get_link_names(only_end_effector=False)
    print(f"\nAll link names (total {len(all_link_names)}):")
    for i, link_name in enumerate(all_link_names, 1):
        print(f"  {i}. {link_name}")

    # getend effectorexecute link
    ee_link_names = motion.get_link_names(only_end_effector=True)
    print(f"\nEnd-effector link names (total {len(ee_link_names)}):")
    for i, link_name in enumerate(ee_link_names, 1):
        print(f"  {i}. {link_name}")

except Exception as e:
    print(f"❌ Link-name retrieval exception: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
