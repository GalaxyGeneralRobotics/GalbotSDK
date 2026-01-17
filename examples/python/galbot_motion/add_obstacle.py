import time

import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotMotion, GalbotRobot

# NOTE:
# - GalbotMotion currently does NOT provide real-time obstacle perception / automatic environment updates.
# - If you want Motion collision checking to consider obstacles (including point clouds), you must load them
#   manually via add_obstacle()/attach_target_object().

motion = GalbotMotion.get_instance()
robot = GalbotRobot.get_instance()

def printStatus(status):
        if(status == gm.MotionStatus.SUCCESS):
            print("Result: SUCCESS")
        elif(status == gm.MotionStatus.TIMEOUT):
            print("Result: TIMEOUT")
        elif(status == gm.MotionStatus.FAULT):
            print("Result: FAULT")
        elif(status == gm.MotionStatus.INVALID_INPUT):
            print("Result: INVALID_INPUT")
        elif(status == gm.MotionStatus.INIT_FAILED):
            print("Result: INIT_FAILED")
        elif(status == gm.MotionStatus.IN_PROGRESS):
            print("Result: IN_PROGRESS")
        elif(status == gm.MotionStatus.STOPPED_UNREACHED):
            print("Result: STOPPED_UNREACHED")
        elif(status == gm.MotionStatus.DATA_FETCH_FAILED):
            print("Result: DATA_FETCH_FAILED")
        elif(status == gm.MotionStatus.PUBLISH_FAIL):
            print("Result: PUBLISH_FAIL")
        elif(status == gm.MotionStatus.COMM_DISCONNECTED):
            print("Result: COMM_DISCONNECTED")

if motion.init():
    print("GalbotMotion init OK")
else:
    print("GalbotMotion init FAILED")
if robot.init():
    print("GalbotRobot init OK")
else:
    print("GalbotRobot init FAILED")

# Wait for data to be ready.
time.sleep(2)

# 1) Add a box collision object into Motion environment.
#    This affects Motion-side collision checking (e.g., motion_plan/check_collision).
try:
    obstacle_id = "box_test_1"
    obj_type = "box"
    obj_pose = [1.0, 0.0, 1.0, 0,0,0,1]
    obj_size = [1.0, 1.0, 1.0]
    target_frame = "world"
    status = motion.add_obstacle(
        obstacle_id=obstacle_id,
        obstacle_type=obj_type,
        pose=obj_pose,
        scale=obj_size,
        target_frame=target_frame
    )
    printStatus(status)
    motion.clear_obstacle()
    assert status == gm.MotionStatus.SUCCESS, "Failed to add obstacle"
    print(f"OK: added obstacle: {obstacle_id}")
except Exception as e:
    print(f"ERROR: add obstacle exception: {e}")

# 2) Add a duplicate ID (expected to fail).
try:
    obstacle_id = "box_test_1"
    obj_type = "box"
    obj_pose = [1.0, 0.0, 1.0, 0,0,0,1]
    obj_size = [1.0, 1.0, 1.0]
    target_frame = "world"
    status = motion.add_obstacle(
        obstacle_id=obstacle_id,
        obstacle_type=obj_type,
        pose=obj_pose,
        scale=obj_size,
        target_frame=target_frame
    )
    status = motion.add_obstacle(
        obstacle_id=obstacle_id,
        obstacle_type=obj_type,
        pose=obj_pose,
        scale=obj_size,
        target_frame=target_frame
    )
    printStatus(status)
    motion.clear_obstacle()
    assert status == gm.MotionStatus.FAULT, "Expected duplicate obstacle ID to fail"
    print("OK: duplicate obstacle ID is rejected")
except Exception as e:
    print(f"ERROR: duplicate obstacle exception: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()