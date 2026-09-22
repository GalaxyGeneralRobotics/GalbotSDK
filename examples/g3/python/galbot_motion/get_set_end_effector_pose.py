import time
import galbot_sdk.g3 as gm
from galbot_sdk.g3 import GalbotMotion, GalbotRobot

# Get and initialize the GalbotMotion singleton
motion = GalbotMotion()
robot = GalbotRobot()

def printStatus(status):
        if(status == gm.MotionStatus.SUCCESS):
            print("Execution result: SUCCESS, execution successful")
        elif(status == gm.MotionStatus.TIMEOUT):
            print("Execution result: TIMEOUT, execution timed out")
        elif(status == gm.MotionStatus.FAULT):
            print("Execution result: FAULT, a fault occurred and execution cannot continue")
        elif(status == gm.MotionStatus.INVALID_INPUT):
            print("Execution result: INVALID_INPUT, input parameters do not meet requirements")
        elif(status == gm.MotionStatus.INIT_FAILED):
            print("Execution result: INIT_FAILED, failed to create internal communication components")
        elif(status == gm.MotionStatus.IN_PROGRESS):
            print("Execution result: IN_PROGRESS, in motion but not yet in position")
        elif(status == gm.MotionStatus.STOPPED_UNREACHED):
            print("Execution result: STOPPED_UNREACHED, stopped but target not reached")
        elif(status == gm.MotionStatus.DATA_FETCH_FAILED):
            print("Execution result: DATA_FETCH_FAILED, failed to fetch data")
        elif(status == gm.MotionStatus.PUBLISH_FAIL):
            print("Execution result: PUBLISH_FAIL, data transmission failed")
        elif(status == gm.MotionStatus.COMM_DISCONNECTED):
            print("Execution result: COMM_DISCONNECTED, connection failed")

if motion.init():
    print("GalbotMotion initialized successfully")
else:
    print("GalbotMotion initialization failed")
if robot.init():
    print("GalbotRobot initialized successfully")
else:
    print("GalbotRobot initialization failed")

# Program started, waiting for data
time.sleep(1)

chain_pose_baselink = {
    "leg": [0.0541, -0.0013, 1.0364, 0.4970, 0.4964, 0.5037, 0.5028],
    "head": [0.0519, -0.0011, 1.4145, -0.7061, -0.0029, -0.0056, 0.7081],
    "left_arm": [0.2827, 0.2329, 0.8383, 0.0405, 0.0396, -0.0604, 0.9966],
    "right_arm": [0.4818, -0.2430, 0.8389, -0.0456, 0.0479, 0.0573, 0.9962]
}
custom_param = gm.Parameter()
target_frame = "EndEffector"
reference_frame = "base_link"
target_chain = "left_arm"
# Scenario 1: Basic version
try:
    end_ee_link = "left_arm_end_effector_mount_link"
    status, pose = motion.get_end_effector_pose(
        end_effector_frame=end_ee_link,
        reference_frame=reference_frame
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Failed to get end-effector pose"
    print(f"✅ Basic end-effector pose retrieval succeeded: {pose}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ Basic end-effector pose retrieval exception: {e}")

# Scenario 2: Specify chain name + custom frame
try:
    status, pose = motion.get_end_effector_pose_on_chain(
        chain_name=target_chain,
        frame_id=target_frame,
        reference_frame=reference_frame
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Failed to get end-effector pose"
    print(f"✅ End-effector pose retrieval by specified chain succeeded: {pose}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ End-effector pose retrieval by specified chain exception: {e}")

end_effector_frame="left_arm"
reference_frame = "base_link"
motion_timeout = 10.0
try:
    status = motion.set_end_effector_pose(
        target_pose=chain_pose_baselink[end_effector_frame],
        end_effector_frame=end_effector_frame,
        reference_frame=reference_frame,
        enable_collision_check=False,
        is_blocking=False,
        timeout=motion_timeout,
        params=custom_param
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Set end-effector pose failed"
    print(f"✅ End-effector pose set succeeded: status={status}")
    time.sleep(motion_timeout + 1.0)
except Exception as e:
    print(f"❌ End-effector pose setting exception: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
