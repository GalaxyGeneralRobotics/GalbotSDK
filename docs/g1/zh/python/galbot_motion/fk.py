import time
import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotMotion, GalbotRobot

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

chain_joints = {
    "leg": [0.4992, 1.4991, 1.0005, 0.0000, -0.0004],
    "head": [0.0000, 0.0],
    "left_arm": [1.9999, -1.6000, -0.5999, -1.6999, 0.0000, -0.7999, 0.0000],
    "right_arm": [-2.0000, 1.6001, 0.6001, 1.7000, 0.0000, 0.8000, 0.0000],
}
end_link = "left_arm_end_effector_mount_link"
reference_frame = "base_link"

# Scenario 1: Forward kinematics using the current robot state
try:
    status, pose = motion.forward_kinematics(end_link, reference_frame)
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Forward kinematics calculation failed"
    print(f"✅ Basic forward kinematics successful: pose={pose}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ Basic forward kinematics exception: {e}")

# Scenario 2: Forward kinematics with custom arm joints
try:
    status, pose = motion.forward_kinematics(
        end_link, reference_frame, {"left_arm": chain_joints["left_arm"]}, gm.Parameter()
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Forward kinematics calculation failed"
    print(f"✅ Custom-joint forward kinematics successful: pose={pose}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ Custom-joint forward kinematics exception: {e}")

# Scenario 3: Forward kinematics based on RobotStates
try:
    current_state = motion.get_robot_states()
    if not current_state.whole_body_joint:
        print("❌ RobotStates-based FK: robot state is empty; ensure sensors/WBC are ready")
    else:
        status, pose = motion.forward_kinematics_by_state(
            end_link, current_state, reference_frame, gm.Parameter()
        )
        printStatus(status)
        assert status == gm.MotionStatus.SUCCESS, "Forward kinematics calculation failed"
        print(f"✅ RobotStates-based forward kinematics successful: pose={pose}")
except Exception as e:
    print(f"❌ RobotStates-based forward kinematics exception: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
