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

chain_joints = {
    "leg": [0.4992,1.4991,1.0005,0.0000,-0.0004],
    "head": [0.0000,0.0],
    "left_arm": [1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0],
    "right_arm": [-1.5, 1.36, 0.45, -1.53, 0.1, 0.42, 0.0]
}
chain_pose_baselink = {
    "left_arm": [0.4827, 0.2329, 0.8883, 0.0405, 0.0396, -0.0604, 0.9966],
    "right_arm": [0.2818, -0.2430, 0.8389, -0.0456, 0.0479, 0.0573, 0.9962]
}
whole_body_joint = [
    num for key in ["leg", "head", "left_arm", "right_arm"] 
    for num in chain_joints[key]
]
base_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
custom_param = gm.Parameter()
reference_frame = "base_link"
target_frame = "EndEffector"
target_chain = "left_arm"
one_chain = [target_chain]
chain_with_torso = [target_chain, "torso"]
error_chains = [target_chain, "torso", "head"]
# Scenario 1: Single-chain inverse kinematics
try:
    status, joint_map = motion.inverse_kinematics(
        target_pose=chain_pose_baselink[target_chain],
        chain_names=one_chain,
        target_frame=target_frame,
        reference_frame=reference_frame,
        enable_collision_check=False  # Disable collision checking for accelerated testing
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Inverse kinematics calculation failed"
    print(f"✅ Basic IK succeeded: joint angles={joint_map}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ Basic IK exception: {e}")

# Scenario 2: Arm chain + torso inverse kinematics
try:
    status, joint_map = motion.inverse_kinematics(
        target_pose=chain_pose_baselink[target_chain],
        chain_names=chain_with_torso,
        target_frame=target_frame,
        reference_frame=reference_frame,
        enable_collision_check=False  # Disable collision checking for accelerated testing
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Inverse kinematics calculation failed"
    print(f"✅ IK with custom initial joints succeeded: joint angles={joint_map}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ IK with custom initial joints exception: {e}")

# Scenario 3: invalid chain combination
try:
    status, joint_map = motion.inverse_kinematics(
        target_pose=chain_pose_baselink[target_chain],
        chain_names=error_chains,
        target_frame=target_frame,
        reference_frame=reference_frame,
        enable_collision_check=False  # Disable collision checking for accelerated testing
    )
    printStatus(status)
    assert status == gm.MotionStatus.INVALID_INPUT, "Inverse kinematics calculation failed"
    print(f"✅ Invalid chain-combination input check passed")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ IK with custom initial joints exception: {e}")

# Scenario 4: Use reference joints
try:
    # initial_joint_positions can specify chain joints as IK reference; unspecified chain joints are filled from whole-body joints
    status, joint_map = motion.inverse_kinematics(
        target_pose=chain_pose_baselink[target_chain],
        chain_names=one_chain,
        target_frame=target_frame,
        reference_frame=reference_frame,
        initial_joint_positions=chain_joints,
        enable_collision_check=False  # Disable collision checking for accelerated testing
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Inverse kinematics calculation failed"
    print(f"✅ IK with custom initial joints succeeded: joint angles={joint_map}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ IK with custom initial joints exception: {e}")

# Scenario 5: Use RobotStates
try:
    ref_robot_state = gm.RobotStates()
    ref_robot_state.chain_name = target_chain
    ref_robot_state.whole_body_joint = whole_body_joint
    ref_robot_state.base_state = base_state
    target_frame = "EndEffector"
    reference_frame = "base_link"
    status, joint_map = motion.inverse_kinematics_by_state(
        target_pose=chain_pose_baselink[target_chain],
        chain_names=one_chain,
        target_frame=target_frame,
        reference_frame=reference_frame,
        reference_robot_states=ref_robot_state,
        enable_collision_check=False
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Inverse kinematics calculation failed"
    print(f"✅ RobotStates-based IK succeeded: joint angles={joint_map}")
except Exception as e:
    print(f"❌ RobotStates-based IK exception: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
