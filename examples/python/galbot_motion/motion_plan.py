import time

import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotMotion, GalbotRobot

# NOTE:
# - GalbotMotion currently does NOT provide real-time obstacle perception / automatic environment updates.
# - Motion collision checking uses self-collision + a collision world built from objects you load manually via
#   add_obstacle()/attach_target_object() (including point clouds if you load them explicitly).

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
time.sleep(1)

chain_joints = {
    "leg": [0.4992,1.4991,1.0005,0.0000,-0.0004],
    "head": [0.0000,0.0],
    "left_arm": [1.9999,-1.6000,-0.5999,-1.6999,0.0000,-0.7999,0.0000],
    "right_arm": [-2.0000,1.6001,0.6001,1.7000,0.0000,0.8000,0.0000]
}
chain_pose_baselink = {
    "leg": [0.0596,-0.0000,1.0327,0.5000,0.5003,0.4997,0.5000],
    "head": [0.0599,0.0002,1.4098,-0.7072,0.0037,0.0037,0.7069],
    "left_arm": [0.1267,0.2342,0.7356,0.0220,0.0127,0.0343,0.9991],
    "right_arm": [0.1267,-0.2345,0.7358,-0.0225,0.0126,-0.0343,0.9991]
}
whole_body_joint = [
    num for key in ["leg", "head", "left_arm", "right_arm"] 
    for num in chain_joints[key]
]
base_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
custom_param = gm.Parameter()

# Scenario 1: joint-space planning, target type = joint state
try:
    # 构造目标关节状态

    target_joint = gm.JointStates()
    target_joint.chain_name = "left_arm"
    target_joint.joint_positions = chain_joints[target_joint.chain_name]

    status, traj = motion.motion_plan(
        target=target_joint,
        # When enable_collision_check=True, collision is checked against Motion-side explicitly loaded obstacles
        # (add_obstacle/attach_target_object) and self-collision.
        enable_collision_check=False,
        params=custom_param
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Planning failed"
    if traj != {}:
        print(f"✅ 关节空间规划 + 目标点类型为关节单链单点规划规划成功: 轨迹点数={len(traj[target_joint.chain_name])}")
        time.sleep(0.8)
    else:
        print(f"⚠️ 返回状态为SUCCESS，轨迹为空，可能已到达，检查目标值与当前状态是否一致或在误差范围内")

except Exception as e:
    print(f"ERROR: joint-space single-point planning exception: {e}")

# Scenario 2: joint-space planning, target type = end-effector pose (Cartesian)
try:
    # 构造目标位姿状态
    target_pose_state = gm.PoseState()
    target_pose_state.chain_name = "left_arm"
    target_pose_state.frame_id = "EndEffector"
    target_pose_state.reference_frame = "base_link"
    target_pose_state.pose = gm.Pose(chain_pose_baselink[target_pose_state.chain_name])
    # target_pose_state.pose.position.x += 0.2

    status, traj = motion.motion_plan(
        target=target_pose_state,
        enable_collision_check=False
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Planning failed"
    if traj != {}:
        print(f"✅ 关节空间规划 + 目标点类型为末端位姿单链单点规划成功: 轨迹长度={len(traj[target_pose_state.chain_name])}")
        time.sleep(0.8)
    else:
        print(f"⚠️ 返回状态为SUCCESS，轨迹为空，可能已到达，检查目标值与当前状态是否一致或在误差范围内")

except Exception as e:
    print(f"ERROR: Cartesian single-point planning exception: {e}")

# Scenario 3: joint-space planning with an explicit start state
try:
    # 构造目标关节状态

    target_joint = gm.JointStates()
    target_joint.chain_name = "left_arm"
    target_joint.joint_positions = chain_joints[target_joint.chain_name]

    start_joint = gm.JointStates()
    start_joint.chain_name = "left_arm"
    start_joint.joint_positions = [0] * 7

    status, traj = motion.motion_plan(
        target=target_joint,
        start=start_joint,
        enable_collision_check=False,
        params=custom_param
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Planning failed"
    if traj != {}:
        print(f"✅ 关节空间规划 + 目标点类型为关节单链单点规划规划成功: 轨迹点数={len(traj[target_joint.chain_name])}")
    else:
        print(f"⚠️ 返回状态为SUCCESS，轨迹为空，可能已到达，检查目标值与当前状态是否一致或在误差范围内")

except Exception as e:
    print(f"ERROR: joint-space single-point planning exception: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()