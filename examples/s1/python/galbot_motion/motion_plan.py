import time

import galbot_sdk.s1 as gm
from galbot_sdk.s1 import GalbotMotion, GalbotRobot

# NOTE:
# - GalbotMotion currently does NOT provide real-time obstacle perception / automatic environment updates.
# - Motion collision checking uses self-collision + a collision world built from objects you load manually via
#   add_obstacle()/attach_target_object() (including point clouds if you load them explicitly).


def confirm_robot_safety():
    print("WARNING: The robot will move to the initial joint state. Release the emergency stop and clear nearby obstacles.")
    return input("Continue? (y/n): ").strip().lower() == "y"


if not confirm_robot_safety():
    print("Example cancelled.")
    raise SystemExit(0)


motion = GalbotMotion()
robot = GalbotRobot()

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
    "torso":     [0.65],
    "head":      [0.0000, -0.26],
    "left_arm":  [-0.47, -0.94, -0.54, -1.92, 0.2, 0.0, 0.0],
    "right_arm": [0.47, 0.94, 0.54, 1.92, -0.2, 0.0, 0.0]
}

whole_body_joint = [
    value
    for chain in ["torso", "head", "left_arm", "right_arm"]
    for value in chain_joints[chain]
]
move_status = robot.set_joint_positions(
    whole_body_joint,
    ["torso", "head", "left_arm", "right_arm"],
    [],
    True,
    0.1,
    30.0,
)
if move_status != gm.ControlStatus.SUCCESS:
    print(f"❌ Failed to move to the initial joint state: {move_status}")
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    raise SystemExit(1)
print("✅ Moved to the initial whole-body joint state")
time.sleep(3.0)

custom_param = gm.Parameter()

# Scenario 1: joint-space planning, target type = joint state
try:
    print(">> Scenario 1: joint-space planning (joint target)...")

    target_joint = gm.JointStates()
    target_joint.chain_name = "left_arm"
    target_positions = list(chain_joints[target_joint.chain_name])
    target_positions[4] += 0.1
    target_joint.joint_positions = target_positions
    # 若设备处于 home 位，target 就是当前点、规划轨迹会为空，故增加偏移。
    # 不使用 set_joint()：SDK 1.10.0 的该接口只接受整数，会拒绝浮点关节角。

    status, traj = motion.motion_plan(
        target=target_joint,
        enable_collision_check=False,
        params=custom_param
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Planning failed"
    if traj != {}:
        print(f"✅ Joint-space planning (joint target) success: trajectory points={len(traj[target_joint.chain_name])}")
        time.sleep(0.8)
    else:
        print(f"⚠️ SUCCESS but trajectory is empty, target may already be reached")

except Exception as e:
    print(f"ERROR: Scenario 1 exception: {e}")

# Scenario 2: joint-space planning, target type = end-effector pose (Cartesian)
try:
    print(">> Scenario 2: joint-space planning (pose target)...")

    target_pose_state = gm.PoseState()
    target_pose_state.chain_name = "left_arm"
    target_pose_state.frame_id = "EndEffector"
    target_pose_state.reference_frame = "base_link"

    status, current_pose = motion.get_end_effector_pose_on_chain(
        chain_name=target_pose_state.chain_name,
        frame_id=target_pose_state.frame_id,
        reference_frame=target_pose_state.reference_frame
    )
    assert status == gm.MotionStatus.SUCCESS and len(current_pose) == 7, \
        "Failed to get current end-effector pose"

    target_pose = list(current_pose)
    target_pose[0] += 0.04  # Move forward 4 cm from the current pose.
    target_pose[2] += 0.02  # Move upward 2 cm from the current pose.
    target_pose_state.pose = gm.Pose(target_pose)

    status, traj = motion.motion_plan(
        target=target_pose_state,
        enable_collision_check=False,
        params=custom_param
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Planning failed"
    if traj != {}:
        print(f"✅ Joint-space planning (pose target) success: trajectory points={len(traj[target_pose_state.chain_name])}")
        time.sleep(0.8)
    else:
        print(f"⚠️ SUCCESS but trajectory is empty, target may already be reached")

except Exception as e:
    print(f"ERROR: Scenario 2 exception: {e}")

# Scenario 3: joint-space planning with an explicit start state
try:
    print(">> Scenario 3: joint-space planning (explicit start)...")

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
        print(f"✅ Joint-space planning (explicit start) success: trajectory points={len(traj[target_joint.chain_name])}")
    else:
        print(f"⚠️ SUCCESS but trajectory is empty, target may already be reached")

except Exception as e:
    print(f"ERROR: Scenario 3 exception: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
