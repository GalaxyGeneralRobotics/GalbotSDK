import sys

import galbot_sdk.s1 as gm
from galbot_sdk.s1 import GalbotMotion, GalbotNavigation, GalbotRobot


def print_collision_details(motion, title, status, collision_infos):
    print(f"{title} status: {motion.status_to_string(status)}")
    if status != gm.MotionStatus.SUCCESS:
        return
    if not collision_infos:
        print("  no collision pair returned")
        return
    for i, info in enumerate(collision_infos):
        label = "COLLISION" if info.is_collision else "NO COLLISION"
        print(
            f"  - pair [{i}]: {label}, {info.link1} <-> {info.link2}, "
            f"distance: {info.distance}, type: {info.collision_type}"
        )


motion = GalbotMotion()
robot = GalbotRobot()
navigation = GalbotNavigation()

if not motion.init():
    print("GalbotMotion init FAILED", file=sys.stderr)
    sys.exit(1)
if not robot.init():
    print("GalbotRobot init FAILED", file=sys.stderr)
    sys.exit(1)
if not navigation.init():
    print("GalbotNavigation init FAILED", file=sys.stderr)
    sys.exit(1)

params = gm.Parameter()
params.set_timeout(5.0)

try:
    print(">> Detailed collision check: current robot state")
    status, collision_infos = motion.check_collision_detail(
        robot_states=[],
        is_check_once=False,
        is_log=False,
        params=params,
    )
    print_collision_details(motion, "current robot state", status, collision_infos)

    chain_joints = {
        "torso": [1.1],
        "head": [0.0000, -0.26],
        "left_arm": [-0.47, -0.94, -0.54, -1.92, 0.2, 0.0, 0.0],
        "right_arm": [0.47, 0.94, 0.54, 1.92, -0.2, 0.0, 0.0],
    }
    whole_body_joint = [num for key in ["torso", "head", "left_arm", "right_arm"] for num in chain_joints[key]]

    whole_body_state = gm.RobotStates()
    whole_body_state.whole_body_joint = whole_body_joint
    whole_body_state.base_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    left_arm_state = gm.JointStates()
    left_arm_state.chain_name = "left_arm"
    left_arm_state.joint_positions = [1.99995, -1.60004, 0.599905, -1.69994, 0.0, -0.799924, 0.0]

    print(">> Detailed collision check: explicit robot states")
    status, collision_infos = motion.check_collision_detail(
        robot_states=[whole_body_state, left_arm_state],
        is_check_once=False,
        is_log=True,
        params=params,
    )
    print_collision_details(motion, "explicit robot states", status, collision_infos)
except Exception as e:
    print(f"ERROR: check_collision_detail exception: {e}", file=sys.stderr)

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
