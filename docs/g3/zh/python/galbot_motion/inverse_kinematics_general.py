import time

import galbot_sdk.g3 as gm
from galbot_sdk.g3 import GalbotMotion, GalbotRobot


def make_pose_target_from_current(current_pose, axis=0):
    # Pose is [x, y, z, qx, qy, qz, qw]; move one translation axis only.
    target = list(current_pose)
    if len(target) == 7 and axis < 3:
        target[axis] += max(abs(target[axis]) * 0.1, 0.02)
    return target


def make_cartesian_target(chain, pose, assist_chains=None):
    target = gm.MotionPlanChainTarget()
    target.chain_name = chain
    target.mode = gm.MotionPlanTargetMode.kCartesian
    # frame_id is the end-effector frame; reference_frame is the IK reference frame.
    target.cart.chain_name = chain
    target.cart.frame_id = "EndEffector"
    target.cart.reference_frame = "base_link"
    target.cart.pose = gm.Pose(list(pose))
    target.cart.assist_chains = set(assist_chains or [])
    return target


def run_example():
    motion = GalbotMotion()
    robot = GalbotRobot()
    # Initialize SDK interfaces directly in the example flow.
    if not motion.init():
        print("GalbotMotion init FAILED")
        return 1
    if not robot.init():
        print("GalbotRobot init FAILED")
        return 1
    print("GalbotMotion/GalbotRobot init OK")
    time.sleep(1)

    # Read the current end-effector pose first; the IK target is a small relative move.
    pose_status, current_pose = motion.get_end_effector_pose_on_chain(
        chain_name="left_arm",
        frame_id="EndEffector",
        reference_frame="base_link",
    )
    print(f"get_end_effector_pose_on_chain(left_arm) status: {motion.status_to_string(pose_status)}")
    if pose_status != gm.MotionStatus.SUCCESS:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return 1

    current_pose = list(current_pose)
    target_pose = make_pose_target_from_current(current_pose)
    print(f"Current left_arm pose: {current_pose}")
    print(f"Target left_arm pose (+10% x translation): {target_pose}")

    # inverse_kinematics_general accepts the same waypoint target format as planning APIs.
    target_waypoint = [
        make_cartesian_target("left_arm", target_pose, ["leg"]),
    ]

    # Use the current whole-body state as the IK reference seed when it is available.
    reference_state = motion.get_robot_states()
    if not getattr(reference_state, "whole_body_joint", []):
        reference_state = None

    # IK uses Parameter for common options such as blocking, timeout, and reference frame.
    params = gm.Parameter()
    params.set_direct_execute(False)
    params.set_blocking(True)
    params.set_timeout(20.0)
    params.set_check_collision(False)
    params.set_reference_frame("base_link")

    # This is the API being demonstrated: solve joint values for the target waypoint.
    status, joint_map = motion.inverse_kinematics_general(
        target_waypoint=target_waypoint,
        reference_robot_states=reference_state,
        params=params,
    )
    print(f"inverse_kinematics_general status: {motion.status_to_string(status)}")
    for chain, joints in joint_map.items():
        print(f"  {chain} joint_names: {list(joints.joint_names)}")
        print(f"  {chain} joint_positions: {list(joints.joint_positions)}")

    # Clean shutdown releases SDK resources before the process exits.
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    return 0 if status == gm.MotionStatus.SUCCESS else 1


if __name__ == "__main__":
    raise SystemExit(run_example())
