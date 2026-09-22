"""Move the G3 leg end-effector down 0.1 m in Cartesian space.

NOTE: MPS supports Cartesian straight-line motion using only the leg chain, but
the target must be valid to avoid unreachable poses or a risk of tipping over.
This example demonstrates a common safe use: translation along the height axis
only. MPS leg-only joint-space planning is not exposed for the same safety reason.

注意：MPS 支持仅腿链的笛卡尔直线运动，但目标数据必须合法，以避免目标
不可达或机器人倾倒风险。本example展示一种常见的安全用法：仅沿高度方向
平移。基于同样的安全原因，MPS 暂未开放仅腿链的joint空间规划。
"""

import time

import galbot_sdk.g3 as gm
from galbot_sdk.g3 import GalbotMotion, GalbotRobot


def confirm_robot_safety():
    print("WARNING: Release the emergency stop and clear obstacles around the robot.")
    return input("Continue? (y/n): ").strip().lower() == "y"


def move_to_initial_position(robot):
    chain_joints = {
        "leg": [0.5, 1.5, 1.0, 0.0, 0.0],
        "head": [0.0, 0.0],
        "left_arm": [1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0],
        "right_arm": [-1.5, 1.36, 0.45, -1.53, 0.1, 0.42, 0.0],
    }
    joint_positions = [
        value
        for chain in ["leg", "head", "left_arm", "right_arm"]
        for value in chain_joints[chain]
    ]
    status = robot.set_joint_positions(
        joint_positions,
        ["leg", "head", "left_arm", "right_arm"],
        [],
        True,
        0.1,
        30.0,
    )
    if status != gm.ControlStatus.SUCCESS:
        raise RuntimeError(f"set_joint_positions failed: {status}")


def get_leg_pose(motion):
    status, pose = motion.get_end_effector_pose_on_chain(
        "leg",
        "EndEffector",
        "base_link",
    )
    if status != gm.MotionStatus.SUCCESS or len(pose) != 7:
        raise RuntimeError(
            "get_end_effector_pose_on_chain failed: "
            f"{motion.status_to_string(status)}"
        )
    return list(pose)


def main():
    if not confirm_robot_safety():
        print("Example cancelled.")
        return 0

    motion = GalbotMotion()
    robot = GalbotRobot()
    robot_initialized = False

    try:
        if not motion.init():
            raise RuntimeError("GalbotMotion init failed")
        print("GalbotMotion initialized successfully")

        if not robot.init():
            raise RuntimeError("GalbotRobot init failed")
        robot_initialized = True
        print("GalbotRobot initialized successfully")

        move_to_initial_position(robot)
        print("Moved to the initial whole-body joint state")
        time.sleep(3.0)

        current_pose = get_leg_pose(motion)
        print(f"Current leg pose: {current_pose}")

        target_pose = current_pose.copy()
        target_pose[2] -= 0.1
        print(f"Target leg pose: {target_pose}")

        target = gm.PoseState()
        target.chain_name = "leg"
        target.frame_id = "EndEffector"
        target.reference_frame = "base_link"
        target.pose = gm.Pose(target_pose)

        params = gm.Parameter()
        params.set_move_line(True)
        params.set_direct_execute(True)
        params.set_blocking(True)
        params.set_timeout(120.0)
        params.set_check_collision(False)
        params.set_actuate("with_chain_only")

        status, trajectory = motion.motion_plan(
            target=target,
            enable_collision_check=False,
            params=params,
        )
        print(f"Leg Cartesian move_line: {motion.status_to_string(status)}")
        if status != gm.MotionStatus.SUCCESS:
            raise RuntimeError("leg Cartesian move_line failed")
        print(f"Leg trajectory points: {len(trajectory.get('leg', []))}")

        time.sleep(1.0)
        reached_pose = get_leg_pose(motion)
        pose_diff = [actual - expected for actual, expected in zip(reached_pose, target_pose)]
        print(f"Reached leg pose: {reached_pose}")
        print(f"Pose diff (reached - target): {pose_diff}")

        move_to_initial_position(robot)
        print("Returned to the initial whole-body joint state")
        return 0
    except Exception as error:
        print(f"ERROR: {error}")
        return 1
    finally:
        if robot_initialized:
            robot.request_shutdown()
            robot.wait_for_shutdown()
            robot.destroy()


if __name__ == "__main__":
    raise SystemExit(main())
