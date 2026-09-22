import time
import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotMotion, GalbotRobot


def confirm_robot_safety():
    print("WARNING: The robot will move to the initial joint state. Release the emergency stop and clear nearby obstacles.")
    return input("Continue? (y/n): ").strip().lower() == "y"


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


def main():
    if not confirm_robot_safety():
        print("Example cancelled.")
        return

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

    chain_joints = {
        "leg": [0.5, 1.5, 1.0, 0.0, 0.0],
        "head": [0.0, 0.0],
        "left_arm": [2.0, -1.5, -0.6, -1.7, 0.0, -0.8, 0.0],
        "right_arm": [-2.0, 1.5, 0.6, 1.7, 0.0, 0.8, 0.0]
    }

    # FK results of chain_joints in base_link, showing their physical correspondence.
    chain_pose_baselink = {
        "leg": [0.0545, -0.0013, 1.0377, 0.4956, 0.4950, 0.5060, 0.5033],
        "head": [0.0517, -0.0016, 1.4155, -0.7046, -0.0042, -0.0056, 0.7096],
        "left_arm": [0.1298, 0.2608, 0.7418, 0.0734, 0.0209, -0.0233, 0.9968],
        "right_arm": [0.1261, -0.2575, 0.7424, -0.0466, 0.0223, 0.0058, 0.9986]
    }
    whole_body_joint = [
        num for key in ["leg", "head", "left_arm", "right_arm"]
        for num in chain_joints[key]
    ]
    move_status = robot.set_joint_positions(
        whole_body_joint,
        ["leg", "head", "left_arm", "right_arm"],
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
        return
    print("✅ Moved to the initial whole-body joint state")
    time.sleep(3.0)

    base_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    custom_param = gm.Parameter()

    # NOTE: The two scenarios express the same physical waypoints as Cartesian
    # poses and joint positions, so their planning results are consistent. Setting
    # custom_param.set_move_line(True) applies Cartesian-space line planning to both.
    # 注意：以下两个场景分别使用笛卡尔位姿和关节位置表达相同的物理路点，
    # 因此规划结果一致。设置 custom_param.set_move_line(True) 后，二者均按
    # 笛卡尔空间直线方式规划。

    # Scenario 1: Multi-waypoint planning in Cartesian space (PoseState target)
    try:
        # Construct target pose
        target_pose_state = gm.PoseState()
        target_pose_state.chain_name = "left_arm"

        # Construct waypoints (3 intermediate poses)
        waypoint_poses = [
            [0.1297, 0.2608, 0.7417, 0.0734, 0.0209, -0.0233, 0.9968],
            [0.2297, 0.2608, 0.7417, 0.0734, 0.0209, -0.0233, 0.9968],
            [0.3297, 0.2608, 0.7417, 0.0734, 0.0209, -0.0233, 0.9968],
            [0.3297, 0.2608, 0.8417, 0.0734, 0.0209, -0.0233, 0.9968],
        ]

        status, traj = motion.motion_plan_multi_waypoints(
            target=target_pose_state,
            waypoint_poses=waypoint_poses,
            enable_collision_check=False,
            params=custom_param
        )
        printStatus(status)
        assert status == gm.MotionStatus.SUCCESS, "Cartesian multi-waypoint single-chain planning failed"
        if traj != {}:
            print(f"✅ Cartesian waypoint single-chain planning succeeded: trajectory points={len(traj[target_pose_state.chain_name])}")
            time.sleep(0.8)
        else:
            print(f"⚠️ Return status is SUCCESS, but trajectory is empty; possibly already reached, check whether the target matches current state or is within tolerance")
    except Exception as e:
        print(f"❌ Cartesian multi-point motion planning exception: {e}")

    # Scenario 2: Multi-waypoint planning in joint space (JointStates target)
    try:
        # Construct target pose
        target_joint = gm.JointStates()
        target_joint.chain_name = "left_arm"

        # Construct waypoints (3 intermediate poses)
        waypoints = [
            [2.0000, -1.5001, -0.6001, -1.7000, 0.0001, -0.7999, 0.0000],
            [1.7805, -1.4841, -0.5856, -1.6857, -0.0083, -0.5952, 0.0090],
            [1.5313, -1.4746, -0.5672, -1.5923, -0.0109, -0.4406, 0.0185],
            [1.6264, -1.4551, -0.5976, -1.9286, -0.0427, -0.1972, 0.0289]
        ]

        status, traj = motion.motion_plan_multi_waypoints(
            target=target_joint,
            waypoint_poses=waypoints,
            enable_collision_check=False,
            params=custom_param
        )
        printStatus(status)
        assert status == gm.MotionStatus.SUCCESS, "Cartesian multi-waypoint single-chain planning failed"
        if traj != {}:
            print(f"✅ Joint-waypoint single-chain planning succeeded: trajectory points={len(traj[target_joint.chain_name])}")
        else:
            print(f"⚠️ Return status is SUCCESS, but trajectory is empty; possibly already reached, check whether the target matches current state or is within tolerance")
    except Exception as e:
        print(f"❌ Joint-space multi-point motion planning exception: {e}")

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()


if __name__ == "__main__":
    main()
