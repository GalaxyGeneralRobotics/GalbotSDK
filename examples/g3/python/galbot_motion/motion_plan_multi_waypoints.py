import time
import galbot_sdk.g3 as gm
from galbot_sdk.g3 import GalbotMotion, GalbotRobot


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
        "left_arm": [1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0],
        "right_arm": [-1.5, 1.36, 0.45, -1.53, 0.1, 0.42, 0.0]
    }

    # FK results of chain_joints in base_link, showing their physical correspondence.
    chain_pose_baselink = {
        "leg": [0.0541, -0.0013, 1.0364, 0.4970, 0.4964, 0.5037, 0.5028],
        "head": [0.0519, -0.0011, 1.4145, -0.7061, -0.0029, -0.0056, 0.7081],
        "left_arm": [0.2827, 0.2329, 0.8383, 0.0405, 0.0396, -0.0604, 0.9966],
        "right_arm": [0.2818, -0.2430, 0.8389, -0.0456, 0.0479, 0.0573, 0.9962]
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
            [0.2826, 0.2329, 0.8382, 0.0405, 0.0397, -0.0604, 0.9966],
            [0.3826, 0.2329, 0.8382, 0.0405, 0.0397, -0.0604, 0.9966],
            [0.4826, 0.2329, 0.8382, 0.0405, 0.0397, -0.0604, 0.9966],
            [0.4826, 0.2329, 0.9382, 0.0405, 0.0397, -0.0604, 0.9966],
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
            [1.5001, -1.3601, -0.4500, 1.5298, -0.1000, -0.4201, 0.0001],
            [1.1402, -1.3833, -0.4217, 1.2422, -0.1168, -0.3496, 0.0364],
            [0.6887, -1.4364, -0.3930, 0.6654, -0.1178, -0.4724, 0.0895],
            [0.8356, -1.3735, -0.4206, 1.2988, -0.1444, 0.0090, 0.0550]
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
            print(f"✅ Joint-waypoint single-chain planning succeeded: trajectory points={len(traj[target_pose_state.chain_name])}")
        else:
            print(f"⚠️ Return status is SUCCESS, but trajectory is empty; possibly already reached, check whether the target matches current state or is within tolerance")
    except Exception as e:
        print(f"❌ Joint-space multi-point motion planning exception: {e}")

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()


if __name__ == "__main__":
    main()
