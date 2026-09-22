import time

import galbot_sdk.s1 as gm
from galbot_sdk.s1 import GalbotMotion, GalbotRobot


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
        "torso": [0.65],
        "head": [0.0, -0.26],
        "left_arm": [-0.47, -0.94, -0.54, -1.92, 0.2, 0.0, 0.0],
        "right_arm": [0.47, 0.94, 0.54, 1.92, -0.2, 0.0, 0.0]
    }

     # FK results of chain_joints in base_link, showing their physical correspondence.
    chain_pose_baselink = {
        "torso": [-0.0715, 0.0000, 1.2044, 0.0000, 0.0000, 0.0000, 1.0000],
        "head": [-0.0123, 0.0046, 1.4975, -0.0002, 0.1298, -0.0001, 0.9915],
        "left_arm": [0.2851, 0.3190, 1.9214, -0.3816, 0.7681, -0.1496, -0.4920],
        "right_arm": [0.2819, -0.3212, 1.9315, 0.3701, 0.7731, 0.1567, -0.4907],
    }

    whole_body_joint = [
        num for key in ["torso", "head", "left_arm", "right_arm"]
        for num in chain_joints[key]
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
        return
    print("✅ Moved to the initial whole-body joint state")
    time.sleep(3.0)

    custom_param = gm.Parameter()
    target_chain = "left_arm"

    # Scenario 1: Multi-waypoint planning in Cartesian space (PoseState target)
    try:
        # Construct target pose
        target_pose_state = gm.PoseState()
        target_pose_state.chain_name = target_chain

        # Construct waypoints (3 intermediate poses)
        waypoint_poses = [
            [0.2851, 0.3190, 1.9214, -0.3816, 0.7681, -0.1496, -0.4920],
            [0.2851, 0.4190, 1.9214, -0.3816, 0.7681, -0.1496, -0.4920],
            [0.2851, 0.5190, 1.9214, -0.3816, 0.7681, -0.1496, -0.4920],
            [0.2851, 0.5190, 1.8214, -0.3816, 0.7681, -0.1496, -0.4920]
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
            print("⚠️ Return status is SUCCESS, but trajectory is empty; possibly already reached, check whether the target matches current state or is within tolerance")
    except Exception as e:
        print(f"❌ Cartesian multi-point motion planning exception: {e}")

    # Scenario 2: Multi-waypoint planning in joint space (JointStates target)
    try:
        # Construct target joint state
        target_joint = gm.JointStates()
        target_joint.chain_name = target_chain

        # Construct waypoints (3 intermediate joint states)
        waypoints = [
            [-0.47, -0.94, -0.54, -1.92, 0.2, 0.0, 0.0],
            [-0.52, -0.94, -0.54, -1.92, 0.2, 0.05, 0.0],
            [-0.57, -0.94, -0.54, -1.90, 0.2, 0.10, 0.0],
            [-0.55, -0.93, -0.54, -1.98, 0.19, 0.15, 0.01]
        ]

        status, traj = motion.motion_plan_multi_waypoints(
            target=target_joint,
            waypoint_poses=waypoints,
            enable_collision_check=False,
            params=custom_param
        )
        printStatus(status)
        assert status == gm.MotionStatus.SUCCESS, "Joint multi-waypoint single-chain planning failed"
        if traj != {}:
            print(f"✅ Joint-waypoint single-chain planning succeeded: trajectory points={len(traj[target_joint.chain_name])}")
        else:
            print("⚠️ Return status is SUCCESS, but trajectory is empty; possibly already reached, check whether the target matches current state or is within tolerance")
    except Exception as e:
        print(f"❌ Joint-space multi-point motion planning exception: {e}")

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()


if __name__ == "__main__":
    main()
