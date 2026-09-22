"""G1 traj_plan example aligned with traj_plan_example.cpp."""

import time

import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotMotion, GalbotRobot

def initialize_interfaces():
    motion = GalbotMotion()
    print("GalbotMotion::get_instance() status: no return status; call completed.")
    robot = GalbotRobot()
    print("GalbotRobot::get_instance() status: no return status; call completed.")
    if not motion.init():
        print("GalbotMotion::init() status: FAILED")
        return None, None
    print("GalbotMotion::init() status: SUCCESS")
    if not robot.init():
        print("GalbotRobot::init() status: FAILED")
        return None, None
    print("GalbotRobot::init() status: SUCCESS")
    time.sleep(1.0)
    return motion, robot


def shutdown_robot(robot):
    robot.request_shutdown()
    print("request_shutdown() status: no return status; call completed.")
    robot.wait_for_shutdown()
    print("wait_for_shutdown() status: no return status; call completed.")
    robot.destroy()
    print("destroy() status: no return status; call completed.")


def print_control_status(api_name, status):
    result = "SUCCESS" if status == gm.ControlStatus.SUCCESS else "FAILED"
    print(f"{api_name} status: {str(status).rsplit('.', 1)[-1]} ({result})")


def move_robot_to_home_position(robot):
    print("Move robot to the G1 home position.")
    status = robot.set_joint_positions(
        [
            0.5, 1.5, 1.0, 0.0, 0.0,
            0.0, 0.0,
            2.0, -1.5, -0.6, -1.7, 0.0, -0.8, 0.0,
            -2.0, 1.5, 0.6, 1.7, 0.0, 0.8, 0.0,
        ],
        ["leg", "head", "left_arm", "right_arm"],
        [],
        True,
        0.1,
        30.0,
    )
    print_control_status("GalbotRobot::set_joint_positions(home)", status)
    return status == gm.ControlStatus.SUCCESS


def capture_current_joints(motion, robot, chains, current):
    for chain in chains:
        joint_names = list(motion.get_chain_joint_names(chain))
        print(f"get_chain_joint_names({chain}) status: {'SUCCESS' if joint_names else 'FAILED'}")
        if not joint_names:
            return False
        joints = list(robot.get_joint_positions([], joint_names))
        success = len(joints) == len(joint_names)
        print(f"GalbotRobot::get_joint_positions({chain}) status: {'SUCCESS' if success else 'FAILED'}")
        if not success:
            return False
        current["joint_names"][chain] = joint_names
        current["joints"][chain] = joints
        print(f"Current {chain} joints: {joints}")
    return True


def make_joint_target(chain, joint_positions, joint_names=None):
    target = gm.MotionPlanChainTarget()
    target.chain_name = chain
    target.mode = gm.MotionPlanTargetMode.kJoint
    target.joint.chain_name = chain
    target.joint.joint_positions = list(joint_positions)
    target.joint.joint_names = list(joint_names or [])
    return target


def print_waypoints(label, waypoints):
    print(f"{label} waypoints: {len(waypoints)}")
    for index, waypoint in enumerate(waypoints):
        print(f"  waypoint[{index}] targets: {len(waypoint)}")
        for target in waypoint:
            print(f"    {target.chain_name} JOINT q={list(target.joint.joint_positions)}")


def print_traj_result(label, status, traj, motion):
    result = "SUCCESS" if status == gm.MotionStatus.SUCCESS else "FAILED"
    print(f"{label} status: {motion.status_to_string(status)} ({result})")
    if status == gm.MotionStatus.SUCCESS:
        if not traj:
            print("Trajectory map is empty.")
        for chain, points in traj.items():
            print(f"  {chain} trajectory points: {len(points)}")


def make_example_inputs(current):
    left_names = current["joint_names"]["left_arm"]
    right_names = current["joint_names"]["right_arm"]

    waypoints = [
        [
            make_joint_target(
                "left_arm", [1.99995, -1.4004, -0.599905, -1.69994, 0.0, -0.799924, 0.0], left_names
            ),
            make_joint_target(
                "right_arm", [-1.99995, 1.4004, 0.599905, 1.69994, 0.0, 0.799924, 0.0], right_names
            ),
        ],
        [
            make_joint_target(
                "left_arm", [1.7995, -1.6004, -0.599905, -1.69994, 0.0, -0.799924, 0.0], left_names
            ),
            make_joint_target(
                "right_arm", [-1.7995, 1.6004, 0.599905, 1.69994, 0.0, 0.799924, 0.0], right_names
            ),
        ],
        [
            make_joint_target(
                "left_arm", [1.99995, -1.60004, -0.599905, -1.69994, 0.0, -0.799924, 0.0], left_names
            ),
            make_joint_target(
                "right_arm", [-2.0, 1.60008, 0.600051, 1.70001, 0.0, 0.799993, 0.0], right_names
            ),
        ],
    ]

    params = gm.Parameter()
    params.set_direct_execute(True)
    params.set_blocking(True)
    params.set_timeout(120.0)
    params.set_check_collision(False)
    params.set_reference_frame("base_link")
    params.set_actuate("with_chain_only")
    return waypoints, params


def make_example_inputs_with_leg(current):
    left_names = current["joint_names"]["left_arm"]
    right_names = current["joint_names"]["right_arm"]
    leg_names = current["joint_names"]["leg"]
    leg_target = list(current["joints"]["leg"])
    leg_target[3] = 0.0
    leg_target[4] = 0.0

    waypoints = [
        [
            make_joint_target(
                "left_arm", [1.99995, -1.4004, -0.599905, -1.69994, 0.0, -0.799924, 0.0], left_names
            ),
            make_joint_target(
                "right_arm", [-1.99995, 1.4004, 0.599905, 1.69994, 0.0, 0.799924, 0.0], right_names
            ),
            make_joint_target("leg", leg_target, leg_names),
        ],
        [
            make_joint_target(
                "left_arm", [1.7995, -1.6004, -0.599905, -1.69994, 0.0, -0.799924, 0.0], left_names
            ),
            make_joint_target(
                "right_arm", [-1.7995, 1.6004, 0.599905, 1.69994, 0.0, 0.799924, 0.0], right_names
            ),
            make_joint_target("leg", leg_target, leg_names),
        ],
        [
            make_joint_target(
                "left_arm", [1.99995, -1.60004, -0.599905, -1.69994, 0.0, -0.799924, 0.0], left_names
            ),
            make_joint_target(
                "right_arm", [-2.0, 1.60008, 0.600051, 1.70001, 0.0, 0.799993, 0.0], right_names
            ),
            make_joint_target("leg", leg_target, leg_names),
        ],
    ]

    params = gm.Parameter()
    params.set_direct_execute(True)
    params.set_blocking(True)
    params.set_timeout(120.0)
    params.set_check_collision(False)
    params.set_reference_frame("base_link")
    params.set_actuate("with_leg")
    return waypoints, params


def make_example_inputs_with_torso(current):
    left_names = current["joint_names"]["left_arm"]
    right_names = current["joint_names"]["right_arm"]

    waypoints = [
        [
            make_joint_target(
                "left_arm", [1.99995, -1.4004, -0.599905, -1.69994, 0.0, -0.799924, 0.0], left_names
            ),
            make_joint_target(
                "right_arm", [-1.99995, 1.4004, 0.599905, 1.69994, 0.0, 0.799924, 0.0], right_names
            ),
            make_joint_target("torso", [0.0], ["leg_joint4"]),
        ],
        [
            make_joint_target(
                "left_arm", [1.7995, -1.6004, -0.599905, -1.69994, 0.0, -0.799924, 0.0], left_names
            ),
            make_joint_target(
                "right_arm", [-1.7995, 1.6004, 0.599905, 1.69994, 0.0, 0.799924, 0.0], right_names
            ),
            make_joint_target("torso", [0.0], ["leg_joint4"]),
        ],
        [
            make_joint_target(
                "left_arm", [1.99995, -1.60004, -0.599905, -1.69994, 0.0, -0.799924, 0.0], left_names
            ),
            make_joint_target(
                "right_arm", [-2.0, 1.60008, 0.600051, 1.70001, 0.0, 0.799993, 0.0], right_names
            ),
            make_joint_target("torso", [0.0], ["leg_joint4"]),
        ],
    ]

    params = gm.Parameter()
    params.set_direct_execute(True)
    params.set_blocking(True)
    params.set_timeout(120.0)
    params.set_check_collision(False)
    params.set_reference_frame("base_link")
    params.set_actuate("with_torso")
    return waypoints, params


def run_example():
    motion, robot = initialize_interfaces()
    if motion is None or robot is None:
        return 1

    try:
        if not move_robot_to_home_position(robot):
            return 1
        time.sleep(0.5)

        current = {"joint_names": {}, "joints": {}}
        if not capture_current_joints(
            motion, robot, ["leg", "left_arm", "right_arm"], current
        ):
            return 1

        waypoints, params = make_example_inputs_with_torso(current)
        print_waypoints("traj_plan", waypoints)
        print(
            "Immediate execution is enabled; waypoints are loaded from "
            "motion_plan_targets.json reference values."
        )
        status, traj = motion.traj_plan(waypoints, params)
        print_traj_result("traj_plan", status, traj, motion)
        return 0 if status == gm.MotionStatus.SUCCESS else 1
    finally:
        shutdown_robot(robot)


if __name__ == "__main__":
    raise SystemExit(run_example())
