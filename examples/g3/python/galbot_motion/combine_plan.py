"""G3 combine_plan example aligned with combine_plan_example.cpp."""

import time

import galbot_sdk.g3 as gm
from galbot_sdk.g3 import GalbotMotion, GalbotRobot

START_JOINT_STATE = {
    "leg": [0.4992, 1.4991, 1.0005, 0.0000, -0.0004],
    "head": [0.0000, 0.0],
    "left_arm": [1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0],
    "right_arm": [-1.5, 1.36, 0.45, -1.53, 0.1, 0.42, 0.0],
}
START_MOVE_SPEED_RAD_S = 0.12
START_MOVE_TIMEOUT_S = 30.0
UPPER_BODY_GROUPS = ["head", "left_arm", "right_arm"]


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


def move_robot_to_start_state(robot):
    print("Move robot to combine_plan start state.")
    print(f"  leg start joints: {START_JOINT_STATE['leg']}")
    # controller_status = robot.switch_controller(gm.G3ControllerName.LEG_PVT_CTRL)
    # print_control_status("GalbotRobot::switch_controller(leg_pvt_ctrl)", controller_status)
    # if controller_status != gm.ControlStatus.SUCCESS:
    #     return False
    # time.sleep(0.5)
    leg_status = robot.set_joint_positions(
        START_JOINT_STATE["leg"], ["leg"], [], True,
        START_MOVE_SPEED_RAD_S, START_MOVE_TIMEOUT_S
    )
    print_control_status("GalbotRobot::set_joint_positions(leg start)", leg_status)
    if leg_status != gm.ControlStatus.SUCCESS:
        return False
    upper_body = (
        START_JOINT_STATE["head"]
        + START_JOINT_STATE["left_arm"]
        + START_JOINT_STATE["right_arm"]
    )
    print(f"  upper-body start joints [head, left_arm, right_arm]: {upper_body}")
    status = robot.set_joint_positions(
        upper_body, UPPER_BODY_GROUPS, [], True,
        START_MOVE_SPEED_RAD_S, START_MOVE_TIMEOUT_S
    )
    print_control_status("GalbotRobot::set_joint_positions(head+arms start)", status)
    return status == gm.ControlStatus.SUCCESS


def capture_current_pose(motion, robot, chain, current):
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
    status, pose = motion.get_end_effector_pose_on_chain(chain, "EndEffector", "base_link")
    result = "SUCCESS" if status == gm.MotionStatus.SUCCESS else "FAILED"
    print(f"get_end_effector_pose_on_chain({chain}) status: {motion.status_to_string(status)} ({result})")
    pose = list(pose)
    if status != gm.MotionStatus.SUCCESS or len(pose) != 7:
        return False
    current["poses"][chain] = pose
    print(f"Current {chain} pose [x, y, z, qx, qy, qz, qw]: {pose}")
    return True


def make_joint_target(chain, joint_positions, joint_names=None):
    target = gm.MotionPlanChainTarget()
    target.chain_name = chain
    target.mode = gm.MotionPlanTargetMode.kJoint
    target.joint.chain_name = chain
    target.joint.joint_positions = list(joint_positions)
    target.joint.joint_names = list(joint_names or [])
    return target


def make_cartesian_target(chain, pose, assist_chains=None):
    target = gm.MotionPlanChainTarget()
    target.chain_name = chain
    target.mode = gm.MotionPlanTargetMode.kCartesian
    target.cart.chain_name = chain
    target.cart.frame_id = "EndEffector"
    target.cart.reference_frame = "base_link"
    target.cart.pose = gm.Pose(list(pose))
    target.cart.assist_chains = set(assist_chains or [])
    return target


def make_planner_config(check_collision):
    params = gm.Parameter()
    params.set_direct_execute(True)
    params.set_blocking(True)
    params.set_timeout(120.0)
    params.set_check_collision(check_collision)
    params.set_reference_frame("base_link")
    params.set_actuate("with_torso")
    return params


def print_plan_requests(plan_requests):
    plan_type_names = {
        gm.MotionPlanType.MOTION_PLAN: "MOTION_PLAN",
        gm.MotionPlanType.TRAJ_PLAN: "TRAJ_PLAN",
        gm.MotionPlanType.MOVE_LINE: "MOVE_LINE",
    }
    print(f"combine_plan plan requests: {len(plan_requests)}")
    for request_index, request in enumerate(plan_requests):
        print(
            f"  request[{request_index}] type={plan_type_names.get(request.plan_type, 'UNKNOWN')} "
            f"waypoints={len(request.target)}"
        )
        for waypoint_index, waypoint in enumerate(request.target):
            print(f"    waypoint[{waypoint_index}] targets: {len(waypoint)}")
            for target in waypoint:
                mode = "JOINT" if target.mode == gm.MotionPlanTargetMode.kJoint else "CART"
                print(f"      {target.chain_name} {mode}")


def print_traj_result(label, status, traj, motion):
    result = "SUCCESS" if status == gm.MotionStatus.SUCCESS else "FAILED"
    print(f"{label} status: {motion.status_to_string(status)} ({result})")
    if status == gm.MotionStatus.SUCCESS:
        if not traj:
            print("Trajectory map is empty.")
        for chain, points in traj.items():
            print(f"  {chain} trajectory points: {len(points)}")


def make_plan_request(plan_type, target):
    request = gm.PlanRequest()
    request.plan_type = plan_type
    request.enforce_pass = True
    request.target = target
    return request


def make_example_inputs(current):
    left_names = current["joint_names"]["left_arm"]
    right_names = current["joint_names"]["right_arm"]
    cart = lambda chain, pose: make_cartesian_target(chain, pose, ["torso"])

    request0 = make_plan_request(
        gm.MotionPlanType.MOTION_PLAN,
        [
            [
                make_joint_target(
                    "left_arm",
                    [1.2, -1.0, -0.2, 1.4, 0.1, -0.3, 0.0],
                    left_names,
                ),
                cart("right_arm", [0.26, -0.2324, 0.76428, -0.0453, 0.0152, -0.068, 0.996]),
            ],
            [
                make_joint_target(
                    "left_arm",
                    [1.46326, -1.36281, -0.44721, 1.50544, -0.10183, -0.40826, 0.00287],
                    left_names,
                )
            ],
            [
                make_joint_target(
                    "left_arm",
                    [1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0],
                    left_names,
                ),
                cart("right_arm", [0.26, -0.2324, 0.76428, -0.0453, 0.0152, -0.068, 0.996]),
            ],
        ],
    )

    request1 = make_plan_request(
        gm.MotionPlanType.MOVE_LINE,
        [
            [
                cart("left_arm", [0.302504, 0.232819, 0.819552, 0.0413871, 0.0393282, -0.0675238, 0.996083]),
                cart("right_arm", [0.304916, -0.240799, 0.822258, -0.0411131, 0.0379809, 0.0507316, 0.997143]),
            ],
            [cart("left_arm", [0.272504, 0.232819, 0.819552, 0.0413871, 0.0393282, -0.0675238, 0.996083])],
            [cart("right_arm", [0.274916, -0.240799, 0.822258, -0.0411131, 0.0379809, 0.0507316, 0.997143])],
        ],
    )

    request2 = make_plan_request(
        gm.MotionPlanType.MOTION_PLAN,
        [
            [
                make_joint_target(
                    "left_arm",
                    [1.2, -1.0, -0.2, 1.4, 0.1, -0.3, 0.0],
                    left_names,
                )
            ],
            [
                make_joint_target(
                    "right_arm",
                    [-1.2, 1.0, 0.2, -1.4, -0.1, 0.3, 0.0],
                    right_names,
                )
            ],
            [make_joint_target("torso", [0.0, 0.0], ["leg_joint4", "leg_joint5"])],
        ],
    )

    plan_requests = [request0, request1, request2]
    return plan_requests, make_planner_config(check_collision=True)


def run_example():
    motion, robot = initialize_interfaces()
    if motion is None or robot is None:
        return 1

    try:
        if not move_robot_to_start_state(robot):
            return 1
        time.sleep(0.5)

        current = {"joint_names": {}, "joints": {}, "poses": {}}
        if not capture_current_pose(motion, robot, "left_arm", current):
            return 1
        if not capture_current_pose(motion, robot, "right_arm", current):
            return 1

        plan_requests, params = make_example_inputs(current)
        print_plan_requests(plan_requests)
        print(
            "Immediate execution is enabled; plan requests are loaded from "
            "combine_file_example_plan_reqs.json reference values."
        )
        status, traj = motion.combine_plan(plan_requests, params)
        print_traj_result("combine_plan", status, traj, motion)
        return 0 if status == gm.MotionStatus.SUCCESS else 1
    finally:
        shutdown_robot(robot)


if __name__ == "__main__":
    raise SystemExit(run_example())
