import time

from galbot_sdk.g1 import ControlStatus, GalbotRobot, Pose


def set_yaw_orientation(pose, yaw):
    import math

    half_yaw = 0.5 * yaw
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(half_yaw)
    pose.orientation.w = math.cos(half_yaw)




def test1(robot):
    # Test 1: set base pose using Pose.
    pose = Pose()
    pose.position.x = 0.5
    pose.position.y = 0.0
    pose.position.z = 0.0
    set_yaw_orientation(pose, 0.0)

    status = robot.set_base_pose(pose, True, 15.0)
    if status == ControlStatus.SUCCESS:
        print("[Test 1] Pose-struct set succeeded.")
    else:
        print("[Test 1] Pose-struct set failed or timed out.")

    time.sleep(2)

def test2(robot):
    # Test 2: set base pose using frame ids. "rel(0)" is relative to the current base pose.
    x = 0.1
    y = 0.0
    yaw = 0.0
    frame_id = "rel(0)"
    reference_frame_id = "odom"
    is_blocking = True
    timeout_s = 15.0

    status = robot.set_base_pose(x, y, yaw, frame_id, reference_frame_id, is_blocking, timeout_s)
    if status == ControlStatus.SUCCESS:
        print("[Test 2] Relative set succeeded.")
    else:
        print("[Test 2] Relative set failed or timed out.")

    time.sleep(2)

def test3(robot):
    # Test 3: set base pose using explicit interpolation time.
    x = 0.2
    y = 0.0
    yaw = 0.0
    # frame_id = "base_link"
    frame_id = "rel(0)"
    # frame_id = "odom"
    reference_frame_id = "odom"
    time_from_start_s = 5.0
    is_blocking = True
    timeout_s = 15.0

    status = robot.set_base_pose(
        x,
        y,
        yaw,
        frame_id,
        reference_frame_id,
        time_from_start_s,
        is_blocking,
        timeout_s,
    )
    if status == ControlStatus.SUCCESS:
        print("[Test 3] Full-control set succeeded.")
    else:
        print("[Test 3] Full-control set failed or timed out.")

    time.sleep(2)


if __name__ == "__main__":

    robot = GalbotRobot()
    robot.init()
    time.sleep(2)
    print("Initialization succeeded")

    # test1(robot)
    # test2(robot)
    test3(robot)

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    print("Resources released successfully")
