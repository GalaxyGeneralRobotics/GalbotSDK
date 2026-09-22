from galbot_sdk.g3 import GalbotRobot
from galbot_sdk.g3 import G3JointGroup, ControlStatus, Trajectory, TrajectoryPoint, JointCommand
import time
import numpy as np
from typing import List

# Generate trajectory point with position and time info
def generate_target_point(q: List[float], target_time: float = 10):
    """Generate target for joints"""
    joint_position = TrajectoryPoint()
    joint_position.time_from_start_second = target_time
    joint_command_vec = []
    for joint in q:
        joint_cmd = JointCommand()
        joint_cmd.position = joint
        joint_command_vec.append(joint_cmd)
    joint_position.joint_command_vec = joint_command_vec
    return joint_position

def generate_target_trajectory(trajectory, joint_groups=[], joint_names=[], dt=0.008):
    """Generate trajectory for joints"""
    if trajectory is None or np.ndim(trajectory) != 2 or len(trajectory) == 0:
        return None

    # Create Trajectory
    traj = Trajectory()
    traj.joint_groups = joint_groups
    traj.joint_names = joint_names

    time = 0.0
    points = []
    for state in trajectory:
        time += dt
        # Create single trajectory point
        traj_point = generate_target_point(state, time)
        points.append(traj_point)

    traj.points = points
    return traj

# Trajectory execution function
def traj_exec():
    # Get GalbotRobot singleton and initialize; only needs to be initialized once
    robot = GalbotRobot()
    robot.init()
    time.sleep(1)
    print("Initialization succeeded")

    head_traj = np.linspace(
        [0.0, 0.0],
        [0.5, 0.0],
        num=200,
    )
    # Whether to block and wait for trajectory execution to complete
    is_block = True
    # Specify which joint group trajectory to execute
    joint_groups = ["head"]
    # Execute specified joint trajectory; if provided, overrides joint_groups parameters
    joint_names = []
    status = robot.execute_joint_trajectory(generate_target_trajectory(head_traj.tolist(), joint_groups, joint_names), is_block)

    # Check execution results
    if status != ControlStatus.SUCCESS:
        print("Trajectory execution failed")
    else:
        print("Trajectory execution succeeded")

    # send SIGINT shutdown signal
    robot.request_shutdown()
    # Wait until entering shutdown state
    robot.wait_for_shutdown()
    # Perform SDK resource release
    robot.destroy()
    print('Resources released successfully')

traj_exec()