from galbot_sdk.g3 import GalbotRobot, G3JointGroup, ControlStatus, Trajectory, TrajectoryPoint, JointCommand
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
        joint_cmd.velocity = 0.0
        joint_cmd.acceleration = 0.0
        joint_cmd.effort = 0.0
        # joint_cmd.Kp = 0.0
        # joint_cmd.Kd = 0.0
        joint_command_vec.append(joint_cmd)
    joint_position.joint_command_vec = joint_command_vec
    return joint_position

def generate_batch_trajectory(trajectory, joint_groups=[], joint_names=[], dt=0.008):
    """Generate batch trajectory for joints"""
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

# Batch set-joint-command function
def batch_commands_exec():
    # Get GalbotRobot singleton and initialize; only needs to be initialized once
    robot = GalbotRobot()
    robot.init()
    time.sleep(1)
    print("Initialization succeeded")

    # Generate batched trajectory data (joint commands at multiple time points)
    head_traj = np.linspace(
        [0.0, 0.0],
        [0.5, 0.0],
        num=10,  # Number of batch trajectory points
    )
    # Specify which joint group trajectory to execute
    joint_groups = ["head"]
    # Execute specified joint trajectory; if provided, overrides joint_groups parameters
    joint_names = []
    
    # Batch set joint commands (non-blocking, returns immediately)
    status = robot.set_joint_commands_batch(generate_batch_trajectory(head_traj.tolist(), joint_groups, joint_names))

    # Check execution results
    if status != ControlStatus.SUCCESS:
        print("Batch command submission failed")
    else:
        print("Batch commands submitted, executing in background (non-blocking)")

    # Wait for a while to let the command execute
    time.sleep(1)

    # send SIGINT shutdown signal
    robot.request_shutdown()
    # Wait until entering shutdown state
    robot.wait_for_shutdown()
    # Perform SDK resource release
    robot.destroy()
    print('Resources released successfully')

batch_commands_exec()
