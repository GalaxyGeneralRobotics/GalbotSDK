import signal
import threading
import time

from galbot_sdk.s1 import (
    ControlStatus,
    GalbotRobot,
    JointCommand,
    Trajectory,
    TrajectoryPoint,
)

# Set when the trajectory call has returned, so the worker stops waiting for a
# Ctrl-C that is never coming.
trajectory_done = threading.Event()
# Set once the worker has requested shutdown, so the main flow does not repeat it.
shutdown_requested = threading.Event()


def make_head_trajectory():
    trajectory = Trajectory()
    trajectory.joint_names = ["head_joint2"]
    point = TrajectoryPoint()
    point.time_from_start_second = 10.0
    command = JointCommand()
    command.position = 0.4
    point.joint_command_vec = [command]
    trajectory.points = [point]
    return trajectory


def stop_trajectory(robot):
    print("Worker thread: stopping trajectory execution...")
    status = robot.stop_trajectory_execution()
    if status == ControlStatus.SUCCESS:
        print("Worker thread: trajectory stop succeeded")
    else:
        print(f"Worker thread: trajectory stop FAILED: {status}")

    shutdown_requested.set()


def stop_trajectory_on_ctrl_c(robot):
    """Call stop from a worker while the main thread waits for the trajectory."""
    # sigwait() cannot be cancelled, so wait in slices: when the trajectory
    # finishes on its own no Ctrl-C ever arrives, and the worker still has to
    # exit or the join() in the main flow would block forever.
    while not trajectory_done.is_set():
        if signal.sigtimedwait({signal.SIGINT}, 0.2) is None:
            continue
        stop_trajectory(robot)
        return


def confirm_motion():
    print("⚠️  Ensure the emergency-stop button is released and the area is clear.")
    if input("Start trajectory execution? (y/n): ").strip().lower() != "y":
        raise SystemExit("Motion was not confirmed.")


confirm_motion()
original_signal_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
robot = GalbotRobot()
robot.init()
time.sleep(2)
print("Initialization succeeded")

# execute_joint_trajectory blocks in the main thread. Ctrl-C is consumed by
# the worker, which stops the motion and then shuts the SDK down.
print("Trajectory is executing. Press Ctrl-C to stop it.")
stop_thread = threading.Thread(target=stop_trajectory_on_ctrl_c, args=(robot,))
stop_thread.start()
status = robot.execute_joint_trajectory(make_head_trajectory(), is_blocking=True)
trajectory_done.set()
print(f"Trajectory execution returned: {status}")
stop_thread.join()

signal.pthread_sigmask(signal.SIG_SETMASK, original_signal_mask)
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print("Resources released successfully")
