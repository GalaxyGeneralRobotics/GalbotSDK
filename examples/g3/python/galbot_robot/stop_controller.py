"""Stop an active head controller from another Python thread."""

import signal
import threading
import time

from galbot_sdk.g3 import GalbotRobot

motion_done = threading.Event()
shutdown_requested = threading.Event()


def move_head_joint(robot, target_position):
    status = robot.set_joint_positions(
        [target_position], [], ["head_joint2"], False, 0.05, 15.0
    )
    print(f"Head motion command sent: {status}")


def stop_controller_on_ctrl_c(robot):
    while not motion_done.is_set():
        if signal.sigtimedwait({signal.SIGINT}, 0.2) is not None:
            break
    else:
        return
    print("Worker thread: stopping head controller...")
    status = robot.stop_controller("head")
    print(f"Worker thread: stop_controller returned: {status}")
    shutdown_requested.set()


def confirm_motion():
    print("⚠️  Ensure the emergency-stop button is released and the area is clear.")
    if input("Start head motion? (y/n): ").strip().lower() != "y":
        raise SystemExit("Motion was not confirmed.")


confirm_motion()
original_signal_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
robot = GalbotRobot()
robot.init()
time.sleep(1)

print("Head is moving. Press Ctrl-C to stop its controller.")
stop_thread = threading.Thread(target=stop_controller_on_ctrl_c, args=(robot,))
stop_thread.start()
move_head_joint(robot, 0.4)
# The command is non-blocking, so retain the Ctrl-C listener for its full
# command timeout unless it has already requested shutdown.
shutdown_requested.wait(timeout=15.0)
motion_done.set()
stop_thread.join()

signal.pthread_sigmask(signal.SIG_SETMASK, original_signal_mask)
robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
print("Resources released successfully")
