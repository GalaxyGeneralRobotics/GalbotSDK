"""Stop a blocking navigation request from another Python thread."""

import signal
import threading
import time

import numpy as np
from galbot_sdk.g3 import ControlStatus, G3ControllerName, GalbotNavigation, GalbotRobot

# Set when the navigation call has returned, so the worker stops waiting for a
# Ctrl-C that is never coming.
navigation_done = threading.Event()
def stop_navigation_on_ctrl_c(navigation):
    # sigwait() cannot be cancelled, so wait in slices: when the navigation
    # finishes on its own no Ctrl-C ever arrives, and the worker still has to
    # exit or the join() in the main flow would block forever.
    while not navigation_done.is_set():
        if signal.sigtimedwait({signal.SIGINT}, 0.2) is None:
            continue
        print("Worker thread: stopping navigation...")
        succeeded, detail = navigation.stop_navigation()
        if succeeded:
            print("Worker thread: navigation stop succeeded")
        else:
            print(f"Worker thread: navigation stop FAILED: {detail}")
        return


def confirm_motion():
    print("⚠️  Ensure the emergency-stop button is released and the path is clear.")
    if input("Start straight navigation? (y/n): ").strip().lower() != "y":
        raise SystemExit("Motion was not confirmed.")


confirm_motion()
original_signal_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
robot = GalbotRobot()
navigation = GalbotNavigation()
robot.init()
navigation.init()
time.sleep(3)  # Wait for the PNS service to enter its working state.

if robot.switch_controller(G3ControllerName.CHASSIS_POSE_CTRL) != ControlStatus.SUCCESS:
    print("Failed to switch controller!")
else:
    print("Robot is moving straight. Press Ctrl-C to stop navigation.")
    stop_thread = threading.Thread(
        target=stop_navigation_on_ctrl_c, args=(navigation,)
    )
    stop_thread.start()
    result = navigation.move_straight_to(
        np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]),
        is_blocking=True,
        timeout=10,
    )
    navigation_done.set()
    print(f"move_straight_to returned: {result}")
    stop_thread.join()

signal.pthread_sigmask(signal.SIG_SETMASK, original_signal_mask)
robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
print("Resources released successfully")
