import signal
import threading
import time

from galbot_sdk.g3 import ControlStatus, GalbotRobot

motion_done = threading.Event()
emergency_triggered = threading.Event()


def move_head_joint(robot, target_position):
    status = robot.set_joint_positions(
        [target_position], [], ["head_joint2"], True, 0.05, 15.0
    )
    print(f"Head motion returned: {status}")


def emergency_stop_on_ctrl_c(robot):
    while not motion_done.is_set():
        if signal.sigtimedwait({signal.SIGINT}, 0.2) is not None:
            break
    else:
        return
    print("Worker thread: triggering software emergency stop...")
    emergency_triggered.set()
    status = robot.emergency_stop()
    print(f"Worker thread: emergency_stop returned: {status}")


def confirm_motion():
    print("⚠️  Ensure the emergency-stop button is released and the area is clear.")
    if input("Start head emergency-stop test? (y/n): ").strip().lower() != "y":
        raise SystemExit("Motion was not confirmed.")


confirm_motion()
original_signal_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
robot = GalbotRobot()
robot.init()
time.sleep(1)
print("Initialization succeeded")

print("Head is moving. Press Ctrl-C to trigger software emergency stop.")
stop_thread = threading.Thread(target=emergency_stop_on_ctrl_c, args=(robot,))
stop_thread.start()
target_position = 0.4
move_head_joint(robot, target_position)
motion_done.set()
stop_thread.join()

if emergency_triggered.is_set():
    print("Waiting 3 seconds before resuming from emergency stop...")
    time.sleep(3)
    print("Resuming from software emergency stop...")
    status = robot.resume_from_emergency_stop()
    if status == ControlStatus.SUCCESS:
        print("Resume from emergency stop successfully.")
        print("Waiting 10 seconds for the robot to stabilize after recovery...")
        time.sleep(10)
        print("Resuming head motion to the original target position...")
        move_head_joint(robot, target_position)
    else:
        print("Resume from emergency stop failed.")

signal.pthread_sigmask(signal.SIG_SETMASK, original_signal_mask)
robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
print("Resources released successfully")
