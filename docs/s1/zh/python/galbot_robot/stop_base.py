import signal
import threading
import time

from galbot_sdk.s1 import ControlStatus, GalbotRobot

motion_done = threading.Event()
shutdown_requested = threading.Event()


def stop_base_on_ctrl_c():
    """Notify the main thread to stop publishing velocity after Ctrl-C."""
    while not motion_done.is_set():
        if signal.sigtimedwait({signal.SIGINT}, 0.2) is not None:
            shutdown_requested.set()
            return


def confirm_motion():
    print("⚠️  Ensure the emergency-stop button is released and the area is clear.")
    if input("Start chassis motion? (y/n): ").strip().lower() != "y":
        raise SystemExit("Motion was not confirmed.")


confirm_motion()
# Block SIGINT before SDK initialization so all SDK threads inherit the mask.
original_signal_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
robot = GalbotRobot()
robot.init()
time.sleep(1)
print("Initialization succeeded")

print("Chassis is moving. Press Ctrl-C to stop it.")
stop_thread = threading.Thread(target=stop_base_on_ctrl_c)
stop_thread.start()
start_time = time.monotonic()
while not shutdown_requested.is_set() and time.monotonic() - start_time < 5.0:
    next_publish_time = time.monotonic() + 0.1
    status = robot.set_base_velocity(
        [0.1, 0.0, 0.0], [0.0, 0.0, 0.0], duration_s=0.0
    )
    if status != ControlStatus.SUCCESS:
        print(f"Failed to send chassis velocity: {status}")
        break
    shutdown_requested.wait(
        timeout=max(0.0, min(next_publish_time, start_time + 5.0) - time.monotonic())
    )
motion_done.set()
stop_thread.join()

print("Sending chassis stop command...")
for _ in range(10):
    status = robot.stop_base()
    if status == ControlStatus.SUCCESS:
        print("Chassis motion stopped successfully")
        break
    print("Chassis stop failed, retrying...")
    time.sleep(0.2)
else:
    print("Chassis stop timed out")

# AsyncShutdown raises SIGINT on its calling thread, so unblock it here first.
signal.pthread_sigmask(signal.SIG_SETMASK, original_signal_mask)
robot.request_shutdown()
# Wait until entering shutdown state
robot.wait_for_shutdown()
# Perform SDK resource release
robot.destroy()
print("Resources released successfully")
