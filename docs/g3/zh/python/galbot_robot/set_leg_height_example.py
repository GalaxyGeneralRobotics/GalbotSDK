import time

from galbot_sdk.g3 import ControlStatus, G3ControllerName, GalbotRobot


def print_status(operation, status):
    if status == ControlStatus.SUCCESS:
        print(f"{operation} succeeded.")
    else:
        print(f"{operation} failed, status: {status}")


def get_leg_height(robot, label):
    pose, timestamp_ns = robot.get_transform(
        "base_link", "head_base_link", timestamp_ns=0, timeout_ms=500
    )
    height_m = pose[2]
    print(
        f"{label}: pose=[{', '.join(f'{value:.3f}' for value in pose)}], "
        f"tf_timestamp_ns={timestamp_ns}"
    )
    return height_m >= 0.0, height_m


def confirm_leg_height_motion(current_height_m, target_height_m):
    direction = "lower" if target_height_m < current_height_m else "raise"
    motion_distance_m = abs(target_height_m - current_height_m)
    response = input(
        f"The leg mechanism is about to {direction} the body by "
        f"{motion_distance_m:.3f} m, from {current_height_m:.3f} m "
        f"to {target_height_m:.3f} m. "
        "Confirm the surrounding environment is clear and safe to avoid collisions. "
        "Enter y to continue; any other input cancels: "
    )
    return response.strip().lower() == "y"


def main():
    # Get and initialize the G3 robot instance.
    robot = GalbotRobot()
    print("Initializing robot...")
    if not robot.init():
        print("System initialization failed!")
        return
    print("System initialized successfully!")

    # Wait for WBC communication and state data to become ready.
    time.sleep(2)

    height_offset_m = 0.20
    duration_s = 4.0

    height_valid, initial_height_m = get_leg_height(robot, "Initial leg height")
    if not height_valid:
        print("Failed to get a valid initial leg height.")
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return

    lowered_height_m = initial_height_m - height_offset_m
    if not confirm_leg_height_motion(initial_height_m, lowered_height_m):
        print("Leg height motion cancelled. Exiting program.")
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return

    print(
        f"Lowering leg height by {height_offset_m:.3f} m "
        f"to {lowered_height_m:.3f} m."
    )
    status = robot.set_leg_height(lowered_height_m, duration_s, True)
    print_status("set_leg_height(lowered)", status)

    lowered_height_valid, actual_lowered_height_m = get_leg_height(
        robot, "Height after lowering"
    )
    if not lowered_height_valid:
        print("Failed to get the current leg height; cancelling the restore motion.")
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return
    print(
        "Lowered target error: "
        f"{abs(actual_lowered_height_m - lowered_height_m):.3f} m."
    )

    if not confirm_leg_height_motion(actual_lowered_height_m, initial_height_m):
        print("Leg height restore motion cancelled. Exiting program.")
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()
        return

    print(f"Restoring initial leg height: {initial_height_m:.3f} m.")
    status = robot.set_leg_height(initial_height_m, duration_s, True)
    print_status("set_leg_height(initial)", status)

    _, final_height_m = get_leg_height(robot, "Final leg height")
    print(
        "Restored height error: "
        f"{abs(final_height_m - initial_height_m):.3f} m."
    )

    time.sleep(0.5)
    print("Switching controller back to leg_pvt_ctrl...")
    status = robot.switch_controller(G3ControllerName.LEG_PVT_CTRL)
    print_status("switch_controller(leg_pvt_ctrl)", status)

    print("Waiting 1 second before shutdown...")
    time.sleep(1)

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    print("Resources released successfully.")


if __name__ == "__main__":
    main()
