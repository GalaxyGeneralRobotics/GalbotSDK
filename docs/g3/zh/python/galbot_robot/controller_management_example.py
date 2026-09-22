import time

from galbot_sdk.g3 import GalbotRobot, ControlStatus, G3ControllerName, G3JointGroup


def print_active_controller(group_name, controller_name):
    print(f"Active controller for {group_name}: {controller_name}")


def print_status(operation, status):
    if status == ControlStatus.SUCCESS:
        print(f"{operation}: SUCCESS")
    else:
        # Assuming status can be cast to int or printed directly
        print(f"{operation}: FAILED (Status: {status})")


def main():
    # Get robot instance and init
    robot = GalbotRobot()
    print("Initializing robot...")
    if robot.init():
        print("System initialized successfully!")
    else:
        print("System initialization failed!")
        return

    # Wait for data readiness
    time.sleep(1)

    # Define the controller we want to test
    # Using LEFT_ARM_PVT_CTRL as the test subject
    test_controller = G3ControllerName.LEFT_ARM_PVT_CTRL
    controller_str = "LEFT_ARM_PVT_CTRL"
    # Using string for display and G3JointGroup enum for querying
    group_str = "left_arm"
    test_group = G3JointGroup.left_arm

    print("\n--- Starting Controller Management Example ---")

    # 1. Get active controller before switching
    active_controller = robot.get_active_controller(test_group)
    print_active_controller(group_str, active_controller)

    # 2. Switch controller
    print(f"\n[Test 1] Switching to {controller_str}...")
    status = robot.switch_controller(test_controller)
    print_status("switch_controller", status)

    time.sleep(0.5)
    active_controller = robot.get_active_controller(test_group)
    print_active_controller(group_str, active_controller)

    # 3. Stop controller
    print(f"\n[Test 2] Stopping controller for group {group_str}...")
    # Using string overload/version for group
    status = robot.stop_controller(group_str)
    print_status("stop_controller", status)

    time.sleep(0.5)

    # 4. Release controller
    print(f"\n[Test 3] Releasing controller for group {group_str}...")
    status = robot.release_controller(group_str)
    print_status("release_controller", status)

    time.sleep(0.5)

    # 5. Acquire controller
    print(f"\n[Test 4] Acquiring {controller_str}...")
    status = robot.acquire_controller(test_controller)
    print_status("acquire_controller", status)

    time.sleep(0.5)

    # 6. Start controller
    print(f"\n[Test 5] Starting controller for group {group_str}...")
    status = robot.start_controller(group_str)
    print_status("start_controller", status)

    time.sleep(0.5)

    print("\n--- Controller Management Example Finished ---")

    # Shutdown
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    print("Resources released successfully")


if __name__ == "__main__":
    main()
