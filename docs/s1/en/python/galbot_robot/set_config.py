import time

from galbot_sdk.s1 import ConfigService, ConfigItem, ControlStatus, GalbotRobot

# Full list of settable fields (navigation service, camera services, motion
# planning service and control service), their meaning, type and valid range:
# docs/s1/en/set_config_reference.md


def example_navigation(robot):
    # Navigation service: adjust replanning thresholds and select the short-distance
    # motion mode. omni_plan is G1-only (passing it on S1 returns INVALID_INPUT), so S1
    # uses adjust_motion_type instead -- 2 = rotate + crab-walk, the recommended S1 mode
    # (avoids joint collisions).
    status = robot.set_config(
        ConfigService.NAVIGATION,
        [
            ConfigItem("replan_threshold", 30.0),
            ConfigItem("no_replan_threshold", 1.0),
            ConfigItem("adjust_motion_type", 2),
        ],
    )
    if status == ControlStatus.SUCCESS:
        print("[Navigation] set_config succeeded. Restart the device for the new config to take effect.")
    else:
        print(f"[Navigation] set_config failed: {status}. Check the SDK log for details.")


def example_camera(robot):
    # Front head camera: set resolution.
    # color_width/color_height must be set together in the same call, and the
    # combination must be one of the documented valid resolutions.
    status = robot.set_config(
        ConfigService.FRONT_HEAD_CAMERA,
        [
            ConfigItem("color_width", 1280),
            ConfigItem("color_height", 992),
        ],
    )
    if status == ControlStatus.SUCCESS:
        print("[Camera] set_config succeeded. Restart the device for the new config to take effect.")
    else:
        print(f"[Camera] set_config failed: {status}. Check the SDK log for details.")


def example_motion_plan(robot):
    # Motion planning service: set a non-per-chain field (plan_timeout) together with
    # leg trajectory velocity/acceleration/jerk limits. Per-chain fields are keyed as
    # "{field_name}_{chain_name}"; the leg chain has only 1 joint on S1 (5 on G1), so the
    # array length here is S1-specific.
    status = robot.set_config(
        ConfigService.MOTION_PLAN,
        [
            ConfigItem("plan_timeout", 5.0),
            ConfigItem("max_velocity_leg", [0.8]),
            ConfigItem("max_acceleration_leg", [3.0]),
            ConfigItem("max_jerk_leg", [10.0]),
        ],
    )
    if status == ControlStatus.SUCCESS:
        print("[MotionPlan] set_config succeeded. Restart the device for the new config to take effect.")
    else:
        print(f"[MotionPlan] set_config failed: {status}. Check the SDK log for details.")


def example_control(robot):
    # Control service: report_error_skip is a general parameter shared by
    # both G-series and S1 (robot_config.toml).
    status = robot.set_config(
        ConfigService.CONTROL,
        [
            ConfigItem("report_error_skip", 1),
        ],
    )
    if status == ControlStatus.SUCCESS:
        print("[Control] set_config succeeded. Restart the device for the new config to take effect.")
    else:
        print(f"[Control] set_config failed: {status}. Check the SDK log for details.")


if __name__ == "__main__":
    robot = GalbotRobot()
    robot.init()
    time.sleep(2)
    print("Initialization succeeded")

    example_navigation(robot)
    example_camera(robot)
    example_motion_plan(robot)
    example_control(robot)

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    print("Resources released successfully")
