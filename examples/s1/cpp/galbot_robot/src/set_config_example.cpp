#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

// Full list of settable fields (navigation service, camera services, motion
// planning service and control service), their meaning, type and valid range:
// docs/s1/en/set_config_reference.md

using namespace galbot::sdk;

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::S1);

    // Initialize system
    if (robot.init()) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    // Program started, waiting for data
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Navigation service: adjust replanning thresholds and select the short-distance
    // motion mode. omni_plan is G1-only (passing it on S1 returns INVALID_INPUT), so S1
    // uses adjust_motion_type instead -- 2 = rotate + crab-walk, the recommended S1 mode
    // (avoids joint collisions).
    std::vector<ConfigItem> navigation_fields = {
        ConfigItem{"replan_threshold", 30.0},
        ConfigItem{"no_replan_threshold", 1.0},
        ConfigItem{"adjust_motion_type", static_cast<int64_t>(2)},
    };
    ControlStatus navigation_status = robot.set_config(ConfigService::NAVIGATION, navigation_fields);
    if (navigation_status == ControlStatus::SUCCESS) {
        std::cout << "[Navigation] set_config succeeded. Restart the device for the new config to take effect."
                  << std::endl;
    } else {
        std::cerr << "[Navigation] set_config failed. Check the SDK log for details." << std::endl;
    }

    // Front head camera: set resolution.
    // color_width/color_height must be set together in the same call, and the
    // combination must be one of the documented valid resolutions.
    std::vector<ConfigItem> camera_fields = {
        ConfigItem{"color_width", static_cast<int64_t>(1280)},
        ConfigItem{"color_height", static_cast<int64_t>(992)},
    };
    ControlStatus camera_status = robot.set_config(ConfigService::FRONT_HEAD_CAMERA, camera_fields);
    if (camera_status == ControlStatus::SUCCESS) {
        std::cout << "[Camera] set_config succeeded. Restart the device for the new config to take effect."
                  << std::endl;
    } else {
        std::cerr << "[Camera] set_config failed. Check the SDK log for details." << std::endl;
    }

    // Motion planning service: set a non-per-chain field (plan_timeout) together with
    // leg trajectory velocity/acceleration/jerk limits. Per-chain fields are keyed as
    // "{field_name}_{chain_name}"; the leg chain has only 1 joint on S1 (5 on G1), so the
    // array length here is S1-specific.
    std::vector<ConfigItem> motion_plan_fields = {
        ConfigItem{"plan_timeout", 5.0},
        ConfigItem{"max_velocity_leg", std::vector<double>{0.8}},
        ConfigItem{"max_acceleration_leg", std::vector<double>{3.0}},
        ConfigItem{"max_jerk_leg", std::vector<double>{10.0}},
    };
    ControlStatus motion_plan_status = robot.set_config(ConfigService::MOTION_PLAN, motion_plan_fields);
    if (motion_plan_status == ControlStatus::SUCCESS) {
        std::cout << "[MotionPlan] set_config succeeded. Restart the device for the new config to take effect."
                  << std::endl;
    } else {
        std::cerr << "[MotionPlan] set_config failed. Check the SDK log for details." << std::endl;
    }

    // Control service: report_error_skip is a general parameter shared by
    // both G-series and S1 (robot_config.toml).
    std::vector<ConfigItem> control_fields = {
        ConfigItem{"report_error_skip", static_cast<int64_t>(1)},
    };
    ControlStatus control_status = robot.set_config(ConfigService::CONTROL, control_fields);
    if (control_status == ControlStatus::SUCCESS) {
        std::cout << "[Control] set_config succeeded. Restart the device for the new config to take effect."
                  << std::endl;
    } else {
        std::cerr << "[Control] set_config failed. Check the SDK log for details." << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
