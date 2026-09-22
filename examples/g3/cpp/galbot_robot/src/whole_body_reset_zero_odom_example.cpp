#include <iostream>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {
const char* control_status_to_string(ControlStatus status) {
    switch (status) {
        case ControlStatus::SUCCESS:
            return "SUCCESS";
        case ControlStatus::TIMEOUT:
            return "TIMEOUT";
        case ControlStatus::FAULT:
            return "FAULT";
        case ControlStatus::INVALID_INPUT:
            return "INVALID_INPUT";
        case ControlStatus::INIT_FAILED:
            return "INIT_FAILED";
        case ControlStatus::IN_PROGRESS:
            return "IN_PROGRESS";
        case ControlStatus::STOPPED_UNREACHED:
            return "STOPPED_UNREACHED";
        case ControlStatus::DATA_FETCH_FAILED:
            return "DATA_FETCH_FAILED";
        case ControlStatus::PUBLISH_FAIL:
            return "PUBLISH_FAIL";
        case ControlStatus::COMM_DISCONNECTED:
            return "COMM_DISCONNECTED";
        default:
            return "UNKNOWN_STATUS";
    }
}
}  // namespace

int main() {
    auto& robot = GalbotRobot::get_instance(MachineType::G3);
    auto& motion = GalbotMotion::get_instance(MachineType::G3);

    if (!robot.init()) {
        std::cerr << "GalbotRobot init failed." << std::endl;
        return -1;
    }
    if (!motion.init()) {
        std::cerr << "GalbotMotion init failed." << std::endl;
        return -1;
    }

    // Optional frames (frame_id: base_link/odom/map, reference_frame_id: odom/map)
    const std::string frame_id = "base_link";
    const std::string reference_frame_id = "odom";

    // reset to zero
    auto result = robot.zero_whole_body_and_base(
        frame_id,
        reference_frame_id,
        /*is_blocking=*/true,
        /*leg_head_speed_rad_s=*/0.2,
        /*leg_head_timeout_s=*/15.0,
        /*params=*/default_param);
    
    std::cout << "Zero joint status: " << motion.status_to_string(result.first) << std::endl;
    std::cout << "Zero base status: " << control_status_to_string(result.second) << std::endl;

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
