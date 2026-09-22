#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_joint_positions(const std::vector<double>& positions) {
    std::cout << "Current joint positions:" << std::endl;
    for (size_t i = 0; i < positions.size(); ++i) {
        std::cout << "  joint " << i << ": " << positions[i] << " rad" << std::endl;
    }
    std::cout << std::endl;
}

std::string execution_status_to_string(ControlStatus status) {
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

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    // Initialize system
    if (robot.init()) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    // data
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::vector<std::string> joint_groups = {"head"};
    std::vector<std::string> joint_names = {};

    std::vector<JointCommand> joint_commands(2);
    joint_commands[0].position = 0.2;
    joint_commands[1].position = 0.2;
    // Set head joint angles to 0.3 0.3
    ControlStatus execution_status =
        robot.set_joint_commands(joint_commands, joint_groups, joint_names);
    if (execution_status != ControlStatus::SUCCESS) {
        std::cout << "Joint angle command sending failed" << std::endl;
    } else {
        std::cout << "Joint angle command sent successfully" << std::endl;
    }

    // , waitexecute
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

    // Query joint positions
    auto ret_positions = robot.get_joint_positions(joint_groups, {});
    print_joint_positions(ret_positions);

    // Step 2: Return to initial position —— set both head joints to 0.0 rad
    joint_commands[0].position = 0.0;
    joint_commands[1].position = 0.0;

    // Set joint commands by joint names; setting joint_names overrides joint_groups
    joint_groups = {""};
    joint_names = {"head_joint1", "head_joint2"};
    execution_status =
        robot.set_joint_commands(joint_commands, joint_groups, joint_names);
    if (execution_status != ControlStatus::SUCCESS) {
        std::cout << "Joint angle command sending failed" << std::endl;
    } else {
        std::cout << "Joint angle command sent successfully" << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    std::cout << "\nProgram exited" << std::endl;
    return 0;
}
