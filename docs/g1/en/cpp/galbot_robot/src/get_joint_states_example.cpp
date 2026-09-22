#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_joint_states(const std::vector<JointState>& joint_states) {
    for (const auto& states : joint_states) {
        std::cout << "--- Joint State ---" << std::endl;
        std::cout << "Joint Name:   " << states.joint_name   << std::endl;
        std::cout << "Position:     " << states.position     << " rad" << std::endl;
        std::cout << "Velocity:     " << states.velocity     << " rad/s" << std::endl;
        std::cout << "Acceleration: " << states.acceleration << " rad/s^2" << std::endl;
        std::cout << "Effort:       " << states.effort       << " Nm" << std::endl;
        std::cout << "Current:      " << states.current      << " A" << std::endl;
        std::cout << "Timestamp:    " << states.timestamp_ns << " ns" << std::endl;
        std::cout << "------------------" << std::endl;
    }
}

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::G1);

    // Initialize system
    if (robot.init()) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    // Program started, waiting for data
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Get joint states by joint group names; returns all joints if empty
    std::vector<std::string> joint_groups = {"left_arm"};
    auto ret_states = robot.get_joint_states(joint_groups, {});
    if (ret_states.empty()) {
        std::cout << "No joint states returned for the specified joint groups" << std::endl;
    } else {
        print_joint_states(ret_states);
    }

    // Get specified joint states; if provided, overrides joint group input
    std::vector<std::string> joint_names = {"left_arm_joint1", "left_arm_joint2"};
    ret_states = robot.get_joint_states(joint_groups, joint_names);
    if (ret_states.empty()) {
        std::cout << "No joint states returned for the specified joint names" << std::endl;
    } else {
        print_joint_states(ret_states);
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
