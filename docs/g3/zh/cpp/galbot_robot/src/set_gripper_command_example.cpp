#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_gripper_state(std::shared_ptr<GripperState> gripper_state) {
    std::cout << "Timestamp (ns): " << gripper_state->timestamp_ns << std::endl;

    std::cout << " width "  << gripper_state->width << " velocity " << gripper_state->velocity
                << " effort " << gripper_state->effort << " is moving "
                << gripper_state->is_moving << std::endl;
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

    // Program started, waiting for data
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Gripper width (m)
    double width_m = 0.02;
    // Gripper speed (m/s)
    double velocity_mps = 0.05;
    // Gripper torque (N·m)
    double effort = 10;
    // Whether to block until execution completes
    bool is_blocking = true;
    // Set left gripper width to 0.02m, speed 0.05m/s, torque 10, block until execution completes
    ControlStatus gripper_execution_status =
        robot.set_gripper_command("left_gripper", width_m, velocity_mps,
                                        effort, is_blocking);

    if (gripper_execution_status == ControlStatus::SUCCESS) {
        std::cout << "Gripper command set successfully!" << std::endl;
    } else {
        std::cerr << "Failed to set gripper command!" << std::endl;
    }

    // Get gripper state
    JointStateMessage joint_state;
    auto gripper_state_ptr = robot.get_gripper_state("left_gripper");

    if (gripper_state_ptr == nullptr) {
        std::cerr << "get gripper state error" << std::endl;
    } else {
        print_gripper_state(gripper_state_ptr);
    }

    // Gripper width (m)
    width_m = 0.1;
    // Gripper speed (m/s)
    velocity_mps = 0.05;
    // Gripper torque (N·m)
    effort = 10;
    // Whether to block until execution completes
    is_blocking = false;
    // Set left gripper width to 0.1m, speed 0.05m/s, torque 10, block until execution completes
    gripper_execution_status =
        robot.set_gripper_command("left_gripper", width_m, velocity_mps,
                                        effort, is_blocking);

    if (gripper_execution_status == ControlStatus::SUCCESS) {
        std::cout << "Gripper command set successfully!" << std::endl;
    } else {
        std::cerr << "Failed to set gripper command!" << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
