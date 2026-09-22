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

    // Get gripper state
    auto gripper_state_ptr = robot.get_gripper_state("left_gripper");

    if (gripper_state_ptr == nullptr) {
        std::cerr << "get gripper state error" << std::endl;
    } else {
        std::cout << "Left gripper state:" << std::endl;
        print_gripper_state(gripper_state_ptr);
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
