#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

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

    // Get specified joint names; joint groups include ["leg", "head", "left_arm", "right_arm"]
    std::vector<std::string> joint_groups = {"head"};
    bool only_active_joint = true;  // Get active joints
    auto head_joint_names_vec =
        robot.get_joint_names(only_active_joint, joint_groups);
    std::cout << "Head joint names:" << std::endl;
    for (size_t i = 0; i < head_joint_names_vec.size(); ++i) {
        std::cout << i << ": " << head_joint_names_vec[i] << std::endl;
    }

    // Passing an empty array returns all joint group information by default
    std::vector<std::string> null_vec = {};
    auto all_joint_names_vec =
        robot.get_joint_names(only_active_joint, null_vec);
    std::cout << "All joint names:" << std::endl;
    for (size_t i = 0; i < all_joint_names_vec.size(); ++i) {
        std::cout << i << ": " << all_joint_names_vec[i] << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
