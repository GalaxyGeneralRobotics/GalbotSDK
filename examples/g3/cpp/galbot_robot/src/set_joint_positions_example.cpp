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

    // Joint groups to control; passing an empty array controls leg, head, left_arm, and right_arm by default
    joint_groups = {"head"};
    // Specific joints to control; if provided, this overrides the joint_groups parameter
    std::vector<std::string> joint_names = {};
    // Joint positions; head joint group contains two joints
    std::vector<double> joint_pos = {0.2, 0.2};
    // Whether to block and wait for joint angles to reach position or timeout
    bool is_block = true;
    // Maximum joint speed (rad/s)
    double speed_rad_s = 0.1;
    // Maximum wait time (seconds)
    double timeout_s = 10.0;

    // Set joint positions
    ControlStatus joint_execution_status =
        robot.set_joint_positions(joint_pos, joint_groups,joint_names, 
            is_block, speed_rad_s,timeout_s);

    if (joint_execution_status == ControlStatus::SUCCESS) {
        std::cout << "Joint command set successfully!" << std::endl;
    } else {
        std::cerr << "Failed to set joint command!" << std::endl;
    }

    // Query joint positions by group; empty array defaults to leg, head, dual-arm groups. Second parameter specifies joint names, which overrides joint_groups if provided.
    auto ret_positions = robot.get_joint_positions(joint_groups, {});
    for (auto position : ret_positions) {
        std::cout << "joint positions is " << position << std::endl;
    }

    // Use specific joint names for control; this parameter overrides joint_groups
    joint_names = {"head_joint1", "head_joint2"};
    joint_pos = {0.0, 0.0};

    // Set joint positions
    joint_execution_status = robot.set_joint_positions(joint_pos, joint_groups,joint_names, 
            is_block, speed_rad_s,timeout_s);

    if (joint_execution_status == ControlStatus::SUCCESS) {
        std::cout << "Joint command set successfully!" << std::endl;
    } else {
        std::cerr << "Failed to set joint command!" << std::endl;
    }

    // Query joint positions by group; empty array defaults to leg, head, dual-arm groups. Second parameter specifies joint names, which overrides joint_groups if provided.
    ret_positions = robot.get_joint_positions(joint_groups, {});
    for (auto position : ret_positions) {
        std::cout << "joint positions is " << position << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}