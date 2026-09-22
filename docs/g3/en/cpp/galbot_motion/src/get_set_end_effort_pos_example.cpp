#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <tuple>
#include <memory>
#include <stdexcept>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

// Helper function: print pose information
void print_pose_info(const std::string& label, const std::vector<double>& pose) {
    if (pose.size() == 7) {
        std::cout << "[" << label << "] Pose: "
                  << "pos(" << pose[0] << ", " << pose[1] << ", " << pose[2] << "), "
                  << "ori(" << pose[3] << ", " << pose[4] << ", " << pose[5] << ", " << pose[6] << ")" 
                  << std::endl;
    }
}

bool confirm_robot_safety() {
    std::cout << "WARNING: Release the emergency stop and clear obstacles around the robot." << std::endl;
    std::cout << "Continue? (y/n): " << std::flush;

    std::string response;
    if (!std::getline(std::cin, response)) {
        return false;
    }
    return response == "y" || response == "Y";
}

int main() {

    if (!confirm_robot_safety()) {
        std::cout << "Example cancelled." << std::endl;
        return 0;
    }

    auto& planner = GalbotMotion::get_instance(MachineType::G3);
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    if (!planner.init()) {
        std::cerr << "GalbotMotion initialization failed" << std::endl;
        return -1;
    }
    if (!robot.init()) {
        std::cerr << "GalbotRobot initialization failed" << std::endl;
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    const std::vector<double> joint_pos = {
        0.5, 1.5, 1.0, 0.0, 0.0,
        0.0, 0.0,
        1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0,
        -1.5, 1.36, 0.45, -1.53, 0.1, 0.42, 0.0
    };
    const std::vector<std::string> joint_groups = {"leg", "head", "left_arm", "right_arm"};
    const ControlStatus move_status =
        robot.set_joint_positions(joint_pos, joint_groups, {}, true, 0.1, 30.0);
    if (move_status != ControlStatus::SUCCESS) {
        std::cerr << "Failed to move to the initial whole-body joint state: "
                  << static_cast<int>(move_status) << std::endl;
        robot.request_shutdown();
        robot.wait_for_shutdown();
        robot.destroy();
        return -1;
    }
    std::cout << "Moved to the initial whole-body joint state" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    std::unordered_map<std::string, std::vector<double>> chain_pose_baselink = {
        {"leg",       {0.0541, -0.0013, 1.0364, 0.4970, 0.4964, 0.5037, 0.5028}},
        {"head",      {0.0519, -0.0011, 1.4145, -0.7061, -0.0029, -0.0056, 0.7081}},
        {"left_arm",  {0.4827, 0.2329, 0.8383, 0.0405, 0.0396, -0.0604, 0.9966}},
        {"right_arm", {0.2818, -0.2430, 0.8389, -0.0456, 0.0479, 0.0573, 0.9962}}
    };

    std::string reference_frame = "base_link";
    std::string target_frame = "EndEffector";
    std::string target_chain = "left_arm";
    auto custom_param = std::make_shared<Parameter>();

    // --- Scenario 1: Get end-effector pose (basic version) ---
    try {
        std::cout << ">> Scenario 1: Getting the basic end-effector pose..." << std::endl;
        std::string end_ee_link = "left_arm_end_effector_mount_link";

        auto res = planner.get_end_effector_pose(end_ee_link, reference_frame);
        
        MotionStatus status = std::get<0>(res);
        std::vector<double> pose = std::get<1>(res);

        std::cout << "Execution status: " << planner.status_to_string(status) << std::endl;
        if (status == MotionStatus::SUCCESS) {
            print_pose_info("Basic version", pose);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) {
        std::cerr << "❌ Scenario 1 exception: " << e.what() << std::endl;
    }

    // --- Scenario 2: Get end-effector pose by specified chain name + custom frame ---
    try {
        std::cout << ">> Scenario 2: Getting pose by specified chain name..." << std::endl;

        auto res = planner.get_end_effector_pose_on_chain(target_chain, target_frame, reference_frame);
        
        MotionStatus status = std::get<0>(res);
        std::vector<double> pose = std::get<1>(res);

        std::cout << "Execution status: " << planner.status_to_string(status) << std::endl;
        if (status == MotionStatus::SUCCESS) {
            print_pose_info("Specified chain name version", pose);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) {
        std::cerr << "❌ Scenario 2 exception: " << e.what() << std::endl;
    }

    // --- Scenario 3: Set end-effector pose ---
    try {
        std::cout << ">> Scenario 3: Setting end-effector pose..." << std::endl;
        
        std::string ee_frame = "left_arm";
        std::vector<double> target_pose = chain_pose_baselink[ee_frame];

        MotionStatus status = planner.set_end_effector_pose(
            target_pose,        // 1
            ee_frame,           // 2
            reference_frame,    // 3
            nullptr,            // 4. Important: pass nullptr if no specific reference state is used
            false,              // 5. enable_collision_check
            true,               // 6. is_blocking
            5.0,                // 7. timeout
            custom_param        // 8. params
        );

        std::cout << "Set status: " << planner.status_to_string(status) << std::endl;
        if (status == MotionStatus::SUCCESS) {
            std::cout << "✅ Command sent successfully (blocking wait mode)" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ Pose-setting exception: " << e.what() << std::endl;
    }

    // --- Scenario 4: Get end-effector pose again after execution ---
    try {
        std::cout << ">> Scenario 4: Getting the basic end-effector pose..." << std::endl;
        std::string end_ee_link = "left_arm_end_effector_mount_link";

        auto res = planner.get_end_effector_pose(end_ee_link, reference_frame);
        
        MotionStatus status = std::get<0>(res);
        std::vector<double> pose = std::get<1>(res);

        std::cout << "Execution status: " << planner.status_to_string(status) << std::endl;
        if (status == MotionStatus::SUCCESS) {
            print_pose_info("Basic version", pose);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) {
        std::cerr << "❌ Scenario 4 exception: " << e.what() << std::endl;
    }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
