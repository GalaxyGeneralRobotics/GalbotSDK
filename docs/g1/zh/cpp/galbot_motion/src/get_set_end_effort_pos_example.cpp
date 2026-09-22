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

int main() {

    auto& planner = GalbotMotion::get_instance(MachineType::G1);
    auto& robot = GalbotRobot::get_instance(MachineType::G1);

    if (!planner.init()) {
        std::cerr << "GalbotMotion initialization failed" << std::endl;
        return -1;
    }
    if (!robot.init()) {
        std::cerr << "GalbotRobot initialization failed" << std::endl;
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::unordered_map<std::string, std::vector<double>> chain_pose_baselink = {
        {"leg",       {0.0596, -0.0000, 1.0327, 0.5000, 0.5003, 0.4997, 0.5000}},
        {"head",      {0.0599, 0.0002, 1.4098, -0.7072, 0.0037, 0.0037, 0.7069}},
        {"left_arm",  {0.1267, 0.2342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991}},
        {"right_arm", {0.097768, -0.226021, 0.8, -0.0117403, -0.0098713, 0.0157502, 0.999758}}
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
