#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

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

    // Wait for data to become available
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Get current WBC end-effector poses
    // Keys: "ree_pose", "lee_pose", "head_pose" — each is [x, y, z, qx, qy, qz, qw]
    auto ee_info = robot.get_wbc_end_effector_poses(); // Before using this function, ensure that `using_wbc` in the configuration file is set to `true`.
    std::cout << "\nCurrent WBC end-effector poses:" << std::endl;
    for (const auto& [frame, pose] : ee_info) {
        std::cout << "  Frame: " << frame << ", Pose: [";
        for (size_t i = 0; i < pose.size(); ++i) {
            std::cout << pose[i];
            if (i + 1 < pose.size()) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    auto it = ee_info.find("ree_pose");
    if (it != ee_info.end() && it->second.size() >= 7) {
        std::vector<double> target_pose(it->second.begin(), it->second.begin() + 7);
        target_pose[0] -= 0.1;  // Move 10 cm in -X direction

        // Example: absolute pose — omit reference_frames to default every pose to "world"
        ControlStatus status = robot.set_end_effector_command(
            {target_pose},
            {"right_arm_end_effector_mount_link"}
        );

        if (status == ControlStatus::SUCCESS) {
            std::cout << "\nPublished end-effector command (default reference_frames -> world)" << std::endl;
        } else {
            std::cerr << "Failed to publish end-effector command!" << std::endl;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // Clear end-effector commands
    ControlStatus clear_status = robot.clear_end_effector_command();
    if (clear_status == ControlStatus::SUCCESS) {
        std::cout << "End-effector commands cleared successfully!" << std::endl;
    } else {
        std::cerr << "Failed to clear end-effector commands!" << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    std::cout << "Resource cleanup successful" << std::endl;

    return 0;
}
