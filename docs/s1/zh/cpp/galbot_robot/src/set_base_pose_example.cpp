#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <cmath>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

// Helper function: calculate quaternion
void set_yaw_orientation(Pose& pose, double yaw) {
    const double half_yaw = 0.5 * yaw;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(half_yaw);
    pose.orientation.w = std::cos(half_yaw);
}

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::S1);

    // Initialize system
    if (robot.init()) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    // Program started, waiting for data
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // ========== Test Version 1: Use Pose struct ==========
    std::cout << "\n[Test 1] Set chassis pose using Pose struct..." << std::endl;
    {
        Pose pose;
        pose.position.x = 0.5;    // Target x coordinate
        pose.position.y = 0.0;    // Target y coordinate
        pose.position.z = 0.0;    // z coordinate (chassis ignored)
        set_yaw_orientation(pose, 0.0);  // Oriented to 0 radians

        if (robot.set_base_pose(pose, true, 15.0) == ControlStatus::SUCCESS) {
            std::cout << "[Test 1] Pose-struct set succeeded, target position reached." << std::endl;
        } else {
            std::cerr << "[Test 1] Pose-struct set failed or timed out." << std::endl;
        }
    }

    // Wait
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ========== Test Version 2: Simplified coordinate version (default 1s motion time) ==========
    std::cout << "\n[Test 2] Use simplified coordinate version (default 1s motion time)..." << std::endl;
    {
        double x = 1.0;           // Target x coordinate
        double y = 0.0;           // Target y coordinate
        double yaw = 0.0;         // Target orientation
        // std::string frame_id = "base_link";
        std::string frame_id = "rel(0)";
        std::string reference_frame_id = "odom";
        bool is_blocking = true;
        double timeout_s = 15.0;

        if (robot.set_base_pose(x, y, yaw, frame_id, reference_frame_id, is_blocking, timeout_s) == ControlStatus::SUCCESS) {
            std::cout << "[Test 2] Simplified-coordinate set succeeded, target position reached." << std::endl;
        } else {
            std::cerr << "[Test 2] Simplified-coordinate set failed or timed out." << std::endl;
        }
    }

    // Wait
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // ========== Test Version 3: Full control version (custom motion time) ==========
    std::cout << "\n[Test 3] Use full control version (custom 5s motion time)..." << std::endl;
    {
        double x = 0.0;           // Target x coordinate (return to origin)
        double y = 0.0;           // Target y coordinate
        double yaw = 0.0;         // Target orientation
        // std::string frame_id = "base_link";
        std::string frame_id = "rel(0)";
        std::string reference_frame_id = "odom";
        double time_from_start_s = 5.0;  // Custom 5-second motion (slower and more stable)
        bool is_blocking = true;
        double timeout_s = 15.0;

        if (robot.set_base_pose(x, y, yaw, frame_id, reference_frame_id, time_from_start_s, is_blocking, timeout_s) == ControlStatus::SUCCESS) {
            std::cout << "[Test 3] Full-control set succeeded, target position reached." << std::endl;
        } else {
            std::cerr << "[Test 3] Full-control set failed or timed out." << std::endl;
        }
    }

    std::cout << "\nAll tests completed!" << std::endl;

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
