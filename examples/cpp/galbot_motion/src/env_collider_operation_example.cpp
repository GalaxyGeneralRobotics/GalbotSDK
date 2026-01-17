#include <iostream>
#include <vector>
#include <string>
#include <array>
#include <thread>
#include <chrono>
#include <memory>
#include <stdexcept>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

/*
    NOTE:
    - GalbotMotion does NOT provide real-time obstacle perception / automatic environment updates today.
    - To make Motion collision checking consider environmental obstacles, you must load them manually
        (e.g., box/mesh/point_cloud via add_obstacle/attach_target_object).
    - Real-time perception / navigation-style obstacle updates in Motion is a planned future feature.
*/

int main() {
    auto& planner = GalbotMotion::get_instance();
    auto& robot = GalbotRobot::get_instance();

    if (!planner.init()) {
        std::cerr << "GalbotMotion init FAILED" << std::endl;
        return -1;
    }
    if (!robot.init()) {
        std::cerr << "GalbotRobot init FAILED" << std::endl;
        return -1;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Scenario 1: add a Box collision object into Motion environment.
    try {
        std::cout << ">> Scenario 1: add obstacle..." << std::endl;

        std::string obstacle_id = "box_test_1";
        std::string obj_type = "box";
        std::vector<double> obj_pose = {1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        std::array<double, 3> obj_scale = {1.0, 1.0, 1.0};
        std::string target_frame = "world";

        MotionStatus status = planner.add_obstacle(
            obstacle_id,      // 1. ID
            obj_type,         // 2. 类型
            obj_pose,         // 3. 位姿
            obj_scale,        // 4. 缩放 (array)
            "",               // 5. key
            target_frame,     // 6. target_frame
            "",               // 7. ee_frame
            {},               // 8. ref_joints
            {},               // 9. ref_base
            {},               // 10. ignore_links
            0.0,              // 11. safe_margin
            0.0               // 12. resolution
        );

        std::cout << "Status: " << planner.status_to_string(status) << std::endl;
        
        planner.clear_obstacle();
        
        if (status == MotionStatus::SUCCESS) {
            std::cout << "OK: obstacle added" << std::endl;
        }
    } catch (const std::exception& e) { std::cerr << e.what() << std::endl; }

    // Scenario 2: add a duplicate ID (expected to fail).
    try {
        std::cout << "\n>> Scenario 2: test duplicate ID..." << std::endl;

        std::string obstacle_id = "box_test_2";
        std::string obj_type = "box";
        std::vector<double> obj_pose = {1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        std::array<double, 3> obj_scale = {1.0, 1.0, 1.0};
        std::string target_frame = "world";

        // 第一次添加
        planner.add_obstacle(obstacle_id, obj_type, obj_pose, obj_scale, "", target_frame, "", {}, {}, {}, 0.0, 0.0);

        // 第二次添加相同 ID
        MotionStatus status = planner.add_obstacle(obstacle_id, obj_type, obj_pose, obj_scale, "", target_frame, "", {}, {}, {}, 0.0, 0.0);

        std::cout << "Status: " << planner.status_to_string(status) << std::endl;
        
        planner.clear_obstacle();

        if (status == MotionStatus::FAULT) {
            std::cout << "OK: duplicate ID rejected (expected)" << std::endl;
        }
    } catch (const std::exception& e) { std::cerr << e.what() << std::endl; }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return 0;
}
