#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <tuple>
#include <memory>
#include <stdexcept>
#include <algorithm>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

// Trajectory return type definition
using TrajResult = std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>>;

/**
 * Helper function: print planning result
 */
void print_multi_plan_result(const std::string& label, const TrajResult& res, const std::string& chain_name, GalbotMotion& planner) {
    auto status = std::get<0>(res);
    auto traj_map = std::get<1>(res);

    std::cout << "[" << label << "] Status feedback: " << planner.status_to_string(status) << std::endl;
    if (status == MotionStatus::SUCCESS) {
        if (traj_map.count(chain_name) && !traj_map[chain_name].empty()) {
            std::cout << "✅ Multi-point planning succeeded: total trajectory points = " << traj_map[chain_name].size() << std::endl;
        } else {
            std::cout << "⚠️ Status is SUCCESS but trajectory is empty; target may overlap current pose." << std::endl;
        }
    } else {
        std::cout << "❌ Multi-point planning failed." << std::endl;
    }
    std::cout << "---------------------------------------------------" << std::endl;
}

int main() {
    std::cout << "WARNING: The robot will move to the initial joint state. "
              << "Release the emergency stop and clear nearby obstacles." << std::endl;
    std::cout << "Continue? (y/n): ";
    std::string response;
    std::getline(std::cin, response);
    if (response != "y" && response != "Y") {
        std::cout << "Example cancelled." << std::endl;
        return 0;
    }

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
    
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::unordered_map<std::string, std::vector<double>> chain_joints = {
        {"leg",       {0.5, 1.5, 1.0, 0.0, 0.0}},
        {"head",      {0.0, 0.0}},
        {"left_arm",  {2.0, -1.5, -0.6, -1.7, 0.0, -0.8, 0.0}},
        {"right_arm", {-2.0, 1.5, 0.6, 1.7, 0.0, 0.8, 0.0}}
    };

    std::vector<double> whole_body_joint;
    std::vector<std::string> keys = {"leg", "head", "left_arm", "right_arm"};
    for (const auto& key : keys) {
        whole_body_joint.insert(whole_body_joint.end(), chain_joints[key].begin(), chain_joints[key].end());
    }

    const ControlStatus move_status =
        robot.set_joint_positions(whole_body_joint, keys, {}, true, 0.1, 30.0);
    if (move_status != ControlStatus::SUCCESS) {
        std::cerr << "❌ Failed to move to the initial joint state." << std::endl;
        robot.request_shutdown();
        robot.wait_for_shutdown();
        robot.destroy();
        return -1;
    }
    std::cout << "✅ Moved to the initial whole-body joint state." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));

    auto params = std::make_shared<Parameter>();
    std::string target_chain = "left_arm";

    // NOTE: The two scenarios express the same physical waypoints as Cartesian
    // poses and joint positions, so their planning results are consistent. Calling
    // params->set_move_line(true) applies Cartesian-space line planning to both.
    // 注意：以下两个场景分别使用笛卡尔位姿和关节位置表达相同的物理路点，
    // 因此规划结果一致。调用 params->set_move_line(true) 后，二者均按
    // 笛卡尔空间直线方式规划。

    // --- Scenario 1: Multi-waypoint planning in Cartesian space (PoseState target) ---
    try {
        std::cout << ">> Running Scenario 1: Multi-waypoint planning in Cartesian space..." << std::endl;

        auto target_pose_state = std::make_shared<PoseState>();
        target_pose_state->chain_name = target_chain;

        // Construct waypoints (3 intermediate poses)
        std::vector<std::vector<double>> waypoint_poses = {
            {0.1297, 0.2608, 0.7417, 0.0734, 0.0209, -0.0233, 0.9968},
            {0.2297, 0.2608, 0.7417, 0.0734, 0.0209, -0.0233, 0.9968},
            {0.3297, 0.2608, 0.7417, 0.0734, 0.0209, -0.0233, 0.9968},
            {0.3297, 0.2608, 0.8417, 0.0734, 0.0209, -0.0233, 0.9968}
        };

        auto res = planner.motion_plan_multi_waypoints(
            target_pose_state,
            waypoint_poses,
            nullptr,  // start
            nullptr,  // reference_robot_states
            false,    // enable_collision_check
            params    // params
        );

        print_multi_plan_result("Cartesian multi-waypoint single-chain planning", res, target_chain, planner);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) { std::cerr << "Scenario 1 exception: " << e.what() << std::endl; }

    // --- Scenario 2: Multi-waypoint planning in joint space (JointStates target) ---
    try {
        std::cout << ">> Running Scenario 2: Multi-waypoint planning in joint space..." << std::endl;

        auto target_joint = std::make_shared<JointStates>();
        target_joint->chain_name = target_chain;

        // Construct waypoints (3 intermediate poses)
        std::vector<std::vector<double>> waypoints = {
            {2.0000, -1.5001, -0.6001, -1.7000, 0.0001, -0.7999, 0.0000},
            {1.7805, -1.4841, -0.5856, -1.6857, -0.0083, -0.5952, 0.0090},
            {1.5313, -1.4746, -0.5672, -1.5923, -0.0109, -0.4406, 0.0185},
            {1.6264, -1.4551, -0.5976, -1.9286, -0.0427, -0.1972, 0.0289}
        };

        auto res = planner.motion_plan_multi_waypoints(
            target_joint,
            waypoints,
            nullptr,
            nullptr,
            false,
            params
        );

        print_multi_plan_result("Joint-space multi-waypoint", res, target_chain, planner);
    } catch (const std::exception& e) { std::cerr << "Scenario 2 exception: " << e.what() << std::endl; }

    // 4. Clean up resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return 0;
}
