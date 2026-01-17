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

using namespace galbot::sdk::g1;

// 定义轨迹返回类型以简化代码
using TrajResult = std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>>;

/**
 * 辅助函数：打印规划结果
 */
void print_plan_info(const std::string& label, const TrajResult& res, const std::string& chain_name, GalbotMotion& planner) {
    auto status = std::get<0>(res);
    auto traj_map = std::get<1>(res);

    std::cout << "[" << label << "] 状态: " << planner.status_to_string(status) << std::endl;
    if (status == MotionStatus::SUCCESS) {
        if (traj_map.count(chain_name) && !traj_map[chain_name].empty()) {
            std::cout << "✅ 规划成功: 轨迹点数 = " << traj_map[chain_name].size() << std::endl;
        } else {
            std::cout << "⚠️ 状态为 SUCCESS 但轨迹为空（可能已在目标点）" << std::endl;
        }
    } else {
        std::cout << "❌ 规划失败" << std::endl;
    }
    std::cout << "---------------------------------------" << std::endl;
}

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
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::unordered_map<std::string, std::vector<double>> chain_joints = {
        {"leg",       {0.4992, 1.4991, 1.0005, 0.0000, -0.0004}},
        {"head",      {0.0000, 0.0}},
        {"left_arm",  {1.9999, -1.6000, -0.5999, -1.6999, 0.0000, -0.7999, 0.0000}},
        {"right_arm", {-2.0000, 1.6001, 0.6001, 1.7000, 0.0000, 0.8000, 0.0000}}
    };

    std::unordered_map<std::string, std::vector<double>> chain_pose_baselink = {
        {"leg",       {0.0596, -0.0000, 1.0327, 0.5000, 0.5003, 0.4997, 0.5000}},
        {"head",      {0.0599, 0.0002, 1.4098, -0.7072, 0.0037, 0.0037, 0.7069}},
        {"left_arm",  {0.1267, 0.2342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991}},
        {"right_arm", {0.1267, -0.2345, 0.7358, -0.0225, 0.0126, -0.0343, 0.9991}}
    };

    auto params = std::make_shared<Parameter>();
    std::string target_chain = "left_arm";

    // NOTE:
    // - GalbotMotion does NOT provide real-time obstacle perception / automatic environment updates today.
    // - When enable_collision_check=true, collision checking uses self-collision + objects you load manually via
    //   add_obstacle/attach_target_object (including point clouds if you load them explicitly).

    // Scenario 1: joint-space planning, target type = joint state
    try {
        std::cout << ">> Scenario 1: joint-space planning (joint target)..." << std::endl;

        // 构造目标关节状态
        auto target_joint = std::make_shared<JointStates>();
        target_joint->chain_name = target_chain;
        target_joint->joint_positions = chain_joints[target_chain];

        auto res = planner.motion_plan(
            target_joint,   // 1. target
            nullptr,        // 2. start (nullptr表示从当前开始)
            nullptr,        // 3. reference_robot_states
            false,          // 4. enable_collision_check
            params          // 5. params
        );

        print_plan_info("Joint-space planning (joint target)", res, target_chain, planner);
    } catch (const std::exception& e) { std::cerr << e.what() << std::endl; }

    // Scenario 2: joint-space planning, target type = end-effector pose (Cartesian)
    try {
        std::cout << ">> Scenario 2: joint-space planning (pose target)..." << std::endl;

        // 构造目标位姿状态
        auto target_pose = std::make_shared<PoseState>();
        target_pose->chain_name = target_chain;
        target_pose->frame_id = "EndEffector";
        target_pose->reference_frame = "base_link";
        target_pose->pose = chain_pose_baselink[target_chain];

        auto res = planner.motion_plan(
            target_pose,    // 1. target
            nullptr,        // 2. start
            nullptr,        // 3. reference_robot_states
            false,          // 4. enable_collision_check
            params          // 5. params
        );

        print_plan_info("Joint-space planning (pose target)", res, target_chain, planner);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) { std::cerr << e.what() << std::endl; }

    // Scenario 3: joint-space planning with an explicit start state
    try {
        std::cout << ">> Scenario 3: joint-space planning (explicit start)..." << std::endl;

        auto target_joint = std::make_shared<JointStates>();
        target_joint->chain_name = target_chain;
        target_joint->joint_positions = chain_joints[target_chain];

        auto start_joint = std::make_shared<JointStates>();
        start_joint->chain_name = target_chain;
        start_joint->joint_positions = std::vector<double>(7, 0.0); // 7个0作为起点

        auto res = planner.motion_plan(
            target_joint, 
            start_joint,    // 2. 指定起始点
            nullptr, 
            false, 
            params
        );

        print_plan_info("Joint-space planning (explicit start)", res, target_chain, planner);
    } catch (const std::exception& e) { std::cerr << e.what() << std::endl; }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return 0;
}
