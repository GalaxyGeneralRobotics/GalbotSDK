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

using namespace galbot::sdk::g1;

// 轨迹返回类型定义
using TrajResult = std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>>;

/**
 * 辅助函数：打印规划结果
 */
void print_multi_plan_result(const std::string& label, const TrajResult& res, const std::string& chain_name, GalbotMotion& planner) {
    auto status = std::get<0>(res);
    auto traj_map = std::get<1>(res);

    std::cout << "[" << label << "] 状态反馈: " << planner.status_to_string(status) << std::endl;
    if (status == MotionStatus::SUCCESS) {
        if (traj_map.count(chain_name) && !traj_map[chain_name].empty()) {
            std::cout << "✅ 多点规划成功: 轨迹总点数 = " << traj_map[chain_name].size() << std::endl;
        } else {
            std::cout << "⚠️ 状态 SUCCESS 但轨迹为空，可能由于目标与当前位姿重合。" << std::endl;
        }
    } else {
        std::cout << "❌ 多点规划失败。" << std::endl;
    }
    std::cout << "---------------------------------------------------" << std::endl;
}

int main() {
    auto& planner = GalbotMotion::get_instance();
    auto& robot = GalbotRobot::get_instance();

    if (!planner.init()) {
        std::cerr << "GalbotMotion 初始化失败" << std::endl;
        return -1;
    }
    if (!robot.init()) {
        std::cerr << "GalbotRobot 初始化失败" << std::endl;
        return -1;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::unordered_map<std::string, std::vector<double>> chain_joints = {
        {"leg",       {0.4992, 1.4991, 1.0005, 0.0000, -0.0004}},
        {"head",      {0.0000, 0.0}},
        {"left_arm",  {1.9999, -1.6000, -0.5999, -1.6999, 0.0000, -0.7999, 0.0000}},
        {"right_arm", {-2.0000, 1.6001, 0.6001, 1.7000, 0.0000, 0.8000, 0.0000}}
    };

    std::vector<double> whole_body_joint;
    std::vector<std::string> keys = {"leg", "head", "left_arm", "right_arm"};
    for (const auto& key : keys) {
        whole_body_joint.insert(whole_body_joint.end(), chain_joints[key].begin(), chain_joints[key].end());
    }

    auto params = std::make_shared<Parameter>();
    std::string target_chain = "left_arm";

    // --- 场景 1：笛卡尔空间多路点规划 (PoseState 目标) ---
    try {
        std::cout << ">> 执行场景 1：笛卡尔空间多路点规划..." << std::endl;

        auto target_pose_state = std::make_shared<PoseState>();
        target_pose_state->chain_name = target_chain;

        // 构造路点（3个中间位姿）
        std::vector<std::vector<double>> waypoint_poses = {
            {0.1267, 0.2342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991},
            {0.2267, 0.2342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991},
            {0.3267, 0.2342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991},
            {0.4267, 0.2342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991}
        };

        auto res = planner.motion_plan_multi_waypoints(
            target_pose_state,
            waypoint_poses,
            nullptr,  // start
            nullptr,  // reference_robot_states
            false,    // enable_collision_check
            params    // params
        );

        print_multi_plan_result("笛卡尔多点单链规划", res, target_chain, planner);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) { std::cerr << "场景 1 异常: " << e.what() << std::endl; }

    // --- 场景 2：关节空间多路点规划 (JointStates 目标) ---
    try {
        std::cout << ">> 执行场景 2：关节空间多路点规划..." << std::endl;

        auto target_joint = std::make_shared<JointStates>();
        target_joint->chain_name = target_chain;

        // 构造路点（3个中间位姿）
        std::vector<std::vector<double>> waypoints = {
            {0.1267, 0.2342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991},
            {0.2267, 0.4342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991},
            {0.3267, 0.6342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991},
            {0.4267, 0.8342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991}
        };

        auto res = planner.motion_plan_multi_waypoints(
            target_joint,
            waypoints,
            nullptr,
            nullptr,
            false,
            params
        );

        print_multi_plan_result("关节空间多点", res, target_chain, planner);
    } catch (const std::exception& e) { std::cerr << "场景 2 异常: " << e.what() << std::endl; }

    // 4. 清理资源
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return 0;
}
