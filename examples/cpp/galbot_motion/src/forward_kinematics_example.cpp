#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <thread>
#include <chrono>
#include <tuple>
#include <algorithm>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

// 辅助打印函数
void print_pose(const std::string& label, 
                const std::tuple<MotionStatus, std::vector<double>>& res, 
                GalbotMotion& planner) 
{
    std::cout << "[" << label << "] 状态: " << planner.status_to_string(std::get<0>(res)) << std::endl;
    
    if (std::get<0>(res) == MotionStatus::SUCCESS) {
        std::cout << "末端位姿: ";
        for (double v : std::get<1>(res)) {
            std::cout << v << " ";
        }
        std::cout << "\n" << std::endl;
    } else {
        std::cout << "计算失败！" << std::endl;
    }
}

int main() {

    auto& planner = GalbotMotion::get_instance();
    auto& robot = GalbotRobot::get_instance();

    if (planner.init()) {
        std::cout << "规划器初始化成功！" << std::endl;
    } else {
        std::cerr << "规划器初始化失败！" << std::endl;
        return -1;
    }
    
    if (robot.init()) {
        std::cout << "系统初始化成功！" << std::endl;
    } else {
        std::cerr << "系统初始化失败！" << std::endl;
        return -1;
    }

    // 程序立即启动，稍等数据就绪时间
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::map<std::string, std::vector<double>> chain_joints = {
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

    std::vector<double> base_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::string end_link = "left_arm_end_effector_mount_link";
    std::string reference_frame = "base_link";

    // --- 测试用例 1: 默认参数 (使用当前机器人状态) ---
    try {
        std::cout << ">> 正在执行: 基础版正运动学..." << std::endl;
        auto res1 = planner.forward_kinematics(end_link, reference_frame);
        print_pose("基础版", res1, planner);
    } catch (const std::exception& e) {
        std::cerr << "❌ 基础版异常: " << e.what() << std::endl;
    }

    // --- 测试用例 2: 自定义关节状态 + 参数 ---
    try {
        std::cout << ">> 正在执行: 自定义关节正运动学..." << std::endl;

        std::unordered_map<std::string, std::vector<double>> custom_joint_state = {
            {"left_arm", chain_joints["left_arm"]}
        };
        auto custom_param_ptr = std::make_shared<Parameter>(); 
        auto res2 = planner.forward_kinematics(end_link, reference_frame, custom_joint_state, custom_param_ptr);

        print_pose("自定义参数", res2, planner);
    } catch (const std::exception& e) {
        std::cerr << "❌ 自定义参数异常: " << e.what() << std::endl;
    }

    // --- 测试用例 3: 基于 RobotStates 的正运动学 ---
    try {
        std::cout << ">> 正在执行: 基于 RobotStates 正运动学..." << std::endl;

        auto ref_robot_state_ptr = std::make_shared<RobotStates>();
        ref_robot_state_ptr->whole_body_joint = whole_body_joint;
        ref_robot_state_ptr->base_state = base_state;

        auto res3 = planner.forward_kinematics_by_state(end_link, ref_robot_state_ptr, reference_frame, std::make_shared<Parameter>());

        print_pose("基于RobotStates", res3, planner);
    } catch (const std::exception& e) {
        std::cerr << "❌ 基于RobotStates异常: " << e.what() << std::endl;
    }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
