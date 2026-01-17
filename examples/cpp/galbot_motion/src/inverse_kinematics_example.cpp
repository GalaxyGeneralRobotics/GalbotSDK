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

// 辅助函数：打印逆运动学求解结果
void print_ik_result(const std::string& label, 
                     const std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<double>>>& res, 
                     GalbotMotion& planner) {
    auto status = std::get<0>(res);
    auto joint_map = std::get<1>(res);

    std::cout << "[" << label << "] 状态反馈: " << planner.status_to_string(status) << std::endl;
    
    if (status == MotionStatus::SUCCESS) {
        std::cout << "✅ 逆解计算成功！得到的关节角度如下:" << std::endl;
        for (const auto& [name, joints] : joint_map) {
            std::cout << "  - 链条 [" << name << "]: ";
            for (double v : joints) std::cout << v << " ";
            std::cout << std::endl;
        }
    } else {
        std::cout << "❌ 逆解计算失败，请检查输入或目标位姿是否可达。" << std::endl;
    }
    std::cout << "---------------------------------------------------" << std::endl;
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

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 关节状态定义
    std::unordered_map<std::string, std::vector<double>> chain_joints = {
        {"leg",       {0.4992, 1.4991, 1.0005, 0.0000, -0.0004}},
        {"head",      {0.0000, 0.0}},
        {"left_arm",  {1.9999, -1.6000, -0.5999, -1.6999, 0.0000, -0.7999, 0.0000}},
        {"right_arm", {-2.0000, 1.6001, 0.6001, 1.7000, 0.0000, 0.8000, 0.0000}}
    };

    // 目标位姿定义 (x, y, z, qx, qy, qz, qw)
    std::unordered_map<std::string, std::vector<double>> chain_pose_baselink = {
        {"left_arm",  {0.1267, 0.2342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991}},
        {"right_arm", {0.1267, -0.2345, 0.7358, -0.0225, 0.0126, -0.0343, 0.9991}}
    };

    // 全身体关节向量拼接 (Leg -> Head -> Left Arm -> Right Arm)
    std::vector<double> whole_body_joint;
    std::vector<std::string> key_order = {"leg", "head", "left_arm", "right_arm"};
    for (const auto& key : key_order) {
        whole_body_joint.insert(whole_body_joint.end(), chain_joints[key].begin(), chain_joints[key].end());
    }

    // 通用配置
    std::string reference_frame = "base_link";
    std::string target_frame = "EndEffector";
    std::string target_chain = "left_arm";
    auto params = std::make_shared<Parameter>();

    // 场景 1：单链逆解
    try {
        std::cout << ">> 执行场景 1：单链逆解测试..." << std::endl;
        std::vector<std::string> one_chain = {target_chain};

        auto res = planner.inverse_kinematics(
            chain_pose_baselink[target_chain], // 1. target_pose
            one_chain                         // 2. chain_names
            // target_frame,                      // 3. target_frame
            // reference_frame,                   // 4. reference_frame
            // {},                                // 5. initial_joint_positions (空)
            // false,                             // 6. enable_collision_check (bool)
            // params                             // 7. params (shared_ptr)
        );
        print_ik_result("单链逆解", res, planner);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) {
        std::cerr << "场景 1 异常: " << e.what() << std::endl;
    }

    // 场景 2：手臂链 + 腰部逆解
    try {
        std::cout << ">> 执行场景 2：手臂链 + 腰部逆解测试..." << std::endl;
        std::vector<std::string> chain_with_torso = {target_chain, "torso"};

        auto res = planner.inverse_kinematics(
            chain_pose_baselink[target_chain],
            chain_with_torso,
            target_frame,
            reference_frame,
            {},
            false,
            params
        );
        print_ik_result("手臂+腰部逆解", res, planner);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) {
        std::cerr << "场景 2 异常: " << e.what() << std::endl;
    }

    // 场景 3：非法链组合
    try {
        std::cout << ">> 执行场景 3：非法链组合测试..." << std::endl;
        std::vector<std::string> error_chains = {target_chain, "torso", "head"};

        auto res = planner.inverse_kinematics(
            chain_pose_baselink[target_chain],
            error_chains,
            target_frame,
            reference_frame,
            {},
            false,
            params
        );
        print_ik_result("非法链组合检测", res, planner);
    } catch (const std::exception& e) {
        std::cerr << "场景 3 异常: " << e.what() << std::endl;
    }

    // 场景 4：使用参考关节 (initial_joint_positions可指定链关节作为逆解参考值，未指定链关节使用全身关节补全)
    try {
        std::cout << ">> 执行场景 4：带初始参考值的逆解测试..." << std::endl;
        std::vector<std::string> one_chain = {target_chain};

        auto res = planner.inverse_kinematics(
            chain_pose_baselink[target_chain],
            one_chain,
            target_frame,
            reference_frame,
            chain_joints,
            false,
            params
        );
        print_ik_result("带参考值逆解", res, planner);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) {
        std::cerr << "场景 4 异常: " << e.what() << std::endl;
    }

    // 场景 5：基于 RobotStates 的逆解
    try {
        std::cout << ">> 执行场景 5：基于 RobotStates 的逆解测试..." << std::endl;
        
        // 构造 RobotStates 智能指针
        auto ref_state = std::make_shared<RobotStates>();
        ref_state->chain_name = target_chain;
        ref_state->whole_body_joint = whole_body_joint;
        ref_state->base_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

        std::vector<std::string> one_chain = {target_chain};

        auto res = planner.inverse_kinematics_by_state(
            chain_pose_baselink[target_chain], // 1. target_pose
            one_chain,                         // 2. chain_names
            target_frame,                      // 3. target_frame
            reference_frame,                   // 4. reference_frame
            ref_state,                         // 5. reference_robot_states (shared_ptr)
            false,                             // 6. enable_collision_check (bool)
            params                             // 7. params (shared_ptr)
        );
        print_ik_result("RobotStates逆解", res, planner);
    } catch (const std::exception& e) {
        std::cerr << "场景 5 异常: " << e.what() << std::endl;
    }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    std::cout << "资源已释放。" << std::endl;

    return 0;
}
