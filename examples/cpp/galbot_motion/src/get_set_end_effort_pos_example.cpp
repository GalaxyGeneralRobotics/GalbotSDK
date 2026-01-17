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

// 辅助函数：打印位姿信息
void print_pose_info(const std::string& label, const std::vector<double>& pose) {
    if (pose.size() == 7) {
        std::cout << "[" << label << "] 位姿: "
                  << "pos(" << pose[0] << ", " << pose[1] << ", " << pose[2] << "), "
                  << "ori(" << pose[3] << ", " << pose[4] << ", " << pose[5] << ", " << pose[6] << ")" 
                  << std::endl;
    }
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

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::unordered_map<std::string, std::vector<double>> chain_pose_baselink = {
        {"leg",       {0.0596, -0.0000, 1.0327, 0.5000, 0.5003, 0.4997, 0.5000}},
        {"head",      {0.0599, 0.0002, 1.4098, -0.7072, 0.0037, 0.0037, 0.7069}},
        {"left_arm",  {0.1267, 0.2342, 0.7356, 0.0220, 0.0127, 0.0343, 0.9991}},
        {"right_arm", {0.097768, -0.226021, 0.8, -0.0117403, -0.0098713, 0.0157502, 0.999758}}
    };

    std::string reference_frame = "base_link";
    std::string target_frame = "EndEffector";
    std::string target_chain = "right_arm";
    auto custom_param = std::make_shared<Parameter>();

    // --- 场景 1：基础版获取末端位姿 ---
    try {
        std::cout << ">> 场景 1：正在获取基础末端位姿..." << std::endl;
        std::string end_ee_link = "right_arm_end_effector_mount_link";

        auto res = planner.get_end_effector_pose(end_ee_link, reference_frame);
        
        MotionStatus status = std::get<0>(res);
        std::vector<double> pose = std::get<1>(res);

        std::cout << "执行状态: " << planner.status_to_string(status) << std::endl;
        if (status == MotionStatus::SUCCESS) {
            print_pose_info("基础版", pose);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) {
        std::cerr << "❌ 场景 1 异常: " << e.what() << std::endl;
    }

    // --- 场景 2：指定链名 + 自定义frame获取末端位姿 ---
    try {
        std::cout << ">> 场景 2：正在指定链名获取位姿..." << std::endl;

        auto res = planner.get_end_effector_pose_on_chain(target_chain, target_frame, reference_frame);
        
        MotionStatus status = std::get<0>(res);
        std::vector<double> pose = std::get<1>(res);

        std::cout << "执行状态: " << planner.status_to_string(status) << std::endl;
        if (status == MotionStatus::SUCCESS) {
            print_pose_info("指定链名版", pose);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) {
        std::cerr << "❌ 场景 2 异常: " << e.what() << std::endl;
    }

    // --- 场景 3：设置末端位姿 ---
    try {
        std::cout << ">> 场景 3：正在设置末端位姿..." << std::endl;
        
        std::string ee_frame = "right_arm"; 
        std::vector<double> target_pose = chain_pose_baselink[ee_frame];

        MotionStatus status = planner.set_end_effector_pose(
            target_pose,        // 1
            ee_frame,           // 2
            reference_frame,    // 3
            nullptr,            // 4. 重要：如果不参考特定状态，传入 nullptr
            false,              // 5. enable_collision_check
            true,               // 6. is_blocking
            5.0,                // 7. timeout
            custom_param        // 8. params
        );

        std::cout << "设置状态: " << planner.status_to_string(status) << std::endl;
        if (status == MotionStatus::SUCCESS) {
            std::cout << "✅ 指令发送成功 (阻塞等待模式)" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ 设置位姿异常: " << e.what() << std::endl;
    }

    // --- 场景 4：执行结束再次获取末端位姿 ---
    try {
        std::cout << ">> 场景 4：正在获取基础末端位姿..." << std::endl;
        std::string end_ee_link = "right_arm_end_effector_mount_link";

        auto res = planner.get_end_effector_pose(end_ee_link, reference_frame);
        
        MotionStatus status = std::get<0>(res);
        std::vector<double> pose = std::get<1>(res);

        std::cout << "执行状态: " << planner.status_to_string(status) << std::endl;
        if (status == MotionStatus::SUCCESS) {
            print_pose_info("基础版", pose);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    } catch (const std::exception& e) {
        std::cerr << "❌ 场景 4 异常: " << e.what() << std::endl;
    }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
