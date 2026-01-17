#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

int main() {
    // 获取对象实例
    auto& robot = GalbotRobot::get_instance();

    // 初始化系统
    if (robot.init()) {
        std::cout << "系统初始化成功！" << std::endl;
    } else {
        std::cerr << "系统初始化失败！" << std::endl;
        return -1;
    }

    // 程序立即启动，稍等数据就绪时间
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 获取指定关节名称，关节组名称包括["leg", "head", "left_arm", "right_arm"]
    std::vector<std::string> joint_groups = {"head"};
    bool only_active_joint = true;  // 获取可活动关节
    auto head_joint_names_vec =
        robot.get_joint_names(only_active_joint, joint_groups);
    std::cout << "Head joint names:" << std::endl;
    for (size_t i = 0; i < head_joint_names_vec.size(); ++i) {
        std::cout << i << ": " << head_joint_names_vec[i] << std::endl;
    }
    // 传入空数组默认获取所有关节组信息
    std::vector<std::string> null_vec = {};
    auto all_joint_names_vec =
        robot.get_joint_names(only_active_joint, null_vec);
    std::cout << "All joint names:" << std::endl;
    for (size_t i = 0; i < all_joint_names_vec.size(); ++i) {
        std::cout << i << ": " << all_joint_names_vec[i] << std::endl;
    }

    // 要控制的关节组，传入空数组默认控制腿、头、左臂、右臂关节
    joint_groups = {"head"};
    // 要控制的指定关节，如填充将覆盖joint_groups参数
    std::vector<std::string> joint_names = {};
    // 关节位置，头部关节组包含两个关节
    std::vector<double> joint_pos = {0.2, 0.2};
    // 是否阻塞等待关节角度执行到位或超时
    bool is_block = true;
    // 关节最大运行速度（rad/s）
    double speed_rad_s = 0.1;
    // 最大等待时间（秒）
    double timeout_s = 10.0;

    // 设置关节位置
    galbot::sdk::g1::ControlStatus joint_execution_status =
        robot.set_joint_positions(joint_pos, joint_groups,joint_names, 
            is_block, speed_rad_s,timeout_s);

    if (joint_execution_status == ControlStatus::SUCCESS) {
        std::cout << "关节命令设置成功！" << std::endl;
    } else {
        std::cerr << "关节命令设置失败！" << std::endl;
    }

    // 根据关节组查询关节位置，传入空数组默认填充腿、头、双臂关节组。第二个参数为指定关节名称，如填写将覆盖joint_groups参数。
    auto ret_positions = robot.get_joint_positions(joint_groups, {});
    for (auto position : ret_positions) {
        std::cout << "joint positions is " << position << std::endl;
    }

    // 使用特定关节名称进行控制，该参数将覆盖joint_groups关节组参数
    joint_names = {"head_joint1", "head_joint2"};
    joint_pos = {0.0, 0.0};

    // 设置关节位置
    joint_execution_status = robot.set_joint_positions(joint_pos, joint_groups,joint_names, 
            is_block, speed_rad_s,timeout_s);

    if (joint_execution_status == ControlStatus::SUCCESS) {
        std::cout << "关节命令设置成功！" << std::endl;
    } else {
        std::cerr << "关节命令设置失败！" << std::endl;
    }

    // 根据关节组查询关节位置，传入空数组默认填充腿、头、双臂关节组。第二个参数为指定关节名称，如填写将覆盖joint_groups参数。
    ret_positions = robot.get_joint_positions(joint_groups, {});
    for (auto position : ret_positions) {
        std::cout << "joint positions is " << position << std::endl;
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}