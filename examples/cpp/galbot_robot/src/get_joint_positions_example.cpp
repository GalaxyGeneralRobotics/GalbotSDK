#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

void print_joint_positions(const std::vector<double>& positions) {
    for (const auto& pos : positions) {
        std::cout << "joint positions is " << pos << std::endl;
    }
    std::cout << std::endl;
}

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

    // 使用关节组名称获取关节位置，为空默认返回所有关节
    std::vector<std::string> joint_groups = {"left_arm"};
    auto ret_positions = robot.get_joint_positions(joint_groups, {});
    std::cout << "左臂关节位置：" << std::endl;
    print_joint_positions(ret_positions);

    // 获取指定关节位置，如果填充将覆盖关节组输入
    std::vector<std::string> joint_names = {"left_arm_joint1", "left_arm_joint2"};
    ret_positions = robot.get_joint_positions({joint_groups}, joint_names);
    std::cout << "左臂关节1及关节2位置：" << std::endl;
    print_joint_positions(ret_positions);

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
