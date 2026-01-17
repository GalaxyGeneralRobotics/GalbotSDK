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

    // 激活吸盘
    if (robot.set_suction_cup_command(JointGroup::RIGHT_SUCTION_CUP, true) == ControlStatus::SUCCESS) {
        std::cout << "吸盘激活指令发送成功" << std::endl;
        
    } else {
        std::cerr << "吸盘激活指令发送失败！" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // 关闭吸盘
    if (robot.set_suction_cup_command(JointGroup::RIGHT_SUCTION_CUP, false) == ControlStatus::SUCCESS) {
        std::cout << "吸盘关闭指令发送成功" << std::endl;
        
    } else {
        std::cerr << "吸盘关闭指令发送失败" << std::endl;
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
