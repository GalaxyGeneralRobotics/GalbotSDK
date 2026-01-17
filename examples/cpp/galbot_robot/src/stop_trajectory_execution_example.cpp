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

    // 发送停止轨迹执行指令
    while(true) {
        galbot::sdk::g1::ControlStatus joint_execution_status =
            robot.stop_trajectory_execution();
        
        // 检查执行结果
        if (joint_execution_status == ControlStatus::SUCCESS) {
            std::cout << "停止轨迹执行指令发送成功" << std::endl;
            break;
        } else {
            std::cerr << "停止轨迹执行指令发送失败，重试中..." << std::endl;
        }
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
