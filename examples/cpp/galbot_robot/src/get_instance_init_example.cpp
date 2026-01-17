#include <iostream>
#include <vector>
#include <array>
#include <memory>
#include <thread>

#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

int main() {
    // 获取对象实例
    auto& robot = GalbotRobot::get_instance();
    
    // 初始化单例对象
    if (robot.init()) {
        std::cout << "系统初始化成功！" << std::endl;
    } else {
        std::cerr << "系统初始化失败！" << std::endl;
        return -1;
    }

    // 判断是否处于运行状态
    while (robot.is_running()) {
        // do something
        std::cout << "系统正在运行。" << std::endl;
        break;
    }

    // 注册退出回调(可选项，收到退出信号时会自动触发)
    robot.register_exit_callback([]() {
        std::cout << "系统正在退出..." << std::endl;
    });
    std::cout << "成功注册系统退出回调" << std::endl;

    // 发出退出信号
    robot.request_shutdown();
    // 等待进入shutdown状态
    robot.wait_for_shutdown();
    // 释放SDK相关资源
    robot.destroy();
    std::cout << "程序结束" << std::endl;

    return 0;
}
