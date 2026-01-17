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

    // 请确认周边环境再进行底盘测试
    // 设置底盘速度，linear_velocity前两个字段为底盘x与y方向速度，angular_velocity第三个字段为z方向旋转速度
    // 注意，在下发停止命令前底盘将一直以此速度运动
    std::array<double, 3> linear_velocity = {0.1, 0.0, 0.0};    // 前进 0.1 m/s
    std::array<double, 3> angular_velocity = {0.0, 0.0, 0.1};   // 旋转 0.1 rad/s

    if (robot.set_base_velocity(linear_velocity, angular_velocity) == ControlStatus::SUCCESS) {
        std::cout << "底盘速度设置成功。" << std::endl;
        
    } else {
        std::cerr << "设置底盘速度失败。" << std::endl;
    }

    // 让底盘运动0.5秒(环境空旷可以让底盘运动3～5秒以观察效果)
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 停止底盘运动
    while (true) {
        ControlStatus status = robot.stop_base();
        if (status == ControlStatus::SUCCESS) {
            std::cout << "底盘已成功停止运动！" << std::endl;
            break;
        } else {
            std::cerr << "底盘停止运动失败，正在重试..." << std::endl;
        }
    }

    // TODO: 程序运行结束将底盘控制器切换回位置控制器

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
