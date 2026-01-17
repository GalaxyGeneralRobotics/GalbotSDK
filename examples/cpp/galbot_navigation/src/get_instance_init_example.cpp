#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <thread>

using namespace galbot::sdk::g1;

int main() {
    auto& navigation = GalbotNavigation::get_instance();
    auto& robot = GalbotRobot::get_instance();

    // 初始化系统
    if (!robot.init()) {
        std::cerr << "Base instance 初始化失败！" << std::endl;
        return -1;
    }
    if (!navigation.init()) {
        std::cerr << "Navigation instance 初始化失败！" << std::endl;
        return -1;
    }

    std::cout << "初始化成功！" << std::endl;

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
