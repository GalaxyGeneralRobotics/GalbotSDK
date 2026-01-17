#include <iostream>
#include <set>
#include <string>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

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

    // 仍然可以通过 GalbotRobot 管理机器人生命周期
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
