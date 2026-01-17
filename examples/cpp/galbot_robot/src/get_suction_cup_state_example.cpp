#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

void print_suction_cup_state(
    std::shared_ptr<galbot::sdk::g1::SuctionCupState> suction_cup_state) {
    std::cout << "Timestamp (ns): " << suction_cup_state->timestamp_ns << std::endl;
    std::cout << "Activation: " << suction_cup_state->activation << std::endl;
    std::cout << "Pressure: " << suction_cup_state->pressure << " Pa" << std::endl;
    std::cout << "Action State: " << int(suction_cup_state->action_state) << std::endl;
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

    // 获取吸盘状态
    auto suction_cup_state_ptr = robot.get_suction_cup_state(JointGroup::RIGHT_SUCTION_CUP);

    if (suction_cup_state_ptr == nullptr) {
        std::cerr << "get suction cup state error" << std::endl;
    } else {
        std::cout << "右吸盘状态：" << std::endl;
        print_suction_cup_state(suction_cup_state_ptr);
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
