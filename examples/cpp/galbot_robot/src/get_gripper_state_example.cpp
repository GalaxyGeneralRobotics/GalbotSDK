#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

void print_gripper_state(
    std::shared_ptr<galbot::sdk::g1::GripperState> gripper_state) {
    std::cout << "Timestamp (ns): " << gripper_state->timestamp_ns << std::endl;

    std::cout << " width "  << gripper_state->width << " velocity " << gripper_state->velocity
                << " effort " << gripper_state->effort << " is moving "
                << gripper_state->is_moving << std::endl;
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

    // 获取夹爪状态
    auto gripper_state_ptr = robot.get_gripper_state(JointGroup::LEFT_GRIPPER);

    if (gripper_state_ptr == nullptr) {
        std::cerr << "get gripper state error" << std::endl;
    } else {
        std::cout << "左夹爪状态：" << std::endl;
        print_gripper_state(gripper_state_ptr);
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
