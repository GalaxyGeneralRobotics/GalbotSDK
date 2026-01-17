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

    // 夹爪宽度（米）
    double width_m = 0.1;
    // 夹爪速度（米/秒）
    double velocity_mps = 0.05;
    // 夹爪力矩（牛米）
    double effort = 10;
    // 是否阻塞等待执行完成
    bool is_blocking = true;
    // 设置左夹爪宽度为0.1米，以0.05米速度执行，力矩为10，并阻塞等待执行完成
    galbot::sdk::g1::ControlStatus gripper_execution_status =
        robot.set_gripper_command(JointGroup::LEFT_GRIPPER, width_m, velocity_mps,
                                        effort, is_blocking);

    if (gripper_execution_status == ControlStatus::SUCCESS) {
        std::cout << "夹爪命令设置成功！" << std::endl;
    } else {
        std::cerr << "夹爪命令设置失败！" << std::endl;
    }

    // 获取夹爪状态
    galbot::sdk::g1::JointStateMessage joint_state;
    auto gripper_state_ptr = robot.get_gripper_state(JointGroup::LEFT_GRIPPER);

    if (gripper_state_ptr == nullptr) {
        std::cerr << "get gripper state error" << std::endl;
    } else {
        print_gripper_state(gripper_state_ptr);
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
