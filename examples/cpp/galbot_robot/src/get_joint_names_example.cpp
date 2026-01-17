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

    // 获取指定关节名称，关节组名称包括["leg", "head", "left_arm", "right_arm"]
    std::vector<std::string> joint_groups = {"head"};
    bool only_active_joint = true;  // 获取可活动关节
    auto head_joint_names_vec =
        robot.get_joint_names(only_active_joint, joint_groups);
    std::cout << "Head joint names:" << std::endl;
    for (size_t i = 0; i < head_joint_names_vec.size(); ++i) {
        std::cout << i << ": " << head_joint_names_vec[i] << std::endl;
    }

    // 传入空数组默认获取所有关节组信息
    std::vector<std::string> null_vec = {};
    auto all_joint_names_vec =
        robot.get_joint_names(only_active_joint, null_vec);
    std::cout << "All joint names:" << std::endl;
    for (size_t i = 0; i < all_joint_names_vec.size(); ++i) {
        std::cout << i << ": " << all_joint_names_vec[i] << std::endl;
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
