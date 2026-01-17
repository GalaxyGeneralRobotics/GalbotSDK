#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

void print_pose_vec(const std::vector<double> &pose_vec) {
    // 输出 pose_vec
    std::cout << "pose_vec = [";
    for (size_t i = 0; i < pose_vec.size(); ++i) {
        std::cout << pose_vec[i];
        if (i + 1 < pose_vec.size())
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
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

    // 获取坐标变换
    std::pair<std::vector<double>, int64_t> tf_ret = robot.get_transform("left_arm_link1", "left_arm_link7", 0);

    if (tf_ret.first.empty()) {
        std::cout << "get_transform error" << std::endl;
    } else {
        std::cout << "tf_timestamp_ns: " << tf_ret.second << std::endl;
        print_pose_vec(tf_ret.first);
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
