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
    if (robot.init()) {
        std::cout << "Base instance 初始化成功！" << std::endl;
    } else {
        std::cerr << "Base instance 初始化失败！" << std::endl;
        return -1;
    }
    if (navigation.init()) {
        std::cout << "Navigation instance 初始化成功！" << std::endl;
    } else {
        std::cerr << "Navigation instance 初始化失败！" << std::endl;
        return -1;
    }

    Pose init_pose(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    // 检查重定位是否成功
    int count_relocalize = 0;
    while (!navigation.is_localized() && count_relocalize < 20) {
        navigation.relocalize(init_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "is relocalizing" << std::endl;
        count_relocalize++;
    }

    if (navigation.is_localized()) {
        std::cout << "relocalization success." << std::endl;

        // 获取当前位姿
        Pose current_pose = navigation.get_current_pose();
        std::cout << "当前位姿: 位置(" << current_pose.position.x << ", "
                  << current_pose.position.y << ", " << current_pose.position.z
                  << "), 姿态(" << current_pose.orientation.x << ", "
                  << current_pose.orientation.y << ", " << current_pose.orientation.z
                  << ", " << current_pose.orientation.w << ")" << std::endl;

        robot.request_shutdown();
        robot.wait_for_shutdown();
    } else {
        std::cout << "relocalization failed, cannot proceed with navigation." << std::endl;
    }

    robot.destroy();

    return 0;
}
