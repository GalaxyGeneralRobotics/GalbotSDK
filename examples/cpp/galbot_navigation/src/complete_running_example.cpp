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

    Pose init_pose(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    Pose goal_pose(std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    // 检查重定位是否成功
    int count_relocalize = 0;
    while (!navigation.is_localized() && count_relocalize < 20) {
        navigation.relocalize(init_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "is relocalizing" << std::endl;
        count_relocalize++;
    }

    if (navigation.is_localized()) {
        std::cout << "重定位成功！" << std::endl;

        // 获取当前位姿
        Pose current_pose = navigation.get_current_pose();
        std::cout << "当前位姿: 位置(" << current_pose.position.x << ", "
                << current_pose.position.y << ", " << current_pose.position.z
                << "), 姿态(" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", " << current_pose.orientation.z
                << ", " << current_pose.orientation.w << ")" << std::endl;

        // 是否开启障碍物检查（环境空旷可设置为true）
        bool enable_collision_check = false;
        // 是否阻塞等待到达
        bool is_blocking = true;
        // 最大等待到位时间
        float timeout_s = 20;

        // 检查路径是否可达并导航到目标
        if (navigation.check_path_reachability(goal_pose, init_pose)) {
            std::cout << "路径可达，导航至目标点位" << std::endl;
            // 导航到指定点位，关闭障碍物检查，阻塞等待到达，最大等待时间为20秒
            NavigationStatus status = navigation.navigate_to_goal(
                goal_pose, enable_collision_check, is_blocking, timeout_s);
            if (status == NavigationStatus::SUCCESS) {
                std::cout << "已到达目标点位" << std::endl;
            } else {
                std::cout << "导航失败，状态码: " << static_cast<int>(status) << std::endl;
            }
        } else {
            std::cout << "路径不可达，无法导航至目标点位" << std::endl;
        }

        // 检查路径是否可达并回到起点
        if (navigation.check_path_reachability(init_pose, goal_pose)) {
            std::cout << "路径可达，导航至起点" << std::endl;
            // 导航到指定点位，关闭障碍物检查，非阻塞等待到达
            is_blocking = false;
            NavigationStatus status = navigation.navigate_to_goal(
                init_pose, enable_collision_check, is_blocking);
            // 等待到达
            int count_arrival = 0;
            while (!navigation.check_goal_arrival()) {
                std::cout << "navigate has not arrived" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                if (++count_arrival > 10) {
                    break;
                }
            }
            if (navigation.check_goal_arrival()) {
                std::cout << "已到达目标点位" << std::endl;
            } else {
                std::cout << "导航失败，未到达目标点位" << std::endl;
            }
        } else {
            std::cout << "路径不可达，无法导航至起点" << std::endl;
        }

        // 检查路径是否可达并导航到目标
        if (navigation.check_path_reachability(goal_pose, init_pose)) {
            std::cout << "路径可达，导航至目标点位" << std::endl;
            // 直线移动到目标点位,阻塞等待到达，最大等待时间为10秒
            is_blocking = true;
            NavigationStatus status = navigation.move_straight_to(
                goal_pose, is_blocking, timeout_s);

            if (status == NavigationStatus::SUCCESS) {
                std::cout << "已到达目标点位" << std::endl;
            } else {
                std::cout << "导航失败，状态码: " << static_cast<int>(status) << std::endl;
            }
        } else {
            std::cout << "路径不可达，无法导航至目标点位" << std::endl;
        }

        // 停止导航
        navigation.stop_navigation();

    } else {
        std::cout << "重定位失败！" << std::endl;
    }


    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
