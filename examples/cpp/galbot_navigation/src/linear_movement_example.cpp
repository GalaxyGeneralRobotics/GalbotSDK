#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace galbot::sdk::g1;

// 辅助函数: 获取当前时间字符串 [HH:MM:SS.ms]
std::string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    std::stringstream ss;
    ss << "[" << std::put_time(&tm, "%H:%M:%S") << "." 
       << std::setfill('0') << std::setw(3) << ms.count() << "] ";
    return ss.str();
}

int main() {
    auto& navigation = GalbotNavigation::get_instance();
    auto& robot = GalbotRobot::get_instance();

    // 初始化系统
    std::cout << get_timestamp() << "开始调用 robot.init()..." << std::endl;
    if (robot.init()) {
        std::cout << get_timestamp() << "Base instance 初始化成功！" << std::endl;
    } else {
        std::cerr << get_timestamp() << "Base instance 初始化失败！" << std::endl;
        return -1;
    }

    std::cout << get_timestamp() << "开始调用 navigation.init()..." << std::endl;
    if (navigation.init()) {
        std::cout << get_timestamp() << "Navigation instance 初始化成功！" << std::endl;
    } else {
        std::cerr << get_timestamp() << "Navigation instance 初始化失败！" << std::endl;
        return -1;
    }

    Pose init_pose(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    Pose goal_pose(std::vector<double>{0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    // 检查重定位是否成功
    std::cout << get_timestamp() << "进入重定位检查循环..." << std::endl;
    int count_relocalize = 0;
    while (!navigation.is_localized() && count_relocalize < 20) {
        std::cout << get_timestamp() << "调用 navigation.relocalize()..." << std::endl;
        navigation.relocalize(init_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << get_timestamp() << "is relocalizing" << std::endl;
        count_relocalize++;
    }

    if (navigation.is_localized()) {
        std::cout << get_timestamp() << "relocalization success." << std::endl;

        // 获取当前位姿
        std::cout << get_timestamp() << "调用 navigation.get_current_pose()..." << std::endl;
        Pose current_pose = navigation.get_current_pose();
        std::cout << get_timestamp() << "当前位姿: 位置(" << current_pose.position.x << ", "
                << current_pose.position.y << ", " << current_pose.position.z
                << "), 姿态(" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", " << current_pose.orientation.z
                << ", " << current_pose.orientation.w << ")" << std::endl;

        // 是否阻塞等待到达
        bool is_blocking = true;
        // 最大等待到位时间
        float timeout_s = 20;

        // --- 直线移动并计时 ---
        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << get_timestamp() << "准备执行 move_straight_to (直线移动)..." << std::endl;
        
        // 记录开始时间
        auto start_move = std::chrono::system_clock::now();
        
        // 执行移动
        NavigationStatus status = navigation.move_straight_to(goal_pose, is_blocking, timeout_s);
        
        // 记录结束时间并计算耗时
        auto end_move = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_move - start_move).count();

        if (status == NavigationStatus::SUCCESS) {
            std::cout << get_timestamp() << "已到达目标点位 (耗时: " << duration << "ms)" << std::endl;
        } else {
            std::cout << get_timestamp() << "导航失败，状态码: " << static_cast<int>(status) 
                    << " (耗时: " << duration << "ms)" << std::endl;
        }
        std::cout << "--------------------------------------------------" << std::endl;

        // 停止导航
        std::cout << get_timestamp() << "调用 navigation.stop_navigation()..." << std::endl;
        navigation.stop_navigation();

        // 再次获取当前位姿
        std::cout << get_timestamp() << "调用 navigation.get_current_pose()..." << std::endl;
        current_pose = navigation.get_current_pose();
        std::cout << get_timestamp() << "当前位姿: 位置(" << current_pose.position.x << ", "
                << current_pose.position.y << ", " << current_pose.position.z
                << "), 姿态(" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", " << current_pose.orientation.z
                << ", " << current_pose.orientation.w << ")" << std::endl;
    } else {
        std::cout << get_timestamp() << "重定位失败，无法继续执行导航。" << std::endl;
    }
    
    std::cout << get_timestamp() << "执行 shutdown..." << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    std::cout << get_timestamp() << "程序结束。" << std::endl;

    return 0;
}
