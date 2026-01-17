#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <stdexcept>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

int main() {

    auto& planner = GalbotMotion::get_instance();
    auto& robot = GalbotRobot::get_instance();

    if (!planner.init()) {
        std::cerr << "GalbotMotion 初始化失败" << std::endl;
        return -1;
    }
    if (!robot.init()) {
        std::cerr << "GalbotRobot 初始化失败" << std::endl;
        return -1;
    }

    // 程序立即启动，稍等数据就绪时间
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // --- 执行附加工具操作 ---
    try {
        std::string chain_name = "left_arm";
        std::string tool_name = "suction_cup";

        MotionStatus status = planner.attach_tool(chain_name, tool_name);

        std::cout << "执行状态反馈: " << planner.status_to_string(status) << std::endl;

        if (status == MotionStatus::SUCCESS) {
            std::cout << "✅ 成功加载工具: " << tool_name << std::endl;
        } else {
            std::cerr << "❌ 加载工具失败，请检查工具名称是否在配置文件中定义。" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "❌ 运行过程中发生异常: " << e.what() << std::endl;
    }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return 0;
}
