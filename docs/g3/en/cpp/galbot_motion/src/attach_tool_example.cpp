#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <stdexcept>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

int main() {

    auto& planner = GalbotMotion::get_instance(MachineType::G3);
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    if (!planner.init()) {
        std::cerr << "GalbotMotion initialization failed" << std::endl;
        return -1;
    }
    if (!robot.init()) {
        std::cerr << "GalbotRobot initialization failed" << std::endl;
        return -1;
    }

    // Program started, waiting for data
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // --- Execute tool-attach operation ---
    try {
        std::string chain_name = "left_arm";
        std::string tool_name = "suction_cup";

        MotionStatus status = planner.attach_tool(chain_name, tool_name);

        std::cout << "Execution status feedback: " << planner.status_to_string(status) << std::endl;

        if (status == MotionStatus::SUCCESS) {
            std::cout << "✅ Tool attached successfully: " << tool_name << std::endl;

            // Clean up the attached tool model to avoid affecting subsequent examples.
            status = planner.detach_tool(chain_name);
            std::cout << "Cleanup status feedback: " << planner.status_to_string(status) << std::endl;

            if (status == MotionStatus::SUCCESS) {
                std::cout << "✅ Tool detached successfully." << std::endl;
            } else {
                std::cerr << "❌ Tool cleanup failed." << std::endl;
            }
        } else {
            std::cerr << "❌ Tool attachment failed. Please check whether the tool name is defined in the configuration file." << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "❌ Exception occurred during runtime: " << e.what() << std::endl;
    }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return 0;
}
