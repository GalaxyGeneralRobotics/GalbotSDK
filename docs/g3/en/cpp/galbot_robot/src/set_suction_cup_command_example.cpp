#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    // Initialize system
    if (robot.init()) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    // Program started, waiting for data
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Activate suction cup
    if (robot.set_suction_cup_command("right_suction_cup", true) == ControlStatus::SUCCESS) {
        std::cout << "Suction cup activation command sent successfully" << std::endl;
        
    } else {
        std::cerr << "Suction cup activation command failed to send!" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // Deactivate suction cup
    if (robot.set_suction_cup_command("right_suction_cup", false) == ControlStatus::SUCCESS) {
        std::cout << "Suction cup deactivation command sent successfully" << std::endl;
        
    } else {
        std::cerr << "Suction cup deactivation command failed to send" << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
