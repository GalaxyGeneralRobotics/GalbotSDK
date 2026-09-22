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

    // Stop chassis motion
    while (true) {
        ControlStatus status = robot.stop_base();
        if (status == ControlStatus::SUCCESS) {
            std::cout << "Chassis motion has been stopped successfully!" << std::endl;
            break;
        } else {
            std::cerr << "Chassis stop motion failed, retrying..." << std::endl;
        }
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
