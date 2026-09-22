#include <iostream>
#include <vector>
#include <array>
#include <memory>
#include <thread>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::G3);
    
    // Initialize singleton instance
    if (robot.init()) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    // Check whether it is in the running state
    while (robot.is_running()) {
        // do something
        std::cout << "System is running." << std::endl;
        break;
    }

    // Register an exit callback (optional; automatically triggered when an exit signal is received)
    robot.register_exit_callback([]() {
        std::cout << "System is exiting..." << std::endl;
    });
    std::cout << "System exit callback registered successfully" << std::endl;

    // Send exit signal
    robot.request_shutdown();
    // Wait until entering shutdown state
    robot.wait_for_shutdown();
    // Release SDK related resources
    robot.destroy();
    std::cout << "Program finished" << std::endl;

    return 0;
}
