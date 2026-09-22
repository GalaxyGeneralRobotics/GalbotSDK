#include <iostream>
#include <chrono>
#include <thread>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    // Initialize system
    if (robot.init()) {
        std::cout << "Initialization succeeded" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Trigger software emergency stop
    ControlStatus status = robot.emergency_stop();
    if (status == ControlStatus::SUCCESS) {
        std::cout << "Emergency stop successfully." << std::endl;
    } else {
        std::cout << "Emergency stop failed." << std::endl;
    }

    std::cout << "Waiting 5 seconds before resuming from emergency stop..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Resume from software emergency stop
    status = robot.resume_from_emergency_stop();
    if (status == ControlStatus::SUCCESS) {
        std::cout << "Resume from emergency stop successfully." << std::endl;
    } else {
        std::cout << "Resume from emergency stop failed." << std::endl;
    }

    std::cout << "Waiting 15 seconds before exiting..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(15));

    // Send exit signal
    robot.request_shutdown();
    // Wait until entering shutdown state
    robot.wait_for_shutdown();
    // Release SDK resources
    robot.destroy();
    std::cout << "Resources released successfully" << std::endl;

    return 0;
}
