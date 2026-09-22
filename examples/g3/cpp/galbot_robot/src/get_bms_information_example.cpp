#include <iostream>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_bms_information(const std::shared_ptr<BmsInfo>& bms_info) {
    if (!bms_info) {
        std::cerr << "BMS info is empty" << std::endl;
        return;
    }

    std::cout << "Voltage (V): " << bms_info->voltage << std::endl;
    std::cout << "Current (A): " << bms_info->current << std::endl;
    std::cout << "Battery level (%): " << bms_info->battery_level << std::endl;
    std::cout << "Temperature (C): " << bms_info->temperature << std::endl;
    std::cout << "Charging status: " << std::boolalpha << bms_info->charging_status
              << std::noboolalpha << std::endl;
    std::cout << "Health status: " << std::boolalpha << bms_info->health_status
              << std::noboolalpha << std::endl;
    std::cout << "Capacity (Ah): " << bms_info->capacity << std::endl;
}

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
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // Get BMS information
    auto bms_info = robot.get_bms_information();
    if (bms_info) {
        std::cout << "BMS info retrieved successfully!" << std::endl;
        print_bms_information(bms_info);
    } else {
        std::cerr << "Failed to get BMS info!" << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
