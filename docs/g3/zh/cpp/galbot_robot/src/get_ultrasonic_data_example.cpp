#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <unordered_set>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

std::string ultrasonic_type_to_string(UltrasonicType ultrasonic_type) {
    switch (ultrasonic_type) {
        case UltrasonicType::FRONT_LEFT:
            return "FRONT_LEFT";
        case UltrasonicType::FRONT_RIGHT:
            return "FRONT_RIGHT";
        case UltrasonicType::RIGHT_LEFT:
            return "RIGHT_LEFT";
        case UltrasonicType::RIGHT_RIGHT:
            return "RIGHT_RIGHT";
        case UltrasonicType::BACK_LEFT:
            return "BACK_LEFT";
        case UltrasonicType::BACK_RIGHT:
            return "BACK_RIGHT";
        case UltrasonicType::LEFT_LEFT:
            return "LEFT_LEFT";
        case UltrasonicType::LEFT_RIGHT:
            return "LEFT_RIGHT";
        default:
            return "UNKNOWN_ULTRASONIC";
    }
}

void print_ultrasonic_data(UltrasonicType ultrasonic_type, const std::shared_ptr<UltrasonicData>& ultrasonic_data) {
    std::cout << "--- " << ultrasonic_type_to_string(ultrasonic_type) << " ---" << std::endl;
    if (!ultrasonic_data) {
        std::cerr << "  Ultrasonic data is empty" << std::endl;
        return;
    }

    std::cout << "  Timestamp (ns): " << ultrasonic_data->timestamp_ns << std::endl;
    std::cout << "  Distance (m): " << ultrasonic_data->distance << std::endl;
}

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    // Initialize sensors; only sensors passed during initialization can retrieve data to save resources
    std::unordered_set<SensorType> sensor_types = {
        SensorType::BASE_ULTRASONIC  // Chassis ultrasonic sensor
    };

    // Initialize system
    if (robot.init(sensor_types)) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    // Program started, waiting for data
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Get single ultrasonic sensor data
    std::cout << "\n===== Get single ultrasonic sensor data =====" << std::endl;
    auto ultrasonic_data = robot.get_ultrasonic_data(UltrasonicType::FRONT_LEFT);
    print_ultrasonic_data(UltrasonicType::FRONT_LEFT, ultrasonic_data);

    // Iterate to get all 8 ultrasonic sensor data
    std::cout << "\n===== Get all ultrasonic sensor data =====" << std::endl;
    for (int i = 0; i < static_cast<int>(UltrasonicType::ULTRASONIC_NUM); ++i) {
        auto ultrasonic_type = static_cast<UltrasonicType>(i);
        auto data = robot.get_ultrasonic_data(ultrasonic_type);
        print_ultrasonic_data(ultrasonic_type, data);
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
