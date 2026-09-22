#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

std::string force_sensor_type_to_string(GalbotOneFoxtrotSensor sensor_type) {
    switch (sensor_type) {
        case GalbotOneFoxtrotSensor::LEFT_WRIST_FORCE:
            return "LEFT_WRIST_FORCE";
        case GalbotOneFoxtrotSensor::RIGHT_WRIST_FORCE:
            return "RIGHT_WRIST_FORCE";
        default:
            return "UNKNOWN_FORCE_SENSOR";
    }
}

void print_force_data(GalbotOneFoxtrotSensor sensor_type, const std::shared_ptr<ForceData>& force_data) {
    std::cout << "--- " << force_sensor_type_to_string(sensor_type) << " ---" << std::endl;
    if (!force_data) {
        std::cerr << "  Force data is empty" << std::endl;
        std::cerr << "  Note: S1 force sensor data is supported only on hardware version 2.x; "
                  << "S1 1.x is not supported." << std::endl;
        return;
    }

    std::cout << "  Timestamp (ns): " << force_data->timestamp_ns << std::endl;
    std::cout << "  Force (N):  "
              << "fx=" << force_data->force.x << ", "
              << "fy=" << force_data->force.y << ", "
              << "fz=" << force_data->force.z << std::endl;
    std::cout << "  Torque (Nm): "
              << "tx=" << force_data->torque.x << ", "
              << "ty=" << force_data->torque.y << ", "
              << "tz=" << force_data->torque.z << std::endl;
}

int main() {
    auto& robot = GalbotRobot::get_instance(MachineType::S1);

    if (robot.init()) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    std::cout << "Note: S1 force sensor data is supported only on hardware version 2.x; "
              << "S1 1.x is not supported." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "\n===== Get left wrist force sensor data =====" << std::endl;
    auto left_force_data = robot.get_force_sensor_data(GalbotOneFoxtrotSensor::LEFT_WRIST_FORCE);
    print_force_data(GalbotOneFoxtrotSensor::LEFT_WRIST_FORCE, left_force_data);

    std::cout << "\n===== Get calibrated left wrist force data in base_link =====" << std::endl;
    auto calibrated_left_force_data =
        robot.get_force_sensor_data(GalbotOneFoxtrotSensor::LEFT_WRIST_FORCE, true, "base_link");
    print_force_data(GalbotOneFoxtrotSensor::LEFT_WRIST_FORCE, calibrated_left_force_data);

    std::cout << "\n===== Get right wrist force sensor data =====" << std::endl;
    auto right_force_data = robot.get_force_sensor_data(GalbotOneFoxtrotSensor::RIGHT_WRIST_FORCE);
    print_force_data(GalbotOneFoxtrotSensor::RIGHT_WRIST_FORCE, right_force_data);

    std::cout << "\n===== Get calibrated right wrist force data in mount frame =====" << std::endl;
    auto calibrated_right_force_data = robot.get_force_sensor_data(
        GalbotOneFoxtrotSensor::RIGHT_WRIST_FORCE, true, "right_arm_end_effector_mount_link");
    print_force_data(GalbotOneFoxtrotSensor::RIGHT_WRIST_FORCE, calibrated_right_force_data);

    std::cout << "\n===== Get all force sensor data =====" << std::endl;
    for (int i = 0; i < static_cast<int>(GalbotOneFoxtrotSensor::FORCE_NUM); ++i) {
        auto sensor_type = static_cast<GalbotOneFoxtrotSensor>(i);
        auto force_data = robot.get_force_sensor_data(sensor_type);
        print_force_data(sensor_type, force_data);
    }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
