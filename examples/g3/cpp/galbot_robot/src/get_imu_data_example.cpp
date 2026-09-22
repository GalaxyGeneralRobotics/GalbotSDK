#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_imu_data(const std::shared_ptr<ImuData>& imu_data) {
    if (!imu_data) {
        std::cerr << "IMU data is empty" << std::endl;
        return;
    }

    std::cout << "Timestamp (ns): " << imu_data->timestamp_ns << std::endl;

    std::cout << "Accelerometer: "
              << "x=" << imu_data->accel.x << ", "
              << "y=" << imu_data->accel.y << ", "
              << "z=" << imu_data->accel.z << std::endl;

    std::cout << "Gyroscope: "
              << "x=" << imu_data->gyro.x << ", "
              << "y=" << imu_data->gyro.y << ", "
              << "z=" << imu_data->gyro.z << std::endl;

    std::cout << "Magnetometer: "
              << "x=" << imu_data->magnet.x << ", "
              << "y=" << imu_data->magnet.y << ", "
              << "z=" << imu_data->magnet.z << std::endl;
}

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    std::unordered_set<SensorType> sensor_types = {
        SensorType::CHASSIS_IMU
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

    // Get IMU data
    // - SensorType::LIDAR_IMU: lidar IMU
    // - SensorType::CHASSIS_IMU: Chassis lidar IMU
    std::shared_ptr<ImuData> imu_data = robot.get_imu_data(SensorType::CHASSIS_IMU);
    if (imu_data) {
        std::cout << "IMU data retrieved successfully!" << std::endl;
        print_imu_data(imu_data);
    } else {
        std::cerr << "Failed to get IMU data!" << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
