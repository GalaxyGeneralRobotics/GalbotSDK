#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_odom_data(const std::shared_ptr<OdomData>& odom_data) {
    if (!odom_data) {
        std::cerr << "Odom data is empty" << std::endl;
        return;
    }

    std::cout << "Timestamp (ns): " << odom_data->timestamp_ns << std::endl;

    std::cout << "Position (m): "
              << "x=" << odom_data->position[0] << ", "
              << "y=" << odom_data->position[1] << ", "
              << "z=" << odom_data->position[2] << std::endl;

    std::cout << "Orientation (quaternion): "
              << "qx=" << odom_data->orientation[0] << ", "
              << "qy=" << odom_data->orientation[1] << ", "
              << "qz=" << odom_data->orientation[2] << ", "
              << "qw=" << odom_data->orientation[3] << std::endl;

    std::cout << "Linear velocity (m/s): "
              << "vx=" << odom_data->linear_velocity[0] << ", "
              << "vy=" << odom_data->linear_velocity[1] << ", "
              << "vz=" << odom_data->linear_velocity[2] << std::endl;

    std::cout << "Angular velocity (rad/s): "
              << "wx=" << odom_data->angular_velocity[0] << ", "
              << "wy=" << odom_data->angular_velocity[1] << ", "
              << "wz=" << odom_data->angular_velocity[2] << std::endl;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Get odometry data
    std::shared_ptr<OdomData> odom_data = robot.get_odom();
    if (odom_data) {
        std::cout << "Odometry data retrieved successfully!" << std::endl;
        print_odom_data(odom_data);
    } else {
        std::cerr << "Failed to get odometry data!" << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
