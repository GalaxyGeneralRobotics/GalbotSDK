#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_base_velocity(const std::shared_ptr<BaseVelocityInfo>& base_velocity_info) {
    if (!base_velocity_info) {
        std::cerr << "Base velocity data is empty" << std::endl;
        return;
    }

    // Print linear and angular velocity in the same format as the standalone example.
    const auto& linear = base_velocity_info->linear_velocity;
    std::cout << "Linear velocity (m/s): "
              << "vx=" << linear[0] << ", vy=" << linear[1] << ", vz=" << linear[2] << std::endl;

    const auto& angular = base_velocity_info->angular_velocity;
    std::cout << "Angular velocity (rad/s): "
              << "wx=" << angular[0] << ", wy=" << angular[1] << ", wz=" << angular[2] << std::endl;
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

    // Read base velocity before issuing the motion command.
    auto base_velocity_before = robot.get_base_velocity();
    if (base_velocity_before) {
        std::cout << "Base velocity before command:" << std::endl;
        print_base_velocity(base_velocity_before);
    } else {
        std::cerr << "Failed to get base velocity before command." << std::endl;
    }

    // Please confirm the surrounding environment before chassis testing
    // Set chassis speed, linear_velocity first two fields are x and y velocities, angular_velocity third field is z rotation speed
    std::array<double, 3> linear_velocity = {0.2, 0.0, 0.0};    // 0.2 m/s
    std::array<double, 3> angular_velocity = {0.0, 0.0, 0.0};    // 0.0 rad/s
    double duration_s = 3.0;  // Block while publishing at 10 Hz; no stop is sent on expiry.

    if (robot.set_base_velocity(linear_velocity, angular_velocity, duration_s) == ControlStatus::SUCCESS) {
        std::cout << "Velocity publishing completed after " << duration_s
                  << " seconds; stopping depends on the watchdog." << std::endl;
    } else {
        std::cerr << "Set chassis speed failed." << std::endl;
    }

    // Observe immediately: publishing completion does not imply a stopped base.
    auto base_velocity_after = robot.get_base_velocity();
    if (base_velocity_after) {
        std::cout << "Base velocity after command:" << std::endl;
        print_base_velocity(base_velocity_after);
    } else {
        std::cerr << "Failed to get base velocity after command." << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
