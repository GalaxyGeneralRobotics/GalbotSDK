/**
 * example: navigation get_navigation_status, SUCCESS/FAILED timeout exit,
 * Avoid deadlock and execute error logic.
 */
#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace galbot::sdk;

int main() {
    auto& navigation = GalbotNavigation::get_instance(MachineType::S1);
    auto& robot = GalbotRobot::get_instance(MachineType::S1);

    if (!robot.init()) {
        std::cerr << "Base instance initialization failed!" << std::endl;
        return -1;
    }
    if (!navigation.init()) {
        std::cerr << "Navigation instance initialization failed!" << std::endl;
        return -1;
    }
    auto res = robot.switch_controller(S1ControllerName::SWERVE_CHASSIS_POSE_CTRL);
    if (res != ControlStatus::SUCCESS) {
        std::cerr << "Failed to switch controller!" << std::endl;
        return -1;
    }
    Pose goal_pose(std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    const double timeout_s = 20.0;
    const double poll_interval_s = 0.5;

    // Non-blocking navigation
    navigation.navigate_to_goal(goal_pose, true, false, static_cast<float>(timeout_s));
    auto start = std::chrono::steady_clock::now();

    while (true) {
        NavigationTaskStatus status = navigation.get_navigation_status();
        auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

        if (status == NavigationTaskStatus::SUCCESS) {
            std::cout << "Target reached" << std::endl;
            break;
        }
        if (status == NavigationTaskStatus::FAILED) {
            std::cout << "Navigation failed; exit error-handling logic promptly" << std::endl;
            break;
        }
        if (elapsed >= timeout_s) {
            std::cout << "navigationtimeout, exit" << std::endl;
            break;
        }

        if (status == NavigationTaskStatus::RUNNING) {
            std::cout << "Navigating... Status: RUNNING, elapsed: " << elapsed << "s" << std::endl;
        } else {
            std::cout << "Status: UNKNOWN, : " << elapsed << "s" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(poll_interval_s * 1000)));
    }

    // Stop navigation
    navigation.stop_navigation();

    // Get current pose again
    Pose current_pose = navigation.get_current_pose();
    std::cout << "Current pose: Position(" << current_pose.position.x << ", "
            << current_pose.position.y << ", " << current_pose.position.z
            << "), orientation(" << current_pose.orientation.x << ", "
            << current_pose.orientation.y << ", " << current_pose.orientation.z
                << ", " << current_pose.orientation.w << ")" << std::endl;

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    std::cout << "Resources released successfully" << std::endl;
    return 0;
}
