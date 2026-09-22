#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <thread>

using namespace galbot::sdk;

int main() {
    auto& navigation = GalbotNavigation::get_instance(MachineType::G3);
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    // Initialize system
    if (!robot.init()) {
        std::cerr << "Base instance initialization failed!" << std::endl;
        return -1;
    }
    if (!navigation.init()) {
        std::cerr << "Navigation instance initialization failed!" << std::endl;
        return -1;
    }

    auto res = robot.switch_controller(G3ControllerName::CHASSIS_POSE_CTRL);
    if (res != ControlStatus::SUCCESS) {
        std::cerr << "Failed to switch controller!" << std::endl;
        return -1;
    }
    Pose init_pose(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    Pose goal_pose(std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    // checkrelocalize success
    int count_relocalize = 0;
    while (!navigation.is_localized() && count_relocalize < 20) {
        navigation.relocalize(init_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "is relocalizing" << std::endl;
        count_relocalize++;
    }

    if (navigation.is_localized()) {
        std::cout << "Relocalization successful!" << std::endl;

        // Get current pose
        Pose current_pose = navigation.get_current_pose();
        std::cout << "Current pose: Position(" << current_pose.position.x << ", "
                << current_pose.position.y << ", " << current_pose.position.z
                << "), orientation(" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", " << current_pose.orientation.z
                << ", " << current_pose.orientation.w << ")" << std::endl;

        // Whether to enable obstacle checking (can be set to true in open environments)
        bool enable_collision_check = false;
        // Whether to block and wait for arrival
        bool is_blocking = true;
        // Maximum wait time to reach position
        float timeout_s = 20;

        // checkpath navigation target
        if (navigation.check_path_reachability(goal_pose, init_pose)) {
            std::cout << "Path reachable, navigating to target point" << std::endl;
            // navigation, obstaclecheck, wait, wait 20
            NavigationStatus status = navigation.navigate_to_goal(
                goal_pose, enable_collision_check, is_blocking, timeout_s);
            if (status == NavigationStatus::SUCCESS) {
                std::cout << "Target point reached" << std::endl;
            } else {
                std::cout << "navigationfailed, status: " << static_cast<int>(status) << std::endl;
            }
        } else {
            std::cout << "Path unreachable, cannot navigate to target point" << std::endl;
        }

        // Check path reachability and return to start point
        if (navigation.check_path_reachability(init_pose, goal_pose)) {
            std::cout << "Path reachable, navigating to start point" << std::endl;
            // Navigate to the target waypoint with obstacle checking disabled and non-blocking wait for arrival
            is_blocking = false;
            NavigationStatus status = navigation.navigate_to_goal(
                init_pose, enable_collision_check, is_blocking);
            // wait
            int count_arrival = 0;
            while (!navigation.check_goal_arrival()) {
                std::cout << "navigate has not arrived" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                if (++count_arrival > 10) {
                    break;
                }
            }
            if (navigation.check_goal_arrival()) {
                std::cout << "Target point reached" << std::endl;
            } else {
                std::cout << "Navigation failed; target point not reached" << std::endl;
            }
        } else {
            std::cout << "Path unreachable, cannot navigate to start point" << std::endl;
        }

        // checkpath navigation target
        if (navigation.check_path_reachability(goal_pose, init_pose)) {
            std::cout << "Path reachable, navigating to target point" << std::endl;
            // target, wait, wait 10
            is_blocking = true;
            NavigationStatus status = navigation.move_straight_to(
                goal_pose, is_blocking, timeout_s);

            if (status == NavigationStatus::SUCCESS) {
                std::cout << "Target point reached" << std::endl;
            } else {
                std::cout << "navigationfailed, status: " << static_cast<int>(status) << std::endl;
            }
        } else {
            std::cout << "Path unreachable, cannot navigate to target point" << std::endl;
        }

        // Stop navigation
        navigation.stop_navigation();

    } else {
        std::cout << "Relocalization failed!" << std::endl;
    }


    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
