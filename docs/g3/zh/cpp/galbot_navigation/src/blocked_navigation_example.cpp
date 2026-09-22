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
    if (robot.init()) {
        std::cout << "Base instance initialized successfully!" << std::endl;
    } else {
        std::cerr << "Base instance initialization failed!" << std::endl;
        return -1;
    }
    if (navigation.init()) {
        std::cout << "Navigation instance initialized successfully!" << std::endl;
    } else {
        std::cerr << "Navigation instance initialization failed!" << std::endl;
        return -1;
    }
    auto res = robot.switch_controller(G3ControllerName::CHASSIS_POSE_CTRL);
    if (res != ControlStatus::SUCCESS) {
        std::cerr << "Failed to switch controller!" << std::endl;
        return -1;
    }
    Pose init_pose(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    Pose goal_pose(std::vector<double>{0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

    // checkrelocalize success
    int count_relocalize = 0;
    while (!navigation.is_localized() && count_relocalize < 20) {
        navigation.relocalize(init_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "is relocalizing" << std::endl;
        count_relocalize++;
    }

    if (navigation.is_localized()) {
        std::cout << "relocalization success." << std::endl;

        // Get current pose
        Pose current_pose = navigation.get_current_pose();
        std::cout << "Current pose: Position(" << current_pose.position.x << ", "
                << current_pose.position.y << ", " << current_pose.position.z
                << "), orientation(" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", " << current_pose.orientation.z
                << ", " << current_pose.orientation.w << ")" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Whether to enable obstacle checking (can be set to true in open environments)
        bool enable_collision_check = false;
        // Whether to block and wait for arrival
        bool is_blocking = true;
        // Maximum wait time to reach position
        float timeout_s = 20;

        // Navigate 3 times in loop
        int count = 0;
        while (count++ < 3) {
            std::cout << "No. " << count << " navigation(s)" << std::endl;
            // checkpath navigation target
            if (navigation.check_path_reachability(goal_pose, init_pose)) {
                std::cout << "Path reachable, navigating to target point" << std::endl;
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
                NavigationStatus status = navigation.navigate_to_goal(
                    init_pose, enable_collision_check, is_blocking, timeout_s);
                if (status == NavigationStatus::SUCCESS) {
                    std::cout << "Returned to start point" << std::endl;
                } else {
                    std::cout << "navigationfailed, status: " << static_cast<int>(status) << std::endl;
                }
            } else {
                std::cout << "Path unreachable, cannot navigate to start point" << std::endl;
            }
        }

        // Stop navigation
        navigation.stop_navigation();

        // Get current pose again
        current_pose = navigation.get_current_pose();
        std::cout << "Current pose: Position(" << current_pose.position.x << ", "
                << current_pose.position.y << ", " << current_pose.position.z
                << "), orientation(" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", " << current_pose.orientation.z
                << ", " << current_pose.orientation.w << ")" << std::endl;

    } else {
        std::cout << "relocalization failed." << std::endl;
    }


    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
