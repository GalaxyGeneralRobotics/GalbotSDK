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

    Pose init_pose(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

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

        robot.request_shutdown();
        robot.wait_for_shutdown();
    } else {
        std::cout << "relocalization failed, cannot proceed with navigation." << std::endl;
    }

    robot.destroy();

    return 0;
}
