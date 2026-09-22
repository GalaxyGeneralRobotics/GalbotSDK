#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace galbot::sdk;

// Helper function: get current time string [HH:MM:SS.ms]
std::string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);

    std::stringstream ss;
    ss << "[" << std::put_time(&tm, "%H:%M:%S") << "." 
       << std::setfill('0') << std::setw(3) << ms.count() << "] ";
    return ss.str();
}

int main() {
    auto& navigation = GalbotNavigation::get_instance(MachineType::G3);
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    // Initialize system
    std::cout << get_timestamp() << "Starting call to robot.init()..." << std::endl;
    if (robot.init()) {
        std::cout << get_timestamp() << "Base instance initialized successfully!" << std::endl;
    } else {
        std::cerr << get_timestamp() << "Base instance initialization failed!" << std::endl;
        return -1;
    }

    std::cout << get_timestamp() << "Starting call to navigation.init()..." << std::endl;
    if (navigation.init()) {
        std::cout << get_timestamp() << "Navigation instance initialized successfully!" << std::endl;
    } else {
        std::cerr << get_timestamp() << "Navigation instance initialization failed!" << std::endl;
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
    std::cout << get_timestamp() << "Enter relocalization check loop..." << std::endl;
    int count_relocalize = 0;
    while (!navigation.is_localized() && count_relocalize < 20) {
        std::cout << get_timestamp() << "Calling navigation.relocalize()..." << std::endl;
        navigation.relocalize(init_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << get_timestamp() << "is relocalizing" << std::endl;
        count_relocalize++;
    }

    if (navigation.is_localized()) {
        std::cout << get_timestamp() << "relocalization success." << std::endl;

        // Get current pose
        std::cout << get_timestamp() << "Calling navigation.get_current_pose()..." << std::endl;
        Pose current_pose = navigation.get_current_pose();
        std::cout << get_timestamp() << "Current pose: Position(" << current_pose.position.x << ", "
                << current_pose.position.y << ", " << current_pose.position.z
                << "), orientation(" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", " << current_pose.orientation.z
                << ", " << current_pose.orientation.w << ")" << std::endl;

        // Whether to block and wait for arrival
        bool is_blocking = true;
        // Maximum wait time to reach position
        float timeout_s = 20;

        // --- ---
        std::cout << "--------------------------------------------------" << std::endl;
        std::cout << get_timestamp() << "Preparing to execute move_straight_to (linear movement)..." << std::endl;
        
        // Record start time
        auto start_move = std::chrono::system_clock::now();
        
        // Execute move
        NavigationStatus status = navigation.move_straight_to(goal_pose, is_blocking, timeout_s);
        
        // Record end time and calculate elapsed time
        auto end_move = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_move - start_move).count();

        if (status == NavigationStatus::SUCCESS) {
            std::cout << get_timestamp() << "Target point reached (elapsed: " << duration << "ms)" << std::endl;
        } else {
            std::cout << get_timestamp() << "navigationfailed, status: " << static_cast<int>(status) 
                    << " (elapsed: " << duration << "ms)" << std::endl;
        }
        std::cout << "--------------------------------------------------" << std::endl;

        // Stop navigation
        std::cout << get_timestamp() << "Calling navigation.stop_navigation()..." << std::endl;
        navigation.stop_navigation();

        // Get current pose again
        std::cout << get_timestamp() << "Calling navigation.get_current_pose()..." << std::endl;
        current_pose = navigation.get_current_pose();
        std::cout << get_timestamp() << "Current pose: Position(" << current_pose.position.x << ", "
                << current_pose.position.y << ", " << current_pose.position.z
                << "), orientation(" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", " << current_pose.orientation.z
                << ", " << current_pose.orientation.w << ")" << std::endl;
    } else {
        std::cout << get_timestamp() << "Relocalization failed, cannot continue navigation." << std::endl;
    }
    
    std::cout << get_timestamp() << "Executing shutdown..." << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    std::cout << get_timestamp() << "Program finished." << std::endl;

    return 0;
}
