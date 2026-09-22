#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

using namespace galbot::sdk;

namespace {

constexpr int kLocalizationRetryLimit = 20;
constexpr int kLocalizationRetrySleepMs = 500;
constexpr float kTaskTimeoutSec = 100.0f;

std::string navigation_target_status_to_string(NavigationTaskStatus status) {
    switch (status) {
        case NavigationTaskStatus::UNKNOWN:
            return "UNKNOWN";
        case NavigationTaskStatus::RUNNING:
            return "RUNNING";
        case NavigationTaskStatus::SUCCESS:
            return "SUCCESS";
        case NavigationTaskStatus::FAILED:
            return "FAILED";
        case NavigationTaskStatus::INTERRUPTED:
            return "INTERRUPTED";
        case NavigationTaskStatus::OCCUPIED:
            return "OCCUPIED";
        case NavigationTaskStatus::COLLISION:
            return "COLLISION";
        case NavigationTaskStatus::CLOSE_TO_OBSTACLE:
            return "CLOSE_TO_OBSTACLE";
        default:
            return "UNKNOWN";
    }
}

bool is_terminal_status(NavigationTaskStatus status) {
    return status == NavigationTaskStatus::SUCCESS || status == NavigationTaskStatus::FAILED ||
           status == NavigationTaskStatus::INTERRUPTED || status == NavigationTaskStatus::OCCUPIED ||
           status == NavigationTaskStatus::COLLISION || status == NavigationTaskStatus::CLOSE_TO_OBSTACLE;
}

void print_task_snapshot(const TaskHandle& handle, const NavigationTaskSnapshot& snapshot) {
    std::cout << "  task_id: " << handle.task_id
              << ", request_sent: " << (handle.request_sent ? "true" : "false")
              << ", status: " << navigation_target_status_to_string(snapshot.status) << std::endl;
}

void print_current_pose(GalbotNavigation& navigation, const std::string& indent = "  ") {
    Pose current_pose = navigation.get_current_pose();
    std::cout << indent << "current_pose: ["
              << current_pose.position.x << ", " << current_pose.position.y << ", " << current_pose.position.z << ", "
              << current_pose.orientation.x << ", " << current_pose.orientation.y << ", " << current_pose.orientation.z << ", "
              << current_pose.orientation.w << "]" << std::endl;
}

void monitor_task_until_terminal(GalbotNavigation& navigation, const TaskHandle& handle, float timeout_s) {
    const auto start = std::chrono::steady_clock::now();
    while (true) {
        NavigationTaskSnapshot snapshot = navigation.get_navigation_target_status(handle.task_id);
        print_task_snapshot(handle, snapshot);
        print_current_pose(navigation, "    ");

        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();

        if (is_terminal_status(snapshot.status)) {
            return;
        }

        if (elapsed >= timeout_s) {
            std::cout << "  timeout reached while waiting for task completion." << std::endl;
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

}  // namespace

bool wait_for_localization(GalbotNavigation& navigation, const Pose& init_pose) {
    int retry = 0;
    while (!navigation.is_localized() && retry < kLocalizationRetryLimit) {
        navigation.relocalize(init_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(kLocalizationRetrySleepMs));
        ++retry;
    }
    return navigation.is_localized();
}

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

    /* All coordinates are in the "map" frame. */
    Pose init_pose(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    std::vector<Pose> waypoints = {
        {std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}},
        {std::vector<double>{0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0}},
        {std::vector<double>{1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0}}
    };

    /* Wait for data preparation */
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    if (!wait_for_localization(navigation, init_pose)) {
        std::cerr << "localization failed" << std::endl;
        robot.request_shutdown();
        robot.wait_for_shutdown();
        robot.destroy();
        return -1;
    }

    // Print current pose
    print_current_pose(navigation);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "set target: waypoints" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  waypoint_" << i << ": ["
                  << waypoints[i].position.x << ", " << waypoints[i].position.y << ", " << waypoints[i].position.z << ", "
                  << waypoints[i].orientation.x << ", " << waypoints[i].orientation.y << ", " << waypoints[i].orientation.z << ", "
                  << waypoints[i].orientation.w << "]" << std::endl;
    }

    // Whether to enable obstacle checking (can be set to true in open environments)
    bool enable_collision_check = false;
    // The reference frame of the trajectory (can be set to "map" or "base_link", Defaulting to "map")
    const std::string frame_id = "map";
    // Speed scaling ratio for this trajectory execution, 1.0 means full speed.
    float speed_ratio = 1.0f;
    constexpr float timeout_s = kTaskTimeoutSec;

    TaskHandle handle =
        navigation.navigate_along_trajectory(waypoints, frame_id, speed_ratio, enable_collision_check);

    if (handle.request_sent) {
        monitor_task_until_terminal(navigation, handle, timeout_s);
    } else {
        std::cout << "trajectory task submission failed" << std::endl;
    }

    std::cout << "\nfinal task summary" << std::endl;
    NavigationTaskSnapshot snapshot = navigation.get_navigation_target_status(handle.task_id);
    std::cout << "  task_id: " << handle.task_id
              << ", request_sent: " << (handle.request_sent ? "true" : "false")
              << ", status: " << navigation_target_status_to_string(snapshot.status) << std::endl;

    // Stop navigation
    navigation.stop_navigation();

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
