#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

using namespace galbot::sdk;

namespace {

constexpr int kLocalizationRetryLimit = 20;
constexpr int kLocalizationRetrySleepMs = 500;
constexpr int kPollIntervalMs = 1000;
constexpr double kTaskTimeoutSec = 100.0;

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

void monitor_task_until_terminal(GalbotNavigation& navigation, const TaskHandle& handle, double timeout_s) {
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

        std::this_thread::sleep_for(std::chrono::milliseconds(kPollIntervalMs));
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

static Waypoint make_waypoint(const Pose& pose, const WaypointParams& params = WaypointParams()) {
    return Waypoint(pose, params);
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

    /*
     * Build a waypoint by combining:
     *   1) the target pose
     *   2) an optional WaypointParams object
     *
     * Usage:
     * - Use Waypoint(pose) if you only need the default settings.
     * - Create WaypointParams and set only the fields you want to override.
     * - Pass the params object into the Waypoint constructor to build a target point.
     */
    WaypointParams waypoint_2_params;
    waypoint_2_params.arrival_position_threshold_x = 0.02;
    waypoint_2_params.arrival_position_threshold_y = 0.02;
    waypoint_2_params.arrival_orientation_threshold = 0.08;
    waypoint_2_params.velocity_scale = 0.2;

    WaypointParams waypoint_3_params;
    waypoint_3_params.arrival_position_threshold_x = 0.04;
    waypoint_3_params.arrival_position_threshold_y = 0.04;
    waypoint_3_params.arrival_orientation_threshold = 0.05;
    waypoint_3_params.velocity_scale = 0.8;
    waypoint_3_params.acceleration_scale = 0.8;
    waypoint_3_params.jerk_scale = 0.8;

    std::vector<Waypoint> waypoints = {
        make_waypoint(Pose(std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0})),
        make_waypoint(Pose(std::vector<double>{0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0}), waypoint_2_params),
        make_waypoint(Pose(std::vector<double>{1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0}), waypoint_3_params)
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

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "set target: waypoints" << std::endl;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  waypoint_" << i << ": ["
                  << waypoints[i].pose.position.x << ", " << waypoints[i].pose.position.y << ", " << waypoints[i].pose.position.z << ", "
                  << waypoints[i].pose.orientation.x << ", " << waypoints[i].pose.orientation.y << ", " << waypoints[i].pose.orientation.z << ", "
                  << waypoints[i].pose.orientation.w << "]" << std::endl;
    }

    // The reference frame of the trajectory (can be set to "map" or "base_link", Defaulting to "map")
    const std::string frame_id = "map";
    // Whether to enable collision checking (can be set to true in open environments)
    bool enable_collision_check = true;

    TaskHandle handle = navigation.navigate_through_waypoints(waypoints, frame_id, enable_collision_check);

    if (handle.request_sent) {
        monitor_task_until_terminal(navigation, handle, kTaskTimeoutSec);
    } else {
        std::cout << "navigation failed to submit task" << std::endl;
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
