#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace galbot::sdk;

namespace {

constexpr int kLocalizationRetryLimit = 20;
constexpr int kLocalizationRetrySleepMs = 500;
constexpr int kPollIntervalMs = 500;
constexpr int kMonitorDurationMs = 2000;

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


void print_pose(const Pose& pose, const std::string& label) {
  std::cout << label << ": ["
            << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ", "
            << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", "
            << pose.orientation.w << "]" << std::endl;
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

bool wait_for_localization(GalbotNavigation& navigation, const Pose& init_pose) {
  int retry = 0;
  while (!navigation.is_localized() && retry < kLocalizationRetryLimit) {
    navigation.relocalize(init_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(kLocalizationRetrySleepMs));
    ++retry;
  }
  return navigation.is_localized();
}

void print_task_summary(GalbotNavigation& navigation, const std::vector<TaskHandle>& handles) {
  std::cout << "\nfinal task summary" << std::endl;
  for (const auto& task : handles) {
    const NavigationTaskSnapshot snapshot = navigation.get_navigation_target_status(task.task_id);
    print_task_snapshot(task, snapshot);
  }
}


}  // namespace

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
  auto controller_status = robot.switch_controller(G3ControllerName::CHASSIS_POSE_CTRL);
  if (controller_status != ControlStatus::SUCCESS) {
    std::cerr << "switch controller failed" << std::endl;
    return -1;
  }

  Pose init_pose(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

  /* Wait for data preparation */
  std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 

  if (!wait_for_localization(navigation, init_pose)) {
    std::cerr << "localization failed" << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return -1;
  }

  const std::string frame_id = "map";
  float speed_ratio = 0.6f;
  bool enable_collision_check = true;
  const std::vector<Pose> target_points = {
      Pose(std::vector<double>{1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00}),
      Pose(std::vector<double>{0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00}),
      Pose(std::vector<double>{0.30, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00})
  };

  std::vector<TaskHandle> handles;
  handles.reserve(target_points.size());

  for (size_t i = 0; i < target_points.size(); ++i) {
    std::cout << "set target: dynamic_goal_" << i << ", target_pose: ["
              << target_points[i].position.x << ", " << target_points[i].position.y << ", " << target_points[i].position.z << ", "
              << target_points[i].orientation.x << ", " << target_points[i].orientation.y << ", " << target_points[i].orientation.z << ", "
              << target_points[i].orientation.w << "]" << std::endl;

    TaskHandle handle =
        navigation.set_navigation_target(target_points[i], frame_id, speed_ratio, enable_collision_check);

    handles.push_back(handle);

    const auto start = std::chrono::steady_clock::now();
    while (true) {
      NavigationTaskSnapshot snapshot = navigation.get_navigation_target_status(handle.task_id);
      print_task_snapshot(handle, snapshot);
      print_current_pose(navigation, "    ");

      const auto now = std::chrono::steady_clock::now();
      const auto elapsed_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
      if (elapsed_ms >= kMonitorDurationMs) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(kPollIntervalMs));
    }
  }

  print_task_summary(navigation, handles);

  std::cout << "\nstop navigation and shutdown" << std::endl;
  navigation.stop_navigation();

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return 0;
}
