
#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <array>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace galbot::sdk;

namespace {

constexpr int kLocalizationRetryLimit = 20;
constexpr int kLocalizationRetrySleepMs = 500;

std::string navigation_status_to_string(NavigationStatus status) {
  switch (status) {
    case NavigationStatus::SUCCESS:
      return "SUCCESS";
    case NavigationStatus::FAIL:
      return "FAIL";
    case NavigationStatus::TIMEOUT:
      return "TIMEOUT";
    case NavigationStatus::INVALID_INPUT:
      return "INVALID_INPUT";
    case NavigationStatus::MODE_ERR:
      return "MODE_ERR";
    case NavigationStatus::COMM_ERR:
      return "COMM_ERR";
    case NavigationStatus::WAIT_INITIALIZED:
      return "WAIT_INITIALIZED";
    default:
      return "UNKNOWN_STATUS";
  }
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

}  // namespace

int main() {
  auto& navigation = GalbotNavigation::get_instance(MachineType::G3);
  auto& robot = GalbotRobot::get_instance(MachineType::G3);

  if (!robot.init()) {
    std::cerr << "Base instance initialization failed!" << std::endl;
    return -1;
  }
  if (!navigation.init()) {
    std::cerr << "Navigation instance initialization failed!" << std::endl;
    return -1;
  }

  auto controller_status = robot.switch_controller(G3ControllerName::CHASSIS_POSE_CTRL);
  if (controller_status != ControlStatus::SUCCESS) {
    std::cerr << "switch controller failed" << std::endl;
    return -1;
  }

  Pose init_pose(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  if (!wait_for_localization(navigation, init_pose)) {
    std::cerr << "localization failed" << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return -1;
  }

  print_current_pose(navigation);

  const Pose relative_goal(std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
  const std::array<double, 3> max_vel = {0.5, 0.5, 0.5};
  const std::string pose_frame = "base_link";
  const bool enable_collision_check = true;
  const bool is_blocking = true;
  const float timeout_s = 20.0f;
  const bool omni_plan = false;

  std::cout << "navigate_to_goal_v2 relative target in base_link frame" << std::endl;
  NavigationStatus status = navigation.navigate_to_goal_v2(
      relative_goal, max_vel, pose_frame, enable_collision_check, is_blocking, timeout_s, omni_plan);

  std::cout << "navigate_to_goal_v2 status: " << navigation_status_to_string(status) << std::endl;
  print_current_pose(navigation);

  navigation.stop_navigation();
  std::cout << "navigation stopped" << std::endl;

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();


  std::cout << "example completed ..." << std::endl;
  return 0;
}
