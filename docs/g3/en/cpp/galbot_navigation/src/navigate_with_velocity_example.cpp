
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {

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
  std::cout << indent << "current_pose: [" << current_pose.position.x << ", " << current_pose.position.y << ", "
            << current_pose.position.z << ", " << current_pose.orientation.x << ", " << current_pose.orientation.y
            << ", " << current_pose.orientation.z << ", " << current_pose.orientation.w << "]" << std::endl;
}

}  // namespace

int test_base_vel_cmd() {
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

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  print_current_pose(navigation);

  const double vx = 0.3;
  const double vy = 0.0;
  const double vyaw = 0.0;
  const double duration_s = 6.0;
  const bool enable_collision_check = true;

  std::cout << "navigate_with_velocity: vx=" << vx << ", vy=" << vy << ", vyaw=" << vyaw
            << ", duration_s=" << duration_s << std::endl;

  NavigationStatus status = navigation.navigate_with_velocity(vx, vy, vyaw, duration_s, enable_collision_check);

  std::cout << "navigate_with_velocity status: " << navigation_status_to_string(status) << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((duration_s + 2.0) * 1000)));

  navigation.stop_navigation();
  print_current_pose(navigation);
  std::cout << "navigation stopped" << std::endl;

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  std::cout << "example completed ..." << std::endl;
  return 0;
}

int test_change_vel_cmd() {
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

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  print_current_pose(navigation);

  {
    double vx = 0.3;
    double vy = 0.0;
    double vyaw = 0.0;
    double duration_s = 6.0;
    bool enable_collision_check = true;

    std::cout << "navigate_with_velocity: vx=" << vx << ", vy=" << vy << ", vyaw=" << vyaw
              << ", duration_s=" << duration_s << std::endl;
    NavigationStatus status = navigation.navigate_with_velocity(vx, vy, vyaw, duration_s, enable_collision_check);
    std::cout << "navigate_with_velocity status 1: " << navigation_status_to_string(status) << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  }

  std::cout << "已经运行4s 现在开始下发新的速度指令 =======" << std::endl;
  {
    double vx = 0.1;
    double vy = 0.2;
    double vyaw = 0.0;
    double duration_s = 6.0;
    bool enable_collision_check = true;

    std::cout << "navigate_with_velocity: vx=" << vx << ", vy=" << vy << ", vyaw=" << vyaw
              << ", duration_s=" << duration_s << std::endl;
    NavigationStatus status = navigation.navigate_with_velocity(vx, vy, vyaw, duration_s, enable_collision_check);
    std::cout << "navigate_with_velocity status 2: " << navigation_status_to_string(status) << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((duration_s + 2.0) * 1000)));
  }

  navigation.stop_navigation();
  print_current_pose(navigation);
  std::cout << "navigation stopped" << std::endl;

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  std::cout << "example completed ..." << std::endl;
  return 0;
}

// 根据选择放开不同的注释
int main() {
  test_base_vel_cmd();  // 测试速度导航接口基础功能

  //   test_change_vel_cmd();// 测试中途速度变化Command下发后的导航运动功能

  return 0;
}