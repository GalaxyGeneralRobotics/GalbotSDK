#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <array>
#include <iostream>
#include <string>

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

void print_status(const std::string& name, NavigationStatus status) {
  std::cout << name << ": " << navigation_status_to_string(status) << std::endl;
}

}  // namespace

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

  std::cout << "dump navigation configs before setting parameters" << std::endl;
  print_status("dump_navigation_configs", navigation.dump_navigation_configs());

  const std::array<double, 3> vel_limit = {0.5, 0.5, 0.5};       // valid range: [0.05, 1.5]
  const std::array<double, 3> acc_limit = {0.5, 0.5, 0.5};       // valid range: [0.05, 6.0]
  const std::array<double, 3> jerk_limit = {5.0, 5.0, 5.0};      // valid range: [0.05, 12.0]
  const std::array<double, 3> arrival_threshold = {0.05, 0.05, 0.05};  // valid range: [0.03, 2.0]
  const double timeout_s = 30.0;

  print_status("set_navigation_velocity_limit",
               navigation.set_navigation_velocity_limit(vel_limit));

  print_status("set_navigation_kinematics_limits",
               navigation.set_navigation_kinematics_limits(vel_limit, acc_limit, jerk_limit));

  print_status("set_navigation_timeout",
               navigation.set_navigation_timeout(timeout_s));

  print_status("set_navigation_arrival_threshold",
               navigation.set_navigation_arrival_threshold(arrival_threshold));

  std::cout << "dump navigation configs after setting parameters" << std::endl;
  print_status("dump_navigation_configs", navigation.dump_navigation_configs());

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return 0;
}
