#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_status(const std::string& operation, ControlStatus status) {
  if (status == ControlStatus::SUCCESS) {
    std::cout << operation << " succeeded." << std::endl;
  } else {
    std::cerr << operation << " failed, status: " << static_cast<int>(status) << std::endl;
  }
}

bool get_leg_height(GalbotRobot& robot, const std::string& label, double& height_m) {
  const auto [pose, timestamp_ns] = robot.get_transform("base_link", "head_base_link", 0, 500);
  std::cout << std::fixed << std::setprecision(3) << label << ": pose=[";
  for (size_t i = 0; i < 7; ++i) {
    std::cout << pose[i] << (i + 1 < 7 ? ", " : "");
  }
  std::cout << "], tf_timestamp_ns=" << timestamp_ns << std::endl;

  height_m = pose[2];
  return height_m >= 0.0;
}

bool confirm_leg_height_motion(double current_height_m, double target_height_m) {
  const char* direction = target_height_m < current_height_m ? "lower" : "raise";
  const double motion_distance_m = std::abs(target_height_m - current_height_m);
  std::cout << "The leg mechanism is about to " << direction << " the body by "
            << std::fixed << std::setprecision(3) << motion_distance_m
            << " m, from " << current_height_m << " m to " << target_height_m
            << " m. Confirm the surrounding environment is clear and safe to avoid collisions. "
            << "Enter y to continue; any other input cancels: ";
  std::string response;
  std::getline(std::cin, response);
  return response == "y" || response == "Y";
}

int main() {
  // Get and initialize the G3 robot instance.
  auto& robot = GalbotRobot::get_instance(MachineType::G3);
  std::cout << "Initializing robot..." << std::endl;
  if (!robot.init()) {
    std::cerr << "System initialization failed!" << std::endl;
    return -1;
  }
  std::cout << "System initialized successfully!" << std::endl;

  // Wait for WBC communication and state data to become ready.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  constexpr double kHeightOffsetM = 0.20;
  constexpr double kDurationS = 4.0;

  double initial_height_m = 0.0;
  if (!get_leg_height(robot, "Initial leg height", initial_height_m)) {
    std::cerr << "Failed to get a valid initial leg height." << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return -1;
  }

  const double lowered_height_m = initial_height_m - kHeightOffsetM;
  if (!confirm_leg_height_motion(initial_height_m, lowered_height_m)) {
    std::cout << "Leg height motion cancelled. Exiting program." << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return 0;
  }

  std::cout << "Lowering leg height by " << kHeightOffsetM
            << " m to " << lowered_height_m << " m." << std::endl;
  auto status = robot.set_leg_height(lowered_height_m, kDurationS, true);
  print_status("set_leg_height(lowered)", status);

  double actual_lowered_height_m = 0.0;
  if (!get_leg_height(robot, "Height after lowering", actual_lowered_height_m)) {
    std::cerr << "Failed to get the current leg height; cancelling the restore motion." << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return -1;
  }
  std::cout << "Lowered target error: " << std::abs(actual_lowered_height_m - lowered_height_m)
            << " m." << std::endl;

  if (!confirm_leg_height_motion(actual_lowered_height_m, initial_height_m)) {
    std::cout << "Leg height restore motion cancelled. Exiting program." << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return 0;
  }

  std::cout << "Restoring initial leg height: " << initial_height_m << " m." << std::endl;
  status = robot.set_leg_height(initial_height_m, kDurationS, true);
  print_status("set_leg_height(initial)", status);

  double final_height_m = 0.0;
  get_leg_height(robot, "Final leg height", final_height_m);
  std::cout << "Restored height error: " << std::abs(final_height_m - initial_height_m)
            << " m." << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::cout << "switch_controller back to leg_pvt_ctrl" << std::endl;
  robot.switch_controller(G3ControllerName::LEG_PVT_CTRL);

  std::cout << "Waiting 1 second before shutdown..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  std::cout << "Resources released successfully." << std::endl;
  return 0;
}
