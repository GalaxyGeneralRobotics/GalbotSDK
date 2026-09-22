#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_active_controller(const std::string& group_name, const std::string& controller_name) {
  std::cout << "Active controller for " << group_name << ": " << controller_name << std::endl;
}

// Helper function to print control status
void print_status(const std::string& operation, ControlStatus status) {
  std::cout << operation << ": ";
  if (status == ControlStatus::SUCCESS) {
    std::cout << "SUCCESS";
  } else {
    std::cout << "FAILED (Code: " << static_cast<int>(status) << ")";
  }
  std::cout << std::endl;
}

int main() {
  // Get robot instance
  auto& robot = GalbotRobot::get_instance(MachineType::G3);

  // Initialize system
  std::cout << "Initializing robot..." << std::endl;
  if (robot.init()) {
    std::cout << "System initialized successfully!" << std::endl;
  } else {
    std::cerr << "System initialization failed!" << std::endl;
    return -1;
  }

  // Wait for data readiness
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Define the controller we want to test
  // Using LEFT_ARM_PVT_CTRL as the test subject
  const std::string test_controller = G3ControllerName::LEFT_ARM_PVT_CTRL;
  const std::string controller_str = G3ControllerName::LEFT_ARM_PVT_CTRL;
  const std::string group_str = "left_arm";

  std::cout << "\n--- Starting Controller Management Example ---\n" << std::endl;

  // 1. Get active controller before switching
  std::string active_controller = robot.get_active_controller(group_str);
  print_active_controller(group_str, active_controller);

  // 2. Switch controller
  // Demonstrates switch_controller interface
  // This ensures the controller is acquired and started
  std::cout << "\n[Test 1] Switching to " << controller_str << "..." << std::endl;
  ControlStatus status = robot.switch_controller(test_controller);
  print_status("switch_controller", status);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  active_controller = robot.get_active_controller(group_str);
  print_active_controller(group_str, active_controller);

  // 3. Stop controller
  // Demonstrates stop_controller interface
  std::cout << "\n[Test 2] Stopping controller for group " << group_str << "..." << std::endl;
  status = robot.stop_controller(group_str);
  print_status("stop_controller", status);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 4. Release controller
  // Demonstrates release_controller interface
  std::cout << "\n[Test 3] Releasing controller for group " << group_str << "..." << std::endl;
  status = robot.release_controller(group_str);
  print_status("release_controller", status);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 5. Acquire controller
  // Demonstrates acquire_controller interface
  std::cout << "\n[Test 4] Acquiring " << controller_str << "..." << std::endl;
  status = robot.acquire_controller(test_controller);
  print_status("acquire_controller", status);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 6. Start controller
  // Demonstrates start_controller interface
  std::cout << "\n[Test 5] Starting controller for group " << group_str << "..." << std::endl;
  status = robot.start_controller(group_str);
  print_status("start_controller", status);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::cout << "\n--- Controller Management Example Finished ---\n" << std::endl;

  // Shutdown
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return 0;
}
