#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_link_names(const std::vector<std::string>& link_names, const std::string& title) {
  std::cout << title << " (total " << link_names.size() << " items):" << std::endl;
  for (size_t i = 0; i < link_names.size(); ++i) {
    std::cout << "  " << (i + 1) << ". " << link_names[i] << std::endl;
  }
}

int main() {
  // Get object instance
  auto& motion = GalbotMotion::get_instance(MachineType::G3);
  auto& robot = GalbotRobot::get_instance(MachineType::G3);

  // Initialize system
  if (!motion.init()) {
    std::cerr << "GalbotMotion initialization failed!" << std::endl;
    return -1;
  }
  if (!robot.init()) {
    std::cerr << "GalbotRobot initialization failed!" << std::endl;
    return -1;
  }

  std::cout << "System initialized successfully!" << std::endl;

  // Program started, waiting for data
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  try {
    // Get all link names
    std::vector<std::string> all_link_names = motion.get_link_names(false);
    print_link_names(all_link_names, "\nAll link names");

    // getend effectorexecute link
    std::vector<std::string> ee_link_names = motion.get_link_names(true);
    print_link_names(ee_link_names, "\nEnd-effector link names");
  } catch (const std::exception& e) {
    std::cerr << "Get link name exception: " << e.what() << std::endl;
  }

  // Exit system and release SDK resources
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return 0;
}
