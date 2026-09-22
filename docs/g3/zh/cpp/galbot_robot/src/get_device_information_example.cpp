#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_device_info(const std::shared_ptr<DeviceInfo>& device_info) {
  if (!device_info) {
    std::cerr << "Device information is empty" << std::endl;
    return;
  }

  std::cout << "Device information:" << std::endl;
  std::cout << "  Model: " << (device_info->model.empty() ? "N/A" : device_info->model) << std::endl;
  std::cout << "  Serial number: " << (device_info->serial_number.empty() ? "N/A" : device_info->serial_number) << std::endl;
  std::cout << "  Firmware version: " << (device_info->firmware_version.empty() ? "N/A" : device_info->firmware_version)
            << std::endl;
  std::cout << "  Hardware version: " << (device_info->hardware_version.empty() ? "N/A" : device_info->hardware_version)
            << std::endl;
  std::cout << "  : " << (device_info->manufacturer.empty() ? "N/A" : device_info->manufacturer) << std::endl;
}

int main() {
  // Get object instance
  auto& robot = GalbotRobot::get_instance(MachineType::G3);

  // Initialize system
  if (robot.init()) {
    std::cout << "System initialized successfully!" << std::endl;
  } else {
    std::cerr << "System initialization failed!" << std::endl;
    return -1;
  }

  // Program started, waiting for data
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Get device information
  std::shared_ptr<DeviceInfo> device_info = robot.get_device_information();
  if (device_info) {
    std::cout << "Device information retrieved successfully!" << std::endl;
    print_device_info(device_info);
  } else {
    std::cerr << "Failed to get device information!" << std::endl;
  }

  // Exit system and release SDK resources
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return 0;
}
