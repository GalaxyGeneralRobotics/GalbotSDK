#include <iostream>
#include <string>
#include <vector>
#include "galbot_robot.hpp"
using namespace galbot::sdk;
int main() {
  auto& robot = GalbotRobot::get_instance(MachineType::S1);
  if (!robot.init()) return -1;
  const std::vector<std::string> keys = {"report_error_skip", "report_sensor_skip"};
  std::vector<ConfigItem> fields;
  auto status = robot.get_config(ConfigService::CONTROL, keys, &fields);
  std::cout << "status: " << static_cast<int>(status) << ", fields: " << fields.size() << std::endl;
  fields.clear(); status = robot.get_config(ConfigService::CONTROL, keys, &fields, true);
  std::cout << "default status: " << static_cast<int>(status) << std::endl;
  robot.request_shutdown(); robot.wait_for_shutdown(); robot.destroy(); return 0;
}
