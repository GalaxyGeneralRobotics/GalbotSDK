#include <iostream>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>
#include "galbot_robot.hpp"
using namespace galbot::sdk;
static void print_fields(const char* title, ControlStatus status, const std::vector<ConfigItem>& fields) {
  std::cout << "[" << title << "] status: " << static_cast<int>(status) << std::endl;
  for (const auto& field : fields) {
    std::cout << "  " << field.key << " = ";
    std::visit([](const auto& value) {
      using T = std::decay_t<decltype(value)>;
      if constexpr (std::is_arithmetic_v<T> || std::is_same_v<T, std::string>) std::cout << value;
      else std::cout << "[array, " << value.size() << " elements]";
    }, field.value);
    std::cout << std::endl;
  }
}
int main() {
  auto& robot = GalbotRobot::get_instance(MachineType::G1);
  if (!robot.init()) return -1;
  const std::vector<std::string> keys = {"report_error_skip", "report_sensor_skip"};
  std::vector<ConfigItem> fields;
  auto status = robot.get_config(ConfigService::CONTROL, keys, &fields);
  print_fields("Current selected values", status, fields);
  fields.clear();
  status = robot.get_config(ConfigService::CONTROL, keys, &fields, true);
  print_fields("Built-in default values (use_default=true)", status, fields);
  robot.request_shutdown(); robot.wait_for_shutdown(); robot.destroy();
  return 0;
}
