#include <iostream>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>
#include "galbot_robot.hpp"
using namespace galbot::sdk;
int main() {
  auto& robot = GalbotRobot::get_instance(MachineType::G3);
  if (!robot.init()) return -1;
  const std::vector<std::string> keys = {"report_error_skip", "report_sensor_skip"};
  std::vector<ConfigItem> fields;
  auto status = robot.get_config(ConfigService::CONTROL, keys, &fields);
  for (const auto& field : fields) {
    std::cout << field.key << " = ";
    std::visit([](const auto& value) {
      using T = std::decay_t<decltype(value)>;
      if constexpr (std::is_arithmetic_v<T> || std::is_same_v<T, std::string>) std::cout << value;
      else std::cout << "[array, " << value.size() << " elements]";
    }, field.value);
    std::cout << std::endl;
  }
  fields.clear(); status = robot.get_config(ConfigService::CONTROL, keys, &fields, true);
  std::cout << "default status: " << static_cast<int>(status) << std::endl;
  robot.request_shutdown(); robot.wait_for_shutdown(); robot.destroy(); return 0;
}
