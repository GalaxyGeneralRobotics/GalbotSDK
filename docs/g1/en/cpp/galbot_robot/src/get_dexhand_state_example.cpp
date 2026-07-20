#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {

DexHandType parse_dexhand_type(const std::string& type_name) {
  if (type_name == "inspire" || type_name == "INSPIRE") {
    return DexHandType::INSPIRE;
  }
  if (type_name == "brainco" || type_name == "BRAINCO") {
    return DexHandType::BRAINCO;
  }
  if (type_name == "sharpa" || type_name == "SHARPA") {
    return DexHandType::SHARPA;
  }
  if (type_name == "linker_l20" || type_name == "LINKER_L20") {
    return DexHandType::LINKER_L20;
  }
  std::cerr << "Unknown dexhand type '" << type_name
            << "'. Choose from: inspire, brainco, sharpa, linker_l20" << std::endl;
  std::exit(1);
}

void print_joint_states(const std::string& hand_name, const DexhandState& dexhand_state,
                        DexHandType dexhand_type) {
  const char* type_label = dexhand_type == DexHandType::SHARPA ? "sharpa" : "dexterous";
  std::cout << hand_name << " " << type_label << " hand state:" << std::endl;
  std::cout << "Timestamp (ns): " << dexhand_state.timestamp_ns << std::endl;

  const auto& joint_state_vec = dexhand_state.joint_state.joint_state_vec;
  std::cout << "  Joint states (" << joint_state_vec.size() << " joints):" << std::endl;
  for (size_t i = 0; i < joint_state_vec.size(); ++i) {
    const auto& js = joint_state_vec[i];
    if (dexhand_type == DexHandType::SHARPA) {
      std::cout << "    joint" << (i + 1) << ": position=" << js.position << ", velocity=" << js.velocity
                << ", effort=" << js.effort << ", current=" << js.current << std::endl;
    } else {
      // SDK fills joint_name for LINKER_L20 (e.g. left_dexhand_thumb_roll); fall back for others
      const std::string joint_label = !js.joint_name.empty()
                                          ? js.joint_name
                                          : hand_name + "_dexhand_joint" + std::to_string(i + 1);
      std::cout << "    " << joint_label << ": position=" << js.position
                << ", velocity=" << js.velocity << ", acceleration=" << js.acceleration
                << ", effort=" << js.effort << ", current=" << js.current << std::endl;
    }
  }
}

void print_force_sensors(const std::unordered_map<std::string, EffortInfo>& force_sensor_map) {
  if (force_sensor_map.empty()) {
    std::cout << "  (no force sensor data)" << std::endl;
    return;
  }
  std::cout << "  Force sensors (" << force_sensor_map.size() << " sensors):" << std::endl;
  for (const auto& sensor_pair : force_sensor_map) {
    const auto& effort = sensor_pair.second;
    std::cout << "    " << sensor_pair.first << " @ " << effort.timestamp_ns
              << ": Fx=" << effort.force.x << ", Fy=" << effort.force.y << ", Fz=" << effort.force.z
              << ", Mx=" << effort.torque.x << ", My=" << effort.torque.y << ", Mz=" << effort.torque.z
              << std::endl;
  }
}

void print_dexhand_state(const std::string& hand_name, const DexhandState& dexhand_state,
                         DexHandType dexhand_type) {
  print_joint_states(hand_name, dexhand_state, dexhand_type);
  if (dexhand_type == DexHandType::SHARPA) {
    print_force_sensors(dexhand_state.force_sensor_map);
  }
}

void print_usage(const char* program_name) {
  std::cerr << "Usage: " << program_name << " [inspire|brainco|sharpa|linker_l20]" << std::endl;
  std::cerr << "  inspire (default): joint state only" << std::endl;
  std::cerr << "  brainco:           joint state only" << std::endl;
  std::cerr << "  sharpa:            joint state + force sensors when available" << std::endl;
  std::cerr << "  linker_l20:        joint state with per-joint names from SDK" << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string type_arg = "inspire";
  if (argc > 2) {
    print_usage(argv[0]);
    return 1;
  }
  if (argc == 2) {
    type_arg = argv[1];
  }

  const auto dexhand_type = parse_dexhand_type(type_arg);

  auto& robot = GalbotRobot::get_instance(MachineType::G1);

  if (robot.init()) {
    std::cout << "System initialized successfully!" << std::endl;
  } else {
    std::cerr << "System initialization failed!" << std::endl;
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  DexhandState left_dexhand_state;
  auto status = robot.get_dexhand_state("left_dexhand", left_dexhand_state, dexhand_type);

  if (status == ControlStatus::SUCCESS) {
    print_dexhand_state("Left", left_dexhand_state, dexhand_type);
  } else {
    std::cerr << "Failed to get left dexterous hand state!" << std::endl;
  }

  DexhandState right_dexhand_state;
  status = robot.get_dexhand_state("right_dexhand", right_dexhand_state, dexhand_type);

  if (status == ControlStatus::SUCCESS) {
    print_dexhand_state("Right", right_dexhand_state, dexhand_type);
  } else {
    std::cerr << "Failed to get right dexterous hand state!" << std::endl;
  }

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return 0;
}
