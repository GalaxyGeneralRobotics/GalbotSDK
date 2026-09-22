#include <algorithm>
#include <chrono>
#include <cctype>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {

constexpr int kCycleCount = 3;
constexpr int kStartupDelayMs = 2000;
constexpr int kCommandIntervalMs = 1000;

// These demo positions match the standalone set_dexhand_command example.
// Each dexterous hand model has its own joint count and position unit.
const std::vector<double> INSPIRE_RH56DFX_POSITION_OPEN = {
    1000.0,  // finger1
    1000.0,  // finger2
    1000.0,  // finger3
    1000.0,  // finger4
    1000.0,  // finger5
    1000.0,  // finger6
};

const std::vector<double> INSPIRE_RH56DFX_POSITION_CLOSE = {
    850.0,  // finger1
    350.0,  // finger2
    60.0,   // finger3
    60.0,   // finger4
    60.0,   // finger5
    60.0,   // finger6
};

const std::vector<double> INSPIRE_RH56F2_POSITION_OPEN = {
    1750.0,  // thumb_bend
    1550.0,  // thumb_rotate
    1740.0,  // index
    1740.0,  // middle
    1740.0,  // ring
    1740.0,  // little
};

const std::vector<double> INSPIRE_RH56F2_POSITION_CLOSE = {
    1600.0,  // thumb_bend
    1250.0,  // thumb_rotate
    950.0,   // index
    950.0,   // middle
    950.0,   // ring
    950.0,   // little
};

const std::vector<double> BRAINCO_POSITION_OPEN = {
    100.0,  // finger1
    100.0,  // finger2
    100.0,  // finger3
    100.0,  // finger4
    100.0,  // finger5
    100.0,  // finger6
};

const std::vector<double> BRAINCO_POSITION_CLOSE = {
    33.3,   // finger1
    100.0,  // finger2
    86.7,   // finger3
    86.7,   // finger4
    86.7,   // finger5
    86.7,   // finger6
};

const std::vector<double> LINKER_L20_POSITION_OPEN = {
    1.3543754995475996,   // thumb_roll
    1.562069680534925,    // thumb_yaw
    0.23561944901923448,  // index_yaw
    0.23561944901923448,  // middle_yaw
    0.23561944901923448,  // ring_yaw
    0.23561944901923448,  // little_yaw
    0.82030474843733492,  // thumb_root
    1.2217304763960306,   // index_root
    1.2217304763960306,   // middle_root
    1.2217304763960306,   // ring_root
    1.2217304763960306,   // little_root
    1.2042771838760873,   // thumb_tip
    1.7453292519943295,   // index_tip
    1.7453292519943295,   // middle_tip
    1.7453292519943295,   // ring_tip
    1.7453292519943295,   // little_tip
};

const std::vector<double> LINKER_L20_POSITION_CLOSE = {
    0.85,  // thumb_roll
    1.55,  // thumb_yaw
    0.00,  // index_yaw
    0.00,  // middle_yaw
    0.00,  // ring_yaw
    0.00,  // little_yaw
    0.40,  // thumb_root
    0.00,  // index_root
    0.00,  // middle_root
    0.00,  // ring_root
    0.00,  // little_root
    0.35,  // thumb_tip
    0.00,  // index_tip
    0.00,  // middle_tip
    0.00,  // ring_tip
    0.00,  // little_tip
};

const std::vector<double> SHARPA_POSITION_OPEN = {
    0.0,  // thumb CMC_FE
    0.0,  // thumb CMC_AA
    0.0,  // thumb MCP_FE
    0.0,  // thumb MCP_AA
    0.0,  // thumb IP
    0.0,  // index MCP_FE
    0.0,  // index MCP_AA
    0.0,  // index PIP
    0.0,  // index DIP
    0.0,  // middle MCP_FE
    0.0,  // middle MCP_AA
    0.0,  // middle PIP
    0.0,  // middle DIP
    0.0,  // ring MCP_FE
    0.0,  // ring MCP_AA
    0.0,  // ring PIP
    0.0,  // ring DIP
    0.0,  // pinky CMC
    0.0,  // pinky MCP_FE
    0.0,  // pinky MCP_AA
    0.0,  // pinky PIP
    0.0,  // pinky DIP
};

const std::vector<double> SHARPA_POSITION_CLOSE = {
    0.87,  // thumb CMC_FE
    0.0,   // thumb CMC_AA
    0.44,  // thumb MCP_FE
    0.0,   // thumb MCP_AA
    0.87,  // thumb IP
    0.78,  // index MCP_FE
    0.0,   // index MCP_AA
    0.87,  // index PIP
    0.70,  // index DIP
    0.78,  // middle MCP_FE
    0.0,   // middle MCP_AA
    0.87,  // middle PIP
    0.70,  // middle DIP
    0.78,  // ring MCP_FE
    0.0,   // ring MCP_AA
    0.87,  // ring PIP
    0.70,  // ring DIP
    0.13,  // pinky CMC
    0.78,  // pinky MCP_FE
    0.0,   // pinky MCP_AA
    0.87,  // pinky PIP
    0.70,  // pinky DIP
};

std::string to_lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

bool parse_dexhand_type(const std::string& type_name, DexHandType& dexhand_type) {
  const std::string key = to_lower(type_name);
  if (key == "inspire") {
    dexhand_type = DexHandType::INSPIRE;
    return true;
  }
  if (key == "inspire_rh56dfx") {
    dexhand_type = DexHandType::INSPIRE_RH56DFX;
    return true;
  }
  if (key == "inspire_rh56f2") {
    dexhand_type = DexHandType::INSPIRE_RH56F2;
    return true;
  }
  if (key == "brainco" || key == "revo") {
    dexhand_type = DexHandType::BRAINCO;
    return true;
  }
  if (key == "sharpa") {
    dexhand_type = DexHandType::SHARPA;
    return true;
  }
  if (key == "linker_l20") {
    dexhand_type = DexHandType::LINKER_L20;
    return true;
  }
  return false;
}

bool is_exit_input(const std::string& value) {
  const std::string key = to_lower(value);
  return key == "q" || key == "quit" || key == "exit";
}

void print_supported_types() {
  std::cout << "Supported dexterous hand types:" << std::endl;
  std::cout << "  inspire          Compatibility alias for inspire_rh56f2" << std::endl;
  std::cout << "  inspire_rh56dfx  Inspire RH56DFX dexterous hand" << std::endl;
  std::cout << "  inspire_rh56f2   Inspire RH56F2 dexterous hand" << std::endl;
  std::cout << "  brainco          BrainCo dexterous hand" << std::endl;
  std::cout << "  revo             Alias for brainco" << std::endl;
  std::cout << "  sharpa           Sharpa dexterous hand" << std::endl;
  std::cout << "  linker_l20       Linker Hand L20 dexterous hand" << std::endl;
}

bool read_dexhand_type(const std::string& side_name, DexHandType& dexhand_type, std::string& type_label) {
  while (true) {
    std::cout << "Enter " << side_name << " dexterous hand type (or q to quit): ";
    std::string raw_value;
    if (!std::getline(std::cin, raw_value)) {
      return false;
    }

    type_label = to_lower(raw_value);
    if (is_exit_input(type_label)) {
      return false;
    }
    if (parse_dexhand_type(type_label, dexhand_type)) {
      return true;
    }

    std::cout << "Unsupported dexterous hand type: " << raw_value << std::endl;
    std::cout << "Please choose from: inspire, inspire_rh56dfx, inspire_rh56f2, brainco, revo, sharpa, linker_l20"
              << std::endl;
  }
}

std::vector<JointCommand> make_dexhand_command(const std::vector<double>& positions) {
  std::vector<JointCommand> commands(positions.size());
  for (size_t i = 0; i < positions.size(); ++i) {
    commands[i].position = positions[i];
  }
  return commands;
}

const std::vector<double>& dexhand_motion_positions(DexHandType dexhand_type, const std::string& action) {
  const bool is_open = action == "open";
  if (dexhand_type == DexHandType::INSPIRE_RH56DFX) {
    return is_open ? INSPIRE_RH56DFX_POSITION_OPEN : INSPIRE_RH56DFX_POSITION_CLOSE;
  }
  if (dexhand_type == DexHandType::INSPIRE || dexhand_type == DexHandType::INSPIRE_RH56F2) {
    return is_open ? INSPIRE_RH56F2_POSITION_OPEN : INSPIRE_RH56F2_POSITION_CLOSE;
  }
  if (dexhand_type == DexHandType::BRAINCO) {
    return is_open ? BRAINCO_POSITION_OPEN : BRAINCO_POSITION_CLOSE;
  }
  if (dexhand_type == DexHandType::SHARPA) {
    return is_open ? SHARPA_POSITION_OPEN : SHARPA_POSITION_CLOSE;
  }
  return is_open ? LINKER_L20_POSITION_OPEN : LINKER_L20_POSITION_CLOSE;
}

void print_dexhand_state(const std::string& hand_name, const DexhandState& dexhand_state,
                         DexHandType dexhand_type) {
  const char* type_label = dexhand_type == DexHandType::SHARPA ? "sharpa" : "dexterous";
  std::cout << hand_name << " " << type_label << " hand state:" << std::endl;
  std::cout << "Timestamp (ns): " << dexhand_state.timestamp_ns << std::endl;

  const auto& joint_state_vec = dexhand_state.joint_state.joint_state_vec;
  std::cout << "  Joint states (" << joint_state_vec.size() << " joints):" << std::endl;
  for (size_t i = 0; i < joint_state_vec.size(); ++i) {
    const auto& js = joint_state_vec[i];
    if (dexhand_type == DexHandType::SHARPA) {
      // Sharpa state does not use the generic acceleration field in the same way.
      std::cout << "    joint" << (i + 1) << ": position=" << js.position << ", velocity=" << js.velocity
                << ", effort=" << js.effort << ", current=" << js.current << std::endl;
    } else {
      // Some SDK backends fill joint_name; otherwise keep a stable fallback label.
      const std::string joint_label = !js.joint_name.empty()
                                          ? js.joint_name
                                          : hand_name + "_dexhand_joint" + std::to_string(i + 1);
      std::cout << "    " << joint_label << ": position=" << js.position
                << ", velocity=" << js.velocity << ", acceleration=" << js.acceleration
                << ", effort=" << js.effort << ", current=" << js.current << std::endl;
    }
  }

  if (dexhand_type != DexHandType::SHARPA) {
    return;
  }

  const std::unordered_map<std::string, EffortInfo>& force_sensor_map = dexhand_state.force_sensor_map;
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

}  // namespace

int main() {
  std::cout << "Starting get_set_dexhand_state example" << std::endl;
  print_supported_types();

  // Ask the user to select the actual dexterous hand model installed on each side.
  DexHandType left_type;
  std::string left_label;
  if (!read_dexhand_type("left", left_type, left_label)) {
    std::cout << "Example canceled before robot initialization" << std::endl;
    return 0;
  }

  DexHandType right_type;
  std::string right_label;
  if (!read_dexhand_type("right", right_type, right_label)) {
    std::cout << "Example canceled before robot initialization" << std::endl;
    return 0;
  }

  auto& robot = GalbotRobot::get_instance(MachineType::G3);

  // Initialize the robot SDK before sending commands or reading states.
  if (!robot.init()) {
    std::cerr << "System initialization failed!" << std::endl;
    return -1;
  }

  std::cout << "Initialization succeeded" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(kStartupDelayMs));

  bool completed = true;

  // Run one initial open command, then run three close/open demo cycles.
  for (int action_index = 0; action_index < 1 + kCycleCount * 2; ++action_index) {
    const std::string action = (action_index == 0 || action_index % 2 == 0) ? "open" : "close";
    if (action_index > 0 && action == "close") {
      const int cycle_index = (action_index + 1) / 2;
      std::cout << "Running dexterous hand cycle " << cycle_index << "/" << kCycleCount << std::endl;
    }

    // Send the target position command to the left dexterous hand.
    const auto left_positions = dexhand_motion_positions(left_type, action);
    const auto left_command = make_dexhand_command(left_positions);
    const ControlStatus left_status = robot.set_dexhand_command(
        "left_dexhand",
        left_command,
        left_type,
        false);
    if (left_status != ControlStatus::SUCCESS) {
      std::cerr << "Failed to " << action << " left " << left_label
                << " dexterous hand (" << left_command.size() << " joints), status="
                << static_cast<int>(left_status) << std::endl;
      completed = false;
    } else {
      std::cout << "Left " << left_label << " dexterous hand " << action
                << " command sent (" << left_command.size() << " joints)" << std::endl;
    }

    // Send the same action to the right dexterous hand with its own model type.
    const auto right_positions = dexhand_motion_positions(right_type, action);
    const auto right_command = make_dexhand_command(right_positions);
    const ControlStatus right_status = robot.set_dexhand_command(
        "right_dexhand",
        right_command,
        right_type,
        false);
    if (right_status != ControlStatus::SUCCESS) {
      std::cerr << "Failed to " << action << " right " << right_label
                << " dexterous hand (" << right_command.size() << " joints), status="
                << static_cast<int>(right_status) << std::endl;
      completed = false;
    } else {
      std::cout << "Right " << right_label << " dexterous hand " << action
                << " command sent (" << right_command.size() << " joints)" << std::endl;
    }

    // Give the hardware or simulator time to execute the command before reading state.
    std::this_thread::sleep_for(std::chrono::milliseconds(kCommandIntervalMs));

    // Read back the left hand state after the command has been applied.
    DexhandState left_state;
    const ControlStatus left_state_status = robot.get_dexhand_state("left_dexhand", left_state, left_type);
    if (left_state_status != ControlStatus::SUCCESS) {
      std::cerr << "Failed to get left dexterous hand state!" << std::endl;
      completed = false;
    } else {
      print_dexhand_state("Left", left_state, left_type);
    }

    // Read back the right hand state after the command has been applied.
    DexhandState right_state;
    const ControlStatus right_state_status = robot.get_dexhand_state("right_dexhand", right_state, right_type);
    if (right_state_status != ControlStatus::SUCCESS) {
      std::cerr << "Failed to get right dexterous hand state!" << std::endl;
      completed = false;
    } else {
      print_dexhand_state("Right", right_state, right_type);
    }
  }

  if (completed) {
    std::cout << "get_set_dexhand_state example completed" << std::endl;
  } else {
    std::cout << "get_set_dexhand_state example completed with one or more errors" << std::endl;
  }

  // Release SDK resources after the initialized example finishes or fails.
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  std::cout << "Resources released successfully" << std::endl;

  return completed ? 0 : 1;
}
