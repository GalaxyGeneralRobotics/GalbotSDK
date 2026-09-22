/**
 * Execute absolute-joint VLA action chunks with Galbot SDK interfaces.
 *
 * VLA control requires observations and actions to stay aligned in time. This
 * example uses get_synced_observation() to acquire a synchronized camera image
 * and robot joint state for each model request. It then uses
 * set_joint_commands() to stream the model's absolute-joint action chunk frame
 * by frame at the configured control frequency. The sample JSON has 23
 * dimensions; unavailable left/right gripper dimensions are filtered at
 * startup.
 *
 * For the training-data workflow, Galbot officially supports demonstration
 * data collection with leader-follower arms. See the TM01 user manual:
 * https://developer.galbot.com/docs/tm01/1.4.0/zh/tm01
 * The collected MCAP data can be converted to the LeRobot dataset format with
 * galbot-mcap2lerobot for subsequent model training:
 * https://github.com/GalaxyGeneralRobotics/galbot-mcap2lerobot
 * After deploying the trained model as an inference service, use this example
 * as a reference for synchronized observation acquisition and action-chunk
 * execution on the robot.
 *
 * This example demonstrates absolute joint-position model output. If a model
 * instead outputs end-effector poses (EE poses such as
 * [x, y, z, qx, qy, qz, qw]), use set_end_effector_command() for real-time
 * Cartesian control. See
 * examples/g3/cpp/galbot_robot/src/set_end_effector_commands_example.cpp.
 *
 * A JSON path is required because an installed executable may not have the
 * same directory layout as the SDK source tree. The SDK-provided sample is:
 * examples/g3/assets/tutorials/example5_vla.json
 *
 * Run: ./tutorials_example5_execute_vla <path/to/example5_vla.json>
 *
 * Before running, release the emergency-stop button and clear the area around
 * the robot. Keep a hand near the emergency-stop button during execution.
 */

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "galbot_robot.hpp"

namespace {

using galbot::sdk::ControlStatus;
using galbot::sdk::GalbotRobot;
using galbot::sdk::JointCommand;
using galbot::sdk::SensorType;

constexpr double kControlFrequencyHz = 20.0;
// Number of model action frames returned and executed in one chunk.
constexpr std::size_t kActionChunkSize = 30;
constexpr double kGripperVelocity = 0.5;
constexpr double kGripperEffort = 10.0;
constexpr char kTaskPrompt[] =
    "Based on the current view, predict the next robot action.";

const std::vector<std::string> kActionGroups = {
    "leg",          "head",      "left_arm",
    "left_gripper", "right_arm", "right_gripper",
};

const std::unordered_map<std::string, std::pair<std::size_t, std::size_t>>
    kGroupSlices = {
        {"leg", {0, 5}},           {"head", {5, 7}},
        {"left_arm", {7, 14}},     {"left_gripper", {14, 15}},
        {"right_arm", {15, 22}},   {"right_gripper", {22, 23}},
};

const std::unordered_map<std::string, std::vector<std::string>>
    kGroupJointNames = {
        {"leg",
         {"leg_joint1", "leg_joint2", "leg_joint3", "leg_joint4",
          "leg_joint5"}},
        {"head", {"head_joint1", "head_joint2"}},
        {"left_arm",
         {"left_arm_joint1", "left_arm_joint2", "left_arm_joint3",
          "left_arm_joint4", "left_arm_joint5", "left_arm_joint6",
          "left_arm_joint7"}},
        {"left_gripper", {"left_gripper_joint1"}},
        {"right_arm",
         {"right_arm_joint1", "right_arm_joint2", "right_arm_joint3",
          "right_arm_joint4", "right_arm_joint5", "right_arm_joint6",
          "right_arm_joint7"}},
        {"right_gripper", {"right_gripper_joint1"}},
};

struct VlaObservation {
  std::vector<double> state;
  std::size_t image_bytes = 0;
  std::string prompt;
};

struct MockVlaResponse {
  std::vector<std::vector<double>> actions;
  std::size_t next_cursor = 0;
};

std::vector<std::string> action_joint_names(
    const std::vector<std::string>& action_groups) {
  std::vector<std::string> names;
  for (const auto& group : action_groups) {
    const auto& group_names = kGroupJointNames.at(group);
    names.insert(names.end(), group_names.begin(), group_names.end());
  }
  return names;
}

std::string action_json_path(int argc, char* argv[]) {
  if (argc != 2) {
    throw std::runtime_error(
        "Usage: ./tutorials_example5_execute_vla <path/to/action.json>\n"
        "The SDK sample JSON is located at "
        "<SDK_ROOT>/examples/g3/assets/tutorials/example5_vla.json");
  }
  return argv[1];
}

void confirm_safe_environment() {
  std::cout
      << "WARNING: Release the emergency-stop button, keep the robot in "
         "working mode, and clear people and obstacles from its motion range."
      << std::endl;
  std::cout << "Continue with the VLA execution example? [y/N]: ";

  std::string answer;
  std::cin >> answer;
  std::transform(answer.begin(), answer.end(), answer.begin(),
                 [](unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  if (answer != "y" && answer != "yes") {
    throw std::runtime_error("Execution cancelled by the user");
  }
}

void validate_action_chunk(
    const std::vector<std::vector<double>>& actions,
    std::size_t expected_dimension = 23) {
  for (std::size_t frame_index = 0; frame_index < actions.size();
       ++frame_index) {
    if (actions[frame_index].size() != expected_dimension) {
      throw std::runtime_error(
          "VLA action frame " + std::to_string(frame_index) + " has " +
          std::to_string(actions[frame_index].size()) +
          " values; expected " + std::to_string(expected_dimension));
    }
    for (double value : actions[frame_index]) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("VLA actions contain NaN or infinity");
      }
    }
  }
}

std::vector<std::vector<double>> load_mock_actions(
    const std::string& json_path) {
  std::ifstream file(json_path);
  if (!file.is_open()) {
    throw std::runtime_error("Mock VLA action file does not exist: " +
                             json_path);
  }

  boost::property_tree::ptree payload;
  boost::property_tree::read_json(file, payload);

  std::vector<std::vector<double>> actions;
  for (const auto& frame_node : payload.get_child("frames")) {
    std::vector<double> frame;
    for (const auto& value_node : frame_node.second) {
      frame.push_back(value_node.second.get_value<double>());
    }
    actions.push_back(std::move(frame));
  }
  const std::size_t declared_frames = payload.get<std::size_t>("num_frames");
  if (declared_frames != actions.size()) {
    throw std::runtime_error(
        "Mock num_frames does not match the number of JSON frames: " +
        std::to_string(declared_frames) + " != " +
        std::to_string(actions.size()));
  }
  validate_action_chunk(actions);
  return actions;
}

VlaObservation collect_observation(
    GalbotRobot& robot,
    const std::vector<std::string>& action_groups) {
  const std::vector<SensorType> cameras = {SensorType::HEAD_LEFT_CAMERA};
  auto observation = robot.get_synced_observation(cameras, true);
  if (!observation) {
    throw std::runtime_error("get_synced_observation failed");
  }

  const auto image_iterator =
      observation->rgb_data_map.find(SensorType::HEAD_LEFT_CAMERA);
  if (image_iterator == observation->rgb_data_map.end() ||
      !image_iterator->second) {
    throw std::runtime_error("Synchronized head-left image is missing");
  }
  if (!observation->joint_state) {
    throw std::runtime_error("Synchronized joint state is missing");
  }

  std::unordered_map<std::string, double> state_by_name;
  for (const auto& joint_state :
       observation->joint_state->joint_state_vec) {
    state_by_name[joint_state.joint_name] = joint_state.position;
  }

  VlaObservation result;
  for (const auto& joint_name : action_joint_names(action_groups)) {
    const auto iterator = state_by_name.find(joint_name);
    if (iterator == state_by_name.end()) {
      throw std::runtime_error("Joint state is missing joint: " + joint_name);
    }
    result.state.push_back(iterator->second);
  }
  result.image_bytes = image_iterator->second->data.size();
  result.prompt = kTaskPrompt;
  return result;
}

MockVlaResponse mock_vla_server(
    const VlaObservation& observation,
    const std::vector<std::vector<double>>& all_actions, std::size_t cursor,
    const std::vector<std::string>& action_groups,
    std::size_t chunk_size = kActionChunkSize) {
  const std::size_t expected_dimension =
      action_joint_names(action_groups).size();
  if (observation.state.size() != expected_dimension) {
    throw std::runtime_error(
        "Observation state must contain " +
        std::to_string(expected_dimension) + " joints");
  }

  const std::size_t end =
      std::min(cursor + chunk_size, all_actions.size());
  MockVlaResponse response;
  response.actions.assign(all_actions.begin() + cursor,
                          all_actions.begin() + end);
  response.next_cursor = end;

  std::cout << "Mock VLA server received one request: prompt='"
            << observation.prompt << "', image_bytes="
            << observation.image_bytes << ", frames=" << cursor << ":" << end
            << std::endl;
  std::cout << "Mock VLA server returned action chunk: shape=("
            << response.actions.size() << ", " << expected_dimension << ")"
            << std::endl;
  return response;
}

std::vector<double> values_for_groups(
    const std::vector<double>& action,
    const std::vector<std::string>& groups) {
  std::vector<double> values;
  for (const auto& group : groups) {
    const auto [start, end] = kGroupSlices.at(group);
    values.insert(values.end(), action.begin() + start, action.begin() + end);
  }
  return values;
}

std::vector<std::string> detect_available_action_groups(GalbotRobot& robot) {
  // Some WBC configurations may still publish a placeholder gripper joint
  // without physical hardware. move_to_first_action() therefore performs a
  // second availability check using the initialization command status.
  std::unordered_set<std::string> available_grippers;
  for (const auto& [group, sdk_group] :
       std::vector<std::pair<std::string, std::string>>{
           {"left_gripper", "left_gripper"},
           {"right_gripper", "right_gripper"}}) {
    if (robot.get_gripper_state(sdk_group)) {
      available_grippers.insert(group);
      std::cout << "Detected " << group << " feedback." << std::endl;
    } else {
      std::cout << "No " << group
                << " detected; its action dimension will be skipped."
                << std::endl;
    }
  }

  std::vector<std::string> active_groups;
  for (const auto& group : kActionGroups) {
    if (group.find("gripper") == std::string::npos ||
        available_grippers.count(group) != 0) {
      active_groups.push_back(group);
    }
  }
  return active_groups;
}

std::vector<std::vector<double>> filter_actions_for_groups(
    const std::vector<std::vector<double>>& actions,
    const std::vector<std::string>& action_groups) {
  std::vector<std::vector<double>> filtered_actions;
  filtered_actions.reserve(actions.size());
  for (const auto& action : actions) {
    filtered_actions.push_back(values_for_groups(action, action_groups));
  }
  return filtered_actions;
}

std::vector<std::string> move_to_first_action(
    GalbotRobot& robot, const std::vector<double>& first_action,
    const std::vector<std::string>& action_groups) {
  const std::vector<std::vector<std::string>> movement_stages = {
      {"leg"},
      {"head", "left_arm", "right_arm"},
  };
  for (const auto& groups : movement_stages) {
    const ControlStatus status = robot.set_joint_positions(
        values_for_groups(first_action, groups), groups, {}, true, 0.2, 10.0);
    if (status != ControlStatus::SUCCESS) {
      throw std::runtime_error(
          "Failed to move a joint group stage to the first JSON action");
    }
  }

  std::vector<std::string> active_groups = action_groups;
  for (const auto& group : {std::string("left_gripper"),
                            std::string("right_gripper")}) {
    if (std::find(active_groups.begin(), active_groups.end(), group) ==
        active_groups.end()) {
      continue;
    }
    const ControlStatus status = robot.set_gripper_command(
        group, first_action[kGroupSlices.at(group).first], 0.1,
        kGripperEffort, true);
    if (status == ControlStatus::DATA_FETCH_FAILED ||
        status == ControlStatus::TIMEOUT) {
      std::cout << "WARNING: " << group << " is unavailable (status="
                << static_cast<int>(status)
                << "); its action dimension will be skipped." << std::endl;
      active_groups.erase(
          std::remove(active_groups.begin(), active_groups.end(), group),
          active_groups.end());
    } else if (status != ControlStatus::SUCCESS) {
      throw std::runtime_error(
          "Failed to initialize " + group + " from the first JSON action; "
          "status=" + std::to_string(static_cast<int>(status)));
    }
  }
  std::cout << "Robot reached the first pose in the VLA action file."
            << std::endl;
  return active_groups;
}

std::vector<JointCommand> build_joint_commands(
    const std::vector<double>& action,
    const std::vector<std::string>& action_groups) {
  const std::size_t expected_dimension =
      action_joint_names(action_groups).size();
  if (action.size() != expected_dimension) {
    throw std::runtime_error(
        "Action must contain " + std::to_string(expected_dimension) +
        " joint values");
  }

  std::vector<JointCommand> commands;
  commands.reserve(action.size());
  std::size_t offset = 0;
  for (const auto& group : action_groups) {
    const bool is_gripper = group.find("gripper") != std::string::npos;
    for (std::size_t index = 0; index < kGroupJointNames.at(group).size();
         ++index) {
      JointCommand command;
      command.position = action[offset++];
      if (is_gripper) {
        command.velocity = kGripperVelocity;
        command.effort = kGripperEffort;
      }
      commands.push_back(command);
    }
  }
  return commands;
}

void execute_action_chunk(
    GalbotRobot& robot,
    const std::vector<std::vector<double>>& actions,
    const std::vector<std::string>& action_groups) {
  validate_action_chunk(actions, action_joint_names(action_groups).size());
  if (actions.empty()) {
    std::cout << "The VLA model returned an empty chunk." << std::endl;
    return;
  }

  const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / kControlFrequencyHz));
  auto next_tick = std::chrono::steady_clock::now();
  std::cout << "Streaming " << actions.size() << " frames at "
            << kControlFrequencyHz << " Hz with set_joint_commands..."
            << std::endl;

  for (std::size_t frame_index = 0; frame_index < actions.size();
       ++frame_index) {
    const ControlStatus status = robot.set_joint_commands(
        build_joint_commands(actions[frame_index], action_groups),
        action_groups, {}, 0.0);
    if (status != ControlStatus::SUCCESS) {
      throw std::runtime_error("set_joint_commands failed at frame " +
                               std::to_string(frame_index + 1));
    }
    next_tick += period;
    std::this_thread::sleep_until(next_tick);
  }
  std::cout << "Action chunk execution completed." << std::endl;
}

void shutdown_robot(GalbotRobot& robot) {
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  std::cout << "SDK resources released." << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
  using galbot::sdk::MachineType;

  GalbotRobot& robot = GalbotRobot::get_instance(MachineType::G3);
  bool initialized = false;
  try {
    // Require an explicit path: the executable location on a robot is not
    // guaranteed to match the SDK source-tree layout.
    const std::string json_path = action_json_path(argc, argv);
    confirm_safe_environment();
    const std::unordered_set<SensorType> enabled_sensors = {
        SensorType::HEAD_LEFT_CAMERA};
    if (!robot.init(enabled_sensors, true)) {
      throw std::runtime_error("GalbotRobot initialization failed");
    }
    initialized = true;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    const auto full_actions = load_mock_actions(json_path);
    std::cout << "Loaded VLA actions from: " << json_path << std::endl;
    std::cout << "Action data shape: (" << full_actions.size()
              << ", 23), including head" << std::endl;
    auto action_groups = detect_available_action_groups(robot);
    action_groups =
        move_to_first_action(robot, full_actions.front(), action_groups);
    const auto all_actions =
        filter_actions_for_groups(full_actions, action_groups);
    std::cout << "Active action groups:";
    for (const auto& group : action_groups) {
      std::cout << " " << group;
    }
    std::cout << "; effective action shape: (" << all_actions.size() << ", "
              << action_joint_names(action_groups).size() << ")" << std::endl;

    std::size_t cursor = 0;
    while (true) {
      const VlaObservation observation =
          collect_observation(robot, action_groups);
      MockVlaResponse response =
          mock_vla_server(observation, all_actions, cursor, action_groups);
      cursor = response.next_cursor;
      if (response.actions.empty()) {
        std::cout << "Mock VLA server has no more actions." << std::endl;
        break;
      }
      execute_action_chunk(robot, response.actions, action_groups);
    }

    std::cout << "VLA execution example finished successfully." << std::endl;
    shutdown_robot(robot);
    initialized = false;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    if (initialized) {
      shutdown_robot(robot);
    }
    return 1;
  }
  return 0;
}
