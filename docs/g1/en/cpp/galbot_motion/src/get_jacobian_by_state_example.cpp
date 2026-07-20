#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_vector(const std::vector<double>& values) {
  std::cout << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      std::cout << ", ";
    }
    std::cout << values[i];
  }
  std::cout << "]";
}

void print_robot_states(const std::shared_ptr<RobotStates>& robot_states) {
  if (!robot_states) {
    std::cout << "nullptr (use current complete robot state)" << std::endl;
    return;
  }

  std::cout << "RobotStates" << std::endl;
  std::cout << "    whole_body_joint[" << robot_states->whole_body_joint.size() << "]: ";
  print_vector(robot_states->whole_body_joint);
  std::cout << std::endl;
  std::cout << "    base_state[" << robot_states->base_state.size() << "]: ";
  print_vector(robot_states->base_state);
  std::cout << std::endl;
}

void print_get_jacobian_by_state_params(const std::string& chain_name,
                                        const std::string& target_frame,
                                        const std::string& reference_frame,
                                        const std::shared_ptr<RobotStates>& reference_robot_states) {
  std::cout << "get_jacobian_by_state parameters:" << std::endl;
  std::cout << "  chain_name: " << chain_name << std::endl;
  std::cout << "  target_frame: " << target_frame << std::endl;
  std::cout << "  reference_frame: " << reference_frame << std::endl;
  std::cout << "  reference_robot_states: ";
  print_robot_states(reference_robot_states);
}

void print_jacobian(const std::string& label,
                    const std::tuple<MotionStatus, std::vector<std::vector<double>>>& result,
                    GalbotMotion& motion) {
  const auto status = std::get<0>(result);
  const auto& matrix = std::get<1>(result);

  std::cout << "[" << label << "] Status: " << motion.status_to_string(status) << std::endl;
  if (status != MotionStatus::SUCCESS || matrix.empty()) {
    std::cout << "Jacobian computation failed or returned an empty matrix." << std::endl;
    return;
  }

  std::cout << "Jacobian matrix (" << matrix.size() << "x" << matrix[0].size() << "):" << std::endl;
  for (size_t r = 0; r < matrix.size(); ++r) {
    std::cout << "  row " << r << ": ";
    for (double value : matrix[r]) {
      std::cout << std::setw(10) << std::fixed << std::setprecision(5) << value << " ";
    }
    std::cout << std::endl;
  }
}

std::vector<double> make_fixed_whole_body_joint(size_t dof) {
  static const std::vector<double> fixed_values = {
      0.3, 0.8, 0.5, 0.0, 0.0, 0.1, -0.2, 1.2, -0.8, 0.4, -1.0,
      0.2, -0.6, 0.0, -1.2, 0.8, -0.4, 1.0, -0.2, 0.6, 0.0};
  std::vector<double> values;
  values.reserve(dof);
  for (size_t i = 0; i < dof; ++i) {
    values.push_back(fixed_values[i % fixed_values.size()]);
  }
  return values;
}

int main() {
  auto& motion = GalbotMotion::get_instance(MachineType::G1);
  auto& robot = GalbotRobot::get_instance(MachineType::G1);

  if (!motion.init()) {
    std::cerr << "GalbotMotion initialization failed." << std::endl;
    return 1;
  }
  std::cout << "GalbotMotion initialized successfully" << std::endl;

  if (!robot.init()) {
    std::cerr << "GalbotRobot initialization failed." << std::endl;
    return 1;
  }
  std::cout << "GalbotRobot initialized successfully" << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::set<std::string> support_chains = motion.get_support_chains();
  std::cout << "support_chains (" << support_chains.size() << "): [";
  for (auto it = support_chains.begin(); it != support_chains.end(); ++it) {
    if (it != support_chains.begin()) {
      std::cout << ", ";
    }
    std::cout << *it;
  }
  std::cout << "]" << std::endl;
  if (support_chains.empty()) {
    support_chains = {"head", "left_arm", "right_arm", "torso", "leg"};
  }

  const std::string target_frame = "EndEffector";
  const std::string reference_frame = "base_link";

  size_t whole_body_dof = 21;
  try {
    RobotStates current_state = motion.get_robot_states();
    if (!current_state.whole_body_joint.empty()) {
      whole_body_dof = current_state.whole_body_joint.size();
    }
  } catch (const std::exception& e) {
    std::cerr << "get_robot_states exception: " << e.what() << std::endl;
  }

  auto reference_robot_states = std::make_shared<RobotStates>();
  reference_robot_states->whole_body_joint = make_fixed_whole_body_joint(whole_body_dof);
  reference_robot_states->base_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  // Compute the Jacobian for every chain using an explicit RobotStates
  for (const auto& chain_name : support_chains) {
    try {
      std::cout << "=== get_jacobian_by_state (explicit RobotStates): " << chain_name << " ===" << std::endl;
      print_get_jacobian_by_state_params(chain_name, target_frame, reference_frame, reference_robot_states);
      auto result = motion.get_jacobian_by_state(chain_name, target_frame, reference_frame, reference_robot_states);
      print_jacobian(chain_name + " explicit RobotStates", result, motion);
    } catch (const std::exception& e) {
      std::cerr << chain_name << " explicit RobotStates exception: " << e.what() << std::endl;
    }
  }

  // Compute the Jacobian for every chain using nullptr (the current robot state)
  for (const auto& chain_name : support_chains) {
    try {
      std::cout << "=== get_jacobian_by_state (nullptr RobotStates): " << chain_name << " ===" << std::endl;
      std::shared_ptr<RobotStates> null_robot_states = nullptr;
      print_get_jacobian_by_state_params(chain_name, target_frame, reference_frame, null_robot_states);
      auto result = motion.get_jacobian_by_state(chain_name, target_frame, reference_frame, null_robot_states);
      print_jacobian(chain_name + " nullptr RobotStates", result, motion);
    } catch (const std::exception& e) {
      std::cerr << chain_name << " nullptr RobotStates exception: " << e.what() << std::endl;
    }
  }

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return 0;
}
