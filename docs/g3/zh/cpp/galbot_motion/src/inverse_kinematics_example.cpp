#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

// Helper function: print inverse kinematics result
void print_ik_result(const std::string& label,
                     const std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<double>>>& res,
                     GalbotMotion& planner) {
  auto status = std::get<0>(res);
  auto joint_map = std::get<1>(res);

  std::cout << "[" << label << "] Status feedback: " << planner.status_to_string(status) << std::endl;

  if (status == MotionStatus::SUCCESS) {
    std::cout << "✅ IK computation succeeded! Joint angles obtained:" << std::endl;
    for (const auto& [name, joints] : joint_map) {
      std::cout << "  - Chain [" << name << "]: ";
      for (double v : joints)
        std::cout << v << " ";
      std::cout << std::endl;
    }
  } else {
    std::cout << "❌ inverse kinematics failed, checkinput targetpose." << std::endl;
  }
  std::cout << "---------------------------------------------------" << std::endl;
}

int main() {
  auto& planner = GalbotMotion::get_instance(MachineType::G3);
  auto& robot = GalbotRobot::get_instance(MachineType::G3);

  if (planner.init()) {
    std::cout << "Planner initialized successfully!" << std::endl;
  } else {
    std::cerr << "Planner initialization failed!" << std::endl;
    return -1;
  }

  if (robot.init()) {
    std::cout << "System initialized successfully!" << std::endl;
  } else {
    std::cerr << "System initialization failed!" << std::endl;
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Joint state definition
  std::unordered_map<std::string, std::vector<double>> chain_joints = {
      {"leg", {0.4992, 1.4991, 1.0005, 0.0000, -0.0004}},
      {"head", {0.0000, 0.0}},
      {"left_arm", {1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0}},
      {"right_arm", {-1.5, 1.36, 0.45, -1.53, 0.1, 0.42, 0.0}}};

  // Target pose definition (x, y, z, qx, qy, qz, qw)
  std::unordered_map<std::string, std::vector<double>> chain_pose_baselink = {
      {"left_arm",  { 0.4827, 0.2329, 0.8883, 0.0405, 0.0396, -0.0604, 0.9966 }},
      {"right_arm", { 0.2818, -0.2430, 0.8389, -0.0456, 0.0479, 0.0573, 0.9962 }}}; // pose from fk

  // Whole-body joint vector concatenation (Leg -> Head -> Left Arm -> Right Arm)
  std::vector<double> whole_body_joint;
  std::vector<std::string> key_order = {"leg", "head", "left_arm", "right_arm"};
  for (const auto& key : key_order) {
    whole_body_joint.insert(whole_body_joint.end(), chain_joints[key].begin(), chain_joints[key].end());
  }

  // General configuration
  std::string reference_frame = "base_link";
  std::string target_frame = "EndEffector";
  std::string target_chain = "left_arm";
  auto params = std::make_shared<Parameter>();

  // Scenario 1: Single-chain inverse kinematics
  try {
    std::cout << ">> Running Scenario 1: Single-chain IK test..." << std::endl;
    std::vector<std::string> one_chain = {target_chain};

    auto res = planner.inverse_kinematics(chain_pose_baselink[target_chain],  // 1. target_pose
                                          one_chain                           // 2. chain_names
                                          // target_frame,                      // 3. target_frame
                                          // reference_frame,                   // 4. reference_frame
                                          // {},                                // 5. initial_joint_positions (empty)
                                          // false,                             // 6. enable_collision_check (bool)
                                          // params                             // 7. params (shared_ptr)
    );
    print_ik_result("Single-chain inverse kinematics", res, planner);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
  } catch (const std::exception& e) {
    std::cerr << "Scenario 1 exception: " << e.what() << std::endl;
  }

  // Scenario 2: Arm chain + torso inverse kinematics
  try {
    std::cout << ">> Running Scenario 2: Arm chain + torso IK test..." << std::endl;
    std::vector<std::string> chain_with_torso = {target_chain, "torso"};

    auto res = planner.inverse_kinematics(chain_pose_baselink[target_chain], chain_with_torso, target_frame,
                                          reference_frame, {}, false, params);
    print_ik_result("Arm + torso inverse kinematics", res, planner);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
  } catch (const std::exception& e) {
    std::cerr << "Scenario 2 exception: " << e.what() << std::endl;
  }

  // Scenario 3: invalid chain combination
  try {
    std::cout << ">> Running Scenario 3: Invalid chain-combination test..." << std::endl;
    std::vector<std::string> error_chains = {target_chain, "torso", "head"};

    auto res = planner.inverse_kinematics(chain_pose_baselink[target_chain], error_chains, target_frame,
                                          reference_frame, {}, false, params);
    print_ik_result("Invalid chain combination detection", res, planner);
  } catch (const std::exception& e) {
    std::cerr << "Scenario 3 exception: " << e.what() << std::endl;
  }

  // Scenario 4: use reference joints (initial_joint_positions can specify chain joints as IK references; unspecified chain joints are filled with whole-body joints)
  try {
    std::cout << ">> Running Scenario 4: IK test with initial reference values..." << std::endl;
    std::vector<std::string> one_chain = {target_chain};

    auto res = planner.inverse_kinematics(chain_pose_baselink[target_chain], one_chain, target_frame, reference_frame,
                                          chain_joints, false, params);
    print_ik_result("Inverse kinematics with reference values", res, planner);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
  } catch (const std::exception& e) {
    std::cerr << "Scenario 4 exception: " << e.what() << std::endl;
  }

  // scenario 5: RobotStates inverse kinematics
  try {
    std::cout << ">> Running Scenario 5: IK test based on RobotStates..." << std::endl;

    // Construct RobotStates smart pointer
    auto ref_state = std::make_shared<RobotStates>();
    ref_state->chain_name = target_chain;
    ref_state->whole_body_joint = whole_body_joint;
    ref_state->base_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    std::vector<std::string> one_chain = {target_chain};

    auto res = planner.inverse_kinematics_by_state(chain_pose_baselink[target_chain],  // 1. target_pose
                                                   one_chain,                          // 2. chain_names
                                                   target_frame,                       // 3. target_frame
                                                   reference_frame,                    // 4. reference_frame
                                                   ref_state,  // 5. reference_robot_states (shared_ptr)
                                                   false,      // 6. enable_collision_check (bool)
                                                   params      // 7. params (shared_ptr)
    );
    print_ik_result("RobotStates inverse kinematics", res, planner);
  } catch (const std::exception& e) {
    std::cerr << "Scenario 5 exception: " << e.what() << std::endl;
  }

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  std::cout << "Resources released." << std::endl;

  return 0;
}
