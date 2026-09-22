#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

// Helper print function
void print_pose(const std::string& label, const std::tuple<MotionStatus, std::vector<double>>& res,
                GalbotMotion& planner) {
  std::cout << "[" << label << "] Status: " << planner.status_to_string(std::get<0>(res)) << std::endl;

  if (std::get<0>(res) == MotionStatus::SUCCESS) {
    std::cout << "End-effector pose: ";
    for (double v : std::get<1>(res)) {
      std::cout << v << " ";
    }
    std::cout << "\n" << std::endl;
  } else {
    std::cout << "Calculation failed!" << std::endl;
  }
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

  auto& navigation = GalbotNavigation::get_instance(MachineType::G3);
  bool navigation_ready = navigation.init();
  if (navigation_ready) {
    std::cout << "Navigation initialized successfully!" << std::endl;
  } else {
    std::cerr << "Navigation initialization failed! World/map-frame FK calls below will be skipped." << std::endl;
  }

  // Program started, waiting for data
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  std::map<std::string, std::vector<double>> chain_joints = {
      {"leg", {0.4992, 1.4991, 1.0005, 0.0000, -0.0004}},
      {"head", {0.0000, 0.0}},
      {"left_arm", {1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0}},
      {"right_arm", {-1.5, 1.36, 0.45, -1.53, 0.1, 0.42, 0.0}}};

  std::vector<double> whole_body_joint;
  std::vector<std::string> keys = {"leg", "head", "left_arm", "right_arm"};
  for (const auto& key : keys) {
    whole_body_joint.insert(whole_body_joint.end(), chain_joints[key].begin(), chain_joints[key].end());
  }

  std::vector<double> base_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  std::string end_link = "left_arm_end_effector_mount_link";
  std::string reference_frame = "base_link";

  // --- test 1: defaultparameters (current status) ---
  try {
    std::cout << ">> Executing: Basic forward kinematics..." << std::endl;
    auto res1 = planner.forward_kinematics(end_link, reference_frame);
    print_pose("Basic version", res1, planner);
  } catch (const std::exception& e) {
    std::cerr << "❌ Basic version exception: " << e.what() << std::endl;
  }

  // --- test 2: jointstatus + parameters ---
  try {
    std::cout << ">> Executing: Forward kinematics with custom joints..." << std::endl;

    std::unordered_map<std::string, std::vector<double>> custom_joint_state = {{"left_arm", chain_joints["left_arm"]}};
    auto custom_param_ptr = std::make_shared<Parameter>();
    auto res2 = planner.forward_kinematics(end_link, reference_frame, custom_joint_state, custom_param_ptr);

    print_pose("Custom parameters", res2, planner);

    if (navigation_ready) {
      auto res2_world = planner.forward_kinematics(end_link, "world", custom_joint_state, custom_param_ptr);
      print_pose("Custom joints in world frame", res2_world, planner);
    } else {
      std::cout << "[Custom joints in world frame] skipped: navigation not initialized." << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "❌ Custom-parameter exception: " << e.what() << std::endl;
  }

  // --- test 3: RobotStates forward kinematics(current status, planning)---
  try {
    std::cout << ">> Executing: Forward kinematics based on RobotStates..." << std::endl;

    RobotStates current_state = planner.get_robot_states();
    if (current_state.whole_body_joint.empty()) {
      std::cerr << "❌ RobotStates-based: Unable to get current body joint states; ensure WBC/sensors are ready." << std::endl;
    } else {
      auto ref_robot_state_ptr = std::make_shared<RobotStates>(std::move(current_state));
      auto res3 = planner.forward_kinematics_by_state(end_link, ref_robot_state_ptr, reference_frame,
                                                      std::make_shared<Parameter>());
      print_pose("Based on RobotStates", res3, planner);
    }
  } catch (const std::exception& e) {
    std::cerr << "❌ RobotStates-based exception: " << e.what() << std::endl;
  }

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return 0;
}
