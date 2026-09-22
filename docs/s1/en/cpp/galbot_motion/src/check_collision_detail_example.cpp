#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_collision_details(GalbotMotion& motion, const std::string& title, MotionStatus status,
                             const std::vector<CollisionInfo>& collision_infos) {
  std::cout << title << " status: " << motion.status_to_string(status) << std::endl;
  if (status != MotionStatus::SUCCESS) {
    return;
  }
  if (collision_infos.empty()) {
    std::cout << "  no collision pair returned" << std::endl;
    return;
  }
  for (size_t i = 0; i < collision_infos.size(); ++i) {
    const auto& info = collision_infos[i];
    std::cout << "  - pair [" << i << "]: " << (info.is_collision ? "COLLISION" : "NO COLLISION") << ", "
              << info.link1 << " <-> " << info.link2 << ", distance: " << info.distance
              << ", type: " << info.collision_type << std::endl;
  }
}

int main() {
  auto& motion = GalbotMotion::get_instance(MachineType::S1);
  auto& robot = GalbotRobot::get_instance(MachineType::S1);
  auto& navigation = GalbotNavigation::get_instance(MachineType::S1);

  if (!motion.init()) {
    std::cerr << "GalbotMotion init FAILED" << std::endl;
    return -1;
  }
  if (!robot.init()) {
    std::cerr << "GalbotRobot init FAILED" << std::endl;
    return -1;
  }
  if (!navigation.init()) {
    std::cerr << "GalbotNavigation init FAILED" << std::endl;
    return -1;
  }

  auto params = std::make_shared<Parameter>();
  params->set_timeout(5.0);

  try {
    std::cout << ">> Detailed collision check: current robot state" << std::endl;
    auto current_result = motion.check_collision_detail(std::vector<std::shared_ptr<RobotStates>>{}, false, false, params);
    print_collision_details(motion, "current robot state", std::get<0>(current_result), std::get<1>(current_result));

    std::unordered_map<std::string, std::vector<double>> chain_joints = {
        {"torso", {1.1}},
        {"head", {0.0000, -0.26}},
        {"left_arm", {-0.47, -0.94, -0.54, -1.92, 0.2, 0.0, 0.0}},
        {"right_arm", {0.47, 0.94, 0.54, 1.92, -0.2, 0.0, 0.0}},
    };
    std::vector<double> whole_body_joint;
    for (const auto& key : {"torso", "head", "left_arm", "right_arm"}) {
      whole_body_joint.insert(whole_body_joint.end(), chain_joints[key].begin(), chain_joints[key].end());
    }

    std::vector<std::shared_ptr<RobotStates>> robot_states;
    auto whole_body_state = std::make_shared<RobotStates>();
    whole_body_state->whole_body_joint = whole_body_joint;
    whole_body_state->base_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    robot_states.push_back(whole_body_state);

    auto left_arm_state = std::make_shared<JointStates>();
    left_arm_state->chain_name = "left_arm";
    left_arm_state->joint_positions = {1.99995, -1.60004, 0.599905, -1.69994, 0.0, -0.799924, 0.0};
    robot_states.push_back(left_arm_state);

    std::cout << ">> Detailed collision check: explicit robot states" << std::endl;
    auto explicit_result = motion.check_collision_detail(robot_states, false, true, params);
    print_collision_details(motion, "explicit robot states", std::get<0>(explicit_result), std::get<1>(explicit_result));
  } catch (const std::exception& e) {
    std::cerr << "ERROR: check_collision_detail exception: " << e.what() << std::endl;
  }

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return 0;
}
