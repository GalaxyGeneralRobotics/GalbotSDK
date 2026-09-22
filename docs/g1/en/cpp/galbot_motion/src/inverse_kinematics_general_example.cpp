#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {

void print_vector(const std::vector<double>& values) {
  std::cout << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    std::cout << values[i];
    if (i + 1 < values.size()) {
      std::cout << ", ";
    }
  }
  std::cout << "]";
}

struct ExampleInputs {
  std::vector<double> left_arm_target_pose;
  MotionPlanWaypoint target_waypoint;
  std::shared_ptr<RobotStates> reference_state;
  std::shared_ptr<Parameter> params;
};

ExampleInputs make_example_inputs() {
  ExampleInputs inputs;

  // Fixed Cartesian pose for the left_arm IK target.
  // Pose layout: [x, y, z, qx, qy, qz, qw], reference frame: base_link.
  inputs.left_arm_target_pose = {
      0.405795,
      0.510630,
      0.872605,
      -0.0701184,
      -0.0226926,
      0.202097,
      0.976589,
  };

  MotionPlanChainTarget left_arm_target;
  left_arm_target.chain_name = "left_arm";
  left_arm_target.mode = MotionPlanTargetMode::kCartesian;
  left_arm_target.cart.chain_name = "left_arm";
  left_arm_target.cart.frame_id = "left_arm_end_effector_mount_link";
  left_arm_target.cart.reference_frame = "base_link";
  left_arm_target.cart.pose = Pose(inputs.left_arm_target_pose);
  left_arm_target.cart.assist_chains.insert("leg");

  // inverse_kinematics_general(target_waypoint, reference_state, params) input: target_waypoint.
  inputs.target_waypoint = {
      left_arm_target,
  };

  // inverse_kinematics_general input: reference_state. nullptr lets the service choose its default seed.
  inputs.reference_state = nullptr;

  // inverse_kinematics_general input: params.
  inputs.params = std::make_shared<Parameter>();
  inputs.params->set_direct_execute(false);
  inputs.params->set_blocking(true);
  inputs.params->set_timeout(20.0);
  inputs.params->set_check_collision(false);
  inputs.params->set_reference_frame("base_link");

  return inputs;
}

}  // namespace

int main() {
  auto& motion = GalbotMotion::get_instance(MachineType::G1);
  auto& robot = GalbotRobot::get_instance(MachineType::G1);

  // Initialize SDK interfaces directly in the example so users can see the required order.
  if (!motion.init()) {
    std::cerr << "GalbotMotion init FAILED" << std::endl;
    return -1;
  }
  if (!robot.init()) {
    std::cerr << "GalbotRobot init FAILED" << std::endl;
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  const ExampleInputs inputs = make_example_inputs();
  std::cout << "Fixed target left_arm pose [x, y, z, qx, qy, qz, qw]: ";
  print_vector(inputs.left_arm_target_pose);
  std::cout << std::endl;

  // This is the API being demonstrated: solve joint values for the target waypoint.
  auto [status, joint_map] =
      motion.inverse_kinematics_general(inputs.target_waypoint, inputs.reference_state, inputs.params);

  std::cout << "inverse_kinematics_general status: " << motion.status_to_string(status) << std::endl;
  for (const auto& [chain, joints] : joint_map) {
    std::cout << "  " << chain << " joint names: ";
    for (const auto& name : joints.joint_names) {
      std::cout << name << " ";
    }
    std::cout << std::endl << "  " << chain << " joint positions: ";
    print_vector(joints.joint_positions);
    std::cout << std::endl;
  }

  // Clean shutdown releases SDK resources before the process exits.
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  return status == MotionStatus::SUCCESS ? 0 : 1;
}
