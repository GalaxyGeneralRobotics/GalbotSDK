#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {

using TrajMap = std::unordered_map<std::string, std::vector<std::vector<double>>>;
using TrajResult = std::tuple<MotionStatus, TrajMap>;

struct CurrentRobotState {
  std::unordered_map<std::string, std::vector<std::string>> joint_names;
  std::unordered_map<std::string, std::vector<double>> joints;
  std::unordered_map<std::string, std::vector<double>> poses;
};

struct ExampleInputs {
  MotionPlanWaypoints waypoints;
  Parameter params;
};

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

std::vector<double> pose_to_vector(const Pose& pose) {
  return {pose.position.x,    pose.position.y,    pose.position.z,   pose.orientation.x,
          pose.orientation.y, pose.orientation.z, pose.orientation.w};
}

void print_api_status(const std::string& api_name, bool success) {
  std::cout << api_name << " status: " << (success ? "SUCCESS" : "FAILED") << std::endl;
}

void print_api_status(const std::string& api_name, MotionStatus status, GalbotMotion& motion) {
  std::cout << api_name << " status: " << motion.status_to_string(status) << " ("
            << (status == MotionStatus::SUCCESS ? "SUCCESS" : "FAILED") << ")" << std::endl;
}

std::string control_status_to_string(ControlStatus status) {
  switch (status) {
    case ControlStatus::SUCCESS:
      return "SUCCESS";
    case ControlStatus::TIMEOUT:
      return "TIMEOUT";
    case ControlStatus::FAULT:
      return "FAULT";
    case ControlStatus::INVALID_INPUT:
      return "INVALID_INPUT";
    case ControlStatus::INIT_FAILED:
      return "INIT_FAILED";
    case ControlStatus::IN_PROGRESS:
      return "IN_PROGRESS";
    case ControlStatus::STOPPED_UNREACHED:
      return "STOPPED_UNREACHED";
    case ControlStatus::DATA_FETCH_FAILED:
      return "DATA_FETCH_FAILED";
    case ControlStatus::PUBLISH_FAIL:
      return "PUBLISH_FAIL";
    case ControlStatus::COMM_DISCONNECTED:
      return "COMM_DISCONNECTED";
    default:
      return "UNKNOWN_STATUS";
  }
}

void print_api_status(const std::string& api_name, ControlStatus status) {
  std::cout << api_name << " status: " << control_status_to_string(status) << " ("
            << (status == ControlStatus::SUCCESS ? "SUCCESS" : "FAILED") << ")" << std::endl;
}

void print_no_status_api_completed(const std::string& api_name) {
  std::cout << api_name << " status: no return status; call completed." << std::endl;
}

void shutdown_robot(GalbotRobot& robot) {
  robot.request_shutdown();
  print_no_status_api_completed("request_shutdown()");
  robot.wait_for_shutdown();
  print_no_status_api_completed("wait_for_shutdown()");
  robot.destroy();
  print_no_status_api_completed("destroy()");
}

bool move_robot_to_home_position(GalbotRobot& robot) {
  std::cout << "Move robot to the G1 home position." << std::endl;
  const std::vector<double> joint_positions = {
      0.5, 1.5, 1.0, 0.0, 0.0,
      0.0, 0.0,
      2.0, -1.5, -0.6, -1.7, 0.0, -0.8, 0.0,
      -2.0, 1.5, 0.6, 1.7, 0.0, 0.8, 0.0};
  const std::vector<std::string> joint_group_names = {"leg", "head", "left_arm", "right_arm"};
  const ControlStatus status =
      robot.set_joint_positions(joint_positions, joint_group_names, {}, true, 0.1, 30.0);
  print_api_status("GalbotRobot::set_joint_positions(home)", status);
  return status == ControlStatus::SUCCESS;
}

MotionPlanChainTarget make_cartesian_target(const std::string& chain, const std::vector<double>& pose) {
  MotionPlanChainTarget target;
  target.chain_name = chain;
  target.mode = MotionPlanTargetMode::kCartesian;
  target.cart.chain_name = chain;
  target.cart.frame_id = "EndEffector";
  target.cart.reference_frame = "base_link";
  target.cart.pose = Pose(pose);
  target.cart.assist_chains.insert("torso");
  return target;
}

bool capture_current_joints(GalbotMotion& motion, GalbotRobot& robot, const std::vector<std::string>& chains,
                            CurrentRobotState& current) {
  for (const auto& chain : chains) {
    if (current.joints.find(chain) != current.joints.end()) {
      continue;
    }

    std::vector<std::string> joint_names = motion.get_chain_joint_names(chain);
    print_api_status("get_chain_joint_names(" + chain + ")", !joint_names.empty());
    if (joint_names.empty()) {
      std::cerr << "Failed to get joint names for chain: " << chain << std::endl;
      return false;
    }

    std::vector<double> joint_positions = robot.get_joint_positions({}, joint_names);
    const bool joint_positions_success = joint_positions.size() == joint_names.size();
    print_api_status("GalbotRobot::get_joint_positions(" + chain + ")", joint_positions_success);
    if (!joint_positions_success) {
      std::cerr << "Failed to get current joint positions for chain: " << chain << std::endl;
      return false;
    }

    current.joint_names[chain] = joint_names;
    current.joints[chain] = joint_positions;

    std::cout << "Current " << chain << " joints: ";
    print_vector(joint_positions);
    std::cout << std::endl;
  }
  return true;
}

bool capture_current_pose(GalbotMotion& motion, GalbotRobot& robot, const std::string& chain,
                          CurrentRobotState& current) {
  if (!capture_current_joints(motion, robot, {chain}, current)) {
    return false;
  }

  auto [status, pose] = motion.get_end_effector_pose_on_chain(chain, "EndEffector", "base_link");
  print_api_status("get_end_effector_pose_on_chain(" + chain + ")", status, motion);
  if (status != MotionStatus::SUCCESS || pose.size() != 7) {
    std::cerr << "Failed to get current pose for chain: " << chain << std::endl;
    return false;
  }
  current.poses[chain] = pose;
  std::cout << "Current " << chain << " pose [x, y, z, qx, qy, qz, qw]: ";
  print_vector(pose);
  std::cout << std::endl;
  return true;
}

ExampleInputs make_example_inputs() {
  ExampleInputs inputs;

  inputs.waypoints = {
      {make_cartesian_target("left_arm", {0.42666, 0.33435, 0.73569, 0.0, 0.0, 0.0, 1.0}),
       make_cartesian_target("right_arm", {0.422666, -0.33435, 0.73569, 0.0, 0.0, 0.0, 1.0})},
      {make_cartesian_target("left_arm", {0.42666, 0.33435, 1.03569, 0.0, 0.0, 0.0, 1.0}),
       make_cartesian_target("right_arm", {0.422666, -0.33435, 1.03569, 0.0, 0.0, 0.0, 1.0})},
      {make_cartesian_target("left_arm", {0.12666, 0.33435, 1.03569, 0.0, 0.0, 0.0, 1.0}),
       make_cartesian_target("right_arm", {0.12666, -0.33435, 1.03569, 0.0, 0.0, 0.0, 1.0})},
      {make_cartesian_target("left_arm", {0.12666, 0.33435, 0.73569, 0.0, 0.0, 0.0, 1.0}),
       make_cartesian_target("right_arm", {0.12666, -0.33435, 0.73569, 0.0, 0.0, 0.0, 1.0})},
  };

  inputs.params.set_direct_execute(true);
  inputs.params.set_blocking(true);
  inputs.params.set_timeout(120.0);
  inputs.params.set_check_collision(false);
  inputs.params.set_reference_frame("base_link");
  inputs.params.set_actuate("with_torso");

  return inputs;
}

void print_waypoints(const MotionPlanWaypoints& waypoints) {
  std::cout << "move_line waypoints: " << waypoints.size() << std::endl;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    std::cout << "  waypoint[" << i << "] targets: " << waypoints[i].size() << std::endl;
    for (const auto& target : waypoints[i]) {
      std::cout << "    " << target.chain_name << " CART pose=";
      print_vector(pose_to_vector(target.cart.pose));
      std::cout << " frame_id=" << target.cart.frame_id
                << " reference_frame=" << target.cart.reference_frame;
      if (!target.cart.assist_chains.empty()) {
        std::cout << " assist_chains=[";
        size_t assist_chain_index = 0;
        for (const auto& assist_chain : target.cart.assist_chains) {
          std::cout << assist_chain;
          if (++assist_chain_index < target.cart.assist_chains.size()) {
            std::cout << ", ";
          }
        }
        std::cout << "]";
      }
      std::cout << std::endl;
    }
  }
}

void print_traj_result(const std::string& label, const TrajResult& result, GalbotMotion& motion) {
  const auto status = std::get<0>(result);
  const auto& traj = std::get<1>(result);
  print_api_status(label, status, motion);
  if (status != MotionStatus::SUCCESS) {
    return;
  }
  if (traj.empty()) {
    std::cout << "Trajectory map is empty." << std::endl;
    return;
  }
  for (const auto& [chain, points] : traj) {
    std::cout << "  " << chain << " trajectory points: " << points.size() << std::endl;
  }
}

}  // namespace

int main() {
  auto& motion = GalbotMotion::get_instance(MachineType::G1);
  print_no_status_api_completed("GalbotMotion::get_instance()");
  auto& robot = GalbotRobot::get_instance(MachineType::G1);
  print_no_status_api_completed("GalbotRobot::get_instance()");

  const bool motion_init_success = motion.init();
  print_api_status("GalbotMotion::init()", motion_init_success);
  if (!motion_init_success) {
    std::cerr << "GalbotMotion init FAILED" << std::endl;
    return -1;
  }
  const bool robot_init_success = robot.init();
  print_api_status("GalbotRobot::init()", robot_init_success);
  if (!robot_init_success) {
    std::cerr << "GalbotRobot init FAILED" << std::endl;
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  if (!move_robot_to_home_position(robot)) {
    shutdown_robot(robot);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  CurrentRobotState current;
  if (!capture_current_pose(motion, robot, "left_arm", current) ||
      !capture_current_pose(motion, robot, "right_arm", current)) {
    shutdown_robot(robot);
    return -1;
  }

  const ExampleInputs inputs = make_example_inputs();
  print_waypoints(inputs.waypoints);

  std::cout << "Immediate execution is enabled; waypoints are loaded from move_line_targets.json reference values."
            << std::endl;

  auto result = motion.move_line(inputs.waypoints, inputs.params);
  print_traj_result("move_line", result, motion);

  shutdown_robot(robot);
  return std::get<0>(result) == MotionStatus::SUCCESS ? 0 : 1;
}
