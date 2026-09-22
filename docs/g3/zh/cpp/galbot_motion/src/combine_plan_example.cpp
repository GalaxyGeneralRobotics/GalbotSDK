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
  std::vector<PlanRequest> plan_reqs;
  Parameter params;
};

struct StartJointState {
  std::vector<double> leg;
  std::vector<double> head;
  std::vector<double> left_arm;
  std::vector<double> right_arm;
};

const StartJointState kCombinePlanStartState = {
    {0.4992, 1.4991, 1.0005, 0.0, -0.0004},
    {0.0, 0.0},
    {1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0},
    {-1.5, 1.36, 0.45, -1.53, 0.1, 0.42, 0.0},
};

constexpr double kStartMoveSpeedRadS = 0.12;
constexpr double kStartMoveTimeoutS = 30.0;
const std::vector<std::string> kUpperBodyGroups = {"head", "left_arm", "right_arm"};

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

std::vector<double> make_upper_body_start_positions(const StartJointState& start_state) {
  std::vector<double> positions;
  positions.reserve(start_state.head.size() + start_state.left_arm.size() + start_state.right_arm.size());
  positions.insert(positions.end(), start_state.head.begin(), start_state.head.end());
  positions.insert(positions.end(), start_state.left_arm.begin(), start_state.left_arm.end());
  positions.insert(positions.end(), start_state.right_arm.begin(), start_state.right_arm.end());
  return positions;
}

bool move_robot_to_start_state(GalbotRobot& robot) {
  std::cout << "Move robot to combine_plan start state." << std::endl;
  std::cout << "  leg start joints: ";
  print_vector(kCombinePlanStartState.leg);
  std::cout << std::endl;

  // const ControlStatus controller_status = robot.switch_controller(G3ControllerName::LEG_PVT_CTRL);
  // print_api_status("GalbotRobot::switch_controller(leg_pvt_ctrl)", controller_status);
  // if (controller_status != ControlStatus::SUCCESS) {
  //   return false;
  // }
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  const ControlStatus leg_status =
      robot.set_joint_positions(kCombinePlanStartState.leg, {"leg"}, {}, true, kStartMoveSpeedRadS, kStartMoveTimeoutS);
  print_api_status("GalbotRobot::set_joint_positions(leg start)", leg_status);
  if (leg_status != ControlStatus::SUCCESS) {
    return false;
  }

  const std::vector<double> upper_body_start = make_upper_body_start_positions(kCombinePlanStartState);
  std::cout << "  upper-body start joints [head, left_arm, right_arm]: ";
  print_vector(upper_body_start);
  std::cout << std::endl;

  const ControlStatus upper_body_status =
      robot.set_joint_positions(upper_body_start, kUpperBodyGroups, {}, true, kStartMoveSpeedRadS, kStartMoveTimeoutS);
  print_api_status("GalbotRobot::set_joint_positions(head+arms start)", upper_body_status);
  return upper_body_status == ControlStatus::SUCCESS;
}

MotionPlanChainTarget make_joint_target(const std::string& chain, const std::vector<double>& joint_positions,
                                        const std::vector<std::string>& joint_names = {}) {
  MotionPlanChainTarget target;
  target.chain_name = chain;
  target.mode = MotionPlanTargetMode::kJoint;
  target.joint.chain_name = chain;
  target.joint.joint_positions = joint_positions;
  target.joint.joint_names = joint_names;
  return target;
}

MotionPlanChainTarget make_cartesian_target(const std::string& chain, const std::vector<double>& pose,
                                            const std::vector<std::string>& assist_chains = {}) {
  MotionPlanChainTarget target;
  target.chain_name = chain;
  target.mode = MotionPlanTargetMode::kCartesian;
  target.cart.chain_name = chain;
  target.cart.frame_id = "EndEffector";
  target.cart.reference_frame = "base_link";
  target.cart.pose = Pose(pose);
  target.cart.assist_chains.insert(assist_chains.begin(), assist_chains.end());
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

ExampleInputs make_example_inputs(const CurrentRobotState& current) {
  ExampleInputs inputs;

  const std::vector<std::string> left_arm_joint_names = current.joint_names.at("left_arm");
  const std::vector<std::string> right_arm_joint_names = current.joint_names.at("right_arm");

  PlanRequest request0;
  request0.plan_type = MotionPlanType::MOTION_PLAN;
  request0.enforce_pass = true;
  request0.target = {
      {make_joint_target("left_arm", {1.2, -1.0, -0.2, 1.4, 0.1, -0.3, 0.0},
                         left_arm_joint_names),
       make_cartesian_target("right_arm", {0.26, -0.2324, 0.76428, -0.0453, 0.0152, -0.068, 0.996}, {"torso"})},
      {make_joint_target("left_arm", {1.46326, -1.36281, -0.44721, 1.50544, -0.10183, -0.40826, 0.00287},
                         left_arm_joint_names)},
      {make_joint_target("left_arm", {1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0},
                         left_arm_joint_names),
       make_cartesian_target("right_arm", {0.26, -0.2324, 0.76428, -0.0453, 0.0152, -0.068, 0.996}, {"torso"})},
  };

  PlanRequest request1;
  request1.plan_type = MotionPlanType::MOVE_LINE;
  request1.enforce_pass = true;
  request1.target = {
      {make_cartesian_target("left_arm",
                             {0.302504, 0.232819, 0.819552, 0.0413871, 0.0393282, -0.0675238, 0.996083},
                             {"torso"}),
       make_cartesian_target("right_arm",
                             {0.304916, -0.240799, 0.822258, -0.0411131, 0.0379809, 0.0507316, 0.997143},
                             {"torso"})},
      {make_cartesian_target("left_arm",
                             {0.272504, 0.232819, 0.819552, 0.0413871, 0.0393282, -0.0675238, 0.996083},
                             {"torso"})},
      {make_cartesian_target("right_arm",
                             {0.274916, -0.240799, 0.822258, -0.0411131, 0.0379809, 0.0507316, 0.997143},
                             {"torso"})},
  };

  PlanRequest request2;
  request2.plan_type = MotionPlanType::MOTION_PLAN;
  request2.enforce_pass = true;
  request2.target = {
      {make_joint_target("left_arm", {1.2, -1.0, -0.2, 1.4, 0.1, -0.3, 0.0},
                         left_arm_joint_names)},
      {make_joint_target("right_arm", {-1.2, 1.0, 0.2, -1.4, -0.1, 0.3, 0.0},
                         right_arm_joint_names)},
      {make_joint_target("torso", {0.0, 0.0}, {"leg_joint4", "leg_joint5"})},
  };

  inputs.plan_reqs = {request0, request1, request2};

  inputs.params.set_direct_execute(true);
  inputs.params.set_blocking(true);
  inputs.params.set_timeout(120.0);
  inputs.params.set_check_collision(true);
  inputs.params.set_reference_frame("base_link");
  inputs.params.set_actuate("with_torso");

  return inputs;
}

std::string plan_type_to_string(MotionPlanType plan_type) {
  switch (plan_type) {
    case MotionPlanType::MOTION_PLAN:
      return "MOTION_PLAN";
    case MotionPlanType::TRAJ_PLAN:
      return "TRAJ_PLAN";
    case MotionPlanType::MOVE_LINE:
      return "MOVE_LINE";
    default:
      return "UNKNOWN";
  }
}

void print_joint_names(const std::vector<std::string>& joint_names) {
  if (joint_names.empty()) {
    return;
  }
  std::cout << " joint_names=[";
  for (size_t i = 0; i < joint_names.size(); ++i) {
    std::cout << joint_names[i];
    if (i + 1 < joint_names.size()) {
      std::cout << ", ";
    }
  }
  std::cout << "]";
}

void print_plan_requests(const std::vector<PlanRequest>& plan_requests) {
  std::cout << "combine_plan plan requests: " << plan_requests.size() << std::endl;
  for (size_t request_index = 0; request_index < plan_requests.size(); ++request_index) {
    const auto& request = plan_requests[request_index];
    std::cout << "  request[" << request_index << "] type=" << plan_type_to_string(request.plan_type)
              << " waypoints=" << request.target.size() << std::endl;
    for (size_t waypoint_index = 0; waypoint_index < request.target.size(); ++waypoint_index) {
      std::cout << "    waypoint[" << waypoint_index << "] targets: " << request.target[waypoint_index].size()
                << std::endl;
      for (const auto& target : request.target[waypoint_index]) {
        if (target.mode == MotionPlanTargetMode::kJoint) {
          std::cout << "      " << target.chain_name << " JOINT q=";
          print_vector(target.joint.joint_positions);
          print_joint_names(target.joint.joint_names);
          std::cout << std::endl;
        } else {
          std::cout << "      " << target.chain_name << " CART pose=";
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
  auto& motion = GalbotMotion::get_instance(MachineType::G3);
  print_no_status_api_completed("GalbotMotion::get_instance()");
  auto& robot = GalbotRobot::get_instance(MachineType::G3);
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

  if (!move_robot_to_start_state(robot)) {
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

  const ExampleInputs inputs = make_example_inputs(current);
  print_plan_requests(inputs.plan_reqs);

  std::cout << "Immediate execution is enabled; plan requests are loaded from combine_file_example_plan_reqs.json reference values."
            << std::endl;

  auto result = motion.combine_plan(inputs.plan_reqs, inputs.params);
  print_traj_result("combine_plan", result, motion);

  shutdown_robot(robot);
  return std::get<0>(result) == MotionStatus::SUCCESS ? 0 : 1;
}
