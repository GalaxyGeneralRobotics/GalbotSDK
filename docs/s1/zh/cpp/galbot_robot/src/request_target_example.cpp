/**
 * @file request_target_example.cpp
 * @brief S1 RequestTarget menu example for SDK mirror SingoriXTarget.
 *
 * This example shows how to construct `SingoriXTarget` directly at the SDK layer
 * and send it to the low-level WBCS through `RequestTarget()`.
 *
 * 1. Joint-space commands are written into `target_group_trajectory_map`
 *    - Typical for torso / head / arm style joint-space control
 *    - Each group corresponds to one `TargetGroupTrajectory`
 *    - Each trajectory point is described by `GroupCommand + JointCommand`
 *
 * 2. Chassis pose / twist style task-space commands are written into
 *    `target_task_trajectory_map`
 *    - On S1 this corresponds to the `swerve_chassis` task
 *    - Each task corresponds to one `TargetTaskTrajectory`
 *    - Each trajectory point is described by `TaskCommand + FrameTriad`
 *
 * 3. One `SingoriXTarget` can contain both joint trajectory and task trajectory
 *    - This supports one-shot dispatch for whole-body / mixed control
 *
 * 4. The current SDK does not automatically switch the chassis controller when
 *    calling `PublishTarget()` / `RequestTarget()`
 *    - Therefore this example explicitly calls
 *      `switch_controller(S1ControllerName::SWERVE_CHASSIS_POSE_CTRL)` or
 *      `switch_controller(S1ControllerName::SWERVE_CHASSIS_TWIST_CTRL)` before
 *      chassis pose / twist / mixed scenes
 *
 * 5. For the base twist scene, this example automatically sends a zero-twist
 *    target after the configured duration to stop the chassis.
 */

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {

constexpr const char* kChassisTaskName = S1JointGroup::SWERVE_CHASSIS;
constexpr const char* kChassisSubtaskPose = "swerve_chassis_pose";
constexpr const char* kChassisSubtaskTwist = "swerve_chassis_twist";

int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

Quaternion yaw_to_quaternion(double yaw) {
  Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

SingoriXTarget make_empty_target() {
  SingoriXTarget target;
  target.header.timestamp_ns = now_ns();
  target.header.frame_id = "base_link";
  return target;
}

TargetConfig make_group_target_config() {
  TargetConfig config;
  config.target_data = TARGET_DATA_DEFAULT;
  config.target_type = TARGET_TYPE_PROVERRIDE;
  config.target_sampling = TargetSampling::TARGET_SAMPLING_DEFAULT;
  config.target_priority = 1;
  return config;
}

TargetConfig make_pose_target_config() {
  TargetConfig config;
  config.target_data = TARGET_DATA_FRAME_POSE;
  config.target_type = TARGET_TYPE_OVERRIDE;
  config.target_sampling = TargetSampling::TARGET_SAMPLING_LINEAR_INTERPOLATE;
  config.target_priority = 1;
  return config;
}

TargetConfig make_twist_target_config() {
  TargetConfig config;
  config.target_data = TARGET_DATA_FRAME_TWIST;
  config.target_type = TARGET_TYPE_OVERRIDE;
  config.target_sampling = TargetSampling::TARGET_SAMPLING_DIRECT_PASS;
  config.target_priority = 1;
  return config;
}

SingoriXTarget build_joint_target(const std::string& group_name,
                                  const std::vector<std::string>& joint_names,
                                  const std::vector<double>& positions,
                                  double time_from_start_s) {
  SingoriXTarget target = make_empty_target();
  auto& group_traj = target.target_group_trajectory_map[group_name];
  group_traj.target_config = make_group_target_config();
  group_traj.joint_names = joint_names;

  GroupCommand command;
  command.time_from_start_s = time_from_start_s;
  command.joint_commands.resize(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i) {
    command.joint_commands[i].position = positions[i];
    command.joint_commands[i].velocity = 0.0;
    command.joint_commands[i].acceleration = 0.0;
    command.joint_commands[i].effort = 0.0;
  }
  group_traj.group_commands.push_back(command);
  return target;
}

SingoriXTarget build_swerve_chassis_pose_target(double x,
                                                double y,
                                                double yaw,
                                                double time_from_start_s,
                                                const std::string& frame_id = "rel(0)",
                                                const std::string& reference_frame_id = "odom") {
  SingoriXTarget target = make_empty_target();
  auto& task_traj = target.target_task_trajectory_map[kChassisTaskName];
  task_traj.target_config = make_pose_target_config();
  task_traj.group_names = {S1JointGroup::SWERVE_CHASSIS};
  task_traj.subtask_names = {std::string(kChassisSubtaskPose) + "_" + std::to_string(now_ns())};

  TaskCommand command;
  command.time_from_start_s = time_from_start_s;

  FrameTriad triad;
  triad.header.timestamp_ns = now_ns();
  triad.header.frame_id = frame_id;
  triad.body_frame_id = "base_link";
  triad.reference_frame_id = reference_frame_id;
  triad.pose = Pose();
  triad.pose->position.x = x;
  triad.pose->position.y = y;
  triad.pose->position.z = 0.0;
  triad.pose->orientation = yaw_to_quaternion(yaw);
  command.subtask_commands.push_back(triad);

  task_traj.task_commands.push_back(command);
  return target;
}

SingoriXTarget build_swerve_chassis_twist_target(double vx,
                                                 double vy,
                                                 double wz,
                                                 double time_from_start_s) {
  SingoriXTarget target = make_empty_target();
  auto& task_traj = target.target_task_trajectory_map[kChassisTaskName];
  task_traj.target_config = make_twist_target_config();
  task_traj.group_names = {S1JointGroup::SWERVE_CHASSIS};
  task_traj.subtask_names = {std::string(kChassisSubtaskTwist) + "_" + std::to_string(now_ns())};

  TaskCommand command;
  command.time_from_start_s = time_from_start_s;

  FrameTriad triad;
  triad.header.timestamp_ns = now_ns();
  triad.header.frame_id = "base_link";
  triad.body_frame_id = "base_link";
  triad.reference_frame_id = "base_link";
  triad.twist = Twist();
  triad.twist->linear = Vector3{vx, vy, 0.0};
  triad.twist->angular = Vector3{0.0, 0.0, wz};
  command.subtask_commands.push_back(triad);

  task_traj.task_commands.push_back(command);
  return target;
}

SingoriXTarget build_stop_twist_target() {
  return build_swerve_chassis_twist_target(0.0, 0.0, 0.0, 0.1);
}

SingoriXTarget merge_targets(const std::vector<SingoriXTarget>& targets) {
  SingoriXTarget merged = make_empty_target();
  for (const auto& target : targets) {
    for (const auto& [group_name, group_traj] : target.target_group_trajectory_map) {
      merged.target_group_trajectory_map[group_name] = group_traj;
    }
    for (const auto& [task_name, task_traj] : target.target_task_trajectory_map) {
      merged.target_task_trajectory_map[task_name] = task_traj;
    }
  }
  return merged;
}

void print_menu() {
  std::cout << "\nAvailable commands:\n"
            << "  joint        - request a joint-only torso target\n"
            << "  base_pose    - request a swerve chassis pose target\n"
            << "  base_twist   - request a swerve chassis twist target with auto stop\n"
            << "  mixed_pose   - request torso + swerve chassis pose in one target\n"
            << "  mixed_twist  - request torso + swerve chassis twist in one target\n"
            << "  quit         - exit example\n"
            << std::endl;
}

void print_error_info(const std::string& scene_name, const std::shared_ptr<ErrorInfo>& error_info) {
  if (error_info == nullptr) {
    std::cout << scene_name << ": RequestTarget returned nullptr" << std::endl;
    return;
  }
  if (error_info->error_vec.empty()) {
    std::cout << scene_name << ": RequestTarget success, service returned no errors" << std::endl;
    return;
  }

  std::cout << scene_name << ": RequestTarget returned " << error_info->error_vec.size() << " error entries:"
            << std::endl;
  for (const auto& error : error_info->error_vec) {
    std::cout << "  component=" << error.commpent << ", code=" << error.error_code
              << ", description=" << error.description << std::endl;
  }
}

ControlStatus ensure_controller(GalbotRobot& robot, const std::string& controller_name) {
  const auto status = robot.switch_controller(controller_name);
  std::cout << "switch_controller(" << controller_name << "): ";
  std::cout << (status == ControlStatus::SUCCESS ? "SUCCESS" : "FAILED") << std::endl;
  return status;
}

void run_twist_scene(GalbotRobot& robot,
                     const std::string& scene_name,
                     const SingoriXTarget& target,
                     double twist_duration_s) {
  std::cout << scene_name << ": start moving for " << twist_duration_s << " seconds" << std::endl;
  print_error_info(scene_name, robot.RequestTarget(target));
  std::this_thread::sleep_for(std::chrono::duration<double>(twist_duration_s));
  std::cout << scene_name << ": send stop twist target" << std::endl;
  print_error_info(scene_name + "_stop", robot.RequestTarget(build_stop_twist_target()));
}

}  // namespace

int main() {
  auto& robot = GalbotRobot::get_instance(MachineType::S1);
  if (!robot.init()) {
    std::cerr << "robot init failed" << std::endl;
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::seconds(2));

  const auto torso_joint_names = robot.get_joint_names(true, {S1JointGroup::TORSO});
  if (torso_joint_names.empty()) {
    std::cerr << "failed to fetch active torso joints" << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return -1;
  }

  const std::vector<std::string> torso_single_joint = {torso_joint_names.front()};
  constexpr double kJointTimeS = 3.0;
  constexpr double kPoseTimeS = 4.0;
  constexpr double kTwistCommandTimeS = 0.2;
  constexpr double kTwistDurationS = 2.0;
  constexpr double kTorsoJt = 0.60; // torso target height,  meter

  print_menu();

  std::string command;
  while (true) {
    std::cout << "Enter command: ";
    if (!(std::cin >> command)) {
      break;
    }
    if (command == "quit") {
      break;
    }

    if (command == "joint") {
      const auto target = build_joint_target(S1JointGroup::TORSO, torso_single_joint, {kTorsoJt}, kJointTimeS);
      print_error_info("joint", robot.RequestTarget(target));
      continue;
    }

    if (command == "base_pose") {
      if (ensure_controller(robot, S1ControllerName::SWERVE_CHASSIS_POSE_CTRL) != ControlStatus::SUCCESS) {
        continue;
      }
      const auto target = build_swerve_chassis_pose_target(0.2, 0.0, 0.0, kPoseTimeS);
      print_error_info("base_pose", robot.RequestTarget(target));
      continue;
    }

    if (command == "base_twist") {
      if (ensure_controller(robot, S1ControllerName::SWERVE_CHASSIS_TWIST_CTRL) != ControlStatus::SUCCESS) {
        continue;
      }
      const auto target = build_swerve_chassis_twist_target(0.05, 0.0, 0.0, kTwistCommandTimeS);
      run_twist_scene(robot, "base_twist", target, kTwistDurationS);
      continue;
    }

    if (command == "mixed_pose") {
      if (ensure_controller(robot, S1ControllerName::SWERVE_CHASSIS_POSE_CTRL) != ControlStatus::SUCCESS) {
        continue;
      }
      const auto target = merge_targets({
          build_joint_target(S1JointGroup::TORSO, torso_single_joint, {kTorsoJt}, kJointTimeS),
          build_swerve_chassis_pose_target(0.1, 0.0, 0.0, kPoseTimeS),
      });
      print_error_info("mixed_pose", robot.RequestTarget(target));
      continue;
    }

    if (command == "mixed_twist") {
      if (ensure_controller(robot, S1ControllerName::SWERVE_CHASSIS_TWIST_CTRL) != ControlStatus::SUCCESS) {
        continue;
      }
      const auto target = merge_targets({
          build_joint_target(S1JointGroup::TORSO, torso_single_joint, {kTorsoJt}, kJointTimeS),
          build_swerve_chassis_twist_target(0.05, 0.0, 0.0, kTwistCommandTimeS),
      });
      run_twist_scene(robot, "mixed_twist", target, kTwistDurationS);
      continue;
    }

    std::cout << "Unknown command: " << command << std::endl;
    print_menu();
  }

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  return 0;
}
