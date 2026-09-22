#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

std::string control_status_to_string(ControlStatus status) {
  switch (status) {
    case ControlStatus::SUCCESS:
      return "SUCCESS";
    case ControlStatus::INVALID_INPUT:
      return "INVALID_INPUT";
    case ControlStatus::INIT_FAILED:
      return "INIT_FAILED";
    case ControlStatus::COMM_DISCONNECTED:
      return "COMM_DISCONNECTED";
    case ControlStatus::FAULT:
      return "FAULT";
    case ControlStatus::PUBLISH_FAIL:
      return "PUBLISH_FAIL";
    default:
      return "UNKNOWN_STATUS";
  }
}

std::vector<TrajectoryPoint> generate_batch_trajectory(int32_t joint_size, double ampl = 0.2, double cycle = 10,
                                                       int num_points = 10) {
  double amplitude = -ampl;
  double frequency = 1.0 / cycle;
  double phase = -M_PI / 2;
  double offset = amplitude;
  double dt = cycle / num_points;

  std::vector<TrajectoryPoint> trajectory_data_vec;
  trajectory_data_vec.resize(num_points);
  // Create batch trajectory points
  for (int i = 0; i < num_points; ++i) {
    double t = i * dt;
    trajectory_data_vec[i].time_from_start_second = t;
    trajectory_data_vec[i].joint_command_vec.resize(joint_size);
    // Joint command
    for (int j = 0; j < joint_size; ++j) {
      trajectory_data_vec[i].joint_command_vec[j].position =
          offset + amplitude * std::sin(2 * M_PI * frequency * t + phase);
      trajectory_data_vec[i].joint_command_vec[j].velocity =
          amplitude * 2 * M_PI * frequency * std::cos(2 * M_PI * frequency * t + phase);
      // trajectory_data_vec[i].joint_command_vec[j].acceleration = 0.0;
      // trajectory_data_vec[i].joint_command_vec[j].effort = 0.0;
    }
  }

  return trajectory_data_vec;
}

int main() {
  // Get object instance
  auto& robot = GalbotRobot::get_instance(MachineType::G3);

  // Initialize system
  if (robot.init()) {
    std::cout << "System initialized successfully!" << std::endl;
  } else {
    std::cerr << "System initialization failed!" << std::endl;
    return -1;
  }

  // Program started, waiting for data
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // Batch set joint commands
  Trajectory trajectory;
  // Enter joint group names to control, including ["leg", "head", "left_arm", "right_arm", "left_gripper", "right_gripper"]
  trajectory.joint_groups = {"head"};
  // Fill this field to control specific joint angles, which will override joint_groups if provided
  trajectory.joint_names = {};
  // Generate batched trajectory points (joint commands at multiple time points)
  trajectory.points = generate_batch_trajectory(2, 0.2, 10.0, 10);

  // Batch set joint commands (non-blocking, returns immediately)
  ControlStatus status = robot.set_joint_commands_batch(trajectory);
  std::cout << "Batch joint command status: " << control_status_to_string(status) << std::endl;

  if (status == ControlStatus::SUCCESS) {
    std::cout << "Batch commands submitted, executing in background (non-blocking)" << std::endl;
  } else {
    std::cerr << "Failed to submit batch commands!" << std::endl;
  }

  // Wait for a while to let the command execute
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Exit system and release SDK resources
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return 0;
}
