#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

double g_target_time = 10;
double g_start_time = 10;

std::string trajectory_status_to_string(TrajectoryControlStatus status) {
  switch (status) {
  case TrajectoryControlStatus::INVALID_INPUT:
    return "INVALID_INPUT";
  case TrajectoryControlStatus::RUNNING:
    return "RUNNING";
  case TrajectoryControlStatus::COMPLETED:
    return "COMPLETED";
  case TrajectoryControlStatus::STOPPED_UNREACHED:
    return "STOPPED_UNREACHED";
  case TrajectoryControlStatus::ERROR:
    return "ERROR";
  case TrajectoryControlStatus::DATA_FETCH_FAILED:
    return "DATA_FETCH_FAILED";
  case TrajectoryControlStatus::STATUS_NUM:
    return "STATUS_NUM";
  default:
    return "UNKNOWN_STATUS";
  }
}

void wait_for_traj_reached(const std::vector<std::string> &joint_groups) {
    std::vector<TrajectoryControlStatus> traj_exec_states;
    int count = 0;
    bool all_reached = false;
    while (count++ < 150) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        all_reached = true;
        traj_exec_states = GalbotRobot::get_instance(MachineType::G3)
                            .check_trajectory_execution_status(joint_groups);
        if (traj_exec_states.size() != joint_groups.size()) {
        std::cout << "traj_exec_states size != joint_groups size" << std::endl;
        }
        for (int i = 0; i < joint_groups.size(); ++i) {
        std::cout << joint_groups[i] << " exec state is "
                    << trajectory_status_to_string(traj_exec_states[i])
                    << std::endl;
        if (traj_exec_states[i] != TrajectoryControlStatus::COMPLETED) {
            all_reached = false;
        }
        }

        if (all_reached) {
            std::cout << "all reached" << std::endl;
            break;
        }
    }
    for (const auto &status : traj_exec_states) {
        std::cout << "done reached state is " << trajectory_status_to_string(status)
                << std::endl;
    }
}

std::vector<TrajectoryPoint>
generate_target_trajectory(int32_t joint_size, double ampl = 0.2,
                           double cycle = 10) {
  double amplitude = -ampl;
  double frequency = 1.0 / cycle;
  double phase = -M_PI / 2;
  double offset = amplitude;
  double dt = 0.004;
  int step = g_target_time / dt;

  std::vector<TrajectoryPoint> trajectory_data_vec;
  trajectory_data_vec.resize(step + 1);
  // Create a RobotCommand trajectory
  for (int i = 0; i <= step; ++i) {
    double t = i * dt;
    trajectory_data_vec[i].time_from_start_second = g_start_time + t;
    trajectory_data_vec[i].joint_command_vec.resize(joint_size);
    // Joint command
    for (int j = 0; j < joint_size; ++j) {
      trajectory_data_vec[i].joint_command_vec[j].position =
          offset + amplitude * std::sin(2 * M_PI * frequency * t + phase);
      trajectory_data_vec[i].joint_command_vec[j].velocity =
          amplitude * 2 * M_PI * frequency *
          std::cos(2 * M_PI * frequency * t + phase);
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

    // Execute joint trajectory
    Trajectory trajectory;
    // Enter joint group name to control, including ["leg", "head", "left_arm", "right_arm", "left_gripper", "right_gripper"]
    trajectory.joint_groups = {"head"};
    // Fill this field to control specific joint angles, which will override joint_groups if provided
    trajectory.joint_names = {};
    // Generate trajectory
    trajectory.points = generate_target_trajectory(2);
    // Whether to block until trajectory execution completes; when false, you can use
    bool is_traj_block = false;

    // Wait for trajectory execution to complete; this function wraps check_trajectory_execution_status to check trajectory execution status
    wait_for_traj_reached(trajectory.joint_groups);

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
