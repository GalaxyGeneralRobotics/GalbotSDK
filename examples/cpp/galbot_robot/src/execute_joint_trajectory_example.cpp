#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

double g_target_time = 10;
double g_start_time = 10;

std::string trajectory_status_to_string(
    galbot::sdk::g1::TrajectoryControlStatus status) {
  switch (status) {
  case galbot::sdk::g1::TrajectoryControlStatus::INVALID_INPUT:
    return "INVALID_INPUT";
  case galbot::sdk::g1::TrajectoryControlStatus::RUNNING:
    return "RUNNING";
  case galbot::sdk::g1::TrajectoryControlStatus::COMPLETED:
    return "COMPLETED";
  case galbot::sdk::g1::TrajectoryControlStatus::STOPPED_UNREACHED:
    return "STOPPED_UNREACHED";
  case galbot::sdk::g1::TrajectoryControlStatus::ERROR:
    return "ERROR";
  case galbot::sdk::g1::TrajectoryControlStatus::DATA_FETCH_FAILED:
    return "DATA_FETCH_FAILED";
  case galbot::sdk::g1::TrajectoryControlStatus::STATUS_NUM:
    return "STATUS_NUM";
  default:
    return "UNKNOWN_STATUS";
  }
}

void wait_for_traj_reached(const std::vector<std::string> &joint_groups) {
    std::vector<galbot::sdk::g1::TrajectoryControlStatus> traj_exec_states;
    int count = 0;
    bool all_reached = false;
    while (count++ < 150) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        all_reached = true;
        traj_exec_states = galbot::sdk::g1::GalbotRobot::get_instance()
                            .check_trajectory_execution_status(joint_groups);
        if (traj_exec_states.size() != joint_groups.size()) {
        std::cout << "traj_exec_states size != joint_groups size" << std::endl;
        }
        for (int i = 0; i < joint_groups.size(); ++i) {
        std::cout << joint_groups[i] << " exec state is "
                    << trajectory_status_to_string(traj_exec_states[i])
                    << std::endl;
        if (traj_exec_states[i] !=
            galbot::sdk::g1::TrajectoryControlStatus::COMPLETED) {
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

std::vector<galbot::sdk::g1::TrajectoryPoint>
generate_target_trajectory(int32_t joint_size, double ampl = 0.2,
                           double cycle = 10) {
  double amplitude = -ampl;
  double frequency = 1.0 / cycle;
  double phase = -M_PI / 2;
  double offset = amplitude;
  double dt = 0.004;
  int step = g_target_time / dt;

  std::vector<galbot::sdk::g1::TrajectoryPoint> trajectory_data_vec;
  trajectory_data_vec.resize(step + 1);
  // 创建 RobotCommand 轨迹
  for (int i = 0; i <= step; ++i) {
    double t = i * dt;
    trajectory_data_vec[i].time_from_start_second = g_start_time + t;
    trajectory_data_vec[i].joint_command_vec.resize(joint_size);
    // 添加关节命令
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
    // 获取对象实例
    auto& robot = GalbotRobot::get_instance();

    // 初始化系统
    if (robot.init()) {
        std::cout << "系统初始化成功！" << std::endl;
    } else {
        std::cerr << "系统初始化失败！" << std::endl;
        return -1;
    }

    // 程序立即启动，稍等数据就绪时间
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 执行关节轨迹
    Trajectory trajectory;
    // 填写要控制的关节组名称，关节组名称包括["leg", "head", "left_arm", "right_arm"，"left_gripper", "right_gripper"]
    trajectory.joint_groups = {"head"};
    // 如需控制指定关节角度，可填写该字段，如填写将覆盖joint_groups字段
    trajectory.joint_names = {};
    // 生成轨迹
    trajectory.points = generate_target_trajectory(2);
    // 是否阻塞等待轨迹执行完成，false时可使用
    bool is_traj_block = false;

    // 等待轨迹执行完成，此函数封装了check_trajectory_execution_status函数，用于检查轨迹执行状态
    wait_for_traj_reached(trajectory.joint_groups);

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
