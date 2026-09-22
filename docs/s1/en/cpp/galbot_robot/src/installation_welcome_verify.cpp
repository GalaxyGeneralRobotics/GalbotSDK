#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

constexpr double kSpeedRadS = 0.12;
constexpr double kTimeoutS = 35.0;
constexpr auto kVideoStreamTimeout = std::chrono::seconds(10);

constexpr double kHeadArmsDemoDtS = 0.06;
constexpr int kHeadArmsDemoSeg1 = 28;
constexpr int kHeadArmsDemoSeg2 = 32;
constexpr int kHeadArmsDemoSeg3 = 28;
constexpr double kHeadSwayYawPosRad = 0.22;
constexpr double kHeadSwayYawNegRad = -0.22;

const std::vector<double> kPresetTorso = {0.58};
const std::vector<double> kPresetHead = {0.0, 0.0};
const std::vector<double> kPresetLeftArm = {2.0, -1.5, -0.6, -1.7, 0.0, -0.7, 0.0};
const std::vector<double> kPresetRightArm = {-2.0, 1.5, 0.6, 1.7, 0.0, 0.7, 0.0};

const std::vector<std::string> kHeadArmsDemoGroups = {"head", "left_arm", "right_arm"};

TrajectoryPoint make_point(const std::vector<double>& joint_positions_rad, double time_from_start_sec) {
  TrajectoryPoint point;
  point.time_from_start_second = time_from_start_sec;
  for (double q : joint_positions_rad) {
    JointCommand jc;
    jc.position = q;
    point.joint_command_vec.push_back(jc);
  }
  return point;
}

Trajectory build_trajectory(const std::vector<std::vector<double>>& joint_waypoints_rad,
                            const std::vector<std::string>& joint_groups, double time_step_sec) {
  Trajectory trajectory;
  trajectory.joint_groups = joint_groups;
  trajectory.joint_names = {};
  double time_from_start_sec = 0.0;
  for (const std::vector<double>& waypoint : joint_waypoints_rad) {
    time_from_start_sec += time_step_sec;
    trajectory.points.push_back(make_point(waypoint, time_from_start_sec));
  }
  return trajectory;
}

std::vector<std::vector<double>> linspace_rows(const std::vector<double>& start_positions_rad,
                                               const std::vector<double>& end_positions_rad, int num_samples) {
  std::vector<std::vector<double>> out;
  if (num_samples < 2) {
    out.push_back(start_positions_rad);
    return out;
  }
  out.reserve(static_cast<size_t>(num_samples));
  const int n = num_samples - 1;
  for (int sample_idx = 0; sample_idx < num_samples; ++sample_idx) {
    const double blend = static_cast<double>(sample_idx) / static_cast<double>(n);
    std::vector<double> row(start_positions_rad.size());
    for (size_t j = 0; j < start_positions_rad.size(); ++j) {
      row[j] = start_positions_rad[j] + blend * (end_positions_rad[j] - start_positions_rad[j]);
    }
    out.push_back(std::move(row));
  }
  return out;
}

void append_rows(std::vector<std::vector<double>>& dest, const std::vector<std::vector<double>>& src) {
  dest.insert(dest.end(), src.begin(), src.end());
}

std::vector<double> upper_body_preset_pose_rad() {
  std::vector<double> pose;
  pose.reserve(kPresetHead.size() + kPresetLeftArm.size() + kPresetRightArm.size());
  pose.insert(pose.end(), kPresetHead.begin(), kPresetHead.end());
  pose.insert(pose.end(), kPresetLeftArm.begin(), kPresetLeftArm.end());
  pose.insert(pose.end(), kPresetRightArm.begin(), kPresetRightArm.end());
  return pose;
}

// Slow head yaw left-right (joint2 fixed); arms shift slightly with the sway. Torso not in trajectory.
std::vector<std::vector<double>> build_slow_head_arms_demo_waypoints_rad() {
  const std::vector<double> home = upper_body_preset_pose_rad();
  const std::vector<double> pose_yaw_pos = {
    kHeadSwayYawPosRad, 0.0,
    2.0 + 0.07, -1.5 + 0.04, -0.6, -1.7, 0.0, -0.8 + 0.04, 0.0,
    -2.0 - 0.07, 1.5 - 0.04, 0.6, 1.7, 0.0, 0.8 - 0.04, 0.0
  };
  const std::vector<double> pose_yaw_neg = {
    kHeadSwayYawNegRad, 0.0,
    2.0 - 0.05, -1.5 - 0.03, -0.6, -1.7, 0.0, -0.8 - 0.03, 0.0,
    -2.0 + 0.05, 1.5 + 0.03, 0.6, 1.7, 0.0, 0.8 + 0.03, 0.0
  };

  std::vector<std::vector<double>> rows;
  append_rows(rows, linspace_rows(home, pose_yaw_pos, kHeadArmsDemoSeg1));
  append_rows(rows, linspace_rows(pose_yaw_pos, pose_yaw_neg, kHeadArmsDemoSeg2));
  append_rows(rows, linspace_rows(pose_yaw_neg, home, kHeadArmsDemoSeg3));
  return rows;
}

void print_summary(const std::string& step_description, bool step_passed) {
  std::cout << (step_passed ? "[PASS] " : "[FAIL] ") << step_description << std::endl;
}

// Verify that the head camera publishes a non-empty H.264 frame without writing
// installation-test data to disk. Shared state keeps callback data valid even if
// the SDK dispatch thread finishes the callback while unsubscription is running.
bool verify_head_h264_stream(GalbotRobot& robot) {
  struct VideoCheckState {
    std::mutex mutex;
    std::condition_variable frame_received;
    bool has_valid_frame{false};
    std::string format;
    std::size_t frame_size_bytes{0};
  };

  const auto state = std::make_shared<VideoCheckState>();
  const SensorStatus subscribe_status = robot.subscribe_video_data(
      SensorType::HEAD_LEFT_CAMERA, [state](const std::shared_ptr<EncodedVideoData>& video_data) {
        // Accept both the documented lowercase spelling and the uppercase middleware payload.
        if (video_data == nullptr || video_data->data.empty() ||
            (video_data->format != "h264" && video_data->format != "H264")) {
          return;
        }

        {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->has_valid_frame = true;
          state->format = video_data->format;
          state->frame_size_bytes = video_data->data.size();
        }
        state->frame_received.notify_one();
      });
  if (subscribe_status != SensorStatus::SUCCESS) {
    std::cerr << "[FAIL] subscribe_video_data failed, status=" << static_cast<int>(subscribe_status) << std::endl;
    return false;
  }

  bool received_valid_frame = false;
  std::string received_format;
  std::size_t received_frame_size_bytes = 0;
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    received_valid_frame =
        state->frame_received.wait_for(lock, kVideoStreamTimeout, [&state]() { return state->has_valid_frame; });
    if (received_valid_frame) {
      received_format = state->format;
      received_frame_size_bytes = state->frame_size_bytes;
    }
  }

  // Always unsubscribe after a successful subscription, including timeout paths.
  const SensorStatus unsubscribe_status = robot.unsubscribe_video_data(SensorType::HEAD_LEFT_CAMERA);
  if (unsubscribe_status != SensorStatus::SUCCESS) {
    std::cerr << "[FAIL] unsubscribe_video_data failed, status=" << static_cast<int>(unsubscribe_status) << std::endl;
    return false;
  }
  if (!received_valid_frame) {
    std::cerr << "[FAIL] Timed out waiting for a non-empty H.264 frame" << std::endl;
    return false;
  }

  std::cout << "Head camera H.264 frame: " << received_frame_size_bytes << " bytes format=" << received_format
            << std::endl;
  return true;
}

int main() {
  bool body_preset_step_passed = false;
  bool head_video_stream_ok = false;

  auto& robot = GalbotRobot::get_instance(MachineType::S1);

  const std::unordered_set<SensorType> required_sensors = {SensorType::HEAD_LEFT_CAMERA};
  if (!robot.init(required_sensors)) {
    std::cerr << "GalbotRobot::init failed" << std::endl;
    return 1;
  }
  std::cout << "Robot initialized (S1, HEAD_LEFT_CAMERA enabled)" << std::endl;
  std::this_thread::sleep_for(std::chrono::duration<double>(5.0));

  bool torso_step_ok = false;
  bool upper_step_ok = false;
  bool demo_step_ok = false;

  const ControlStatus torso_set_status =
      robot.set_joint_positions(kPresetTorso, {"torso"}, {}, true, kSpeedRadS, kTimeoutS);
  torso_step_ok = (torso_set_status == ControlStatus::SUCCESS);
  if (!torso_step_ok) {
    std::cerr << "[FAIL] Torso set_joint_positions not SUCCESS (blocking)" << std::endl;
  }
  print_summary("Torso set_joint_positions (blocking)", torso_step_ok);

  if (torso_step_ok) {
    std::vector<double> upper_body_joint_targets_rad;
    upper_body_joint_targets_rad.insert(upper_body_joint_targets_rad.end(), kPresetHead.begin(), kPresetHead.end());
    upper_body_joint_targets_rad.insert(upper_body_joint_targets_rad.end(), kPresetLeftArm.begin(),
                                        kPresetLeftArm.end());
    upper_body_joint_targets_rad.insert(upper_body_joint_targets_rad.end(), kPresetRightArm.begin(),
                                        kPresetRightArm.end());

    const ControlStatus upper_body_set_status =
        robot.set_joint_positions(upper_body_joint_targets_rad, kHeadArmsDemoGroups, {}, true, kSpeedRadS, kTimeoutS);
    upper_step_ok = (upper_body_set_status == ControlStatus::SUCCESS);
    if (!upper_step_ok) {
      std::cerr << "[FAIL] head/left_arm/right_arm set_joint_positions not SUCCESS (blocking)" << std::endl;
    }
    print_summary("Upper set_joint_positions head+arms (blocking)", upper_step_ok);

    if (upper_step_ok) {
      const std::vector<std::vector<double>> demo_waypoints_rad = build_slow_head_arms_demo_waypoints_rad();
      const Trajectory demo_trajectory = build_trajectory(demo_waypoints_rad, kHeadArmsDemoGroups, kHeadArmsDemoDtS);
      const ControlStatus demo_exec_status = robot.execute_joint_trajectory(demo_trajectory, true);
      demo_step_ok = (demo_exec_status == ControlStatus::SUCCESS);
      if (!demo_step_ok) {
        std::cerr << "[FAIL] Head/arms execute_joint_trajectory not SUCCESS (blocking)" << std::endl;
      }
      print_summary("Head/arms demo trajectory (blocking)", demo_step_ok);
    } else {
      std::cerr << "[INFO] Skipping head/arms demo: upper set_joint_positions did not return SUCCESS." << std::endl;
      print_summary("Head/arms demo trajectory (blocking)", false);
    }
  } else {
    std::cerr << "[INFO] Skipping upper-body preset and head/arms demo: torso set_joint_positions not SUCCESS."
              << std::endl;
    print_summary("Upper set_joint_positions head+arms (blocking)", false);
    print_summary("Head/arms demo trajectory (blocking)", false);
  }

  body_preset_step_passed = torso_step_ok && upper_step_ok && demo_step_ok;

  head_video_stream_ok = verify_head_h264_stream(robot);
  print_summary("Head camera H.264 video stream", head_video_stream_ok);

  std::cout << "\n======== Summary ========" << std::endl;
  print_summary("Motion verify (blocking)", body_preset_step_passed);
  print_summary("Head camera H.264 video stream", head_video_stream_ok);

  const bool installation_all_checks_passed = body_preset_step_passed && head_video_stream_ok;
  std::cout << (installation_all_checks_passed ? "\nOverall: PASS\n" : "\nOverall: FAIL\n");

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();

  return installation_all_checks_passed ? 0 : 1;
}
