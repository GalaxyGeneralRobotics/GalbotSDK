#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "galbot_robot.hpp"

namespace {

std::string sensor_name(galbot::sdk::SensorType sensor) {
  using galbot::sdk::SensorType;
  switch (sensor) {
    case SensorType::HEAD_LEFT_CAMERA:
      return "HEAD_LEFT_CAMERA";
    case SensorType::HEAD_RIGHT_CAMERA:
      return "HEAD_RIGHT_CAMERA";
    case SensorType::LEFT_ARM_CAMERA:
      return "LEFT_ARM_CAMERA";
    case SensorType::RIGHT_ARM_CAMERA:
      return "RIGHT_ARM_CAMERA";
    default:
      return "UNKNOWN_CAMERA";
  }
}

}  // namespace

int main() {
  using namespace galbot::sdk;

  auto& robot = GalbotRobot::get_instance(MachineType::G3);
  std::unordered_set<SensorType> enable_sensor_set = {
      SensorType::HEAD_LEFT_CAMERA,
      SensorType::LEFT_ARM_CAMERA,
      SensorType::RIGHT_ARM_CAMERA,
  };
  if (!robot.init(enable_sensor_set, true)) {
    std::cout << "init failed" << std::endl;
    robot.destroy();
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  const std::vector<SensorType> cameras = {
      SensorType::LEFT_ARM_CAMERA,   // anchor camera
      SensorType::RIGHT_ARM_CAMERA,  // nearest-neighbor aligned
      SensorType::HEAD_LEFT_CAMERA,  // nearest-neighbor aligned
  };
  auto obs = robot.get_synced_observation(cameras, true);
  if (!obs) {
    std::cout << "get_synced_observation failed" << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return -1;
  }

  // Sync-mode RGB payloads are tightly packed CPU-owned NV12, not JPEG.
  std::cout << "[Synced RGB]" << std::endl;
  const auto anchor_it = obs->rgb_data_map.find(cameras[0]);
  if (anchor_it == obs->rgb_data_map.end() || !anchor_it->second) {
    std::cout << "  anchor missing" << std::endl;
  } else {
    const int64_t anchor_ts_ns = anchor_it->second->header.timestamp_ns;
    std::cout << "  anchor=" << sensor_name(cameras[0]) << " timestamp_ns=" << anchor_ts_ns << std::endl;
    for (const auto& cam : cameras) {
      const auto it = obs->rgb_data_map.find(cam);
      if (it == obs->rgb_data_map.end() || !it->second) {
        std::cout << "  " << sensor_name(cam) << ": missing" << std::endl;
        continue;
      }
      const int64_t cam_ts_ns = it->second->header.timestamp_ns;
      std::cout << "  " << sensor_name(cam) << " timestamp_ns=" << cam_ts_ns
                << " delta_to_anchor_ms=" << (cam_ts_ns - anchor_ts_ns) / 1e6
                << " format=" << it->second->format
                << " dimensions=" << it->second->width << "x" << it->second->height
                << " bytes=" << it->second->data.size() << std::endl;
    }
  }

  if (obs->joint_state) {
    std::cout << "[Joint] timestamp_ns=" << obs->joint_state->timestamp_ns
              << " joint_count=" << obs->joint_state->joint_state_vec.size() << std::endl;
    const size_t print_n = std::min<size_t>(obs->joint_state->joint_state_vec.size(), 5);
    for (size_t i = 0; i < print_n; ++i) {
      const auto& js = obs->joint_state->joint_state_vec[i];
      std::cout << "  [" << i << "] joint_name=" << js.joint_name
                << " position=" << js.position
                << " velocity=" << js.velocity
                << " effort=" << js.effort << std::endl;
    }
  } else {
    std::cout << "[Joint] missing" << std::endl;
  }

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  return 0;
}
