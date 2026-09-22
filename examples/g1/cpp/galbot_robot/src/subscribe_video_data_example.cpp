#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_set>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {
constexpr const char* kOutputPath = "head_left.h264";
constexpr int kDurationSeconds = 10;
constexpr int kLogEveryNFrames = 30;

std::mutex g_state_mutex;
std::ofstream g_output_file;
int g_frame_count = 0;

// Important:
// The video callback is executed by the SDK internal dispatch thread.
// Do not perform time-consuming operations in this callback; keep it
// short and non-blocking. Long-running work may delay video frame delivery
// for this subscription.
void video_data_callback(const std::shared_ptr<EncodedVideoData>& video_data) {
  if (video_data == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(g_state_mutex);
  if (g_output_file.is_open() && !video_data->data.empty()) {
    g_output_file.write(reinterpret_cast<const char*>(video_data->data.data()),
                        static_cast<std::streamsize>(video_data->data.size()));
  }

  ++g_frame_count;
  if (g_frame_count == 1 || g_frame_count % kLogEveryNFrames == 0) {
    std::cout << "frame_count=" << g_frame_count
              << ", timestamp_ns=" << video_data->header.timestamp_ns
              << ", format=" << video_data->format
              << ", bytes=" << video_data->data.size() << std::endl;
  }
}
}  // namespace

int main() {
  auto& robot = GalbotRobot::get_instance(MachineType::G1);

  std::unordered_set<SensorType> sensor_types = {SensorType::HEAD_LEFT_CAMERA};
  if (!robot.init(sensor_types)) {
    std::cerr << "System initialization failed!" << std::endl;
    return -1;
  }
  std::cout << "System initialized successfully!" << std::endl;

  g_output_file.open(kOutputPath, std::ios::binary | std::ios::trunc);
  if (!g_output_file.is_open()) {
    std::cerr << "Failed to open output file: " << kOutputPath << std::endl;
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return -1;
  }
  std::cout << "writing H.264 stream to " << kOutputPath << std::endl;

  SensorStatus status = robot.subscribe_video_data(SensorType::HEAD_LEFT_CAMERA, video_data_callback);
  if (status != SensorStatus::SUCCESS) {
    std::cerr << "subscribe_video_data failed, status=" << static_cast<int>(status) << std::endl;
    g_output_file.close();
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    return -1;
  }
  std::cout << "subscribe_video_data success" << std::endl;

  std::this_thread::sleep_for(std::chrono::seconds(kDurationSeconds));

  SensorStatus unsubscribe_status = robot.unsubscribe_video_data(SensorType::HEAD_LEFT_CAMERA);
  std::cout << "unsubscribe_video_data status=" << static_cast<int>(unsubscribe_status) << std::endl;

  {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (g_output_file.is_open()) {
      g_output_file.close();
    }
    std::cout << "received " << g_frame_count << " frames" << std::endl;
  }
  std::cout << "saved H.264 stream to " << kOutputPath << std::endl;

  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  std::cout << "Resources released successfully" << std::endl;
  return 0;
}
