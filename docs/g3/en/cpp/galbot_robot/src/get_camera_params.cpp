#include <iostream>
#include <vector>
#include <memory>
#include <unordered_set>
#include <chrono>
#include <thread>

#include "galbot_robot.hpp"
#include "opencv2/opencv.hpp"

using namespace galbot::sdk;

void print_rgb_data(
  const std::shared_ptr<RgbData> &rgb_data) {
  if (rgb_data == nullptr) {
    std::cout << "rgb_data is nullptr" << std::endl;
    return;
  }

  std::cout << "Camera image timestamp: "
            << rgb_data->header.timestamp_ns
            << std::endl;
  std::cout << "format is " << rgb_data->format << std::endl;
  std::cout << "frame_id is " << rgb_data->header.frame_id << std::endl;
  std::cout << "data size is " << rgb_data->data.size() << std::endl;

  std::cout << "show image:";

  std::shared_ptr<cv::Mat> img = rgb_data->convert_to_cv2_mat();

  cv::imwrite("result_image.jpg", *img);

  std::cout << "Image saved to result_image.jpg" << std::endl;
}

void print_camera_info(const std::shared_ptr<CameraInfo>& camerainfo){
    if (camerainfo == nullptr) {
        std::cout << "camerainfo is nullptr" << std::endl;
        return;
    }

    std::cout << "camera info:" << std::endl;
    std::cout << "header {" << std::endl;
    std::cout << "  timestamp_ns: " << camerainfo->header.timestamp_ns << std::endl;
    std::cout << "  frame_id: " << camerainfo->header.frame_id << std::endl;
    std::cout << "}" << std::endl;
    std::cout << "width: " << camerainfo->width << std::endl;
    std::cout << "height: " << camerainfo->height << std::endl;
    std::cout << "distortion_model: " << camerainfo->distortion_model << std::endl;
    std::cout << "d: ";
    for (auto& d_i : camerainfo->d) {
        std::cout << d_i << " ";
    }
    std::cout << std::endl;
    std::cout << "k: ";
    for (auto& k_i : camerainfo->k) {
        std::cout << k_i << " ";
    }
    std::cout << std::endl;
    std::cout << "r: ";
    for (auto& r_i : camerainfo->r) {
        std::cout << r_i << " ";
    }
    std::cout << std::endl;
    std::cout << "p: ";
    for (auto& p_i : camerainfo->p) {
        std::cout << p_i << " ";
    }
    std::cout << std::endl;
    std::cout << "T: ";
    for (auto& t_i : camerainfo->T) {
        std::cout << t_i << " ";
    }
    std::cout << std::endl;
    std::cout << "roi {" << std::endl;
    std::cout << "  width: " << camerainfo->roi.width << std::endl;
    std::cout << "  height: " << camerainfo->roi.height << std::endl;
    std::cout << "}" << std::endl;
    std::cout << "camera_type: " << camerainfo->camera_type << std::endl;
}

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    // G3 provides RGB cameras but no arm-mounted depth cameras.
    std::unordered_set<SensorType> sensor_types =  {
        SensorType::HEAD_LEFT_CAMERA,       // Head left camera
    };

    // Initialize system
    if (robot.init(sensor_types)) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }
    // Wait for camera data ready
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Get RGB image data
    std::shared_ptr<RgbData> rgb_data = robot.get_rgb_data(SensorType::HEAD_LEFT_CAMERA, RgbOutputFormat::JPEG, true);
    if (rgb_data) {
        std::cout << "RGB image data retrieved successfully!" << std::endl;
        print_rgb_data(rgb_data);
    } else {
        std::cerr << "Failed to get RGB image data!" << std::endl;
    }

    // Get camera parameters
    std::shared_ptr<CameraInfo> rgb_camerainfo = robot.get_camera_intrinsic(SensorType::HEAD_LEFT_CAMERA);
    if (rgb_camerainfo) {
        std::cout << "Camera parameters retrieved successfully!" << std::endl;
        print_camera_info(rgb_camerainfo);
    } else {
        std::cerr << "Failed to get camera parameters!" << std::endl;
    }

    // Get sensor extrinsics
    std::pair<std::vector<double>, int64_t> sensor_extrinsic = robot.get_sensor_extrinsic(SensorType::HEAD_LEFT_CAMERA);
    if (!sensor_extrinsic.first.empty()) {
        std::cout << "Sensor extrinsics retrieved successfully!" << std::endl;
        std::cout << "transform: ";
        for (auto& t_i : sensor_extrinsic.first) {
            std::cout << t_i << " ";
        }
        std::cout << std::endl;
        std::cout << "timestamp: " << sensor_extrinsic.second << std::endl;
    } else {
        std::cerr << "Failed to get sensor extrinsics!" << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
