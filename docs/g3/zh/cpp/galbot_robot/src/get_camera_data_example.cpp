#include <iostream>
#include <vector>
#include <memory>
#include <unordered_set>
#include <chrono>
#include <thread>

#include "galbot_robot.hpp"
#include "opencv2/opencv.hpp"

using namespace galbot::sdk;

void print_rgb_data(const std::shared_ptr<RgbData> &rgb_data) {
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

void print_ir_data(const std::string& name, const std::shared_ptr<IrData>& ir_data) {
  if (ir_data == nullptr) {
    std::cout << name << " is nullptr" << std::endl;
    return;
  }

  std::cout << name << " timestamp: " << ir_data->header.timestamp_ns << std::endl;
  std::cout << "format is " << ir_data->format << std::endl;
  std::cout << "frame_id is " << ir_data->header.frame_id << std::endl;
  std::cout << "size is " << ir_data->width << "x" << ir_data->height << std::endl;
  std::cout << "data size is " << ir_data->data.size() << std::endl;

  std::shared_ptr<cv::Mat> img = ir_data->convert_to_cv2_mat();
  if (img && !img->empty()) {
    std::string filename = name + ".png";
    cv::imwrite(filename, *img);
    std::cout << "Image saved to " << filename << std::endl;
  }
}

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    // G3 does not have arm-mounted depth cameras.
    std::unordered_set<SensorType> sensor_types =  {
        SensorType::HEAD_LEFT_CAMERA,       // Head left camera
        SensorType::LEFT_ARM_INFRA_CAMERA_1,    // Left arm IR camera 1
        SensorType::LEFT_ARM_INFRA_CAMERA_2,    // Left arm IR camera 2
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

    // Get left arm IR camera 1 image data
    std::shared_ptr<IrData> ir1_data = robot.get_ir_data(SensorType::LEFT_ARM_INFRA_CAMERA_1);
    if (ir1_data) {
        std::cout << "IR camera 1 data retrieved successfully!" << std::endl;
        print_ir_data("left_arm_infra1", ir1_data);
    } else {
        std::cerr << "Failed to get IR camera 1 data!" << std::endl;
    }

    // Get left arm IR camera 2 image data
    std::shared_ptr<IrData> ir2_data = robot.get_ir_data(SensorType::LEFT_ARM_INFRA_CAMERA_2);
    if (ir2_data) {
        std::cout << "IR camera 2 data retrieved successfully!" << std::endl;
        print_ir_data("left_arm_infra2", ir2_data);
    } else {
        std::cerr << "Failed to get IR camera 2 data!" << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
