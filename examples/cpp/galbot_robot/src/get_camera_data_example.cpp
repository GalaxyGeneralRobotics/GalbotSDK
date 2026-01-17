#include <iostream>
#include <vector>
#include <memory>
#include <unordered_set>
#include <chrono>
#include <thread>

#include "galbot_robot.hpp"
#include "opencv2/opencv.hpp"

using namespace galbot::sdk::g1;

void print_rgb_data(
  const std::shared_ptr<galbot::sdk::g1::RgbData> &rgb_data) {
  if (rgb_data == nullptr) {
    std::cout << "rgb_data is nullptr" << std::endl;
    return;
  }

  std::cout << "Camera image timestamp: "
            << rgb_data->header.stamp.sec << "." << std::setw(9)
            << std::setfill('0') << rgb_data->header.stamp.nanosec
            << std::endl;
  std::cout << "format is " << rgb_data->format << std::endl;
  std::cout << "frame_id is " << rgb_data->header.frame_id << std::endl;
  std::cout << "data size is " << rgb_data->data.size() << std::endl;

  std::cout << "show image:";

  std::shared_ptr<cv::Mat> img = rgb_data->convert_to_cv2_mat();

  cv::imwrite("result_image.jpg", *img);

  std::cout << "图片已保存至 result_image.jpg" << std::endl;
}

void print_depth_data(
    const std::shared_ptr<galbot::sdk::g1::DepthData>
        depth_data_ptr) {
  if (depth_data_ptr == nullptr) {
    std::cout << "depth_data_ptr is nullptr" << std::endl;
    return;
  }

  std::cout << "Camera image timestamp: "
            << depth_data_ptr->header.stamp.sec << "." << std::setw(9)
            << std::setfill('0') << depth_data_ptr->header.stamp.nanosec
            << std::endl;
  std::cout << "format is " << depth_data_ptr->format << std::endl;
  std::cout << "frame_id is " << depth_data_ptr->header.frame_id << std::endl;
  std::cout << "data size is " << depth_data_ptr->data.size() << std::endl;

  std::shared_ptr<cv::Mat> img = depth_data_ptr->convert_to_cv2_mat();

  if (img && !img->empty()) {
    cv::Mat img_vis;

    // 图片信息归一化
    cv::normalize(*img, img_vis, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // 伪彩色增强
    cv::Mat img_color;
    cv::applyColorMap(img_vis, img_color, cv::COLORMAP_JET);

    // 保存图片
    cv::imwrite("check_raw_data.png", *img); 
    cv::imwrite("check_visual_view.jpg", img_color); 

    std::cout << "图片保存完毕：\n"
              << "1. check_raw_data.png -> 包含真实物理深度的全黑图\n"
              << "2. check_visual_view.jpg -> 能看清物体轮廓的彩色图" << std::endl;
    }
}

int main() {
    // 获取对象实例
    auto& robot = GalbotRobot::get_instance();

    // 初始化传感器，为节省资源，只有初始化中传入的相机与雷达传感器可获取数据
    std::unordered_set<SensorType> sensor_types =  {
        SensorType::HEAD_LEFT_CAMERA,       // 头部左相机
        SensorType::LEFT_ARM_DEPTH_CAMERA,  // 左臂深度相机
    };

    // 初始化系统
    if (robot.init(sensor_types)) {
        std::cout << "系统初始化成功！" << std::endl;
    } else {
        std::cerr << "系统初始化失败！" << std::endl;
        return -1;
    }
    // 等待相机数据就绪
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 获取 RGB 图像数据
    std::shared_ptr<RgbData> rgb_data = robot.get_rgb_data(SensorType::HEAD_LEFT_CAMERA);
    if (rgb_data) {
        std::cout << "RGB 图像数据获取成功！" << std::endl;
        print_rgb_data(rgb_data);
    } else {
        std::cerr << "RGB 图像数据获取失败！" << std::endl;
    }

    // 获取深度图像数据
    std::shared_ptr<DepthData> depth_data = robot.get_depth_data(SensorType::LEFT_ARM_DEPTH_CAMERA);
    if (depth_data) {
        std::cout << "深度图像数据获取成功！" << std::endl;
        print_depth_data(depth_data);
    } else {
        std::cerr << "深度图像数据获取失败！" << std::endl;
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
