/**
 * Foundation stereo depth example: single run_once + wait_for_new_result to fetch one inference result.
 */

#include <chrono>
#include <iostream>
#include <thread>

#include "galbot_perception.hpp"
#include "galbot_robot.hpp"
#include "galbot_sdk_type.hpp"
#include "opencv2/opencv.hpp"

using namespace galbot::sdk;

bool run_foundation_stereo_once(GalbotPerception& perception) {
    if (!perception.run_once(PerceptionModule::FOUNDATION_STEREO)) {
        std::cerr << "run_once failed to send command" << std::endl;
        return false;
    }

    std::cout << "Waiting for inference result..." << std::endl;
    if (!perception.wait_for_new_result(PerceptionModule::FOUNDATION_STEREO, 5.0)) {
        std::cerr << "Timed out waiting for inference result" << std::endl;
        return false;
    }

    DetectionResult result;
    if (!perception.get_latest_result(PerceptionModule::FOUNDATION_STEREO, result)) {
        std::cerr << "get_latest_result failed" << std::endl;
        return false;
    }

    if (!result.instanceMask.empty()) {
        cv::Mat depth_map = result.instanceMask;
        std::cout << "Depth map size: " << depth_map.cols << "x" << depth_map.rows
                  << ", type: " << depth_map.type() << std::endl;

        double min_val, max_val;
        cv::minMaxLoc(depth_map, &min_val, &max_val);
        std::cout << "Depth value range: [" << min_val << ", " << max_val << "]" << std::endl;

        cv::Mat depth_f;
        depth_map.convertTo(depth_f, CV_32F);
        cv::Mat mask = (depth_f > 0);

        cv::Mat normalized;
        cv::normalize(depth_f, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1, mask);

        cv::Mat colored;
        cv::applyColorMap(normalized, colored, cv::COLORMAP_TURBO);

        cv::imwrite("foundation_stereo_depth.jpg", colored);
        std::cout << "Depth map saved to foundation_stereo_depth.jpg" << std::endl;
    } else {
        std::cout << "No depth map (instanceMask is empty)" << std::endl;
    }

    return true;
}

int main() {
    auto& robot = GalbotRobot::get_instance(MachineType::S1);
    robot.init();

    auto& perception = GalbotPerception::get_instance(MachineType::S1);
    perception.init({PerceptionModule::FOUNDATION_STEREO});

    // Wait for perception models to load
    std::this_thread::sleep_for(std::chrono::seconds(12));
    std::cout << "Init OK, sending single inference request..." << std::endl;

    run_foundation_stereo_once(perception);

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
