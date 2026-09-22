#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_pose_vec(const std::vector<double> &pose_vec) {
    // Output pose_vec
    std::cout << "pose_vec = [";
    for (size_t i = 0; i < pose_vec.size(); ++i) {
        std::cout << pose_vec[i];
        if (i + 1 < pose_vec.size())
        std::cout << ", ";
    }
    std::cout << "]" << std::endl;
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

    // Get coordinate transform
    std::pair<std::vector<double>, int64_t> tf_ret = robot.get_transform("left_arm_link1", "left_arm_link7", 0);

    if (tf_ret.first.empty()) {
        std::cout << "get_transform error" << std::endl;
    } else {
        std::cout << "tf_timestamp_ns: " << tf_ret.second << std::endl;
        print_pose_vec(tf_ret.first);
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
