#include <iostream>
#include <string>

#include "galbot_motion.hpp"

using namespace galbot::sdk::g1;

int main() {

    auto& planner = GalbotMotion::get_instance();

    MotionStatus status = MotionStatus::SUCCESS;
    std::string status_str = planner.status_to_string(status);
    
    std::cout << "MotionStatus 字符串: " << status_str << std::endl;

    auto js = std::make_shared<JointStates>();
    auto ps = std::make_shared<PoseState>();

    js->chain_name = "left_arm";
    js->joint_positions = std::vector<double>(7, 0.0); 

    ps->chain_name = "left_arm";
    ps->pose = Pose(std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    ps->reference_frame = "base_link"; 
    ps->frame_id = "EndEffector";

    std::cout << "--- JointStates ---" << std::endl;
    std::cout << "Type: " << typeid(*js).name() << std::endl; // 打印类名
    std::cout << "Chain Name: " << js->chain_name << std::endl;
    std::cout << "Joints size: " << js->joint_positions.size() << std::endl;

    std::cout << "\n--- PoseState ---" << std::endl;
    std::cout << "Type: " << typeid(*ps).name() << std::endl;
    std::cout << "Chain Name: " << ps->chain_name << std::endl;
    std::cout << "Pose Z: " << ps->pose.position.z << std::endl;

    return 0;
}
