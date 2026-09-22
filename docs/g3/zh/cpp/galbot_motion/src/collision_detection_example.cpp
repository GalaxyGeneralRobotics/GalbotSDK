#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <tuple>
#include <memory>
#include <stdexcept>
#include <algorithm>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"
#include "galbot_navigation.hpp"

using namespace galbot::sdk;

int main() {

    auto& planner = GalbotMotion::get_instance(MachineType::G3);
    auto& robot = GalbotRobot::get_instance(MachineType::G3);
    auto& navigation = GalbotNavigation::get_instance(MachineType::G3);

    // NOTE:
    // - GalbotNavigation (galbotNav) may use real-time obstacle perception/avoidance during navigation (deployment dependent).
    // - GalbotMotion does NOT provide real-time obstacle perception today; Motion collision uses self-collision +
    //   manually loaded obstacles (add_obstacle/attach_target_object).

    if (!planner.init()) {
        std::cerr << "GalbotMotion init FAILED" << std::endl;
        return -1;
    }
    if (!robot.init()) {
        std::cerr << "GalbotRobot init FAILED" << std::endl;
        return -1;
    }
    if (!navigation.init()) {
        std::cerr << "GalbotNavigation init FAILED" << std::endl;
        return -1;
    }

    std::unordered_map<std::string, std::vector<double>> chain_joints = {
        {"leg",       {0.5, 1.5, 1.0, 0.0, 0.0}},
        {"head",      {0.0, 0.0}},
        {"left_arm",  {1.5, -1.36, -0.45, 1.53, -0.1, -0.42, 0.0}},
        {"right_arm", {-1.5, 1.36, 0.45, -1.53, 0.1, 0.42, 0.0}}
    };

    std::vector<double> whole_body_joint;
    std::vector<std::string> keys = {"leg", "head", "left_arm", "right_arm"};
    for (const auto& key : keys) {
        whole_body_joint.insert(whole_body_joint.end(), chain_joints[key].begin(), chain_joints[key].end());
    }

    std::vector<double> base_state = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> bad_left_arm_joint = {1.00217,-1.60012,0.450993,1.19433,0.450993,-0.387987,0.076236};

    auto custom_param = std::make_shared<Parameter>();

    try {
        std::cout << ">> Running collision check..." << std::endl;

        // Construct a RobotStates list for collision checking
        std::vector<std::shared_ptr<RobotStates>> check_states;

        // status 0: status (RobotStates)
        auto state0 = std::make_shared<RobotStates>();
        state0->whole_body_joint = whole_body_joint;
        state0->base_state = base_state;
        check_states.push_back(state0);

        // status 1: status (JointStates RobotStates)
        auto state1 = std::make_shared<JointStates>();
        state1->chain_name = "left_arm";
        state1->joint_positions = bad_left_arm_joint;
        check_states.push_back(state1);

        // Call collision detection interface
        auto res = planner.check_collision(check_states, true, custom_param);

        MotionStatus status = std::get<0>(res);
        std::vector<bool> collision_results = std::get<1>(res);

        std::cout << "Status: " << planner.status_to_string(status) << std::endl;

        if (status == MotionStatus::SUCCESS) {
            std::cout << "OK: collision check finished (false=no collision, true=collision):" << std::endl;
            for (size_t i = 0; i < collision_results.size(); ++i) {
                std::cout << "  - status [" << i << "]: " 
                          << (collision_results[i] ? "COLLISION" : "NO COLLISION") 
                          << std::endl;
            }
        } else {
            std::cerr << "ERROR: collision check returned failure." << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "ERROR: collision check exception: " << e.what() << std::endl;
    }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
