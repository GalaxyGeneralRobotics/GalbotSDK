#include <iostream>
#include <set>
#include <string>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

int main() {

    auto& planner = GalbotMotion::get_instance(MachineType::G3);
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    if (planner.init()) {
        std::cout << "Planner initialized successfully!" << std::endl;
    } else {
        std::cerr << "Planner initialization failed!" << std::endl;
        return -1;
    }
    
    if (robot.init()) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    // You can still manage the robot lifecycle through GalbotRobot
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
