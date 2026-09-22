#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_suction_cup_state(std::shared_ptr<SuctionCupState> suction_cup_state) {
    std::cout << "Timestamp (ns): " << suction_cup_state->timestamp_ns << std::endl;
    std::cout << "Activation: " << suction_cup_state->activation << std::endl;
    std::cout << "Pressure: " << suction_cup_state->pressure << " Pa" << std::endl;
    std::cout << "Action State: " << int(suction_cup_state->action_state) << std::endl;
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

    // Get suction cup state
    auto suction_cup_state_ptr = robot.get_suction_cup_state("right_suction_cup");

    if (suction_cup_state_ptr == nullptr) {
        std::cerr << "get suction cup state error" << std::endl;
    } else {
        std::cout << "Right suction cup status:" << std::endl;
        print_suction_cup_state(suction_cup_state_ptr);
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
