#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <thread>

using namespace galbot::sdk;

int main() {
    auto& navigation = GalbotNavigation::get_instance(MachineType::G3);
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    // Initialize system
    if (!robot.init()) {
        std::cerr << "Base instance initialization failed!" << std::endl;
        return -1;
    }
    if (!navigation.init()) {
        std::cerr << "Navigation instance initialization failed!" << std::endl;
        return -1;
    }

    std::cout << "Initialization successful!" << std::endl;

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
