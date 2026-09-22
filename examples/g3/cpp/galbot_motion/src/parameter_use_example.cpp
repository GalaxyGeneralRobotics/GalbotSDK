#include <iostream>
#include <vector>
#include <string>
#include <memory>

#include "galbot_motion.hpp"

using namespace galbot::sdk;

int main() {
    // Create Parameter via constructor and set options
    auto p = std::make_shared<Parameter>();

    p->set_blocking(true);            // Set whether to block execution
    p->set_check_collision(false);     // Disable collision detection
    p->set_timeout(5.0);              // Set timeout (seconds)
    p->set_actuate("with_chain_only");// Set drive mode
    p->set_tool_pose(false);           // Whether to consider tool pose
    p->set_reference_frame("base_link");

    std::cout << "--- Parameter p ---" << std::endl;
    std::cout << "blocking: " << (p->get_blocking() ? "True" : "False") << std::endl;
    std::cout << "collision check: " << (p->get_check_collision() ? "True" : "False") << std::endl;
    std::cout << "timeout: " << p->get_timeout() << "s" << std::endl;

    return 0;
}
