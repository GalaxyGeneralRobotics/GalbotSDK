#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {

struct ExampleInputs {
  std::vector<std::string> chain_names;
};

ExampleInputs make_example_inputs() {
  ExampleInputs inputs;

  // get_chain_joint_names(chain_name) input.
  inputs.chain_names = {
      "left_arm",
      "right_arm",
      "leg",
      "head",
      "invalid_chain",
  };

  return inputs;
}

}  // namespace

int main() {
  auto& motion = GalbotMotion::get_instance(MachineType::G1);
  auto& robot = GalbotRobot::get_instance(MachineType::G1);

  // Initialize SDK interfaces directly in the example so users can see the required order.
  if (!motion.init()) {
    std::cerr << "GalbotMotion init FAILED" << std::endl;
    return -1;
  }
  if (!robot.init()) {
    std::cerr << "GalbotRobot init FAILED" << std::endl;
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  const ExampleInputs inputs = make_example_inputs();
  for (const auto& chain : inputs.chain_names) {
    // This is the API being demonstrated: query joint names by chain name.
    const auto joint_names = motion.get_chain_joint_names(chain);
    std::cout << "get_chain_joint_names(" << chain << "): ";
    if (joint_names.empty()) {
      std::cout << "<empty>";
    } else {
      for (const auto& name : joint_names) {
        std::cout << name << " ";
      }
    }
    std::cout << std::endl;
  }

  // Clean shutdown releases SDK resources before the process exits.
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  return 0;
}
