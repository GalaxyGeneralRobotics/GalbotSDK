#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>

#include "galbot_motion.hpp"
#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"
#include "opencv2/opencv.hpp"

using namespace galbot::sdk;

namespace {
constexpr size_t kExpectedS1JointCount = 17;  // torso(1)+head(2)+left_arm(7)+right_arm(7)

const std::vector<std::string> kS1GroupOrder = {"torso", "head", "left_arm", "right_arm"};

std::vector<std::string> buildOrderedGroups(
    const std::map<std::string, std::vector<std::vector<double>>>& jointPositions) {
  std::vector<std::string> orderedGroups;
  for (const auto& group : kS1GroupOrder) {
    if (jointPositions.find(group) == jointPositions.end()) {
      throw std::runtime_error("Missing trajectory group: " + group);
    }
    orderedGroups.push_back(group);
  }
  return orderedGroups;
}

std::vector<std::vector<double>> mergeTrajectoryByGroupOrder(
    const std::map<std::string, std::vector<std::vector<double>>>& jointPositions,
    const std::vector<std::string>& orderedGroups) {
  if (orderedGroups.empty()) {
    return {};
  }

  const auto& firstGroupTraj = jointPositions.at(orderedGroups.front());
  if (firstGroupTraj.empty()) {
    return {};
  }

  std::vector<std::vector<double>> mergedTraj = firstGroupTraj;
  for (size_t groupIdx = 1; groupIdx < orderedGroups.size(); ++groupIdx) {
    const auto& groupTraj = jointPositions.at(orderedGroups[groupIdx]);
    if (groupTraj.size() != mergedTraj.size()) {
      throw std::runtime_error("Trajectory point count mismatch in group: " + orderedGroups[groupIdx]);
    }
    for (size_t pointIdx = 0; pointIdx < mergedTraj.size(); ++pointIdx) {
      mergedTraj[pointIdx].insert(mergedTraj[pointIdx].end(), groupTraj[pointIdx].begin(), groupTraj[pointIdx].end());
    }
  }
  return mergedTraj;
}

void printJointGroupSummary(const std::vector<std::string>& jointGroups, const std::vector<JointState>& jointStates) {
  std::cout << "Joint groups to execute: [";
  for (const auto& group : jointGroups) {
    std::cout << group << " ";
  }
  std::cout << "]" << std::endl;
  std::cout << "Joint states size before execution: " << jointStates.size() << std::endl;
}

double computeAbsPositionErrorNorm(const std::vector<double>& target, const std::vector<JointState>& measured) {
  const size_t n = std::min(target.size(), measured.size());
  double errorNorm = 0.0;
  for (size_t i = 0; i < n; ++i) {
    errorNorm += std::abs(target[i] - measured[i].position);
  }
  return errorNorm;
}
}  // namespace

/**
 * Generate target point for trajectory
 */
TrajectoryPoint generateTargetPoint(const std::vector<double>& q, double targetTime = 10.0) {
  TrajectoryPoint jointPosition;
  jointPosition.time_from_start_second = targetTime;

  for (double joint : q) {
    JointCommand jointCmd;
    jointCmd.position = joint;
    jointPosition.joint_command_vec.push_back(jointCmd);
  }

  return jointPosition;
}

/**
 * Generate trajectory for joints
 */
std::shared_ptr<Trajectory> generateTargetTrajectory(const std::vector<std::vector<double>>& trajectory,
                                                     const std::vector<std::string>& jointGroups,
                                                     const std::vector<std::string>& jointNames, double dt = 0.008) {
  if (trajectory.empty()) {
    return nullptr;
  }

  auto traj = std::make_shared<Trajectory>();
  traj->joint_groups = jointGroups;
  traj->joint_names = jointNames;

  double currentTime = 0.0;
  for (const auto& state : trajectory) {
    currentTime += dt;
    TrajectoryPoint trajPoint = generateTargetPoint(state, currentTime);
    traj->points.push_back(trajPoint);
  }

  return traj;
}

/**
 * Print joint states
 */
void printJointStates(const std::vector<JointState>& jointStates) {
  for (const auto& js : jointStates) {
    std::cout << " : position = " << js.position << " , velocity = " << js.velocity
              << " , acceleration = " << js.acceleration << " , effort = " << js.effort << " , current = " << js.current
              << std::endl;
  }
}

/**
 * Fake VLA (Vision-Language-Action) model implementation
 * Generate a small, safe whole-body trajectory based on S1 official zero pose
 */
std::map<std::string, std::vector<std::vector<double>>> fakeVla(
    const std::unordered_map<SensorType, std::shared_ptr<cv::Mat>>& rgbDataDict) {
  std::cout << "Fake VLA executing..." << std::endl;

  const int numPoints = 200;
  std::map<std::string, std::vector<std::vector<double>>> result;

  // S1 zero pose (kept consistent with GalbotMotionS1::move_whole_body_joint_zero)
  const std::vector<double> torso_zero{1.1};
  const std::vector<double> head_zero{0.0, -0.26};
  const std::vector<double> left_arm_zero{-0.47, -0.94, -0.54, -1.92, 0.2, 0.0, 0.0};
  const std::vector<double> right_arm_zero{0.47, 0.94, 0.54, 1.92, -0.2, 0.0, 0.0};

  // Torso: Move linearly from 1.1 to 0.7
  std::vector<std::vector<double>> torsoTraj(numPoints, std::vector<double>(1));
  const double torso_start = 1.1;
  const double torso_end = 0.7;
  for (int i = 0; i < numPoints; ++i) {
    double s = static_cast<double>(i) / (numPoints - 1);  // 0 -> 1
    torsoTraj[i][0] = torso_start + (torso_end - torso_start) * s;
  }

  // Head: Apply small sinusoidal motion on the pitch joint
  std::vector<std::vector<double>> headTraj(numPoints, std::vector<double>(2));
  for (int i = 0; i < numPoints; ++i) {
    double alpha = std::sin(M_PI * i / (numPoints - 1));  // 0 -> 1 -> 0
    headTraj[i][0] = head_zero[0];
    headTraj[i][1] = head_zero[1] + 0.1 * alpha;  // Slightly oscillate around -0.26
  }

  // Arm: Start from zero pose and apply small linear motion on joint 4 for each arm
  auto makeArmTraj = [numPoints](const std::vector<double>& zero, const std::vector<double>& delta) {
    std::vector<std::vector<double>> traj(numPoints, std::vector<double>(zero.size()));
    for (int i = 0; i < numPoints; ++i) {
      double s = static_cast<double>(i) / (numPoints - 1);  // 0 -> 1
      for (size_t j = 0; j < zero.size(); ++j) {
        traj[i][j] = zero[j] + delta[j] * s;
      }
    }
    return traj;
  };

  std::vector<double> left_delta(left_arm_zero.size(), 0.0);
  std::vector<double> right_delta(right_arm_zero.size(), 0.0);
  // Example: make only small motions near the elbow joint; reduce the amplitude further for more conservative behavior
  if (left_delta.size() > 3) {
    left_delta[3] = 0.2;
  }
  if (right_delta.size() > 3) {
    right_delta[3] = -0.2;
  }

  std::vector<std::vector<double>> leftArmTraj = makeArmTraj(left_arm_zero, left_delta);
  std::vector<std::vector<double>> rightArmTraj = makeArmTraj(right_arm_zero, right_delta);

  result["torso"] = torsoTraj;
  result["head"] = headTraj;
  result["left_arm"] = leftArmTraj;
  result["right_arm"] = rightArmTraj;

  return result;
}

std::map<std::string, std::vector<std::vector<double>>> estimate_vla(
    GalbotRobot& robot, const std::unordered_set<SensorType>& enable_sensor_set) {
  /** Get RGB images from enabled cameras */
  std::unordered_map<SensorType, std::shared_ptr<cv::Mat>> rgb_images;
  for (const auto& sensor_type : enable_sensor_set) {
    if (sensor_type == SensorType::LEFT_ARM_CAMERA || sensor_type == SensorType::RIGHT_ARM_CAMERA) {
      std::shared_ptr<RgbData> rgb_data = robot.get_rgb_data(sensor_type, RgbOutputFormat::JPEG, true);
      if (rgb_data) {
        std::shared_ptr<cv::Mat> rgb_image = rgb_data->convert_to_cv2_mat();
        rgb_images[sensor_type] = rgb_image;
      } else {
        std::cerr << "Failed to get RGB data. " << std::endl;
      }
    }
  }
  std::cout << "RGB images size: " << rgb_images.size() << std::endl;

  /** Estimate VLA */
  std::map<std::string, std::vector<std::vector<double>>> vla = fakeVla(rgb_images);
  return vla;
}

/* @brief Check if the robot is safe
 */
void check_robot_safety() {
  std::cout << "⚠️  Note: 1. Please ensure the robot's emergency stop button is released; 2. Please ensure there are no "
               "obstacles in front, back, left, and right of the robot to avoid unexpected situations. \n"
            << std::endl;

  char key;
  for (;;) {
    std::cout << "Please confirm that the robot's emergency stop button is released and there are no obstacles. "
                 "Continue? (y/n)...";
    std::cin >> key;

    if (std::tolower(key) == 'y') {
      std::cout << "User confirmed, continuing execution...\n" << std::endl;
      break;
    } else if (std::tolower(key) == 'n') {
      std::cout << "User not confirmed, program exiting...\n" << std::endl;
      exit(0);
    } else {
      std::cout << "Input error, please enter 'y' or 'n'\n" << std::endl;
    }
  }
}

int main() {
  check_robot_safety();
  try {
    /* Get robot instance  */
    auto& robot = GalbotRobot::get_instance(MachineType::S1);
    auto& motion = GalbotMotion::get_instance(MachineType::S1);
    auto& navigation = GalbotNavigation::get_instance(MachineType::S1);

    /** Enable sensor type - S1 only has arm cameras, not head cameras */
    std::unordered_set<SensorType> enable_sensor_set = {SensorType::LEFT_ARM_CAMERA, SensorType::RIGHT_ARM_CAMERA};

    /* Initialize robot */
    if (robot.init(enable_sensor_set)) {
      std::cout << "Initialization successful" << std::endl;
      std::cout << "Is robot running: " << robot.is_running() << std::endl;
    } else {
      std::cerr << "Initialization failed" << std::endl;
    }
    if (!motion.init()) {
      std::cerr << "Motion initialization failed" << std::endl;
    } else {
      std::cout << "Motion initialization successful" << std::endl;
    }
    if (!navigation.init()) {
      std::cerr << "Navigation initialization failed" << std::endl;
    } else {
      std::cout << "Navigation initialization successful" << std::endl;
    }

    /* Wait for data preparation */
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    /** Estimate VLA actions */
    std::map<std::string, std::vector<std::vector<double>>> joint_positions = estimate_vla(robot, enable_sensor_set);

    // Generate target trajectory with fixed group order to avoid map-order ambiguity.
    std::vector<std::string> jointGroups = buildOrderedGroups(joint_positions);
    std::vector<std::vector<double>> jointTraj = mergeTrajectoryByGroupOrder(joint_positions, jointGroups);
    if (jointTraj.empty()) {
      throw std::runtime_error("Merged trajectory is empty.");
    }

    // Final joint position check
    std::vector<double> wholeBodyJoint = jointTraj.back();
    if (wholeBodyJoint.size() != kExpectedS1JointCount) {
      throw std::runtime_error("Invalid joint dimension: expected 17, got " + std::to_string(wholeBodyJoint.size()));
    }
    std::vector<double> baseState = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    RobotStates checkState;
    checkState.whole_body_joint = wholeBodyJoint;
    checkState.base_state = baseState;

    std::cout << "✅ Final joint position check state: [";
    for (double val : wholeBodyJoint)
      std::cout << val << " ";
    std::cout << "]" << std::endl;

    // Check collision
    // std::tuple<MotionStatus, std::vector<bool>> collisionResult =
    // motion.check_collision(std::vector<std::shared_ptr<RobotStates>>{std::make_shared<RobotStates>(checkState)},
    // true); MotionStatus status = std::get<0>(collisionResult); std::vector<bool> collisionRes =
    // std::get<1>(collisionResult);
    auto [status, collisionRes] = motion.check_collision(
        std::vector<std::shared_ptr<RobotStates>>{std::make_shared<RobotStates>(checkState)}, true);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (status == MotionStatus::SUCCESS) {
      std::cout << "✅ OK: collision check finished: " << collisionRes[0] << " (False=no collision)" << std::endl;

      auto beforeJointStates = robot.get_joint_states(jointGroups, {});
      printJointGroupSummary(jointGroups, beforeJointStates);
      printJointStates(beforeJointStates);
      std::cout << "Absolute position error norm before execution: "
                << computeAbsPositionErrorNorm(wholeBodyJoint, beforeJointStates) << std::endl;

      // Execute trajectory
      auto trajectory = generateTargetTrajectory(jointTraj, jointGroups, {});
      if (trajectory != nullptr) {
        ControlStatus execStatus = robot.execute_joint_trajectory(*trajectory, true);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (execStatus == ControlStatus::SUCCESS) {
          std::cout << "✅ Joint trajectory execution successful." << std::endl;
        } else {
          std::cout << "❌ Joint trajectory execution failed." << std::endl;
          std::cout << "Diagnostic: target points = " << trajectory->points.size()
                    << ", joint dimension = " << wholeBodyJoint.size() << std::endl;
        }

        // Check joint state
        auto jointStates = robot.get_joint_states(jointGroups, {});
        printJointStates(jointStates);
        std::cout << "Absolute position error norm after execution: "
                  << computeAbsPositionErrorNorm(wholeBodyJoint, jointStates) << std::endl;
        std::cout << "✅ Final joint position check state after execution" << std::endl;
      } else {
        std::cout << "❌ Generated trajectory is invalid, cannot execute." << std::endl;
      }
    } else {
      std::cout << "❌ Collision check failed, will not execute the joint trajectory." << std::endl;
    }

    /** Actively send SIGINT exit signal to the robot */
    robot.request_shutdown();
    /** Wait to enter shutdown state */
    robot.wait_for_shutdown();
    /** Release SDK resources */
    robot.destroy();
    std::cout << "Resource release successful" << std::endl;
  } catch (const std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
  }

  return 0;
}
