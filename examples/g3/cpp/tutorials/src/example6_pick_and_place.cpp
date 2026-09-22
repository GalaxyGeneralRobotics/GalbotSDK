// This tutorial demonstrates a complete pick-and-place task for a standard application scenario.
// It integrates navigation, perception, motion planning, and robot control, and can be used as a
// reference implementation for standard applications.
// Before the task starts, the robot moves to the known 21-joint initial pose defined by this example.
// After picking, the robot navigates to its current map pose with the X coordinate offset by 0.1 m,
// then performs placement.
// The taught grasp approach and place descent use move_line; other end-effector motions use
// set_end_effector_pose. All Cartesian poses below were taught for the bare left-arm
// EndEffector frame relative to base_link on G3.

#include <array>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_set>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <opencv2/opencv.hpp>

using namespace galbot::sdk;

constexpr double kPlaceNavigationXOffsetM = 0.1;
// Slow the arm trajectory down and make each task waypoint visually distinct.
constexpr double kArmMinMoveTimeS = 3.0;
constexpr double kArmMoveTimeoutS = 30.0;
constexpr double kArmJointSpeedRadS = 0.1;
constexpr auto kWaypointHold = std::chrono::milliseconds(1000);
constexpr auto kGripperSettle = std::chrono::milliseconds(500);
const std::vector<double> kPreGraspPose = {
    0.43645796, 0.18778193, 0.92196164, -0.01950938, 0.02598107, -0.05602237, 0.99790073,
};
const std::vector<double> kGraspPose = {
    0.51412043, 0.18056627, 0.96286294, -0.02264866, 0.00051349, -0.05636788, 0.99815301,
};
const std::vector<double> kLiftGraspPose = {
    0.53047695, 0.15313211, 1.09918264, -0.00801662, -0.19539735, -0.09980387, 0.97559971,
};
const std::vector<double> kRetreatPose = {
    0.43381901, 0.16343561, 1.0412931, -0.02825337, -0.11813149, -0.09585053, 0.98795717,
};
const std::vector<double> kPrePlacePose = {
    0.50956077, 0.20189686, 1.05753193, -0.02514585, -0.11590543, -0.04433032, 0.99195183,
};
const std::vector<double> kPlacePose = {
    0.49092053, 0.19268637, 0.97338715, -0.02437315, -0.01105621, -0.05520225, 0.99811644,
};
const std::vector<std::string> kInitialLegJointNames = {
    "leg_joint1", "leg_joint2", "leg_joint3", "leg_joint4", "leg_joint5",
};
const std::vector<double> kInitialLegJointPositions = {0.4, 1.2, 0.8, 0.0, 0.0};

const std::vector<std::string> kInitialUpperBodyJointNames = {
    "head_joint1",      "head_joint2",      "left_arm_joint1",  "left_arm_joint2",  "left_arm_joint3",
    "left_arm_joint4",  "left_arm_joint5",  "left_arm_joint6",  "left_arm_joint7",  "right_arm_joint1",
    "right_arm_joint2", "right_arm_joint3", "right_arm_joint4", "right_arm_joint5", "right_arm_joint6",
    "right_arm_joint7",
};
const std::vector<double> kInitialUpperBodyJointPositions = {
    0.00001198, 0.0, 1.50008953, -1.3599937, -0.45010296, 1.53000093, -0.09994802, -0.41997507,
    0.00004712, -1.49992013, 1.3599937, 0.44991097, -1.5300498, 0.10004402, 0.4200955, 0.00004887,
};

bool move_to_initial_pose(GalbotRobot& robot) {
  const ControlStatus leg_status =
      robot.set_joint_positions(kInitialLegJointPositions, {}, kInitialLegJointNames, true, 0.2, 20.0);
  if (leg_status != ControlStatus::SUCCESS) {
    std::cout << "❌ Failed to move the legs to the initial pose: status=" << (int) leg_status << std::endl;
    return false;
  }
  std::cout << "✅ Successfully moved the legs to the initial pose: status=" << (int) leg_status << std::endl;

  const ControlStatus upper_body_status = robot.set_joint_positions(
      kInitialUpperBodyJointPositions, {}, kInitialUpperBodyJointNames, true, kArmJointSpeedRadS, 20.0);
  if (upper_body_status != ControlStatus::SUCCESS) {
    std::cout << "❌ Failed to move the head and arms to the initial pose: status=" << (int) upper_body_status
              << std::endl;
    return false;
  }

  std::cout << "✅ Successfully moved the head and arms to the initial pose: status=" << (int) upper_body_status
            << std::endl;
  return true;
}

static std::vector<double> pose_to_vector7(const Pose& p) {
  return {p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
}

// Quaternion and rotation matrix utilities (without Eigen)
struct QuaternionST {
  double x, y, z, w;

  QuaternionST(double x = 0, double y = 0, double z = 0, double w = 1) : x(x), y(y), z(z), w(w) {}

  // Convert quaternion to 3x3 rotation matrix
  std::array<std::array<double, 3>, 3> to_matrix() const {
    std::array<std::array<double, 3>, 3> mat;

    double xx = x * x, yy = y * y, zz = z * z;
    double xy = x * y, xz = x * z, yz = y * z;
    double wx = w * x, wy = w * y, wz = w * z;

    mat[0][0] = 1 - 2 * (yy + zz);
    mat[0][1] = 2 * (xy - wz);
    mat[0][2] = 2 * (xz + wy);

    mat[1][0] = 2 * (xy + wz);
    mat[1][1] = 1 - 2 * (xx + zz);
    mat[1][2] = 2 * (yz - wx);

    mat[2][0] = 2 * (xz - wy);
    mat[2][1] = 2 * (yz + wx);
    mat[2][2] = 1 - 2 * (xx + yy);

    return mat;
  }

};

// 4x4 transformation matrix
struct Transform {
  std::array<std::array<double, 4>, 4> mat;

  Transform() {
    // Identity matrix
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        mat[i][j] = (i == j) ? 1.0 : 0.0;
      }
    }
  }

  // Set rotation from quaternion
  void set_rotation(const QuaternionST& q) {
    auto rot = q.to_matrix();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        mat[i][j] = rot[i][j];
      }
    }
  }

  // Set translation
  void set_translation(const std::array<double, 3>& t) {
    mat[0][3] = t[0];
    mat[1][3] = t[1];
    mat[2][3] = t[2];
  }

  // Transform a 3D point
  std::array<double, 3> transform_point(const std::array<double, 3>& p) const {
    std::array<double, 3> result;
    for (int i = 0; i < 3; ++i) {
      result[i] = mat[i][0] * p[0] + mat[i][1] * p[1] + mat[i][2] * p[2] + mat[i][3];
    }
    return result;
  }

};

void navigation_to_goal(GalbotNavigation& nav, const std::vector<double>& goal_pose, int retry_cnt = 3) {
  try {
    auto cur_pose = nav.get_current_pose();
    std::cout << "Current pose: [";
    for (auto val : pose_to_vector7(cur_pose))
      std::cout << val << " ";
    std::cout << "]" << std::endl;

    Pose goal(goal_pose);
    if (nav.check_path_reachability(goal, cur_pose)) {
      retry_cnt = 3;
      NavigationStatus status;

      while (true) {
        status = nav.navigate_to_goal(goal, true, true, 20.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        retry_cnt--;

        if (nav.check_goal_arrival() || retry_cnt < 0) {
          break;
        } else {
          std::cout << "Navigation failed: status=" << (int) status << ", retrying: " << retry_cnt << std::endl;
        }
      }

      std::cout << "navigate_to_goal return status: " << (int) status << std::endl;
      std::cout << "Has arrived: " << nav.check_goal_arrival() << std::endl;
    } else {
      std::cout << "Path unreachable or unsafe" << std::endl;
    }
  } catch (const std::exception& e) {
    std::cout << "Exception occurred during navigation: " << e.what() << std::endl;
  }
}

std::vector<double> pose_camera_to_base(GalbotRobot& robot, const std::vector<double>& pose_camera) {
  // The G3 TF tree publishes the arm RGB camera under this frame name.
  std::string source_frame = "left_arm_camera";
  std::string target_frame = "base_link";

  auto [base_to_cam, success] = robot.get_transform(target_frame, source_frame);

  if (!success || base_to_cam.size() != 7) {
    std::cout << "Failed to get transform from camera to chassis" << std::endl;
    return {};
  } else {
    std::cout << "base_to_cam: [";
    for (auto val : base_to_cam)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
  }

  Transform base_to_cam_mat;
  QuaternionST quat(base_to_cam[3], base_to_cam[4], base_to_cam[5], base_to_cam[6]);
  base_to_cam_mat.set_rotation(quat);
  base_to_cam_mat.set_translation({base_to_cam[0], base_to_cam[1], base_to_cam[2]});

  std::array<double, 3> cam_pos = {pose_camera[0], pose_camera[1], pose_camera[2]};
  auto pose_base = base_to_cam_mat.transform_point(cam_pos);

  return {pose_base[0], pose_base[1], pose_base[2], 0.0, 0.0, 0.0, 1.0};
}

std::vector<double> detect_object(GalbotRobot& robot) {
  const std::vector<double> k_zero_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

  try {
    // G3 uses the left-arm RGB camera. The pose below is a placeholder for
    // an application-specific perception result.
    auto rgb_image_data = robot.get_rgb_data(SensorType::LEFT_ARM_CAMERA);

    if (!rgb_image_data) {
      std::cout << "No rgb image data!" << std::endl;
      return k_zero_pose;
    }
    std::cout << "Get rgb image success" << std::endl;

    auto img_ptr = rgb_image_data->convert_to_cv2_mat();
    if (!img_ptr || img_ptr->empty()) {
      std::cout << "Failed to decode rgb image" << std::endl;
      return k_zero_pose;
    }

    // Placeholder perception result in the OpenCV camera frame (x right, y down, z forward).
    // Replace this pose with the output of an application-specific detector.
    std::vector<double> object_pose_camera = {0.0, 0.20, 0.29, 0.0, 0.71, 0.0, 0.71};

    std::cout << "object_pose_camera: [";
    for (auto val : object_pose_camera)
      std::cout << val << " ";
    std::cout << "]" << std::endl;

    auto object_pose_base = pose_camera_to_base(robot, object_pose_camera);
    if (object_pose_base.size() != 7) {
      std::cout << "Target pose in chassis coordinate system: (invalid / transform failed)" << std::endl;
      return k_zero_pose;
    }
    std::cout << "Target pose in chassis coordinate system: [";
    for (auto val : object_pose_base)
      std::cout << val << " ";
    std::cout << "]" << std::endl;

    return object_pose_base;

  } catch (const std::exception& e) {
    std::cout << "Target detection exception: " << e.what() << std::endl;
    return k_zero_pose;
  }
}

void check_robot_safety() {
  std::cout << "⚠️  Note: 1. Please ensure the emergency stop button of the robot is released; "
            << "2. Please ensure there are no obstructions around the robot to avoid unexpected situations. "
            << "3. Please ensure the area around the robot is clear of obstacles." << std::endl;

  while (true) {
    std::cout << "Please confirm that the robot's emergency stop button is released "
              << "and there are no obstructions, continue? (y/n)..." << std::endl;

    std::string key;
    std::cin >> key;

    if (key == "y" || key == "Y") {
      std::cout << "User confirmed, continuing..." << std::endl;
      break;
    } else if (key == "n" || key == "N") {
      std::cout << "User did not confirm, exiting program..." << std::endl;
      exit(1);
    } else {
      std::cout << "Invalid input, please enter 'y' or 'n'" << std::endl;
    }
  }
}

bool pick_and_place(GalbotRobot& robot, GalbotNavigation& nav, GalbotMotion& motion,
                    const std::vector<double>& object_pose_base, bool navigation_enabled) {
  try {
    // attach_tool() only loads a planning model; it does not detect hardware.
    // Do not add a virtual gripper when no physical gripper feedback exists.
    const auto left_gripper_state = robot.get_gripper_state("left_gripper");
    bool has_left_gripper = static_cast<bool>(left_gripper_state);
    ControlStatus gripper_status = ControlStatus::DATA_FETCH_FAILED;
    if (has_left_gripper) {
      // A gripper-less G3 can still publish placeholder feedback. Confirm
      // hardware availability with the blocking open command that the pick
      // task needs anyway, before changing the planning model.
      gripper_status = robot.set_gripper_command("left_gripper", 0.1, 0.05, 10, true);
      if (gripper_status == ControlStatus::DATA_FETCH_FAILED || gripper_status == ControlStatus::TIMEOUT) {
        has_left_gripper = false;
        std::cout << "⚠️ Left gripper feedback is only a placeholder; status=" << (int) gripper_status << std::endl;
      } else if (gripper_status != ControlStatus::SUCCESS) {
        std::cout << "❌ Failed to open left gripper: status=" << (int) gripper_status << std::endl;
        return false;
      }
    }
    MotionStatus status = MotionStatus::SUCCESS;
    if (has_left_gripper) {
      status = motion.attach_tool("left_arm", "galbot_gripper");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (status != MotionStatus::SUCCESS) {
        std::cout << "❌ Failed to attach tool: status=" << (int) status << std::endl;
        return false;
      }
      std::cout << "✅ Successfully attached tool: status=" << (int) status << std::endl;
    } else {
      std::cout << "⚠️ Left gripper was not detected; skipping its tool model and gripper commands." << std::endl;
      // Clear a model that may have been left attached by an earlier
      // interrupted/failed run of this example.
      const MotionStatus stale_tool_status = motion.detach_tool("left_arm");
      if (stale_tool_status == MotionStatus::SUCCESS) {
        std::cout << "Cleared a previously attached left-arm tool model." << std::endl;
      }
    }

    // These poses were taught for the bare arm EndEffector frame, not the tool TCP.
    auto motion_params = std::make_shared<Parameter>();
    motion_params->set_direct_execute(true);
    motion_params->set_blocking(true);
    motion_params->set_timeout(kArmMoveTimeoutS);
    motion_params->set_tool_pose(false);
    motion_params->set_check_collision(true);
    motion_params->set_reference_frame("base_link");
    motion_params->set_actuate("with_chain_only");
    if (has_left_gripper) {
      std::cout << "✅ Successfully set left gripper width to 0.1m: status=" << (int) gripper_status << std::endl;
    }

    // These G3 poses were recorded by drag teaching in the base_link frame.
    // Perception remains informational until an application-specific grasp-pose
    // generator replaces the taught trajectory.
    std::cout << "Perception pose: [";
    for (auto val : object_pose_base)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    std::vector<double> pre_grasp_pose = kPreGraspPose;
    std::vector<double> grasp_pose = kGraspPose;
    std::vector<double> lift_grasp_pose = kLiftGraspPose;
    std::vector<double> retreat_pose = kRetreatPose;

    std::cout << "pre_grasp_pose: [";
    for (auto val : pre_grasp_pose)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    status = motion.set_end_effector_pose(pre_grasp_pose, "left_arm", "base_link", nullptr, true, true,
                                          kArmMoveTimeoutS,
                                          motion_params);
    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to execute pre_grasp pose command: status=" << (int) status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully executed pre_grasp pose command" << std::endl;
    std::cout << "Holding at pre_grasp for 1.0s..." << std::endl;
    std::this_thread::sleep_for(kWaypointHold);

    // Use move_line for the taught final approach to the object.
    MotionPlanChainTarget grasp_target;
    grasp_target.chain_name = "left_arm";
    grasp_target.mode = MotionPlanTargetMode::kCartesian;
    grasp_target.cart.chain_name = "left_arm";
    grasp_target.cart.frame_id = "EndEffector";
    grasp_target.cart.reference_frame = "base_link";
    grasp_target.cart.pose = Pose(grasp_pose);

    std::cout << "grasp_pose: [";
    for (auto val : grasp_pose)
      std::cout << val << " ";
    std::cout << "]" << std::endl;

    auto move_line_result = motion.move_line({{grasp_target}}, *motion_params);
    status = std::get<0>(move_line_result);
    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to execute grasp move_line: status=" << (int) status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully executed grasp move_line" << std::endl;
    std::cout << "Holding at grasp for 1.0s..." << std::endl;
    std::this_thread::sleep_for(kWaypointHold);

    if (has_left_gripper) {
      const ControlStatus gripper_status = robot.set_gripper_command("left_gripper", 0.02, 0.05, 10, true);
      if (gripper_status != ControlStatus::SUCCESS) {
        std::cout << "❌ Failed to close left gripper: status=" << (int) gripper_status << std::endl;
        return false;
      }
      std::cout << "✅ Successfully closed left gripper: status=" << (int) gripper_status << std::endl;
      std::this_thread::sleep_for(kGripperSettle);
    }

    std::cout << "lift_grasp_pose: [";
    for (auto val : lift_grasp_pose)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    status = motion.set_end_effector_pose(lift_grasp_pose, "left_arm", "base_link", nullptr, true, true,
                                          kArmMoveTimeoutS,
                                          motion_params);
    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to execute lift_grasp pose command: status=" << (int) status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully executed lift_grasp pose command" << std::endl;
    std::cout << "Holding at lift_grasp for 1.0s..." << std::endl;
    std::this_thread::sleep_for(kWaypointHold);

    std::cout << "retreat_pose: [";
    for (auto val : retreat_pose)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    status = motion.set_end_effector_pose(retreat_pose, "left_arm", "base_link", nullptr, true, true,
                                          kArmMoveTimeoutS,
                                          motion_params);
    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to execute retreat pose command: status=" << (int) status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully executed retreat pose command" << std::endl;
    std::cout << "Holding at retreat for 1.0s..." << std::endl;
    std::this_thread::sleep_for(kWaypointHold);

    // Return to the initial left-arm joint pose before navigation.
    const std::vector<std::string> left_arm_joint_names(kInitialUpperBodyJointNames.begin() + 2,
                                                        kInitialUpperBodyJointNames.begin() + 9);
    const std::vector<double> left_arm_joint_positions(kInitialUpperBodyJointPositions.begin() + 2,
                                                       kInitialUpperBodyJointPositions.begin() + 9);
    const ControlStatus left_arm_status =
        robot.set_joint_positions(left_arm_joint_positions, {}, left_arm_joint_names, true, kArmJointSpeedRadS, 20.0);
    if (left_arm_status != ControlStatus::SUCCESS) {
      std::cout << "❌ Failed to restore the left-arm initial pose: status=" << (int) left_arm_status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully restored the left-arm initial pose: status=" << (int) left_arm_status << std::endl;

    if (navigation_enabled) {
      auto place_navigation_goal = pose_to_vector7(nav.get_current_pose());
      place_navigation_goal[0] += kPlaceNavigationXOffsetM;
      std::cout << "Place navigation target pose: [";
      for (auto val : place_navigation_goal)
        std::cout << val << " ";
      std::cout << "]" << std::endl;
      navigation_to_goal(nav, place_navigation_goal);
      std::this_thread::sleep_for(std::chrono::seconds(2));
      std::cout << "✅ Navigation stage completed; starting Place" << std::endl;
    } else {
      std::cout << "⚠️ Navigation is unavailable; skipping navigation and continuing with Place at the current "
                   "position."
                << std::endl;
    }

    // Move to the taught pre-place pose, then follow the taught placement approach.
    std::vector<double> pre_place_pose = kPrePlacePose;
    std::vector<double> place_pose = kPlacePose;
    std::cout << "pre_place_pose: [";
    for (auto val : pre_place_pose)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    status = motion.set_end_effector_pose(pre_place_pose, "left_arm", "base_link", nullptr, true, true,
                                          kArmMoveTimeoutS,
                                          motion_params);
    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to execute pre_place pose command: status=" << (int) status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully executed pre_place pose command" << std::endl;
    std::cout << "Holding at pre_place for 1.0s..." << std::endl;
    std::this_thread::sleep_for(kWaypointHold);

    MotionPlanChainTarget place_target;
    place_target.chain_name = "left_arm";
    place_target.mode = MotionPlanTargetMode::kCartesian;
    place_target.cart.chain_name = "left_arm";
    place_target.cart.frame_id = "EndEffector";
    place_target.cart.reference_frame = "base_link";
    place_target.cart.pose = Pose(place_pose);

    std::cout << "place_pose: [";
    for (auto val : place_pose)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    move_line_result = motion.move_line({{place_target}}, *motion_params);
    status = std::get<0>(move_line_result);
    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to execute place move_line: status=" << (int) status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully executed place move_line" << std::endl;
    std::cout << "Holding at place for 1.0s..." << std::endl;
    std::this_thread::sleep_for(kWaypointHold);

    if (has_left_gripper) {
      const ControlStatus gripper_status = robot.set_gripper_command("left_gripper", 0.1, 0.05, 10, true);
      if (gripper_status != ControlStatus::SUCCESS) {
        std::cout << "❌ Failed to release left gripper: status=" << (int) gripper_status << std::endl;
        return false;
      }
      std::cout << "✅ Successfully released left gripper: status=" << (int) gripper_status << std::endl;
      std::this_thread::sleep_for(kGripperSettle);
    }

    // Return the left arm to its initial joint pose after placing the object.
    const ControlStatus post_place_arm_status =
        robot.set_joint_positions(left_arm_joint_positions, {}, left_arm_joint_names, true, kArmJointSpeedRadS, 20.0);
    if (post_place_arm_status != ControlStatus::SUCCESS) {
      std::cout << "❌ Failed to restore the left-arm pose after placing: status=" << (int) post_place_arm_status
                << std::endl;
      return false;
    }
    std::cout << "✅ Successfully restored the left-arm pose after placing: status=" << (int) post_place_arm_status
              << std::endl;

    if (has_left_gripper) {
      status = motion.detach_tool("left_arm");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (status != MotionStatus::SUCCESS) {
        std::cout << "❌ Failed to detach tool: status=" << (int) status << std::endl;
      } else {
        std::cout << "✅ Successfully detached tool: status=" << (int) status << std::endl;
      }
    }

    return true;

  } catch (const std::exception& e) {
    std::cout << "Exception occurred during pick_and_place: " << e.what() << std::endl;
    return false;
  }
}

int main() {
  check_robot_safety();

  // Get robot instances
  auto& robot = GalbotRobot::get_instance(MachineType::G3);
  auto& motion = GalbotMotion::get_instance(MachineType::G3);
  auto& nav = GalbotNavigation::get_instance(MachineType::G3);
  std::shared_ptr<MotionPlanConfig> original_motion_config;
  bool slow_motion_config_applied = false;

  try {
    // Enable sensors
    // G3 does not have arm-mounted depth cameras.
    std::unordered_set<SensorType> enable_sensor_set = {SensorType::LEFT_ARM_CAMERA};

    // Initialize robot
    if (robot.init(enable_sensor_set)) {
      std::cout << "GalbotRobot initialization successful" << std::endl;
    } else {
      std::cout << "GalbotRobot initialization failed" << std::endl;
    }

    if (motion.init()) {
      std::cout << "GalbotMotion initialization successful" << std::endl;
    } else {
      std::cout << "GalbotMotion initialization failed" << std::endl;
    }

    if (nav.init()) {
      std::cout << "GalbotNavigation initialization successful" << std::endl;
    } else {
      std::cout << "GalbotNavigation initialization failed" << std::endl;
    }

    // Wait for data readiness
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!move_to_initial_pose(robot)) {
      throw std::runtime_error("Failed to move to the task initial pose");
    }

    // Cartesian motion duration is an MPS-wide setting in this SDK version.
    // Preserve it and restore it before exit so this example has no lasting
    // effect on later motion-planning clients.
    auto [config_status, current_motion_config] = motion.get_motion_plan_config();
    if (config_status != MotionStatus::SUCCESS) {
      throw std::runtime_error("Failed to read motion-plan configuration");
    }
    original_motion_config = std::make_shared<MotionPlanConfig>(current_motion_config);
    auto slow_motion_config = std::make_shared<MotionPlanConfig>();
    auto trajectory_config = slow_motion_config->create_trajectory_plan_config();
    trajectory_config->set_min_move_time(kArmMinMoveTimeS);
    trajectory_config->set_way_point_plan_expected_time(kArmMinMoveTimeS);
    config_status = motion.set_motion_plan_config(slow_motion_config);
    if (config_status != MotionStatus::SUCCESS) {
      throw std::runtime_error("Failed to apply slow motion-plan configuration");
    }
    slow_motion_config_applied = true;

    // Check localization once. If unavailable, Pick still runs and navigation
    // between Pick and Place is skipped.
    bool navigation_enabled = false;
    try {
      navigation_enabled = nav.is_localized();
    } catch (const std::exception& e) {
      std::cout << "⚠️ Navigation status check failed: " << e.what() << std::endl;
    }
    if (navigation_enabled) {
      std::cout << "✅ Robot is localized; Place navigation is enabled." << std::endl;
    } else {
      std::cout << "⚠️ Robot is not localized; navigation will be skipped. This run demonstrates Pick and Place "
                   "at the current position."
                << std::endl;
    }

    std::cout << std::endl;

    // Detect the target before Pick.
    auto object_pose_base = detect_object(robot);
    std::cout << std::endl;

    if (object_pose_base.size() != 7) {
      std::cout << "Skipping pick_and_place: invalid object pose (expected 7 values)" << std::endl;
    } else {
      // Execute Pick, navigate, and then Place.
      if (!pick_and_place(robot, nav, motion, object_pose_base, navigation_enabled)) {
        throw std::runtime_error("Pick-and-place execution failed");
      }
    }
    std::cout << std::endl;

  } catch (const std::exception& e) {
    std::cout << "Exception occurred: " << e.what() << std::endl;
  }

  if (slow_motion_config_applied && original_motion_config) {
    const MotionStatus restore_status = motion.set_motion_plan_config(original_motion_config);
    if (restore_status != MotionStatus::SUCCESS) {
      std::cout << "⚠️ Failed to restore the original motion-plan configuration: status="
                << (int) restore_status << std::endl;
    }
  }

  // Cleanup
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  std::cout << "Resource release successful" << std::endl;

  return 0;
}
