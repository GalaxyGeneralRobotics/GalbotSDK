// This tutorial demonstrates a complete pick-and-place task for a standard application scenario.
// It integrates navigation, perception, motion planning, and robot control, and can be used as a
// reference implementation for standard applications.
// Before the task starts, the robot moves to the known 21-joint initial pose defined by this example.
// After picking, the robot navigates to its current map pose with the X coordinate offset by 0.1 m,
// then performs placement.
// The final grasp approach and place descent use move_line; other end-effector motions use
// set_end_effector_pose.

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
constexpr double kGraspTransitionOffsetM = 0.05;
const std::vector<double> kPlacePoseBase = {0.4, 0.3, 0.7, 0.0, 0.0, 0.0, 1.0};
// Placeholder grasp pose for this runnable tutorial. Replace it with the
// base-frame pose produced by the application's perception module.
const std::vector<double> kGraspPoseBasePlaceholder = {0.4, 0.3, 0.65, 0.0, 0.0, 0.0, 1.0};
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
    0.0, 0.0, 1.90, -1.46, -0.54, -1.96, 0.0, -0.4, 0.0, -1.90, 1.46, 0.54, 1.96, 0.0, 0.4, 0.0,
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
      kInitialUpperBodyJointPositions, {}, kInitialUpperBodyJointNames, true, 0.2, 20.0);
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
  std::string source_frame = "left_arm_camera_color_optical_frame";
  std::string target_frame = "base_link";

  auto [base_to_cam, success] = robot.get_transform(target_frame, source_frame);

  if (!success || base_to_cam.empty()) {
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
    // This example uses the left-arm RGB and depth cameras.
    auto rgb_image_data = robot.get_rgb_data(SensorType::LEFT_ARM_CAMERA);
    auto depth_data = robot.get_depth_data(SensorType::LEFT_ARM_DEPTH_CAMERA);

    // Match Python: need both RGB and depth before running detection (otherwise NameError there).
    if (!rgb_image_data) {
      std::cout << "No rgb image data!" << std::endl;
      return k_zero_pose;
    }
    std::cout << "Get rgb image success" << std::endl;

    if (!depth_data) {
      std::cout << "No depth_data!" << std::endl;
      return k_zero_pose;
    }
    std::cout << "Get depth data success" << std::endl;

    auto img_ptr = rgb_image_data->convert_to_cv2_mat();
    auto depth_ptr = depth_data->convert_to_cv2_mat();
    if (!img_ptr || img_ptr->empty()) {
      std::cout << "Failed to decode rgb image" << std::endl;
      return k_zero_pose;
    }
    if (!depth_ptr || depth_ptr->empty()) {
      std::cout << "Failed to decode depth image" << std::endl;
      return k_zero_pose;
    }

    cv::Mat img = *img_ptr;
    cv::Mat depth_img = *depth_ptr;

    // Placeholder perception result in the OpenCV camera frame (x right, y down, z forward).
    // Replace this pose with the output of an application-specific RGB-D detector.
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
    // Attach tool to end effector
    auto status = motion.attach_tool("left_arm", "galbot_gripper");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to attach tool: status=" << (int) status << std::endl;
      return false;
    } else {
      std::cout << "✅ Successfully attached tool: status=" << (int) status << std::endl;
    }

    // The target poses represent the attached gripper TCP, not the bare arm flange.
    // Tool pose mode makes set_end_effector_pose control the attached tool TCP.
    auto motion_params = std::make_shared<Parameter>();
    motion_params->set_direct_execute(true);
    motion_params->set_blocking(true);
    motion_params->set_timeout(20.0);
    motion_params->set_tool_pose(true);
    motion_params->set_check_collision(true);
    motion_params->set_reference_frame("base_link");
    motion_params->set_actuate("with_chain_only");

    // Open left gripper
    auto gripper_status = robot.set_gripper_command("left_gripper", 0.1, 0.05, 10, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "✅ Successfully set left gripper width to 0.1m: status=" << (int) gripper_status << std::endl;

    // This fixed grasp pose is only a placeholder for the runnable tutorial.
    // A real application should use object_pose_base from perception instead.
    std::cout << "Perception pose: [";
    for (auto val : object_pose_base)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    std::vector<double> grasp_pose = kGraspPoseBasePlaceholder;
    std::vector<double> pre_grasp_pose = grasp_pose;
    pre_grasp_pose[0] -= kGraspTransitionOffsetM;
    std::vector<double> lift_grasp_pose = grasp_pose;
    lift_grasp_pose[2] += kGraspTransitionOffsetM;
    std::vector<double> retreat_pose = lift_grasp_pose;
    retreat_pose[0] -= kGraspTransitionOffsetM;

    std::cout << "pre_grasp_pose: [";
    for (auto val : pre_grasp_pose)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    status = motion.set_end_effector_pose(pre_grasp_pose, "left_arm", "base_link", nullptr, true, true, 20.0,
                                          motion_params);
    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to execute pre_grasp pose command: status=" << (int) status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully executed pre_grasp pose command" << std::endl;

    // Use move_line for the final 5 cm straight-line approach to the object.
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

    // Close gripper to grasp object
    gripper_status = robot.set_gripper_command("left_gripper", 0.02, 0.05, 10, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "✅ Successfully closed left gripper: status=" << (int) gripper_status << std::endl;

    std::cout << "lift_grasp_pose: [";
    for (auto val : lift_grasp_pose)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    status = motion.set_end_effector_pose(lift_grasp_pose, "left_arm", "base_link", nullptr, true, true, 20.0,
                                          motion_params);
    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to execute lift_grasp pose command: status=" << (int) status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully executed lift_grasp pose command" << std::endl;

    std::cout << "retreat_pose: [";
    for (auto val : retreat_pose)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    status = motion.set_end_effector_pose(retreat_pose, "left_arm", "base_link", nullptr, true, true, 20.0,
                                          motion_params);
    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to execute retreat pose command: status=" << (int) status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully executed retreat pose command" << std::endl;

    // Return to the initial left-arm joint pose before navigation.
    const std::vector<std::string> left_arm_joint_names(kInitialUpperBodyJointNames.begin() + 2,
                                                        kInitialUpperBodyJointNames.begin() + 9);
    const std::vector<double> left_arm_joint_positions(kInitialUpperBodyJointPositions.begin() + 2,
                                                       kInitialUpperBodyJointPositions.begin() + 9);
    const ControlStatus left_arm_status =
        robot.set_joint_positions(left_arm_joint_positions, {}, left_arm_joint_names, true, 0.2, 20.0);
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

    // Move above the place point, then descend 5 cm in a straight line.
    std::vector<double> place_pose = kPlacePoseBase;
    std::vector<double> pre_place_pose = place_pose;
    pre_place_pose[2] += kGraspTransitionOffsetM;
    std::cout << "pre_place_pose: [";
    for (auto val : pre_place_pose)
      std::cout << val << " ";
    std::cout << "]" << std::endl;
    status = motion.set_end_effector_pose(pre_place_pose, "left_arm", "base_link", nullptr, true, true, 20.0);
    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to execute pre_place pose command: status=" << (int) status << std::endl;
      return false;
    }
    std::cout << "✅ Successfully executed pre_place pose command" << std::endl;

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

    // Release target
    gripper_status = robot.set_gripper_command("left_gripper", 0.1, 0.05, 10, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "✅ Successfully released left gripper: status=" << (int) gripper_status << std::endl;

    // Return the left arm to its initial joint pose after placing the object.
    const ControlStatus post_place_arm_status =
        robot.set_joint_positions(left_arm_joint_positions, {}, left_arm_joint_names, true, 0.2, 20.0);
    if (post_place_arm_status != ControlStatus::SUCCESS) {
      std::cout << "❌ Failed to restore the left-arm pose after placing: status=" << (int) post_place_arm_status
                << std::endl;
      return false;
    }
    std::cout << "✅ Successfully restored the left-arm pose after placing: status=" << (int) post_place_arm_status
              << std::endl;

    // Detach tool from end effector
    status = motion.detach_tool("left_arm");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (status != MotionStatus::SUCCESS) {
      std::cout << "❌ Failed to detach tool: status=" << (int) status << std::endl;
    } else {
      std::cout << "✅ Successfully detached tool: status=" << (int) status << std::endl;
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
  auto& robot = GalbotRobot::get_instance(MachineType::G1);
  auto& motion = GalbotMotion::get_instance(MachineType::G1);
  auto& nav = GalbotNavigation::get_instance(MachineType::G1);

  try {
    // Enable sensors
    std::unordered_set<SensorType> enable_sensor_set = {SensorType::LEFT_ARM_CAMERA,
                                                        SensorType::LEFT_ARM_DEPTH_CAMERA};

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

  // Cleanup
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  std::cout << "Resource release successful" << std::endl;

  return 0;
}
