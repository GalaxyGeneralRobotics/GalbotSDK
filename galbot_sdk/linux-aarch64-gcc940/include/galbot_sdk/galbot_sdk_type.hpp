/**
 * @file type.hpp
 * @brief Core type definitions for Galbot SDK
 *
 * This file defines fundamental data structures, enumerations, and types used throughout
 * the Galbot SDK, including robot states, sensor data, motion planning configurations,
 * and geometric primitives for robotic manipulation and navigation.
 *
 * @author Galbot SDK Team
 * @version 1.5.1
 * @copyright Copyright (c) 2023-2026 Galbot. All rights reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

/**
 * @namespace galbot
 * @brief Root namespace for Galbot robotics software
 */
namespace galbot {

/**
 * @namespace galbot::sdk
 * @brief Galbot Software Development Kit namespace
 */
namespace sdk {

/**
 * @brief Galbot G1 joint-group names.
 *
 * A "joint group" is the SDK's primary control/planning unit, not a single joint:
 * - Kinematic-consistent control: commands are validated and executed per chain/end-effector group.
 * - Deterministic command ordering: `joint_groups` are expanded to concrete `joint_names` in group order.
 * - Group-level behavior: each group has its own active/passive attribute and execution tolerance.
 *
 * Recommended usage:
 * - Use constants in this struct when filling API parameters such as `joint_groups`.
 * - If exact joint names are needed, query them at runtime via
 *   `get_joint_names(true, {group_name})` instead of hard-coding.
 * - In APIs that accept both `joint_groups` and `joint_names`, `joint_names` takes precedence.
 */
struct G1JointGroup {
  using Name = const char*;
  static inline constexpr Name HEAD = "head";           /**< Head chain. Default joints:
                                                            `head_joint1`, `head_joint2`. Typical use: gaze/camera orientation. */
  static inline constexpr Name LEFT_ARM = "left_arm";   /**< Left 7-DoF arm chain. Default joints:
                                                            `left_arm_joint1` ... `left_arm_joint7`.
                                                            Typical use: left-arm reaching/manipulation. */
  static inline constexpr Name RIGHT_ARM = "right_arm"; /**< Right 7-DoF arm chain. Default joints:
                                                            `right_arm_joint1` ... `right_arm_joint7`.
                                                            Typical use: right-arm reaching/manipulation. */
  static inline constexpr Name LEFT_GRIPPER =
      "left_gripper"; /**< Left gripper chain. Default joint:
                          `left_gripper_joint1`.
                          Typical use: left gripper open/close and grasp width. */
  static inline constexpr Name RIGHT_GRIPPER =
      "right_gripper";                      /**< Right gripper chain. Default joint:
                                                `right_gripper_joint1`.
                                                Typical use: right gripper open/close and grasp width. */
  static inline constexpr Name LEG = "leg"; /**< Leg chain. Default joints:
                                                `leg_joint1` ... `leg_joint5`.
                                                Typical use: lower-body posture/locomotion-related body control. */
  static inline constexpr Name CHASSIS =
      "chassis"; /**< Chassis mechanism group (passive in joint-position control).
                     Default joints: `chassis_joint1` ... `chassis_joint4`.
                     Typical use: chassis state grouping; base motion should use base APIs. */
  static inline constexpr Name LEFT_SUCTION_CUP = "left_suction_cup"; /**< Left suction-cup end-effector group.
                                                                          Default joint: `left_suction_cup_joint1`.
                                                                          Typical use: vacuum pick/place on left arm. */
  static inline constexpr Name RIGHT_SUCTION_CUP =
      "right_suction_cup"; /**< Right suction-cup end-effector group.
                               Default joint: `right_suction_cup_joint1`.
                               Typical use: vacuum pick/place on right arm. */
  static inline constexpr Name LEFT_DEXHAND =
      "left_dexhand"; /**< Left dexterous hand group. Default joints:
                          `left_dexhand_joint1` ... `left_dexhand_joint6`.
                          Typical use: multi-finger dexterous manipulation (left). */
  static inline constexpr Name RIGHT_DEXHAND =
      "right_dexhand"; /**< Right dexterous hand group. Default joints:
                           `right_dexhand_joint1` ... `right_dexhand_joint6`.
                           Typical use: multi-finger dexterous manipulation (right). */
};

/**
 * @brief Galbot S1 joint-group names.
 *
 * A "joint group" is the SDK's primary control/planning unit, not a single joint:
 * - Kinematic-consistent control: commands are validated and executed per chain/end-effector group.
 * - Deterministic command ordering: `joint_groups` are expanded to concrete `joint_names` in group order.
 * - Group-level behavior: each group has its own active/passive attribute and execution tolerance.
 *
 * Recommended usage:
 * - Use constants in this struct when filling API parameters such as `joint_groups`.
 * - If exact joint names are needed, query them at runtime via
 *   `get_joint_names(true, {group_name})` instead of hard-coding.
 * - In APIs that accept both `joint_groups` and `joint_names`, `joint_names` takes precedence.
 */
struct S1JointGroup {
  using Name = const char*;
  static inline constexpr Name HEAD = "head";           /**< Head chain. Default joints:
                                                            `head_joint1`, `head_joint2`. Typical use: gaze/camera orientation. */
  static inline constexpr Name LEFT_ARM = "left_arm";   /**< Left 7-DoF arm chain. Default joints:
                                                            `left_arm_joint1` ... `left_arm_joint7`.
                                                            Typical use: left-arm manipulation. */
  static inline constexpr Name RIGHT_ARM = "right_arm"; /**< Right 7-DoF arm chain. Default joints:
                                                            `right_arm_joint1` ... `right_arm_joint7`.
                                                            Typical use: right-arm manipulation. */
  static inline constexpr Name LEFT_GRIPPER =
      "left_gripper"; /**< Left gripper chain. Default joint:
                          `left_gripper_joint1`. Typical use: grasp width control. */
  static inline constexpr Name RIGHT_GRIPPER =
      "right_gripper"; /**< Right gripper chain. Default joint:
                           `right_gripper_joint1`. Typical use: grasp width control. */
  static inline constexpr Name TORSO =
      "torso";                                                    /**< Torso chain. Default joint:
                                                                      `torso_base_joint`. Typical use: upper-body height/pitch adjustment. */
  static inline constexpr Name SWERVE_CHASSIS = "swerve_chassis"; /**< Swerve chassis mechanism group (passive in
                                                                      joint-position control). Default joints:
                                                                      `Wheel_1_direction_joint`, `Wheel_1_drive_joint`,
                                                                      `Wheel_2_direction_joint`, `Wheel_2_drive_joint`,
                                                                      `Wheel_3_direction_joint`, `Wheel_3_drive_joint`,
                                                                      `Wheel_4_direction_joint`, `Wheel_4_drive_joint`.
                                                                      Typical use: chassis state grouping; base motion
                                                                      should use base APIs. */
  static inline constexpr Name LEFT_CAMERA = "left_camera"; /**< Camera mount group placeholder (no active body joints).
                                                               Typical use: semantic grouping for sensor-side logic. */
  static inline constexpr Name RIGHT_CAMERA =
      "right_camera"; /**< Camera mount group placeholder (no active body joints).
                         Typical use: semantic grouping for sensor-side logic. */
};

/// @brief Sensor type enumeration describing various sensors on the robot
///
/// Identifies different sensor types available on the robot for perception,
/// localization, and manipulation tasks.
///
/// @robot G1 S1
enum class SensorType {
  /// @brief Head left camera, typically RGB camera for stereo vision
  /// @robot G1 S1
  HEAD_LEFT_CAMERA,

  /// @brief Head right camera, typically RGB camera for stereo vision
  /// @robot G1 S1
  HEAD_RIGHT_CAMERA,

  /// @brief Left arm camera, mounted on left manipulator for visual servoing
  /// @robot G1 S1
  LEFT_ARM_CAMERA,

  /// @brief Right arm camera, mounted on right manipulator for visual servoing
  /// @robot G1 S1
  RIGHT_ARM_CAMERA,

  /// @brief Left arm depth camera, provides RGB-D data for left arm workspace
  /// @robot G1 S1
  LEFT_ARM_DEPTH_CAMERA,

  /// @brief Right arm depth camera, provides RGB-D data for right arm workspace
  /// @robot G1 S1
  RIGHT_ARM_DEPTH_CAMERA,

  /// @brief Left arm infrared camera 1, provides infrared data for left arm workspace
  /// @robot G1 S1
  LEFT_ARM_INFRA_CAMERA_1,

  /// @brief Left arm infrared camera 2, provides infrared data for left arm workspace
  /// @robot G1 S1
  LEFT_ARM_INFRA_CAMERA_2,

  /// @brief Right arm infrared camera 1, provides infrared data for right arm workspace
  /// @robot G1 S1
  RIGHT_ARM_INFRA_CAMERA_1,

  /// @brief Right arm infrared camera 2, provides infrared data for right arm workspace
  /// @robot G1 S1
  RIGHT_ARM_INFRA_CAMERA_2,

  /// @brief G1 Base LiDAR
  /// @robot G1
  BASE_LIDAR,

  /// @brief S1 Head LiDAR (front), mounted on head for forward perception
  /// @robot S1
  HEAD_LIDAR,

  /// @brief S1 Back LiDAR (rear), mounted on rear for backward perception
  /// @robot S1
  BACK_LIDAR,

  /// @brief S1 Chassis LiDAR (front), mounted on chassis for forward perception
  /// @robot S1
  CHASSIS_LIDAR,

  /// @brief S1 Head LiDAR IMU (Inertial Measurement Unit)
  /// @robot S1
  HEAD_IMU,

  /// @brief S1 Back LiDAR IMU (Inertial Measurement Unit)
  /// @robot S1
  BACK_IMU,

  /// @brief Chassis IMU (Inertial Measurement Unit)
  /// @robot G1 S1
  CHASSIS_IMU,

  /// @brief G1 Torso IMU (Inertial Measurement Unit), measures acceleration and angular velocity
  /// @robot G1
  TORSO_IMU,

  /// @brief G1 LiDAR IMU (Inertial Measurement Unit)
  /// @robot G1
  LIDAR_IMU,

  /// @brief Base ultrasonic sensor array, for proximity detection and collision avoidance
  /// @robot G1
  BASE_ULTRASONIC,

  /// @brief G1 left-front surround color camera
  /// @robot G1
  LEFT_FRONT_SURROUND_CAMERA,

  /// @brief G1 right-front surround color camera
  /// @robot G1
  RIGHT_FRONT_SURROUND_CAMERA,

  /// @brief G1 left-rear surround color camera
  /// @robot G1
  LEFT_REAR_SURROUND_CAMERA,

  /// @brief G1 right-rear surround color camera
  /// @robot G1
  RIGHT_REAR_SURROUND_CAMERA,

  /// @brief Total number of sensor enumerations (for boundary checking or array sizing)
  /// @robot G1 S1
  SENSOR_NUM
};

/**
 * @brief Chassis ultrasonic sensor probe enumeration (8 directions)
 * @robot G1
 *
 * Identifies individual ultrasonic sensors arranged around the mobile base chassis
 * for omnidirectional obstacle detection and proximity sensing.
 */
enum class UltrasonicType {
  FRONT_LEFT,    /**< Front left ultrasonic sensor */
  FRONT_RIGHT,   /**< Front right ultrasonic sensor */
  RIGHT_LEFT,    /**< Right side front ultrasonic sensor */
  RIGHT_RIGHT,   /**< Right side rear ultrasonic sensor */
  BACK_LEFT,     /**< Back left ultrasonic sensor */
  BACK_RIGHT,    /**< Back right ultrasonic sensor */
  LEFT_LEFT,     /**< Left side front ultrasonic sensor */
  LEFT_RIGHT,    /**< Left side rear ultrasonic sensor */
  ULTRASONIC_NUM /**< Total number of ultrasonic sensors (for boundary checking or array sizing) */
};

/**
 * @brief Supported robot machine types.
 *
 * This enumeration defines the different robot platforms or machine types
 * supported by the Galbot SDK. Clients can use these values to specify which
 * robot model they are working with, particularly for factory methods that
 * return platform-specific implementations. Keeping the enumeration in the
 * common type definitions ensures consistency across the SDK while hiding
 * implementation details in the respective modules.
 */
enum class MachineType {
  G1, /**< Galbot G1 humanoid robot platform */
  S1  /**< Galbot S1 humanoid robot platform */
};

/**
 * @brief Dexterous hand model type.
 *
 * The SDK uses this enumeration to route dexhand commands and state queries to
 * the correct implementation. Inspire and BrainCo dexhands share the standard
 * joint command/state path. Sharpa dexhands use a dedicated 22-joint topic
 * interface; full state is returned in DexhandState (including force sensors).
 * Linker Hand L20 uses a dedicated 16-joint path keyed by the dexhand group;
 * like Sharpa, its joint commands and feedback are in radians.
 */
enum class DexHandType {
  INSPIRE,    /**< Inspire dexterous hand */
  BRAINCO,    /**< BrainCo dexterous hand */
  SHARPA,     /**< Sharpa dexterous hand */
  LINKER_L20  /**< Linker Hand L20 dexterous hand */
};

/**
 * @brief Navigation task execution status enumeration
 *
 * Defines the possible outcomes when executing navigation commands such as
 * moving to a goal position or following a path.
 */
enum class NavigationStatus {
  SUCCESS,         /**< Execution succeeded, navigation task completed as expected */
  FAIL,            /**< Execution failed due to unspecified error */
  TIMEOUT,         /**< Execution timeout, task did not complete within allowed time */
  INVALID_INPUT,   /**< Input parameters do not meet requirements or are out of valid range */
  MODE_ERR,        /**< Current mode does not support this operation */
  COMM_ERR,        /**< Communication error occurred during execution */
  WAIT_INITIALIZED /**< Waiting for system initialization, navigation system not ready */
};

/**
 * @brief Navigation task state.
 *
 * This enum describes the current state of a navigation task that has been
 * submitted to the SDK. It is used by both legacy navigation polling and the
 * asynchronous target status query API.
 */
enum class NavigationTaskStatus {
  UNKNOWN = 0,          /**< State has not been reported yet, or the task id is unknown */
  RUNNING = 1,          /**< The task is still executing */
  SUCCESS = 2,          /**< The task finished successfully */
  FAILED = 3,           /**< The task finished with an error */
  INTERRUPTED = 4,      /**< The task was interrupted by a later request or a stop command */
  OCCUPIED = 5,         /**< The goal area or path is occupied */
  COLLISION = 6,        /**< A collision condition was detected */
  CLOSE_TO_OBSTACLE = 7 /**< The robot is too close to an obstacle */
};

/**
 * @brief Snapshot of a navigation task.
 *
 * This object is returned by task status query APIs. It contains the task id,
 * the latest known task state, and a short human-readable message.
 */
struct NavigationTaskSnapshot {
    std::string task_id; /**< Identifier of the navigation task. */
    NavigationTaskStatus status = NavigationTaskStatus::UNKNOWN; /**< Latest known state of the task. */
    std::string msg; /**< Short human-readable message associated with the task state. */
};

/**
 * @brief Handle returned when a navigation task is submitted asynchronously.
 *
 * The handle identifies the task and tells whether the request was accepted by
 * the navigation service. The execution state should be queried separately
 * using task_id.
 */
struct TaskHandle {
    std::string task_id; /**< Identifier of the submitted navigation task. */
    bool request_sent = false; /**< Whether the navigation service accepted the request. */
    std::string msg; /**< Short human-readable message associated with the submission result. */
};

/**
 * @brief Control command execution status enumeration
 *
 * Represents the execution status of robot control commands, including
 * joint control, end-effector control, and other motion control operations.
 */
enum class ControlStatus {
  SUCCESS,           /**< Execution succeeded, command completed with valid result */
  TIMEOUT,           /**< Execution timeout, task not completed within specified time limit */
  FAULT,             /**< Fault occurred, system detected anomaly and aborted execution */
  INVALID_INPUT,     /**< Input parameters invalid or not meeting interface requirements */
  INIT_FAILED,       /**< Initialization failed, internal communication or dependent component creation failed */
  IN_PROGRESS,       /**< Command is executing but has not reached target state */
  STOPPED_UNREACHED, /**< Stopped during execution without reaching target position or state */
  DATA_FETCH_FAILED, /**< Data retrieval failed during operation, unable to read required state */
  PUBLISH_FAIL,      /**< Control or state data publication failed, command may not be transmitted */
  COMM_DISCONNECTED, /**< Communication connection lost, cannot continue execution */
  STATUS_NUM         /**< Total number of status enumerations (for boundary checking or array sizing) */
};

/**
 * @brief Sensor execution status enumeration
 *
 * Represents the status of sensor data acquisition and processing operations,
 * applicable to cameras, lidar, IMU, force sensors, and other sensor types.
 */
enum class SensorStatus {
  SUCCESS,           /**< Execution succeeded, sensor data valid and operation completed */
  TIMEOUT,           /**< Execution timeout, data acquisition or operation not completed within specified time limit */
  FAULT,             /**< Fault occurred, sensor detected anomaly and cannot continue normal operation */
  INVALID_INPUT,     /**< Input parameters invalid or not meeting interface requirements */
  INIT_FAILED,       /**< Initialization failed, internal communication or dependent component creation failed */
  IN_PROGRESS,       /**< Operation in progress but not yet completed */
  STOPPED_UNREACHED, /**< Stopped during execution without completing expected operation */
  DATA_FETCH_FAILED, /**< Data acquisition or reading failed, sensor may be disconnected or malfunctioning */
  PUBLISH_FAIL,      /**< Data transmission or reporting failed, unable to publish sensor data */
  COMM_DISCONNECTED, /**< Sensor communication connection lost, no data available */
  STATUS_NUM         /**< Total number of status enumerations (for boundary checking or array sizing) */
};

/**
 * @brief Robot motion execution status enumeration
 *
 * Represents the execution status of robot motion commands, including
 * trajectory following, pose reaching, and other motion planning operations.
 */
enum class MotionStatus {
  SUCCESS,             /**< Execution succeeded, motion reached expected target position/pose */
  TIMEOUT,             /**< Execution timeout, motion not completed within specified time limit */
  FAULT,               /**< Fault occurred, motion cannot continue due to hardware or safety issue */
  INVALID_INPUT,       /**< Input parameters invalid or not meeting interface requirements */
  INIT_FAILED,         /**< Internal initialization failed, communication component or resource creation failed */
  IN_PROGRESS,         /**< Motion in progress but has not reached target yet */
  STOPPED_UNREACHED,   /**< Stopped during motion without reaching target position/pose */
  DATA_FETCH_FAILED,   /**< Data retrieval failed, e.g., sensor or state reading failure */
  PUBLISH_FAIL,        /**< Data transmission or command delivery failed, motion command may not be executed */
  COMM_DISCONNECTED,   /**< Communication disconnected or control node unavailable */
  STATUS_NUM,          /**< Total number of status enumerations (for boundary checking or array sizing) */
  UNSUPPORTED_FUNCRION /**< Function not yet supported, called interface or operation not implemented (note: typo in
                          enum name preserved for API compatibility) */
};

/**
 * @brief Log level enumeration
 *
 * Represents the severity level of log messages.
 */
enum class LogLevel {
  TRACE = 0, /**< Trace level, detailed information for debugging */
  DEBUG,     /**< Debug level, diagnostic information for developers */
  INFO,      /**< Info level, general operational messages */
  WARN,      /**< Warn level, potentially harmful situations */
  ERROR,     /**< Error level, error events that might still allow the application to continue running */
  CRITICAL   /**< Critical level, severe error events that lead to application termination */
};

/**
 * @brief Logger configuration structure
 *
 * Defines the configuration parameters for the logging system, including
 * file settings, log levels, and output options.
 */
struct LoggerConfig {
  std::string path = "";      /**< Directory path for log files. Default: ~/galbot_sdk_log/user_log */
  std::string file_name = ""; /**< Log file name. Default: <process_name>_<current_time>_<pid>_<thread_id>.log */
  uint64_t file_max_size = 10 * 1024 * 1024; /**< Maximum size of a single log file (bytes) */
  uint64_t file_max_num = 5;                 /**< Number of log files to retain in rotation */
  LogLevel level = LogLevel::INFO;           /**< Minimum log level to record */
  bool console_output = false;               /**< Flag to enable or disable console output */
};

/**
 * @brief Robot trajectory execution status enumeration
 *
 * Represents the real-time execution status when the robot follows a
 * pre-planned trajectory consisting of multiple waypoints.
 */
enum class TrajectoryControlStatus {
  INVALID_INPUT,     /**< Input parameters do not meet requirements, trajectory cannot be executed */
  RUNNING,           /**< Trajectory is currently executing, not yet completed */
  COMPLETED,         /**< Trajectory execution completed successfully, reached final target point */
  STOPPED_UNREACHED, /**< Stopped during trajectory execution without reaching endpoint */
  ERROR,             /**< Error occurred, trajectory execution cannot continue */
  DATA_FETCH_FAILED, /**< Execution data retrieval failed, e.g., joint state or sensor feedback unavailable */
  STATUS_NUM         /**< Total number of status enumerations (for boundary checking or array sizing) */
};

/**
 * @brief Force sensor enumeration describing robot wrist force sensors
 *
 * Identifies force/torque sensors mounted at the robot's wrist joints for
 * force-controlled manipulation and contact detection.
 *
 * @robot G1
 */
enum class GalbotOneFoxtrotSensor {
  LEFT_WRIST_FORCE,  /**< Left wrist force/torque sensor, typically 6-axis (3 forces + 3 torques) */
  RIGHT_WRIST_FORCE, /**< Right wrist force/torque sensor, typically 6-axis (3 forces + 3 torques) */
  FORCE_NUM,         /**< Total number of force sensor enumerations (for boundary checking or array sizing) */
};

/**
 * @brief String constants for G1 controller names
 *
 * Defines the controller names supported by the G1 robot model.
 */
struct G1ControllerName {
  using Name = const char*;
  static inline constexpr Name CHASSIS_POSE_CTRL = "chassis_pose_ctrl";       /**< Chassis pose controller */
  static inline constexpr Name CHASSIS_TWIST_CTRL = "chassis_twist_ctrl";     /**< Chassis twist controller */
  static inline constexpr Name LEG_PVT_BYPASS_CTRL = "leg_pvt_bypass_ctrl";   /**< Leg PVT bypass controller */
  static inline constexpr Name LEG_PVT_CTRL = "leg_pvt_ctrl";                 /**< Leg PVT controller */
  static inline constexpr Name HEAD_PVT_BYPASS_CTRL = "head_pvt_bypass_ctrl"; /**< Head PVT bypass controller */
  static inline constexpr Name HEAD_PVT_CTRL = "head_pvt_ctrl";               /**< Head PVT controller */
  static inline constexpr Name LEFT_ARM_PVT_BYPASS_CTRL =
      "left_arm_pvt_bypass_ctrl";                                       /**< Left arm PVT bypass controller */
  static inline constexpr Name LEFT_ARM_PVT_CTRL = "left_arm_pvt_ctrl"; /**< Left arm PVT controller */
  static inline constexpr Name RIGHT_ARM_PVT_BYPASS_CTRL =
      "right_arm_pvt_bypass_ctrl";                                        /**< Right arm PVT bypass controller */
  static inline constexpr Name RIGHT_ARM_PVT_CTRL = "right_arm_pvt_ctrl"; /**< Right arm PVT controller */
  static inline constexpr Name LEFT_GRIPPER_CTRL = "left_gripper_ctrl";   /**< Left gripper controller */
  static inline constexpr Name RIGHT_GRIPPER_CTRL = "right_gripper_ctrl"; /**< Right gripper controller */
  static inline constexpr Name LEFT_DEXHAND_CTRL = "left_dexhand_ctrl";   /**< Left dexhand controller */
  static inline constexpr Name RIGHT_DEXHAND_CTRL = "right_dexhand_ctrl"; /**< Right dexhand controller */
  static inline constexpr Name CONTROLLER_NAME_NUM =
      "CONTROLLER_NAME_NUM"; /**< Sentinel value for invalid controller name */
};

/**
 * @brief String constants for S1 controller names
 *
 * Defines the controller names supported by the S1 robot model.
 */
struct S1ControllerName {
  using Name = const char*;
  static inline constexpr Name SWERVE_CHASSIS_POSE_CTRL =
      "swerve_chassis_pose_ctrl"; /**< Swerve chassis pose controller */
  static inline constexpr Name SWERVE_CHASSIS_TWIST_CTRL =
      "swerve_chassis_twist_ctrl";                                        /**< Swerve chassis twist controller */
  static inline constexpr Name ELEVATOR_CTRL = "elevator_ctrl";           /**< Elevator controller */
  static inline constexpr Name HEAD_PVT_CTRL = "head_pvt_ctrl";           /**< Head PVT controller */
  static inline constexpr Name LEFT_ARM_PVT_CTRL = "left_arm_pvt_ctrl";   /**< Left arm PVT controller */
  static inline constexpr Name RIGHT_ARM_PVT_CTRL = "right_arm_pvt_ctrl"; /**< Right arm PVT controller */
  static inline constexpr Name LEFT_GRIPPER_CTRL = "left_gripper_ctrl";   /**< Left gripper controller */
  static inline constexpr Name RIGHT_GRIPPER_CTRL = "right_gripper_ctrl"; /**< Right gripper controller */
  static inline constexpr Name LEFT_CAMERA_CTRL = "left_camera_ctrl";     /**< Left camera controller */
  static inline constexpr Name RIGHT_CAMERA_CTRL = "right_camera_ctrl";   /**< Right camera controller */
};

/**
 * @brief Device information structure
 *
 * Describes basic information about the robot or module, used for device management,
 * logging, diagnostics, and maintenance tracking.
 */
struct DeviceInfo {
  std::string model;            /**< Device model name or identifier */
  std::string serial_number;    /**< Unique serial number for device identification */
  std::string firmware_version; /**< System firmware version string (e.g., "1.2.3") */
  std::string hardware_version; /**< Hardware version or revision number */
  std::string manufacturer;     /**< Manufacturer name or company identifier */
};

/**
 * @brief Ultrasonic sensor data structure
 * @robot G1
 *
 * Contains a single ultrasonic distance measurement with timestamp.
 */
struct UltrasonicData {
  int64_t timestamp_ns; /**< Measurement timestamp in nanoseconds since epoch */
  double distance;      /**< Measured distance to nearest obstacle (meters) */
};

/**
 * @brief Robot kinematic chain type enumeration
 *
 * Identifies different kinematic chains in the robot structure for forward/inverse
 * kinematics calculations and motion planning.
 */
enum class ChainType {
  HEAD,      /**< Head kinematic chain, from base/torso to head end-effector */
  LEFT_ARM,  /**< Left arm kinematic chain, from base/torso to left end-effector */
  RIGHT_ARM, /**< Right arm kinematic chain, from base/torso to right end-effector */
  LEG,       /**< Leg kinematic chain, for legged locomotion */
  TORSO,     /**< Torso kinematic chain, connects base to upper body */
  CHAIN_NUM  /**< Total number of kinematic chains (for boundary checking or array sizing) */
};

/**
 * @brief Single joint control command
 *
 * Specifies desired motion parameters for a single robot joint in a trajectory or control command.
 */
struct JointCommand {
  double position;     /**< Desired joint position (radians) */
  double velocity;     /**< Desired joint velocity (radians/second) */
  double acceleration; /**< Desired joint acceleration (radians/second²) */
  double effort;       /**< Desired joint torque/effort (Newton-meters) */
};

/**
 * @brief Single trajectory point
 *
 * Represents a waypoint in a robot trajectory, specifying joint states at a particular time.
 *
 * @note The `joint_command_vec` order must match the joint order defined by `Trajectory.joint_names`
 *       or `Trajectory.joint_groups` (expanded in order). Mismatched order will cause undefined behavior.
 */
struct TrajectoryPoint {
  double time_from_start_second;               /**< Time from trajectory start (seconds) */
  std::vector<JointCommand> joint_command_vec; /**< List of joint commands for all joints at this waypoint.
                                                     Order must match joint_names or joint_groups expansion order. */
};

/**
 * @brief Joint trajectory
 *
 * Represents a complete robot trajectory consisting of multiple waypoints over time.
 *
 * @note Both `joint_groups` and `joint_names` MUST NOT be empty at the same time.
 *       If both are empty, the function returns ControlStatus::INVALID_INPUT.
 *       This restriction ensures deterministic joint ordering and prevents subtle
 *       bugs caused by implicit internal default order (which may change between SDK versions).
 *
 * Usage:
 * - Preferred: Set `joint_groups` with semantic group names (e.g., `{"left_arm", "right_arm"}`).
 * - Fine-grained: Set `joint_names` with explicit joint names in the exact order of your point data.
 * - `joint_names` takes precedence over `joint_groups` if both are set.
 *
 * Example:
 * @code
 * Trajectory traj;
 * traj.joint_groups = {"left_arm", "right_arm"};  // Recommended
 * // Or: traj.joint_names = {"left_arm_joint1", ..., "right_arm_joint1", ...};
 * @endcode
 */
struct Trajectory {
  std::vector<TrajectoryPoint> points;   /**< Ordered list of trajectory waypoints.
                                             Each point's joint_command_vec must match the joint order
                                             defined by either joint_names or joint_groups.
                                             @note The joint order is determined by: (1) joint_names if non-empty,
                                             otherwise (2) joint_groups expanded in order (e.g., {"head","left_arm"}
                                             expands to head_joint1, head_joint2, left_arm_joint1, ... left_arm_joint7). */
  std::vector<std::string> joint_groups; /**< Joint-group names used to expand target joints in-order.
                                             Example: `{"head","left_arm"}` maps to
                                             `head_joint1, head_joint2, left_arm_joint1 ... left_arm_joint7`.
                                             @note Cannot be empty when joint_names is also empty. */
  std::vector<std::string> joint_names;  /**< Explicit joint names. When non-empty, takes precedence over
                                             joint_groups and is validated as active joints only.
                                             @note Cannot be empty when joint_groups is also empty. */
};

/**
 * @brief Error information
 *
 * Describes an error from a single module or component, including error code and
 * human-readable description for debugging and diagnostics.
 */
struct Error {
  std::string commpent;    /**< Fault module or component name (note: field name contains typo but preserved for API
                              compatibility) */
  uint64_t error_code;     /**< Numerical error code for programmatic error handling */
  std::string description; /**< Human-readable error description */

  /**
   * @brief Constructor
   * @param commpent_input Fault module or component name
   * @param error_code_input Numerical error code
   * @param description_input Human-readable error description
   */
  Error(std::string commpent_input, int error_code_input, std::string description_input)
      : commpent(std::move(commpent_input)), error_code(error_code_input), description(std::move(description_input)) {}
};

/**
 * @brief Error information collection
 *
 * Contains a timestamped collection of error messages from multiple modules or components.
 */
struct ErrorInfo {
  int64_t timestamp_ns;         /**< Timestamp when errors were collected (nanoseconds since epoch) */
  std::vector<Error> error_vec; /**< Vector of error messages from various system components */
};

/**
 * @brief 2D point
 *
 * Represents a position in two-dimensional Cartesian space.
 */
struct Point2d {
  double x = 0.0; /**< X coordinate (meters) */
  double y = 0.0; /**< Y coordinate (meters) */
};

/**
 * @brief 2D pose (position + yaw) structure， for mobile base and navigation targets
 *
 * Represents a planar pose in two-dimensional Cartesian space, combining
 * position (translation) and heading angle information.
 * Commonly used for mobile base poses and local navigation targets.
 */
struct Pose2d {
  Point2d position;  /**< Position in 2D space (x, y) in meters */
  double theta = 0.0; /**< Yaw angle in radians */


    /**
   * @brief Default constructor
   *
   * Initializes pose2d at origin (0,0) with zero yaw angle.
   */
  Pose2d() = default;

  /**
   * @brief Initialize Pose2d using a single 3-dimensional vector
   * @tparam T Container type supporting subscript access and size() method
   * @param vec 3D vector: [x, y, theta] where first 2 elements are position (meters) and last element is yaw angle (radians)
   * @throws std::runtime_error if vec size != 3
   */
  template <typename T>
  Pose2d(const T& vec) {
    if (vec.size() != 3) {
      throw std::runtime_error("Pose2d size error, must be 3!");
    }
    position.x = vec[0];
    position.y = vec[1];
    theta = vec[2];
  }
};

/**
 * @brief 3D point
 *
 * Represents a position in three-dimensional Cartesian space.
 */
struct Point {
  double x = 0.0; /**< X coordinate (meters) */
  double y = 0.0; /**< Y coordinate (meters) */
  double z = 0.0; /**< Z coordinate (meters) */
};

/**
 * @brief Quaternion
 *
 * Represents a 3D rotation using quaternion representation (x, y, z, w).
 * A unit quaternion has magnitude 1 and represents a valid rotation.
 */
struct Quaternion {
  double x = 0.0; /**< X component of the quaternion vector part */
  double y = 0.0; /**< Y component of the quaternion vector part */
  double z = 0.0; /**< Z component of the quaternion vector part */
  double w = 1.0; /**< W component, scalar part (for identity rotation, w=1 and x=y=z=0) */
};

/**
 * @brief Pose (position + orientation) structure
 *
 * Represents a full 6-DOF (Degrees of Freedom) pose in 3D space, combining
 * position (translation) and orientation (rotation) information.
 * Commonly used for robot end-effector poses, object poses, and coordinate frame transforms.
 */
struct Pose {
  Point position;         /**< Position in 3D space (x, y, z) in meters */
  Quaternion orientation; /**< Orientation as unit quaternion (x, y, z, w) */

  /**
   * @brief Default constructor
   *
   * Initializes pose at origin (0,0,0) with identity rotation.
   */
  Pose() = default;

  /**
   * @brief Initialize Pose using separate position and quaternion containers
   * @tparam T Container type supporting subscript access and size() method (e.g., std::vector<double>,
   * std::array<double>)
   * @param pos 3D position vector [x, y, z] in meters, must have size 3
   * @param quat Quaternion vector [x, y, z, w], must have size 4
   * @throws std::runtime_error if pos size != 3 or quat size != 4
   */
  template <typename T>
  Pose(const T& pos, const T& quat) {
    if (pos.size() != 3 || quat.size() != 4) {
      throw std::runtime_error("Pose size error, pos size must be 3, quat(x,y,z,w) size must be 4!");
    }
    position.x = pos[0];
    position.y = pos[1];
    position.z = pos[2];
    orientation.x = quat[0];
    orientation.y = quat[1];
    orientation.z = quat[2];
    orientation.w = quat[3];
  }

  /**
   * @brief Initialize Pose using a single 7-dimensional vector
   * @tparam T Container type supporting subscript access and size() method
   * @param vec 7D vector: [x, y, z, qx, qy, qz, qw] where first 3 elements are position (meters)
   *            and last 4 elements are quaternion
   * @throws std::runtime_error if vec size != 7
   */
  template <typename T>
  Pose(const T& vec) {
    if (vec.size() != 7) {
      throw std::runtime_error("Pose size error, must be 7!");
    }
    position.x = vec[0];
    position.y = vec[1];
    position.z = vec[2];
    orientation.x = vec[3];
    orientation.y = vec[4];
    orientation.z = vec[5];
    orientation.w = vec[6];
  }
};

/**
 * @brief Axis-aligned box dimensions.
 *
 * Describes the physical size of a box along its local x/y/z axes. Units are meters.
 */
struct BoxSize {
  float length_x; /**< Length of the box along the x-axis (meters) */
  float length_y; /**< Length of the box along the y-axis (meters) */
  float length_z; /**< Length of the box along the z-axis (meters) */
};

/**
 * @brief Bounding box information for navigation obstacle filtering.
 *
 * Used by navigation APIs to add, remove, and query box regions that should be ignored by navigation
 * obstacle fusion, for example carried boxes that should not be treated as obstacles.
 * `box_pose` follows the SDK Pose convention: position plus quaternion stored as x/y/z/w.
 * `box_tag` is the SDK-side numeric identifier and is converted internally to an SDK-marked box name.
 */
struct BoxInfo {
  BoxSize box_size;             /**< Box dimensions in meters along the box local x/y/z axes */
  Pose box_pose;                /**< Box pose relative to parent_link_name, using SDK quaternion order x/y/z/w */
  int box_tag;                  /**< SDK-side numeric box identifier */
  std::string parent_link_name; /**< Parent link/frame name that box_pose is expressed in */
};

/**
 * @brief Actuation type enumeration
 *
 * Specifies which kinematic chains should be actuated during motion planning and execution.
 * This controls whether the robot uses only the target arm, or also involves torso or leg motion.
 */
enum class ActuateType {
  ACTUATE_WITH_CHAIN_ONLY, /**< Actuate only the target joint chain (e.g., arm only), base remains fixed */
  ACTUATE_WITH_TORSO,      /**< Actuate target joint chain and torso for extended workspace */
  ACTUATE_WITH_LEG,        /**< Actuate target joint chain and legs for mobile manipulation */
  ACTUATE_TYPE_NUM         /**< Total number of actuation types (for boundary checking or array sizing) */
};

/**
 * @brief IK solver seed type enumeration
 *
 * Specifies the initialization strategy for inverse kinematics (IK) solvers.
 * Different seed types affect convergence speed and solution quality.
 */
enum class SeedType {
  RANDOM_SEED,             /**< Random seed, generates random initial joint configurations */
  RANDOM_PROGRESSIVE_SEED, /**< Random progressive seed, tries multiple random seeds iteratively (recommended for
                              robustness) */
  USER_DEFINED_SEED,       /**< User-defined seed, uses explicitly provided initial joint configuration */
  SEED_TYPE_NUM            /**< Total number of seed types (for boundary checking or array sizing) */
};

/**
 * @brief Reference frame enumeration
 *
 * Specifies the coordinate frame in which poses, positions, or trajectories are expressed.
 * Note: This is a plain enum (not enum class) for C-style compatibility.
 */
enum ReferenceFrame {
  FRAME_WORLD, /**< World/global coordinate frame, fixed reference frame */
  FRAME_BASE   /**< Robot base coordinate frame, attached to mobile base */
};

/**
 * @brief Motion planning configuration structure
 *
 * Comprehensive configuration for robot motion planning and execution, controlling behavior
 * such as planning mode, collision checking, reference frames, and execution parameters.
 */
struct PlannerConfig {

  bool enable_env_collision_check=false;

  /**
   * @brief Whether to execute trajectory immediately after planning
   *
   * - true: Plan and execute the trajectory in one operation
   * - false: Only plan the trajectory without executing (for preview or validation)
   */
  bool is_direct_execute = false;

  /**
   * @brief Whether to wait synchronously for operation completion
   *
   * - true: Block until planning/execution completes or timeout occurs
   * - false: Return immediately after initiating operation (asynchronous mode)
   */
  bool is_blocking = false;

  /**
   * @brief Timeout duration for blocking operations (seconds)
   *
   * Maximum time to wait for planning or execution completion when is_blocking = true.
   * Default: 20 seconds
   */
  double timeout_second = 20;

  /**
   * @brief Actuation mode specifying which kinematic chains to use
   *
   * Determines whether to use only the target arm, or also involve torso/leg motion
   * for extended workspace or mobile manipulation.
   */
  ActuateType actuate_type = ActuateType::ACTUATE_WITH_CHAIN_ONLY;

  /**
   * @brief Whether target is specified as tool center point (TCP) pose
   *
   * - true: Target is end-effector TCP pose (Cartesian space)
   * - false: Target is joint space configuration
   */
  bool is_tool_pose = false;

  /**
   * @brief Whether target pose is relative to current pose
   *
   * - true: Target pose is relative displacement from current end-effector pose
   * - false: Target pose is absolute in the specified reference frame
   */
  bool is_relative_pose = false;

  /**
   * @brief Whether to enable collision checking during planning
   *
   * - true: Check collisions with obstacles and self-collisions
   * - false: Disable collision checking (use with caution)
   */
  bool is_check_collision = true;

  /**
   * @brief Reference coordinate frame for target pose
   *
   * Specifies the coordinate frame in which target poses are expressed.
   * Common values: "base_link", "world", "odom"
   */
  std::string reference_frame = "base_link";

  /**
   * @brief Initial joint state for planning
   *
   * Specifies starting joint configuration for planning. If empty, uses current robot state.
   * Key: Joint group identifier
   * Value: Vector of joint angles (radians) for that group
   */
  std::unordered_map<std::string, std::vector<double>> joint_state = {};

  /**
   * @brief Whether to plan Cartesian linear path
   *
   * - true: Plan straight-line motion in Cartesian space (end-effector moves in straight line)
   * - false: Plan standard joint-space or task-space trajectory (may not be Cartesian linear)
   */
  bool move_line = false;
};

/**
 * @cond GALBOT_INTERNAL_DOC_HIDDEN
 * @brief Planning task result structure
 *
 * Contains the complete result of a motion planning operation, including success status,
 * generated trajectory, kinematics solutions, and collision information.
 *
 * @deprecated This struct is currently unused in the SDK. Commented out to avoid confusion.
 *             May be re-enabled if motion planning APIs return this type in the future.
 */
// struct PlanTaskResult {
//   std::string task_id;
//   bool success;
//   int error_code;
//   std::string error_message;
//
//   struct Trajectory {
//     std::vector<std::vector<double>> joint_positions;
//     std::vector<double> timestamps;
//   } trajectory;
//
//   std::unordered_map<std::string, std::vector<double>> ik_result;
//   std::unordered_map<std::string, Pose> fk_result;
//   std::vector<double> collision_result;
// };
/** @endcond */

/**
 * @brief Single joint state structure
 *
 * Represents the complete real-time state of a single robot joint, including
 * joint identity and sampling time metadata, plus
 * kinematic quantities (position, velocity, acceleration) and dynamic quantities
 * (torque/effort and motor current).
 */
struct JointState {
  std::string joint_name; /**< Joint name */
  int64_t timestamp_ns; /**< Joint state timestamp (nanoseconds since epoch) */
  double position;     /**< Joint angular position (radians) */
  double velocity;     /**< Joint angular velocity (radians/second) */
  double acceleration; /**< Joint angular acceleration (radians/second²) */
  double effort;       /**< Joint torque/effort (Newton-meters) */
  double current;      /**< Motor current (amperes) */

  /**
   * @brief Default constructor
   *
   * Uses compiler-generated default initialization.
   */
  JointState() = default;

  /**
   * @brief Parameterized constructor
   *
   * Initializes joint state with specified values.
   *
   * @param position_input Joint angular position (radians)
   * @param velocity_input Joint angular velocity (radians/second)
   * @param acceleration_input Joint angular acceleration (radians/second²)
   * @param effort_input Joint torque/effort (Newton-meters)
   * @param current_input Motor current (amperes)
   * @param timestamp_ns_input Joint state timestamp (nanoseconds since epoch)
   * @param joint_name_input Joint name for identifying this sample
   */
  JointState(double position_input, double velocity_input, double acceleration_input, double effort_input,
             double current_input, int64_t timestamp_ns_input, const std::string& joint_name_input = "")
      : joint_name(joint_name_input),
        timestamp_ns(timestamp_ns_input),
        position(position_input),
        velocity(velocity_input),
        acceleration(acceleration_input),
        effort(effort_input),
        current(current_input) {}
};

/**
 * @brief Joint state message structure
 *
 * Timestamped collection of joint states for multiple joints, typically representing
 * a snapshot of the robot's complete joint configuration at one instant.
 */
struct JointStateMessage {
  int64_t timestamp_ns;                    /**< Acquisition timestamp (nanoseconds since epoch) */
  std::vector<JointState> joint_state_vec; /**< Vector of individual joint states */
};

/**
 * @brief Transform message structure
 *
 * Represents a timestamped coordinate frame transformation, consisting of
 * translation and rotation. Commonly used for TF (Transform) trees in robotics.
 */
struct TransformMessage {
  int64_t timestamp_ns; /**< Transform timestamp (nanoseconds since epoch) */
  Point translation;    /**< Translation vector (meters) */
  Quaternion rotation;  /**< Rotation as unit quaternion (x, y, z, w) */
};

/**
 * @brief 3D vector structure
 *
 * Represents a three-dimensional vector, used for forces, torques, velocities,
 * accelerations, and other vectorial quantities.
 */
struct Vector3 {
  double x; /**< X component */
  double y; /**< Y component */
  double z; /**< Z component */
};

/**
 * @brief 6D wrench information (force + torque)
 *
 * Represents a 6-DOF wrench (force and torque) typically measured by a force/torque sensor.
 * Also known as a spatial force or generalized force.
 */
struct EffortInfo {
  int64_t timestamp_ns; /**< Measurement timestamp (nanoseconds since epoch) */
  Vector3 force;        /**< Force vector (Newtons): [fx, fy, fz] */
  Vector3 torque;       /**< Torque vector (Newton-meters): [tx, ty, tz] */
};

/**
 * @brief Force sensor data
 *
 * Contains timestamped force and torque measurements from a 6-axis force/torque sensor,
 * typically mounted at robot wrists or tool interfaces.
 */
struct ForceData {
  int64_t timestamp_ns; /**< Measurement timestamp (nanoseconds since epoch) */
  Vector3 force;        /**< Force vector (Newtons): [fx, fy, fz] */
  Vector3 torque;       /**< Torque vector (Newton-meters): [tx, ty, tz] */
};

/**
 * @brief Full dexterous hand state
 *
 * Contains timestamped joint feedback and, when available (e.g. Sharpa),
 * per-sensor force/torque measurements from the dexhand force topic.
 * sharpa per finger has 22-joint
 */
struct DexhandState {
  int64_t timestamp_ns;                                         /**< State timestamp (nanoseconds since epoch) */
  JointStateMessage joint_state;                                /**< Dexhand joint state message */
  std::unordered_map<std::string, EffortInfo> force_sensor_map; /**< Named force sensor map (Sharpa; empty otherwise) */
};

/**
 * @brief IMU data structure
 *
 * Contains timestamped data from an Inertial Measurement Unit (IMU), including
 * accelerometer, gyroscope, and magnetometer measurements.
 */
struct ImuData {
  int64_t timestamp_ns; /**< Measurement timestamp (nanoseconds since epoch) */
  Vector3 accel;        /**< Linear acceleration (meters/second²): [ax, ay, az] */
  Vector3 gyro;         /**< Angular velocity (radians/second): [ωx, ωy, ωz] */
  Vector3 magnet;       /**< Magnetic field strength (micro-Tesla): [mx, my, mz] */
};

/**
 * @brief Odometry data
 *
 * Contains robot pose and velocity estimates from odometry sources (wheel encoders, IMU fusion, etc.).
 * Used for robot localization and navigation.
 */
struct OdomData {
  int64_t timestamp_ns;                   /**< Odometry timestamp (nanoseconds since epoch) */
  std::array<double, 3> position;         /**< Position [x, y, z] (meters) */
  std::array<double, 4> orientation;      /**< Orientation as quaternion [qx, qy, qz, qw] */
  std::array<double, 3> linear_velocity;  /**< Linear velocity [vx, vy, vz] (meters/second) */
  std::array<double, 3> angular_velocity; /**< Angular velocity [ωx, ωy, ωz] (radians/second) */
};

/**
 * @brief Gripper state
 *
 * Represents the current state of a parallel-jaw gripper, including opening width,
 * motion status, and grasping force.
 */
struct GripperState {
  int64_t timestamp_ns;   /**< State timestamp (nanoseconds since epoch) */
  double width;           /**< Gripper opening width (meters), distance between fingers */
  double velocity;        /**< Gripper closing/opening velocity (meters/second), positive = opening */
  double effort;          /**< Gripper grasping force (Newtons), force applied by fingers */
  bool is_moving = false; /**< Motion flag from a movement window: false if no effective movement is observed within the
                             configured time window */
  std::vector<double>
      joint_positions; /**< Gripper joint positions (radians), typically 1-2 joints for finger actuators */
};

/**
 * @brief Suction cup action state enumeration
 *
 * Represents the operational state of a vacuum suction cup end-effector,
 * tracking the suction process from idle to success or failure.
 */
enum class SUCTION_ACTION_STATE {
  suction_action_idle,    /**< Idle state, vacuum not activated */
  suction_action_sucking, /**< Suction in progress, attempting to grasp object */
  suction_action_success, /**< Suction successful, pressure decreased indicating secure grasp */
  suction_action_failed,  /**< Suction failed, pressure did not decrease (no object or seal failure) */
};

/**
 * @brief Suction cup state
 *
 * Contains the current state of a vacuum suction cup gripper, including
 * activation status, pressure reading, and action state.
 */
struct SuctionCupState {
  int64_t timestamp_ns;              /**< State timestamp (nanoseconds since epoch) */
  bool activation;                   /**< Activation flag: true if vacuum is on, false if off */
  double pressure;                   /**< Current vacuum pressure (Pascals), typically negative for suction */
  SUCTION_ACTION_STATE action_state; /**< Current suction action state */
};

/**
 * @brief Timestamp structure
 *
 * Represents high-precision time points with second and nanosecond components.
 * Compatible with ROS 2 builtin_interfaces/Time and std_msgs/Header timestamp format.
 */
struct Timestamp {
  /**
   * @brief Seconds component
   *
   * Number of seconds since UNIX Epoch (1970-01-01 00:00:00 UTC).
   */
  int64_t sec;

  /**
   * @brief Nanoseconds component
   *
   * Nanosecond portion within the current second.
   * Valid range: [0, 999,999,999] (< 1 second)
   */
  uint32_t nanosec;
};

/**
 * @brief Message header structure
 *
 * Standard message header containing timestamp and coordinate frame information.
 * Timestamp is stored as nanoseconds since epoch (unified with other sensor types).
 */
struct Header {
  /**
   * @brief Timestamp of data acquisition (nanoseconds since epoch)
   *
   * Records when the data was captured or generated.
   */
  int64_t timestamp_ns;

  /**
   * @brief Frame ID
   *
   * Identifies the coordinate frame in which the data is expressed.
   * Examples: "base_link", "world", "camera_optical_frame", "lidar_link", "map"
   */
  std::string frame_id;
};

/**
 * @brief Bitmask describing which fields in a target are meaningful.
 *
 * Mirrors galbot.singorix_proto.TargetData while remaining in the SDK type layer.
 */
enum TargetDataBits : int32_t {
  TARGET_DATA_NONE = 0x00,
  TARGET_DATA_JOINT_POS = 0x01,
  TARGET_DATA_JOINT_VEL = 0x02,
  TARGET_DATA_JOINT_ACC = 0x04,
  TARGET_DATA_JOINT_EFF = 0x08,
  TARGET_DATA_FRAME_POSE = 0x10,
  TARGET_DATA_FRAME_TWIST = 0x20,
  TARGET_DATA_FRAME_WRENCH = 0x40,
  TARGET_DATA_DEFAULT = 0xff,
};

/**
 * @brief Bitmask describing how a target should be applied.
 *
 * Mirrors galbot.singorix_proto.TargetType while remaining in the SDK type layer.
 */
enum TargetTypeBits : int32_t {
  TARGET_TYPE_NONE = 0x00,
  TARGET_TYPE_TOUCH = 0x01,
  TARGET_TYPE_CLEAR = 0x02,
  TARGET_TYPE_PREPENDNOW = 0x04,
  TARGET_TYPE_APPEND = 0x08,
  TARGET_TYPE_OVERRIDE = 0x0a,
  TARGET_TYPE_PROVERRIDE = 0x0e,
  TARGET_TYPE_DEFAULT = 0xff,
};

/**
 * @brief Sampling strategy for a target trajectory.
 *
 * Mirrors galbot.singorix_proto.TargetSampling while remaining in the SDK type layer.
 */
enum class TargetSampling : int32_t {
  TARGET_SAMPLING_DEFAULT = 0,
  TARGET_SAMPLING_DIRECT_PASS = 1,
  TARGET_SAMPLING_LINEAR_INTERPOLATE = 2,
  TARGET_SAMPLING_TRAPEZOIDAL_PROFILE = 3,
  TARGET_SAMPLING_S_CURVE_PROFILE = 4,
  TARGET_SAMPLING_CUBIC_SPLINES = 5,
  TARGET_SAMPLING_QUINTIC_SPLINES = 6,
  TARGET_SAMPLING_B_SPLINES = 7,
  TARGET_SAMPLING_CUSTOM = 15,
};

/**
 * @brief 6D twist command (linear + angular velocity).
 */
struct Twist {
  Vector3 linear = {};
  Vector3 angular = {};
};

/**
 * @brief 6D wrench command (force + torque).
 */
struct Wrench {
  Vector3 force = {};
  Vector3 torque = {};
};

/**
 * @brief Task-space command for a body frame relative to a reference frame.
 *
 * Mirrors galbot.spatial_proto.FrameTriad at the SDK type layer.
 */
struct FrameTriad {
  Header header = {};
  std::string body_frame_id;
  std::string reference_frame_id;
  std::optional<Pose> pose;
  std::optional<Twist> twist;
  std::optional<Wrench> wrench;
};

/**
 * @brief Common target configuration shared by group and task trajectories.
 */
struct TargetConfig {
  int32_t target_data = TARGET_DATA_DEFAULT;
  int32_t target_type = TARGET_TYPE_DEFAULT;
  TargetSampling target_sampling = TargetSampling::TARGET_SAMPLING_DEFAULT;
  int32_t target_priority = 0;
  std::string target_id;
  Timestamp target_ts = {};
};

/**
 * @brief Group-space command at a specific time point.
 */
struct GroupCommand {
  double time_from_start_s = 0.0;
  std::vector<JointCommand> joint_commands;
};

/**
 * @brief Task-space command at a specific time point.
 */
struct TaskCommand {
  double time_from_start_s = 0.0;
  std::vector<FrameTriad> subtask_commands;
};

/**
 * @brief Target trajectory for a group of joints.
 */
struct TargetGroupTrajectory {
  TargetConfig target_config = {};
  std::vector<std::string> joint_names;
  std::vector<GroupCommand> group_commands;
};

/**
 * @brief Target trajectory for task-space control.
 */
struct TargetTaskTrajectory {
  TargetConfig target_config = {};
  std::vector<std::string> group_names;
  std::vector<std::string> joint_names;
  std::vector<std::string> subtask_names;
  std::vector<TaskCommand> task_commands;
};

/**
 * @brief SDK mirror of galbot.singorix_proto.SingoriXTarget.
 */
struct SingoriXTarget {
  Header header = {};
  std::unordered_map<std::string, TargetGroupTrajectory> target_group_trajectory_map;
  std::unordered_map<std::string, TargetTaskTrajectory> target_task_trajectory_map;
};

/**
 * @brief RGB/color image data structure
 *
 * Contains compressed color image data from RGB cameras.
 * Compatible with ROS 2 sensor_msgs/CompressedImage format.
 */
struct RgbData {
  /**
   * @brief Message header
   *
   * Contains acquisition timestamp and camera coordinate frame ID.
   */
  Header header;

  /**
   * @brief Image format descriptor
   *
   * Specifies compression format and encoding.
   * Examples: "jpeg", "png", "bgr8; jpeg compressed bgr8"
   */
  std::string format;

  /**
   * @brief Compressed image data
   *
   * Binary blob containing the compressed image (JPEG, PNG, etc.).
   */
  std::vector<uint8_t> data;

  /**
   * @brief Decode compressed image data to OpenCV Mat
   *
   * Decodes the internally stored compressed binary data using cv::imdecode.
   *
   * @return std::shared_ptr<cv::Mat> Smart pointer to decoded image on success, nullptr on failure
   */
  std::shared_ptr<cv::Mat> convert_to_cv2_mat();
};

/**
 * @brief Infrared image data structure
 *
 * Contains compressed infrared (IR) image data from depth cameras (e.g., RealSense infra1/infra2).
 * Compatible with ROS 2 sensor_msgs/CompressedImage format with mono encoding.
 */
struct IrData {
  /**
   * @brief Message header
   *
   * Contains acquisition timestamp and camera coordinate frame ID.
   */
  Header header;

  /**
   * @brief Image format descriptor
   *
   * Specifies compression format and encoding.
   * Examples: "mono8; jpeg compressed mono8"
   */
  std::string format;

  /**
   * @brief Image height in pixels
   */
  uint32_t height = 0;

  /**
   * @brief Image width in pixels
   */
  uint32_t width = 0;

  /**
   * @brief Compressed image data
   *
   * Binary blob containing the compressed infrared image.
   */
  std::vector<uint8_t> data;

  /**
   * @brief Decode compressed IR image data to OpenCV Mat
   *
   * Decodes the internally stored compressed binary data using cv::imdecode
   * with IMREAD_GRAYSCALE to produce a single-channel Mat.
   *
   * @return std::shared_ptr<cv::Mat> Smart pointer to decoded image on success, nullptr on failure
   */
  std::shared_ptr<cv::Mat> convert_to_cv2_mat();
};

/**
 * @brief Depth image data structure
 *
 * Contains compressed depth image data from depth cameras or RGB-D sensors.
 * Compatible with ROS 2 sensor_msgs/CompressedImage format with depth extensions.
 */
struct DepthData {
  /**
   * @brief Message header
   *
   * Contains acquisition timestamp and camera coordinate frame ID.
   */
  Header header;

  /**
   * @brief Image height (pixels)
   *
   * Number of rows in the depth image.
   */
  uint32_t height;

  /**
   * @brief Image width (pixels)
   *
   * Number of columns in the depth image.
   */
  uint32_t width;

  /**
   * @brief Image format descriptor
   *
   * Specifies depth encoding and compression format.
   * Example: "16UC1; compressedDepth png" (16-bit unsigned, 1 channel, PNG compressed)
   */
  std::string format;

  /**
   * @brief Depth image data
   *
   * Binary blob containing raw or compressed depth image data.
   */
  std::vector<uint8_t> data;

  /**
   * @brief Depth scale factor
   *
   * Quantization factor for converting pixel values to metric depth.
   * True depth (meters) = pixel_value / depth_scale
   * Example: depth_scale = 1000 means pixel values are in millimeters
   */
  uint32_t depth_scale;

  /**
   * @brief Convert depth data to OpenCV Mat
   *
   * Decodes and converts depth data to cv::Mat format for processing.
   *
   * @return std::shared_ptr<cv::Mat> Smart pointer to decoded depth image on success, nullptr on failure
   */
  std::shared_ptr<cv::Mat> convert_to_cv2_mat();
};

/**
 * @brief Synchronized multi-sensor observation payload.
 *
 * Contains timestamp-aligned camera frames and optional joint state.
 * The alignment anchor is the first camera passed to get_synced_observation().
 */
struct SyncedObservation {
  std::unordered_map<SensorType, std::shared_ptr<RgbData>> rgb_data_map;
  std::unordered_map<SensorType, std::shared_ptr<DepthData>> depth_data_map;
  std::shared_ptr<JointStateMessage> joint_state = nullptr;
};

/**
 * @brief Point cloud field descriptor
 *
 * Describes one data field in a PointCloud2 point structure, defining its name,
 * type, offset, and count. Compatible with ROS 2 sensor_msgs/PointField.
 */
struct PointField {
  /**
   * @brief Data type enumeration
   *
   * Defines primitive data types for point cloud fields, determining byte size
   * and interpretation method for each field value.
   */
  enum DataType : uint8_t {
    UNKNOWN = 0, /**< Unknown or unspecified type */
    INT8 = 1,    /**< 8-bit signed integer (1 byte) */
    UINT8 = 2,   /**< 8-bit unsigned integer (1 byte) */
    INT16 = 3,   /**< 16-bit signed integer (2 bytes) */
    UINT16 = 4,  /**< 16-bit unsigned integer (2 bytes) */
    INT32 = 5,   /**< 32-bit signed integer (4 bytes) */
    UINT32 = 6,  /**< 32-bit unsigned integer (4 bytes) */
    FLOAT32 = 7, /**< 32-bit IEEE 754 floating point (4 bytes) */
    FLOAT64 = 8  /**< 64-bit IEEE 754 floating point (8 bytes) */
  };

  /**
   * @brief Field name
   *
   * Identifier for this data channel. Standard field names include:
   * - "x", "y", "z": 3D Cartesian coordinates (meters)
   * - "intensity": Reflection intensity (unitless or sensor-specific)
   * - "rgb" or "rgba": Color information (packed RGB or RGBA)
   * - "ring": Lidar ring/laser index (integer)
   * - "timestamp": Per-point timestamp (seconds or nanoseconds)
   */
  std::string name;

  /**
   * @brief Byte offset from point start
   *
   * Byte offset of this field from the beginning of a point's data structure.
   * Example: For point layout {x:float32, y:float32, z:float32}, offsets are:
   * x=0, y=4, z=8
   */
  uint32_t offset;

  /**
   * @brief Data type of this field
   *
   * Specifies the primitive data type using the DataType enumeration.
   */
  DataType datatype;

  /**
   * @brief Number of elements in this field
   *
   * Array length for this field. Typically 1 for scalar fields (x, y, z, intensity).
   * May be > 1 for array fields (e.g., count=3 for a 3-element vector).
   */
  uint32_t count;
};

/**
 * @brief Lidar data structure
 *
 * Generic N-dimensional point cloud structure compatible with ROS 2 sensor_msgs/PointCloud2.
 * Stores point data as a binary blob with field descriptors defining the data layout.
 * Supports both ordered (structured) and unordered (unstructured) point clouds.
 */
struct LidarData {
  /**
   * @brief Message header
   *
   * Contains acquisition timestamp and coordinate frame for temporal and spatial reference.
   */
  Header header;

  /**
   * @brief Point cloud height (rows)
   *
   * - Unordered point cloud: height = 1 (single row)
   * - Ordered point cloud: height = number of rows (e.g., from spinning lidar or depth camera)
   */
  uint32_t height;

  /**
   * @brief Point cloud width (points per row)
   *
   * - Unordered point cloud: width = total number of points
   * - Ordered point cloud: width = number of points per row (columns)
   * Total points = height × width
   */
  uint32_t width;

  /**
   * @brief Field descriptors
   *
   * Describes the data channels (x, y, z, intensity, rgb, etc.) present in each point
   * and their binary layout (offset, type, count).
   */
  std::vector<PointField> fields;

  /**
   * @brief Endianness flag
   *
   * - true: Data is Big Endian byte order
   * - false: Data is Little Endian byte order (typical for x86/ARM systems)
   */
  bool is_bigendian;

  /**
   * @brief Point step (bytes per point)
   *
   * Total byte size of a single point structure, including all fields and padding.
   * Must be ≥ sum of all field sizes, may include alignment padding.
   */
  uint32_t point_step;

  /**
   * @brief Row step (bytes per row)
   *
   * Total byte size of one row of points.
   * Formula: row_step = point_step × width
   */
  uint32_t row_step;

  /**
   * @brief Point cloud binary data
   *
   * Binary blob containing all point data in row-major order.
   * Size should equal: row_step × height bytes
   * Each point occupies point_step bytes, laid out according to fields descriptors.
   */
  std::vector<uint8_t> data;

  /**
   * @brief Dense cloud flag
   *
   * - true: All points are valid, no NaN or Inf values present
   * - false: Cloud may contain invalid points (NaN/Inf coordinates or fields)
   */
  bool is_dense;
};

/**
 * @brief Region of interest (ROI)
 *
 * Defines a rectangular sub-region within an image for selective processing.
 * Compatible with ROS 2 sensor_msgs/RegionOfInterest.
 */
struct RegionOfInterest {
  /**
   * @brief X offset (left edge)
   *
   * Horizontal pixel coordinate of the ROI's left edge.
   * 0 means ROI starts at the image's left edge.
   */
  uint32_t x_offset;

  /**
   * @brief Y offset (top edge)
   *
   * Vertical pixel coordinate of the ROI's top edge.
   * 0 means ROI starts at the image's top edge.
   */
  uint32_t y_offset;

  /**
   * @brief ROI height (pixels)
   *
   * Number of pixel rows in the region of interest.
   */
  uint32_t height;

  /**
   * @brief ROI width (pixels)
   *
   * Number of pixel columns in the region of interest.
   */
  uint32_t width;

  /**
   * @brief Rectification flag
   *
   * - true: Apply camera rectification to this ROI before processing
   * - false: Use raw image data without rectification, or capture full resolution
   */
  bool do_rectify;
};

/**
 * @brief Camera calibration information
 *
 * Complete camera calibration data including intrinsic parameters, distortion coefficients,
 * rectification, and projection matrices. Compatible with ROS 2 sensor_msgs/CameraInfo.
 */
struct CameraInfo {
  /**
   * @brief Message header
   *
   * Contains timestamp and camera coordinate frame ID (e.g., "camera_optical_frame").
   */
  Header header;

  /**
   * @brief Image height (pixels)
   *
   * Vertical resolution of images produced by this camera at calibration time.
   */
  uint32_t height;

  /**
   * @brief Image width (pixels)
   *
   * Horizontal resolution of images produced by this camera at calibration time.
   */
  uint32_t width;

  /**
   * @brief Distortion model name
   *
   * Specifies the lens distortion model used.
   * Common values:
   * - "plumb_bob": Brown-Conrady model with radial (k1,k2,k3) and tangential (p1,p2) distortion
   * - "rational_polynomial": Extended model with additional parameters
   * - "equidistant": Fisheye lens model
   */
  std::string distortion_model;

  /**
   * @brief Distortion coefficients (D)
   *
   * Vector of distortion parameters, size and interpretation depend on distortion_model.
   * For "plumb_bob": [k1, k2, p1, p2, k3] (5 parameters)
   * - k1, k2, k3: Radial distortion coefficients
   * - p1, p2: Tangential distortion coefficients
   */
  std::vector<double> d;

  /**
   * @brief Intrinsic camera matrix (K)
   *
   * 3×3 matrix in row-major order, maps 3D points in camera frame to 2D pixel coordinates:
   * @code
   * [fx  0  cx]
   * [ 0  fy cy]
   * [ 0  0   1]
   * @endcode
   * - fx, fy: Focal lengths in pixel units
   * - cx, cy: Principal point (optical center) in pixels
   */
  std::array<double, 9> k;

  /**
   * @brief Rectification matrix (R)
   *
   * 3×3 rotation matrix in row-major order.
   * For stereo cameras: rotates left/right camera image planes to be coplanar and row-aligned.
   * For monocular cameras: typically identity matrix (no rectification needed).
   */
  std::array<double, 9> r;

  /**
   * @brief Projection matrix (P)
   *
   * 3×4 matrix in row-major order, projects 3D points to rectified image coordinates:
   * @code
   * [fx'  0   cx' Tx]
   * [ 0   fy' cy' Ty]
   * [ 0   0    1   0]
   * @endcode
   * - fx', fy': Rectified focal lengths
   * - cx', cy': Rectified principal point
   * - Tx, Ty: Stereo baseline (Tx = -fx' × baseline for right camera)
   */
  std::array<double, 12> p;

  /**
   * @brief Horizontal binning factor
   *
   * Number of camera pixels combined horizontally for each output pixel.
   * Values: 0 or 1 = no binning, 2 = 2×1 binning, etc.
   */
  uint32_t binning_x;

  /**
   * @brief Vertical binning factor
   *
   * Number of camera pixels combined vertically for each output pixel.
   * Values: 0 or 1 = no binning, 2 = 1×2 binning, etc.
   */
  uint32_t binning_y;

  /**
   * @brief Region of interest (ROI)
   *
   * Specifies a sub-window within the full sensor resolution.
   */
  RegionOfInterest roi;

  // ==========================================
  // Extended fields (not in standard ROS 2 CameraInfo)
  // ==========================================

  /**
   * @brief Camera type identifier
   *
   * Optional field specifying camera type or model.
   * Examples: "monocular", "stereo_left", "stereo_right", "depth"
   */
  std::string camera_type;

  /**
   * @brief Additional transform matrix
   *
   * Optional transformation matrix for vendor-specific or extended calibration data.
   * Size and interpretation depend on implementation.
   */
  std::vector<double> T;
};

 /**
  * @brief Audio data structure
  *
  * Encapsulates audio event payloads.
  */
 struct AudioData {
  /**
   * @brief Message header
   *
   * Timestamp and frame identifier for this audio message. See @ref Header for field details:
   * - timestamp_ns: Timestamp of data acquisition (nanoseconds since epoch).
   * - frame_id: Stream or source frame identifier associated with the audio message.
   */
  Header header;

  /**
   * @brief Audio type
   *
   * Audio data type identifier. Possible values include:
   * - "waken_up": Wake-up event; @ref AudioData::format is "json"; @ref AudioData::data is a UTF-8 JSON string.
   * - "denoise_chunk": Denoised audio stream chunk; @ref AudioData::format is "pcm"; @ref AudioData::data is PCM binary.
   * - "vad_begin": Voice Activity Detection (VAD) start marker; @ref AudioData::data is empty.
   * - "vad_chunk": Audio data during voice activity detection; @ref AudioData::format is "pcm";
   *   @ref AudioData::data is PCM binary.
   * - "vad_end": Voice Activity Detection (VAD) end marker; @ref AudioData::data is empty.
   */
  std::string type;

  /**
   * @brief Audio format
   *
   * Describes how to interpret @ref AudioData::data. Possible values include:
   * - "pcm": PCM audio — sample rate 16000 Hz, 16-bit, mono.
   * - "json": UTF-8 encoded JSON text.
   */
  std::string format;

  /**
   * @brief Data packet
   *
   * Binary payload; interpretation depends on @ref AudioData::format:
   * - "pcm": Raw PCM samples; each 80 ms chunk is 2560 bytes.
   * - "json": UTF-8 JSON text; length varies, or empty for marker-only messages (e.g. "vad_begin", "vad_end").
   */
  std::vector<uint8_t> data;
};
/**
 * @brief Bms information
 *
 */
struct BmsInfo {
  float voltage = 0.0f;       /**< Voltage (V) */
  float current = 0.0f;       /**< Current (A) */
  float battery_level = 0.0f; /**< Battery level (0-100%) */
  float temperature = 0.0f;   /**< Temperature (℃) */
  bool charging_status;       /**< Charging status：False: not charging, True: charging */
  bool health_status;         /**< Health status：False: good, True: bad */
  float capacity = 0.0f;      /**< Remaining capacity (Ah) */
};

/**
 * @brief Log information
 *
 */
struct LogInfo {
  std::string level;   /**"error" "warning" */
  std::string message; /**message */
};

/**
 * @brief Optional tuning parameters for a waypoint.
 *
 * These parameters control how precisely the robot reaches a waypoint and how
 * smoothly it moves while doing so. Fields left unchanged keep their default
 * values.
 */
struct WaypointParams {
    double arrival_position_threshold_x = 0.05;  /**< Arrival tolerance along the X axis in meters.
                                                   * Smaller values require the robot to arrive more precisely in the
                                                   * forward/backward direction.
                                                   */

    double arrival_position_threshold_y = 0.05;  /**< Arrival tolerance along the Y axis in meters.
                                                   * Smaller values require the robot to arrive more precisely in the
                                                   * lateral direction.
                                                   */

    double arrival_orientation_threshold = 0.05; /**< Orientation tolerance in radians.
                                                   * Smaller values require the robot to align its heading more accurately
                                                   * before considering the waypoint reached.
                                                   */

    double velocity_scale = 1.0;                 /**< Speed scaling factor for waypoint execution.
                                                  * Lower values make the robot move more slowly and smoothly; higher
                                                  * values make it move faster.
                                                  */

    double acceleration_scale = 1.0;             /**< Acceleration scaling factor for waypoint execution.
                                                  * Lower values make starts and stops gentler; higher values allow more
                                                  * aggressive motion.
                                                  */

    double jerk_scale = 1.0;                     /**< Jerk scaling factor for waypoint execution.
                                                  * Lower values make motion changes smoother; higher values allow sharper
                                                  * acceleration changes.
                                                  */
};

struct Waypoint {
    /** @brief Target pose of this waypoint.
     *
     * The robot will navigate toward this pose when executing the waypoint.
     */
    Pose pose;

    /** @brief Optional tuning parameters for this waypoint.
     *
     * These parameters control how strictly the robot reaches the waypoint and
     * how smoothly it moves while doing so.
     */
    WaypointParams params;

    /**
     * @brief Construct a waypoint with default tuning parameters.
     * @param pose Target pose.
     */
    Waypoint(const Pose& pose)
        : pose(pose), params(WaypointParams()) {}

    /**
     * @brief Construct a waypoint with custom tuning parameters.
     * @param pose Target pose.
     * @param params Optional waypoint tuning parameters.
     */
    Waypoint(const Pose & pose, const WaypointParams & params)
        : pose(pose), params(params) {}
};

}  // namespace sdk
}  // namespace galbot
