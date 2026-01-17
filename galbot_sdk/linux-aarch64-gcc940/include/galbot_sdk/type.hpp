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
 */

#pragma once

#include <array>
#include <memory>
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
   * @namespace galbot::sdk::g1
   * @brief Namespace for Galbot G1 humanoid robot
   */
  namespace g1 {

/**
 * @brief Navigation task execution status enumeration
 *
 * Defines the possible outcomes when executing navigation commands such as
 * moving to a goal position or following a path.
 */
enum class NavigationStatus {
  SUCCESS,          /**< Execution succeeded, navigation task completed as expected */
  FAIL,             /**< Execution failed due to unspecified error */
  TIMEOUT,          /**< Execution timeout, task did not complete within allowed time */
  INVALID_INPUT,    /**< Input parameters do not meet requirements or are out of valid range */
  MODE_ERR,         /**< Current mode does not support this operation */
  COMM_ERR,         /**< Communication error occurred during execution */
  WAIT_INITIALIZED  /**< Waiting for system initialization, navigation system not ready */
};

/**
 * @brief Control command execution status enumeration
 *
 * Represents the execution status of robot control commands, including
 * joint control, end-effector control, and other motion control operations.
 */
enum class ControlStatus {
  SUCCESS,            /**< Execution succeeded, command completed with valid result */
  TIMEOUT,            /**< Execution timeout, task not completed within specified time limit */
  FAULT,              /**< Fault occurred, system detected anomaly and aborted execution */
  INVALID_INPUT,      /**< Input parameters invalid or not meeting interface requirements */
  INIT_FAILED,        /**< Initialization failed, internal communication or dependent component creation failed */
  IN_PROGRESS,        /**< Command is executing but has not reached target state */
  STOPPED_UNREACHED,  /**< Stopped during execution without reaching target position or state */
  DATA_FETCH_FAILED,  /**< Data retrieval failed during operation, unable to read required state */
  PUBLISH_FAIL,       /**< Control or state data publication failed, command may not be transmitted */
  COMM_DISCONNECTED,  /**< Communication connection lost, cannot continue execution */
  STATUS_NUM          /**< Total number of status enumerations (for boundary checking or array sizing) */
};

/**
 * @brief Sensor execution status enumeration
 *
 * Represents the status of sensor data acquisition and processing operations,
 * applicable to cameras, lidar, IMU, force sensors, and other sensor types.
 */
enum class SensorStatus {
  SUCCESS,            /**< Execution succeeded, sensor data valid and operation completed */
  TIMEOUT,            /**< Execution timeout, data acquisition or operation not completed within specified time limit */
  FAULT,              /**< Fault occurred, sensor detected anomaly and cannot continue normal operation */
  INVALID_INPUT,      /**< Input parameters invalid or not meeting interface requirements */
  INIT_FAILED,        /**< Initialization failed, internal communication or dependent component creation failed */
  IN_PROGRESS,        /**< Operation in progress but not yet completed */
  STOPPED_UNREACHED,  /**< Stopped during execution without completing expected operation */
  DATA_FETCH_FAILED,  /**< Data acquisition or reading failed, sensor may be disconnected or malfunctioning */
  PUBLISH_FAIL,       /**< Data transmission or reporting failed, unable to publish sensor data */
  COMM_DISCONNECTED,  /**< Sensor communication connection lost, no data available */
  STATUS_NUM          /**< Total number of status enumerations (for boundary checking or array sizing) */
};

/**
 * @brief Robot motion execution status enumeration
 *
 * Represents the execution status of robot motion commands, including
 * trajectory following, pose reaching, and other motion planning operations.
 */
enum class MotionStatus {
  SUCCESS,              /**< Execution succeeded, motion reached expected target position/pose */
  TIMEOUT,              /**< Execution timeout, motion not completed within specified time limit */
  FAULT,                /**< Fault occurred, motion cannot continue due to hardware or safety issue */
  INVALID_INPUT,        /**< Input parameters invalid or not meeting interface requirements */
  INIT_FAILED,          /**< Internal initialization failed, communication component or resource creation failed */
  IN_PROGRESS,          /**< Motion in progress but has not reached target yet */
  STOPPED_UNREACHED,    /**< Stopped during motion without reaching target position/pose */
  DATA_FETCH_FAILED,    /**< Data retrieval failed, e.g., sensor or state reading failure */
  PUBLISH_FAIL,         /**< Data transmission or command delivery failed, motion command may not be executed */
  COMM_DISCONNECTED,    /**< Communication disconnected or control node unavailable */
  STATUS_NUM,           /**< Total number of status enumerations (for boundary checking or array sizing) */
  UNSUPPORTED_FUNCRION  /**< Function not yet supported, called interface or operation not implemented (note: typo in enum name preserved for API compatibility) */
};

/**
 * @brief Robot trajectory execution status enumeration
 *
 * Represents the real-time execution status when the robot follows a
 * pre-planned trajectory consisting of multiple waypoints.
 */
enum class TrajectoryControlStatus {
  INVALID_INPUT,      /**< Input parameters do not meet requirements, trajectory cannot be executed */
  RUNNING,            /**< Trajectory is currently executing, not yet completed */
  COMPLETED,          /**< Trajectory execution completed successfully, reached final target point */
  STOPPED_UNREACHED,  /**< Stopped during trajectory execution without reaching endpoint */
  ERROR,              /**< Error occurred, trajectory execution cannot continue */
  DATA_FETCH_FAILED,  /**< Execution data retrieval failed, e.g., joint state or sensor feedback unavailable */
  STATUS_NUM          /**< Total number of status enumerations (for boundary checking or array sizing) */
};

/**
 * @brief Joint group enumeration describing different functional modules of the robot
 *
 * Groups robot joints into functional units for coordinated control and motion planning.
 * Each group typically represents a kinematic chain or end-effector subsystem.
 */
enum class JointGroup {
  HEAD,               /**< Head joint group, typically for pan-tilt or vision orientation */
  LEFT_ARM,           /**< Left arm joint group, multi-DOF manipulator chain */
  RIGHT_ARM,          /**< Right arm joint group, multi-DOF manipulator chain */
  LEG,                /**< Leg joint group, for humanoid or legged robots */
  CHASSIS,            /**< Chassis joint group, mobile base or lower platform */
  LEFT_GRIPPER,       /**< Left gripper joint group, typically 1-2 DOF parallel jaw gripper */
  RIGHT_GRIPPER,      /**< Right gripper joint group, typically 1-2 DOF parallel jaw gripper */
  LEFT_SUCTION_CUP,   /**< Left suction cup joint group, vacuum-based end-effector */
  RIGHT_SUCTION_CUP,  /**< Right suction cup joint group, vacuum-based end-effector */
  JOINT_GROUP_NUM     /**< Total number of joint group enumerations (for boundary checking or array sizing) */
};

/**
 * @brief Sensor type enumeration describing various sensors on the robot
 *
 * Identifies different sensor types available on the robot for perception,
 * localization, and manipulation tasks.
 */
enum class SensorType {
  HEAD_LEFT_CAMERA,        /**< Head left camera, typically RGB camera for stereo vision */
  HEAD_RIGHT_CAMERA,       /**< Head right camera, typically RGB camera for stereo vision */
  LEFT_ARM_CAMERA,         /**< Left arm camera, mounted on left manipulator for visual servoing */
  RIGHT_ARM_CAMERA,        /**< Right arm camera, mounted on right manipulator for visual servoing */
  LEFT_ARM_DEPTH_CAMERA,   /**< Left arm depth camera, provides RGB-D data for left arm workspace */
  RIGHT_ARM_DEPTH_CAMERA,  /**< Right arm depth camera, provides RGB-D data for right arm workspace */
  BASE_LIDAR,              /**< Base lidar, laser scanner for 2D/3D environment mapping and obstacle detection */
  TORSO_IMU,               /**< Torso IMU (Inertial Measurement Unit), measures acceleration and angular velocity */
  BASE_ULTRASONIC,         /**< Base ultrasonic sensor array, for proximity detection and collision avoidance */
  SENSOR_NUM               /**< Total number of sensor enumerations (for boundary checking or array sizing) */
};

/**
 * @brief Chassis ultrasonic sensor probe enumeration (8 directions)
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
 * @brief Force sensor enumeration describing robot wrist force sensors
 *
 * Identifies force/torque sensors mounted at the robot's wrist joints for
 * force-controlled manipulation and contact detection.
 */
enum class GalbotOneFoxtrotSensor {
  LEFT_WRIST_FORCE,   /**< Left wrist force/torque sensor, typically 6-axis (3 forces + 3 torques) */
  RIGHT_WRIST_FORCE,  /**< Right wrist force/torque sensor, typically 6-axis (3 forces + 3 torques) */
  FORCE_NUM,          /**< Total number of force sensor enumerations (for boundary checking or array sizing) */
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
  HEAD,       /**< Head kinematic chain, from base/torso to head end-effector */
  LEFT_ARM,   /**< Left arm kinematic chain, from base/torso to left end-effector */
  RIGHT_ARM,  /**< Right arm kinematic chain, from base/torso to right end-effector */
  LEG,        /**< Leg kinematic chain, for legged locomotion */
  TORSO,      /**< Torso kinematic chain, connects base to upper body */
  CHAIN_NUM   /**< Total number of kinematic chains (for boundary checking or array sizing) */
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
 */
struct TrajectoryPoint {
  double time_from_start_second;               /**< Time from trajectory start (seconds) */
  std::vector<JointCommand> joint_command_vec; /**< List of joint commands for all joints at this waypoint */
};

/**
 * @brief Joint trajectory
 *
 * Represents a complete robot trajectory consisting of multiple waypoints over time.
 */
struct Trajectory {
  std::vector<TrajectoryPoint> points;     /**< Ordered list of trajectory waypoints */
  std::vector<std::string> joint_groups;   /**< Names of joint groups involved in this trajectory */
  std::vector<std::string> joint_names;    /**< Names of individual joints controlled by this trajectory */
};

/**
 * @brief Error information
 *
 * Describes an error from a single module or component, including error code and
 * human-readable description for debugging and diagnostics.
 */
struct Error {
  std::string commpent;    /**< Fault module or component name (note: field name contains typo but preserved for API compatibility) */
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
   * @tparam T Container type supporting subscript access and size() method (e.g., std::vector<double>, std::array<double>)
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
  RANDOM_PROGRESSIVE_SEED, /**< Random progressive seed, tries multiple random seeds iteratively (recommended for robustness) */
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
  std::unordered_map<JointGroup, std::vector<double>> joint_state = {};

  /**
   * @brief Whether to plan Cartesian linear path
   *
   * - true: Plan straight-line motion in Cartesian space (end-effector moves in straight line)
   * - false: Plan standard joint-space or task-space trajectory (may not be Cartesian linear)
   */
  bool move_line = false;
};

/**
 * @brief Planning task result structure
 *
 * Contains the complete result of a motion planning operation, including success status,
 * generated trajectory, kinematics solutions, and collision information.
 */
struct PlanTaskResult {
  /**
   * @brief Unique task identifier
   *
   * Used to track and distinguish different planning tasks, especially in asynchronous operations.
   */
  std::string task_id;

  /**
   * @brief Success flag
   *
   * - true: Planning completed successfully
   * - false: Planning failed (check error_code and error_message for details)
   */
  bool success;

  /**
   * @brief Numerical error code
   *
   * Used for programmatic error handling. Zero typically indicates success,
   * non-zero values indicate specific error conditions.
   */
  int error_code;

  /**
   * @brief Human-readable error message
   *
   * Provides detailed description of failure reason or exception information when success = false.
   */
  std::string error_message;

  /**
   * @brief Trajectory result
   *
   * Contains the complete planned trajectory with joint positions and timing information.
   */
  struct Trajectory {
    /**
     * @brief Joint positions at each waypoint
     *
     * Each element is a vector of joint angles (radians) representing robot configuration
     * at one waypoint. Inner vector size = number of joints, outer vector size = number of waypoints.
     */
    std::vector<std::vector<double>> joint_positions;

    /**
     * @brief Timestamps for each waypoint (seconds)
     *
     * Cumulative time from trajectory start. Size must match joint_positions size.
     */
    std::vector<double> timestamps;
  } trajectory;

  /**
   * @brief Inverse kinematics solution
   *
   * Maps kinematic chain name to solved joint configuration (radians).
   * Key: Joint chain name (e.g., "left_arm", "right_arm")
   * Value: Vector of joint angles (radians)
   */
  std::unordered_map<std::string, std::vector<double>> ik_result;

  /**
   * @brief Forward kinematics solution
   *
   * Maps link or end-effector name to computed pose.
   * Key: Link or end-effector name (e.g., "left_gripper", "right_hand")
   * Value: Computed pose (position + orientation)
   */
  std::unordered_map<std::string, Pose> fk_result;

  /**
   * @brief Collision detection result
   *
   * Optional field containing collision distances or penetration depths.
   * Empty vector typically means no collision check was performed.
   * Non-empty values may represent minimum distances to obstacles or collision severity.
   */
  std::vector<double> collision_result;
};

/**
 * @brief Single joint state structure
 *
 * Represents the complete real-time state of a single robot joint, including
 * kinematic quantities (position, velocity, acceleration) and dynamic quantities
 * (torque/effort and motor current).
 */
struct JointState {
  double position;     /**< Joint angular position (radians) */
  double velocity;     /**< Joint angular velocity (radians/second) */
  double acceleration; /**< Joint angular acceleration (radians/second²) */
  double effort;       /**< Joint torque/effort (Newton-meters) */
  double current;      /**< Motor current (amperes) */

  /**
   * @brief Default constructor
   *
   * Initializes all state variables to zero.
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
   */
  JointState(double position_input, double velocity_input, double acceleration_input, double effort_input,
             double current_input)
      : position(position_input),
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
 * Contains robot pose estimate from odometry sources (wheel encoders, visual odometry, etc.).
 * Used for robot localization and navigation.
 */
struct OdomData {
  int64_t timestamp;                  /**< Odometry timestamp (seconds since epoch) */
  std::array<double, 3> position;     /**< Position [x, y, z] (meters) */
  std::array<double, 4> orientation;  /**< Orientation as quaternion [qx, qy, qz, qw] */

  /**
   * @note Velocity fields temporarily unavailable in this version:
   * - std::array<double, 3> linear_velocity;  // Linear velocity [vx, vy, vz] (meters/second)
   * - std::array<double, 3> angular_velocity; // Angular velocity [ωx, ωy, ωz] (radians/second)
   */
};

/**
 * @brief Gripper state
 *
 * Represents the current state of a parallel-jaw gripper, including opening width,
 * motion status, and grasping force.
 */
struct GripperState {
  int64_t timestamp_ns;                 /**< State timestamp (nanoseconds since epoch) */
  double width;                         /**< Gripper opening width (meters), distance between fingers */
  double velocity;                      /**< Gripper closing/opening velocity (meters/second), positive = opening */
  double effort;                        /**< Gripper grasping force (Newtons), force applied by fingers */
  bool is_moving = false;               /**< Motion flag: true if gripper is currently moving, false if stationary */
  std::vector<double> joint_positions;  /**< Gripper joint positions (radians), typically 1-2 joints for finger actuators */
};

/**
 * @brief Suction cup action state enumeration
 *
 * Represents the operational state of a vacuum suction cup end-effector,
 * tracking the suction process from idle to success or failure.
 */
enum class SUCTION_ACTION_STATE {
  suction_action_idle,     /**< Idle state, vacuum not activated */
  suction_action_sucking,  /**< Suction in progress, attempting to grasp object */
  suction_action_success,  /**< Suction successful, pressure decreased indicating secure grasp */
  suction_action_failed,   /**< Suction failed, pressure did not decrease (no object or seal failure) */
};

/**
 * @brief Suction cup state
 *
 * Contains the current state of a vacuum suction cup gripper, including
 * activation status, pressure reading, and action state.
 */
struct SuctionCupState {
  int64_t timestamp_ns;                  /**< State timestamp (nanoseconds since epoch) */
  bool activation;                       /**< Activation flag: true if vacuum is on, false if off */
  double pressure;                       /**< Current vacuum pressure (Pascals), typically negative for suction */
  SUCTION_ACTION_STATE action_state;     /**< Current suction action state */
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
 * Compatible with ROS 2 std_msgs/Header format, used in sensor messages and transforms.
 */
struct Header {
  /**
   * @brief Timestamp of data acquisition
   *
   * Records when the data was captured or generated.
   */
  Timestamp stamp;

  /**
   * @brief Frame ID
   *
   * Identifies the coordinate frame in which the data is expressed.
   * Examples: "base_link", "world", "camera_optical_frame", "lidar_link", "map"
   */
  std::string frame_id;
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
    UNKNOWN = 0,   /**< Unknown or unspecified type */
    INT8 = 1,      /**< 8-bit signed integer (1 byte) */
    UINT8 = 2,     /**< 8-bit unsigned integer (1 byte) */
    INT16 = 3,     /**< 16-bit signed integer (2 bytes) */
    UINT16 = 4,    /**< 16-bit unsigned integer (2 bytes) */
    INT32 = 5,     /**< 32-bit signed integer (4 bytes) */
    UINT32 = 6,    /**< 32-bit unsigned integer (4 bytes) */
    FLOAT32 = 7,   /**< 32-bit IEEE 754 floating point (4 bytes) */
    FLOAT64 = 8    /**< 64-bit IEEE 754 floating point (8 bytes) */
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

}  // namespace g1
}  // namespace sdk
}  // namespace galbot
