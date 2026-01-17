/**
 * @file galbot_motion.hpp
 * @brief Motion planning and control interface for Galbot robots
 *
 * This file provides the GalbotMotion class and related types for comprehensive robot motion control,
 * including forward/inverse kinematics computation, single-chain and multi-chain trajectory planning,
 * collision detection (self-collision and environment), tool and obstacle management, and whole-body
 * coordinated motion planning.
 *
 * All motion planning operations use SI units: radians for angles, meters for distances.
 * Quaternions must be unit-normalized (x² + y² + z² + w² = 1) for all pose specifications.
 *
 * @author Galbot SDK Team
 * @version 1.5.1
 * @copyright Copyright (c) 2023-2026 Galbot. All rights reserved.
 */

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "motion_plan_config.hpp"
#include "type.hpp"

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
 * @var g_actuate_type_map
 * @brief Whole-body planning actuation type mapping table.
 *
 * This static map provides a string-to-enum conversion for ActuateType enumeration values,
 * enabling actuation type selection from configuration files or function parameters:
 *   - "with_chain_only" -> ACTUATE_WITH_CHAIN_ONLY : Execute kinematic chain actions only (arms only)
 *   - "with_torso"      -> ACTUATE_WITH_TORSO      : Include torso joint actuation
 *   - "with_leg"        -> ACTUATE_WITH_LEG        : Include leg joint actuation
 *   - "type_num"        -> ACTUATE_TYPE_NUM        : Total number of actuation types (for iteration)
 *
 * @note This map is used for runtime string-based actuation type resolution.
 * @see ActuateType
 */
static std::unordered_map<std::string, ActuateType> g_actuate_type_map = {
    {"with_chain_only", ActuateType::ACTUATE_WITH_CHAIN_ONLY},
    {"with_torso", ActuateType::ACTUATE_WITH_TORSO},
    {"with_leg", ActuateType::ACTUATE_WITH_LEG},
    {"type_num", ActuateType::ACTUATE_TYPE_NUM}};

/**
 * @var status_string_map_
 * @brief MotionStatus enumeration to human-readable string mapping table.
 *
 * This static map converts MotionStatus enumeration values to descriptive strings
 * for logging, debugging, and user interface display purposes.
 *
 * Status mappings:
 * - MotionStatus::SUCCESS             -> "SUCCESS: Execution succeeded"
 * - MotionStatus::TIMEOUT             -> "TIMEOUT: Execution timeout"
 * - MotionStatus::FAULT               -> "FAULT: Fault occurred, unable to continue execution"
 * - MotionStatus::INVALID_INPUT       -> "INVALID_INPUT: Input parameters do not meet requirements"
 * - MotionStatus::INIT_FAILED         -> "INIT_FAILED: Internal communication component creation failed"
 * - MotionStatus::IN_PROGRESS         -> "IN_PROGRESS: In motion but target not yet reached"
 * - MotionStatus::STOPPED_UNREACHED   -> "STOPPED_UNREACHED: Motion stopped before reaching target"
 * - MotionStatus::DATA_FETCH_FAILED   -> "DATA_FETCH_FAILED: Failed to retrieve required data"
 * - MotionStatus::PUBLISH_FAIL        -> "PUBLISH_FAIL: Failed to transmit command/data"
 * - MotionStatus::COMM_DISCONNECTED   -> "COMM_DISCONNECTED: Communication link disconnected"
 * - MotionStatus::STATUS_NUM          -> "STATUS_NUM: Total number of status enumerations"
 * - MotionStatus::UNSUPPORTED_FUNCRION -> "UNSUPPORTED_FUNCRION: Function not supported yet"
 *
 * @note The map is primarily used by status_to_string() method.
 * @see MotionStatus
 * @see status_to_string()
 */
static std::unordered_map<MotionStatus, std::string> status_string_map_ = {
    {MotionStatus::SUCCESS, "SUCCESS: Execution succeeded"},
    {MotionStatus::TIMEOUT, "TIMEOUT: Execution timeout"},
    {MotionStatus::FAULT, "FAULT: Fault occurred, unable to continue execution"},
    {MotionStatus::INVALID_INPUT, "INVALID_INPUT: Input parameters do not meet requirements"},
    {MotionStatus::INIT_FAILED, "INIT_FAILED: Internal communication component creation failed"},
    {MotionStatus::IN_PROGRESS, "IN_PROGRESS: In motion but not reached"},
    {MotionStatus::STOPPED_UNREACHED, "STOPPED_UNREACHED: Stopped but target not reached"},
    {MotionStatus::DATA_FETCH_FAILED, "DATA_FETCH_FAILED: Data retrieval failed"},
    {MotionStatus::PUBLISH_FAIL, "PUBLISH_FAIL: Data transmission failed"},
    {MotionStatus::COMM_DISCONNECTED, "COMM_DISCONNECTED: Connection failed"},
    {MotionStatus::STATUS_NUM, "STATUS_NUM: Total number of status enumerations"},
    {MotionStatus::UNSUPPORTED_FUNCRION, "UNSUPPORTED_FUNCRION: Function not supported yet"}};
/**
 * @class Parameter
 * @brief Motion planning parameter configuration class.
 *
 * This class extends PlannerConfig to provide comprehensive configuration options for
 * whole-body motion planning and execution. It encapsulates execution mode, actuation type,
 * tool frame handling, collision checking, and coordinate frame specifications.
 *
 * @note All angular parameters are expected in radians, linear parameters in meters (SI units).
 */
class Parameter : public PlannerConfig {
 public:
  /**
   * @brief Default constructor.
   *
   * Initializes Parameter with default values inherited from PlannerConfig.
   */
  Parameter() = default;

  /**
   * @brief Parameterized constructor for whole-body motion planning configuration.
   *
   * @param direct_execute  If true, immediately executes the planned motion; if false, only computes the plan
   * @param blocking        If true, blocks until motion completes or times out; if false, returns immediately
   * @param timeout         Maximum allowed time for motion execution (in seconds)
   * @param actuate         Actuation type string key (see g_actuate_type_map): "with_chain_only", "with_torso", or "with_leg"
   * @param tool_pose       If true, target pose is relative to tool frame; if false, relative to end-effector flange
   * @param check_collision If true, enables collision checking during planning; if false, skips collision detection
   * @param frame           Reference frame for pose specifications, defaults to "base_link" (robot base frame)
   *
   * @note timeout is only relevant when blocking is true.
   * @note Invalid actuate strings will cause undefined behavior; ensure key exists in g_actuate_type_map.
   */
  Parameter(bool direct_execute, bool blocking, double timeout, std::string actuate, bool tool_pose,
            bool check_collision, const std::string& frame = "base_link") {
    is_direct_execute = direct_execute;
    is_blocking = blocking;
    timeout_second = timeout;
    actuate_type = g_actuate_type_map[actuate];
    is_tool_pose = tool_pose;
    is_check_collision = check_collision;
  }

  // ---------- Setter methods ----------

  /**
   * @brief Set direct execution mode.
   *
   * @param direct_execute If true, planned motion is immediately executed on the robot;
   *                       if false, only computes the trajectory without execution
   */
  void setDirectExecute(bool direct_execute) { is_direct_execute = direct_execute; }

  /**
   * @brief Set blocking execution mode.
   *
   * @param blocking If true, function blocks until motion completes or times out;
   *                 if false, returns immediately after sending motion command
   */
  void setBlocking(bool blocking) { is_blocking = blocking; }

  /**
   * @brief Set motion execution timeout.
   *
   * @param timeout Maximum allowed time for motion execution (in seconds, must be positive)
   *
   * @note Only applies when blocking mode is enabled.
   */
  void setTimeout(double timeout) { timeout_second = timeout; }

  /**
   * @brief Set actuation type for whole-body coordination.
   *
   * @param actuate Actuation type string key: "with_chain_only" (arms only),
   *                "with_torso" (arms + torso), or "with_leg" (arms + legs)
   *
   * @warning Must be a valid key in g_actuate_type_map, otherwise behavior is undefined.
   */
  void setActuate(const std::string& actuate) { actuate_type = g_actuate_type_map[actuate]; }

  /**
   * @brief Set tool coordinate frame usage.
   *
   * @param tool_pose If true, target poses are interpreted relative to the tool frame (TCP);
   *                  if false, relative to the end-effector flange frame
   */
  void setToolPose(bool tool_pose) { is_tool_pose = tool_pose; }

  /**
   * @brief Enable or disable collision checking.
   *
   * @param check_collision If true, planner performs collision detection against
   *                        self-collisions and environment obstacles; if false, skips collision checking
   *
   * @warning Disabling collision checking may result in unsafe trajectories.
   */
  void setCheckCollision(bool check_collision) { is_check_collision = check_collision; }

  /**
   * @brief Set the reference frame for pose specifications.
   *
   * @param frame Reference frame name (e.g., "base_link", "world", "odom")
   *
   * @note Must be a valid frame in the robot's TF tree.
   */
  void setReferenceFrame(const std::string& frame) { reference_frame = frame; }

  /**
   * @brief Set Cartesian linear motion mode.
   *
   * @param flag If true, uses linear (straight-line) Cartesian motion;
   *             if false, uses joint-space interpolation (may result in curved end-effector paths)
   *
   * @note Linear motion provides predictable Cartesian paths but may have joint velocity discontinuities.
   */
  void setMoveLine(bool flag) { move_line = flag; }

  // ---------- Getter methods ----------

  /**
   * @brief Get direct execution mode status.
   *
   * @return true if direct execution is enabled, false otherwise
   */
  bool getDirectExecute() const { return is_direct_execute; }

  /**
   * @brief Get blocking execution mode status.
   *
   * @return true if blocking mode is enabled, false otherwise
   */
  bool getBlocking() const { return is_blocking; }

  /**
   * @brief Get motion execution timeout value.
   *
   * @return Timeout duration in seconds (positive value)
   */
  double getTimeout() const { return timeout_second; }

  /**
   * @brief Get actuation type as string.
   *
   * Performs reverse lookup in g_actuate_type_map to convert enum to string key.
   *
   * @return Actuation type string ("with_chain_only", "with_torso", "with_leg", or "unknown" if not found)
   */
  std::string getActuateType() const {
    for (const auto& pair : g_actuate_type_map) {
      const auto& key = pair.first;
      const auto& value = pair.second;
      if (value == actuate_type) {
        return key;
      }
    }
    return "unknown";
  }

  /**
   * @brief Get tool coordinate frame usage status.
   *
   * @return true if using tool frame (TCP), false if using end-effector flange frame
   */
  bool getToolPose() const { return is_tool_pose; }

  /**
   * @brief Get collision checking status.
   *
   * @return true if collision checking is enabled, false otherwise
   */
  bool getCheckCollision() const { return is_check_collision; }

  /**
   * @brief Get reference frame name.
   *
   * @return Reference frame identifier string (e.g., "base_link", "world")
   */
  std::string getReferenceFrame() const { return reference_frame; }

  /**
   * @brief Check if Cartesian linear motion mode is enabled.
   *
   * @return true if using linear Cartesian interpolation, false if using joint-space interpolation
   */
  bool isMoveLine() { return move_line; }
};

/**
 * @var default_param
 * @brief Default parameter object for motion planning.
 *
 * Provides a shared default Parameter instance with standard configuration values.
 * Can be used when custom parameters are not required.
 *
 * @see Parameter
 */
static std::shared_ptr<Parameter> default_param = std::make_shared<Parameter>();

/**
 * @class RobotStates
 * @brief Robot kinematic state representation.
 *
 * Encapsulates the complete kinematic state of the robot, including whole-body joint
 * configuration and mobile base pose. This class serves as a base for more specialized
 * state representations (PoseState, JointStates) and is used throughout the planning
 * and control pipeline for state specification and feedback.
 *
 * @note All angular values are in radians, linear values in meters (SI units).
 * @note Base pose uses quaternion representation for orientation (x, y, z, qx, qy, qz, qw).
 */
class RobotStates {
 public:
  /**
   * @enum Type
   * @brief Enumeration for distinguishing derived state types.
   *
   * Used for runtime type identification of RobotStates-derived classes.
   */
  enum class Type {
    POSE,          ///< PoseState: Cartesian pose target
    JOINT,         ///< JointStates: Joint space target
    ROBOT_STATES   ///< RobotStates: Generic whole-body state
  };

  /**
   * @brief Get the runtime type of this state object.
   *
   * @return Type enumeration value, ROBOT_STATES for base class
   */
  virtual Type getType() const { return Type::ROBOT_STATES; };

  std::string chain_name = "";           ///< Kinematic chain identifier (e.g., "left_arm", "right_arm")
  std::vector<double> whole_body_joint;  ///< Complete robot joint configuration (radians), ordered by joint index
  std::vector<double> base_state;        ///< Mobile base pose: [x, y, z, qx, qy, qz, qw] (meters, quaternion)

  /**
   * @brief Default constructor.
   *
   * Creates an empty RobotStates object with uninitialized joint and base states.
   */
  RobotStates() = default;

  /**
   * @brief Set complete whole-body joint configuration.
   *
   * @param joint_positions Vector of joint angles (in radians), must match robot DOF
   *
   * @note Vector size should equal the total number of actuated joints in the robot.
   */
  void setWholeBodyJoint(const std::vector<double>& joint_positions) { whole_body_joint = joint_positions; }

  /**
   * @brief Set mobile base pose.
   *
   * Converts Pose structure to internal base_state vector representation.
   *
   * @param base_pose Base pose in SE(3): position (meters) and orientation (quaternion)
   *
   * @note Quaternion must be unit-normalized (x^2 + y^2 + z^2 + w^2 = 1).
   */
  void setBaseState(const Pose& base_pose) {
    base_state = {base_pose.position.x,    base_pose.position.y,    base_pose.position.z,   base_pose.orientation.x,
                  base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w};
  }

  /**
   * @brief Parameterized constructor for complete robot state initialization.
   *
   * @param chain       Kinematic chain name (e.g., "left_arm", "right_arm")
   * @param whole_joint Complete joint configuration vector (radians)
   * @param base_pose   Mobile base pose: position (meters) + orientation (unit quaternion)
   */
  RobotStates(const std::string& chain, const std::vector<double>& whole_joint, const Pose& base_pose)
      : chain_name(chain), whole_body_joint(whole_joint) {
    base_state = {base_pose.position.x,    base_pose.position.y,    base_pose.position.z,   base_pose.orientation.x,
                  base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w};
  }
};

/**
 * @class PoseState
 * @brief Cartesian pose target specification.
 *
 * Represents a target end-effector pose in Cartesian space (SE(3)). Extends RobotStates
 * to specify pose-based motion goals for kinematic chains. Used in inverse kinematics
 * and Cartesian trajectory planning.
 *
 * @note Pose values: position in meters, orientation as unit quaternion.
 * @note Coordinate frames must exist in the robot's TF tree.
 */
class PoseState : public RobotStates {
 public:
  /**
   * @brief Get runtime type identifier.
   *
   * @return Type::POSE, indicating this is a Cartesian pose target
   */
  Type getType() const override { return Type::POSE; }

  std::string frame_id = "EndEffector";       ///< Target frame on the kinematic chain (e.g., "EndEffector", "Camera", "TCP")
  std::string reference_frame = "base_link";  ///< Reference coordinate frame (e.g., "base_link", "world", "odom")

  Pose pose;  ///< Target Cartesian pose: position (meters) + orientation (unit quaternion)

  // Pose start_pose;  ///< [Unused] Starting pose for single-point planning
  // std::unordered_map<std::string, std::vector<std::vector<double>>> multi_chain_pose_configs;
  // ///< [Future] Multi-chain pose waypoints: chain_name -> pose_list
};

/**
 * @class JointStates
 * @brief Joint-space target specification.
 *
 * Represents target joint configuration for a kinematic chain. Extends RobotStates
 * to specify joint-based motion goals. Used in joint trajectory planning and
 * forward kinematics computation.
 *
 * @note All joint angles must be in radians.
 * @note Vector size must match the DOF of the specified kinematic chain.
 */
class JointStates : public RobotStates {
 public:
  /**
   * @brief Get runtime type identifier.
   *
   * @return Type::JOINT, indicating this is a joint-space target
   */
  Type getType() const override { return Type::JOINT; }

  std::vector<double> joint_positions;  ///< Target joint configuration for the chain (radians), ordered by joint index

  // std::vector<double> start_joint_positions;  ///< [Unused] Starting joint configuration for single-point planning

  /**
   * @brief Set complete joint configuration for the kinematic chain.
   *
   * @param joints Vector of joint angles (radians), must match chain DOF
   *
   * @note Vector size should equal the number of actuated joints in the specified chain.
   */
  void set_joint_positions(const std::vector<double>& joints) { joint_positions = joints; }

  /**
   * @brief Set individual joint angle by index.
   *
   * @param index Zero-based joint index within the chain
   * @param val   Joint angle value (radians)
   *
   * @note Function performs bounds checking; invalid indices are silently ignored.
   * @warning No error is returned for out-of-bounds access; ensure index validity externally.
   */
  void set_joint(int index, int val) {
    if (joint_positions.size() > index && index >= 0) {
      joint_positions[index] = val;
    }
  }
};

/**
 * @class GalbotMotion
 * @brief Unified motion planning and control interface for Galbot robots.
 *
 * This class provides a comprehensive API for robot motion control, including:
 * - Forward and inverse kinematics computation
 * - Single-chain and multi-chain trajectory planning
 * - Collision detection (self-collision and environment)
 * - Tool and obstacle management
 * - Whole-body coordinated motion planning
 *
 * GalbotMotion follows the Singleton design pattern to ensure a single instance
 * manages all motion-related operations and robot state. Thread-safety is not
 * guaranteed; external synchronization required for multi-threaded access.
 *
 * @note All angular units are radians, linear units are meters (SI standard).
 * @note Quaternions must be unit-normalized: sqrt(x² + y² + z² + w²) = 1.
 *
 * @par Usage Example:
 * @code
 *   GalbotMotion& motion = GalbotMotion::get_instance();
 *   if (!motion.init()) {
 *     // Handle initialization failure
 *   }
 * @endcode
 */
class GalbotMotion {
 public:
  /**
   * @brief Get the singleton instance of GalbotMotion.
   *
   * Uses static local variable to ensure thread-safe lazy initialization (C++11 guarantee).
   *
   * @return Reference to the unique GalbotMotion instance
   *
   * @note First call may block briefly for initialization.
   */
  static GalbotMotion& get_instance() {
    static GalbotMotion instance;
    return instance;
  }

  /**
   * @brief Initialize motion planning system and communication interfaces.
   *
   * Must be called before any other API functions. Initializes internal communication
   * middleware, loads robot kinematic models, and establishes connections to control services.
   *
   * @return true  if initialization succeeds
   * @return false if initialization fails (check logs for details)
   *
   * @note Safe to call multiple times; subsequent calls after successful init are no-ops.
   * @warning All other API calls will fail if init() returns false.
   */
  bool init();

  /**
   * @brief Check if the motion interface is properly initialized and operational.
   *
   * @return true  if object is valid and ready for use
   * @return false if initialization failed or object is in invalid state
   *
   * @note Should be checked before critical operations if init status is uncertain.
   */
  bool is_valid();

  /**
   * @brief Compute forward kinematics for a target link.
   *
   * Calculates the Cartesian pose of a specified link given joint configurations.
   * Useful for determining end-effector positions, validating configurations, or
   * computing intermediate link poses.
   *
   * @param target_frame     Name of the link whose pose is to be computed (e.g., "left_ee_link", "camera_link")
   * @param reference_frame  Coordinate frame for pose expression (default: "base_link")
   * @param joint_state      Joint configurations by chain: {chain_name -> joint_angles}.
   *                         Empty map uses current robot joint state.
   * @param params           Planning parameters (collision checking, timeout, etc.)
   *
   * @return Tuple of (status, pose_vector):
   *         - status: MotionStatus::SUCCESS on success, error code otherwise
   *         - pose_vector: [x, y, z, qx, qy, qz, qw] (meters, quaternion) or empty on failure
   *
   * @note Joint angles in radians, output pose in meters with unit quaternion.
   * @warning target_frame must be a valid link in the URDF model.
   */
  std::tuple<MotionStatus, std::vector<double>> forward_kinematics(
      const std::string& target_frame, const std::string& reference_frame = "base_link",
      const std::unordered_map<std::string, std::vector<double>>& joint_state = {},
      std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief Compute forward kinematics using complete robot state.
   *
   * Similar to forward_kinematics(), but accepts a RobotStates object for specifying
   * the complete robot configuration (whole-body joints + base pose).
   *
   * @param target_frame            Link name for pose computation
   * @param reference_robot_states  Complete robot state; nullptr uses current robot state
   * @param reference_frame         Coordinate frame for pose expression (default: "base_link")
   * @param params                  Planning parameters
   *
   * @return Tuple of (status, pose_vector):
   *         - status: MotionStatus::SUCCESS on success, error code otherwise
   *         - pose_vector: [x, y, z, qx, qy, qz, qw] (meters, quaternion) or empty on failure
   *
   * @note Useful when computing FK for hypothetical states without modifying current robot state.
   */
  std::tuple<MotionStatus, std::vector<double>> forward_kinematics_by_state(
      const std::string& target_frame, const std::shared_ptr<RobotStates>& reference_robot_states = nullptr,
      const std::string& reference_frame = "base_link", std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief Compute inverse kinematics for target Cartesian pose.
   *
   * Solves for joint configurations that achieve the specified end-effector pose.
   * Supports single-chain IK (arm only) or coordinated multi-chain IK (arm + torso/legs).
   *
   * @param target_pose              Target Cartesian pose: [x, y, z, qx, qy, qz, qw] (meters, quaternion)
   * @param chain_names              Kinematic chains to coordinate (e.g., {"left_arm"}, {"right_arm", "torso"})
   * @param target_frame             Frame on chain for pose target (e.g., "EndEffector", "Camera", "TCP")
   * @param reference_frame          Coordinate frame for pose specification (default: "base_link")
   * @param initial_joint_positions  Seed configurations by chain: {chain_name -> joint_angles}.
   *                                 Empty map uses current robot state as seed.
   * @param enable_collision_check   If true, only returns collision-free solutions
   * @param params                   Planning parameters (timeout, actuation type, etc.)
   *
   * @return Tuple of (status, solution_map):
   *         - status: MotionStatus::SUCCESS if solvable, error code otherwise
   *         - solution_map: {chain_name -> joint_angles} (radians) for each chain, empty on failure
   *
   * @note IK may have multiple solutions; returns first valid solution found.
   * @note Seed configuration affects convergence speed and which solution is returned.
   * @warning No solution guaranteed if target is outside workspace or in singular configuration.
   */
  std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<double>>> inverse_kinematics(
      const std::vector<double>& target_pose, const std::vector<std::string>& chain_names,
      const std::string& target_frame = "EndEffector", const std::string& reference_frame = "base_link",
      const std::unordered_map<std::string, std::vector<double>>& initial_joint_positions = {},
      const bool& enable_collision_check = true, std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief Compute inverse kinematics using complete robot state as seed.
   *
   * Similar to inverse_kinematics(), but accepts RobotStates for specifying the seed
   * configuration, allowing precise control over the entire robot state.
   *
   * @param target_pose              Target Cartesian pose: [x, y, z, qx, qy, qz, qw] (meters, quaternion)
   * @param chain_names              Kinematic chains to coordinate
   * @param target_frame             Frame on chain for pose target
   * @param reference_frame          Coordinate frame for pose specification (default: "base_link")
   * @param reference_robot_states   Complete robot state as IK seed; nullptr uses current state
   * @param enable_collision_check   If true, only returns collision-free solutions
   * @param params                   Planning parameters
   *
   * @return Tuple of (status, solution_map):
   *         - status: MotionStatus::SUCCESS if solvable, error code otherwise
   *         - solution_map: {chain_name -> joint_angles} (radians), empty on failure
   *
   * @note Useful for offline planning with hypothetical robot states.
   */
  std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<double>>> inverse_kinematics_by_state(
      const std::vector<double>& target_pose, const std::vector<std::string>& chain_names,
      const std::string& target_frame = "EndEffector", const std::string& reference_frame = "base_link",
      const std::shared_ptr<RobotStates>& reference_robot_states = nullptr, const bool& enable_collision_check = true,
      std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief Get current end-effector pose from robot state.
   *
   * Queries the TF (Transform) tree to retrieve the current Cartesian pose of a
   * specified end-effector link. Requires the link to be defined in the robot's URDF model.
   *
   * @param end_effector_frame  Name of end-effector link (must exist in URDF, e.g., "left_ee_link")
   * @param reference_frame     Coordinate frame for pose expression (default: "base_link")
   *
   * @return Tuple of (status, pose_vector):
   *         - status: MotionStatus::SUCCESS on success, error codes:
   *                   - DATA_FETCH_FAILED: TF lookup failed
   *                   - INVALID_INPUT: Invalid frame names
   *         - pose_vector: [x, y, z, qx, qy, qz, qw] (meters, quaternion) or empty on failure
   *
   * @note Reflects the current actual robot state (not planned state).
   * @warning Requires TF tree to be properly published and up-to-date.
   */
  std::tuple<MotionStatus, std::vector<double>> get_end_effector_pose(const std::string& end_effector_frame,
                                                                      const std::string& reference_frame = "base_link");

  /**
   * @brief Get current end-effector pose for a specific kinematic chain.
   *
   * Convenience method for retrieving end-effector pose by chain name and frame type,
   * without needing to know the exact link name in URDF.
   *
   * @param chain_name       Kinematic chain identifier (e.g., "left_arm", "right_arm")
   * @param frame_id         End-effector frame type: "EndEffector" (flange), "Camera", etc.
   * @param reference_frame  Coordinate frame for pose expression (default: "base_link")
   *
   * @return Tuple of (status, pose_vector):
   *         - status: MotionStatus::SUCCESS on success, error code otherwise
   *         - pose_vector: [x, y, z, qx, qy, qz, qw] (meters, quaternion) or empty on failure
   *
   * @note Internally maps chain_name + frame_id to actual URDF link name.
   */
  std::tuple<MotionStatus, std::vector<double>> get_end_effector_pose_on_chain(
      const std::string& chain_name,
      const std::string frame_id = "EndEffector",  // "EndEffector", "Camera"
      const std::string& reference_frame = "base_link");

  /**
   * @brief Command end-effector to move to target Cartesian pose.
   *
   * High-level interface for Cartesian motion commands. Internally performs IK, plans trajectory,
   * and optionally executes the motion. Supports both blocking (wait for completion) and
   * non-blocking (return immediately) modes.
   *
   * @param target_pose              Target Cartesian pose: [x, y, z, qx, qy, qz, qw] (meters, quaternion)
   * @param end_effector_frame       Kinematic chain identifier (e.g., "left_arm", "right_arm")
   * @param reference_frame          Coordinate frame for pose specification (default: "base_link")
   * @param reference_robot_states   Planning seed state; nullptr uses current state.
   *                                 **Warning:** For direct execution, typically leave as nullptr to avoid
   *                                 conflicts between seed and actual robot state.
   * @param enable_collision_check   If true, only executes collision-free trajectories
   * @param is_blocking              If true, blocks until motion completes or times out; if false, returns immediately
   * @param timeout                  Blocking timeout (seconds). If < 0 and is_blocking=true, uses params->timeout_second
   * @param params                   Motion planning parameters (linear motion, actuation type, etc.)
   *
   * @return MotionStatus:
   *         - SUCCESS: Motion completed successfully (blocking) or command sent (non-blocking)
   *         - TIMEOUT: Motion exceeded timeout duration
   *         - INVALID_INPUT: Invalid pose or parameters
   *         - FAULT: Planning or execution failure
   *
   * @note Motion type (linear/joint-space) controlled by params->move_line flag.
   * @note For direct execution (params->is_direct_execute=true), avoid passing reference_robot_states.
   * @warning Blocking calls will halt execution until motion completes; use with caution in real-time contexts.
   */
  MotionStatus set_end_effector_pose(const std::vector<double>& target_pose, const std::string& end_effector_frame,
                                     const std::string& reference_frame = "base_link",
                                     std::shared_ptr<RobotStates> reference_robot_states = nullptr,
                                     const bool& enable_collision_check = true, const bool& is_blocking = true,
                                     const double& timeout = -1.0, std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief Plan trajectory for a single kinematic chain.
    *
   * Core planning interface for single-chain motion. Supports both Cartesian (PoseState)
   * and joint-space (JointStates) targets. Returns a time-parameterized joint trajectory.
    * @note Collision semantics: galbotMotion does not have real-time obstacle perception.
    * When `enable_collision_check=true`, collision checking is evaluated against self-collision and the
    * Motion-side environment objects that the user loads manually via `add_obstacle()` / `attach_target_object()`.
   *
   * @param target                   Target state (must be PoseState or JointStates, not base RobotStates).
   *                                 Specifies the goal configuration for planning.
   * @param start                    Optional start state (typically JointStates).
   *                                 nullptr uses current robot state as start.
   *                                 **Warning:** For direct execution, leave as nullptr to avoid conflicts.
   * @param reference_robot_states   Whole-body reference state for planning context.
   *                                 nullptr uses current robot state. If start is provided, its joint values
   *                                 overwrite the corresponding chain in reference_robot_states.
   *                                 **Warning:** For direct execution, leave as nullptr.
   * @param enable_collision_check   If true, only returns collision-free trajectories
   * @param params                   Planning parameters (timeout, actuation type, linear motion, etc.)
   *
   * @return Tuple of (status, trajectory_map):
   *         - status: MotionStatus::SUCCESS if planning succeeds, error code otherwise
   *         - trajectory_map: {chain_name -> waypoint_list}, where waypoint_list is a sequence
   *                           of joint configurations (radians) along the trajectory. Empty on failure.
   *
   * @note Trajectory is time-parameterized with velocity/acceleration limits respected.
   * @note For direct execution (params->is_direct_execute=true), trajectory is automatically sent to robot.
   * @warning target must be PoseState or JointStates; passing base RobotStates will cause INVALID_INPUT error.
   */
  std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>> motion_plan(
      std::shared_ptr<RobotStates> target, std::shared_ptr<RobotStates> start = nullptr,
      std::shared_ptr<RobotStates> reference_robot_states = nullptr, bool enable_collision_check = true,
      std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief Plan trajectory through multiple waypoints for a single chain.
    *
    * @note Collision semantics: same as `motion_plan()`. The collision world for Motion planning is built from
    * user-loaded objects and is not updated by real-time perception automatically.
   *
   * Generates a smooth trajectory that passes through a sequence of waypoints.
   * Useful for complex motions like pick-and-place or obstacle avoidance paths.
   *
   * @param target                   Template state defining waypoint type (PoseState or JointStates)
   *                                 and specifying chain_name. The state values in target are not used;
   *                                 only the type and chain_name are referenced.
   * @param targets                  Sequence of waypoint values. Format depends on target type:
   *                                 - PoseState: each waypoint is [x, y, z, qx, qy, qz, qw] (meters, quaternion)
   *                                 - JointStates: each waypoint is joint configuration (radians)
   * @param start                    Optional start state (JointStates). nullptr uses current robot state.
   *                                 **Warning:** For direct execution, leave as nullptr.
   * @param reference_robot_states   Whole-body reference state for planning context. nullptr uses current state.
   *                                 **Warning:** For direct execution, leave as nullptr.
   * @param enable_collision_check   If true, only returns collision-free trajectories
   * @param params                   Planning parameters
   *
   * @return Tuple of (status, trajectory_map):
   *         - status: MotionStatus::SUCCESS if planning succeeds, error code otherwise
   *         - trajectory_map: {chain_name -> waypoint_list}, smooth trajectory through all waypoints
   *
   * @note Planner ensures C1 continuity (continuous velocity) at waypoints.
   * @note Intermediate waypoints may not be exactly reached (blending); use multiple plans for exact passing.
   */
  std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>>
  motion_plan_multi_waypoints(std::shared_ptr<RobotStates> target, std::vector<std::vector<double>> targets,
                              std::shared_ptr<RobotStates> start = nullptr,
                              std::shared_ptr<RobotStates> reference_robot_states = nullptr,
                              bool enable_collision_check = true, std::shared_ptr<Parameter> params = default_param);

    /**
   * @brief Plan coordinated trajectories through waypoints for multiple chains.
   *
   * Enables coordinated multi-arm or whole-body motion through waypoint sequences.
   * Each chain can have its own waypoint sequence, executed in synchronized fashion.
   *
   * @param targets                  Map of {state_template -> waypoint_list} for each chain.
   *                                 Keys are RobotStates (with chain_name set) defining waypoint type;
   *                                 values are waypoint sequences in same format as single-chain version.
   * @param start                    Optional start states (JointStates) for each chain. Empty uses current state.
   *                                 **Warning:** For direct execution, leave empty.
   * @param reference_robot_states   Whole-body reference state for planning context. nullptr uses current state.
   *                                 **Warning:** For direct execution, leave as nullptr.
   * @param enable_collision_check   If true, only returns collision-free trajectories
   * @param params                   Planning parameters
   *
   * @return Tuple of (status, trajectory_map):
   *         - status: MotionStatus::SUCCESS if planning succeeds, error code otherwise
   *         - trajectory_map: {chain_name -> waypoint_list} for all chains
   *
   * @note All chain trajectories are time-synchronized for coordinated execution.
   * @note Useful for bimanual manipulation or mobile manipulation tasks.
   */
   std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>>
   motion_plan_multi_waypoints(std::unordered_map<std::shared_ptr<RobotStates>, std::vector<std::vector<double>>> targets, std::vector<std::shared_ptr<RobotStates>> start = {},
                              std::shared_ptr<RobotStates> reference_robot_states = nullptr,
                              bool enable_collision_check = true, std::shared_ptr<Parameter> params = default_param);


  /**
   * @note [Obstacle perception & point-cloud usage: galbotNav vs galbotMotion]
   *
   * - galbotNav (navigation): during navigation (e.g., `navigate_to_goal()` with `enable_collision_check=true`),
   *   the navigation stack performs obstacle detection/avoidance and may leverage real-time environment perception
   *   (e.g., point-cloud map / grid map / ESDF depending on deployment).
   *
   * - galbotMotion (whole-body planning/manipulation): this module does NOT currently provide real-time obstacle
   *   perception or automatic environment updates. Collision checking for Motion planning is based on:
   *     1) self-collision, and
   *     2) an obstacle/collision world that the user loads manually via `add_obstacle()` / `attach_target_object()`.
   *
   * Therefore, if you need Motion to consider environmental obstacles (including point clouds), you must load the
   * obstacle map/objects explicitly (e.g., `obstacle_type = point_cloud` with a file path in `key`).
   *
   * Note: integrating real-time perception (navigation-style obstacle updates / point-cloud map) into galbotMotion
   * is a planned future feature and has limited internal validation at the moment.
   */


  /**
   * @brief Check robot states for collisions.
   *
   * Validates whether given robot configurations are collision-free. Checks both
   * self-collisions (robot links with each other) and environment collisions
   * (robot with scene obstacles). Batch processing supported for efficiency.
   *
   * @param start                   Vector of robot states to check. Each state can be:
   *                                - RobotStates: complete whole-body configuration
   *                                - JointStates: single-chain configuration (other joints use current state)
   * @param enable_collision_check  If true, checks both self and environment collisions;
   *                                if false, only checks self-collisions
   * @param params                  Optional parameters
   *
   * @return Tuple of (status, collision_results):
   *         - status: MotionStatus::SUCCESS if check completes, error code otherwise
   *         - collision_results: Boolean vector (same size as start):
   *                              true = collision detected, false = collision-free
   *
   * @note Useful for validating planned trajectories or sampling-based planners.
   * @note Respects safe_margin settings in previously added obstacles.
   */
  std::tuple<MotionStatus, std::vector<bool>> check_collision(const std::vector<std::shared_ptr<RobotStates>>& start,
                                                              bool enable_collision_check = true,
                                                              std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief Attach a tool to an end-effector.
   *
   * Loads a tool (gripper, camera, custom end-effector) onto a kinematic chain.
   * Updates the kinematic model and collision geometry to include the tool.
   *
   * @param chain  Kinematic chain for tool attachment (e.g., "left_arm", "right_arm")
   * @param tool   Tool identifier (must be predefined in tool library, see getSupportToolList())
   *
   * @return MotionStatus:
   *         - SUCCESS: Tool attached successfully
   *         - INVALID_INPUT: Invalid chain or tool name
   *         - FAULT: Tool attachment failed
   *
   * @note Tool transform and collision geometry must be pre-configured in robot description.
   * @note Attaching a new tool automatically detaches any previously attached tool on that chain.
   * @warning Kinematics and collision checking will reflect the attached tool; update plans accordingly.
   */
  MotionStatus attach_tool(const std::string& chain, const std::string& tool);

  /**
   * @brief Detach the current tool from an end-effector.
   *
   * Removes the attached tool from a kinematic chain, reverting to the base end-effector.
   * Updates kinematic model and collision geometry accordingly.
   *
   * @param chain  Kinematic chain to detach tool from (e.g., "left_arm", "right_arm")
   *
   * @return MotionStatus:
   *         - SUCCESS: Tool detached successfully
   *         - INVALID_INPUT: Invalid chain name or no tool attached
   *         - FAULT: Tool detachment failed
   *
   * @note If no tool is attached, operation succeeds as a no-op.
   */
  MotionStatus detach_tool(const std::string& chain);
  /**
   * @brief Load collision object into environment
   *
   * Inserts a geometric or mesh-based obstacle into the environment for collision avoidance.
   * Obstacles can be static (world-fixed) or robot-relative. Supports primitive shapes,
   * meshes, point clouds, and depth images.
   *
  * @note Point-cloud note: `point_cloud` here refers to a point-cloud obstacle explicitly loaded via this API
  * (typically from a file/offline data). It is NOT the same as a navigation-maintained point-cloud map.
  * galbotMotion does not automatically subscribe to or synchronize with galbotNav's point-cloud map for collision.
   * @param obstacle_id                   Unique obstacle identifier (must not exist in scene).
   *                                      Used for later removal/updates.
   * @param obstacle_type                 Obstacle geometry type (e.g., "box", "sphere", "cylinder",
   *                                      "mesh", "point_cloud", "depth_image").
   *                                      See getSupportObstacleType() for valid types.
   * @param pose                          Obstacle pose: [x, y, z, qx, qy, qz, qw] (meters, quaternion)
   *                                      relative to target_frame.
   * @param scale                         Geometry dimensions (meters):
   *                                      - box: [length, width, height]
   *                                      - sphere: [radius, -, -]
   *                                      - cylinder: [radius, height, -]
   *                                      - mesh/point_cloud: scaling factors [sx, sy, sz]
   * @param key                           Type-specific data:
   *                                      - mesh/point_cloud: file path (e.g., "/path/to/model.stl")
   *                                      - depth_image: camera source ("front_head", "left_arm", "right_arm")
   *                                      - primitives: unused, leave empty
   * @param target_frame                  Reference frame for pose (default: "world").
   *                                      Can be "world", "base_link", or chain name (e.g., "left_arm").
   * @param ee_frame                      If target_frame is a chain, specifies frame on chain:
   *                                      "ee_base" (end-effector), "camera_base", "camera_object".
   * @param reference_joint_positions     Robot joint state for computing frame transforms (radians).
   *                                      Empty uses current robot state.
   * @param reference_base_pose           Robot base pose in map frame: [x, y, z, qx, qy, qz, qw].
   *                                      Empty uses current localization.
   * @param ignore_collision_link_names   Robot links to exclude from collision with this obstacle.
   *                                      Useful for carried objects or mounting surfaces.
   * @param safe_margin                   Safety distance buffer (meters). Collision reported if
   *                                      distance < safe_margin. Default: 0 (contact-based).
   * @param resolution                    Discretization resolution (meters) for complex geometries
   *                                      (mesh, point cloud, depth image). Default: 0.01m.
   *
   * @return MotionStatus:
   *         - SUCCESS: Obstacle added successfully
   *         - INVALID_INPUT: Invalid obstacle_id (duplicate), type, or parameters
   *         - FAULT: Failed to process geometry or add to scene
   *
   * @note Obstacles persist until explicitly removed or cleared.
   * @note For moving obstacles, remove and re-add at new poses (no update method currently).
   * @warning Large safe_margin values may over-constrain planning; use conservatively.
   */
  MotionStatus add_obstacle(const std::string& obstacle_id, const std::string& obstacle_type,
                            const std::vector<double>& pose, const std::array<double, 3>& scale = {},
                            const std::string& key = "", const std::string& target_frame = "world",
                            const std::string& ee_frame = "ee_base",
                            const std::vector<double>& reference_joint_positions = {},
                            const std::vector<double>& reference_base_pose = {},
                            const std::vector<std::string>& ignore_collision_link_names = {},
                            const double& safe_margin = 0, const double& resolution = 0.01);

  /**
   * @brief Remove a collision obstacle from the planning scene.
   *
   * @param obstacle_id  Unique identifier of obstacle to remove (must exist in scene)
   *
   * @return MotionStatus:
   *         - SUCCESS: Obstacle removed successfully
   *         - INVALID_INPUT: obstacle_id not found in scene
   *         - FAULT: Removal operation failed
   *
   * @note Removing a non-existent obstacle returns INVALID_INPUT (not silently ignored).
   */
  MotionStatus remove_obstacle(const std::string& obstacle_id);

  /**
   * @brief Remove all collision obstacles from the planning scene.
   *
   * Clears the entire obstacle set, resetting the planning scene to empty (except robot geometry).
   *
   * @return MotionStatus:
   *         - SUCCESS: All obstacles cleared successfully
   *         - FAULT: Clear operation failed
   *
   * @note Attached objects (see attach_target_object) are not affected.
   * @note Safe to call even if scene is already empty.
   */
  MotionStatus clear_obstacle();
  /**
   * @brief Attach a collision object to the robot (e.g., grasped object).
   *
   * Similar to add_obstacle(), but the object moves with the robot (attached to a link/chain).
   * Used for representing grasped objects, sensors, or payloads. The object's pose is
   * maintained relative to the attachment frame during motion.
   *
   * @param obstacle_id                   Unique object identifier (must not exist in scene).
   * @param obstacle_type                 Object geometry type (e.g., "box", "sphere", "mesh").
   *                                      See getSupportObstacleType() for valid types.
   * @param pose                          Object pose: [x, y, z, qx, qy, qz, qw] (meters, quaternion)
   *                                      relative to target_frame at attachment time.
   * @param scale                         Geometry dimensions (meters):
   *                                      - box: [length, width, height]
   *                                      - sphere: [radius, -, -]
   *                                      - cylinder: [radius, height, -]
   * @param key                           Type-specific data (e.g., mesh file path for "mesh" type).
  * @note Point-cloud note: same as `add_obstacle()`. `point_cloud` here is an explicitly loaded point-cloud object
  * and will not be automatically synchronized with any navigation-side point-cloud map.
   * @param target_frame                  Attachment frame (default: "world"). Typically a chain name
   *                                      (e.g., "left_arm") for grasped objects.
   * @param ee_frame                      If target_frame is a chain, specifies frame on chain
   *                                      ("ee_base", "camera_base", etc.).
   * @param reference_joint_positions     Robot joint state for computing attachment transform (radians).
   *                                      Empty uses current robot state.
   * @param reference_base_pose           Robot base pose in map: [x, y, z, qx, qy, qz, qw].
   *                                      Empty uses current localization.
   * @param ignore_collision_link_names   Robot links to exclude from collision with this object.
   *                                      Typically includes the grasping end-effector links.
   * @param safe_margin                   Safety distance buffer (meters). Default: 0.
   * @param resolution                    Discretization resolution for complex geometries (meters). Default: 0.01m.
   *
   * @return MotionStatus:
   *         - SUCCESS: Object attached successfully
   *         - INVALID_INPUT: Invalid parameters or duplicate obstacle_id
   *         - FAULT: Attachment operation failed
   *
   * @note Attached objects move with the robot; their collision geometry is updated automatically.
   * @note Typically used in pick-and-place: attach_target_object after grasp, detach after release.
   * @warning Ensure ignore_collision_link_names includes grasping links to avoid false collisions.
   *
   * @par Usage Example (grasping):
   * @code
   *   // After successful grasp
   *   motion.attach_target_object("grasped_box", "box", pose, {0.05, 0.05, 0.1},
   *                               "", "right_arm", "ee_base", {}, {},
   *                               {"right_gripper_left_finger", "right_gripper_right_finger"});
   * @endcode
   */
  MotionStatus attach_target_object(const std::string& obstacle_id, const std::string& obstacle_type,
                                    const std::vector<double>& pose, const std::array<double, 3>& scale = {},
                                    const std::string& key = "", const std::string& target_frame = "world",
                                    const std::string& ee_frame = "ee_base",
                                    const std::vector<double>& reference_joint_positions = {},
                                    const std::vector<double>& reference_base_pose = {},
                                    const std::vector<std::string>& ignore_collision_link_names = {},
                                    const double& safe_margin = 0, const double& resolution = 0.01);

  /**
   * @brief Detach an object from the robot (e.g., after release).
   *
   * Removes an attached object from the robot. Typically called after releasing a grasped object.
   * The object is removed from the planning scene entirely (not converted to a static obstacle).
   *
   * @param obstacle_id  Unique identifier of attached object to remove
   *
   * @return MotionStatus:
   *         - SUCCESS: Object detached successfully
   *         - INVALID_INPUT: obstacle_id not found in attached objects
   *         - FAULT: Detachment operation failed
   *
   * @note To keep the object in the scene as a static obstacle after release, call
   *       detach_target_object() then add_obstacle() with the object's final pose.
   *
   * @par Usage Example (release):
   * @code
   *   // After placing object
   *   motion.detach_target_object("grasped_box");
   *   // Optionally add as static obstacle at placement location
   *   motion.add_obstacle("placed_box", "box", final_pose, {0.05, 0.05, 0.1});
   * @endcode
   */
  MotionStatus detach_target_object(const std::string& obstacle_id);

  /**
   * @brief Set global motion planning configuration.
   *
   * Updates planner settings such as velocity/acceleration limits, planning algorithm
   * parameters, and optimization objectives. Affects all subsequent planning operations.
   *
   * @param config  Shared pointer to MotionPlanConfig with desired settings
   *
   * @return MotionStatus:
   *         - SUCCESS: Configuration applied successfully
   *         - INVALID_INPUT: Invalid configuration parameters
   *         - FAULT: Configuration update failed
   *
   * @note Changes persist until explicitly reset or process restart.
   * @note See MotionPlanConfig documentation for available parameters.
   */
  MotionStatus set_motion_plan_config(std::shared_ptr<MotionPlanConfig> config);

  /**
   * @brief Get current motion planning configuration.
   *
   * Retrieves the active planner configuration, including velocity/acceleration limits
   * and planning algorithm parameters.
   *
   * @return Tuple of (status, config):
   *         - status: MotionStatus::SUCCESS on success, error code otherwise
   *         - config: Current MotionPlanConfig object (default-constructed on failure)
   *
   * @note Useful for inspecting current limits or saving/restoring configurations.
   */
  std::tuple<MotionStatus, MotionPlanConfig> get_motion_plan_config();

  /**
   * @brief Get robot link names from kinematic model.
   *
   * Retrieves the list of link names defined in the robot's URDF model.
   * Can filter to only end-effector links or return all links.
   *
   * @param only_end_effector  If true, returns only end-effector/tool links;
   *                           if false, returns all links including base, intermediate, and end-effector links.
   *                           Default: false (all links).
   *
   * @return Vector of link name strings (empty if retrieval fails)
   *
   * @note End-effector detection based on link having no child links in kinematic tree.
   * @note Useful for forward kinematics queries or TF frame validation.
   */
  std::vector<std::string> get_link_names(bool only_end_effector = false);

 public:
  // ---------- Capability query methods ----------

  /**
   * @brief Get set of valid link names in robot model.
   *
   * Returns all link names defined in the loaded URDF kinematic model.
   * Useful for validating forward kinematics queries or TF lookups.
   *
   * @return Const reference to set of supported link name strings
   *
   * @note Set is populated during initialization from robot URDF.
   */
  const std::set<std::string>& getSupportLinks();

  /**
   * @brief Get set of valid kinematic chain names.
   *
   * Returns the names of all predefined kinematic chains (e.g., "left_arm", "right_arm",
   * "torso", "left_leg", "right_leg"). Chains are groups of joints treated as a unit
   * for planning and control.
   *
   * @return Const reference to set of supported chain name strings
   *
   * @note Chain definitions are specified in robot configuration files.
   */
  const std::set<std::string>& getSupportChains();

  /**
   * @brief Get set of valid reference frame names.
   *
   * Returns standard reference frames supported for pose specifications
   * (e.g., "base_link", "world", "odom").
   *
   * @return Const reference to set of supported reference frame name strings
   *
   * @note These are global/base frames; individual link frames accessed via TF.
   */
  const std::set<std::string>& getSupportFrame();

  /**
   * @brief Get set of valid end-effector frame types.
   *
   * Returns frame identifiers that can be specified for end-effector poses
   * (e.g., "EndEffector", "Camera", "TCP"). These are frame types, not specific link names.
   *
   * @return Const reference to set of supported end-effector frame type strings
   *
   * @note Actual link names derived by combining chain name + frame type.
   */
  const std::set<std::string>& getSupportEEFrame();

  /**
   * @brief Get list of available tools that can be attached.
   *
   * Returns names of predefined tools (grippers, sensors, custom end-effectors)
   * that can be attached via attach_tool().
   *
   * @return Const reference to set of tool name strings
   *
   * @note Tools must be pre-configured with geometry and kinematic offsets.
   */
  const std::set<std::string>& getSupportToolList();

  /**
   * @brief Get list of supported collision obstacle geometry types.
   *
   * Returns geometry types supported by add_obstacle() and attach_target_object()
   * (e.g., "box", "sphere", "cylinder", "mesh", "point_cloud", "depth_image").
   *
   * @return Set of supported obstacle type strings
   *
   * @note Different types require different scale/key parameter formats.
   */
  std::set<std::string> getSupportObstacleType();

  /**
   * @brief Get list of currently added obstacles in planning scene.
   *
   * Returns the obstacle IDs of all obstacles currently present in the scene
   * (both static obstacles and attached objects).
   *
   * @return Vector of obstacle ID strings (empty if no obstacles)
   *
   * @note Useful for debugging scene state or preventing duplicate IDs.
   */
  std::vector<std::string> getBuiltObjectList();

  // ---------- Validation methods ----------

  /**
   * @brief Validate link name against robot model.
   *
   * Checks if the provided link name exists in the loaded URDF model.
   *
   * @param value            Link name to validate
   * @param throw_exception  If true, throws std::invalid_argument on validation failure;
   *                         if false, returns false silently. Default: false.
   *
   * @return true if link name is valid, false otherwise
   *
   * @throws std::invalid_argument if throw_exception=true and validation fails
   *
   * @note Use getSupportLinks() to retrieve valid link names.
   */
  bool isLinkNameValid(const std::string& value, bool throw_exception = false);

  /**
   * @brief Validate kinematic chain name.
   *
   * Checks if the provided chain name is defined in robot configuration.
   *
   * @param value            Chain name to validate (e.g., "left_arm", "right_arm")
   * @param throw_exception  If true, throws exception on failure. Default: false.
   *
   * @return true if chain name is valid, false otherwise
   *
   * @throws std::invalid_argument if throw_exception=true and validation fails
   *
   * @note Use getSupportChains() to retrieve valid chain names.
   */
  bool isChainNameValid(const std::string& value, bool throw_exception = false);

  /**
   * @brief Validate reference frame name.
   *
   * Checks if the provided frame name is a valid reference frame for pose specifications.
   *
   * @param value            Frame name to validate (e.g., "base_link", "world")
   * @param throw_exception  If true, throws exception on failure. Default: false.
   *
   * @return true if frame name is valid, false otherwise
   *
   * @throws std::invalid_argument if throw_exception=true and validation fails
   *
   * @note Use getSupportFrame() to retrieve valid frame names.
   */
  bool isFrameNameValid(const std::string& value, bool throw_exception = false);

  /**
   * @brief Validate end-effector frame type.
   *
   * Checks if the provided frame type is valid for end-effector specifications.
   *
   * @param value            Frame type to validate (e.g., "EndEffector", "Camera", "TCP")
   * @param throw_exception  If true, throws exception on failure. Default: false.
   *
   * @return true if frame type is valid, false otherwise
   *
   * @throws std::invalid_argument if throw_exception=true and validation fails
   *
   * @note Use getSupportEEFrame() to retrieve valid frame types.
   */
  bool isEEFrameValid(const std::string& value, bool throw_exception = false);

  /**
   * @brief Validate tool name.
   *
   * Checks if the provided tool name is available for attachment.
   *
   * @param value            Tool name to validate
   * @param throw_exception  If true, throws exception on failure. Default: false.
   *
   * @return true if tool name is valid, false otherwise
   *
   * @throws std::invalid_argument if throw_exception=true and validation fails
   *
   * @note Use getSupportToolList() to retrieve available tools.
   */
  bool isToolNameValid(const std::string& value, bool throw_exception = false);

  /**
   * @brief Validate obstacle geometry type.
   *
   * Checks if the provided type is supported for obstacle creation.
   *
   * @param value            Obstacle type to validate (e.g., "box", "sphere", "mesh")
   * @param throw_exception  If true, throws exception on failure. Default: false.
   *
   * @return true if obstacle type is valid, false otherwise
   *
   * @throws std::invalid_argument if throw_exception=true and validation fails
   *
   * @note Use getSupportObstacleType() to retrieve supported types.
   */
  bool isObstacleTypeValid(const std::string& value, bool throw_exception = false);

  /**
   * @brief Validate whole-body joint configuration vector.
   *
   * Checks if the provided vector has the correct size (robot DOF) for whole-body state.
   *
   * @param value            Joint configuration vector (radians) to validate
   * @param throw_exception  If true, throws exception on failure. Default: false.
   *
   * @return true if vector size matches robot DOF, false otherwise
   *
   * @throws std::invalid_argument if throw_exception=true and validation fails
   *
   * @note Expected size: getRobotDof() elements.
   * @note Does not validate joint limit compliance, only vector dimensionality.
   */
  bool isWholeBodyStateValid(const std::vector<double>& value, bool throw_exception = false);

  /**
   * @brief Validate 7D pose vector (position + quaternion).
   *
   * Checks if the provided pose vector has exactly 7 elements: [x, y, z, qx, qy, qz, qw].
   * Optionally validates quaternion normalization.
   *
   * @param value            Pose vector to validate
   * @param throw_exception  If true, throws exception on failure. Default: false.
   *
   * @return true if vector size is 7, false otherwise
   *
   * @throws std::invalid_argument if throw_exception=true and validation fails
   *
   * @note Does not enforce quaternion normalization; only checks dimensionality.
   * @warning Using non-normalized quaternions will cause undefined behavior in kinematics.
   */
  bool isPose7dValid(const std::vector<double>& value, bool throw_exception = false);

  /**
   * @brief Validate chain-indexed joint configuration map.
   *
   * Checks if chain names are valid and joint vectors have correct dimensions for each chain.
   *
   * @param value            Map of {chain_name -> joint_configuration} to validate
   * @param throw_exception  If true, throws exception on failure. Default: false.
   *
   * @return true if all chain names and vector sizes are valid, false otherwise
   *
   * @throws std::invalid_argument if throw_exception=true and validation fails
   *
   * @note Validates both chain name existence and joint vector dimensionality per chain.
   */
  bool isChainJointStateValid(const std::unordered_map<std::string, std::vector<double>>& value,
                              bool throw_exception = false);

  // ---------- State query methods ----------

  /**
   * @brief Get robot total degrees of freedom (DOF).
   *
   * Returns the number of actuated joints in the complete robot (whole-body).
   * Used for validating joint state vector dimensions.
   *
   * @return Number of robot DOF (actuated joints)
   *
   * @note Typical humanoid/mobile manipulator: 15-30 DOF.
   */
  int getRobotDof() { return robot_dof_; }

  /**
   * @brief Get current complete robot state.
   *
   * Retrieves the current whole-body joint configuration and mobile base pose.
   * Represents the full kinematic state of the robot.
   *
   * @return RobotStates object containing:
   *         - whole_body_joint: complete joint configuration (radians)
   *         - base_state: mobile base pose [x, y, z, qx, qy, qz, qw] (meters, quaternion)
   *
   * @note Reflects actual robot state (from sensor feedback/state estimation).
   * @note Useful as seed/reference for planning operations.
   */
  RobotStates getRobotStates();

  /**
   * @brief Get current whole-body joint configuration.
   *
   * Retrieves joint angles for all actuated joints in the robot.
   *
   * @return Vector of joint angles (radians), size = getRobotDof()
   *
   * @note Joint ordering defined by robot URDF/configuration.
   */
  std::vector<double> getWholeBodyState();

  /**
   * @brief Get current mobile base pose.
   *
   * Retrieves the position and orientation of the robot's mobile base in the map frame.
   *
   * @return Base pose vector: [x, y, z, qx, qy, qz, qw] (meters, unit quaternion)
   *
   * @note For non-mobile robots, typically returns identity pose.
   * @note Pose obtained from localization/odometry system.
   */
  std::vector<double> getChassisState();

  /**
   * @brief Get current joint configurations for all kinematic chains.
   *
   * Retrieves per-chain joint states, decomposing the whole-body configuration
   * into individual chain contributions.
   *
   * @return Map of {chain_name -> joint_configuration} (radians) for each chain
   *         (e.g., {"left_arm" -> [7 joint angles], "right_arm" -> [7 joint angles]})
   *
   * @note Joint vector sizes vary by chain DOF.
   */
  std::unordered_map<std::string, std::vector<double>> getChainJointState();

  /**
   * @brief Get current Cartesian poses for all kinematic chains.
   *
   * Computes forward kinematics for all chain end-effectors, returning their poses
   * in the specified reference frame.
   *
   * @param frame  Reference frame for pose expression (default: "base_link")
   *
   * @return Map of {chain_name -> pose_vector}, where pose_vector is
   *         [x, y, z, qx, qy, qz, qw] (meters, quaternion) for each chain's end-effector
   *
   * @note Computationally more expensive than getChainJointState() (requires FK computation).
   */
  std::unordered_map<std::string, std::vector<double>> getChainPoseState(const std::string& frame = "base_link");

  /**
   * @brief Update a specific chain's joints in a whole-body configuration.
   *
   * Utility function for modifying a chain's joint values within a complete whole-body
   * joint vector, while preserving other chains' states.
   *
   * @param chain_name       Chain identifier whose joints to replace
   * @param chain_joint      New joint configuration for the chain (radians)
   * @param whole_body_joint Whole-body joint vector to modify (in/out parameter)
   *
   * @return true if replacement succeeds, false if chain_name invalid or size mismatch
   *
   * @note whole_body_joint is modified in-place; ensure it has correct size (getRobotDof()).
   * @note Useful for offline state manipulation or custom planning seeds.
   */
  bool replace_joint_state(const std::string& chain_name, const std::vector<double>& chain_joint,
                           std::vector<double>& whole_body_joint);

  // ---------- Utility methods ----------

  /**
   * @brief Convert MotionStatus enum to human-readable string.
   *
   * Maps status codes to descriptive strings for logging, error reporting, or UI display.
   *
   * @param status  MotionStatus enumeration value
   *
   * @return Descriptive status string (e.g., "SUCCESS: Execution succeeded",
   *         "TIMEOUT: Execution timeout")
   *
   * @note Uses status_string_map_ for lookup; returns "UNKNOWN" if status not found.
   */
  std::string status_to_string(MotionStatus status);

 private:
  // ---------- Internal helper methods ----------

  /**
   * @brief Validate chain names and determine actuation type.
   *
   * Internal method for validating a set of chain names and inferring the appropriate
   * ActuateType based on the combination (e.g., arm-only, arm+torso, arm+legs).
   *
   * @param chain_names  Vector of chain names to validate and classify
   *
   * @return Tuple of (error_message, actuate_type_string):
   *         - error_message: Empty if valid, error description if invalid
   *         - actuate_type_string: Actuation type key (e.g., "with_chain_only", "with_torso")
   *
   * @note Used internally by IK and planning methods to validate chain combinations.
   */
  std::tuple<std::string, std::string> validateAndGetChainType(const std::vector<std::string>& chain_names);

  /**
   * @brief Convert pose vector to Pose structure.
   *
   * Internal utility for converting 7-element pose vectors to structured Pose objects.
   *
   * @param custom_pose  Pose as vector: [x, y, z, qx, qy, qz, qw] (meters, quaternion)
   *
   * @return Pose object (default-constructed if input size != 7)
   */
  Pose convert_custom_pose(const std::vector<double>& custom_pose) {
    Pose pose;
    if (custom_pose.size() == 7) {
      pose.position.x = custom_pose[0];
      pose.position.y = custom_pose[1];
      pose.position.z = custom_pose[2];
      pose.orientation.x = custom_pose[3];
      pose.orientation.y = custom_pose[4];
      pose.orientation.z = custom_pose[5];
      pose.orientation.w = custom_pose[6];
    }
    return pose;
  }

  /**
   * @brief Convert Pose structure to pose vector.
   *
   * Internal utility for converting structured Pose objects to 7-element vectors.
   *
   * @param pose  Pose object to convert
   *
   * @return Vector: [x, y, z, qx, qy, qz, qw] (meters, quaternion)
   */
  std::vector<double> convert_pose_to_vec(const Pose& pose) {
    std::vector<double> vec = std::vector<double>(7);
    vec[0] = pose.position.x;
    vec[1] = pose.position.y;
    vec[2] = pose.position.z;
    vec[3] = pose.orientation.x;
    vec[4] = pose.orientation.y;
    vec[5] = pose.orientation.z;
    vec[6] = pose.orientation.w;
    return vec;
  }

  /**
   * @brief Internal unified obstacle/attached object addition.
   *
   * Common implementation for add_obstacle() and attach_target_object().
   *
   * @param attached_frame  Empty for static obstacles, frame name for attached objects
   * @param obstacle_id     Unique object identifier
   * @param obstacle_type   Geometry type
   * @param pose            Object pose
   * @param scale           Geometry dimensions
   * @param key             Type-specific data
   * @param target_frame    Reference frame for pose
   * @param ee_frame        End-effector frame (if target_frame is chain)
   * @param reference_joint_positions  Robot joint state for transform computation
   * @param reference_base_pose        Robot base pose in map
   * @param ignore_collision_link_names  Links to exclude from collision
   * @param safe_margin     Safety distance buffer
   * @param resolution      Discretization resolution
   *
   * @return MotionStatus indicating success/failure
   */
  MotionStatus add_object(const std::string& attached_frame, const std::string& obstacle_id,
                          const std::string& obstacle_type, const std::vector<double>& pose,
                          const std::array<double, 3>& scale = {}, const std::string& key = "",
                          const std::string& target_frame = "world", const std::string& ee_frame = "ee_base",
                          const std::vector<double>& reference_joint_positions = {},
                          const std::vector<double>& reference_base_pose = {},
                          const std::vector<std::string>& ignore_collision_link_names = {},
                          const double& safe_margin = 0, const double& resolution = 0.01);

  /**
   * @brief Internal unified obstacle/attached object removal.
   *
   * Common implementation for remove_obstacle() and detach_target_object().
   *
   * @param attached_frame  Empty for static obstacles, frame name for attached objects
   * @param obstacle_id     Unique object identifier to remove
   *
   * @return MotionStatus indicating success/failure
   */
  MotionStatus remove_object(const std::string& attached_frame, const std::string& obstacle_id);

  /**
   * @brief Internal unified scene clearing.
   *
   * Common implementation for clearing obstacles/attached objects.
   *
   * @param attached_frame  Empty for static obstacles, frame name for attached objects
   *
   * @return MotionStatus indicating success/failure
   */
  MotionStatus clear_object(const std::string& attached_frame);

 private:
  // ---------- Singleton enforcement (non-copyable, non-movable) ----------
  
  GalbotMotion() = default;                                  ///< Private default constructor
  GalbotMotion(const GalbotMotion&) = delete;               ///< Copy constructor deleted
  GalbotMotion& operator=(const GalbotMotion&) = delete;    ///< Copy assignment deleted
  GalbotMotion(GalbotMotion&&) = delete;                    ///< Move constructor deleted
  GalbotMotion& operator=(GalbotMotion&&) = delete;         ///< Move assignment deleted

  // ---------- Member variables ----------
  
  int robot_dof_ = 21;  ///< Robot degrees of freedom (total actuated joints), default: 21
};

}  // namespace g1
}  // namespace sdk
}  // namespace galbot