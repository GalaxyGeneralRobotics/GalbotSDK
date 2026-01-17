/**
 * @file motion_plan_config.hpp
 * @brief Motion planning configuration structures and classes for robot motion control
 *
 * This file defines comprehensive configuration structures for robot motion planning,
 * including kinematic boundaries, sampling strategies, trajectory planning parameters,
 * inverse kinematics solver settings, collision detection options, and feasibility
 * checking configurations. These structures provide a flexible and type-safe interface
 * for configuring complex motion planning tasks in robotic systems.
 *
 * @author Galbot SDK Team
 * @version 1.5.1
 * @copyright Copyright (c) 2023-2026 Galbot. All rights reserved.
 */

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

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
 * @struct KinematicsBoundary
 * @brief Kinematic boundary parameters for robot kinematic chain joints
 *
 * This structure defines the kinematic constraints for a robot kinematic chain
 * (e.g., manipulator arms, mobile base, or leg chains). It specifies position,
 * velocity, acceleration, and jerk limits for each joint in the chain.
 *
 * These boundaries are critical for ensuring safe and physically feasible motion
 * during trajectory planning and execution. Each vector should contain one value
 * per joint in the kinematic chain.
 *
 * @note All joint space quantities are specified in radians or radians per unit time.
 */
struct KinematicsBoundary {
 public:
  // Setter methods
  /**
   * @brief Set the name identifier for this kinematic chain
   * @param name Chain name identifier, e.g., "left_arm", "right_arm", "mobile_base"
   */
  void set_chain_name(const std::string& name) { chain_name = name; }

  /**
   * @brief Set joint position lower bounds
   * @param limits Vector of lower position limits for each joint (units: rad)
   * @note Vector size must equal the number of joints in the chain
   */
  void set_lower_limit(const std::vector<double>& limits) { lower_limit = limits; }

  /**
   * @brief Set joint position upper bounds
   * @param limits Vector of upper position limits for each joint (units: rad)
   * @note Vector size must equal the number of joints in the chain
   */
  void set_upper_limit(const std::vector<double>& limits) { upper_limit = limits; }

  /**
   * @brief Set joint velocity lower bounds
   * @param limits Vector of lower velocity limits for each joint (units: rad/s)
   * @note Typically negative values for bidirectional joints
   */
  void set_vel_lower_limit(const std::vector<double>& limits) { vel_lower_limit = limits; }

  /**
   * @brief Set joint velocity upper bounds
   * @param limits Vector of upper velocity limits for each joint (units: rad/s)
   * @note Typically positive values for bidirectional joints
   */
  void set_vel_upper_limit(const std::vector<double>& limits) { vel_upper_limit = limits; }

  /**
   * @brief Set joint acceleration lower bounds
   * @param limits Vector of lower acceleration limits for each joint (units: rad/s²)
   * @note Used for trajectory optimization and smoothness constraints
   */
  void set_acc_lower_limit(const std::vector<double>& limits) { acc_lower_limit = limits; }

  /**
   * @brief Set joint acceleration upper bounds
   * @param limits Vector of upper acceleration limits for each joint (units: rad/s²)
   * @note Used for trajectory optimization and smoothness constraints
   */
  void set_acc_upper_limit(const std::vector<double>& limits) { acc_upper_limit = limits; }

  /**
   * @brief Set joint jerk lower bounds
   * @param limits Vector of lower jerk limits for each joint (units: rad/s³)
   * @note Jerk constraints improve motion smoothness and reduce mechanical wear
   */
  void set_jerk_lower_limit(const std::vector<double>& limits) { jerk_lower_limit = limits; }

  /**
   * @brief Set joint jerk upper bounds
   * @param limits Vector of upper jerk limits for each joint (units: rad/s³)
   * @note Jerk constraints improve motion smoothness and reduce mechanical wear
   */
  void set_jerk_upper_limit(const std::vector<double>& limits) { jerk_upper_limit = limits; }

  // Getter methods
  /**
   * @brief Get the kinematic chain name identifier
   * @return Const reference to chain name string
   */
  const std::string& get_chain_name() const { return chain_name; }

  /**
   * @brief Get joint position lower bounds
   * @return Const reference to vector of lower position limits (units: rad)
   */
  const std::vector<double>& get_lower_limit() const { return lower_limit; }

  /**
   * @brief Get joint position upper bounds
   * @return Const reference to vector of upper position limits (units: rad)
   */
  const std::vector<double>& get_upper_limit() const { return upper_limit; }

  /**
   * @brief Get joint velocity lower bounds
   * @return Const reference to vector of lower velocity limits (units: rad/s)
   */
  const std::vector<double>& get_vel_lower_limit() const { return vel_lower_limit; }

  /**
   * @brief Get joint velocity upper bounds
   * @return Const reference to vector of upper velocity limits (units: rad/s)
   */
  const std::vector<double>& get_vel_upper_limit() const { return vel_upper_limit; }

  /**
   * @brief Get joint acceleration lower bounds
   * @return Const reference to vector of lower acceleration limits (units: rad/s²)
   */
  const std::vector<double>& get_acc_lower_limit() const { return acc_lower_limit; }

  /**
   * @brief Get joint acceleration upper bounds
   * @return Const reference to vector of upper acceleration limits (units: rad/s²)
   */
  const std::vector<double>& get_acc_upper_limit() const { return acc_upper_limit; }

  /**
   * @brief Get joint jerk lower bounds
   * @return Const reference to vector of lower jerk limits (units: rad/s³)
   */
  const std::vector<double>& get_jerk_lower_limit() const { return jerk_lower_limit; }

  /**
   * @brief Get joint jerk upper bounds
   * @return Const reference to vector of upper jerk limits (units: rad/s³)
   */
  const std::vector<double>& get_jerk_upper_limit() const { return jerk_upper_limit; }

  /**
   * @brief Print kinematic boundary information to standard output
   *
   * Outputs all boundary parameters for debugging and visualization purposes.
   */
  void print() const;

 private:
  std::string chain_name = "";           /**< Kinematic chain name identifier */
  std::vector<double> lower_limit;       /**< Joint position lower bounds (rad) */
  std::vector<double> upper_limit;       /**< Joint position upper bounds (rad) */
  std::vector<double> vel_lower_limit;   /**< Joint velocity lower bounds (rad/s) */
  std::vector<double> vel_upper_limit;   /**< Joint velocity upper bounds (rad/s) */
  std::vector<double> acc_lower_limit;   /**< Joint acceleration lower bounds (rad/s²) */
  std::vector<double> acc_upper_limit;   /**< Joint acceleration upper bounds (rad/s²) */
  std::vector<double> jerk_lower_limit;  /**< Joint jerk lower bounds (rad/s³) */
  std::vector<double> jerk_upper_limit;  /**< Joint jerk upper bounds (rad/s³) */
};

/**
 * @struct SamplerConfig
 * @brief Configuration parameters for sampling-based motion planners
 *
 * This structure configures sampling-based planning algorithms (e.g., RRT, RRT*).
 * It controls state space sampling resolution, interpolation settings, path
 * simplification, and planning termination conditions.
 *
 * Sampling-based planners explore the configuration space by randomly sampling
 * states and connecting them to build a motion plan graph.
 */
struct SamplerConfig {
  /**
   * @enum StateCheckType
   * @brief Distance metric for comparing states in configuration space
   */
  enum StateCheckType {
    EUCLIDEAN_DISTANCE,  /**< Cartesian Euclidean distance in joint space (treats joint angles as Cartesian coordinates) */
    RADIAN_DISTANCE      /**< Angular distance metric accounting for joint angle wraparound and weighting */
  };

  /**
   * @enum TerminationConditionType
   * @brief Planning termination criteria
   */
  enum TerminationConditionType {
    TIMEOUT = 0,                    /**< Terminate only when maximum planning time is exceeded */
    TIMEOUT_AND_EXACT_SOLUTION = 1  /**< Terminate when timeout occurs OR exact goal solution is found */
  };

 public:
  // Setter methods
  /**
   * @brief Set the distance metric for state comparison
   * @param type Distance metric type for evaluating state similarity
   */
  void set_state_check_type(StateCheckType type) { state_check_type_ = type; }

  /**
   * @brief Set state comparison resolution threshold
   * @param resolution Minimum distance threshold to consider two states as distinct (units: rad or dimensionless)
   * @note Lower values increase planning precision but may slow down computation
   */
  void set_state_check_resolution(double resolution) { state_check_resolution_ = resolution; }

  /**
   * @brief Enable or disable path interpolation between sampled states
   * @param enable true to enable interpolation, false to use only sampled waypoints
   * @note Interpolation improves trajectory smoothness and collision checking accuracy
   */
  void set_interpolate(bool enable) { interpolate_ = enable; }

  /**
   * @brief Set the number of interpolation waypoints between consecutive samples
   * @param cnt Number of intermediate waypoints to insert (0 = use automatic calculation)
   * @note Higher counts improve collision detection but increase computational cost
   */
  void set_interpolation_cnt(int cnt) { interpolation_cnt_ = cnt; }

  /**
   * @brief Enable or disable path simplification post-processing
   * @param enable true to enable path shortcutting and smoothing, false to use raw planned path
   * @note Simplification reduces waypoints and improves trajectory efficiency
   */
  void set_simplify(bool enable) { simplify_ = enable; }

  /**
   * @brief Set maximum time budget for path simplification
   * @param time Maximum simplification duration (units: s); negative values indicate no time limit
   * @note Longer simplification time may yield shorter, smoother paths
   */
  void set_max_simplification_time(double time) { max_simplification_time_ = time; }

  /**
   * @brief Set planning termination condition
   * @param type Termination criterion (timeout only vs. timeout or exact solution)
   */
  void set_termination_condition_type(TerminationConditionType type) { termination_condition_type_ = type; }

  /**
   * @brief Set maximum time budget for motion planning
   * @param time Maximum planning duration (units: s)
   * @note Planning may terminate earlier if exact solution is found (depends on termination condition)
   */
  void set_max_planning_time(double time) { max_planning_time_ = time; }

  // Getter methods
  /**
   * @brief Get the configured distance metric for state comparison
   * @return Current state check type
   */
  StateCheckType get_state_check_type() const { return state_check_type_; }

  /**
   * @brief Get state comparison resolution threshold
   * @return Resolution value (units: rad or dimensionless, depending on state check type)
   */
  double get_state_check_resolution() const { return state_check_resolution_; }

  /**
   * @brief Check if path interpolation is enabled
   * @return true if interpolation is enabled, false otherwise
   */
  bool get_interpolate() const { return interpolate_; }

  /**
   * @brief Get the number of interpolation waypoints
   * @return Number of intermediate waypoints between samples
   */
  int get_interpolation_cnt() const { return interpolation_cnt_; }

  /**
   * @brief Check if path simplification is enabled
   * @return true if path simplification is enabled, false otherwise
   */
  bool get_simplify() const { return simplify_; }

  /**
   * @brief Get maximum path simplification time budget
   * @return Maximum simplification time (units: s); negative values indicate no limit
   */
  double get_max_simplification_time() const { return max_simplification_time_; }

  /**
   * @brief Get planning termination condition
   * @return Current termination condition type
   */
  TerminationConditionType get_termination_condition_type() const { return termination_condition_type_; }

  /**
   * @brief Get maximum planning time budget
   * @return Maximum planning time (units: s)
   */
  double get_max_planning_time() const { return max_planning_time_; }

  /**
   * @brief Print sampler configuration to standard output
   *
   * Outputs all configuration parameters for debugging and verification.
   */
  void print() const;

 private:
  StateCheckType state_check_type_;                                           /**< Distance metric for state comparison */
  double state_check_resolution_ = 0.01;                                      /**< State similarity threshold (rad or dimensionless) */
  bool interpolate_ = true;                                                   /**< Enable path interpolation between samples */
  int interpolation_cnt_ = 0;                                                 /**< Number of interpolation waypoints (0 = auto) */
  bool simplify_ = true;                                                      /**< Enable path simplification post-processing */
  double max_simplification_time_ = -1.0;                                     /**< Max simplification time budget (s, <0 = no limit) */
  TerminationConditionType termination_condition_type_ = TIMEOUT_AND_EXACT_SOLUTION;  /**< Planning termination criterion */
  double max_planning_time_ = 2.0;                                            /**< Maximum planning time budget (s) */
};

/**
 * @struct TrajectoryPlanConfig
 * @brief Trajectory planning and parameterization configuration
 *
 * This structure configures trajectory generation parameters for converting
 * discrete motion plans into smooth, time-parameterized trajectories.
 * It supports both single-segment and multi-waypoint trajectory planning.
 *
 * Trajectory planning involves computing velocity and acceleration profiles
 * along a geometric path while respecting kinematic constraints.
 */
struct TrajectoryPlanConfig {
 public:
  // Setter methods
  /**
   * @brief Set minimum duration for any motion segment
   * @param time Minimum trajectory execution time (units: s)
   * @note Non-zero values prevent excessively fast motions; 0.0 allows maximum speed within kinematic limits
   */
  void set_min_move_time(double time) { min_move_time_ = time; }

  /**
   * @brief Set waypoint density for Cartesian linear motion interpolation
   * @param value Number of intermediate waypoints for linear path segments (dimensionless)
   * @note Higher values improve Cartesian path accuracy but increase computational cost
   */
  void set_move_line_intermediate_point(double value) { move_line_intermediate_point_ = value; }

  /**
   * @brief Set expected total duration for multi-waypoint trajectory planning
   * @param time Expected trajectory execution time for waypoint sequences (units: s)
   * @note Used as a hint for time-optimal trajectory generation algorithms
   */
  void set_way_point_plan_expected_time(double time) { way_point_plan_expected_time_ = time; }

  // Getter methods
  /**
   * @brief Get minimum motion segment duration
   * @return Minimum movement time (units: s)
   */
  double get_min_move_time() const { return min_move_time_; }

  /**
   * @brief Get waypoint density for linear motion interpolation
   * @return Number of intermediate waypoints (dimensionless)
   */
  double get_move_line_intermediate_point() const { return move_line_intermediate_point_; }

  /**
   * @brief Get expected multi-waypoint trajectory duration
   * @return Expected planning time (units: s)
   */
  double get_way_point_plan_expected_time() const { return way_point_plan_expected_time_; }

  /**
   * @brief Print trajectory planning configuration to standard output
   *
   * Outputs all configuration parameters for debugging and verification.
   */
  void print() const;

 private:
  double min_move_time_ = 0.0;                  /**< Minimum trajectory execution time (s) */
  double move_line_intermediate_point_ = 50.0;  /**< Intermediate waypoint count for linear segments (dimensionless) */
  double way_point_plan_expected_time_ = 0.5;   /**< Expected total duration for multi-waypoint trajectories (s) */
};

/**
 * @struct IKSolverConfig
 * @brief Inverse kinematics (IK) solver configuration parameters
 *
 * This structure configures the numerical inverse kinematics solver used to
 * compute joint configurations that achieve desired end-effector poses.
 * It supports collision-aware IK with configurable seed strategies, convergence
 * tolerances, joint limit handling, and timeout parameters.
 *
 * IK solving is an iterative numerical optimization process that may benefit
 * from multiple random initializations to find feasible collision-free solutions.
 */
struct IKSolverConfig {
  /**
   * @enum SeedType
   * @brief Initial guess generation strategy for IK optimization
   */
  enum SeedType {
    RANDOM_SEED,             /**< Uniformly random joint configurations within limits */
    RANDOM_PROGRESSIVE_SEED, /**< Progressive random sampling with increasing coverage */
    USER_DEFINED_SEED,       /**< User-provided initial joint configurations */
  };

 public:
  // Setter methods
  /**
   * @brief Set timeout for collision-aware IK solver
   * @param timeout Maximum solver iteration time (units: ms)
   * @note Longer timeouts allow more seed attempts but delay planning
   */
  void set_col_aware_ik_timeout(double timeout) { col_aware_ik_timeout_ = timeout; }

  /**
   * @brief Set initial configuration seed generation strategy
   * @param type Seed generation method for IK optimization initialization
   */
  void set_seed_type(SeedType type) { seed_type_ = type; }

  /**
   * @brief Set safety margin from joint position limits
   * @param bias Distance from joint limits to treat as forbidden region (units: rad)
   * @note Prevents IK solver from proposing configurations near singularities or mechanical limits
   */
  void set_col_aware_ik_joint_limit_bias(double bias) { col_aware_ik_joint_limit_bias_ = bias; }

  /**
   * @brief Set Cartesian position error tolerance for IK convergence
   * @param eps Per-axis position error tolerance {x, y, z} (units: m)
   * @note IK solution is accepted when position error is within this threshold
   */
  void set_translation_eps(const std::array<double, 3>& eps) { translation_eps_ = eps; }

  /**
   * @brief Set orientation error tolerance for IK convergence
   * @param eps Per-axis orientation error tolerance {roll, pitch, yaw} (units: rad)
   * @note IK solution is accepted when orientation error is within this threshold
   */
  void set_rotation_eps(const std::array<double, 3>& eps) { rotation_eps_ = eps; }

  /**
   * @brief Enable or disable detailed collision checking diagnostic logs
   * @param enable true to output collision detection logs, false to suppress
   * @note Useful for debugging IK failures due to collision constraints
   */
  void set_enable_collision_check_log(bool enable) { enable_collision_check_log_ = enable; }

  // Getter methods
  /**
   * @brief Get collision-aware IK solver timeout
   * @return Timeout duration (units: ms)
   */
  double get_col_aware_ik_timeout() const { return col_aware_ik_timeout_; }

  /**
   * @brief Get IK solver seed generation strategy
   * @return Current seed type
   */
  SeedType get_seed_type() const { return seed_type_; }

  /**
   * @brief Get joint limit safety margin
   * @return Joint limit bias distance (units: rad)
   */
  double get_col_aware_ik_joint_limit_bias() const { return col_aware_ik_joint_limit_bias_; }

  /**
   * @brief Get Cartesian position error tolerance
   * @return Per-axis position tolerance {x, y, z} (units: m)
   */
  const std::array<double, 3>& get_translation_eps() const { return translation_eps_; }

  /**
   * @brief Get orientation error tolerance
   * @return Per-axis orientation tolerance {roll, pitch, yaw} (units: rad)
   */
  const std::array<double, 3>& get_rotation_eps() const { return rotation_eps_; }

  /**
   * @brief Check if collision check logging is enabled
   * @return true if logging is enabled, false otherwise
   */
  bool get_enable_collision_check_log() const { return enable_collision_check_log_; }

  /**
   * @brief Print IK solver configuration to standard output
   *
   * Outputs all configuration parameters for debugging and verification.
   */
  void print() const;

 private:
  double col_aware_ik_timeout_ = 10.0;                   /**< Collision-aware IK solver timeout (ms) */
  SeedType seed_type_ = RANDOM_PROGRESSIVE_SEED;         /**< IK initial guess generation strategy */
  double col_aware_ik_joint_limit_bias_ = 0.001;         /**< Safety margin from joint limits (rad) */
  std::array<double, 3> translation_eps_ = {0.0, 0.0, 0.0};  /**< Position error tolerance (m) */
  std::array<double, 3> rotation_eps_ = {0.0, 0.0, 0.0};     /**< Orientation error tolerance (rad) */
  bool enable_collision_check_log_ = false;              /**< Enable collision detection diagnostic logging */
};

/**
 * @struct CollisionCheckOption
 * @brief Collision detection enable/disable configuration
 *
 * This structure provides fine-grained control over collision checking during
 * motion planning and execution. It supports independent toggling of self-collision
 * detection (robot links colliding with each other) and environment collision
 * detection (robot colliding with obstacles or workspace boundaries).
 *
 * @note Disabling collision checks improves computational performance but may
 *       result in unsafe trajectories. Use with caution in controlled environments.
 */
struct CollisionCheckOption {
 public:
  // Setter methods
  /**
   * @brief Enable or disable self-collision detection
   * @param disable true to disable self-collision checking, false to enable
   * @warning Disabling self-collision checks may result in physically infeasible configurations
   */
  void set_disable_self_collision_check(bool disable) { disable_self_collision_check_ = disable; }

  /**
   * @brief Enable or disable environment collision detection
   * @param disable true to disable environment collision checking, false to enable
   * @warning Disabling environment checks may result in collisions with obstacles
   */
  void set_disable_env_collision_check(bool disable) { disable_env_collision_check_ = disable; }

  // Getter methods
  /**
   * @brief Check if self-collision detection is disabled
   * @return true if self-collision detection is currently disabled, false if enabled
   */
  bool get_disable_self_collision_check() const { return disable_self_collision_check_; }

  /**
   * @brief Check if environment collision detection is disabled
   * @return true if environment collision detection is currently disabled, false if enabled
   */
  bool get_disable_env_collision_check() const { return disable_env_collision_check_; }

  /**
   * @brief Print collision detection configuration to standard output
   *
   * Outputs enabled/disabled status for each collision check type.
   */
  void print() const;

 private:
  bool disable_self_collision_check_ = false;  /**< Disable robot self-collision detection if true */
  bool disable_env_collision_check_ = false;   /**< Disable robot-environment collision detection if true */
};

/**
 * @struct TrajectoryFeasibilityCheckOption
 * @brief Trajectory validation and feasibility checking configuration
 *
 * This structure provides fine-grained control over which feasibility constraints
 * are enforced during trajectory validation. It supports independent toggling of
 * collision detection, joint limit compliance, and velocity profile feasibility.
 *
 * Selectively disabling checks can improve computational performance for debugging,
 * simulation, or scenarios where certain constraints are guaranteed to be satisfied.
 *
 * @note Disabling feasibility checks may produce trajectories that are unsafe or
 *       physically unrealizable. Use with caution and only when constraints are
 *       verified through other means.
 */
struct TrajectoryFeasibilityCheckOption {
 public:
  // Setter methods
  /**
   * @brief Enable or disable collision detection during trajectory validation
   * @param disable true to skip collision checking, false to enforce collision-free trajectories
   * @warning Disabling collision checks may result in unsafe motion plans
   */
  void set_disable_collision_check(bool disable) { disable_collision_check_ = disable; }

  /**
   * @brief Enable or disable joint limit compliance checking
   * @param disable true to skip joint limit validation, false to enforce position limits
   * @warning Disabling joint limit checks may damage hardware or violate safety constraints
   */
  void set_disable_joint_limit_check(bool disable) { disable_joint_limit_check_ = disable; }

  /**
   * @brief Enable or disable velocity profile feasibility checking
   * @param disable true to skip velocity limit validation, false to enforce velocity constraints
   * @note Velocity feasibility ensures the trajectory can be executed within actuator speed limits
   */
  void set_disable_velocity_feasibility_check(bool disable) {
    disable_velocity_feasibility_check_ = disable;
  }

  // Getter methods
  /**
   * @brief Check if collision detection is disabled
   * @return true if collision checking is currently disabled, false if enabled
   */
  bool get_disable_collision_check() const { return disable_collision_check_; }

  /**
   * @brief Check if joint limit checking is disabled
   * @return true if joint limit validation is currently disabled, false if enabled
   */
  bool get_disable_joint_limit_check() const { return disable_joint_limit_check_; }

  /**
   * @brief Check if velocity feasibility checking is disabled
   * @return true if velocity validation is currently disabled, false if enabled
   */
  bool get_disable_velocity_feasibility_check() const { return disable_velocity_feasibility_check_; }

  /**
   * @brief Print trajectory feasibility check configuration to standard output
   *
   * Outputs enabled/disabled status for each feasibility check type.
   */
  void print() const;

 private:
  bool disable_collision_check_ = false;           /**< Disable collision detection if true */
  bool disable_joint_limit_check_ = false;         /**< Disable joint limit checking if true */
  bool disable_velocity_feasibility_check_ = false;/**< Disable velocity feasibility checking if true */
};


/**
 * @struct LineTrajCheckPrimitive
 * @brief Geometric primitive configuration for Cartesian linear trajectory validation
 *
 * This structure configures the collision detection geometric representation for
 * linear end-effector trajectories in Cartesian space. It supports two primitive
 * types: infinitesimally thin lines and swept-volume cylinders.
 *
 * Choosing the appropriate primitive affects collision detection conservativeness
 * and computational cost. Cylinder primitives model the robot's actual swept volume
 * more accurately but require more expensive geometric queries.
 */
struct LineTrajCheckPrimitive {
  /**
   * @enum PrimitiveType
   * @brief Geometric representation for linear trajectory collision checking
   */
  enum PrimitiveType {
    LINE,      /**< Zero-thickness line segment (fast but less conservative) */
    CYLINDER,  /**< Swept-volume cylinder with configurable radius (accurate but slower) */
  };

 public:
  // Setter methods
  /**
   * @brief Set geometric primitive type for linear trajectory validation
   * @param type Primitive representation (LINE or CYLINDER)
   * @note CYLINDER is recommended for safety-critical applications
   */
  void set_line_check_primitive_type(PrimitiveType type) {
    line_check_primitive_type_ = type;
  }

  /**
   * @brief Set swept-volume cylinder radius for trajectory collision checking
   * @param radius Cylinder radius representing robot swept volume (units: m)
   * @note Larger radii increase safety margins but may be overly conservative
   * @note Only applies when primitive type is CYLINDER
   */
  void set_cylinder_prim_radius(double radius) {
    cylinder_prim_radius_ = radius;
  }

  /**
   * @brief Set curvature approximation tolerance for line primitive
   * @param curvature Maximum deviation tolerance for piecewise-linear approximation (units: m)
   * @note Controls how finely curved paths are discretized into line segments
   * @note Lower values improve accuracy but increase computational cost
   */
  void set_line_prim_curvature(double curvature) {
    line_prim_curvature_ = curvature;
  }

  // Getter methods
  /**
   * @brief Get the geometric primitive type for trajectory checking
   * @return Current primitive type (LINE or CYLINDER)
   */
  PrimitiveType get_line_check_primitive_type() const {
    return line_check_primitive_type_;
  }

  /**
   * @brief Get the cylinder primitive swept-volume radius
   * @return Cylinder radius (units: m)
   */
  double get_cylinder_prim_radius() const {
    return cylinder_prim_radius_;
  }

  /**
   * @brief Get the line primitive curvature approximation tolerance
   * @return Curvature tolerance (units: m)
   */
  double get_line_prim_curvature() const {
    return line_prim_curvature_;
  }

  /**
   * @brief Print line trajectory check primitive configuration to standard output
   *
   * Outputs selected primitive type and associated parameters for debugging.
   */
  void print() const;

 private:
  PrimitiveType line_check_primitive_type_ = CYLINDER;  /**< Trajectory collision check primitive type */
  double cylinder_prim_radius_ = 0.02;                  /**< Swept-volume cylinder radius (m) */
  double line_prim_curvature_ = 0.02;                   /**< Piecewise-linear curvature approximation tolerance (m) */
};

/**
 * @class MotionPlanConfig
 * @brief Comprehensive motion planning configuration management
 *
 * MotionPlanConfig serves as a centralized configuration container for all
 * motion planning subsystems. It aggregates sampling strategies, trajectory
 * generation parameters, inverse kinematics solver settings, collision detection
 * options, feasibility validation criteria, and kinematic constraint boundaries.
 *
 * This class provides a unified interface for configuring complex motion planning
 * pipelines, supporting both simple manipulator planning and whole-body humanoid
 * motion generation with multiple kinematic chains.
 *
 * Configuration objects are lazily initialized and managed through shared pointers
 * to optimize memory usage and support optional feature configuration.
 */
class MotionPlanConfig {
 public:
  /**
   * @brief Default constructor
   *
   * Initializes an empty configuration with all sub-configurations set to nullptr.
   * Sub-configurations are created on-demand via create_* or get_*_ref methods.
   */
  MotionPlanConfig() = default;

  /**
   * @brief Set configuration update timestamp
   * @param t Timestamp of last configuration modification (units: ns, typically CLOCK_MONOTONIC)
   * @note Used for configuration versioning and cache invalidation
   */
  void set_update_time(int64_t t) { update_nsec_ = t; }

  /**
   * @brief Get configuration update timestamp
   * @return Timestamp of last configuration modification (units: ns)
   */
  int64_t get_update_time() { return update_nsec_; }

  // Factory methods to create and get configuration sub-objects
  /**
   * @brief Create or retrieve sampler configuration
   *
   * Lazily initializes the sampler configuration if it does not exist.
   * Safe to call multiple times; returns the same instance after first creation.
   *
   * @return Shared pointer to sampler configuration with default settings
   */
  std::shared_ptr<SamplerConfig> create_sampler_config() {
    if (!sampler_config_)
      sampler_config_ = std::make_shared<SamplerConfig>();
    return sampler_config_;
  }

  /**
   * @brief Create or retrieve trajectory planning configuration
   *
   * Lazily initializes the trajectory planning configuration if it does not exist.
   *
   * @return Shared pointer to trajectory planning configuration with default settings
   */
  std::shared_ptr<TrajectoryPlanConfig> create_trajectory_plan_config() {
    if (!traj_plan_config_)
      traj_plan_config_ = std::make_shared<TrajectoryPlanConfig>();
    return traj_plan_config_;
  }

  /**
   * @brief Create or retrieve inverse kinematics solver configuration
   *
   * Lazily initializes the IK solver configuration if it does not exist.
   *
   * @return Shared pointer to IK solver configuration with default settings
   */
  std::shared_ptr<IKSolverConfig> create_ik_solver_config() {
    if (!ik_solver_config_)
      ik_solver_config_ = std::make_shared<IKSolverConfig>();
    return ik_solver_config_;
  }

  /**
   * @brief Create or retrieve collision check option configuration
   *
   * Lazily initializes the collision check options if they do not exist.
   *
   * @return Shared pointer to collision check options with default settings
   */
  std::shared_ptr<CollisionCheckOption> create_collision_check_option() {
    if (!collision_check_option_)
      collision_check_option_ = std::make_shared<CollisionCheckOption>();
    return collision_check_option_;
  }

  /**
   * @brief Create or retrieve trajectory feasibility check option configuration
   *
   * Lazily initializes the trajectory feasibility check options if they do not exist.
   *
   * @return Shared pointer to trajectory feasibility check options with default settings
   */
  std::shared_ptr<TrajectoryFeasibilityCheckOption> create_trajectory_feasibility_check_option() {
    if (!traj_feasibility_check_option_)
      traj_feasibility_check_option_ = std::make_shared<TrajectoryFeasibilityCheckOption>();
    return traj_feasibility_check_option_;
  }

  /**
   * @brief Create or retrieve line trajectory check primitive configuration
   *
   * Lazily initializes the line trajectory check primitive configuration if it does not exist.
   *
   * @return Shared pointer to line trajectory check primitive with default settings
   */
  std::shared_ptr<LineTrajCheckPrimitive> create_line_traj_check_primitive() {
    if (!line_traj_check_primitive_)
      line_traj_check_primitive_ = std::make_shared<LineTrajCheckPrimitive>();
    return line_traj_check_primitive_;
  }

  // Configuration setter methods
  /**
   * @brief Set or replace sampler configuration
   * @param config Shared pointer to sampler configuration; nullptr clears the configuration
   */
  void set_sampler_config(const std::shared_ptr<SamplerConfig>& config) { sampler_config_ = config; }

  /**
   * @brief Set or replace trajectory planning configuration
   * @param config Shared pointer to trajectory planning configuration; nullptr clears the configuration
   */
  void set_trajectory_plan_config(const std::shared_ptr<TrajectoryPlanConfig>& config) {
    traj_plan_config_ = config;
  }

  /**
   * @brief Set or replace inverse kinematics solver configuration
   * @param config Shared pointer to IK solver configuration; nullptr clears the configuration
   */
  void set_ik_solver_config(const std::shared_ptr<IKSolverConfig>& config) { ik_solver_config_ = config; }

  /**
   * @brief Set or replace collision check option configuration
   * @param option Shared pointer to collision check options; nullptr clears the configuration
   */
  void set_collision_check_option(const std::shared_ptr<CollisionCheckOption>& option) {
    collision_check_option_ = option;
  }

  /**
   * @brief Set or replace trajectory feasibility check option configuration
   * @param option Shared pointer to trajectory feasibility check options; nullptr clears the configuration
   */
  void set_trajectory_feasibility_check_option(
      const std::shared_ptr<TrajectoryFeasibilityCheckOption>& option) {
    traj_feasibility_check_option_ = option;
  }

  /**
   * @brief Set kinematic feasibility boundaries for all chains
   * @param boundary Vector of kinematic boundaries, one per chain
   * @note These boundaries are used for general trajectory feasibility validation
   */
  void set_feasibility_boundary(const std::vector<KinematicsBoundary>& boundary) {
    feasibility_boundary_ = boundary;
  }

  /**
   * @brief Set or replace line trajectory check primitive configuration
   * @param primitive Shared pointer to line trajectory check primitive; nullptr clears the configuration
   */
  void set_line_traj_check_primitive(const std::shared_ptr<LineTrajCheckPrimitive>& primitive) {
    line_traj_check_primitive_ = primitive;
  }

  /**
   * @brief Set joint limits used during IK solving phase
   * @param boundary Vector of kinematic boundaries for IK solver joint constraints
   * @note IK limits may be tighter than hard limits to improve convergence and avoid singularities
   */
  void set_ik_joint_limit(const std::vector<KinematicsBoundary>& boundary) {
    ik_joint_limit_ = boundary;
  }

  /**
   * @brief Set joint limits used during sampling-based planning phase
   * @param boundary Vector of kinematic boundaries for sampling algorithms
   * @note Sampling limits define the valid configuration space for exploration
   */
  void set_sampler_joint_limit(const std::vector<KinematicsBoundary>& boundary) {
    sampler_joint_limit_ = boundary;
  }

  /**
   * @brief Set absolute hard joint limits (safety-critical boundaries)
   * @param boundary Vector of kinematic boundaries representing mechanical/safety limits
   * @note Hard limits must never be violated; typically correspond to physical joint stops
   */
  void set_hard_joint_limit(const std::vector<KinematicsBoundary>& boundary) {
    hard_joint_limit_ = boundary;
  }

  /**
   * @brief Enable or disable IK joint limit reversion to hard limits
   * @param flag true to revert IK joint limits to hard limits, false to use configured IK limits
   * @note Useful for recovering from constrained configurations by temporarily relaxing IK limits
   */
  void set_revert_ik_joint_limit(bool flag) { revert_ik_joint_limit_ = flag; }

  /**
   * @brief Set specific kinematic chains for IK joint limit reversion
   * @param chains Vector of chain names to apply IK limit reversion (e.g., {"left_arm", "torso"})
   * @note If non-empty, automatically enables revert_ik_joint_limit flag
   * @note Empty vector disables selective reversion (applies to all chains if flag is set)
   */
  void set_revert_ik_joint_limit_chains(const std::vector<std::string>& chains) {
    revert_ik_joint_limit_ = !chains.empty();
    revert_ik_joint_limit_chains_ = chains;
  }

  // =======================
  // Getter methods (returning shared pointers, may be nullptr)
  // =======================

  /**
   * @brief Get sampler configuration (may be nullptr if not initialized)
   * @return Shared pointer to sampler configuration, or nullptr if not set
   * @note Use create_sampler_config() to ensure a valid configuration exists
   */
  std::shared_ptr<SamplerConfig> get_sampler_config() const { return sampler_config_; }

  /**
   * @brief Get trajectory planning configuration (may be nullptr if not initialized)
   * @return Shared pointer to trajectory planning configuration, or nullptr if not set
   * @note Use create_trajectory_plan_config() to ensure a valid configuration exists
   */
  std::shared_ptr<TrajectoryPlanConfig> get_trajectory_plan_config() const { return traj_plan_config_; }

  /**
   * @brief Get inverse kinematics solver configuration (may be nullptr if not initialized)
   * @return Shared pointer to IK solver configuration, or nullptr if not set
   * @note Use create_ik_solver_config() to ensure a valid configuration exists
   */
  std::shared_ptr<IKSolverConfig> get_ik_solver_config() const { return ik_solver_config_; }

  /**
   * @brief Get collision check option configuration (may be nullptr if not initialized)
   * @return Shared pointer to collision check options, or nullptr if not set
   * @note Use create_collision_check_option() to ensure a valid configuration exists
   */
  std::shared_ptr<CollisionCheckOption> get_collision_check_option() const { return collision_check_option_; }

  /**
   * @brief Get trajectory feasibility check option configuration (may be nullptr if not initialized)
   * @return Shared pointer to trajectory feasibility check options, or nullptr if not set
   * @note Use create_trajectory_feasibility_check_option() to ensure a valid configuration exists
   */
  std::shared_ptr<TrajectoryFeasibilityCheckOption> get_trajectory_feasibility_check_option() const {
      return traj_feasibility_check_option_;
  }

  /**
   * @brief Get line trajectory check primitive configuration (may be nullptr if not initialized)
   * @return Shared pointer to line trajectory check primitive, or nullptr if not set
   * @note Use create_line_traj_check_primitive() to ensure a valid configuration exists
   */
  std::shared_ptr<LineTrajCheckPrimitive> get_line_traj_check_primitive() const { return line_traj_check_primitive_; }


  // =======================
  // Getter methods (returning references, lazily creating object if null)
  // =======================

  /**
   * @brief Get mutable reference to sampler configuration
   *
   * Lazily creates a new sampler configuration with default values if not already initialized.
   * Useful for in-place modification of configuration parameters.
   *
   * @return Mutable reference to sampler configuration (guaranteed non-null)
   */
  SamplerConfig& get_sampler_config_ref() {
      if (!sampler_config_)
          sampler_config_ = std::make_shared<SamplerConfig>();
      return *sampler_config_;
  }

  /**
   * @brief Get mutable reference to trajectory planning configuration
   *
   * Lazily creates a new trajectory planning configuration with default values if not already initialized.
   *
   * @return Mutable reference to trajectory planning configuration (guaranteed non-null)
   */
  TrajectoryPlanConfig& get_trajectory_plan_config_ref() {
      if (!traj_plan_config_)
          traj_plan_config_ = std::make_shared<TrajectoryPlanConfig>();
      return *traj_plan_config_;
  }

  /**
   * @brief Get mutable reference to inverse kinematics solver configuration
   *
   * Lazily creates a new IK solver configuration with default values if not already initialized.
   *
   * @return Mutable reference to IK solver configuration (guaranteed non-null)
   */
  IKSolverConfig& get_ik_solver_config_ref() {
      if (!ik_solver_config_)
          ik_solver_config_ = std::make_shared<IKSolverConfig>();
      return *ik_solver_config_;
  }

  /**
   * @brief Get mutable reference to collision check option configuration
   *
   * Lazily creates a new collision check option with default values if not already initialized.
   *
   * @return Mutable reference to collision check options (guaranteed non-null)
   */
  CollisionCheckOption& get_collision_check_option_ref() {
      if (!collision_check_option_)
          collision_check_option_ = std::make_shared<CollisionCheckOption>();
      return *collision_check_option_;
  }

  /**
   * @brief Get mutable reference to trajectory feasibility check option configuration
   *
   * Lazily creates a new trajectory feasibility check option with default values if not already initialized.
   *
   * @return Mutable reference to trajectory feasibility check options (guaranteed non-null)
   */
  TrajectoryFeasibilityCheckOption& get_trajectory_feasibility_check_option_ref() {
      if (!traj_feasibility_check_option_)
          traj_feasibility_check_option_ = std::make_shared<TrajectoryFeasibilityCheckOption>();
      return *traj_feasibility_check_option_;
  }

  /**
   * @brief Get mutable reference to line trajectory check primitive configuration
   *
   * Lazily creates a new line trajectory check primitive with default values if not already initialized.
   *
   * @return Mutable reference to line trajectory check primitive (guaranteed non-null)
   */
  LineTrajCheckPrimitive& get_line_traj_check_primitive_ref() {
      if (!line_traj_check_primitive_)
          line_traj_check_primitive_ = std::make_shared<LineTrajCheckPrimitive>();
      return *line_traj_check_primitive_;
  }

  // =======================
  // Getter methods (kinematic boundaries and configuration flags)
  // =======================

  /**
   * @brief Get kinematic feasibility boundaries for all chains (read-only)
   *
   * Returns immutable access to the feasibility boundary configuration.
   *
   * @return Const reference to vector of kinematic feasibility boundaries
   */
  const std::vector<KinematicsBoundary>& get_feasibility_boundary() const { return feasibility_boundary_; }

  /**
   * @brief Get kinematic feasibility boundaries for all chains (mutable)
   *
   * Returns mutable access to the feasibility boundary configuration for in-place modification.
   *
   * @return Mutable reference to vector of kinematic feasibility boundaries
   */
  std::vector<KinematicsBoundary>& get_feasibility_boundary() { return feasibility_boundary_; }

  /**
   * @brief Get IK phase joint limits (read-only)
   * @return Const reference to vector of IK joint limit boundaries
   */
  const std::vector<KinematicsBoundary>& get_ik_joint_limit() const { return ik_joint_limit_; }

  /**
   * @brief Get IK phase joint limits (mutable)
   * @return Mutable reference to vector of IK joint limit boundaries
   */
  std::vector<KinematicsBoundary>& get_ik_joint_limit() { return ik_joint_limit_; }

  /**
   * @brief Get sampling phase joint limits (read-only)
   * @return Const reference to vector of sampling joint limit boundaries
   */
  const std::vector<KinematicsBoundary>& get_sampler_joint_limit() const { return sampler_joint_limit_; }

  /**
   * @brief Get sampling phase joint limits (mutable)
   * @return Mutable reference to vector of sampling joint limit boundaries
   */
  std::vector<KinematicsBoundary>& get_sampler_joint_limit() { return sampler_joint_limit_; }

  /**
   * @brief Get hard joint limits (read-only)
   * @return Const reference to vector of hard joint limit boundaries
   * @note Hard limits represent absolute mechanical/safety constraints
   */
  const std::vector<KinematicsBoundary>& get_hard_joint_limit() const { return hard_joint_limit_; }

  /**
   * @brief Get hard joint limits (mutable)
   * @return Mutable reference to vector of hard joint limit boundaries
   */
  std::vector<KinematicsBoundary>& get_hard_joint_limit() { return hard_joint_limit_; }

  /**
   * @brief Get kinematic chains for selective IK joint limit reversion (read-only)
   * @return Const reference to vector of chain names for IK limit reversion
   * @note Empty vector means reversion applies to all chains (if reversion is enabled)
   */
  const std::vector<std::string>& get_revert_ik_joint_limit_chains() const { return revert_ik_joint_limit_chains_; }

  /**
   * @brief Get kinematic chains for selective IK joint limit reversion (mutable)
   * @return Mutable reference to vector of chain names for IK limit reversion
   */
  std::vector<std::string>& get_revert_ik_joint_limit_chains() { return revert_ik_joint_limit_chains_; }

  /**
   * @brief Check if IK joint limit reversion is enabled
   * @return true if IK limits should revert to hard limits, false otherwise
   */
  bool get_revert_ik_joint_limit() { return revert_ik_joint_limit_; }

  /**
   * @brief Print comprehensive motion planning configuration to standard output
   *
   * Outputs all sub-configuration parameters and kinematic boundaries in human-readable format.
   * Useful for debugging, logging, and verification of configuration state.
   */
  void print() const;

 private:
  int64_t update_nsec_ = 0;  /**< Configuration last update timestamp (ns, typically CLOCK_MONOTONIC) */

  std::shared_ptr<SamplerConfig> sampler_config_ = nullptr;  /**< Sampling-based planner configuration */
  std::shared_ptr<TrajectoryPlanConfig> traj_plan_config_ = nullptr;  /**< Trajectory generation configuration */
  std::shared_ptr<IKSolverConfig> ik_solver_config_ = nullptr;  /**< Inverse kinematics solver configuration */
  std::shared_ptr<CollisionCheckOption> collision_check_option_ = nullptr;  /**< Collision detection options */
  std::shared_ptr<TrajectoryFeasibilityCheckOption> traj_feasibility_check_option_ = nullptr;  /**< Trajectory validation options */
  std::vector<KinematicsBoundary> feasibility_boundary_;  /**< General kinematic feasibility constraints */
  std::shared_ptr<LineTrajCheckPrimitive> line_traj_check_primitive_ = nullptr;  /**< Linear trajectory collision primitive */

  std::vector<KinematicsBoundary> ik_joint_limit_;       /**< Joint constraints for IK solving phase */
  std::vector<KinematicsBoundary> sampler_joint_limit_;  /**< Joint constraints for sampling phase */
  std::vector<KinematicsBoundary> hard_joint_limit_;     /**< Absolute mechanical/safety joint limits */

  bool revert_ik_joint_limit_ = false;  /**< Enable IK joint limit reversion to hard limits */
  std::vector<std::string> revert_ik_joint_limit_chains_;  /**< Chains for selective IK limit reversion */
};

}  // namespace g1
}  // namespace sdk
}  // namespace galbot