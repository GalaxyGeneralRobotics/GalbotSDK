/**
 * @file galbot_navigation.hpp
 * @brief Navigation interface for mobile robot chassis motion planning and localization
 *
 * This file provides the navigation control interface for the Galbot G1 mobile base,
 * including:
 * - 2D pose estimation and relocalization
 * - Goal-directed navigation with obstacle avoidance
 * - Path planning and reachability checking
 * - Omnidirectional and differential drive motion planning
 *
 * @note All pose coordinates use the map frame unless explicitly stated otherwise.
 * @note Distances are in meters (m), angles in radians (rad), time in seconds (s).
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
#include "galbot_sdk_type.hpp"

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
 * @class GalbotNavigation
 * @brief Navigation interface for mobile robot chassis motion planning and localization.
 *
 * This class provides a thread-safe singleton interface for controlling the mobile base
 * navigation system. It supports 2D pose estimation, relocalization, goal-directed navigation
 * with dynamic obstacle avoidance, and path planning capabilities.
 *
 * The navigation system operates in a global map frame and provides both blocking and
 * non-blocking navigation modes. It supports both differential drive and omnidirectional
 * motion planning strategies.
 *
 * @note This class uses the singleton pattern with thread-safe initialization.
 * @note All pose coordinates are specified in the map frame unless explicitly stated otherwise.
 *
 * Typical usage:
 * @code
 *   auto& nav = GalbotNavigation::get_instance(MachineType::G1);
 *   if (nav.init()) {
 *     Pose goal;
 *     goal.x = 1.0;  // meters
 *     goal.y = 2.0;  // meters
 *     goal.orientation.w = 1.0;  // identity quaternion (x,y,z default 0)
 *     nav.navigate_to_goal(goal, true, false, 30.0, true);
 *   }
 * @endcode
 *
 */
class GalbotNavigation {
 public:
  virtual ~GalbotNavigation() = default;

  /**
   * @brief Runtime factory for selecting a concrete navigation singleton.
   *
   * This static factory method allows runtime selection of the navigation
   * implementation based on the robot machine type. The method declaration
   * resides in the interface header for compile-time availability, while the
   * actual implementation logic (including platform-specific includes and
   * switch statements) is contained in the corresponding .cpp file. This
   * design keeps the interface clean while enabling platform-specific
   * instantiation without exposing implementation details.
   *
   * @param m Machine type identifier specifying which robot platform to use.
   * @return Reference to the singleton navigation interface instance for the
   *         specified machine type.
   *
   * @note Adding support for a new machine type requires updating the
   *       MachineType enumeration and the factory implementation in the .cpp file.
   */
  static GalbotNavigation& get_instance(MachineType m);

  /**
   * @brief Initialize the navigation subsystem and its dependencies.
   *
   * This method must be called before using any other navigation functions.
   * It initializes communication channels, loads the map, starts the localization
   * module, and prepares the path planner.
   *
   * @return true if initialization succeeded, false otherwise.
   *
   * @note This method should only be called once after obtaining the singleton instance.
   * @note Subsequent calls will return the result of the first initialization attempt.
   *
   * @warning Calling navigation methods before successful initialization will result
   *          in undefined behavior.
   */
  virtual bool init() = 0;

  /**
   * @brief Perform relocalization to re-estimate the robot's pose in the map frame.
   *
   * This method resets the localization filter and provides an initial pose estimate
   * to help the robot re-establish its position in the known map. This is useful
   * when the robot has lost localization or when manually placing the robot at a
   * known position.
   *
   * @param init_pose Initial pose estimate in the map frame: position (x, y, z) in meters
   *                  and orientation as unit quaternion (x, y, z, w). Serves as the starting
   *                  point for the relocalization algorithm.
   *
   * @return NavigationStatus indicating the result of the relocalization request.
   *         See NavigationStatus enumeration for possible values.
   *
   * @note The robot should be stationary during relocalization for best results.
   * @note After calling this method, use is_localized() to verify successful
   *       relocalization before proceeding with navigation tasks.
   */
  virtual NavigationStatus relocalize(const Pose& init_pose) = 0;

  /**
   * @brief Check whether the robot is currently localized in the map.
   *
   * This method queries the localization system to determine if the robot has
   * a valid pose estimate with sufficient confidence. A robot that is not
   * localized should not perform navigation tasks.
   *
   * @return true if the robot is localized with acceptable confidence,
   *         false if localization is lost or uncertain.
   *
   * @note It is recommended to check localization status before issuing
   *       navigation commands.
   * @note If this returns false, consider calling relocalize() with a known
   *       pose estimate.
   */
  virtual bool is_localized() = 0;

  /**
   * @brief Get the current estimated pose of the robot chassis in the map frame.
   *
   * This method returns the most recent pose estimate from the localization system.
   * The pose represents the position and orientation of the robot's base_link frame
   * relative to the map frame origin.
   *
   * @return Pose: position (x, y, z) in meters and orientation as unit quaternion (x, y, z, w)
   *         in the map frame.
   *
   * @note The returned pose is only valid if is_localized() returns true.
   * @note The pose represents the center of the robot's base footprint.
   */
  virtual Pose get_current_pose() = 0;

  /**
   * @brief Navigate the robot to a target goal pose in the map frame.
   *
   * This method commands the mobile base to navigate to a specified goal pose using
   * the global path planner and local trajectory controller. The planner will compute
   * a collision-free path from the current pose to the goal, considering both static
   * map obstacles and dynamic obstacles if collision checking is enabled.
   *
   * @param goal_pose Target goal pose in the map frame: position (x, y, z) in meters and
   *                  target orientation as unit quaternion (x, y, z, w).
   * @param enable_collision_check If true, enables dynamic obstacle detection and
   *                               avoidance during navigation. If false, only static
   *                               map obstacles are considered. Default: true.
   * @param is_blocking Execution mode flag. Default: false.
   *                    - false (non-blocking): Returns immediately after sending the
   *                      navigation command and starts an SDK-side monitor thread. The
   *                      return status indicates command acceptance, not whether the
   *                      goal was reached.
   *                    - true (blocking): Blocks until the goal is reached, navigation
   *                      fails, or timeout occurs in the current thread. The return status
   *                      reflects the final navigation outcome.
   * @param timeout SDK-side navigation monitor timeout in seconds. Default: 8.0 seconds.
   *                In blocking mode, the current thread monitors this timeout. In
   *                non-blocking mode, a background monitor thread uses this timeout. If
   *                the goal is not reached before timeout and navigation has not already
   *                stopped, the SDK automatically calls stop_navigation.
   * @param omni_plan Motion planning mode flag. Default: false.
   *                  - true: Enables omnidirectional motion planning (holonomic drive),
   *                    allowing the robot to move in any direction and rotate independently.
   *                  - false: Uses differential drive planning with kinematic constraints.
   *                  S1 does not support omnidirectional motion planning. For S1, keep this
   *                  parameter false; setting it to true returns NavigationStatus::INVALID_INPUT.
   *
   * @return NavigationStatus indicating the result:
   *         - In non-blocking mode: Command acceptance status
   *         - In blocking mode: Final navigation outcome (success, failure, timeout)
   *
   * @note The robot must be localized (is_localized() returns true) before calling this method.
   * @note The navigation request communication timeout is fixed internally and is separate
   *       from the SDK-side navigation monitor timeout.
   * @note In both blocking and non-blocking modes, timeout is used by SDK-side monitoring.
   *
   * @warning In non-blocking mode, monitor navigation progress separately to detect
   *          completion or failures.
   */
  virtual NavigationStatus navigate_to_goal(const Pose& goal_pose, bool enable_collision_check = true,
                                            bool is_blocking = false, float timeout = 8, bool omni_plan = false) = 0;

  /**
   * @brief Submit a dynamic navigation target asynchronously.
   *
   * This API is designed for targets that may change over time, such as dynamic
   * tracking or remote control. When a new target is submitted, the previous
   * target may be preempted and the navigation system will re-plan toward the
   * latest target.
   *
   * To keep the navigation stable, do not call this API at a very high rate.
   * Frequent updates may cause planning jitter and reduce motion smoothness.
   *
   * @param target Target pose expressed as Pose (x, y, z, qx, qy, qz, qw).
   * @param frame Target frame identifier. Supported values: "map", "base_link".
   * @param speed_ratio Velocity scaling factor in (0, 1.0].
   * @param enable_collision_check Whether to enable collision checking and avoidance.
   *
   * @return TaskHandle containing the submitted task id, request result, and message.
   */
  virtual TaskHandle set_navigation_target(const Pose& target, const std::string& frame = "map",
                                           float speed_ratio = 1.0, bool enable_collision_check = true) = 0;

  /**
   * @brief Query the status of an asynchronous navigation task.
   *
   * Use this API to check the latest state of a task submitted by an asynchronous
   * navigation interface.
   *
   * @param task_id Task identifier returned by the corresponding navigation API.
   * @return NavigationTaskSnapshot containing the latest known state for the requested task.
   */
  virtual NavigationTaskSnapshot get_navigation_target_status(const std::string& task_id) = 0;

  /**
   * @brief Submit a multi-waypoint navigation task.
   *
   * This API sends multiple waypoints in one request and executes them in order.
   *
   * @param waypoints Ordered waypoint targets to follow.
   * @param frame_id Reference frame for the waypoint poses. Supported values:
   *                 "map", "base_link".
   * @param enable_collision_check Whether to enable collision checking and avoidance.
   *
   * @return TaskHandle containing the submitted task id, request result, and message.
   */
  virtual TaskHandle navigate_through_waypoints(const std::vector<Waypoint>& waypoints,
                                                 const std::string& frame_id = "map",
                                                 bool enable_collision_check = true) = 0;

  /**
   * @brief Submit a trajectory navigation task using ordered 3D poses.
   *
   * This API uses the input poses as a trajectory reference. Intermediate poses
   * are not guaranteed to be reached exactly; only the final pose is guaranteed
   * as the navigation goal.
   *
   * @param waypoints Ordered pose waypoints that define the trajectory.
   * @param frame_id Reference frame for the waypoint poses. Supported values:
   *                 "map", "base_link".
   * @param speed_ratio Velocity scaling factor in (0, 1.0].
   * @param enable_collision_check Whether to enable collision checking and avoidance.
   *
   * @return TaskHandle containing the submitted task id, request result, and message.
   */
  virtual TaskHandle navigate_along_trajectory(const std::vector<Pose>& waypoints, const std::string& frame_id = "map",
                                               float speed_ratio = 1.0, bool enable_collision_check = true) = 0;

  /**
   * @brief Navigate the robot to a target goal pose using the navigation v2 interface.
   *
   * This method sends a navigation v2 planning request to the PNS service. It supports
   * global and local target frames through pose_frame, and applies navigation velocity
   * and runtime timeout configurations before sending the goal request.
   *
   * @param goal_pose Target goal pose: position (x, y, z) in meters and orientation as
   *                  unit quaternion (x, y, z, w), interpreted in pose_frame.
   * @param max_vel Maximum navigation velocity limit [vx, vy, vyaw]. Each element must be
   *                in range [0.05, 1.5]. Linear velocity components are in meters per second
   *                and yaw velocity is in radians per second. Values below 0.05 may be too
   *                small to drive the base reliably.
   * @param pose_frame Reference frame of goal_pose. Only "map" and "base_link" are valid.
   *                   Default: "map".
   *                   - "map": global target pose in the map frame.
   *                   - "base_link": local target pose relative to the robot base.
   * @param enable_collision_check If true, enables the closest matching v2 collision
   *                               checking fields for planning and runtime execution.
   *                               Default: true.
   * @param is_blocking Execution mode flag. Default: false.
   *                    - false (non-blocking): Returns immediately after sending the
   *                      navigation command and starts an SDK-side monitor thread.
   *                      The return status indicates command acceptance.
   *                    - true (blocking): Blocks until the goal is reached, navigation
   *                      fails, is interrupted, or the SDK-side monitor timeout occurs.
   *                      The return status reflects the final navigation outcome.
   * @param timeout Navigation runtime timeout in seconds. Default: 5.0 seconds.
   *                This value is sent to the PNS service through set_navigation_timeout().
   *                A negative value disables the navigation motion time limit. SDK-side
   *                monitoring uses timeout + 5.0 seconds when timeout is non-negative;
   *                for negative timeout, SDK-side monitoring waits until a terminal task
   *                status is reported.
   * @param omni_plan Motion planning mode flag. Default: false.
   *                  - true: Enables omnidirectional motion planning.
   *                  - false: Uses heading-based planning.
   *                  S1 does not support omnidirectional motion planning. For S1, keep this
   *                  parameter false; setting it to true returns NavigationStatus::INVALID_INPUT.
   *
   * @return NavigationStatus indicating the result:
   *         - In non-blocking mode: Command acceptance status
   *         - In blocking mode: Final navigation outcome. SUCCESS and INTERRUPTED are
   *           reported as NavigationStatus::SUCCESS, FAILED is reported as
   *           NavigationStatus::FAIL, and SDK monitor timeout is reported as
   *           NavigationStatus::TIMEOUT.
   *
   * @note This method uses the navigation v2 topic and protocol.
   * @note pose_frame currently supports "map" for global goals and "base_link" for
   *       local goals.
   *
   * @warning This method may change navigation control parameters that also affect
   *          base control behavior, such as velocity limits and navigation timeout. If
   *          the application needs independent low-level base control afterward, set the
   *          base control parameters again to override the navigation configuration.
   * @warning In non-blocking mode, monitor navigation progress separately to detect
   *          completion or failures.
   */
  virtual NavigationStatus navigate_to_goal_v2(const Pose& goal_pose, const std::array<double, 3>& max_vel,
                                              const std::string& pose_frame = "map",
                                              bool enable_collision_check = true, bool is_blocking = false,
                                              float timeout = 5.0f, bool omni_plan = false) = 0;

  /**
   * @brief Move the robot to a relative target pose in the odometry frame.
   *
   * This method commands the robot to move to a pose specified relative to its
   * current position in the odometry (odom) frame. This is useful for short,
   * precise movements where map-based planning is not needed. Unlike navigate_to_goal(),
   * this method does NOT perform dynamic obstacle detection or global path planning.
   * It uses omnidirectional motion planning for direct movement to the target.
   *
   * @param goal_pose Target pose relative to the current base_link frame: position (x, y, z)
   *                  in meters (typically x forward, y left, z up) and relative orientation as
   *                  unit quaternion (x, y, z, w).
   * @param is_blocking Execution mode flag. Default: true.
   *                    - true (blocking): Blocks until the motion is complete, fails,
   *                      or timeout occurs. The return status reflects the final outcome.
   *                    - false (non-blocking): Returns immediately after sending the
   *                      motion command. The return status indicates command acceptance.
   * @param timeout Maximum wait time in seconds for blocking mode. Default: 8.0 seconds.
   *                Only relevant when is_blocking is true.
   *
   * @return NavigationStatus indicating the result:
   *         - In non-blocking mode: Command acceptance status
   *         - In blocking mode: Final motion outcome (success, failure, timeout)
   *
   * @note This method does NOT check for obstacles or collisions. Use only when the
   *       path is known to be clear.
   * @note This method uses the odometry frame and does NOT require map localization.
   * @note Suitable for small, precise adjustments such as final approach positioning
   *       or docking maneuvers.
   *
   * @warning Since collision checking is disabled, ensure the path is obstacle-free
   *          before calling this method to avoid collisions.
   * @warning Odometry drift may affect accuracy over longer distances. For accurate
   *          long-distance navigation, use navigate_to_goal() instead.
   */
  virtual NavigationStatus move_straight_to(const Pose& goal_pose, bool is_blocking = true, float timeout = 8) = 0;

  /**
   * @brief Navigate the robot with a velocity command using the navigation v2 interface.
   *
   * This method sends a velocity-based navigation command. The command contains planar
   * base velocity components and a duration; execution duration is handled by the
   * navigation service.
   *
   * @param vx Linear velocity along the x axis in meters per second.
   * @param vy Linear velocity along the y axis in meters per second.
   * @param vyaw Angular velocity around the z axis in radians per second.
   * @param duration_s Command duration in seconds. Must be greater than 0.0. Default: 3.0 seconds.
   * @param enable_collision_check If true, enables runtime collision checking through
   *                               the closest matching v2 collision field. Default: true.
   *
   * @return NavigationStatus indicating whether the velocity command was successfully
   *         accepted by the navigation service.
   *
   * @note This method uses the navigation v2 topic and protocol.
   * @note enable_collision_check is a runtime collision checking flag and is not a
   *       complete obstacle-avoidance policy switch.
   *
   * @warning This method is non-blocking. It returns after the velocity command is
   *          accepted, not after the velocity duration has completed.
   */
  virtual NavigationStatus navigate_with_velocity(double vx, double vy, double vyaw, double duration_s = 3.0, bool enable_collision_check = true) = 0;

  /**
   * @brief Stop the current navigation task and bring the robot to a halt.
   *
   * This method immediately cancels any ongoing navigation command (from either
   * navigate_to_goal() or move_straight_to()) and commands the robot to stop.
   * The robot will decelerate according to its kinematic constraints and come
   * to a safe stop.
   *
   * @return NavigationStatus indicating whether the stop command was successfully
   *         sent to the navigation system.
   *
   * @note This method can be called at any time during navigation, including
   *       when using non-blocking navigation modes.
   * @note After stopping, the robot's position may not match the original goal.
   * @note The robot will attempt to stop smoothly following its acceleration limits.
   */
  virtual NavigationStatus stop_navigation() = 0;

  /**
   * @brief Add a bounding box for navigation obstacle filtering.
   *
   * Sends SDK-defined box regions to the fusion service so navigation can ignore
   * the corresponding fused obstacle points instead of treating the boxes as obstacles.
   * Each box is identified by `box_tag` and attached relative to `parent_link_name`.
   *
   * @param box_info Box size, pose, tag, and parent link information.
   * @return NavigationStatus::SUCCESS if the box was accepted by fusion;
   *         NavigationStatus::INVALID_INPUT for malformed box information;
   *         NavigationStatus::COMM_ERR if the fusion service request failed.
   */
  virtual NavigationStatus add_bounding_box(const BoxInfo& box_info) = 0;

  /**
   * @brief Remove a bounding box from navigation obstacle filtering.
   *
   * Removes an SDK-created box by its `box_tag`. The tag is converted to the
   * same SDK-marked box name used by add_bounding_box().
   *
   * @param box_tag SDK box tag to remove.
   * @return NavigationStatus::SUCCESS if the box was removed by fusion;
   *         NavigationStatus::COMM_ERR if the fusion service request failed.
   */
  virtual NavigationStatus remove_bounding_box(int box_tag) = 0;

  /**
   * @brief Get bounding boxes currently used by navigation obstacle filtering.
   *
   * Queries the fusion service and converts returned SDK-marked box names back
   * to `box_tag` values when possible.
   *
   * @return Vector of current bounding box information. Returns an empty vector
   *         if communication fails or no boxes are available.
   */
  virtual std::vector<BoxInfo> get_bounding_box() = 0;

  /**
   * @brief Attach a box collision object to a robot link.
   *
   * Sends the box as an attached collision object to PNS. The box object name is
   * generated from `box_tag`, and `box_info.parent_link_name` is used as the
   * parent link for `box_info.box_pose`.
   *
   * @param box_info Box size, pose, tag, and parent link information.
   * @param ignore_collision_links Robot links to ignore for collision checking
   *        against this attached box.
   * @return NavigationStatus::SUCCESS if the box was attached;
   *         NavigationStatus::INVALID_INPUT for malformed box information;
   *         NavigationStatus::WAIT_INITIALIZED if the PNS service is not ready;
   *         NavigationStatus::COMM_ERR if the PNS request failed.
   */
  virtual NavigationStatus attach_box_to_link(
      const BoxInfo& box_info, const std::vector<std::string>& ignore_collision_links = {}) = 0;

  /**
   * @brief Detach a box collision object from its robot link.
   *
   * Removes the attached collision object whose SDK object name is generated
   * from `box_tag`.
   *
   * @param box_tag SDK box tag to detach.
   * @return NavigationStatus::SUCCESS if the box was detached;
   *         NavigationStatus::WAIT_INITIALIZED if the PNS service is not ready;
   *         NavigationStatus::COMM_ERR if the PNS request failed.
   */
  virtual NavigationStatus detach_box_from_link(int box_tag) = 0;

  /**
   * @brief Check if a collision-free path exists from start to goal in the map.
   *
   * This method queries the global path planner to determine if a valid,
   * collision-free path exists between the specified start and goal poses.
   * This is useful for validating goal poses before attempting navigation,
   * or for multi-goal path planning.
   *
   * @param goal_pose Goal pose in the map frame: position (x, y, z) in meters and orientation
   *                  quaternion (x, y, z, w).
   * @param start_pose Start pose in the map frame: position (x, y, z) in meters and orientation
   *                   quaternion (x, y, z, w).
   *
   * @return true if a collision-free path exists from start to goal,
   *         false if no valid path can be found.
   *
   * @note This method only checks for static obstacles based on the map data.
   *       Dynamic obstacles are not considered.
   * @note The path computation may take some time depending on distance and
   *       map complexity.
   * @note A return value of true does not guarantee successful navigation, as
   *       dynamic obstacles or localization errors may still cause failures.
   */
  virtual bool check_path_reachability(const Pose& goal_pose, const Pose& start_pose) = 0;

  /**
   * @brief Check if the robot has successfully reached the current goal.
   *
   * This method queries the navigation system to determine if the robot has
   * arrived at the goal pose within acceptable position and orientation tolerances.
   * This is particularly useful when using non-blocking navigation mode to poll
   * for completion.
   *
   * @return true if the robot has reached the goal within tolerance thresholds,
   *         false if still navigating, no active goal, or goal not yet reached.
   *
   * @note This method is most useful in non-blocking navigation scenarios where
   *       the application needs to monitor progress.
   * @note The tolerance thresholds for "arrival" are defined by the navigation
   *       system's internal parameters (typically a few centimeters in position
   *       and a few degrees in orientation).
   * @note If no navigation command is active, this method returns false.
   */
  virtual bool check_goal_arrival() = 0;

  /**
   * @brief Get the current navigation task state.
   *
   * Returns the latest task state reported by the navigation system. This API
   * is useful when monitoring a navigation task in non-blocking mode.
   *
   * @return NavigationTaskStatus Current task state. UNKNOWN if no status has
   *         been reported yet; RUNNING while the task is executing; terminal
   *         states when the task has finished.
   *
   * @note Useful in non-blocking navigation: loop on get_navigation_status()
   *       and break on terminal states or after a timeout.
   */
  virtual NavigationTaskStatus get_navigation_status() = 0;

  /**
   * @brief Set the navigation velocity limit.
   *
   * This method updates the navigation velocity limit through the PNS dynamic
   * configuration interface.
   *
   * @param vel_limit Maximum navigation velocity limit [vx, vy, vyaw]. Each element must be
   *                  in range [0.05, 1.5]. Linear velocity components are in meters per second
   *                  and yaw velocity is in radians per second. Values below 0.05 may be too
   *                  small to drive the base reliably.
   *
   * @return NavigationStatus indicating whether the configuration request was accepted.
   *
   * @note This configuration is intended to be set before starting a navigation task.
   *
   * @warning This method may change navigation control parameters that also affect
   *          base control behavior. If the application needs independent low-level
   *          base control afterward, set the base control parameters again to override
   *          the navigation configuration.
   */
  virtual NavigationStatus set_navigation_velocity_limit(const std::array<double, 3>& vel_limit) = 0;

  /**
   * @brief Set the navigation kinematic limits.
   *
   * This method updates navigation velocity, acceleration, and jerk limits through the
   * PNS dynamic configuration interface.
   *
   * @param vel_limit Maximum velocity limit [vx, vy, vyaw]. Each element must be in range
   *                  [0.05, 1.5]. Linear velocity components are in meters per second and
   *                  yaw velocity is in radians per second. Values below 0.05 may be too
   *                  small to drive the base reliably.
   * @param acc_limit Maximum acceleration limit [ax, ay, ayaw]. Each element must be in
   *                  range [0.05, 6.0]. Linear acceleration components are in meters per
   *                  second squared and yaw acceleration is in radians per second squared.
   *                  Values below 0.05 may be too small to drive the base reliably.
   * @param jerk_limit Maximum jerk limit [jx, jy, jyaw]. Each element must be in range
   *                   [0.05, 12.0]. Linear jerk components are in meters per second cubed
   *                   and yaw jerk is in radians per second cubed. Values below 0.05 may be
   *                   too small to drive the base reliably.
   *
   * @return NavigationStatus indicating whether the configuration request was accepted.
   *
   * @note This configuration is intended to be set before starting a navigation task.
   *
   * @warning This method may change navigation control parameters that also affect
   *          base control behavior. If the application needs independent low-level
   *          base control afterward, set the base control parameters again to override
   *          the navigation configuration.
   */
  virtual NavigationStatus set_navigation_kinematics_limits(const std::array<double, 3>& vel_limit,
                                                           const std::array<double, 3>& acc_limit,
                                                           const std::array<double, 3>& jerk_limit) = 0;

  /**
   * @brief Set the navigation timeout configuration.
   *
   * This method updates the navigation timeout through the PNS dynamic configuration
   * interface. It is useful as a safety guard for tasks that should not run indefinitely.
   *
   * @param timeout_s Navigation timeout in seconds. A value less than or equal to 0 disables
   *                  the navigation motion time limit.
   *
   * @return NavigationStatus indicating whether the configuration request was accepted.
   *
   * @note The exact service-side meaning of this timeout is defined by the PNS
   *       navigation service.
   */
  virtual NavigationStatus set_navigation_timeout(double timeout_s) = 0;

  /**
   * @brief Set the navigation arrival threshold.
   *
   * This method updates the position and yaw tolerances used by navigation planning
   * and control to determine whether a target has been reached.
   *
   * @param threshold Arrival threshold [x_error, y_error, yaw_error]. Each element must be
   *                  in range [0.03, 2.0]. Position errors are in meters and yaw error is
   *                  in radians. The minimum supported precision is 0.03 m or rad.
   *
   * @return NavigationStatus indicating whether the configuration request was accepted.
   *
   * @note Different task types may require different arrival precision. Configure this
   *       value before starting the corresponding navigation task.
   */
  virtual NavigationStatus set_navigation_arrival_threshold(const std::array<double, 3>& threshold) = 0;


  /**
   * @brief Dump navigation dynamic configuration for debugging.
   *
   * This method queries commonly used navigation configuration keys and prints the
   * response through the SDK logger. It is intended for diagnostics and debugging.
   *
   * @return NavigationStatus indicating whether all debug query requests were accepted.
   *
   * @note The output format depends on the PNS service response and may be JSON or
   *       protobuf text format.
   */
  virtual NavigationStatus dump_navigation_configs() = 0;

};

}  // namespace sdk
}  // namespace galbot
