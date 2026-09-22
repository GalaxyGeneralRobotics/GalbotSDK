/**
 * @file galbot_robot.hpp
 * @brief Core robot control interface for Galbot G1 humanoid robot
 *
 * This file provides the main control interface for the Galbot G1 robot,
 * including joint control, end-effector manipulation, sensor data acquisition,
 * and coordinate frame transformations.
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
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "galbot_motion.hpp"
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
 * @class GalbotRobot
 * @brief Main robot control interface for Galbot humanoid robot
 *
 * This class provides a singleton interface for controlling the Galbot robot.
 * It supports:
 * - Joint position and trajectory control
 * - End-effector control
 * - Mobile base velocity control
 * - Sensor data acquisition
 * - Coordinate frame transformations
 * - System lifecycle management
 *
 * Use GalbotRobot::get_instance(MachineType) to obtain a reference for a specific platform (G1/S1/G3).
 *
 * @note All angles are in radians unless otherwise specified.
 * @note All linear distances are in meters unless otherwise specified.
 * @note All timestamps are in nanoseconds unless otherwise specified.
 */
class GalbotRobot {
   public:
    virtual ~GalbotRobot() = default;

    /**
     * @brief Runtime factory for selecting a concrete robot singleton.
     * @param m Machine type identifier (e.g. MachineType::G1, MachineType::S1, MachineType::G3).
     * @return Reference to the singleton robot interface for the specified machine type.
     */
    static GalbotRobot& get_instance(MachineType m);

    /**
     * @brief Initialize the robot control system
     *
     * Initializes the robot hardware communication, middleware, and sensor interfaces.
     * To optimize resource usage, only sensors specified in the enable_sensor_set
     * will be initialized and available for data reading.
     *
     * This method should only be called once at program startup. Calling it multiple
     * times without calling destroy() will not error, but only the first call has effect.
     *
     * @param enable_sensor_set Set of sensors to enable. If empty, a default set of sensors
     *                          will be enabled. Specify only required sensors to reduce
     *                          computational overhead and memory consumption.
     * @return true if initialization succeeded
     * @return false if initialization failed
     */
    virtual bool init(const std::unordered_set<SensorType>& enable_sensor_set = {},
                      bool enable_sync_mode = false) = 0;

    /**
     * @brief Publish a raw target without waiting for a service response.
     *
     * This is the lightweight high-frequency path for advanced users who want to
     * construct a SingoriXTarget directly and send it through the WBC publish channel.
     * The SDK performs only basic structural validation before publishing.
     *
     * @param target SDK mirror of singorix target data
     * @return ControlStatus indicating local validation / publish result
     */
    virtual ControlStatus PublishTarget(const SingoriXTarget& target) = 0;

    /**
     * @brief Request execution of a raw target and receive the service response.
     *
     * This is the request/service path for advanced users who want direct target
     * control with request-side error screening and a returned error payload.
     * The SDK uses the middleware default timeout configured by the underlying client.
     *
     * @param target SDK mirror of singorix target data
     * @return Shared pointer to ErrorInfo on local pre-check failure or service response.
     *         Returns nullptr when the client is unavailable, disconnected, times out,
     *         or returns an empty response.
     */
    virtual std::shared_ptr<ErrorInfo> RequestTarget(const SingoriXTarget& target) = 0;

    /**
     * @brief Set low-level joint commands for high-frequency streaming control
     *
     * Suitable for high-frequency command streaming (for example, per-frame model inference output).
     *
     * This API does not interpolate from the current/start position to the first target.
     * The controller drives joints toward each commanded target as quickly as possible.
     *
     * For standard joints (head, legs, arms), only `JointCommand::position` is effective in current versions;
     * `velocity`, `acceleration`, and `effort` are currently ignored.
     *
     * For gripper joints, the position field represents gripper width and both velocity and effort fields are supported
     * and effective.
     *
     * @param joint_commands Vector of low-level joint commands.
     * @param joint_groups Joint groups to control. Supported groups: legs, head, left_arm,
     *                     right_arm, gripper, suction_cup.
     *                     Must not be empty if `joint_names` is also empty; otherwise returns INVALID_INPUT.
     * @param joint_names Specific joint names to control. This parameter takes precedence
     *                    over joint_groups. When provided, joint_groups is ignored.
     *                    Must not be empty if `joint_groups` is also empty; otherwise returns INVALID_INPUT.
     * @param time_from_start_s Execution will begin after time_start_s seconds.(optional, default: 0.0)
     * @warning Especially on the first command, avoid a large gap between current and target joint angles.
     *          Large jumps may cause excessively fast motion and safety risk.
     * @return ControlStatus indicating success or failure of command transmission.
     */
    virtual ControlStatus set_joint_commands(const std::vector<JointCommand>& joint_commands,
                                             const std::vector<std::string>& joint_groups = {},
                                             const std::vector<std::string>& joint_names = {},
                                             const double time_from_start_s = 0.0) = 0;
    /**
     * @brief Set target joint positions for specified joint groups by name (for low-frequency keyframe/posture
     * transitions)
     *
     * Commands the robot to move specified joints to target positions. The motion
     * is executed as a smooth trajectory with configurable speed limits.
     *
     * @param joint_positions Vector of target joint angles in radians. The order must match
     *                        the joint ordering returned by get_joint_names() for the specified
     *                        joint_groups or joint_names.
     * @param joint_groups Joint group names to control. Supported groups: "legs", "head",
     *                     "left_arm", "right_arm".
     *                     Must not be empty if `joint_names` is also empty; otherwise returns INVALID_INPUT.
     * @param joint_names Specific joint names to control. This parameter takes precedence over
     *                    joint_groups. When provided, joint_groups is ignored.
     *                    Must not be empty if `joint_groups` is also empty; otherwise returns INVALID_INPUT.
     * @param is_blocking If true, blocks until motion completes or timeout occurs.
     *                    If false, returns immediately after command is sent.
     * @param speed_rad_s Maximum joint angular velocity in radians per second (rad/s).
     *                    Default: 0.2 rad/s.
     * @param timeout_s Maximum blocking wait time in seconds. Returns immediately upon timeout
     *                  regardless of execution completion. Default: 15 seconds.
     *
     * @warning This API is not suitable for high-frequency frame-by-frame model inference control.
     *          Each call creates a new interpolation goal, and continuous calls can cause lag or
     *          discontinuous motion. If your task is model-inference command streaming, use
     *          `set_joint_commands` or `set_joint_commands_batch` instead.
     * @return ControlStatus indicating success or failure of the motion command
     */
    virtual ControlStatus set_joint_positions(const std::vector<double>& joint_positions,
                                              const std::vector<std::string>& joint_groups = {},
                                              const std::vector<std::string>& joint_names = {},
                                              const bool is_blocking = true, const double speed_rad_s = 0.2,
                                              const double timeout_s = 15) = 0;

    /**
     * @brief Get trajectory execution status for specified joint groups
     *
     * Queries the current execution status of trajectories for the specified joint groups.
     * This is useful for monitoring trajectory progress in non-blocking execution mode.
     *
     * @param joint_groups Vector of joint group names to query
     * @return Vector of TrajectoryControlStatus indicating execution state for each group
     */
    virtual std::vector<TrajectoryControlStatus> check_trajectory_execution_status(
        std::vector<std::string> joint_groups) = 0;
    /**
     * @brief Execute a pre-planned joint trajectory
     *
     * Executes a trajectory consisting of waypoints with associated joint positions,
     * velocities, and timing information. The trajectory controller interpolates
     * between waypoints to generate smooth motion.
     * 
     * For standard joints (head, legs, arms), only `position` is effective in current versions;
     * `velocity`, `acceleration`, and `effort` are currently ignored.
     *
     * @param trajectory Trajectory data structure containing waypoints and timing.
     *                   Either `trajectory.joint_groups` or `trajectory.joint_names` must be specified;
     *                   returns INVALID_INPUT if both are empty.
     *                   @warning Each `TrajectoryPoint.joint_command_vec` order MUST match the joint order
     *                   defined by `trajectory.joint_names` or the expanded order of `trajectory.joint_groups`.
     * @param is_blocking If true, blocks until trajectory execution completes.
     *                    If false, returns immediately after trajectory is submitted.
     * @warning For per-frame model inference output, prefer command streaming interfaces
     *          (`set_joint_commands` / `set_joint_commands_batch`) rather than repeatedly
     *          re-submitting full trajectories.
     * @return ControlStatus indicating success or failure of trajectory execution/submission
     */
    virtual ControlStatus execute_joint_trajectory(Trajectory trajectory, bool is_blocking = true) = 0;
    /**
     * @brief Set joint commands in batch mode (non-blocking)
     *
     * Sets multiple joint command trajectory points in real-time control mode,
     * supporting one-time submission of trajectory control commands for multiple
     * time points. Provides a non-blocking high-frequency trajectory execution
     * interface. Similar to set_joint_commands but supports batch trajectory control,
     * suitable for scenarios such as VLA inference batch output.
     *
     * @param trajectory Trajectory data structure containing waypoints with joint commands.
     *                   Either `trajectory.joint_groups` or `trajectory.joint_names` must be specified;
     *                   returns INVALID_INPUT if both are empty.
     *                   Each TrajectoryPoint contains time_from_start and a list of JointCommand.
     *                   JointCommand includes position (rad), velocity (rad/s), acceleration (rad/s²),
     *                   effort (N·m), Kp (position gain), and Kd (velocity gain).
     *                   @warning Each `TrajectoryPoint.joint_command_vec` order MUST match the joint order
     *                   defined by `trajectory.joint_names` or the expanded order of `trajectory.joint_groups`.
     * @return ControlStatus indicating success or failure of command submission.
     *         Returns immediately without waiting for execution completion (non-blocking).
     */
    virtual ControlStatus set_joint_commands_batch(const Trajectory& trajectory) = 0;

    /**
     * @brief Control suction cup activation state
     *
     * Activates or deactivates the specified suction cup end-effector.
     *
     * @param end_effector Joint group name specifying which suction cup to control (e.g., "left_suction_cup",
     * "right_suction_cup")
     * @param activate If true, activates vacuum suction. If false, releases suction.
     * @return ControlStatus indicating success or failure of command transmission
     * @robot G1 G3
     */
    virtual ControlStatus set_suction_cup_command(const std::string& end_effector, bool activate) = 0;
    /**
     * @brief Control gripper opening width and force
     *
     * Commands the gripper to move to a specified opening width with controlled
     * velocity and maximum gripping force.
     *
     * @param end_effector Joint group name specifying which gripper to control (e.g., "left_gripper", "right_gripper")
     * @param width_m Target gripper opening width in meters (m), measured between
     *                the inner surfaces of the gripper fingers. G1 gripper width range is 0 to 0.12 m. S1 long-stroke gripper
     *                width range is 0.007 to 0.11 m. S1 short-stroke gripper width range is 0.007 to 0.076 m.
     * @param velocity_mps Gripper closing/opening velocity in meters per second (m/s).
     *                     Default: 0.03 m/s. The value range is greater than 0 and 
     *                     less than or equal to 0.2 m/s.
     * @param effort Maximum gripping force in Newton-meters (N·m). This limits the
     *               torque applied to prevent damage to grasped objects. The value range is greater than 0 and 
     *               less than or equal to 100. Default: 5 N·m.
     * @param is_blocking If true, blocks until gripper reaches target position or times out.
     *                    If false, returns immediately after command is sent.
     * @return ControlStatus indicating success or failure of gripper command
     */
    virtual ControlStatus set_gripper_command(const std::string& end_effector, double width_m,
                                              double velocity_mps = 0.03, double effort = 5,
                                              bool is_blocking = true) = 0;
    /**
     * @brief Get current gripper state
     *
     * Retrieves the current state of the specified gripper, including position,
     * velocity, force, and motion-state estimation.
     *
     * `GripperState::is_moving` is window-based: if no effective width change is
     * detected within the internal time window, `is_moving` becomes false.
     *
     * @param end_effector Joint group name specifying which gripper to query (e.g., "left_gripper", "right_gripper")
     * @return Shared pointer to GripperState, or nullptr if retrieval fails
     */
    virtual std::shared_ptr<GripperState> get_gripper_state(const std::string& end_effector) = 0;

    /**
     * @brief Publish one opaque 64-byte frame to an arm's TIB end-tool channel.
     *
     * This is a transport-level API. The SDK does not encode or validate the
     * device-specific CAN/RS485 payload contained in @p frame. A SUCCESS result
     * means that the DDS message was published locally; it does not prove that
     * WBCS, the TIB, or the end tool accepted the command.
     *
     * @param side Left or right arm/TIB channel.
     * @param frame Device-specific frame in the S1 WBCS 64-byte TIB layout.
     * @return Command publication status.
     * @note On S1 GBS 1.18.1, set
     *       `[robot_info.custom_params].endtool_raw_enabled = true` before WBCS starts.
     * @warning The raw TX path has no server-side exclusive-writer lock. Do not
     *          send raw frames concurrently with a production controller that
     *          writes to the same end tool.
     * @robot S1
     */
    virtual ControlStatus send_endtool_raw_frame(
        EndToolSide side, const std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE>& frame) = 0;

    /**
     * @brief Register a callback for raw end-tool receive frames.
     *
     * Both endpoint/channel combinations are delivered through a shared DDS
     * topic to every registered callback. Callbacks execute on middleware
     * threads. The caller must
     * filter EndToolRawData::side/kind as needed and make its callback
     * thread-safe.
     *
     * @param callback Callback to register.
     * @return Non-zero registration handle, or 0 when registration fails.
     * @robot S1
     */
    virtual uint64_t register_endtool_raw_callback(EndToolRawCallback callback) = 0;

    /**
     * @brief Unregister a raw end-tool callback.
     *
     * A callback invocation already in progress may finish after this method
     * returns.
     *
     * @param handle Handle returned by register_endtool_raw_callback().
     * @return true if the handle existed and was removed.
     * @robot S1
     */
    virtual bool unregister_endtool_raw_callback(uint64_t handle) = 0;

    /**
     * @brief Get current suction cup state
     *
     * Retrieves the current state of the specified suction cup, including
     * activation status and vacuum pressure measurements.
     *
     * @param end_effector Joint group name specifying which suction cup to query (e.g., "left_suction_cup",
     * "right_suction_cup")
     * @return Shared pointer to SuctionCupState, or nullptr if retrieval fails
     * @robot G1 G3
     */
    virtual std::shared_ptr<SuctionCupState> get_suction_cup_state(const std::string& end_effector) = 0;
    /**
     * @brief Get current dexterous hand (dexhand) state
     * 
     * Retrieves dexhand feedback into @p dexhand_state. For INSPIRE,
     * INSPIRE_RH56DFX, INSPIRE_RH56F2, and BRAINCO, only
     * `dexhand_state.joint_state` is filled and `force_sensor_map` is empty.
     * For SHARPA, fills full joint state plus named force sensor data when available.
     *
     * @param end_effector Joint group name specifying which dexhand to query (e.g., "left_dexhand", "right_dexhand")
     * @param dexhand_state Output dexhand state (joint_state and optional force_sensor_map)
     * @param dexhand_type Dexterous hand model type, which supports INSPIRE, INSPIRE_RH56DFX, INSPIRE_RH56F2, BRAINCO, SHARPA and LINKER_L20(INSPIRE is kept for
     *                     compatibility and is treated as INSPIRE_RH56F2).
     * @return ControlStatus indicating success or failure
     *
     * @robot G1 G3
     */
    virtual ControlStatus get_dexhand_state(const std::string& end_effector, DexhandState& dexhand_state,
                                            DexHandType dexhand_type = DexHandType::INSPIRE) = 0;

    /**
     * @brief Control dexhand with joint commands
     *
     * Commands the dexhand with a vector of joint commands (position, velocity, effort, etc.).
     * 
     * @param end_effector Joint group name specifying which dexhand to control (e.g., "left_dexhand", "right_dexhand")
     * @param dexhand_command Vector of joint commands for each dexhand joint
     *                        inspire series: 6 joint commands. Position range depends on
     *                        INSPIRE_RH56DFX / INSPIRE_RH56F2, while velocity/effort use
     *                        the common inspire validation range.
     *                        brainco: [position, velocity, acceleration, effort] range [0-100, -100-100, --, --]
     *                        sharpa: 22 joint commands, [position, velocity, acceleration, effort]
     * @param dexhand_type Dexterous hand model type, which supports INSPIRE, INSPIRE_RH56DFX, INSPIRE_RH56F2, BRAINCO, SHARPA and LINKER_L20(INSPIRE is kept for
     *                     compatibility and is treated as INSPIRE_RH56F2).
     * @param is_blocking If true, blocks until the hand reaches target position or times out.
     * @return ControlStatus indicating success or failure of command transmission
     * 
     * @robot G1 G3
     */
    virtual ControlStatus set_dexhand_command(const std::string& end_effector,
                                              const std::vector<JointCommand>& dexhand_command,
                                              DexHandType dexhand_type = DexHandType::INSPIRE,
                                              bool is_blocking = true) = 0;

    /**
     * @brief Get current joint positions by group name
     *
     * Retrieves the current angular positions of joints in the specified groups.
     *
     * @param joint_groups Joint group names to query. Empty vector retrieves all body joints.
     * @param joint_names Specific joint names to query. This parameter takes precedence
     *                    over joint_groups. When provided, joint_groups is ignored.
     * @return Vector of current joint angles in radians
     *
     * @note Return order:
     *       - joint_names specified: Returns in the exact order of joint_names.
     *       - Only joint_groups specified: Returns in the order groups are defined.
     *       - Both empty (machine-dependent body-joint order):
     *         - G1: chassis, head, left_arm, right_arm, leg
     *         - S1: torso, head, left_arm, right_arm
     */
    virtual std::vector<double> get_joint_positions(const std::vector<std::string>& joint_groups,
                                                    const std::vector<std::string>& joint_names) = 0;
    /**
     * @brief Get available joint group names for the robot
     *
     * Retrieves all joint group names defined in the robot's kinematic configuration.
     * This is useful for discovering available control groups at runtime.
     *
     * @return Vector of joint group names, or empty vector if retrieval fails
     */
    virtual std::vector<std::string> get_joint_group_names() = 0;
    /**
     * @brief Get robot joint names by group name
     *
     * Retrieves the names of joints belonging to specified joint groups. This
     * is useful for determining the correct ordering when setting joint positions.
     *
     * @param only_active_joint If true, returns only actuated joints (excludes passive/fixed joints).
     *                          If false, returns all joints including passive ones.
     * @param joint_groups Joint group names to query. Empty vector retrieves joints from all groups.
     * @return Vector of joint names
     */
    virtual std::vector<std::string> get_joint_names(bool only_active_joint = true,
                                                     const std::vector<std::string>& joint_groups = {}) = 0;
    /**
     * @brief Get real-time joint states by group name
     *
     * Retrieves comprehensive state information for specified joints, including
     * position, velocity, acceleration, effort (torque), and other feedback data.
     *
     * @param joint_group_vec Joint group names to query. Empty vector defaults to all body joints.
     * @param joint_names_vec Specific joint names to query. This parameter takes precedence
     *                        over joint_group_vec. When provided, joint_group_vec is ignored.
     * @return Vector of JointState structures containing current state for each joint
     *
     * @note Return order:
     *       - joint_names_vec specified: Returns in the exact order of joint_names_vec.
     *       - Only joint_group_vec specified: Returns in the order groups are defined.
     *       - Both empty (machine-dependent body-joint order):
     *         - G1: chassis, head, left_arm, right_arm, leg
     *         - S1: torso, head, left_arm, right_arm
     */
    virtual std::vector<JointState> get_joint_states(const std::vector<std::string>& joint_group_vec,
                                                     const std::vector<std::string>& joint_names_vec = {}) = 0;
    /**
     * @brief Set mobile base velocity command
     *
     * Commands the robot's mobile base to move with specified linear and angular velocities.
     * Velocities are expressed in the robot's base frame coordinate system.
     *
     * @param linear_velocity Linear velocity in meters per second (m/s), expressed in base frame.
     *                        Order: {vx, vy, vz} where:
     *                        - vx: forward/backward velocity (positive forward)
     *                        - vy: left/right velocity (positive left)
     *                        - vz: up/down velocity (typically 0 for ground robots)
     * @param angular_velocity Angular velocity in radians per second (rad/s), expressed in base frame.
     *                         Order: {wx, wy, wz} where:
     *                         - wx: roll rate (rotation about x-axis)
     *                         - wy: pitch rate (rotation about y-axis)
     *                         - wz: yaw rate (rotation about z-axis, positive counter-clockwise)
     * @param duration_s Velocity publishing window in seconds (default: 0.0).
     *                   Zero publishes once and returns. Positive values block the calling thread,
     *                   publishing immediately and then at 10 Hz until the window ends.
     *                   Negative, non-finite, or unrepresentable durations are invalid.
     * @return SUCCESS when publishing completes; INVALID_INPUT for invalid input;
     *         STOPPED_UNREACHED on SDK shutdown;
     *         otherwise the initialization, controller, communication, fault, or publishing error.
     * @note No stop command is sent on expiry or early exit. Actual stopping depends on the
     *       underlying watchdog and braking; SUCCESS does not mean the base has stopped.
     * @note The window starts after the first successful publish. Controller switching and
     *       publishing overhead add to call latency. Waiting releases the CPU; Python releases the GIL.
     * @note Do not issue concurrent base commands during this call; other commands do not
     *       cancel this publishing loop. For caller-controlled stopping, use single-publish mode.
     */
    virtual ControlStatus set_base_velocity(const std::array<double, 3>& linear_velocity,
                                            const std::array<double, 3>& angular_velocity, double duration_s = 0.0) = 0;
    /**
     * @brief Set the G1 or G3 robot leg height
     *
     * Switches the leg group to the leg height controller and asynchronously publishes
     * a smooth motion target for head_base_link in base_link. The supported target
     * range is [0.96, 1.54] m and the supported duration range is [0, 60] s. A duration
     * of 0 requests the fastest motion and should be used with caution. This method
     * returns after publishing the target by default. In blocking mode, it waits for
     * the matching leg_height task to report target_finished, while monitoring runtime
     * errors and leg sensor availability.
     *
     * @param link_height Target Z coordinate of head_base_link in base_link, in meters.
     * @param duration Requested minimum motion duration in seconds. If it is too short,
     *                 the server extends the actual duration to satisfy the configured
     *                 velocity, acceleration, and jerk limits.
     * @param is_blocking Whether to wait for the physical motion to finish.
     * @param timeout_s Blocking timeout in seconds. A value <= 0 uses duration + 8 seconds.
     * @return ControlStatus indicating the publish result, or the completed motion result in blocking mode.
     * @note The current height can be read from the z component returned by
     *       get_transform("base_link", "head_base_link").
     * @robot G1 G3
     */
    virtual ControlStatus set_leg_height(double link_height, double duration = 4.0,
                                         bool is_blocking = false, double timeout_s = -1.0) = 0;
    /**
     * @brief Get current mobile base velocity
     *
     * Returns the latest base velocity in a structured form.
     *
     * @return Shared pointer to BaseVelocityInfo on success, or nullptr on failure.
     */
    virtual std::shared_ptr<BaseVelocityInfo> get_base_velocity() = 0;
    /**
     * @brief Set mobile base pose command
     *
     * Commands the robot's mobile base to move to a specified pose in its reference frame.
     * This uses the chassis pose controller (CHASSIS_POSE_CTRL).
     * Use this overload when a full 3D pose (position + quaternion orientation) is already available.
     *
     * @param base_pose Target base pose [x, y, z, qx, qy, qz, qw]
     * @param is_blocking If true, waits for controller response; if false, returns immediately after request
     * @param timeout_s Timeout for blocking request (seconds)
     * @return ControlStatus indicating success or failure of command transmission
     */
    virtual ControlStatus set_base_pose(const Pose& base_pose, bool is_blocking = true, double timeout_s = 15.0) = 0;
    /**
     * @brief Set mobile base pose (x, y, yaw) with selectable frames
     *
     * Use this overload for planar 2D goal commands defined by x/y/yaw in the selected frame.
     *
     * @param x Target x position (meters)
     * @param y Target y position (meters)
     * @param yaw Target yaw (radians)
     * @param frame_id Frame id of target. Current recommended value: "rel(0)".
     *                 "rel(0)" means the x/y/yaw target is interpreted relative to the current base pose.
     *                 "base_link", "odom", and "map" are retained for compatibility, but are not recommended
     *                 for current use and may be changed or removed in a future update.
     *                 Default: "rel(0)"
     * @param reference_frame_id Reference frame id. Options: "odom" / "map"
     * @param is_blocking If true, waits for controller response; if false, returns immediately after request
     * @param timeout_s Timeout for blocking request (seconds)
     * @return ControlStatus indicating success or failure of command transmission
     */
    virtual ControlStatus set_base_pose(double x, double y, double yaw, const std::string& frame_id = "rel(0)",
                                        const std::string& reference_frame_id = "odom", bool is_blocking = true,
                                        double timeout_s = 15.0) = 0;
    /**
     * @brief Set mobile base pose (x, y, yaw) with explicit interpolation time
     *
     * Use this overload when arrival timing must be coordinated through `time_from_start_s`.
     *
     * @param x Target x position (meters)
     * @param y Target y position (meters)
     * @param yaw Target yaw (radians)
     * @param frame_id Frame id of target. Current recommended value: "rel(0)".
     *                 "rel(0)" means the x/y/yaw target is interpreted relative to the current base pose.
     *                 "base_link", "odom", and "map" are retained for compatibility, but are not recommended
     *                 for current use and may be changed or removed in a future update.
     * @param reference_frame_id Reference frame id. Options: "odom" / "map"
     * @param time_from_start_s Chassis pose interpolation time (seconds)
     * @param is_blocking If true, waits for controller response; if false, returns immediately after request
     * @param timeout_s Request timeout (seconds)
     * @return ControlStatus indicating success or failure of command transmission
     */
    virtual ControlStatus set_base_pose(double x, double y, double yaw, const std::string& frame_id,
                                        const std::string& reference_frame_id, double time_from_start_s,
                                        bool is_blocking = true, double timeout_s = 15.0) = 0;

    /**
     * @brief Set WBC end-effector pose commands for high-frequency real-time control (task trajectory publish).
     *
     * Each row of `poses` is [x, y, z, qx, qy, qz, qw] (meters, quaternion xyzw).
     * `poses` and `end_effector_frames` must have equal length.
     *
     * @param poses Row-major poses: one vector per end effector, seven values each.
     * @param end_effector_frames Target frame names (e.g. link names).
     * @param reference_frames Optional; per-pose reference frame id. If omitted or empty, every pose uses `"world"`.
     *                         Typical values: `"world"` (default). Length must match `poses` when non-empty.
     * @return ControlStatus indicating success or failure of command transmission
     */
    virtual ControlStatus set_end_effector_command(const std::vector<std::vector<double>>& poses,
                                                   const std::vector<std::string>& end_effector_frames,
                                                   const std::vector<std::string>& reference_frames = {}) = 0;

    /**
     * @brief Clear WBC end-effector task commands
     *
     * Clears published end-effector task trajectory commands for the WBC channel.
     *
     * @param hold If true, the WBC joint reference is latched to the current
     *             sensor pose so joints hold their current configuration after
     *             the clear. If false (default), joints return to the
     *             configured reference pose.
     * @return ControlStatus indicating success or failure of command transmission
     */
    virtual ControlStatus clear_end_effector_command(bool hold = false) = 0;

    /**
     * @brief Get WBC end effector poses
     *
     * Retrieves the current poses of WBC end effectors (left arm, right arm, head).
     *
     * @return Dictionary-style map containing:
     *         - "lee_pose": [x, y, z, qx, qy, qz, qw] for left arm end effector
     *         - "ree_pose": [x, y, z, qx, qy, qz, qw] for right arm end effector
     *         - "head_pose": [x, y, z, qx, qy, qz, qw] for head
     *         Returns empty entries or partial data on failure or missing keys.
     */
    virtual std::unordered_map<std::string, std::vector<double>> get_wbc_end_effector_poses() = 0;

    /**
     * @brief Emergency stop mobile base movement
     *
     * Immediately commands the mobile base to stop all motion. This is a safety
     * function that should be used when immediate cessation of base motion is required.
     *
     * @return ControlStatus indicating success or failure of command transmission
     */
    virtual ControlStatus stop_base() = 0;
    /**
     * @brief One-key zero: move whole-body joints to zero and base to zero pose
     *
     * This calls move_whole_body_joint_zero for joint zeroing, and
     * commands the base pose to zero. If params is nullptr, default_param is used.
     *
     * @param base_zero_pose Target base zero pose [x, y, z, qx, qy, qz, qw]
     * @param is_blocking Whether to block on joint zeroing
     * @param leg_head_speed_rad_s Max joint speed for leg/head direct control (rad/s)
     * @param leg_head_timeout_s Timeout for leg/head direct control (seconds)
     * @param params Motion planning parameters (nullptr uses default_param)
     * @return Pair of (MotionStatus for joints, ControlStatus for base)
     */
    virtual std::pair<MotionStatus, ControlStatus> zero_whole_body_and_base(
        const Pose& base_zero_pose, bool is_blocking = true, double leg_head_speed_rad_s = 0.2,
        double leg_head_timeout_s = 15.0, std::shared_ptr<Parameter> params = nullptr) = 0;
    /**
     * @brief One-key zero: move whole-body joints to zero and base (x,y,yaw) to zero with selectable frames
     *
     * @param frame_id Frame id of target. Options: "base_link" / "odom" / "map". Default "odom".
     * @param reference_frame_id Reference frame id. Options: "odom" / "map"
     * @param is_blocking Whether to block on joint zeroing
     * @param leg_head_speed_rad_s Max joint speed for leg/head direct control (rad/s)
     * @param leg_head_timeout_s Timeout for leg/head direct control (seconds)
     * @param params Motion planning parameters (nullptr uses default_param)
     * @return Pair of (MotionStatus for joints, ControlStatus for base)
     */
    virtual std::pair<MotionStatus, ControlStatus> zero_whole_body_and_base(
        const std::string& frame_id = "odom", const std::string& reference_frame_id = "odom", bool is_blocking = true,
        double leg_head_speed_rad_s = 0.2, double leg_head_timeout_s = 15.0,
        std::shared_ptr<Parameter> params = nullptr) = 0;

    /**
     * @brief Stop all currently executing joint trajectories
     *
     * Immediately halts execution of all active joint trajectories across all joint groups.
     * Joints will maintain their current positions after stopping.
     *
     * @return ControlStatus indicating success or failure of command transmission
     */
    virtual ControlStatus stop_trajectory_execution() = 0;
    /**
     * @brief Reload a controller
     *
     * Reinitializes the controller. Equivalent to a full restart cycle: stop -> reset -> start.
     * Useful for error recovery or applying configuration changes.
     *
     * @param group_name Name of the joint group to reload. Supported groups: chassis,
     *                   legs, head, left_arm, right_arm, gripper, or "all" to reload all controllers (default).
     * @return ControlStatus indicating success or failure of the reload operation
     */
    virtual ControlStatus reload_controller(const std::string& group_name = "all") = 0;
    /**
     * @brief Switch active controller strategy
     *
     * Transitions hardware control to a new strategy.
     * The request is sent to the WBCS controller manager, which performs the
     * safe switch and starts the selected controller when accepted.
     *
     * @note WBCS enforces controller priority. When a BYPASS controller has a
     *       higher priority than the target PVT controller, it cannot switch
     *       directly to PVT. Call stop_controller(group_name),
     *       release_controller(group_name), acquire_controller(pvt_controller),
     *       and start_controller(group_name) in that order.
     *
     * @param controller_name Controller name string, for example "chassis_pose_ctrl".
     * @return ControlStatus::SUCCESS if WBCS accepts the switch; ControlStatus::INVALID_INPUT
     *         if the controller name is unknown; ControlStatus::INIT_FAILED if the WBCS client
     *         is unavailable; ControlStatus::TIMEOUT if no response is received within 5 seconds;
     *         or ControlStatus::FAULT if the response contains any controller-command error.
     */
    virtual ControlStatus switch_controller(const std::string& controller_name) = 0;
    /**
     * @brief Get active controller name for specified joint group
     *
     * Returns the controller last known by this SDK instance. It is initialized
     * with the model default controller and updated after successful SDK
     * controller management calls. If control has been released or the group has
     * no known controller, returns "CONTROLLER_NAME_NUM".
     *
     * @param group_name Name of the joint group to query.
     * @return std::string Name of the active controller known by this SDK instance.
     */
    virtual std::string get_active_controller(const std::string& group_name) = 0;
    /**
     * @brief Acquire hardware authority
     *
     * Requests the specified controller to take ownership of the hardware. This
     * uses the same WBCS switch request as switch_controller.
     *
     * @param controller_name Controller name string, for example "left_arm_pvt_ctrl".
     * @return ControlStatus indicating success or failure of the acquire operation.
     */
    virtual ControlStatus acquire_controller(const std::string& controller_name) = 0;
    /**
     * @brief Release hardware authority
     *
     * Yields control of the hardware, freeing the joints.
     * Opposite of acquire_controller. Implicitly stops execution if running.
     *
     * @param group_name Name of the joint group to release. Supported groups: chassis,
     *                   legs, head, left_arm, right_arm, gripper, or "all" to release all controllers (default).
     * @return ControlStatus indicating success or failure of the release operation
     */
    virtual ControlStatus release_controller(const std::string& group_name = "all") = 0;
    /**
     * @brief Start controller execution
     *
     * Activates the controller to begin sending commands.
     * Opposite of stop_controller. Requires prior hardware authority (acquire).
     *
     * @param group_name Name of the joint group to start. Supported groups: chassis,
     *                   legs, head, left_arm, right_arm, gripper, or "all" to start all controllers (default).
     * @return ControlStatus indicating success or failure of the start operation
     */
    virtual ControlStatus start_controller(const std::string& group_name = "all") = 0;
    /**
     * @brief Stop controller execution
     *
     * Halts command execution but retains hardware authority.
     * Opposite of start_controller.
     *
     * @param group_name Name of the joint group to stop. Supported groups: chassis,
     *                   legs, head, left_arm, right_arm, gripper, or "all" to stop all controllers (default).
     * @return ControlStatus indicating success or failure of the stop operation
     */
    virtual ControlStatus stop_controller(const std::string& group_name = "all") = 0;

    /**
     * @brief Get IMU (Inertial Measurement Unit) sensor data
     *
     * Retrieves the latest IMU measurements including linear acceleration,
     * angular velocity, and orientation estimation.
     *
     * @param imu_type SensorType enumeration specifying which IMU to query
     * @return Shared pointer to ImuData structure, or nullptr if sensor is not enabled
     *         or data retrieval fails
     *
     * @note The IMU sensor must be enabled during initialization via enable_sensor_set
     * @note Acceleration is in meters per second squared (m/s²)
     * @note Angular velocity is in radians per second (rad/s)
     */
    virtual std::shared_ptr<ImuData> get_imu_data(SensorType imu_type) = 0;
    /**
     * @brief Get robot odometry information
     *
     * Retrieves the robot's current pose and velocity estimates from the odometry system.
     * Odometry typically fuses wheel encoders, IMU, and other proprioceptive sensors.
     *
     * @return Shared pointer to OdomData containing:
     *         - Position in meters (m) relative to odometry frame origin
     *         - Orientation as quaternion
     *         - Linear velocity in meters per second (m/s)
     *         - Angular velocity in radians per second (rad/s)
     *         - Timestamp in nanoseconds
     *         Returns nullptr if odometry is unavailable.
     */
    virtual std::shared_ptr<OdomData> get_odom() = 0;
    /**
     * @brief Get device information
     *
     * Retrieves basic device information including device model, serial number,
     * firmware version, hardware version, and manufacturer. This information
     * is used for device management, version control, system diagnostics, and
     * device identification.
     *
     * @return Shared pointer to DeviceInfo structure containing:
     *         - model: Device model name or identifier
     *         - serial_number: Unique serial number for device identification
     *         - firmware_version: System firmware version string
     *         - hardware_version: Hardware version or revision number
     *         - manufacturer: Manufacturer name or company identifier
     *         Returns nullptr if device information retrieval fails.
     */
    virtual std::shared_ptr<DeviceInfo> get_device_information() = 0;

    /**
     * @brief Get one RGB image from the specified camera
     *
     * Retrieves one DMA FD camera frame and returns CPU-owned data in the
     * requested representation. JPEG uses the Jetson hardware encoder;
     * NV12/BGR/RGB return tightly packed CPU bytes with layout metadata in
     * RgbData.
     *
     * @param rgb_camera SensorType enumeration specifying which RGB camera to query
     * @param format JPEG, NV12, BGR, or RGB. Defaults to JPEG.
     * @param once True waits for one fresh frame of the requested format, then removes
     *             that one-shot format request. The per-camera DMA FD subscriber is
     *             started lazily and reused. False starts or reuses the persistent
     *             per-camera worker/cache and keeps the requested format active.
     * @return Shared pointer to RgbData containing image buffer, dimensions, layout,
     *         encoding, and timestamp. Returns nullptr if camera is not enabled or
     *         data retrieval fails.
     *
     * @note The camera sensor must be enabled during initialization via enable_sensor_set
     * @note DMA FD frame headers do not provide frame_id. For RGB DMA output,
     *       use get_camera_intrinsic() / camera_info for frame metadata.
     */
    virtual std::shared_ptr<RgbData> get_rgb_data(
        const SensorType rgb_camera,
        RgbOutputFormat format = RgbOutputFormat::JPEG,
        bool once = true) = 0;
    /**
     * @brief Subscribe to H.264 encoded video frames from the specified camera
     *
     * Registers a callback for one H.264 video stream. The user callback is executed
     * by an internal SDK dispatch thread and does not block the middleware receive callback thread.
     *
     * @param rgb_camera SensorType enumeration specifying which video camera to subscribe to
     * @param callback Callback function with signature void(const std::shared_ptr<EncodedVideoData>)
     * @return SensorStatus::SUCCESS on success, or an error status otherwise
     *
     * @note The camera sensor must be enabled during initialization via enable_sensor_set.
     * @note Keep the callback short and non-blocking. Long-running work may delay video frame delivery
     *       for this subscription.
     */
    virtual SensorStatus subscribe_video_data(
        const SensorType rgb_camera,
        std::function<void(const std::shared_ptr<EncodedVideoData>)> callback) = 0;
    /**
     * @brief Unsubscribe from H.264 encoded video frames for the specified camera
     *
     * Clears all user callbacks previously registered for this camera through subscribe_video_data.
     * Internal SDK reader, ring buffer, and dispatch thread remain running to avoid repeated
     * thread and resource creation/destruction.
     *
     * @param rgb_camera SensorType enumeration specifying which video camera to unsubscribe from
     * @return SensorStatus::SUCCESS on success, or an error status otherwise
     */
    virtual SensorStatus unsubscribe_video_data(const SensorType rgb_camera) = 0;
    /**
     * @brief Get latest depth image from specified arm depth camera
     *
     * Retrieves the most recent depth frame from an arm-mounted RGB-D depth stream.
     * Only SensorType::LEFT_ARM_DEPTH_CAMERA and SensorType::RIGHT_ARM_DEPTH_CAMERA
     * are valid for this method.
     *
     * @param depth_camera SensorType specifying which depth camera to query.
     * @return Shared pointer to DepthData containing encoded depth pixels, image
     *         dimensions, format, depth_scale, and timestamp. Returns nullptr if
     *         the sensor type is invalid, the sensor is not enabled, or data retrieval fails.
     *
     * @note The depth sensor must be included in enable_sensor_set during initialization.
     * @note This API is supported on G1 and S1 only. G3 does not provide arm-mounted depth cameras.
     * @note DepthData::data stores encoded depth image bytes. Decode the image first,
     *       then convert pixel values to meters with:
     *       depth_m = pixel_value / depth_scale.
     *       For example, depth_scale = 1000 means 1000 counts = 1.0 m.
     * @note A decoded pixel value of 0 usually indicates invalid or missing depth.
     * @see DepthData
     * @robot G1 S1
     */
    virtual std::shared_ptr<DepthData> get_depth_data(const SensorType depth_camera) = 0;
    /**
     * @brief Get latest infrared image from specified IR camera
     *
     * Retrieves the most recent infrared image captured by the specified IR camera.
     * Only available when ir_enabled is true in the camera parameter configuration.
     *
     * @param ir_camera SensorType enumeration specifying which IR camera to query
     *                  (LEFT_ARM_INFRA_CAMERA_1/2 or RIGHT_ARM_INFRA_CAMERA_1/2)
     * @return Shared pointer to IrData containing grayscale image buffer and timestamp.
     *         Returns nullptr if camera is not enabled, ir_enabled is false, or data
     *         retrieval fails.
     *
     * @note The IR sensor must be included in enable_sensor_set during initialization
     * @note ir_enabled must be true in the camera parameter topic for subscription to occur
     */
    virtual std::shared_ptr<IrData> get_ir_data(const SensorType ir_camera) = 0;
    /**
     * @brief Get synchronized observation aligned by camera timestamp.
     *
     * Uses the first sensor in @p cameras as anchor. The anchor camera takes latest frame,
     * other cameras are matched by nearest-neighbor timestamp in internal buffers.
     * If @p with_joint_state is true, returns nearest-neighbor joint state by same anchor timestamp.
     * RGB entries use CPU-owned NV12 data in synchronization mode so all camera
     * acquisition timestamps can remain aligned without waiting for serialized JPEG encoding.
     * The call waits up to one second for an initially empty requested camera history.
     * Each entry in returned joint_state->joint_state_vec includes joint_name so callers can
     * identify the semantic meaning of each joint sample.
     *
     * @note This is software timestamp alignment rather than hardware-trigger synchronization.
     *       The nearest-neighbor lookup does not enforce a maximum timestamp difference;
     *       callers with strict synchronization requirements should validate the returned timestamps.
     *
     * @param cameras Cameras to synchronize. cameras[0] is anchor and must be enabled.
     * @param with_joint_state Whether to include nearest-neighbor joint state.
     * @return Shared pointer to synchronized observation. On success:
     *         - rgb_data_map/depth_data_map contain timestamp-aligned camera frames
     *         - joint_state (when requested) contains nearest-neighbor joint sample with joint names
     *         Returns nullptr on invalid input or fetch failure.
     */
    virtual std::shared_ptr<SyncedObservation> get_synced_observation(
        const std::vector<SensorType>& cameras, bool with_joint_state = true) = 0;
    /**
     * @brief Get latest LiDAR point cloud data
     *
     * Retrieves the most recent 3D point cloud captured by the specified LiDAR sensor.
     * Each point typically contains (x, y, z) coordinates and optional intensity values.
     *
     * @param lidar SensorType enumeration specifying which LiDAR to query
     * @return Shared pointer to LidarData (PointCloud2 format) containing point cloud
     *         with coordinates in meters (m) relative to the LiDAR frame. Returns nullptr
     *         if LiDAR is not enabled or data retrieval fails.
     *
     * @note The LiDAR sensor must be enabled during initialization via enable_sensor_set
     */
    virtual std::shared_ptr<LidarData> get_lidar_data(const SensorType lidar) = 0;
    /**
     * @brief Get distance measurement from specified ultrasonic sensor
     *
     * Retrieves the latest distance measurement from one of the ultrasonic range sensors.
     * The robot typically has multiple ultrasonic sensors arranged around its perimeter.
     *
     * @param ultrasonic_type UltrasonicType enumeration specifying which ultrasonic
     *                        sensor to query (one of 8 directional sensors)
     * @return Shared pointer to UltrasonicData containing distance in meters (m),
     *         or nullptr if sensor is not enabled or data retrieval fails
     *
     * @note The ultrasonic sensor must be enabled during initialization via enable_sensor_set
     * @robot G1 G3
     */
    virtual std::shared_ptr<UltrasonicData> get_ultrasonic_data(const UltrasonicType ultrasonic_type) = 0;
    /**
     * @brief Get camera intrinsic parameters
     *
     * Retrieves the intrinsic parameters of the specified camera, including
     * focal lengths, principal points, and distortion coefficients, etc.
     *
     * @param camera SensorType enumeration specifying which camera to query
     * @return Shared pointer to CameraInfo containing:
     *         - focal_length_x, focal_length_y: Focal lengths in pixels
     *         - principal_point_x, principal_point_y: Principal point coordinates in pixels
     *         - distortion_coeffs: Vector of distortion coefficients (e.g., k1, k2, p1, p2, k3)
     *         - more camera-specific parameters as needed
     *         Returns nullptr if camera is not enabled or data retrieval fails.
     *
     * @note The camera sensor must be enabled during initialization via enable_sensor_set
     */
    virtual std::shared_ptr<CameraInfo> get_camera_intrinsic(const SensorType camera) = 0;
    /**
     * @brief Query coordinate frame transformation (TF)
     *
     * Queries the transformation between two coordinate frames in the robot's TF tree.
     * This is used for converting poses and positions between different reference frames
     * (e.g., from camera frame to base frame, from end-effector to world frame).
     *
     * @param target_frame Name of the target coordinate frame (frame to transform into)
     * @param source_frame Name of the source coordinate frame (frame to transform from)
     * @param timestamp_ns Desired transform timestamp in nanoseconds. Pass 0 to get
     *                     the most recent available transformation.
     * @param timeout_ms Maximum time to wait for the transform in milliseconds.
     *                   Default: 100 milliseconds.
     *
     * @return Pair containing:
     *         - Vector of 7 doubles representing the transform [x, y, z, qx, qy, qz, qw]
     *           where (x, y, z) is translation in meters and (qx, qy, qz, qw) is
     *           orientation quaternion
     *         - Timestamp in nanoseconds when the transform was valid
     *         Returns empty vector and timestamp 0 if retrieval fails or times out.
     */
    virtual std::pair<std::vector<double>, int64_t> get_transform(const std::string& target_frame,
                                                                  const std::string& source_frame,
                                                                  int64_t timestamp_ns = 0,
                                                                  int64_t timeout_ms = 100) = 0;
    /**
     * @brief Get all available coordinate frame names in the TF tree
     *
     * @return std::vector<std::string> All frame names
     */
    virtual std::vector<std::string> get_frame_names() = 0;
    /**
     * @brief Get sensor extrinsic parameters
     *
     * Retrieves the extrinsic parameters of the specified sensor, including
     * rotation and translation vectors relative to the robot's base frame.
     *
     * @param sensor_id SensorType enumeration specifying which sensor to query
     * @param reference_frame Name of the reference coordinate frame (frame to transform from). Default is "base_link".
     * @return Pair containing:
     *         - Vector of 7 doubles representing the transform [x, y, z, qx, qy, qz, qw]
     *           where (x, y, z) is translation in meters and (qx, qy, qz, qw) is
     *           orientation quaternion
     *         - Timestamp in nanoseconds when the transform was valid
     *         Returns empty vector and timestamp 0 if retrieval fails.
     *
     * @note The sensor must be enabled during initialization via enable_sensor_set
     */
    virtual std::pair<std::vector<double>, int64_t> get_sensor_extrinsic(
        const SensorType sensor_id, const std::string& reference_frame = "base_link") = 0;
    /**
     * @brief Get force/torque sensor data
     * @robot G1 S1 G3
     *
     * Retrieves the latest raw or calibrated measurements from the specified
     * force/torque sensor. The force and torque components can optionally be
     * rotated to the axes of a supported reference frame while retaining the
     * sensor measurement origin.
     *
     * @param sensor_type GalbotOneFoxtrotSensor enumeration specifying which force sensor to query
     * @param calibrated Whether to read calibrated contact wrench data. If false, reads the raw wrist force sensor.
     * @param ref_frame Frame whose axes are used to express the returned force and torque. An empty string keeps the
     *                  source-frame axes. Supported non-empty frames are "base_link", "torso_base_link", and the
     *                  matching "left_arm_end_effector_mount_link" or "right_arm_end_effector_mount_link".
     * @return Shared pointer to ForceData containing:
     *         - Force vector in Newtons (N): [fx, fy, fz]
     *         - Torque vector in Newton-meters (N·m): [tx, ty, tz]
     *         - Timestamp in nanoseconds
     *         Returns nullptr if the input is invalid, data retrieval fails, or the transform is unavailable.
     *
     * @note Raw force sensor data requires the sensor to be enabled during initialization via enable_sensor_set.
     * @note Coordinate conversion rotates force and torque only; it does not shift the wrench reference point or add
     *       a translation-induced moment.
     */
    virtual std::shared_ptr<ForceData> get_force_sensor_data(
        const GalbotOneFoxtrotSensor sensor_type, bool calibrated = false,
        const std::string& ref_frame = "") = 0;

    /**
     * @brief Start microphone streaming audio input
     *
     * @param callback Audio data callback function with signature: void(const std::shared_ptr<AudioData>)
     * @param chunk_size Audio data chunk size in bytes, default value 2560. Dynamic configuration not supported yet
     * @param use_raw_audio Whether to use raw audio, default false. Dynamic configuration not supported yet.
     *                      true means output raw audio directly, false means output processed audio
     * @return std::string Stream ID used to identify this audio input stream
     * @robot G1 G3
     */
    virtual std::string start_microphone_stream_input(std::function<void(const std::shared_ptr<AudioData>)> callback,
                                                      int chunk_size = 2560, bool use_raw_audio = false) = 0;
    /**
     * @brief Stop the specified microphone streaming audio input
     *
     * @param stream_id Audio input stream ID to stop. If empty string, stops all active streams
     * @robot G1 G3
     */
    virtual void stop_microphone_stream_input(const std::string& stream_id = "") = 0;
    /**
     * @brief Write PCM format audio data chunk to audio output stream for real-time playback
     *
     * @param audio_chunk Audio data chunk in PCM format (16000 Hz, 16-bit little-endian), single channel
     * @param stream_id Audio stream ID to distinguish different audio sources. Empty string means use default stream
     * @return bool Returns operation result. True means audio data has been successfully written and playback task
     * issued, False means write failed
     * @robot G1 G3
     */
    virtual bool write_audio_stream_output(const std::string& audio_chunk, const std::string& stream_id = "") = 0;
    /**
     * @brief Stop the specified audio output stream or all active audio output streams playback
     *
     * @param stream_id Audio output stream ID to stop. Empty string means stop all active audio output streams
     * @robot G1 G3
     */
    virtual void stop_audio_stream_output(const std::string& stream_id = "") = 0;
    /**
     * @brief Get current system global volume value
     *
     * @return float Returns current volume value, range 0.0 to 100.0
     * @robot G1 G3
     */
    virtual float get_volume() = 0;
    /**
     * @brief Set system global volume value
     *
     * @param volume Target volume value, range 0.0 to 100.0
     * @return bool Returns the volume setting result. True indicates the volume was set successfully, False indicates
     * the volume setting failed.
     * @robot G1 G3
     */
    virtual bool set_volume(float volume) = 0;

    /**
     * @brief Robot emergency stop
     * 
     * The robot's safety stop mechanism is immediately triggered via the software interface, 
     * interrupting all ongoing motion control commands and trajectory planning tasks, 
     * forcing the robot into a safe stop state. This operation takes effect immediately and 
     * has higher priority than all other control commands.
     * 
     * @return ControlStatus indicating success or failure of the reload operation
     * @robot G1 G3
     */
    virtual ControlStatus emergency_stop() = 0;
    /**
     * @brief Robot emergency stop recovery
     * 
     * After confirming that the safety conditions are met, deactivate the robot's software emergency stop, 
     * restoring the robot from a safe stop state to a controllable state and re-enabling all control-related functions. 
     * Before resuming, the user must ensure that all safety conditions are met.
     * 
     * @return ControlStatus indicating success or failure of the reload operation
     * @robot G1 G3
     */
    virtual ControlStatus resume_from_emergency_stop() = 0;

    /**
     * @brief Check if the robot control system is running
     *
     * Queries whether the robot control system is still active or if a shutdown
     * signal (e.g., SIGINT, SIGTERM) has been received.
     *
     * @return true if system is running normally
     * @return false if shutdown signal has been received and system is preparing to exit
     */
    virtual bool is_running() = 0;
    /**
     * @brief Request system shutdown
     *
     * Sends a shutdown signal to initiate graceful system shutdown. This triggers
     * registered exit callbacks and begins resource cleanup. Call this as the first
     * step of the shutdown sequence: request_shutdown() -> wait_for_shutdown()
     * -> destroy().
     */
    virtual void request_shutdown() = 0;
    /**
     * @brief Block until shutdown is complete
     *
     * Blocks the calling thread until all modules have finished shutting down gracefully.
     * This is the second step of the shutdown sequence, after request_shutdown() and
     * before destroy().
     *
     * @note This function will return when is_running() becomes false
     */
    virtual void wait_for_shutdown() = 0;
    /**
     * @brief Clean up system resources
     *
     * Performs final cleanup of robot control system resources including middleware
     * connections, sensor interfaces, and communication channels. This is the last
     * step of the shutdown sequence: request_shutdown() -> wait_for_shutdown()
     * -> destroy().
     *
     * This method should only be called once at program exit. After destroy(), the SDK
     * enters a terminal state and cannot be re-initialized in the same process.
     * To use the SDK again, exit the current process and launch a new one.
     */
    virtual void destroy() = 0;
    /**
     * @brief Register callback function for shutdown event
     *
     * Registers a user-defined callback function that will be invoked when a
     * shutdown signal is received. Multiple callbacks can be registered and
     * will be executed in registration order.
     *
     * @param exit_function Callback function with signature void() to be executed
     *                      during shutdown. Use this to perform application-specific
     *                      cleanup (e.g., saving data, stopping threads).
     *
     * @note Callbacks should complete quickly to avoid delaying shutdown
     */
    virtual void register_exit_callback(std::function<void()> exit_function) = 0;

    /**
     * @brief Get BMS (Battery Management System) information
     *
     * @return Shared pointer to BmsInfo containing battery information
     * @robot G1 G3
     */
    virtual std::shared_ptr<BmsInfo> get_bms_information() = 0;
    /**
     * @brief Get log information
     *
     * @return Shared pointer to LogInfo containing log information
     */
    virtual std::shared_ptr<LogInfo> get_log_information(uint64 timewindow_s, LogLevel log_level) = 0;

    /**
     * @brief Set one or more configuration fields for a service.
     *
     * If any field in @p fields is invalid, none of it takes effect and this
     * call returns ControlStatus::INVALID_INPUT.
     *
     * This call reports only a single aggregate status; it does not return
     * which field(s) failed or why. Check the log output when this call
     * does not return SUCCESS.
     *
     * @param service Which service's configuration to edit.
     * @param fields Fields to set, addressed by ConfigItem::key. For the full
     *        list of supported keys, their types, and valid value ranges per
     *        service, see the "Set Config Reference" page in the SDK
     *        documentation.
     * @return ControlStatus::SUCCESS if every field was validated and set;
     *         ControlStatus::INVALID_INPUT if field validation failed;
     *         ControlStatus::DATA_FETCH_FAILED / PUBLISH_FAIL on read/write
     *         failure.
     * @note A successful call only persists the configuration; it does not
     *       take effect until the device is restarted and the owning
     *       service reloads it.
     */
    virtual ControlStatus set_config(ConfigService service, const std::vector<ConfigItem>& fields) = 0;
    /**
     * @brief Read current or robot-default configuration values.
     *
     * By default, values are resolved as the current user-configured values;
     * this does not query the running service's in-memory state. When @p use_default
     * is true, only the robot's built-in default configuration is read. An empty
     * @p keys vector reads all registered fields applicable to the current robot.
     *
     * @param service Service whose persisted configuration should be read.
     * @param keys SDK-defined friendly keys; empty means all applicable fields.
     * @param fields Output key/value pairs. Cleared before reading. If individual
     *        fields fail, successfully read fields remain in this output and failed
     *        fields are omitted.
     * @param use_default When true, read the robot's built-in default values instead
     *        of current user-configured values. Defaults to false.
     * @return SUCCESS on complete read, INVALID_INPUT for an invalid key/service/model,
     *         INIT_FAILED before initialization, DATA_FETCH_FAILED on remote read failure,
     *         or FAULT for TOML parse/type errors. For field-level failures, the first
     *         failure in request order is returned after all fields have been attempted.
     */
    virtual ControlStatus get_config(ConfigService service, const std::vector<std::string>& keys,
                                     std::vector<ConfigItem>* fields, bool use_default = false) = 0;

   protected:
    /** Best-effort async GitHub release check once per process; failures are silent. */
    static void notify_new_sdk_version_if_any_async();
};

}  // namespace sdk
}  // namespace galbot
