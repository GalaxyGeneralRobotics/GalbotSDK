"""
Motion Plan Configuration module
"""
from __future__ import annotations
import collections.abc
import numpy
import typing
__all__: list[str] = ['COMM_DISCONNECTED', 'CYLINDER', 'CollisionCheckOption', 'ControlStatus', 'DATA_FETCH_FAILED', 'DepthData', 'EUCLIDEAN_DISTANCE', 'FAULT', 'ForceData', 'GalbotMotion', 'GalbotNavigation', 'GalbotOneFoxtrotSensor', 'GalbotRobot', 'GripperState', 'Header', 'IKSolverConfig', 'INIT_FAILED', 'INVALID_INPUT', 'IN_PROGRESS', 'ImuData', 'JOINT', 'JointCommand', 'JointGroup', 'JointState', 'JointStates', 'KinematicsBoundary', 'LINE', 'LidarData', 'LineTrajCheckPrimitive', 'MotionPlanConfig', 'MotionStatus', 'OdomData', 'POSE', 'PUBLISH_FAIL', 'Parameter', 'Point', 'PointField', 'PointFieldDataType', 'Pose', 'PoseState', 'PrimitiveType', 'Quaternion', 'RADIAN_DISTANCE', 'RANDOM_PROGRESSIVE_SEED', 'RANDOM_SEED', 'ROBOT_STATES', 'RgbData', 'RobotStates', 'RobotStatesType', 'STATUS_NUM', 'STOPPED_UNREACHED', 'SUCCESS', 'SUCTION_ACTION_STATE', 'SamplerConfig', 'SeedType', 'SensorType', 'StateCheckType', 'SuctionCupState', 'TIMEOUT', 'TIMEOUT_AND_EXACT_SOLUTION', 'TerminationConditionType', 'Timestamp', 'Trajectory', 'TrajectoryControlStatus', 'TrajectoryFeasibilityCheckOption', 'TrajectoryPlanConfig', 'TrajectoryPoint', 'UNSUPPORTED_FUNCRION', 'USER_DEFINED_SEED', 'UltrasonicData', 'UltrasonicType', 'Vector3', 'WBCException', 'check_motion_status', 'create_joint_state', 'create_parameter', 'create_pose_state']
class CollisionCheckOption:
    def __init__(self) -> None:
        ...
    def get_disable_env_collision_check(self) -> bool:
        ...
    def get_disable_self_collision_check(self) -> bool:
        ...
    def print(self) -> None:
        ...
    def set_disable_env_collision_check(self, arg0: bool) -> None:
        ...
    def set_disable_self_collision_check(self, arg0: bool) -> None:
        ...
class ControlStatus:
    """
    Members:
    
      SUCCESS : Execution successful
    
      TIMEOUT : Execution timeout
    
      FAULT : Fault occurred, cannot continue execution
    
      INVALID_INPUT : Input parameters do not meet requirements
    
      INIT_FAILED : Internal communication component creation failed
    
      IN_PROGRESS : Motion in progress but not reached target
    
      STOPPED_UNREACHED : Stopped but not reached target
    
      DATA_FETCH_FAILED : Data fetch failed
    
      PUBLISH_FAIL : Data sending failed
    
      COMM_DISCONNECTED : Connection failed
    """
    COMM_DISCONNECTED: typing.ClassVar[ControlStatus]  # value = <ControlStatus.COMM_DISCONNECTED: 9>
    DATA_FETCH_FAILED: typing.ClassVar[ControlStatus]  # value = <ControlStatus.DATA_FETCH_FAILED: 7>
    FAULT: typing.ClassVar[ControlStatus]  # value = <ControlStatus.FAULT: 2>
    INIT_FAILED: typing.ClassVar[ControlStatus]  # value = <ControlStatus.INIT_FAILED: 4>
    INVALID_INPUT: typing.ClassVar[ControlStatus]  # value = <ControlStatus.INVALID_INPUT: 3>
    IN_PROGRESS: typing.ClassVar[ControlStatus]  # value = <ControlStatus.IN_PROGRESS: 5>
    PUBLISH_FAIL: typing.ClassVar[ControlStatus]  # value = <ControlStatus.PUBLISH_FAIL: 8>
    STOPPED_UNREACHED: typing.ClassVar[ControlStatus]  # value = <ControlStatus.STOPPED_UNREACHED: 6>
    SUCCESS: typing.ClassVar[ControlStatus]  # value = <ControlStatus.SUCCESS: 0>
    TIMEOUT: typing.ClassVar[ControlStatus]  # value = <ControlStatus.TIMEOUT: 1>
    __members__: typing.ClassVar[dict[str, ControlStatus]]  # value = {'SUCCESS': <ControlStatus.SUCCESS: 0>, 'TIMEOUT': <ControlStatus.TIMEOUT: 1>, 'FAULT': <ControlStatus.FAULT: 2>, 'INVALID_INPUT': <ControlStatus.INVALID_INPUT: 3>, 'INIT_FAILED': <ControlStatus.INIT_FAILED: 4>, 'IN_PROGRESS': <ControlStatus.IN_PROGRESS: 5>, 'STOPPED_UNREACHED': <ControlStatus.STOPPED_UNREACHED: 6>, 'DATA_FETCH_FAILED': <ControlStatus.DATA_FETCH_FAILED: 7>, 'PUBLISH_FAIL': <ControlStatus.PUBLISH_FAIL: 8>, 'COMM_DISCONNECTED': <ControlStatus.COMM_DISCONNECTED: 9>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class DepthData:
    """
    Depth image data
    """
    def __init__(self) -> None:
        ...
    @property
    def data(self) -> list[int]:
        """
        Compressed depth data
        """
    @data.setter
    def data(self, arg0: collections.abc.Sequence[typing.SupportsInt]) -> None:
        ...
    @property
    def depth_scale(self) -> int:
        """
        Depth scale/quantization factor
        """
    @depth_scale.setter
    def depth_scale(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def format(self) -> str:
        """
        Image format
        """
    @format.setter
    def format(self, arg0: str) -> None:
        ...
    @property
    def header(self) -> Header:
        """
        Message header
        """
    @header.setter
    def header(self, arg0: Header) -> None:
        ...
    @property
    def height(self) -> int:
        """
        Image height
        """
    @height.setter
    def height(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def width(self) -> int:
        """
        Image width
        """
    @width.setter
    def width(self, arg0: typing.SupportsInt) -> None:
        ...
class ForceData:
    """
    Force sensor data
    """
    def __init__(self) -> None:
        ...
    @property
    def force(self) -> Vector3:
        """
        Force vector Vector3
        """
    @force.setter
    def force(self, arg0: Vector3) -> None:
        ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def torque(self) -> Vector3:
        """
        Torque vector Vector3
        """
    @torque.setter
    def torque(self, arg0: Vector3) -> None:
        ...
class GalbotMotion:
    @staticmethod
    def get_instance() -> GalbotMotion:
        """
                            Get the singleton instance of GalbotMotion.
                                
                            Parameters:
                                None
        
                            Returns:
                                GalbotMotion: The singleton instance of GalbotMotion.
        """
    def __repr__(self) -> str:
        ...
    def add_obstacle(self, obstacle_id: str, obstacle_type: str, pose: collections.abc.Sequence[typing.SupportsFloat], scale: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"] = [0.0, 0.0, 0.0], key: str = '', target_frame: str = 'world', ee_frame: str = 'ee_base', reference_joint_positions: collections.abc.Sequence[typing.SupportsFloat] = [], reference_base_pose: collections.abc.Sequence[typing.SupportsFloat] = [], ignore_collision_link_names: collections.abc.Sequence[str] = [], safe_margin: typing.SupportsFloat = 0.0, resolution: typing.SupportsFloat = 0.01) -> MotionStatus:
        """
                        Add an obstacle to the robot's collision detection system.
                    
                        Parameters:
                            obstacle_id (str): Unique ID for the obstacle (cannot be duplicated)
                            obstacle_type (str): Obstacle type. Options: box / sphere / cylinder / mesh / point_cloud / depth_image
                            pose (list[float]): Position and orientation of the obstacle. Length 7: [x, y, z, qx, qy, qz, qw]
                            scale (tuple[float]): Geometric size of the obstacle
                                    `box: length / width / height (l / w / h)` / 
                                    `sphere: radius / - / -` /
                                    `cylinder: radius / height / -`
                            key (str): key for the obstacle. 
                                    `mesh / point_cloud: file path` /
                                    `depth_image: camera type (front_head / left_arm / right_arm)`
                            target_frame (str): Target coordinate frame. Options: world / base_link / motion chain name
                            ee_frame (str): End-effector coordinate frame. Only effective when target_frame is a motion chain name
                            reference_joint_positions (list[float]): Robot joint state when loading obstacle. If empty, current joint state is used
                            reference_base_pose (list[float]): Robot base pose in map coordinate frame. If empty, current base pose is used
                            ignore_collision_link_names (list[str]): List of robot link names to ignore in collision detection
                            safe_margin (float): Safe distance to obstacle. Collision is detected when obstacle distance is less than this value
                            resolution (float): Loading precision for some obstacle types. Defaults to 0.01
        
        
        
                        Returns:
                            MotionStatus: Result of adding the obstacle
        """
    def attach_target_object(self, obstacle_id: str, obstacle_type: str, pose: collections.abc.Sequence[typing.SupportsFloat], scale: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"] = [0.0, 0.0, 0.0], key: str = '', target_frame: str = 'world', ee_frame: str = 'ee_base', reference_joint_positions: collections.abc.Sequence[typing.SupportsFloat] = [], reference_base_pose: collections.abc.Sequence[typing.SupportsFloat] = [], ignore_collision_link_names: collections.abc.Sequence[str] = [], safe_margin: typing.SupportsFloat = 0.0, resolution: typing.SupportsFloat = 0.01) -> MotionStatus:
        """
                     Add an obstacle to the robot's collision detection system.
                     
                    Notes:
                        - GalbotMotion currently does NOT provide real-time obstacle perception / automatic environment updates.
                        - Obstacle inputs here are treated as manual environment setup for collision checking.
                        - For obstacle_type == "point_cloud", `key` is typically a point cloud file path provided by the user.
                        - For obstacle_type == "depth_image", the camera type selects a depth source captured/loaded for the collision world;
                        it is not a continuous real-time perception stream for motion planning.
        
                     Parameters:
                         obstacle_id (str): Unique ID for the obstacle (cannot repeat)
                         obstacle_type (str): Type of obstacle (box/sphere/cylinder/mesh/point_cloud/depth_image)
                         pose (list[float]): Position and orientation of the obstacle (length 7: xyz+quat)
                         scale (tuple[float]): Geometry size (box: l/w/h; sphere: r/-/-; cylinder: r/h/-)
                         key (str): File path (mesh/point_cloud) or camera type (depth_image: front_head/left_arm/right_arm)
                         target_frame (str): Target coordinate frame (world/base_link/chain name)
                         ee_frame (str): End-effector frame (only valid if target_frame is a chain)
                         reference_joint_positions (list[float]): Robot joint state when loading obstacle (current if empty)
                         reference_base_pose (list[float]): Robot base pose in map frame (current if empty)
                         ignore_collision_link_names (list[str]): Links to ignore collision with
                         safe_margin (float): Safe distance (collision if < this value)
                         resolution (float): Loading precision for some obstacle types
                     
                     Returns:
                         MotionStatus: Result of adding obstacle
        """
    def attach_tool(self, chain: str, tool: str) -> MotionStatus:
        """
                            Attach a tool to the specified robot motion chain (left_arm / right_arm).
                                
                            Parameters:
                                chain (str): The robot motion chain.
                                tool (str): The tool to attach.
                                params (dict, optional): Additional parameters for the tool attachment. Defaults to default_param.
                                
                            Returns:
                                bool: True if the tool attachment is successful, False otherwise.
        """
    def check_collision(self, start: collections.abc.Sequence[RobotStates], enable_collision_check: bool = True, params: Parameter = ...) -> tuple[MotionStatus, list[bool]]:
        """
                            Check collision between robot and environment.
                                
                            Parameters:
                                start (RobotStates): The robot states.
                                enable_collision_check (bool, optional): Whether to enable collision checking. Defaults to true.
                                params (dict, optional): Additional parameters for the collision checking. Defaults to default_param.
        
                            Notes:
                                - GalbotMotion currently does NOT provide real-time obstacle perception / automatic environment updates.
                                - The environment for collision checking is the set of obstacles you manually load via add_obstacle()
                                  and attach_target_object().
                                
                            Returns:
                                bool: True if there is a collision, False otherwise.
        """
    def clear_obstacle(self) -> MotionStatus:
        """
        Remove all loaded obstacles
        """
    def detach_target_object(self, obstacle_id: str) -> MotionStatus:
        """
        Remove the specified obstacle by ID
        """
    def detach_tool(self, chain: str) -> MotionStatus:
        """
                            Detach a tool from the specified robot motion chain (left_arm / right_arm).
                                
                            Parameters:
                                chain (str): The robot motion chain.
                                params (dict, optional): Additional parameters for the tool detachment. Defaults to default_param.
                                
                            Returns:
                                bool: True if the tool detachment is successful, False otherwise.
        """
    def forward_kinematics(self, target_frame: str, reference_frame: str = 'base_link', joint_state: collections.abc.Mapping[str, collections.abc.Sequence[typing.SupportsFloat]] = {}, params: Parameter = ...) -> tuple[MotionStatus, list[float]]:
        """
                            Perform forward kinematics to compute the pose of a target frame.
                                
                            Parameters:
                                target_frame (str): The name of the target frame.
                                reference_frame (str, optional): The name of the reference frame. Defaults to "base_link".
                                joint_state (dict, optional): A dictionary mapping joint names to their positions. Defaults to an empty dictionary.
                                params (dict, optional): Additional parameters for the forward kinematics. Defaults to default_param.
                                
                            Returns:
                                Pose: The computed pose of the target frame.
        """
    def forward_kinematics_by_state(self, target_frame: str, reference_robot_states: RobotStates = None, reference_frame: str = 'base_link', params: Parameter = ...) -> tuple[MotionStatus, list[float]]:
        """
                            Perform forward kinematics to compute the pose of a target frame.
                                
                            Parameters:
                                target_frame (str): The name of the target frame.
                                reference_robot_states (RobotStates, optional): The reference robot states. Defaults to nullptr.
                                reference_frame (str, optional): The name of the reference frame. Defaults to "base_link".
                                params (dict, optional): Additional parameters for the forward kinematics. Defaults to default_param.
                                
                            Returns:
                                Pose: The computed pose of the target frame.
        """
    def get_built_obstacles_list(self) -> list[str]:
        """
        Get the list of built obstacles.
        """
    def get_end_effector_pose(self, end_effector_frame: str, reference_frame: str = 'base_link') -> tuple[MotionStatus, list[float]]:
        """
                            Get the pose of a specified end-effector frame.
                                
                            Parameters:
                                end_effector_frame (str): The name of the end-effector frame.
                                reference_frame (str, optional): The name of the reference frame. Defaults to "base_link".
                                
                            Returns:
                                Pose: The computed pose of the end-effector frame.
        """
    def get_end_effector_pose_on_chain(self, chain_name: str, frame_id: str = 'EndEffector', reference_frame: str = 'base_link') -> tuple[MotionStatus, list[float]]:
        """
                            Get the pose of a specified end-effector frame on a given chain.
                                
                            Parameters:
                                chain_name (str): The name of the chain.
                                frame_id (str, optional): The name of the end-effector frame. Defaults to "EndEffector".
                                reference_frame (str, optional): The name of the reference frame. Defaults to "base_link".
                                
                            Returns:
                                Pose: The computed pose of the end-effector frame on the specified chain.
        """
    def get_motion_plan_config(self) -> tuple[MotionStatus, MotionPlanConfig]:
        """
        get motion config
        """
    def get_supported_chains(self) -> set[str]:
        """
        Get the list of supported links.
        """
    def get_supported_ee_frames(self) -> set[str]:
        """
        Get the list of supported end-effector frames.
        """
    def get_supported_frames(self) -> set[str]:
        """
        Get the list of supported reference frames.
        """
    def get_supported_links(self) -> set[str]:
        """
        Initialize system interface dependencies.
        """
     
    def get_supported_obstacle_types(self) -> set[str]:
        """
        Get the list of supported obstacle types.
        """
     
    def get_supported_obstacle_types(self) -> set[str]:
        """
        Get the list of supported obstacle types.
        """
    def get_supported_tool_list(self) -> set[str]:
        """
        Get the list of supported tools.
        """
    def init(self) -> bool:
        """
                            Initialize the system interface dependencies.
                                
                            Parameters:
                                None
        
                            Returns:
                                bool: True if initialization is successful, False otherwise.
        """
    def inverse_kinematics(self, target_pose: collections.abc.Sequence[typing.SupportsFloat], chain_names: collections.abc.Sequence[str], target_frame: str = 'EndEffector', reference_frame: str = 'base_link', initial_joint_positions: collections.abc.Mapping[str, collections.abc.Sequence[typing.SupportsFloat]] = {}, enable_collision_check: bool = True, params: Parameter = ...) -> tuple[MotionStatus, dict[str, list[float]]]:
        """
                            Perform inverse kinematics to compute the joint positions for a target pose.
                                
                            Parameters:
                                target_pose (Pose): The target pose.
                                chain_names (list of str): The list of chain names to consider.
                                target_frame (str, optional): The name of the target frame. Defaults to "EndEffector".
                                reference_frame (str, optional): The name of the reference frame. Defaults to "base_link".
                                initial_joint_positions (dict, optional): A dictionary mapping joint names to their initial positions. Defaults to an empty dictionary.
                                enable_collision_check (bool, optional): Whether to enable collision checking. Defaults to true.
                                params (dict, optional): Additional parameters for the inverse kinematics. Defaults to default_param.
                                
                            Returns:
                                dict: A dictionary mapping joint names to their computed positions.
        """
    def inverse_kinematics_by_state(self, target_pose: collections.abc.Sequence[typing.SupportsFloat], chain_names: collections.abc.Sequence[str], target_frame: str = 'EndEffector', reference_frame: str = 'base_link', reference_robot_states: RobotStates = None, enable_collision_check: bool = True, params: Parameter = ...) -> tuple[MotionStatus, dict[str, list[float]]]:
        """
                            Perform inverse kinematics to compute the joint positions for a target pose.
                                
                            Parameters:
                                target_pose (Pose): The target pose.
                                chain_names (list of str): The list of chain names to consider.
                                target_frame (str, optional): The name of the target frame. Defaults to "EndEffector".
                                reference_frame (str, optional): The name of the reference frame. Defaults to "base_link".
                                reference_robot_states (RobotStates, optional): The reference robot states. Defaults to nullptr.
                                enable_collision_check (bool, optional): Whether to enable collision checking. Defaults to true.
                                params (dict, optional): Additional parameters for the inverse kinematics. Defaults to default_param.
                                
                            Returns:
                                dict: A dictionary mapping joint names to their computed positions.
        """
    def motion_plan(self, target: RobotStates, start: RobotStates = None, reference_robot_states: RobotStates = None, enable_collision_check: bool = True, params: Parameter = ...) -> tuple[MotionStatus, dict[str, list[list[float]]]]:
        """
                            Plan a motion to a single waypoint.
                                
                            Parameters:
                                target (Pose): The target pose.
                                start (RobotStates, optional): The initial robot states. Defaults to nullptr.
                                reference_robot_states (RobotStates, optional): The reference robot states. Defaults to nullptr.
                                enable_collision_check (bool, optional): Whether to enable collision checking. Defaults to true.
                                params (dict, optional): Additional parameters for the motion planning. Defaults to default_param.
        
                            Notes:
                                - GalbotMotion currently does NOT provide real-time obstacle perception / automatic environment updates.
                                - If collision checking is enabled, collisions are checked against self-collision and obstacles that you
                                    manually load via add_obstacle()/attach_target_object().
                                - In contrast, the Navigation module (GalbotNavigation) may use real-time perception/avoidance depending
                                    on deployment; this is not currently integrated into GalbotMotion.
                                
                            Returns:
                                bool: True if the motion planning is successful, False otherwise.
        """
     
    def motion_plan_multi_waypoints(self, target: RobotStates, waypoint_poses: collections.abc.Sequence[collections.abc.Sequence[typing.SupportsFloat]], start: RobotStates = None, reference_robot_states: RobotStates = None, enable_collision_check: bool = True, params: Parameter = ...) -> tuple[MotionStatus, dict[str, list[list[float]]]]:
        """
                            Plan a motion to multiple waypoints.
                                
                            Parameters:
                                target (Pose): The target pose.
                                waypoint_poses (list of list of float): The waypoint poses.
                                start (RobotStates, optional): The initial robot states. Defaults to nullptr.
                                reference_robot_states (RobotStates, optional): The reference robot states. Defaults to nullptr.
                                enable_collision_check (bool, optional): Whether to enable collision checking. Defaults to true.
                                params (dict, optional): Additional parameters for the motion planning. Defaults to default_param.
        
                            Notes:
                                - GalbotMotion currently does NOT provide real-time obstacle perception / automatic environment updates.
                                - If collision checking is enabled, collisions are checked against self-collision and obstacles that you
                                    manually load via add_obstacle()/attach_target_object().
                                - In contrast, the Navigation module (GalbotNavigation) may use real-time perception/avoidance depending
                                    on deployment; this is not currently integrated into GalbotMotion.
                                
                            Returns:
                                bool: True if the motion planning is successful, False otherwise.
        """
     
    def motion_plan_multi_waypoints(self, targets: collections.abc.Mapping[RobotStates, collections.abc.Sequence[collections.abc.Sequence[typing.SupportsFloat]]], start: collections.abc.Sequence[RobotStates] = [], reference_robot_states: RobotStates = None, enable_collision_check: bool = True, params: Parameter = ...) -> tuple[MotionStatus, dict[str, list[list[float]]]]:
        """
                            Plan a motion to multiple waypoints.
                                
                            Parameters:
                                targets (dict of Pose): The target poses.
                                waypoint_poses (list of list of float): The waypoint poses.
                                start (RobotStates, optional): The initial robot states. Defaults to nullptr.
                                reference_robot_states (RobotStates, optional): The reference robot states. Defaults to nullptr.
                                enable_collision_check (bool, optional): Whether to enable collision checking. Defaults to true.
                                params (dict, optional): Additional parameters for the motion planning. Defaults to default_param.
        
                            Notes:
                                - GalbotMotion currently does NOT provide real-time obstacle perception / automatic environment updates.
                                - If collision checking is enabled, collisions are checked against self-collision and obstacles that you
                                    manually load via add_obstacle()/attach_target_object().
                                - In contrast, the Navigation module (GalbotNavigation) may use real-time perception/avoidance depending
                                    on deployment; this is not currently integrated into GalbotMotion.
                                
                            Returns:
                                bool: True if the motion planning is successful, False otherwise.
        """
    def remove_obstacle(self, obstacle_id: str) -> MotionStatus:
        """
        Remove an obstacle by its ID
        """
    def set_end_effector_pose(self, target_pose: collections.abc.Sequence[typing.SupportsFloat], end_effector_frame: str, reference_frame: str = 'base_link', reference_robot_states: RobotStates = None, enable_collision_check: bool = True, is_blocking: bool = True, timeout: typing.SupportsFloat = -1.0, params: Parameter = ...) -> MotionStatus:
        """
                            Set the pose of a specified end-effector frame.
                                
                            Parameters:
                                target_pose (Pose): The target pose.
                                end_effector_frame (str): The name of the end-effector frame.
                                reference_frame (str, optional): The name of the reference frame. Defaults to "base_link".
                                reference_robot_states (RobotStates, optional): The reference robot states. Defaults to nullptr.
                                enable_collision_check (bool, optional): Whether to enable collision checking. Defaults to true.
                                is_blocking (bool, optional): Whether to block until the motion is completed. Defaults to true.
                                timeout (float, optional): The maximum time to wait for the motion to complete. Defaults to -1.0.
                                params (dict, optional): Additional parameters for the motion planning. Defaults to default_param.
                                
                            Returns:
                                bool: True if the motion planning is successful, False otherwise.
        """
    def set_motion_plan_config(self, config: MotionPlanConfig) -> MotionStatus:
        """
        set motion config
        """
class GalbotNavigation:
    @staticmethod
    def get_instance() -> GalbotNavigation:
        """
                            Get the GalbotNavigation singleton instance.
        
                            Parameters:
                                None
                            
                            Returns:
                                GalbotNavigation: The GalbotNavigation singleton instance.
        """
    def check_goal_arrival(self) -> bool:
        """
                    Check whether the robot has reached the goal pose.
        
                    Parameters:
                        None
        
                    Returns:
                        bool: Whether the robot has reached the goal pose.
        """
    def check_path_reachability(self, goal_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64], start_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64]) -> bool:
        """
                    Check whether there is a reachable path from the start pose to the goal pose in the map frame.
        
                    Parameters:
                        goal_pose (array): The goal pose [x, y, z, qx, qy, qz, qw], map frame.
                        start_pose (array): The start pose [x, y, z, qx, qy, qz, qw], map frame.
        
                    Returns:
                        bool: Whether there is a reachable path from the start pose to the goal pose.
        """
    def get_current_pose(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
                            Get the current pose of the robot.
        
                            Parameters:
                                None
                            
                            Returns:
                                array: [x, y, z, qx, qy, qz, qw], map frame
        """
    def init(self) -> bool:
        """
                                 Initialize the GalbotNavigation object.
        
                            Parameters:
                                None
                            
                            Returns:
                                bool: True if initialization is successful, False otherwise.
        """
    def is_localized(self) -> bool:
        """
                            Check if the robot is localized.
        
                            Parameters:
                                None
                            
                            Returns:
                                bool: True if the robot is localized, False otherwise.
        """
    def move_straight_to(self, goal_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64], is_blocking: bool = True, timeout: typing.SupportsFloat = 8) -> tuple:
        """
                    Navigate to the given pose in odom frame, disable collision check and enable omni-directional planning.
        
                    Parameters:
                        goal_pose (array): The goal pose [x, y, z, qx, qy, qz, qw], base_link frame.
                        is_blocking (bool): Whether to block the call until the goal is reached, default false.
                        timeout (int): The timeout in seconds, default 8.
        
                    Returns:
                        tuple: (success: bool, status_string: str)
                            - success: Whether the navigation is successful.
                            - status_string: Status string.
        """
    def navigate_to_goal(self, goal_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64], enable_collision_check: bool = True, is_blocking: bool = False, timeout: typing.SupportsFloat = 8, omni_plan: bool = True) -> tuple:
        """
                            Plan and navigate to the given pose.
        
                            Parameters:
                                goal_pose (array): The goal pose [x, y, z, qx, qy, qz, qw], map frame.
                                collision_check (bool): Whether to enable collision check, default true.
                                is_blocking (bool): Whether to block the call until the goal is reached, default false.
                                timeout (int): The timeout in seconds, default 8.
                                omni_plan (bool): Whether to enable omni-directional planning, default true.
                                    when configure omni_plan option, the parameter `forbidden_modify_omni_item` in
                                    /data/galbot/config/aphropm_pns_config/local_plan_config.toml must be set to false.
                            
                            Returns:
                                tuple: (success: bool, status_string: str)
                                    - success: Whether the navigation is successful.
                                    - status_string: Status string.
        """
    def relocalize(self, init_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64]) -> tuple:
        """
                                 Relocalize the robot to the given pose.
        
                            Parameters:
                                init_pose (array): The initial pose [x, y, z, qx, qy, qz, qw].
                            
                            Returns:
                                tuple: (success: bool, status_string: str)
                                    - success:是否重定位成功
                                    - status_string:状态描述字符串
        """
    def stop_navigation(self) -> tuple:
        """
                    Stop the current navigation task.
        
                    Parameters: 
                        None
        
                    Returns:
                        tuple: (success: bool, status_string: str)
                            - success: Whether the navigation is successfully stopped.
                            - status_string: Status string.
        """
class GalbotOneFoxtrotSensor:
    """
    Members:
    
      LEFT_WRIST_FORCE : Left wrist force sensor
    
      RIGHT_WRIST_FORCE : Right wrist force sensor
    """
    LEFT_WRIST_FORCE: typing.ClassVar[GalbotOneFoxtrotSensor]  # value = <GalbotOneFoxtrotSensor.LEFT_WRIST_FORCE: 0>
    RIGHT_WRIST_FORCE: typing.ClassVar[GalbotOneFoxtrotSensor]  # value = <GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE: 1>
    __members__: typing.ClassVar[dict[str, GalbotOneFoxtrotSensor]]  # value = {'LEFT_WRIST_FORCE': <GalbotOneFoxtrotSensor.LEFT_WRIST_FORCE: 0>, 'RIGHT_WRIST_FORCE': <GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE: 1>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class GalbotRobot:
    @staticmethod
    def get_instance() -> GalbotRobot:
        """
                    Get the singleton instance of GalbotRobot.
                        
                    Parameters:
                        None
                        
                    Returns:
                        GalbotRobot: The singleton instance of GalbotRobot.
        """
     
    def check_trajectory_execution_status(self, joint_groups: collections.abc.Sequence[str] = []) -> list[TrajectoryControlStatus]:
        ...
     
    def check_trajectory_execution_status(self, joint_groups: collections.abc.Sequence[JointGroup] = []) -> list[TrajectoryControlStatus]:
        """
                    Get trajectory execution status for specified joint groups.
              
                    Parameters:
                        joint_groups (List[str] or List[JointGroup]): Joint groups to query, can use strings or JointGroup enums (optional).
                    
                    Returns:
                        List[TrajectoryControlStatus]: List of trajectory execution statuses.
        """
    def destroy(self) -> None:
        """
                    Clean up system and middleware resources.
              
                    Parameters:
                        None
                    
                    Returns:
                        None
        """
    def execute_joint_trajectory(self, trajectory: Trajectory, is_blocking: bool = True) -> ControlStatus:
        """
                    Execute trajectory data.
              
                    Parameters:
                        trajectory (Trajectory): Trajectory data to execute.
                        is_blocking (bool): Whether to block until trajectory execution completes (optional, default: True).
                    
                    Returns:
                        ControlStatus: Trajectory execution/sending result.
        """
    def get_depth_data(self, camera_id: SensorType) -> dict:
        """
                    Get latest depth image data from specified camera.
              
                    Parameters:
                        camera_id (SensorType): Depth camera sensor ID to query.
                    
                    Returns:
                        dict: Dictionary containing the following keys:
                            - 'header': Message header with timestamp and frame information
                            - 'format': Image format, e.g., 'depth16' or other
                            - 'depth_scale': Depth scaling factor
                            - 'height': Image height in pixels
                            - 'width': Image width in pixels
                            - 'data': Compressed depth image binary data (bytes).
                        Returns empty dictionary on failure.
        """
    def get_force_sensor_data(self, sensor_type: GalbotOneFoxtrotSensor) -> dict:
        """
                    Get data from specified force sensor.
                
                    Parameters:
                        sensor_type (GalbotOneFoxtrotSensor): Force sensor enum to query.
                    
                    Returns:
                        dict: Dictionary containing the following keys:
                            - 'timestamp_ns': Timestamp in nanoseconds
                            - 'force': Force vector dictionary with 'x', 'y', 'z' keys
                            - 'torque': Torque vector dictionary with 'x', 'y', 'z' keys
                        
                        Returns empty dictionary on failure.
        """
    def get_gripper_state(self, end_effector: JointGroup) -> GripperState:
        """
                    Get gripper state.
              
                    Parameters:
                        end_effector (JointGroup): Gripper enum value.
                    
                    Returns:
                        GripperState: Gripper state information.
        """
    def get_imu_data(self, sensor_id: SensorType) -> dict:
        """
                    Get data from specified IMU sensor.
              
                    Parameters:
                        sensor_id (SensorType): IMU sensor enum to query.
                    
                    Returns:
                        dict: Dictionary containing the following keys:
                            - 'timestamp_ns': Timestamp in nanoseconds
                            - 'accel': Acceleration Vector3 {'x': float, 'y': float, 'z': float}
                            - 'gyro': Gyroscope Vector3 {'x': float, 'y': float, 'z': float}
                            - 'magnet': Magnetometer Vector3 {'x': float, 'y': float, 'z': float}
                        
                        Returns empty dictionary on failure.
        """
    def get_joint_group_names(self) -> list[str]:
        """
                    Get available joint group names for the robot.
              
                    Parameters:
                        None
                    
                    Returns:
                        List[str]: Array of available joint group names, returns empty list on failure.
        """
     
    def get_joint_names(self, only_active_joint: bool = True, joint_groups: collections.abc.Sequence[str] = []) -> list[str]:
        ...
     
    def get_joint_names(self, only_active_joint: bool = True, joint_groups: collections.abc.Sequence[JointGroup] = []) -> list[str]:
        """
                    Get robot joint names.
              
                    Parameters:
                        only_active_joint (bool): Whether to only get active joints (optional, default: True).
                        joint_groups (List[str] or List[JointGroup]): Joint groups, can use strings or JointGroup enums (optional).
                    
                    Returns:
                        List[str]: Array of corresponding joint names.
        """
     
    def get_joint_positions(self, joint_groups: collections.abc.Sequence[str], joint_names: collections.abc.Sequence[str] = []) -> list[float]:
        ...
     
    def get_joint_positions(self, joint_groups: collections.abc.Sequence[JointGroup], joint_names: collections.abc.Sequence[str] = []) -> list[float]:
        """
                    Get joint positions.
              
                    Parameters:
                        joint_groups (List[str] or List[JointGroup]): Joint groups to query, can use strings or JointGroup enums.
                        joint_names (List[str]): Specific joint names, takes priority over joint_groups (optional).
                    
                    Returns:
                        List[float]: Array of corresponding joint angles in radians.
        """
     
    def get_joint_states(self, joint_group_vec: collections.abc.Sequence[str], joint_names_vec: collections.abc.Sequence[str] = []) -> list[JointState]:
        ...
     
    def get_joint_states(self, joint_group_vec: collections.abc.Sequence[JointGroup], joint_names_vec: collections.abc.Sequence[str] = []) -> list[JointState]:
        """
                    Get real-time joint states.
                    
                    Parameters:
                        joint_group_vec (List[str] or List[JointGroup]): Joint groups to query, can use strings or JointGroup enums (optional).
                        joint_names_vec (List[str]): Specific joint names, takes priority over joint_group_vec (optional).
                    
                    Returns:
                        List[JointState]: Real-time state data for corresponding joints.
        """
    def get_lidar_data(self, sensor_id: SensorType) -> dict:
        """
                    Get latest point cloud data from specified LiDAR sensor.
                    
                    Parameters:
                        sensor_id (SensorType): LiDAR sensor enum to query.
                    
                    Returns:
                        dict: Dictionary containing point cloud data fields and binary point data.
                            Returns empty dictionary on failure.
        """
    def get_odom(self) -> dict:
        """
                    Get odometry information.
                    
                    Parameters:
                        None
                    
                    Returns:
                        dict: Dictionary containing the following keys:
                            - 'timestamp': Timestamp in seconds
                            - 'position': Position array [x, y, z] in meters
                            - 'orientation': Quaternion array [x, y, z, w]
                        
                        Returns empty dictionary on failure.
        """
    def get_rgb_data(self, camera_id: SensorType) -> dict:
        """
                    Get latest RGB image data from specified camera.
              
                    Parameters:
                        camera_id (SensorType): Camera sensor ID to query.
                    
                    Returns:
                        dict: Dictionary containing the following keys:
                            - 'header': Message header with timestamp and frame information
                            - 'format': Image format, e.g., 'jpeg' or 'png'
                            - 'data': Compressed image binary data (bytes)
                        
                        Returns empty dictionary on failure.
        """
    def get_suction_cup_state(self, end_effector: JointGroup) -> SuctionCupState:
        """
                    Get suction cup state.
              
                    Parameters:
                        end_effector (JointGroup): Suction cup enum value.
                    
                    Returns:
                        SuctionCupState: Suction cup state information.
        """
    def get_transform(self, target_frame: str, source_frame: str, timestamp_ns: typing.SupportsInt = 0, timeout_ms: typing.SupportsInt = 100) -> tuple:
        """
                    Query coordinate frame transform (TF).
              
                    Parameters:
                        target_frame (str): Target coordinate frame.
                        source_frame (str): Source coordinate frame.
                        timestamp_ns (int): Desired transform timestamp in nanoseconds, 0 for latest (optional, default: 0).
                        timeout_ms (int): Query timeout in milliseconds (optional, default: 100).
                    
                    Returns:
                        tuple(List[float], int): Transform matrix list and timestamp. Returns empty list on failure.
        """
    def get_ultrasonic_data(self, ultrasonic_type: UltrasonicType) -> dict:
        """
                    Get data from specified ultrasonic sensor.
                     
                    Parameters:
                        ultrasonic_type (UltrasonicType): Ultrasonic sensor enum to query.
                    
                    Returns:
                        dict: Dictionary containing the following keys:
                            - 'timestamp_ns': Timestamp in nanoseconds
                            - 'distance': Distance value in meters
                        
                        Returns empty dictionary on failure.
        """
    def init(self, enable_sensor_set: collections.abc.Set[SensorType] = ...) -> bool:
        """
                    Initialize the singleton object.
                    
                    Parameters:
                        enable_sensor_set (set[SensorType]): Set of sensors to enable.
                    
                    Returns:
                        bool: True if initialization succeeded, False otherwise.
        """
    def is_running(self) -> bool:
        """
                    Check if the system is running.
            
                    Parameters:
                        None
                    
                    Returns:
                        bool: True if system is running, False if shutdown signal captured and preparing to shutdown.
        """
    def request_shutdown(self) -> None:
        """
                    Send SIGINT signal to request shutdown.
              
                    Parameters:
                        None
                    
                    Returns:
                        None
        """
    def set_base_velocity(self, linear_velocity: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"], angular_velocity: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"]) -> ControlStatus:
        """
                    Set base velocity command.
              
                    Parameters:
                        linear_velocity (List[float]): Linear velocity command [vx, vy, vz] in m/s.
                        angular_velocity (List[float]): Angular velocity command [wx, wy, wz] in rad/s.
                    
                    Returns:
                        ControlStatus: Command sending result.
        """
    def set_gripper_command(self, end_effector: JointGroup, width_m: typing.SupportsFloat, velocity_mps: typing.SupportsFloat = 0.03, effort: typing.SupportsFloat = 30, is_blocking: bool = True) -> ControlStatus:
        """
                    Set gripper command.
              
                    Parameters:
                        end_effector (JointGroup): Gripper enum value.
                        width_m (float): Target gripper width in meters.
                        velocity_mps (float): Gripper motion speed in m/s (optional, default: 0.03).
                        effort (float): Gripper effort in Nm (optional, default: 30).
                        is_blocking (bool): Whether to block until action completes (optional, default: True).
                    
                    Returns:
                        ControlStatus: Command execution/sending result.
        """
     
    def set_joint_commands(self, joint_commands: collections.abc.Sequence[JointCommand], joint_groups: collections.abc.Sequence[str] = [], joint_names: collections.abc.Sequence[str] = []) -> ControlStatus:
        ...
     
    def set_joint_commands(self, joint_commands: collections.abc.Sequence[JointCommand], joint_groups: collections.abc.Sequence[JointGroup] = [], joint_names: collections.abc.Sequence[str] = []) -> ControlStatus:
        """
                    Set joint commands with JointGroup enums.
              
                    Parameters:
                        joint_commands (List[JointCommand]): List of joint commands to control.
                        joint_groups (List[str] or List[JointGroup]): Joint groups to control, can use strings or enums (optional).
                        joint_names (List[str]): Specific joint names, takes priority over joint_groups (optional).
                    
                    Returns:
                        ControlStatus: Result of command execution.
        """
     
    def set_joint_positions(self, joint_positions: collections.abc.Sequence[typing.SupportsFloat], joint_groups: collections.abc.Sequence[str] = [], joint_names: collections.abc.Sequence[str] = [], is_blocking: bool = True, speed_rad_s: typing.SupportsFloat = 0.2, timeout_s: typing.SupportsFloat = 15.0) -> ControlStatus:
        ...
     
    def set_joint_positions(self, joint_positions: collections.abc.Sequence[typing.SupportsFloat], joint_groups: collections.abc.Sequence[JointGroup] = [], joint_names: collections.abc.Sequence[str] = [], is_blocking: bool = True, speed_rad_s: typing.SupportsFloat = 0.2, timeout_s: typing.SupportsFloat = 15.0) -> ControlStatus:
        """
                    Set target joint positions for specified joint groups.
              
                    Parameters:
                        joint_positions (List[float]): Array of joint angles in radians.
                        joint_groups (List[str] or List[JointGroup]): Joint groups to control, can use strings or enums (optional).
                        joint_names (List[str]): Specific joint names, takes priority over joint_groups (optional).
                        is_blocking (bool): Whether to block until command execution completes (optional, default: True).
                        speed_rad_s (float): Maximum joint speed in rad/s (optional, default: 0.2).
                        timeout_s (float): Maximum blocking wait time in seconds (optional, default: 15.0).
                    
                    Returns:
                        ControlStatus: Execution result status.
        """
    def set_suction_cup_command(self, end_effector: JointGroup, activate: bool) -> ControlStatus:
        """
                    Set suction cup command.
              
                    Parameters:
                        end_effector (JointGroup): Suction cup enum value.
                        activate (bool): Whether to activate the suction cup.
                    
                    Returns:
                        ControlStatus: Command sending result.
        """
    def stop_base(self) -> ControlStatus:
        """
                    Stop base motion.
              
                    Parameters:
                        None
                    
                    Returns:
                        ControlStatus: Command sending result.
        """
    def stop_trajectory_execution(self) -> ControlStatus:
        """
                    Stop all currently executing trajectories.
              
                    Parameters:
                        None
                    
                    Returns:
                        ControlStatus: Command sending result.
        """
    def wait_for_shutdown(self) -> None:
        """
                    Sleep continuously until shutdown signal is received.
                    
                    Parameters:
                        None
                    
                    Returns:
                        None
        """
class GripperState:
    """
    Gripper state information
    """
    def __init__(self) -> None:
        ...
    @property
    def effort(self) -> float:
        """
        Gripper torque (newton-meters)
        """
    @effort.setter
    def effort(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def is_moving(self) -> bool:
        """
        Whether currently moving
        """
    @is_moving.setter
    def is_moving(self, arg0: bool) -> None:
        ...
    @property
    def joint_positions(self) -> list[float]:
        """
        Joint positions array
        """
    @joint_positions.setter
    def joint_positions(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def velocity(self) -> float:
        """
        Gripper velocity (meters/second)
        """
    @velocity.setter
    def velocity(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def width(self) -> float:
        """
        Gripper width (meters)
        """
    @width.setter
    def width(self, arg0: typing.SupportsFloat) -> None:
        ...
class Header:
    """
    Message header
    """
    def __init__(self) -> None:
        ...
    @property
    def frame_id(self) -> str:
        """
        Frame ID
        """
    @frame_id.setter
    def frame_id(self, arg0: str) -> None:
        ...
    @property
    def stamp(self) -> Timestamp:
        """
        Timestamp
        """
    @stamp.setter
    def stamp(self, arg0: Timestamp) -> None:
        ...
class IKSolverConfig:
    def __init__(self) -> None:
        ...
    def get_col_aware_ik_joint_limit_bias(self) -> float:
        ...
    def get_col_aware_ik_timeout(self) -> float:
        ...
    def get_enable_collision_check_log(self) -> bool:
        ...
    def get_rotation_eps(self) -> typing.Annotated[list[float], "FixedSize(3)"]:
        ...
    def get_seed_type(self) -> SeedType:
        ...
    def get_translation_eps(self) -> typing.Annotated[list[float], "FixedSize(3)"]:
        ...
    def print(self) -> None:
        ...
    def set_col_aware_ik_joint_limit_bias(self, arg0: typing.SupportsFloat) -> None:
        ...
    def set_col_aware_ik_timeout(self, arg0: typing.SupportsFloat) -> None:
        ...
    def set_enable_collision_check_log(self, arg0: bool) -> None:
        ...
    def set_rotation_eps(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"]) -> None:
        ...
    def set_seed_type(self, arg0: SeedType) -> None:
        ...
    def set_translation_eps(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"]) -> None:
        ...
class ImuData:
    """
    IMU data
    """
    def __init__(self) -> None:
        ...
    @property
    def accel(self) -> Vector3:
        """
        Acceleration Vector3
        """
    @accel.setter
    def accel(self, arg0: Vector3) -> None:
        ...
    @property
    def gyro(self) -> Vector3:
        """
        Gyroscope Vector3
        """
    @gyro.setter
    def gyro(self, arg0: Vector3) -> None:
        ...
    @property
    def magnet(self) -> Vector3:
        """
        Magnetometer Vector3
        """
    @magnet.setter
    def magnet(self, arg0: Vector3) -> None:
        ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None:
        ...
class JointCommand:
    """
    Single joint command object
    """
    def __init__(self) -> None:
        ...
    @property
    def acceleration(self) -> float:
        """
        - `acceleration` (`float`): Joint acceleration
        """
    @acceleration.setter
    def acceleration(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def effort(self) -> float:
        """
        - `effort` (`float`): Joint torque (N·m)
        """
    @effort.setter
    def effort(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def position(self) -> float:
        """
        - `position` (`float`): Joint target position (radians)
        """
    @position.setter
    def position(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def velocity(self) -> float:
        """
        - `velocity` (`float`): Joint velocity (radians/second)
        """
    @velocity.setter
    def velocity(self, arg0: typing.SupportsFloat) -> None:
        ...
class JointGroup:
    """
    Members:
    
      HEAD : Head joint group
    
      LEFT_ARM : Left arm joint group
    
      RIGHT_ARM : Right arm joint group
    
      LEG : Leg joint group
    
      CHASSIS : Chassis joint group
    
      LEFT_GRIPPER : Left gripper
    
      RIGHT_GRIPPER : Right gripper
    
      LEFT_SUCTION_CUP : Left suction cup
    
      RIGHT_SUCTION_CUP : Right suction cup
    """
    CHASSIS: typing.ClassVar[JointGroup]  # value = <JointGroup.CHASSIS: 4>
    HEAD: typing.ClassVar[JointGroup]  # value = <JointGroup.HEAD: 0>
    LEFT_ARM: typing.ClassVar[JointGroup]  # value = <JointGroup.LEFT_ARM: 1>
    LEFT_GRIPPER: typing.ClassVar[JointGroup]  # value = <JointGroup.LEFT_GRIPPER: 5>
    LEFT_SUCTION_CUP: typing.ClassVar[JointGroup]  # value = <JointGroup.LEFT_SUCTION_CUP: 7>
    LEG: typing.ClassVar[JointGroup]  # value = <JointGroup.LEG: 3>
    RIGHT_ARM: typing.ClassVar[JointGroup]  # value = <JointGroup.RIGHT_ARM: 2>
    RIGHT_GRIPPER: typing.ClassVar[JointGroup]  # value = <JointGroup.RIGHT_GRIPPER: 6>
    RIGHT_SUCTION_CUP: typing.ClassVar[JointGroup]  # value = <JointGroup.RIGHT_SUCTION_CUP: 8>
    __members__: typing.ClassVar[dict[str, JointGroup]]  # value = {'HEAD': <JointGroup.HEAD: 0>, 'LEFT_ARM': <JointGroup.LEFT_ARM: 1>, 'RIGHT_ARM': <JointGroup.RIGHT_ARM: 2>, 'LEG': <JointGroup.LEG: 3>, 'CHASSIS': <JointGroup.CHASSIS: 4>, 'LEFT_GRIPPER': <JointGroup.LEFT_GRIPPER: 5>, 'RIGHT_GRIPPER': <JointGroup.RIGHT_GRIPPER: 6>, 'LEFT_SUCTION_CUP': <JointGroup.LEFT_SUCTION_CUP: 7>, 'RIGHT_SUCTION_CUP': <JointGroup.RIGHT_SUCTION_CUP: 8>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class JointState:
    def __init__(self) -> None:
        ...
    @property
    def acceleration(self) -> float:
        ...
    @acceleration.setter
    def acceleration(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def current(self) -> float:
        ...
    @current.setter
    def current(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def effort(self) -> float:
        ...
    @effort.setter
    def effort(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def position(self) -> float:
        ...
    @position.setter
    def position(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def velocity(self) -> float:
        ...
    @velocity.setter
    def velocity(self, arg0: typing.SupportsFloat) -> None:
        ...
class JointStates(RobotStates):
    def __init__(self) -> None:
        ...
    def getType(self) -> RobotStatesType:
        ...
    def set_joint(self, index: typing.SupportsInt, val: typing.SupportsInt) -> None:
        ...
    def set_joint_positions(self, joints: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    @property
    def joint_positions(self) -> list[float]:
        ...
    @joint_positions.setter
    def joint_positions(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
class KinematicsBoundary:
    def __init__(self) -> None:
        ...
    def get_acc_lower_limit(self) -> list[float]:
        ...
    def get_acc_upper_limit(self) -> list[float]:
        ...
    def get_chain_name(self) -> str:
        ...
    def get_jerk_lower_limit(self) -> list[float]:
        ...
    def get_jerk_upper_limit(self) -> list[float]:
        ...
    def get_lower_limit(self) -> list[float]:
        ...
    def get_upper_limit(self) -> list[float]:
        ...
    def get_vel_lower_limit(self) -> list[float]:
        ...
    def get_vel_upper_limit(self) -> list[float]:
        ...
    def print(self) -> None:
        ...
    def set_acc_lower_limit(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    def set_acc_upper_limit(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    def set_chain_name(self, arg0: str) -> None:
        ...
    def set_jerk_lower_limit(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    def set_jerk_upper_limit(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    def set_lower_limit(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    def set_upper_limit(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    def set_vel_lower_limit(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    def set_vel_upper_limit(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
class LidarData:
    """
    LiDAR point cloud data
    """
    def __init__(self) -> None:
        ...
    @property
    def data(self) -> list[int]:
        """
        Point cloud binary data
        """
    @data.setter
    def data(self, arg0: collections.abc.Sequence[typing.SupportsInt]) -> None:
        ...
    @property
    def fields(self) -> list[PointField]:
        """
        Point field description list
        """
    @fields.setter
    def fields(self, arg0: collections.abc.Sequence[PointField]) -> None:
        ...
    @property
    def header(self) -> Header:
        """
        Message header
        """
    @header.setter
    def header(self, arg0: Header) -> None:
        ...
    @property
    def height(self) -> int:
        """
        Point cloud height
        """
    @height.setter
    def height(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def is_bigendian(self) -> bool:
        """
        Whether big-endian
        """
    @is_bigendian.setter
    def is_bigendian(self, arg0: bool) -> None:
        ...
    @property
    def is_dense(self) -> bool:
        """
        Whether dense
        """
    @is_dense.setter
    def is_dense(self, arg0: bool) -> None:
        ...
    @property
    def point_step(self) -> int:
        """
        Bytes per point
        """
    @point_step.setter
    def point_step(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def row_step(self) -> int:
        """
        Bytes per row
        """
    @row_step.setter
    def row_step(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def width(self) -> int:
        """
        Point cloud width
        """
    @width.setter
    def width(self, arg0: typing.SupportsInt) -> None:
        ...
class LineTrajCheckPrimitive:
    def __init__(self) -> None:
        ...
    def get_cylinder_prim_radius(self) -> float:
        ...
    def get_line_check_primitive_type(self) -> PrimitiveType:
        ...
    def get_line_prim_curvature(self) -> float:
        ...
    def print(self) -> None:
        ...
    def set_cylinder_prim_radius(self, arg0: typing.SupportsFloat) -> None:
        ...
    def set_line_check_primitive_type(self, arg0: PrimitiveType) -> None:
        ...
    def set_line_prim_curvature(self, arg0: typing.SupportsFloat) -> None:
        ...
class MotionPlanConfig:
    def __init__(self) -> None:
        ...
    def create_collision_check_option(self) -> CollisionCheckOption:
        ...
    def create_ik_solver_config(self) -> IKSolverConfig:
        ...
    def create_line_traj_check_primitive(self) -> LineTrajCheckPrimitive:
        ...
    def create_sampler_config(self) -> SamplerConfig:
        ...
    def create_trajectory_feasibility_check_option(self) -> TrajectoryFeasibilityCheckOption:
        ...
    def create_trajectory_plan_config(self) -> TrajectoryPlanConfig:
        ...
    def get_collision_check_option(self) -> CollisionCheckOption:
        ...
    def get_collision_check_option_ref(self) -> CollisionCheckOption:
        ...
    def get_feasibility_boundary(self) -> list[KinematicsBoundary]:
        ...
    def get_hard_joint_limit(self) -> list[KinematicsBoundary]:
        ...
    def get_ik_joint_limit(self) -> list[KinematicsBoundary]:
        ...
    def get_ik_solver_config(self) -> IKSolverConfig:
        ...
    def get_ik_solver_config_ref(self) -> IKSolverConfig:
        ...
    def get_line_traj_check_primitive(self) -> LineTrajCheckPrimitive:
        ...
    def get_line_traj_check_primitive_ref(self) -> LineTrajCheckPrimitive:
        ...
    def get_revert_ik_joint_limit(self) -> bool:
        ...
    def get_revert_ik_joint_limit_chains(self) -> list[str]:
        ...
    def get_sampler_config(self) -> SamplerConfig:
        ...
    def get_sampler_config_ref(self) -> SamplerConfig:
        ...
    def get_sampler_joint_limit(self) -> list[KinematicsBoundary]:
        ...
    def get_trajectory_feasibility_check_option(self) -> TrajectoryFeasibilityCheckOption:
        ...
    def get_trajectory_feasibility_check_option_ref(self) -> TrajectoryFeasibilityCheckOption:
        ...
    def get_trajectory_plan_config(self) -> TrajectoryPlanConfig:
        ...
    def get_trajectory_plan_config_ref(self) -> TrajectoryPlanConfig:
        ...
    def get_update_time(self) -> int:
        ...
    def print(self) -> None:
        ...
    def set_collision_check_option(self, arg0: CollisionCheckOption) -> None:
        ...
    def set_feasibility_boundary(self, arg0: collections.abc.Sequence[KinematicsBoundary]) -> None:
        ...
    def set_hard_joint_limit(self, arg0: collections.abc.Sequence[KinematicsBoundary]) -> None:
        ...
    def set_ik_joint_limit(self, arg0: collections.abc.Sequence[KinematicsBoundary]) -> None:
        ...
    def set_ik_solver_config(self, arg0: IKSolverConfig) -> None:
        ...
    def set_line_traj_check_primitive(self, arg0: LineTrajCheckPrimitive) -> None:
        ...
    def set_revert_ik_joint_limit(self, arg0: bool) -> None:
        ...
    def set_revert_ik_joint_limit_chains(self, arg0: collections.abc.Sequence[str]) -> None:
        ...
    def set_sampler_config(self, arg0: SamplerConfig) -> None:
        ...
    def set_sampler_joint_limit(self, arg0: collections.abc.Sequence[KinematicsBoundary]) -> None:
        ...
    def set_trajectory_feasibility_check_option(self, arg0: TrajectoryFeasibilityCheckOption) -> None:
        ...
    def set_trajectory_plan_config(self, arg0: TrajectoryPlanConfig) -> None:
        ...
    def set_update_time(self, arg0: typing.SupportsInt) -> None:
        ...
class MotionStatus:
    """
    Members:
    
      SUCCESS
    
      TIMEOUT
    
      FAULT
    
      INVALID_INPUT
    
      INIT_FAILED
    
      IN_PROGRESS
    
      STOPPED_UNREACHED
    
      DATA_FETCH_FAILED
    
      PUBLISH_FAIL
    
      COMM_DISCONNECTED
    
      STATUS_NUM
    
      UNSUPPORTED_FUNCRION
    """
    COMM_DISCONNECTED: typing.ClassVar[MotionStatus]  # value = <MotionStatus.COMM_DISCONNECTED: 9>
    DATA_FETCH_FAILED: typing.ClassVar[MotionStatus]  # value = <MotionStatus.DATA_FETCH_FAILED: 7>
    FAULT: typing.ClassVar[MotionStatus]  # value = <MotionStatus.FAULT: 2>
    INIT_FAILED: typing.ClassVar[MotionStatus]  # value = <MotionStatus.INIT_FAILED: 4>
    INVALID_INPUT: typing.ClassVar[MotionStatus]  # value = <MotionStatus.INVALID_INPUT: 3>
    IN_PROGRESS: typing.ClassVar[MotionStatus]  # value = <MotionStatus.IN_PROGRESS: 5>
    PUBLISH_FAIL: typing.ClassVar[MotionStatus]  # value = <MotionStatus.PUBLISH_FAIL: 8>
    STATUS_NUM: typing.ClassVar[MotionStatus]  # value = <MotionStatus.STATUS_NUM: 10>
    STOPPED_UNREACHED: typing.ClassVar[MotionStatus]  # value = <MotionStatus.STOPPED_UNREACHED: 6>
    SUCCESS: typing.ClassVar[MotionStatus]  # value = <MotionStatus.SUCCESS: 0>
    TIMEOUT: typing.ClassVar[MotionStatus]  # value = <MotionStatus.TIMEOUT: 1>
    UNSUPPORTED_FUNCRION: typing.ClassVar[MotionStatus]  # value = <MotionStatus.UNSUPPORTED_FUNCRION: 11>
    __members__: typing.ClassVar[dict[str, MotionStatus]]  # value = {'SUCCESS': <MotionStatus.SUCCESS: 0>, 'TIMEOUT': <MotionStatus.TIMEOUT: 1>, 'FAULT': <MotionStatus.FAULT: 2>, 'INVALID_INPUT': <MotionStatus.INVALID_INPUT: 3>, 'INIT_FAILED': <MotionStatus.INIT_FAILED: 4>, 'IN_PROGRESS': <MotionStatus.IN_PROGRESS: 5>, 'STOPPED_UNREACHED': <MotionStatus.STOPPED_UNREACHED: 6>, 'DATA_FETCH_FAILED': <MotionStatus.DATA_FETCH_FAILED: 7>, 'PUBLISH_FAIL': <MotionStatus.PUBLISH_FAIL: 8>, 'COMM_DISCONNECTED': <MotionStatus.COMM_DISCONNECTED: 9>, 'STATUS_NUM': <MotionStatus.STATUS_NUM: 10>, 'UNSUPPORTED_FUNCRION': <MotionStatus.UNSUPPORTED_FUNCRION: 11>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class OdomData:
    """
    Odometry data
    """
    def __init__(self) -> None:
        ...
    @property
    def orientation(self) -> typing.Annotated[list[float], "FixedSize(4)"]:
        """
        Orientation quaternion [x, y, z, w]
        """
    @orientation.setter
    def orientation(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat], "FixedSize(4)"]) -> None:
        ...
    @property
    def position(self) -> typing.Annotated[list[float], "FixedSize(3)"]:
        """
        Position [x, y, z] (meters)
        """
    @position.setter
    def position(self, arg0: typing.Annotated[collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"]) -> None:
        ...
    @property
    def timestamp(self) -> int:
        """
        Timestamp (seconds)
        """
    @timestamp.setter
    def timestamp(self, arg0: typing.SupportsInt) -> None:
        ...
class Parameter:
    actuate_type: ...
    is_blocking: bool
    is_check_collision: bool
    is_direct_execute: bool
    is_tool_pose: bool
    reference_frame: str
     
    def __init__(self) -> None:
        ...
     
    def __init__(self, direct_execute: bool, blocking: bool, timeout: typing.SupportsFloat, actuate: str, tool_pose: bool, check_collision: bool, frame: str = 'base_link') -> None:
        ...
    def __repr__(self) -> str:
        ...
    def get_actuate_type(self) -> str:
        """
        Get the actuation type (only link, including torso, including legs).
        """
    def get_blocking(self) -> bool:
        """
        Get whether to wait synchronously for trajectory execution or planning completion.
        """
    def get_check_collision(self) -> bool:
        """
        Get whether to perform collision detection.
        """
    def get_direct_execute(self) -> bool:
        """
        Get whether to directly execute the trajectory after planning.
        """
    def get_reference_frame(self) -> str:
        """
        Get the reference coordinate frame for planning.
        """
    def get_timeout(self) -> float:
        """
        Get the maximum waiting time for trajectory execution or planning completion (in seconds).
        """
    def get_tool_pose(self) -> bool:
        """
        Get whether to use tool pose for actuation.
        """
    def set_actuate(self, actuate: str) -> None:
        """
        Set the actuation type (only link, including torso, including legs).
        """
    def set_blocking(self, blocking: bool) -> None:
        """
        Set whether to wait synchronously for trajectory execution or planning completion.
        """
    def set_check_collision(self, check_collision: bool) -> None:
        """
        Set whether to perform collision detection.
        """
    def set_direct_execute(self, direct_execute: bool) -> None:
        """
        Set whether to directly execute the trajectory after planning.
        """
    def set_move_line(self, move_line: bool) -> None:
        """
        Set whether to use linear movement for planning.
        """
    def set_reference_frame(self, frame: str) -> None:
        """
        Set the reference coordinate frame for planning.
        """
    def set_timeout(self, timeout: typing.SupportsFloat) -> None:
        """
        Set the maximum waiting time for trajectory execution or planning completion (in seconds).
        """
    def set_tool_pose(self, tool_pose: bool) -> None:
        """
        Set whether to use tool pose for actuation.
        """
    @property
    def joint_state(self) -> dict[JointGroup, list[float]]:
        ...
    @joint_state.setter
    def joint_state(self, arg0: collections.abc.Mapping[JointGroup, collections.abc.Sequence[typing.SupportsFloat]]) -> None:
        ...
    @property
    def timeout_second(self) -> float:
        ...
    @timeout_second.setter
    def timeout_second(self, arg0: typing.SupportsFloat) -> None:
        ...
class Point:
     
    def __init__(self) -> None:
        ...
     
    def __init__(self, x: typing.SupportsFloat = 0.0, y: typing.SupportsFloat = 0.0, z: typing.SupportsFloat = 0.0) -> None:
        ...
    @property
    def x(self) -> float:
        ...
    @x.setter
    def x(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def y(self) -> float:
        ...
    @y.setter
    def y(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def z(self) -> float:
        ...
    @z.setter
    def z(self, arg0: typing.SupportsFloat) -> None:
        ...
class PointField:
    """
    Point cloud field description information
    """
    def __init__(self) -> None:
        ...
    @property
    def count(self) -> int:
        """
        Number of field elements
        """
    @count.setter
    def count(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def datatype(self) -> ...:
        """
        Data type (DataType enum)
        """
    @datatype.setter
    def datatype(self, arg0: ...) -> None:
        ...
    @property
    def name(self) -> str:
        """
        Field name, e.g., x, y, z, intensity, rgb
        """
    @name.setter
    def name(self, arg0: str) -> None:
        ...
    @property
    def offset(self) -> int:
        """
        Byte offset of field in a single point
        """
    @offset.setter
    def offset(self, arg0: typing.SupportsInt) -> None:
        ...
class PointFieldDataType:
    """
    Members:
    
      UNKNOWN
    
      INT8
    
      UINT8
    
      INT16
    
      UINT16
    
      INT32
    
      UINT32
    
      FLOAT32
    
      FLOAT64
    """
    FLOAT32: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.FLOAT32: 7>
    FLOAT64: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.FLOAT64: 8>
    INT16: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.INT16: 3>
    INT32: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.INT32: 5>
    INT8: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.INT8: 1>
    UINT16: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.UINT16: 4>
    UINT32: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.UINT32: 6>
    UINT8: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.UINT8: 2>
    UNKNOWN: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.UNKNOWN: 0>
    __members__: typing.ClassVar[dict[str, PointFieldDataType]]  # value = {'UNKNOWN': <PointFieldDataType.UNKNOWN: 0>, 'INT8': <PointFieldDataType.INT8: 1>, 'UINT8': <PointFieldDataType.UINT8: 2>, 'INT16': <PointFieldDataType.INT16: 3>, 'UINT16': <PointFieldDataType.UINT16: 4>, 'INT32': <PointFieldDataType.INT32: 5>, 'UINT32': <PointFieldDataType.UINT32: 6>, 'FLOAT32': <PointFieldDataType.FLOAT32: 7>, 'FLOAT64': <PointFieldDataType.FLOAT64: 8>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Pose:
    orientation: Quaternion
    position: Point
     
    def __init__(self) -> None:
        ...
     
    def __init__(self, pos: collections.abc.Sequence[typing.SupportsFloat], quat: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
     
    def __init__(self, vec: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
class PoseState(RobotStates):
    frame_id: str
    pose: Pose
    reference_frame: str
    def __init__(self) -> None:
        ...
    def getType(self) -> RobotStatesType:
        ...
class PrimitiveType:
    """
    Members:
    
      LINE
    
      CYLINDER
    """
    CYLINDER: typing.ClassVar[PrimitiveType]  # value = <PrimitiveType.CYLINDER: 1>
    LINE: typing.ClassVar[PrimitiveType]  # value = <PrimitiveType.LINE: 0>
    __members__: typing.ClassVar[dict[str, PrimitiveType]]  # value = {'LINE': <PrimitiveType.LINE: 0>, 'CYLINDER': <PrimitiveType.CYLINDER: 1>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Quaternion:
     
    def __init__(self) -> None:
        ...
     
    def __init__(self, x: typing.SupportsFloat = 0.0, y: typing.SupportsFloat = 0.0, z: typing.SupportsFloat = 0.0, w: typing.SupportsFloat = 1.0) -> None:
        ...
    @property
    def w(self) -> float:
        ...
    @w.setter
    def w(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def x(self) -> float:
        ...
    @x.setter
    def x(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def y(self) -> float:
        ...
    @y.setter
    def y(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def z(self) -> float:
        ...
    @z.setter
    def z(self, arg0: typing.SupportsFloat) -> None:
        ...
class RgbData:
    """
    Compressed image data
    """
    def __init__(self) -> None:
        ...
    @property
    def data(self) -> list[int]:
        """
        Compressed binary data
        """
    @data.setter
    def data(self, arg0: collections.abc.Sequence[typing.SupportsInt]) -> None:
        ...
    @property
    def format(self) -> str:
        """
        Image format
        """
    @format.setter
    def format(self, arg0: str) -> None:
        ...
    @property
    def header(self) -> Header:
        """
        Message header
        """
    @header.setter
    def header(self, arg0: Header) -> None:
        ...
class RobotStates:
    chain_name: str
     
    def __init__(self) -> None:
        ...
     
    def __init__(self, chain: str, whole_joint: collections.abc.Sequence[typing.SupportsFloat], base_pose: Pose) -> None:
        ...
    def getType(self) -> RobotStatesType:
        ...
    def setBaseState(self, base_pose: Pose) -> None:
        ...
    def setWholeBodyJoint(self, joint_positions: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    @property
    def base_state(self) -> list[float]:
        ...
    @base_state.setter
    def base_state(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
    @property
    def whole_body_joint(self) -> list[float]:
        ...
    @whole_body_joint.setter
    def whole_body_joint(self, arg0: collections.abc.Sequence[typing.SupportsFloat]) -> None:
        ...
class RobotStatesType:
    """
    Members:
    
      POSE
    
      JOINT
    
      ROBOT_STATES
    """
    JOINT: typing.ClassVar[RobotStatesType]  # value = <RobotStatesType.JOINT: 1>
    POSE: typing.ClassVar[RobotStatesType]  # value = <RobotStatesType.POSE: 0>
    ROBOT_STATES: typing.ClassVar[RobotStatesType]  # value = <RobotStatesType.ROBOT_STATES: 2>
    __members__: typing.ClassVar[dict[str, RobotStatesType]]  # value = {'POSE': <RobotStatesType.POSE: 0>, 'JOINT': <RobotStatesType.JOINT: 1>, 'ROBOT_STATES': <RobotStatesType.ROBOT_STATES: 2>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class SUCTION_ACTION_STATE:
    """
    Suction cup action state enumeration
    
    Members:
    
      IDLE : Not sucking
    
      SUCKING : Currently sucking
    
      SUCCESS : Suction successful
    
      FAILED : Suction failed
    """
    FAILED: typing.ClassVar[SUCTION_ACTION_STATE]  # value = <SUCTION_ACTION_STATE.FAILED: 3>
    IDLE: typing.ClassVar[SUCTION_ACTION_STATE]  # value = <SUCTION_ACTION_STATE.IDLE: 0>
    SUCCESS: typing.ClassVar[SUCTION_ACTION_STATE]  # value = <SUCTION_ACTION_STATE.SUCCESS: 2>
    SUCKING: typing.ClassVar[SUCTION_ACTION_STATE]  # value = <SUCTION_ACTION_STATE.SUCKING: 1>
    __members__: typing.ClassVar[dict[str, SUCTION_ACTION_STATE]]  # value = {'IDLE': <SUCTION_ACTION_STATE.IDLE: 0>, 'SUCKING': <SUCTION_ACTION_STATE.SUCKING: 1>, 'SUCCESS': <SUCTION_ACTION_STATE.SUCCESS: 2>, 'FAILED': <SUCTION_ACTION_STATE.FAILED: 3>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class SamplerConfig:
    def __init__(self) -> None:
        ...
    def get_interpolate(self) -> bool:
        ...
    def get_interpolation_cnt(self) -> int:
        ...
    def get_max_planning_time(self) -> float:
        ...
    def get_max_simplification_time(self) -> float:
        ...
    def get_simplify(self) -> bool:
        ...
    def get_state_check_resolution(self) -> float:
        ...
    def get_state_check_type(self) -> StateCheckType:
        ...
    def get_termination_condition_type(self) -> TerminationConditionType:
        ...
    def print(self) -> None:
        ...
    def set_interpolate(self, arg0: bool) -> None:
        ...
    def set_interpolation_cnt(self, arg0: typing.SupportsInt) -> None:
        ...
    def set_max_planning_time(self, arg0: typing.SupportsFloat) -> None:
        ...
    def set_max_simplification_time(self, arg0: typing.SupportsFloat) -> None:
        ...
    def set_simplify(self, arg0: bool) -> None:
        ...
    def set_state_check_resolution(self, arg0: typing.SupportsFloat) -> None:
        ...
    def set_state_check_type(self, arg0: StateCheckType) -> None:
        ...
    def set_termination_condition_type(self, arg0: TerminationConditionType) -> None:
        ...
class SeedType:
    """
    Members:
    
      RANDOM_SEED
    
      RANDOM_PROGRESSIVE_SEED
    
      USER_DEFINED_SEED
    """
    RANDOM_PROGRESSIVE_SEED: typing.ClassVar[SeedType]  # value = <SeedType.RANDOM_PROGRESSIVE_SEED: 1>
    RANDOM_SEED: typing.ClassVar[SeedType]  # value = <SeedType.RANDOM_SEED: 0>
    USER_DEFINED_SEED: typing.ClassVar[SeedType]  # value = <SeedType.USER_DEFINED_SEED: 2>
    __members__: typing.ClassVar[dict[str, SeedType]]  # value = {'RANDOM_SEED': <SeedType.RANDOM_SEED: 0>, 'RANDOM_PROGRESSIVE_SEED': <SeedType.RANDOM_PROGRESSIVE_SEED: 1>, 'USER_DEFINED_SEED': <SeedType.USER_DEFINED_SEED: 2>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class SensorType:
    """
    Members:
    
      HEAD_LEFT_CAMERA : Head left camera
    
      HEAD_RIGHT_CAMERA : Head right camera
    
      LEFT_ARM_CAMERA : Left arm camera
    
      RIGHT_ARM_CAMERA : Right arm camera
    
      LEFT_ARM_DEPTH_CAMERA : Left arm depth camera
    
      RIGHT_ARM_DEPTH_CAMERA : Right arm depth camera
    
      BASE_LIDAR : Base LiDAR
    
      TORSO_IMU : Torso IMU
    
      BASE_ULTRASONIC : Base ultrasonic sensor
    """
    BASE_LIDAR: typing.ClassVar[SensorType]  # value = <SensorType.BASE_LIDAR: 6>
    BASE_ULTRASONIC: typing.ClassVar[SensorType]  # value = <SensorType.BASE_ULTRASONIC: 8>
    HEAD_LEFT_CAMERA: typing.ClassVar[SensorType]  # value = <SensorType.HEAD_LEFT_CAMERA: 0>
    HEAD_RIGHT_CAMERA: typing.ClassVar[SensorType]  # value = <SensorType.HEAD_RIGHT_CAMERA: 1>
    LEFT_ARM_CAMERA: typing.ClassVar[SensorType]  # value = <SensorType.LEFT_ARM_CAMERA: 2>
    LEFT_ARM_DEPTH_CAMERA: typing.ClassVar[SensorType]  # value = <SensorType.LEFT_ARM_DEPTH_CAMERA: 4>
    RIGHT_ARM_CAMERA: typing.ClassVar[SensorType]  # value = <SensorType.RIGHT_ARM_CAMERA: 3>
    RIGHT_ARM_DEPTH_CAMERA: typing.ClassVar[SensorType]  # value = <SensorType.RIGHT_ARM_DEPTH_CAMERA: 5>
    TORSO_IMU: typing.ClassVar[SensorType]  # value = <SensorType.TORSO_IMU: 7>
    __members__: typing.ClassVar[dict[str, SensorType]]  # value = {'HEAD_LEFT_CAMERA': <SensorType.HEAD_LEFT_CAMERA: 0>, 'HEAD_RIGHT_CAMERA': <SensorType.HEAD_RIGHT_CAMERA: 1>, 'LEFT_ARM_CAMERA': <SensorType.LEFT_ARM_CAMERA: 2>, 'RIGHT_ARM_CAMERA': <SensorType.RIGHT_ARM_CAMERA: 3>, 'LEFT_ARM_DEPTH_CAMERA': <SensorType.LEFT_ARM_DEPTH_CAMERA: 4>, 'RIGHT_ARM_DEPTH_CAMERA': <SensorType.RIGHT_ARM_DEPTH_CAMERA: 5>, 'BASE_LIDAR': <SensorType.BASE_LIDAR: 6>, 'TORSO_IMU': <SensorType.TORSO_IMU: 7>, 'BASE_ULTRASONIC': <SensorType.BASE_ULTRASONIC: 8>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class StateCheckType:
    """
    Members:
    
      EUCLIDEAN_DISTANCE
    
      RADIAN_DISTANCE
    """
    EUCLIDEAN_DISTANCE: typing.ClassVar[StateCheckType]  # value = <StateCheckType.EUCLIDEAN_DISTANCE: 0>
    RADIAN_DISTANCE: typing.ClassVar[StateCheckType]  # value = <StateCheckType.RADIAN_DISTANCE: 1>
    __members__: typing.ClassVar[dict[str, StateCheckType]]  # value = {'EUCLIDEAN_DISTANCE': <StateCheckType.EUCLIDEAN_DISTANCE: 0>, 'RADIAN_DISTANCE': <StateCheckType.RADIAN_DISTANCE: 1>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class SuctionCupState:
    """
    Suction cup state information
    """
    def __init__(self) -> None:
        ...
    @property
    def action_state(self) -> SUCTION_ACTION_STATE:
        """
        Current suction cup action state (SUCTION_ACTION_STATE enum)
        """
    @action_state.setter
    def action_state(self, arg0: SUCTION_ACTION_STATE) -> None:
        ...
    @property
    def activation(self) -> bool:
        """
        Whether currently sucking
        """
    @activation.setter
    def activation(self, arg0: bool) -> None:
        ...
    @property
    def pressure(self) -> float:
        """
        Current pressure (Pa)
        """
    @pressure.setter
    def pressure(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None:
        ...
class TerminationConditionType:
    """
    Members:
    
      TIMEOUT
    
      TIMEOUT_AND_EXACT_SOLUTION
    """
    TIMEOUT: typing.ClassVar[TerminationConditionType]  # value = <TerminationConditionType.TIMEOUT: 0>
    TIMEOUT_AND_EXACT_SOLUTION: typing.ClassVar[TerminationConditionType]  # value = <TerminationConditionType.TIMEOUT_AND_EXACT_SOLUTION: 1>
    __members__: typing.ClassVar[dict[str, TerminationConditionType]]  # value = {'TIMEOUT': <TerminationConditionType.TIMEOUT: 0>, 'TIMEOUT_AND_EXACT_SOLUTION': <TerminationConditionType.TIMEOUT_AND_EXACT_SOLUTION: 1>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Timestamp:
    """
    High-precision timestamp
    """
    def __init__(self) -> None:
        ...
    @property
    def nanosec(self) -> int:
        """
        Nanoseconds
        """
    @nanosec.setter
    def nanosec(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def sec(self) -> int:
        """
        Seconds
        """
    @sec.setter
    def sec(self, arg0: typing.SupportsInt) -> None:
        ...
class Trajectory:
    """
    Trajectory object
    """
    def __init__(self) -> None:
        ...
    @property
    def joint_groups(self) -> list[str]:
        """
        List of joint group names
        """
    @joint_groups.setter
    def joint_groups(self, arg0: collections.abc.Sequence[str]) -> None:
        ...
    @property
    def joint_names(self) -> list[str]:
        """
        List of joint names
        """
    @joint_names.setter
    def joint_names(self, arg0: collections.abc.Sequence[str]) -> None:
        ...
    @property
    def points(self) -> list[TrajectoryPoint]:
        """
        List of trajectory points (TrajectoryPoint list)
        """
    @points.setter
    def points(self, arg0: collections.abc.Sequence[TrajectoryPoint]) -> None:
        ...
class TrajectoryControlStatus:
    """
    Members:
    
      INVALID_INPUT : Input parameters do not meet requirements
    
      RUNNING : Currently running
    
      COMPLETED : Reached target position
    
      STOPPED_UNREACHED : Stopped but not reached target
    
      ERROR : Error occurred, cannot continue execution
    
      DATA_FETCH_FAILED : Failed to fetch execution data
    """
    COMPLETED: typing.ClassVar[TrajectoryControlStatus]  # value = <TrajectoryControlStatus.COMPLETED: 2>
    DATA_FETCH_FAILED: typing.ClassVar[TrajectoryControlStatus]  # value = <TrajectoryControlStatus.DATA_FETCH_FAILED: 5>
    ERROR: typing.ClassVar[TrajectoryControlStatus]  # value = <TrajectoryControlStatus.ERROR: 4>
    INVALID_INPUT: typing.ClassVar[TrajectoryControlStatus]  # value = <TrajectoryControlStatus.INVALID_INPUT: 0>
    RUNNING: typing.ClassVar[TrajectoryControlStatus]  # value = <TrajectoryControlStatus.RUNNING: 1>
    STOPPED_UNREACHED: typing.ClassVar[TrajectoryControlStatus]  # value = <TrajectoryControlStatus.STOPPED_UNREACHED: 3>
    __members__: typing.ClassVar[dict[str, TrajectoryControlStatus]]  # value = {'INVALID_INPUT': <TrajectoryControlStatus.INVALID_INPUT: 0>, 'RUNNING': <TrajectoryControlStatus.RUNNING: 1>, 'COMPLETED': <TrajectoryControlStatus.COMPLETED: 2>, 'STOPPED_UNREACHED': <TrajectoryControlStatus.STOPPED_UNREACHED: 3>, 'ERROR': <TrajectoryControlStatus.ERROR: 4>, 'DATA_FETCH_FAILED': <TrajectoryControlStatus.DATA_FETCH_FAILED: 5>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class TrajectoryFeasibilityCheckOption:
    def __init__(self) -> None:
        ...
    def get_disable_collision_check(self) -> bool:
        ...
    def get_disable_joint_limit_check(self) -> bool:
        ...
    def get_disable_velocity_feasibility_check(self) -> bool:
        ...
    def print(self) -> None:
        ...
    def set_disable_collision_check(self, arg0: bool) -> None:
        ...
    def set_disable_joint_limit_check(self, arg0: bool) -> None:
        ...
    def set_disable_velocity_feasibility_check(self, arg0: bool) -> None:
        ...
class TrajectoryPlanConfig:
    def __init__(self) -> None:
        ...
    def get_min_move_time(self) -> float:
        ...
    def get_move_line_intermediate_point(self) -> float:
        ...
    def get_way_point_plan_expected_time(self) -> float:
        ...
    def print(self) -> None:
        ...
    def set_min_move_time(self, arg0: typing.SupportsFloat) -> None:
        ...
    def set_move_line_intermediate_point(self, arg0: typing.SupportsFloat) -> None:
        ...
    def set_way_point_plan_expected_time(self, arg0: typing.SupportsFloat) -> None:
        ...
class TrajectoryPoint:
    """
    Single trajectory point object
    """
    def __init__(self) -> None:
        ...
    @property
    def joint_command_vec(self) -> list[JointCommand]:
        """
        - `joint_command_vec` (`List[JointCommand]`): List of specific joint commands to execute
        """
    @joint_command_vec.setter
    def joint_command_vec(self, arg0: collections.abc.Sequence[JointCommand]) -> None:
        ...
    @property
    def time_from_start_second(self) -> float:
        """
        - `time_from_start_second` (`float`): Time from trajectory start (seconds)
        """
    @time_from_start_second.setter
    def time_from_start_second(self, arg0: typing.SupportsFloat) -> None:
        ...
class UltrasonicData:
    """
    Ultrasonic sensor data
    """
    def __init__(self) -> None:
        ...
    @property
    def distance(self) -> float:
        """
        Distance (meters)
        """
    @distance.setter
    def distance(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None:
        ...
class UltrasonicType:
    """
    Members:
    
      FRONT_LEFT : Front left
    
      FRONT_RIGHT : Front right
    
      RIGHT_LEFT : Right left
    
      RIGHT_RIGHT : Right right
    
      BACK_LEFT : Back left
    
      BACK_RIGHT : Back right
    
      LEFT_LEFT : Left left
    
      LEFT_RIGHT : Left right
    """
    BACK_LEFT: typing.ClassVar[UltrasonicType]  # value = <UltrasonicType.BACK_LEFT: 4>
    BACK_RIGHT: typing.ClassVar[UltrasonicType]  # value = <UltrasonicType.BACK_RIGHT: 5>
    FRONT_LEFT: typing.ClassVar[UltrasonicType]  # value = <UltrasonicType.FRONT_LEFT: 0>
    FRONT_RIGHT: typing.ClassVar[UltrasonicType]  # value = <UltrasonicType.FRONT_RIGHT: 1>
    LEFT_LEFT: typing.ClassVar[UltrasonicType]  # value = <UltrasonicType.LEFT_LEFT: 6>
    LEFT_RIGHT: typing.ClassVar[UltrasonicType]  # value = <UltrasonicType.LEFT_RIGHT: 7>
    RIGHT_LEFT: typing.ClassVar[UltrasonicType]  # value = <UltrasonicType.RIGHT_LEFT: 2>
    RIGHT_RIGHT: typing.ClassVar[UltrasonicType]  # value = <UltrasonicType.RIGHT_RIGHT: 3>
    __members__: typing.ClassVar[dict[str, UltrasonicType]]  # value = {'FRONT_LEFT': <UltrasonicType.FRONT_LEFT: 0>, 'FRONT_RIGHT': <UltrasonicType.FRONT_RIGHT: 1>, 'RIGHT_LEFT': <UltrasonicType.RIGHT_LEFT: 2>, 'RIGHT_RIGHT': <UltrasonicType.RIGHT_RIGHT: 3>, 'BACK_LEFT': <UltrasonicType.BACK_LEFT: 4>, 'BACK_RIGHT': <UltrasonicType.BACK_RIGHT: 5>, 'LEFT_LEFT': <UltrasonicType.LEFT_LEFT: 6>, 'LEFT_RIGHT': <UltrasonicType.LEFT_RIGHT: 7>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Vector3:
    """
    Three-dimensional vector
    """
    def __init__(self) -> None:
        ...
    @property
    def x(self) -> float:
        """
        X coordinate
        """
    @x.setter
    def x(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def y(self) -> float:
        """
        Y coordinate
        """
    @y.setter
    def y(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def z(self) -> float:
        """
        Z coordinate
        """
    @z.setter
    def z(self, arg0: typing.SupportsFloat) -> None:
        ...
class WBCException(Exception):
    pass
def check_motion_status(arg0: MotionStatus) -> str:
    """
                        Convert a MotionStatus enum value to a string.
                        
                        Parameters:
                            status (MotionStatus): The motion status to convert.
                            
                        Returns:
                            str: The string representation of the motion status.
    """
def create_joint_state() -> JointStates:
    """
                        Create a JointStates instance.
                        
                        Parameters:
                            None
    
                        Returns:
                            JointStates: A new JointStates instance.
    """
def create_parameter(direct_execute: bool, blocking: bool, timeout: typing.SupportsFloat, actuate: str, tool_pose: bool, check_collision: bool, frame: str = 'base_link') -> Parameter:
    """
                        Create a Parameter instance.
    
                        Notes:
                            - GalbotMotion currently does NOT provide real-time obstacle perception / automatic environment updates.
                            - Attached objects are part of the manually-maintained collision world used by motion planning/checking.
                            - For obstacle_type == "point_cloud", `key` is typically a point cloud file path provided by the user.
                            - For obstacle_type == "depth_image", this is a manual input to construct collision obstacles; it is not a continuous
                            real-time perception stream for motion planning.
    
                        Parameters:
                            direct_execute (bool): Whether to execute the motion directly.
                            blocking (bool): Whether to block the execution until completion.
                            timeout (float): Maximum time to wait for the motion to complete.
                            actuate (str): Actuation type (position/velocity/torque).
                            tool_pose (bool): Whether the motion is for a tool pose.
                            check_collision (bool): Whether to check for collisions.
                            frame (str, optional): Coordinate frame for the motion. Defaults to "base_link".
                            
                        Returns:
                            Parameter: A new Parameter instance.
    """
def create_pose_state() -> PoseState:
    """
                        Create a PoseState instance.
                        
                        Parameters:
                            None
    
                        Returns:
                            PoseState: A new PoseState instance.
    """
COMM_DISCONNECTED: MotionStatus  # value = <MotionStatus.COMM_DISCONNECTED: 9>
CYLINDER: PrimitiveType  # value = <PrimitiveType.CYLINDER: 1>
DATA_FETCH_FAILED: MotionStatus  # value = <MotionStatus.DATA_FETCH_FAILED: 7>
EUCLIDEAN_DISTANCE: StateCheckType  # value = <StateCheckType.EUCLIDEAN_DISTANCE: 0>
FAULT: MotionStatus  # value = <MotionStatus.FAULT: 2>
INIT_FAILED: MotionStatus  # value = <MotionStatus.INIT_FAILED: 4>
INVALID_INPUT: MotionStatus  # value = <MotionStatus.INVALID_INPUT: 3>
IN_PROGRESS: MotionStatus  # value = <MotionStatus.IN_PROGRESS: 5>
JOINT: RobotStatesType  # value = <RobotStatesType.JOINT: 1>
LINE: PrimitiveType  # value = <PrimitiveType.LINE: 0>
POSE: RobotStatesType  # value = <RobotStatesType.POSE: 0>
PUBLISH_FAIL: MotionStatus  # value = <MotionStatus.PUBLISH_FAIL: 8>
RADIAN_DISTANCE: StateCheckType  # value = <StateCheckType.RADIAN_DISTANCE: 1>
RANDOM_PROGRESSIVE_SEED: SeedType  # value = <SeedType.RANDOM_PROGRESSIVE_SEED: 1>
RANDOM_SEED: SeedType  # value = <SeedType.RANDOM_SEED: 0>
ROBOT_STATES: RobotStatesType  # value = <RobotStatesType.ROBOT_STATES: 2>
STATUS_NUM: MotionStatus  # value = <MotionStatus.STATUS_NUM: 10>
STOPPED_UNREACHED: MotionStatus  # value = <MotionStatus.STOPPED_UNREACHED: 6>
SUCCESS: MotionStatus  # value = <MotionStatus.SUCCESS: 0>
TIMEOUT: TerminationConditionType  # value = <TerminationConditionType.TIMEOUT: 0>
TIMEOUT_AND_EXACT_SOLUTION: TerminationConditionType  # value = <TerminationConditionType.TIMEOUT_AND_EXACT_SOLUTION: 1>
UNSUPPORTED_FUNCRION: MotionStatus  # value = <MotionStatus.UNSUPPORTED_FUNCRION: 11>
USER_DEFINED_SEED: SeedType  # value = <SeedType.USER_DEFINED_SEED: 2>
