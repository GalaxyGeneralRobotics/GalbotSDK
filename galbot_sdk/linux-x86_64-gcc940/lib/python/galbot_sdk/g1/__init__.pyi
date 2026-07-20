"""
Motion Plan Configuration module
"""

from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import typing

__all__: list[str] = [
    "AudioData",
    "CLOSE_TO_OBSTACLE",
    "COLLISION",
    "COMM_DISCONNECTED",
    "CYLINDER",
    "CollisionCheckOption",
    "ControlStatus",
    "DATA_FETCH_FAILED",
    "DepthData",
    "DetectionAndSegmentationResult",
    "DetectionResult",
    "DexHandType",
    "DexhandState",
    "EUCLIDEAN_DISTANCE",
    "EffortInfo",
    "Error",
    "ErrorInfo",
    "FAILED",
    "FAULT",
    "FOUNDATION_STEREO",
    "ForceData",
    "FrameTriad",
    "G1ControllerName",
    "G1JointGroup",
    "GalbotMotion",
    "GalbotNavigation",
    "GalbotOneFoxtrotSensor",
    "GalbotPerception",
    "GalbotRobot",
    "GripperState",
    "GroupCommand",
    "Header",
    "IKSolverConfig",
    "INIT_FAILED",
    "INTERRUPTED",
    "INVALID_INPUT",
    "IN_PROGRESS",
    "ImuData",
    "JOINT",
    "JointCommand",
    "JointState",
    "JointStateMessage",
    "JointStates",
    "KinematicsBoundary",
    "LIGHT_STEREO",
    "LINE",
    "LidarData",
    "LineTrajCheckPrimitive",
    "LogLevel",
    "MachineType",
    "MotionPlanConfig",
    "MotionStatus",
    "NavigationTaskSnapshot",
    "NavigationTaskStatus",
    "OCCUPIED",
    "OdomData",
    "POSE",
    "PUBLISH_FAIL",
    "Parameter",
    "PerceptionModule",
    "Point",
    "Point2d",
    "PointField",
    "PointFieldDataType",
    "Pose",
    "Pose2d",
    "PoseState",
    "PrimitiveType",
    "Quaternion",
    "RADIAN_DISTANCE",
    "RANDOM_PROGRESSIVE_SEED",
    "RANDOM_SEED",
    "ROBOT_STATES",
    "RUNNING",
    "RgbData",
    "RobotStates",
    "RobotStatesType",
    "STATUS_NUM",
    "STOPPED_UNREACHED",
    "SUCCESS",
    "SUCTION_ACTION_STATE",
    "SamplerConfig",
    "SeedType",
    "SensorType",
    "SingoriXTarget",
    "StateCheckType",
    "SuctionCupState",
    "SyncedObservation",
    "TARGET_DATA_DEFAULT",
    "TARGET_DATA_FRAME_POSE",
    "TARGET_DATA_FRAME_TWIST",
    "TARGET_DATA_FRAME_WRENCH",
    "TARGET_DATA_JOINT_ACCELERATION",
    "TARGET_DATA_JOINT_EFFORT",
    "TARGET_DATA_JOINT_POSITION",
    "TARGET_DATA_JOINT_VELOCITY",
    "TARGET_DATA_NONE",
    "TARGET_TYPE_APPEND",
    "TARGET_TYPE_CLEAR",
    "TARGET_TYPE_DEFAULT",
    "TARGET_TYPE_NONE",
    "TARGET_TYPE_OVERRIDE",
    "TARGET_TYPE_PREPENDNOW",
    "TARGET_TYPE_PROVERRIDE",
    "TARGET_TYPE_TOUCH",
    "TIMEOUT",
    "TIMEOUT_AND_EXACT_SOLUTION",
    "TargetConfig",
    "TargetGroupTrajectory",
    "TargetSampling",
    "TargetTaskTrajectory",
    "TaskCommand",
    "TaskHandle",
    "TerminationConditionType",
    "Timestamp",
    "Trajectory",
    "TrajectoryControlStatus",
    "TrajectoryFeasibilityCheckOption",
    "TrajectoryPlanConfig",
    "TrajectoryPoint",
    "Twist",
    "UNKNOWN",
    "UNSUPPORTED_FUNCRION",
    "USER_DEFINED_SEED",
    "UltrasonicData",
    "UltrasonicType",
    "Vector3",
    "WBCException",
    "Waypoint",
    "WaypointParams",
    "Wrench",
    "check_motion_status",
    "create_joint_state",
    "create_parameter",
    "create_pose_state",
]

class AudioData:
    """
    Audio stream data from microphone input callbacks
    """
    def __init__(self) -> None: ...
    @property
    def data(self) -> list[int]:
        """
        Binary payload; interpretation depends on format: pcm — 2560 bytes per 80 ms chunk; json — UTF-8 text length varies or empty for markers
        """
    @data.setter
    def data(self, arg0: collections.abc.Sequence[typing.SupportsInt]) -> None: ...
    @property
    def format(self) -> str:
        """
        Audio format: 'pcm' (16000 Hz, 16-bit, mono) or 'json' (UTF-8 encoded JSON text)
        """
    @format.setter
    def format(self, arg0: str) -> None: ...
    @property
    def header(self) -> Header:
        """
        Message header: timestamp_ns (data acquisition time in ns since epoch), frame_id (stream or source frame identifier)
        """
    @header.setter
    def header(self, arg0: Header) -> None: ...
    @property
    def type(self) -> str:
        """
        Audio type identifier. Possible values: 'waken_up' (wake-up event, format json, data is JSON string), 'denoise_chunk' (denoised audio, format pcm, data is PCM binary), 'vad_begin' (VAD start marker, data empty), 'vad_chunk' (VAD audio, format pcm, data is PCM binary), 'vad_end' (VAD end marker, data empty)
        """
    @type.setter
    def type(self, arg0: str) -> None: ...

class CollisionCheckOption:
    def __init__(self) -> None: ...
    def get_disable_env_collision_check(self) -> bool: ...
    def get_disable_self_collision_check(self) -> bool: ...
    def print(self) -> None: ...
    def set_disable_env_collision_check(self, disable: bool) -> None: ...
    def set_disable_self_collision_check(self, disable: bool) -> None: ...

class ControlStatus:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | SUCCESS | Execution successful |
    | TIMEOUT | Execution timeout |
    | FAULT | Fault occurred, cannot continue execution |
    | INVALID_INPUT | Input parameters do not meet requirements |
    | INIT_FAILED | Internal communication component creation failed |
    | IN_PROGRESS | Motion in progress but not reached target |
    | STOPPED_UNREACHED | Stopped but not reached target |
    | DATA_FETCH_FAILED | Data fetch failed |
    | PUBLISH_FAIL | Data sending failed |
    | COMM_DISCONNECTED | Connection failed |
    """

    COMM_DISCONNECTED: typing.ClassVar[
        ControlStatus
    ]  # value = <ControlStatus.COMM_DISCONNECTED: 9>
    DATA_FETCH_FAILED: typing.ClassVar[
        ControlStatus
    ]  # value = <ControlStatus.DATA_FETCH_FAILED: 7>
    FAULT: typing.ClassVar[ControlStatus]  # value = <ControlStatus.FAULT: 2>
    INIT_FAILED: typing.ClassVar[
        ControlStatus
    ]  # value = <ControlStatus.INIT_FAILED: 4>
    INVALID_INPUT: typing.ClassVar[
        ControlStatus
    ]  # value = <ControlStatus.INVALID_INPUT: 3>
    IN_PROGRESS: typing.ClassVar[
        ControlStatus
    ]  # value = <ControlStatus.IN_PROGRESS: 5>
    PUBLISH_FAIL: typing.ClassVar[
        ControlStatus
    ]  # value = <ControlStatus.PUBLISH_FAIL: 8>
    STOPPED_UNREACHED: typing.ClassVar[
        ControlStatus
    ]  # value = <ControlStatus.STOPPED_UNREACHED: 6>
    SUCCESS: typing.ClassVar[ControlStatus]  # value = <ControlStatus.SUCCESS: 0>
    TIMEOUT: typing.ClassVar[ControlStatus]  # value = <ControlStatus.TIMEOUT: 1>
    __members__: typing.ClassVar[
        dict[str, ControlStatus]
    ]  # value = {'SUCCESS': <ControlStatus.SUCCESS: 0>, 'TIMEOUT': <ControlStatus.TIMEOUT: 1>, 'FAULT': <ControlStatus.FAULT: 2>, 'INVALID_INPUT': <ControlStatus.INVALID_INPUT: 3>, 'INIT_FAILED': <ControlStatus.INIT_FAILED: 4>, 'IN_PROGRESS': <ControlStatus.IN_PROGRESS: 5>, 'STOPPED_UNREACHED': <ControlStatus.STOPPED_UNREACHED: 6>, 'DATA_FETCH_FAILED': <ControlStatus.DATA_FETCH_FAILED: 7>, 'PUBLISH_FAIL': <ControlStatus.PUBLISH_FAIL: 8>, 'COMM_DISCONNECTED': <ControlStatus.COMM_DISCONNECTED: 9>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class DepthData:
    """
    Depth image data
    """
    def __init__(self) -> None: ...
    @property
    def data(self) -> bytes:
        """
        Compressed depth data
        """
    @property
    def depth_scale(self) -> int:
        """
        Depth scale/quantization factor
        """
    @depth_scale.setter
    def depth_scale(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def format(self) -> str:
        """
        Image format
        """
    @format.setter
    def format(self, arg0: str) -> None: ...
    @property
    def header(self) -> Header:
        """
        Message header
        """
    @header.setter
    def header(self, arg0: Header) -> None: ...
    @property
    def height(self) -> int:
        """
        Image height
        """
    @height.setter
    def height(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def width(self) -> int:
        """
        Image width
        """
    @width.setter
    def width(self, arg0: typing.SupportsInt) -> None: ...

class DetectionAndSegmentationResult:
    """
    Single detection/segmentation result
    """
    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...
    @property
    def bbox(self) -> tuple[int, int, int, int]:
        """
        Bounding box as (x, y, width, height)
        """
    @property
    def class_index(self) -> int:
        """
        Class index
        """
    @class_index.setter
    def class_index(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def class_name(self) -> str:
        """
        Class name
        """
    @class_name.setter
    def class_name(self, arg0: str) -> None: ...
    @property
    def confidence(self) -> float:
        """
        Confidence score
        """
    @confidence.setter
    def confidence(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def keypoints(self) -> list[tuple[float, float]]:
        """
        Keypoints as list of (x, y) tuples
        """

class DetectionResult:
    """
    Perception detection result
    """
    def __init__(self) -> None: ...
    def clear(self) -> None:
        """
        Clear all result fields
        """
    def get_result_info(self) -> str:
        """
        Get result summary string
        """
    @property
    def bounding_boxes(self) -> list[tuple[int, int, int, int]]:
        """
        Bounding boxes as list of (x, y, width, height)
        """
    @property
    def class_indices(self) -> list[int]:
        """
        List of class indices
        """
    @class_indices.setter
    def class_indices(
        self, arg0: collections.abc.Sequence[typing.SupportsInt]
    ) -> None: ...
    @property
    def class_names(self) -> list[str]:
        """
        List of class names
        """
    @class_names.setter
    def class_names(self, arg0: collections.abc.Sequence[str]) -> None: ...
    @property
    def confidences(self) -> list[float]:
        """
        List of confidences
        """
    @confidences.setter
    def confidences(
        self, arg0: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    @property
    def detection_results(self) -> list[DetectionAndSegmentationResult]:
        """
        List of DetectionAndSegmentationResult
        """
    @detection_results.setter
    def detection_results(
        self, arg0: collections.abc.Sequence[DetectionAndSegmentationResult]
    ) -> None: ...
    @property
    def grasp_pose_result(self) -> list[list[float]]:
        """
        Grasp pose results
        """
    @grasp_pose_result.setter
    def grasp_pose_result(
        self,
        arg0: collections.abc.Sequence[collections.abc.Sequence[typing.SupportsFloat]],
    ) -> None: ...
    @property
    def instance_mask(self) -> typing.Any:
        """
        Instance mask as numpy array (HxW or HxWxC), or None if empty
        """
    @property
    def ocr_string(self) -> list[str]:
        """
        OCR results
        """
    @ocr_string.setter
    def ocr_string(self, arg0: collections.abc.Sequence[str]) -> None: ...
    @property
    def point_clouds(self) -> list:
        """
        Point clouds as list of Nx3 numpy arrays
        """
    @property
    def running_info(self) -> str:
        """
        Running info string
        """
    @running_info.setter
    def running_info(self, arg0: str) -> None: ...
    @property
    def sensor_name(self) -> str:
        """
        Sensor name
        """
    @sensor_name.setter
    def sensor_name(self, arg0: str) -> None: ...
    @property
    def target_point_poses(
        self,
    ) -> list[typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 4]"]]:
        """
        4x4 poses from perception proto field target_point_poses (same buffer as target_poses here)
        """
    @property
    def target_poses(
        self,
    ) -> list[typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 4]"]]:
        """
        List of 4x4 target pose matrices (C++ targetPoses; perception proto target_point_poses fills this)
        """
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp in nanoseconds
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...

class DexHandType:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | INSPIRE | Inspire dexterous hand |
    | BRAINCO | BrainCo dexterous hand |
    | SHARPA | Sharpa dexterous hand |
    | LINKER_L20 | Linker Hand L20 dexterous hand (16 joints, range [0,255]) |
    """

    BRAINCO: typing.ClassVar[DexHandType]  # value = <DexHandType.BRAINCO: 1>
    INSPIRE: typing.ClassVar[DexHandType]  # value = <DexHandType.INSPIRE: 0>
    LINKER_L20: typing.ClassVar[DexHandType]  # value = <DexHandType.LINKER_L20: 3>
    SHARPA: typing.ClassVar[DexHandType]  # value = <DexHandType.SHARPA: 2>
    __members__: typing.ClassVar[
        dict[str, DexHandType]
    ]  # value = {'INSPIRE': <DexHandType.INSPIRE: 0>, 'BRAINCO': <DexHandType.BRAINCO: 1>, 'SHARPA': <DexHandType.SHARPA: 2>, 'LINKER_L20': <DexHandType.LINKER_L20: 3>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class DexhandState:
    """
    Full dexterous hand state (joint feedback and optional force sensors)
    """
    def __init__(self) -> None: ...
    @property
    def force_sensor_map(self) -> dict[str, EffortInfo]:
        """
        Named force sensor map: sensor_name -> EffortInfo (Sharpa; empty for Inspire/BrainCo)
        """
    @force_sensor_map.setter
    def force_sensor_map(
        self, arg0: collections.abc.Mapping[str, EffortInfo]
    ) -> None: ...
    @property
    def joint_state(self) -> JointStateMessage:
        """
        Dexhand joint state message
        """
    @joint_state.setter
    def joint_state(self, arg0: JointStateMessage) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...

class EffortInfo:
    """
    6D force/torque information
    """
    def __init__(self) -> None: ...
    @property
    def force(self) -> Vector3:
        """
        Force vector Vector3
        """
    @force.setter
    def force(self, arg0: Vector3) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def torque(self) -> Vector3:
        """
        Torque vector Vector3
        """
    @torque.setter
    def torque(self, arg0: Vector3) -> None: ...

class Error:
    """
    Single error entry
    """

    def __init__(self) -> None:
        """
        Default error entry
        """

    def __init__(
        self, commpent: str, error_code: typing.SupportsInt, description: str
    ) -> None: ...
    @property
    def commpent(self) -> str:
        """
        Fault component name
        """
    @commpent.setter
    def commpent(self, arg0: str) -> None: ...
    @property
    def description(self) -> str:
        """
        Human-readable error description
        """
    @description.setter
    def description(self, arg0: str) -> None: ...
    @property
    def error_code(self) -> int:
        """
        Numerical error code
        """
    @error_code.setter
    def error_code(self, arg0: typing.SupportsInt) -> None: ...

class ErrorInfo:
    """
    Timestamped error collection
    """
    def __init__(self) -> None: ...
    @property
    def error_vec(self) -> list[Error]:
        """
        List of error entries
        """
    @error_vec.setter
    def error_vec(self, arg0: collections.abc.Sequence[Error]) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Collection timestamp in nanoseconds
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...

class ForceData:
    """
    Force sensor data
    """
    def __init__(self) -> None: ...
    @property
    def force(self) -> Vector3:
        """
        Force vector Vector3
        """
    @force.setter
    def force(self, arg0: Vector3) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def torque(self) -> Vector3:
        """
        Torque vector Vector3
        """
    @torque.setter
    def torque(self, arg0: Vector3) -> None: ...

class FrameTriad:
    """
    Task-space command for a body frame
    """
    def __init__(self) -> None: ...
    @property
    def body_frame_id(self) -> str:
        """
        Body frame id
        """
    @body_frame_id.setter
    def body_frame_id(self, arg0: str) -> None: ...
    @property
    def header(self) -> Header:
        """
        Message header
        """
    @header.setter
    def header(self, arg0: Header) -> None: ...
    @property
    def pose(self) -> Pose | None:
        """
        Optional pose command
        """
    @pose.setter
    def pose(self, arg0: Pose | None) -> None: ...
    @property
    def reference_frame_id(self) -> str:
        """
        Reference frame id
        """
    @reference_frame_id.setter
    def reference_frame_id(self, arg0: str) -> None: ...
    @property
    def twist(self) -> Twist | None:
        """
        Optional twist command
        """
    @twist.setter
    def twist(self, arg0: Twist | None) -> None: ...
    @property
    def wrench(self) -> Wrench | None:
        """
        Optional wrench command
        """
    @wrench.setter
    def wrench(self, arg0: Wrench | None) -> None: ...

class G1ControllerName:
    CHASSIS_POSE_CTRL: typing.ClassVar[str] = "chassis_pose_ctrl"
    CHASSIS_TWIST_CTRL: typing.ClassVar[str] = "chassis_twist_ctrl"
    CONTROLLER_NAME_NUM: typing.ClassVar[str] = "CONTROLLER_NAME_NUM"
    HEAD_PVT_BYPASS_CTRL: typing.ClassVar[str] = "head_pvt_bypass_ctrl"
    HEAD_PVT_CTRL: typing.ClassVar[str] = "head_pvt_ctrl"
    LEFT_ARM_PVT_BYPASS_CTRL: typing.ClassVar[str] = "left_arm_pvt_bypass_ctrl"
    LEFT_ARM_PVT_CTRL: typing.ClassVar[str] = "left_arm_pvt_ctrl"
    LEFT_DEXHAND_CTRL: typing.ClassVar[str] = "left_dexhand_ctrl"
    LEFT_GRIPPER_CTRL: typing.ClassVar[str] = "left_gripper_ctrl"
    LEG_PVT_BYPASS_CTRL: typing.ClassVar[str] = "leg_pvt_bypass_ctrl"
    LEG_PVT_CTRL: typing.ClassVar[str] = "leg_pvt_ctrl"
    RIGHT_ARM_PVT_BYPASS_CTRL: typing.ClassVar[str] = "right_arm_pvt_bypass_ctrl"
    RIGHT_ARM_PVT_CTRL: typing.ClassVar[str] = "right_arm_pvt_ctrl"
    RIGHT_DEXHAND_CTRL: typing.ClassVar[str] = "right_dexhand_ctrl"
    RIGHT_GRIPPER_CTRL: typing.ClassVar[str] = "right_gripper_ctrl"

class G1JointGroup:
    chassis: typing.ClassVar[str] = "chassis"
    head: typing.ClassVar[str] = "head"
    left_arm: typing.ClassVar[str] = "left_arm"
    left_dexhand: typing.ClassVar[str] = "left_dexhand"
    left_gripper: typing.ClassVar[str] = "left_gripper"
    left_suction_cup: typing.ClassVar[str] = "left_suction_cup"
    leg: typing.ClassVar[str] = "leg"
    right_arm: typing.ClassVar[str] = "right_arm"
    right_dexhand: typing.ClassVar[str] = "right_dexhand"
    right_gripper: typing.ClassVar[str] = "right_gripper"
    right_suction_cup: typing.ClassVar[str] = "right_suction_cup"

class GalbotMotion:
    def __repr__(self) -> str: ...
    def add_obstacle(
        self,
        obstacle_id: str,
        obstacle_type: str,
        pose: collections.abc.Sequence[typing.SupportsFloat],
        scale: typing.Annotated[
            collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"
        ] = [0.0, 0.0, 0.0],
        key: str = "",
        target_frame: str = "world",
        ee_frame: str = "ee_base",
        reference_joint_positions: collections.abc.Sequence[typing.SupportsFloat] = [],
        reference_base_pose: collections.abc.Sequence[typing.SupportsFloat] = [],
        ignore_collision_link_names: collections.abc.Sequence[str] = [],
        safe_margin: typing.SupportsFloat = 0.0,
        resolution: typing.SupportsFloat = 0.01,
    ) -> MotionStatus:
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
    def attach_target_object(
        self,
        obstacle_id: str,
        obstacle_type: str,
        pose: collections.abc.Sequence[typing.SupportsFloat],
        scale: typing.Annotated[
            collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"
        ] = [0.0, 0.0, 0.0],
        key: str = "",
        target_frame: str = "world",
        ee_frame: str = "ee_base",
        reference_joint_positions: collections.abc.Sequence[typing.SupportsFloat] = [],
        reference_base_pose: collections.abc.Sequence[typing.SupportsFloat] = [],
        ignore_collision_link_names: collections.abc.Sequence[str] = [],
        safe_margin: typing.SupportsFloat = 0.0,
        resolution: typing.SupportsFloat = 0.01,
    ) -> MotionStatus:
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
    def check_collision(
        self,
        start: collections.abc.Sequence[RobotStates],
        enable_collision_check: bool = True,
        params: Parameter = ...,
    ) -> tuple[MotionStatus, list[bool]]:
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
    def forward_kinematics(
        self,
        target_frame: str,
        reference_frame: str = "base_link",
        joint_state: collections.abc.Mapping[
            str, collections.abc.Sequence[typing.SupportsFloat]
        ] = {},
        params: Parameter = ...,
    ) -> tuple[MotionStatus, list[float]]:
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
    def forward_kinematics_by_state(
        self,
        target_frame: str,
        reference_robot_states: RobotStates = None,
        reference_frame: str = "base_link",
        params: Parameter = ...,
    ) -> tuple[MotionStatus, list[float]]:
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
        Get the list of currently loaded obstacle IDs.
        """
    def get_chain_joint_state(self) -> dict[str, list[float]]:
        """
        Get current joint positions per kinematic chain (map: chain name -> joint angle list).
        """
    def get_end_effector_pose(
        self, end_effector_frame: str, reference_frame: str = "base_link"
    ) -> tuple[MotionStatus, list[float]]:
        """
        Get the pose of a specified end-effector frame.

        Parameters:
            end_effector_frame (str): The name of the end-effector frame.
            reference_frame (str, optional): The name of the reference frame. Defaults to "base_link".

        Returns:
            Pose: The computed pose of the end-effector frame.
        """
    def get_end_effector_pose_on_chain(
        self,
        chain_name: str,
        frame_id: str = "EndEffector",
        reference_frame: str = "base_link",
    ) -> tuple[MotionStatus, list[float]]:
        """
        Get the pose of a specified end-effector frame on a given chain.

        Parameters:
            chain_name (str): The name of the chain.
            frame_id (str, optional): The name of the end-effector frame. Defaults to "EndEffector".
            reference_frame (str, optional): The name of the reference frame. Defaults to "base_link".

        Returns:
            Pose: The computed pose of the end-effector frame on the specified chain.
        """
    def get_jacobian(
        self,
        chain_name: str,
        target_frame: str = "EndEffector",
        reference_frame: str = "base_link",
        joint_state: collections.abc.Mapping[
            str, collections.abc.Sequence[typing.SupportsFloat]
        ] = {},
        params: Parameter = ...,
    ) -> tuple[MotionStatus, list[list[float]]]:
        """
        Compute the Jacobian matrix for a kinematic chain.

        This is the chain-level convenience API. If joint_state is provided,
        the SDK reads the current whole-body state and replaces the specified
        chain joint values before computing the Jacobian. If joint_state is
        empty, the current complete robot state is used directly.

        Use this API when you only need chain-level joint overrides. Use
        get_jacobian_by_state() when you need to provide a complete
        RobotStates object including whole-body joints and base pose.

        Parameters:
            chain_name (str): Kinematic chain (e.g., "left_arm", "right_arm")
            target_frame (str, optional): Frame on chain. Defaults to "EndEffector".
            reference_frame (str, optional): Reference frame. Defaults to "base_link".
            joint_state (dict, optional): Chain joint override map. Uses current complete state if empty.
            params (Parameter, optional): Planning parameters. Defaults to default_param.

        Returns:
            tuple: (MotionStatus, jacobian_matrix)
                - jacobian_matrix is a list of lists: [[float]] (6 rows x N cols)
        """
    def get_jacobian_by_state(
        self,
        chain_name: str,
        target_frame: str = "EndEffector",
        reference_frame: str = "base_link",
        reference_robot_states: RobotStates = None,
        params: Parameter = ...,
    ) -> tuple[MotionStatus, list[list[float]]]:
        """
        Compute the Jacobian matrix using complete robot state.

        This API accepts a RobotStates object for specifying the complete
        robot configuration (whole-body joints + base pose). Passing None
        uses the current complete robot state.

        Use this API for offline or hypothetical-state Jacobian computation,
        reproducible tests, or cases where the base pose must be controlled
        explicitly. Use get_jacobian() for simpler chain-level current-state
        or chain-joint override queries.

        Parameters:
            chain_name (str): Kinematic chain (e.g., "left_arm", "right_arm")
            target_frame (str, optional): Frame on chain. Defaults to "EndEffector".
            reference_frame (str, optional): Reference frame. Defaults to "base_link".
            reference_robot_states (RobotStates, optional): Complete robot state. Uses current complete state if None.
            params (Parameter, optional): Planning parameters. Defaults to default_param.

        Returns:
            tuple: (MotionStatus, jacobian_matrix)
                - jacobian_matrix is a list of lists: [[float]] (6 rows x N cols)
        """
    def get_link_names(self, only_end_effector: bool = False) -> list[str]:
        """
        Get robot link names from kinematic model.

        Parameters:
            only_end_effector (bool, optional): If true, returns only end-effector/tool links;
                if false, returns all links including base, intermediate, and end-effector links.
                Default: false (all links).

        Returns:
            list: Vector of link name strings (empty if retrieval fails)

        Note:
            End-effector detection based on link having no child links in kinematic tree.
            Useful for forward kinematics queries or TF frame validation.
        """
    def get_motion_plan_config(self) -> tuple[MotionStatus, MotionPlanConfig]:
        """
        get motion config
        """
    def get_robot_states(self) -> RobotStates:
        """
        Get current whole-body joint and base state as RobotStates (requires WBC/sensors when used live).
        """
    def get_supported_chains(self) -> set[str]:
        """
        Get the set of supported kinematic chain names (e.g. left_arm, right_arm).
        """
    def get_supported_ee_frames(self) -> set[str]:
        """
        Get the set of supported end-effector frame identifiers.
        """
    def get_supported_frames(self) -> set[str]:
        """
        Get the set of supported reference frame names.
        """
    def get_supported_links(self) -> set[str]:
        """
        Get the set of supported link names (URDF link names for FK/IK).
        """
    def get_supported_obstacle_types(self) -> set[str]:
        """
        Get the set of supported obstacle types (e.g. box, sphere, cylinder, mesh).
        """
    def get_supported_tool_list(self) -> set[str]:
        """
        Get the list of supported tool names for attach_tool.
        """
    def init(self) -> bool:
        """
        Initialize the motion planning system. Must be called before other APIs.
        Parameters: None
        Returns: bool: True if succeeded; False otherwise.
        """
    def inverse_kinematics(
        self,
        target_pose: collections.abc.Sequence[typing.SupportsFloat],
        chain_names: collections.abc.Sequence[str],
        target_frame: str = "EndEffector",
        reference_frame: str = "base_link",
        initial_joint_positions: collections.abc.Mapping[
            str, collections.abc.Sequence[typing.SupportsFloat]
        ] = {},
        enable_collision_check: bool = True,
        params: Parameter = ...,
    ) -> tuple[MotionStatus, dict[str, list[float]]]:
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
    def inverse_kinematics_by_state(
        self,
        target_pose: collections.abc.Sequence[typing.SupportsFloat],
        chain_names: collections.abc.Sequence[str],
        target_frame: str = "EndEffector",
        reference_frame: str = "base_link",
        reference_robot_states: RobotStates = None,
        enable_collision_check: bool = True,
        params: Parameter = ...,
    ) -> tuple[MotionStatus, dict[str, list[float]]]:
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
    def motion_plan(
        self,
        target: RobotStates,
        start: RobotStates = None,
        reference_robot_states: RobotStates = None,
        enable_collision_check: bool = True,
        params: Parameter = ...,
    ) -> tuple[MotionStatus, dict[str, list[list[float]]]]:
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

    def motion_plan_multi_waypoints(
        self,
        target: RobotStates,
        waypoint_poses: collections.abc.Sequence[
            collections.abc.Sequence[typing.SupportsFloat]
        ],
        start: RobotStates = None,
        reference_robot_states: RobotStates = None,
        enable_collision_check: bool = True,
        params: Parameter = ...,
    ) -> tuple[MotionStatus, dict[str, list[list[float]]]]:
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

    def motion_plan_multi_waypoints(
        self,
        targets: collections.abc.Mapping[
            RobotStates,
            collections.abc.Sequence[collections.abc.Sequence[typing.SupportsFloat]],
        ],
        start: collections.abc.Sequence[RobotStates] = [],
        reference_robot_states: RobotStates = None,
        enable_collision_check: bool = True,
        params: Parameter = ...,
    ) -> tuple[MotionStatus, dict[str, list[list[float]]]]:
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
    def move_whole_body_joint_zero(
        self,
        is_blocking: bool = True,
        leg_head_speed_rad_s: typing.SupportsFloat = 0.2,
        leg_head_timeout_s: typing.SupportsFloat = 15.0,
        params: Parameter = ...,
    ) -> MotionStatus:
        """
        Move whole-body joints to the predefined zero (home) configuration.

        - leg/head are commanded via GalbotRobot direct joint control
        - left/right arms are planned via motion planner with collision checking enabled
        """
    def remove_obstacle(self, obstacle_id: str) -> MotionStatus:
        """
        Remove an obstacle by its ID
        """
    def set_end_effector_pose(
        self,
        target_pose: collections.abc.Sequence[typing.SupportsFloat],
        end_effector_frame: str,
        reference_frame: str = "base_link",
        reference_robot_states: RobotStates = None,
        enable_collision_check: bool = True,
        is_blocking: bool = True,
        timeout: typing.SupportsFloat = -1.0,
        params: Parameter = ...,
    ) -> MotionStatus:
        """
        Set the pose of a specified end-effector frame.

        The robot executes the motion regardless of is_blocking. When is_blocking is True,
        this call waits until execution completes or times out. When is_blocking is False,
        this call returns immediately after starting a background task that waits for
        execution completion.

        Parameters:
            target_pose (Pose): The target pose.
            end_effector_frame (str): The name of the end-effector frame.
            reference_frame (str, optional): The name of the reference frame. Defaults to "base_link".
            reference_robot_states (RobotStates, optional): The reference robot states. Defaults to nullptr.
            enable_collision_check (bool, optional): Whether to enable collision checking. Defaults to true.
            is_blocking (bool, optional): Whether this API waits for execution completion.
                False still starts robot motion and returns immediately. Defaults to true.
            timeout (float, optional): Maximum time in seconds for the SDK to wait for motion
                completion. If negative, params.timeout_second is used. In non-blocking mode,
                the timeout is applied inside the background task. Defaults to -1.0.
            params (dict, optional): Additional parameters for the motion planning. Defaults to default_param.

        Note:
            Motion speed parameters are configured under
            /data/galbot/config/default/service_motion_plan/traj_plan.

        Returns:
            MotionStatus: SUCCESS if execution completed in blocking mode, or if background execution
                started in non-blocking mode; otherwise an error status.
        """
    def set_motion_plan_config(self, config: MotionPlanConfig) -> MotionStatus:
        """
        set motion config
        """
    def status_to_string(self, status: MotionStatus) -> str:
        """
        Convert MotionStatus to a human-readable string.
        """

class GalbotNavigation:
    def add_bounding_box(self, box_info: dict) -> tuple:
        """
        Add a bounding box so navigation can ignore the corresponding fused obstacle points.

        Parameters:
            box_info (dict): Contains:
                box_size: [length_x, length_y, length_z], meters.
                box_pose: [x, y, z, qx, qy, qz, qw] relative to parent_link_name.
                box_tag: SDK box tag, converted internally to an SDK-marked box name.
                parent_link_name: Parent link for the box pose.

        Returns:
            tuple: (success: bool, status_string: str)
        """
    def attach_box_to_link(
        self, box_info: dict, ignore_collision_links: collections.abc.Sequence[str] = []
    ) -> tuple:
        """
        Attach a box collision object to a robot link.

        Parameters:
            box_info (dict): Contains:
                box_size: [length_x, length_y, length_z], meters.
                box_pose: [x, y, z, qx, qy, qz, qw] relative to parent_link_name.
                box_tag: SDK box tag, converted internally to an SDK-marked box name.
                parent_link_name: Parent link for the box pose.
            ignore_collision_links (list[str]): Robot links to ignore for collision checking.

        Returns:
            tuple: (success: bool, status_string: str)
        """
    def check_goal_arrival(self) -> bool:
        """
        Check if the robot has successfully reached the current goal (within tolerance).

        Parameters:
            None

        Returns:
            bool: True if the robot has reached the goal; False if still navigating or no active goal.
        """
    def check_path_reachability(
        self,
        goal_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64],
        start_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64],
    ) -> bool:
        """
        Check if a collision-free path exists from start to goal in the map (static obstacles only).

        Parameters:
            goal_pose (array): Goal pose [x, y, z, qx, qy, qz, qw], map frame.
            start_pose (array): Start pose [x, y, z, qx, qy, qz, qw], map frame.

        Returns:
            bool: True if a collision-free path exists from start to goal; False otherwise.
        """
    def detach_box_from_link(self, box_tag: typing.SupportsInt) -> tuple:
        """
        Detach a box collision object from its robot link.

        Parameters:
            box_tag (int): SDK box tag to detach.

        Returns:
            tuple: (success: bool, status_string: str)
        """
    def dump_navigation_configs(self) -> tuple:
        """
        Dump navigation dynamic configuration through SDK logs.

        Parameters:
            None

        Returns:
            tuple: (success: bool, status_string: str)
        """
    def get_bounding_box(self) -> list:
        """
        Get bounding boxes currently used by navigation obstacle filtering.

        Parameters:
            None

        Returns:
            list[dict]: Each dict contains box_size, box_pose, box_tag, and parent_link_name.
        """
    def get_current_pose(self) -> typing.Annotated[list[float], "FixedSize(7)"]:
        """
        Get the current estimated pose of the robot chassis in the map frame.

        Parameters:
            None

        Returns:
            array: [x, y, z, qx, qy, qz, qw], map frame (meters, unit quaternion). Valid only if is_localized() is True.
        """
    def get_navigation_status(self) -> NavigationTaskStatus:
        """
        Get the latest navigation task state.

        This API is useful when monitoring a navigation task in
        non-blocking mode.

        Parameters:
            None

                Returns:
                    NavigationTaskStatus: Current task state for non-blocking navigation polling.
        """
    def get_navigation_target_status(self, task_id: str) -> NavigationTaskSnapshot:
        """
        Query the status of an asynchronous navigation task.

        Parameters:
            task_id (str): Task identifier returned by the corresponding
            navigation API.

        Returns:
            NavigationTaskSnapshot: Latest known state for the requested task.
        """
    def init(self) -> bool:
        """
        Initialize the navigation subsystem and its dependencies. Must be called before other navigation APIs.

        Parameters:
            None

        Returns:
            bool: True if initialization succeeded; False otherwise.
        """
    def is_localized(self) -> bool:
        """
        Check whether the robot is currently localized in the map (valid pose with sufficient confidence).

        Parameters:
            None

        Returns:
            bool: True if localized; False if localization is lost or uncertain.
        """
    def move_straight_to(
        self,
        goal_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64],
        is_blocking: bool = True,
        timeout: typing.SupportsFloat = 8,
    ) -> tuple:
        """
        Move the robot to a relative target pose in the odometry frame (no global path planning).

        Parameters:
            goal_pose (array): Target pose relative to current base_link [x, y, z, qx, qy, qz, qw], odom frame (meters).
            is_blocking (bool): If True, blocks until motion is complete or timeout; default True.
            timeout (float): Maximum wait time in seconds for blocking mode; default 8.0.

                Returns:
                    tuple: (success: bool, status_string: str)
                        - success: True if motion succeeded.
                        - status_string: Status string.
        """
    def navigate_along_trajectory(
        self,
        waypoints: collections.abc.Sequence[Pose],
        frame_id: str = "map",
        speed_ratio: typing.SupportsFloat = 1.0,
        enable_collision_check: bool = True,
    ) -> TaskHandle:
        """
        Submit a trajectory navigation task using ordered 3D poses.

        This API treats the input poses as a trajectory reference. The
        planner may smooth and optimize the path, so intermediate poses
        are not guaranteed to be reached exactly. Only the final pose is
        guaranteed as the navigation goal.

        Parameters:
            waypoints (list[Pose]): Ordered pose waypoints.
            frame_id (str): Reference frame, typically "map" or "base_link".
            speed_ratio (float): Velocity scaling factor.
            enable_collision_check (bool): Whether to enable collision checking.

        Returns:
            TaskHandle: Submitted task id, request result, and message.
        """
    def navigate_through_waypoints(
        self,
        waypoints: collections.abc.Sequence[Waypoint],
        frame_id: str = "map",
        enable_collision_check: bool = True,
    ) -> TaskHandle:
        """
        Submit a multi-waypoint navigation task.

        This API sends multiple waypoints in one request and executes
        them in the order provided by the caller.

        Parameters:
            waypoints (list[Waypoint]): Ordered waypoint targets.
            frame_id (str): Reference frame, typically "map" or "base_link".
            enable_collision_check (bool): Whether to enable collision checking.

        Returns:
            TaskHandle: Submitted task id, request result, and message.
        """
    def navigate_to_goal(
        self,
        goal_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64],
        enable_collision_check: bool = True,
        is_blocking: bool = False,
        timeout: typing.SupportsFloat = 8,
        omni_plan: bool = False,
    ) -> tuple:
        """
        Navigate the robot to a target goal pose in the map frame.

        Parameters:
            goal_pose (array): Target goal pose [x, y, z, qx, qy, qz, qw], map frame (meters, quaternion).
            enable_collision_check (bool): If True, enables dynamic obstacle detection and avoidance; default True.
            is_blocking (bool): If True, monitor navigation in the current thread; if False, start a background monitor thread and return after command acceptance; default False.
            timeout (float): SDK-side navigation monitor timeout in seconds; used by both blocking and non-blocking modes. If timeout expires before navigation stops, SDK calls stop_navigation automatically; default 8.0.
            omni_plan (bool): If True, omnidirectional motion planning; if False, differential drive; default False.

                Returns:
                    tuple: (success: bool, status_string: str)
                        - success: True if navigation succeeded.
                        - status_string: Status string (SUCCESS, FAIL, TIMEOUT, etc.).
        """
    def navigate_to_goal_v2(
        self,
        goal_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64],
        max_vel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64],
        pose_frame: str = "map",
        enable_collision_check: bool = True,
        is_blocking: bool = False,
        timeout: typing.SupportsFloat = 5.0,
        omni_plan: bool = False,
    ) -> tuple:
        """
        Navigate the robot to a target goal pose using navigation v2.

        Parameters:
            goal_pose (array): Target pose [x, y, z, qx, qy, qz, qw] in pose_frame.
            max_vel (array): Maximum velocity [vx, vy, vyaw].
            pose_frame (str): Reference frame, "map" or "base_link"; default "map".
            enable_collision_check (bool): Enable v2 collision checking fields; default True.
            is_blocking (bool): If True, blocks until goal is reached or timeout; default False.
            timeout (float): Navigation runtime timeout sent to the PNS service; negative means no motion time limit; default 5.0.
            omni_plan (bool): If True, omnidirectional motion planning; if False, heading-based planning; default False.

        Returns:
            tuple: (success: bool, status_string: str)
                - success: True if navigation succeeded.
                - status_string: Status string (SUCCESS, FAIL, TIMEOUT, etc.).
        """
    def navigate_with_velocity(
        self,
        vx: typing.SupportsFloat,
        vy: typing.SupportsFloat,
        vyaw: typing.SupportsFloat,
        duration_s: typing.SupportsFloat = 3.0,
        enable_collision_check: bool = True,
    ) -> tuple:
        """
        Navigate with a velocity command using navigation v2.

        Parameters:
            vx (float): Linear velocity in x direction.
            vy (float): Linear velocity in y direction.
            vyaw (float): Angular velocity around z axis.
            duration_s (float): Command duration in seconds. Must be greater than 0.0; default 3.0.
            enable_collision_check (bool): Enable runtime collision checking; default True.

        Returns:
            tuple: (success: bool, status_string: str)

        Warning:
            This API is non-blocking. It returns after the velocity command is accepted,
            not after the command duration has completed.
        """
    def relocalize(
        self, init_pose: typing.Annotated[numpy.typing.ArrayLike, numpy.float64]
    ) -> tuple:
        """
        Perform relocalization to re-estimate the robot's pose in the map frame.

        Parameters:
            init_pose (array): Initial pose estimate [x, y, z, qx, qy, qz, qw], map frame (meters, quaternion).

        Returns:
            tuple: (success: bool, status_string: str)
                - success: True if relocalization succeeded.
                - status_string: Status string (SUCCESS, FAIL, etc.).
        """
    def remove_bounding_box(self, box_tag: typing.SupportsInt) -> tuple:
        """
        Remove a bounding box from navigation obstacle filtering.

        Parameters:
            box_tag (int): SDK box tag to remove.

        Returns:
            tuple: (success: bool, status_string: str)
        """
    def set_navigation_arrival_threshold(
        self, threshold: typing.Annotated[numpy.typing.ArrayLike, numpy.float64]
    ) -> tuple:
        """
        Set navigation arrival threshold.

        Parameters:
            threshold (array): [x_error, y_error, yaw_error].

        Returns:
            tuple: (success: bool, status_string: str)
        """
    def set_navigation_kinematics_limits(
        self,
        vel_limit: typing.Annotated[numpy.typing.ArrayLike, numpy.float64],
        acc_limit: typing.Annotated[numpy.typing.ArrayLike, numpy.float64],
        jerk_limit: typing.Annotated[numpy.typing.ArrayLike, numpy.float64],
    ) -> tuple:
        """
        Set navigation velocity, acceleration, and jerk limits.

        Parameters:
            vel_limit (array): [vx_limit, vy_limit, vyaw_limit].
            acc_limit (array): [ax_limit, ay_limit, ayaw_limit].
            jerk_limit (array): [jx_limit, jy_limit, jyaw_limit].

        Returns:
            tuple: (success: bool, status_string: str)
        """
    def set_navigation_target(
        self,
        target: Pose,
        frame: str = "map",
        speed_ratio: typing.SupportsFloat = 1.0,
        enable_collision_check: bool = True,
    ) -> TaskHandle:
        """
        Submit a dynamic navigation target asynchronously.

        This API is designed for targets that may change over time, such
        as dynamic tracking or remote control. When a new target is
        submitted, the previous target may be preempted and the
        navigation system will re-plan toward the latest target.

        To keep the navigation stable, do not call this API at a very
        high rate. Frequent updates may cause planning jitter and reduce
        motion smoothness.

        Parameters:
            target (Pose): Target pose [x, y, z, qx, qy, qz, qw].
            frame (str): Target frame identifier, supported values: "map", "base_link".
            speed_ratio (float): Velocity scaling factor in (0, 1.0].
            enable_collision_check (bool): Whether to enable collision checking and avoidance.

        Returns:
            TaskHandle: Submitted task id, request result, and message.
        """
    def set_navigation_timeout(self, timeout_s: typing.SupportsFloat) -> tuple:
        """
        Set navigation timeout.

        Parameters:
            timeout_s (float): Navigation timeout in seconds. A value less than or equal to 0 disables
                the navigation motion time limit.

        Returns:
            tuple: (success: bool, status_string: str)
        """
    def set_navigation_velocity_limit(
        self, vel_limit: typing.Annotated[numpy.typing.ArrayLike, numpy.float64]
    ) -> tuple:
        """
        Set navigation velocity limit.

        Parameters:
            vel_limit (array): [vx_limit, vy_limit, vyaw_limit].

        Returns:
            tuple: (success: bool, status_string: str)
        """
    def stop_navigation(self) -> tuple:
        """
        Stop the current navigation task and bring the robot to a halt.

        Parameters:
            None

        Returns:
            tuple: (success: bool, status_string: str)
                - success: True if stop command was successfully sent.
                - status_string: Status string.
        """

class GalbotOneFoxtrotSensor:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | LEFT_WRIST_FORCE | Left wrist force sensor |
    | RIGHT_WRIST_FORCE | Right wrist force sensor |
    """

    LEFT_WRIST_FORCE: typing.ClassVar[
        GalbotOneFoxtrotSensor
    ]  # value = <GalbotOneFoxtrotSensor.LEFT_WRIST_FORCE: 0>
    RIGHT_WRIST_FORCE: typing.ClassVar[
        GalbotOneFoxtrotSensor
    ]  # value = <GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE: 1>
    __members__: typing.ClassVar[
        dict[str, GalbotOneFoxtrotSensor]
    ]  # value = {'LEFT_WRIST_FORCE': <GalbotOneFoxtrotSensor.LEFT_WRIST_FORCE: 0>, 'RIGHT_WRIST_FORCE': <GalbotOneFoxtrotSensor.RIGHT_WRIST_FORCE: 1>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class GalbotPerception:
    """
    Perception module interface. Use get_instance(machine_type) for the platform singleton; G1 and S1 are supported.
    """
    def get_latest_result(self, module: PerceptionModule) -> tuple:
        """
        Return the latest cached result for the module without blocking.

        Args:
            module (PerceptionModule): Perception module.

        Returns:
            tuple[bool, DetectionResult]: (success, result). success is True if a result is available, False if none.
        """
    def init(self, enabled_modules: collections.abc.Set[PerceptionModule]) -> bool:
        """
        Initialize perception and load models for the selected modules.

        Args:
            enabled_modules (set[PerceptionModule]): Set of perception modules to enable.

        Returns:
            bool: True if every requested module loaded successfully.
        """
    def run_once(self, module: PerceptionModule) -> bool:
        """
        Run a single inference for the given module.

        Note:
            After init, wait ~10s for models to be ready before calling run_once.

        Args:
            module (PerceptionModule): Perception module to run.

        Returns:
            bool: True if the command was sent successfully.
        """
    def wait_for_new_result(
        self, module: PerceptionModule, timeout_s: typing.SupportsFloat = 5.0
    ) -> bool:
        """
        Block until the module produces a new result, or timeout. Use with run_once to fetch the latest output.

        Args:
            module (PerceptionModule): Perception module.
            timeout_s (float): Timeout in seconds (default 5.0).

        Returns:
            bool: True if new data arrived, False on timeout.
        """

class GalbotRobot:
    def acquire_controller(self, controller_name: str) -> ControlStatus:
        """
        Acquire a controller for a specific joint group.

        Requests the specified controller to take hardware authority. This uses the
        same WBCS switch request as switch_controller.

        Parameters:
            controller_name (str): Controller name, for example "left_arm_pvt_ctrl".

        Returns:
            ControlStatus: Result of the operation.
        """
    def check_trajectory_execution_status(
        self, joint_groups: collections.abc.Sequence[str] = []
    ) -> list[TrajectoryControlStatus]:
        """
        Get trajectory execution status for specified joint groups.

        Parameters:
            joint_groups (List[str]): Joint groups to query (optional).

        Returns:
            List[TrajectoryControlStatus]: List of trajectory execution statuses.
        """
    def clear_end_effector_command(self) -> ControlStatus:
        """
        Clear WBC end-effector task commands.

        Returns:
            ControlStatus: Command publishing result.
        """
    def destroy(self) -> None:
        """
        Clean up system and middleware resources.

        This MUST be called as the final step of the shutdown sequence:
        request_shutdown() -> wait_for_shutdown() -> destroy().

        After this method returns, the SDK is in a terminal state and cannot be
        re-initialized in the same process. The destroy() method clears all internal
        resources (middleware, readers/writers, etc.). To use the SDK again, exit
        the current process and launch a new one.

        Parameters:
            None

        Returns:
            None
        """
    def execute_joint_trajectory(
        self, trajectory: Trajectory, is_blocking: bool = True
    ) -> ControlStatus:
        """
        Execute trajectory data.

        Parameters:
            trajectory (Trajectory): Trajectory data to execute.
                                     Either `joint_groups` or `joint_names` must be specified;
                                     returns INVALID_INPUT if both are empty.
            is_blocking (bool): Whether to block until trajectory execution completes (optional, default: True).

        Returns:
            ControlStatus: Trajectory execution/sending result.
        """
    def get_active_controller(self, group_name: str) -> str:
        """
        Get the active controller for a joint group name.

        This returns the controller last known by this SDK instance. It is initialized
        with the model default controller and updated after successful SDK controller
        management calls.

        Parameters:
            group_name (str): The joint group name to query.

        Returns:
            str: Active controller name for the group.
        """
    def get_bms_information(self) -> dict:
        """
        Get EMS/BMS information.

        Parameters:
            None

        Returns:
            dict: Dictionary containing the following keys:
                - 'voltage': Battery voltage in V
                - 'current': Battery current in A
                - 'battery_level': Battery level in %
                - 'temperature': Battery temperature in C
                - 'charging_status': Charging status (bool)
                - 'health_status': Health status (bool)
                - 'capacity': Remaining capacity in Ah

            Returns empty dictionary on failure.
        """
    def get_camera_intrinsic(self, camera_id: SensorType) -> dict:
        """
        Get camera intrinsic parameters.

        Parameters:
            camera_id (SensorType): Camera sensor ID to query.

        Returns:
            dict: Dictionary containing camera intrinsic parameters.
                - header: Message header with timestamp and frame information
                - height: Image height in pixels
                - width: Image width in pixels
                - distortion_model: Distortion model, e.g., 'plumb_bob'
                - D: Distortion coefficients (list of float)
                - K: Camera intrinsic matrix (list of 9 float)
                - binning_x: Horizontal binning factor
                - binning_y: Vertical binning factor
                - roi: Region of interest (list of int)
                - camera_type: camera type
                ...
                Returns empty dictionary on failure.
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
    def get_device_information(self) -> dict:
        """
        Get device information including model, serial number, firmware version, hardware version, and manufacturer.

        Parameters:
            None

        Returns:
            dict: Dictionary containing the following keys:
                - 'model': Device model name or identifier (str)
                - 'serial_number': Unique serial number for device identification (str)
                - 'firmware_version': System firmware version string (str)
                - 'hardware_version': Hardware version or revision number (str)
                - 'manufacturer': Manufacturer name or company identifier (str)

            Returns empty dictionary on failure.
        """
    def get_dexhand_state(
        self, end_effector: str, dexhand_type: DexHandType = ...
    ) -> typing.Any:
        """
        Get dexhand state.

        Parameters:
            end_effector (str): Dexhand name, e.g. "left_dexhand" or "right_dexhand".
            dexhand_type (DexHandType): Dexhand model type (optional, default: INSPIRE).

        Returns:
            DexhandState | None: Dexhand state on success (use .joint_state; .force_sensor_map for Sharpa), otherwise None.
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
    def get_frame_names(self) -> list[str]:
        """
        Get all frame names.

        Parameters:
            None

        Returns:
            list(str): List of all frame names.
        """
    def get_gripper_state(self, end_effector: str) -> GripperState:
        """
        Get gripper state.

        Parameters:
            end_effector (str): Gripper name, e.g. "left_gripper" or "right_gripper".

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
    def get_ir_data(self, camera_id: SensorType) -> dict:
        """
        Get latest infrared image data from specified IR camera.

        Parameters:
            camera_id (SensorType): IR camera sensor ID to query.
                Valid values: LEFT_ARM_INFRA_CAMERA_1, LEFT_ARM_INFRA_CAMERA_2,
                              RIGHT_ARM_INFRA_CAMERA_1, RIGHT_ARM_INFRA_CAMERA_2

        Returns:
            dict: Dictionary containing the following keys:
                - 'header': Message header with timestamp and frame information
                - 'format': Image format, e.g., 'mono8; jpeg compressed mono8'
                - 'data': Compressed grayscale image binary data (bytes)

            Returns empty dictionary if camera is not enabled, ir_enabled is false,
            or no data has been received yet.
        """
    def get_joint_group_names(self) -> list[str]:
        """
        Get available joint group names for the robot.

        Parameters:
            None

        Returns:
            List[str]: Array of available joint group names, returns empty list on failure.
        """
    def get_joint_names(
        self,
        only_active_joint: bool = True,
        joint_groups: collections.abc.Sequence[str] = [],
    ) -> list[str]:
        """
        Get robot joint names.

        Parameters:
            only_active_joint (bool): Whether to only get active joints (optional, default: True).
            joint_groups (List[str]): Joint groups (optional).

        Returns:
            List[str]: Array of corresponding joint names.
        """
    def get_joint_positions(
        self,
        joint_groups: collections.abc.Sequence[str] = [],
        joint_names: collections.abc.Sequence[str] = [],
    ) -> list[float]:
        """
        Get joint positions.

        Parameters:
            joint_groups (List[str]): Joint groups to query (optional).
            joint_names (List[str]): Specific joint names, takes priority over joint_groups (optional).

        Returns:
            List[float]: Array of corresponding joint angles in radians.

        .. note::
            Return order:
            - **joint_names specified**: Returns in the exact order of joint_names.
            - **Only joint_groups specified**: Returns in the order groups are defined.
            - **Both empty** (machine-dependent body-joint order):
              - G1: chassis, head, left_arm, right_arm, leg
              - S1: torso, head, left_arm, right_arm
        """
    def get_joint_states(
        self,
        joint_groups: collections.abc.Sequence[str] = [],
        joint_names: collections.abc.Sequence[str] = [],
    ) -> list[JointState]:
        """
        Get real-time joint states.

        Parameters:
            joint_groups (List[str]): Joint groups to query (optional).
            joint_names (List[str]): Specific joint names, takes priority over joint_groups (optional).

        Returns:
            List[JointState]: Real-time state data for corresponding joints.

        .. note::
            Return order:
            - **joint_names specified**: Returns in the exact order of joint_names.
            - **Only joint_groups specified**: Returns in the order groups are defined.
            - **Both empty** (machine-dependent body-joint order):
              - G1: chassis, head, left_arm, right_arm, leg
              - S1: torso, head, left_arm, right_arm
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
    def get_log_information(
        self, timewindow_s: typing.SupportsInt, log_level: LogLevel
    ) -> dict:
        """
        Get log information.

        Parameters:
            timewindow_s (int64_t): Time window in seconds.
            log_level (int): Log level.

        Returns:
            dict: Dictionary containing the following keys:
                - 'level': Log level
                - 'message': Log message
            Returns empty dictionary on failure.
        """
    def get_odom(self) -> dict:
        """
        Get odometry information.

        Parameters:
            None

        Returns:
            dict: Dictionary containing the following keys:
                - 'timestamp_ns': Timestamp in nanoseconds
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
    def get_sensor_extrinsic(
        self, sensor_id: SensorType, reference_frame: str = "base_link"
    ) -> tuple:
        """
        Query sensor extrinsic transform (TF) from reference frame to sensor frame.

        Parameters:
            sensor_id (SensorType): Sensor enum to query.
            reference_frame (str): Name of the reference coordinate frame (frame to transform from). Default is "base_link".

        Returns:
            tuple(List[float], int): Transform [x, y, z, qx, qy, qz, qw] and timestamp. Returns empty list on failure.
        """
    def get_suction_cup_state(self, end_effector: str) -> SuctionCupState:
        """
        Get suction cup state.

        Parameters:
            end_effector (str): Suction cup name, e.g. "left_suction_cup" or "right_suction_cup".

        Returns:
            SuctionCupState: Suction cup state information.
        """
    def get_synced_observation(
        self,
        cameras: collections.abc.Sequence[SensorType],
        with_joint_state: bool = True,
    ) -> SyncedObservation:
        """
        Get timestamp-synchronized observation as a typed SyncedObservation object.

        Parameters:
            cameras (list[SensorType]): Cameras to synchronize. First item is anchor.
            with_joint_state (bool): Whether to include nearest-neighbor joint state.

        Returns:
            SyncedObservation | None:
                - rgb_data_map: dict[SensorType, RgbData]
                - depth_data_map: dict[SensorType, DepthData]
                - joint_state: JointStateMessage | None
            Returns None on failure.
        """
    def get_transform(
        self,
        target_frame: str,
        source_frame: str,
        timestamp_ns: typing.SupportsInt = 0,
        timeout_ms: typing.SupportsInt = 100,
    ) -> tuple:
        """
        Query coordinate frame transform (TF).

        Parameters:
            target_frame (str): Target coordinate frame (e.g., map, base_link, imu_base_link; actual list is from get_frame_names()).
            source_frame (str): Source coordinate frame (e.g., map, base_link, imu_base_link; actual list is from get_frame_names()).
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
    def get_volume(self) -> float:
        """
        Get current system global volume value.

        Parameters:
            None

        Returns:
            float: Current volume value, range 0.0 to 100.0.
        """
    def get_wbc_end_effector_poses(self) -> dict[str, list[float]]:
        """
        Get WBC end effector poses (lee_pose, ree_pose, head_pose).

        Returns:
            dict: Pose vectors [x, y, z, qx, qy, qz, qw] per key, or empty lists if unavailable.
        """
    def init(
        self,
        enable_sensor_set: collections.abc.Set[SensorType] = ...,
        enable_sync_mode: bool = False,
    ) -> bool:
        """
        Initialize the robot control system (hardware communication, middleware, sensor interfaces).
        Only sensors in enable_sensor_set are initialized; specify only required sensors to reduce overhead.

        This method should only be called once at program startup. Calling it multiple
        times without calling destroy() will not error, but only the first call has effect.

        Parameters:
            enable_sensor_set (set[SensorType]): Set of sensors to enable. Empty set uses default sensors.
            enable_sync_mode (bool): Enable internal sync buffers for timestamp-aligned observation APIs.

        Returns:
            bool: True if initialization succeeded; False otherwise.
        """
    def is_running(self) -> bool:
        """
        Check if the system is running.

        Parameters:
            None

        Returns:
            bool: True if system is running, False if shutdown signal captured and preparing to shutdown.
        """
    def publish_target(self, target: SingoriXTarget) -> ControlStatus:
        """
        Publish a raw SingoriXTarget through the WBC publish channel.

        This is the advanced high-frequency path. Construct a SingoriXTarget directly,
        then call this interface to send it to the low-level controller without waiting
        for a service response. The SDK performs only basic structural validation.

        Parameters:
            target (SingoriXTarget): SDK mirror target containing group-space and/or task-space trajectories.

        Returns:
            ControlStatus: Local validation / publish result.
        """
    def release_controller(self, group_name: str = "all") -> ControlStatus:
        """
        Release a controller for a specific joint group.

        Releases the specified controller for the given joint group. This puts the
        controller in a released state where it stops sending commands to the joints.
        This is the opposite operation of acquire_controller.

        Parameters:
            group_name (str): Name of the joint group (default: "all").

        Returns:
            ControlStatus: Result of the operation.
        """
    def reload_controller(self, group_name: str = "all") -> ControlStatus:
        """
        Reload a controller for a specific joint group.

        Parameters:
            group_name (str): Name of the joint group (default: "all").

        Returns:
            ControlStatus: Result of the operation.
        """
    def request_shutdown(self) -> None:
        """
        Request graceful shutdown of the robot system.

        Sends an async shutdown signal to all modules (WBC controller, middleware nodes,
        sensor data loops, etc.). This is the first step of the shutdown sequence and
        does NOT block — follow with wait_for_shutdown() and destroy().

        This is a singleton instance and can only be initialized once per process. After
        destroy() is called, the SDK cannot be re-initialized. To restart, exit the
        current process and launch a new one.

        Parameters:
            None

        Returns:
            None
        """
    def request_target(self, target: SingoriXTarget) -> ErrorInfo:
        """
        Request execution of a raw SingoriXTarget through the WBC service channel.

        This is the advanced request path. The SDK performs request-side runtime error
        screening, sends the target through the middleware client, and returns the
        ErrorInfo service payload. A return value of None means the client was unavailable,
        disconnected, timed out, or returned an empty response.

        Parameters:
            target (SingoriXTarget): SDK mirror target containing group-space and/or task-space trajectories.

        Returns:
            ErrorInfo | None: Error response payload or None when no valid response was received.
        """

    def set_base_pose(
        self,
        base_pose: Pose,
        is_blocking: bool = True,
        timeout_s: typing.SupportsFloat = 15.0,
    ) -> ControlStatus:
        """
        Set base pose command using Pose.

        Parameters:
            base_pose (Pose): Target base pose.
            is_blocking (bool): Whether to block until command execution completes (optional, default: True).
            timeout_s (float): Blocking timeout in seconds (optional, default: 15.0).

        Returns:
            ControlStatus: Command sending result.
        """

    def set_base_pose(
        self,
        x: typing.SupportsFloat,
        y: typing.SupportsFloat,
        yaw: typing.SupportsFloat,
        frame_id: str = "rel(0)",
        reference_frame_id: str = "odom",
        is_blocking: bool = True,
        timeout_s: typing.SupportsFloat = 15.0,
    ) -> ControlStatus:
        """
        Set base pose command with frame ids.

        Parameters:
            x (float): Target x position.
            y (float): Target y position.
            yaw (float): Target yaw (rad).
            frame_id (str): Frame id. Current recommended value: "rel(0)", which means the
                x/y/yaw target is interpreted relative to the current base pose.
                "base_link", "odom", and "map" are retained for compatibility, but are not
                recommended for current use and may be changed or removed in a future update.
                Default "rel(0)".
            reference_frame_id (str): Reference frame id ("odom"/"map"). Default "odom".
            is_blocking (bool): Whether to block until command execution completes (optional, default: True).
            timeout_s (float): Blocking timeout in seconds (optional, default: 15.0).

        Returns:
            ControlStatus: Command sending result.
        """

    def set_base_pose(
        self,
        x: typing.SupportsFloat,
        y: typing.SupportsFloat,
        yaw: typing.SupportsFloat,
        frame_id: str,
        reference_frame_id: str,
        time_from_start_s: typing.SupportsFloat,
        is_blocking: bool = True,
        timeout_s: typing.SupportsFloat = 15.0,
    ) -> ControlStatus:
        """
        Set base pose (x, y, yaw) with explicit interpolation time.

        Parameters:
            x (float): Target x position (meters).
            y (float): Target y position (meters).
            yaw (float): Target yaw (radians).
            frame_id (str): Frame id of target. Current recommended value: "rel(0)", which means
                the x/y/yaw target is interpreted relative to the current base pose.
                "base_link", "odom", and "map" are retained for compatibility, but are not
                recommended for current use and may be changed or removed in a future update.
            reference_frame_id (str): Reference frame id ("odom"/"map").
            time_from_start_s (float): Chassis pose interpolation time (seconds).
            is_blocking (bool): Whether to block until command execution completes (optional, default: True).
            timeout_s (float): Request timeout in seconds (optional, default: 15.0).

        Returns:
            ControlStatus: Command sending result.
        """
    def set_base_velocity(
        self,
        linear_velocity: typing.Annotated[
            collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"
        ],
        angular_velocity: typing.Annotated[
            collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"
        ],
        duration_s: typing.SupportsFloat = 0.0,
    ) -> ControlStatus:
        """
        Set base velocity command.

        Parameters:
            linear_velocity (List[float]): Linear velocity command [vx, vy, vz] in m/s.
            angular_velocity (List[float]): Angular velocity command [wx, wy, wz] in rad/s.
            duration_s (float): Duration in seconds before auto-stop (optional, default: 0.0).
                                If <= 0.0, no automatic stop is performed.

        Returns:
            ControlStatus: Command sending result.
        """
    def set_dexhand_command(
        self,
        end_effector: str,
        dexhand_command: collections.abc.Sequence[JointCommand],
        dexhand_type: DexHandType = ...,
        is_blocking: bool = True,
    ) -> ControlStatus:
        """
        Set dexhand command.

        Parameters:
            end_effector (str): Dexhand name, e.g. "left_dexhand" or "right_dexhand".
            dexhand_command (List[JointCommand]): Joint command list for the dexhand.
            dexhand_type (DexHandType): Dexhand model type (optional, default: INSPIRE).
            is_blocking (bool): Whether to block until action completes (optional, default: True).

        Returns:
            ControlStatus: Command execution/sending result.
        """
    def set_end_effector_command(
        self,
        poses: collections.abc.Sequence[collections.abc.Sequence[typing.SupportsFloat]],
        end_effector_frames: collections.abc.Sequence[str],
        reference_frames: collections.abc.Sequence[str] = [],
    ) -> ControlStatus:
        """
        Set WBC end-effector pose commands.

        Parameters:
            poses (List[List[float]]): One pose per end effector; each row is
                [x, y, z, qx, qy, qz, qw] (meters, quaternion xyzw).
            end_effector_frames (List[str]): Target frame id per pose (e.g. link names).
            reference_frames (List[str], optional): Reference frame per pose. Omit or pass [] to use
                ``"world"`` for every pose. Otherwise length must match ``poses``. Common values:
                ``"world"`` (default)

        Returns:
            ControlStatus: Command publishing result.
        """
    def set_gripper_command(
        self,
        end_effector: str,
        width_m: typing.SupportsFloat,
        velocity_mps: typing.SupportsFloat = 0.03,
        effort: typing.SupportsFloat = 5,
        is_blocking: bool = True,
    ) -> ControlStatus:
        """
        Set gripper command.

        Parameters:
            end_effector (str): Gripper name, e.g. "left_gripper" or "right_gripper".
            width_m (float): Target gripper width in meters. G1 gripper width range is 0 to 0.12 m. S1 long-stroke gripper
            width range is 0.007 to 0.11 m. S1 short-stroke gripper width range is 0.007 to 0.076 m.
            velocity_mps (float): Gripper motion speed in m/s (optional, default: 0.03). The value range is greater than 0 and
            less than or equal to 0.2 m/s.
            effort (float): Gripper effort in Nm (optional, default: 5). The value range is greater than 0 and
            less than or equal to 100.
            is_blocking (bool): Whether to block until action completes (optional, default: True).

        Returns:
            ControlStatus: Command execution/sending result.
        """
    def set_joint_commands(
        self,
        joint_commands: collections.abc.Sequence[JointCommand],
        joint_groups: collections.abc.Sequence[str] = [],
        joint_names: collections.abc.Sequence[str] = [],
        time_from_start_s: typing.SupportsFloat = 0.0,
    ) -> ControlStatus:
        """
        Set joint commands.
        This interface is suitable for high-frequency control usage
        For standard joints (legs, head, arms, etc.), only the position field in each JointCommand will be effective;
        other fields such as velocity, current/effort, are ignored.
        For gripper joints, the position field represents gripper width and both velocity and effort fields are supported and effective.

        Parameters:
            joint_commands (List[JointCommand]): List of joint commands to control.
            joint_groups (List[str]): Joint groups to control. Must not be empty if `joint_names` is also empty.
            joint_names (List[str]): Specific joint names, takes priority over `joint_groups`. Must not be empty if `joint_groups` is also empty.
            time_from_start_s (float): Execution will begin after time_start_s seconds.(optional, default: 0.0).

        Returns:
            ControlStatus: Result of command execution.
        """
    def set_joint_commands_batch(self, trajectory: Trajectory) -> ControlStatus:
        """
        Set joint commands in batch mode (non-blocking).

        Sets multiple joint command trajectory points in real-time control mode,
        supporting one-time submission of trajectory control commands for multiple
        time points. Provides a non-blocking high-frequency trajectory execution
        interface. Similar to set_joint_commands but supports batch trajectory control,
        suitable for scenarios such as VLA inference batch output.

        Parameters:
            trajectory (Trajectory): Trajectory data structure containing waypoints with joint commands.
                                   Either `joint_groups` or `joint_names` must be specified;
                                   returns INVALID_INPUT if both are empty.
                                   Each TrajectoryPoint contains time_from_start and a list of JointCommand.
                                   JointCommand includes position (rad), velocity (rad/s), acceleration (rad/s²),
                                   effort (N·m), Kp (position gain), and Kd (velocity gain).

        Returns:
            ControlStatus: Command submission result. Returns immediately without waiting for execution completion (non-blocking).
        """
    def set_joint_positions(
        self,
        joint_positions: collections.abc.Sequence[typing.SupportsFloat],
        joint_groups: collections.abc.Sequence[str] = [],
        joint_names: collections.abc.Sequence[str] = [],
        is_blocking: bool = True,
        speed_rad_s: typing.SupportsFloat = 0.2,
        timeout_s: typing.SupportsFloat = 15.0,
    ) -> ControlStatus:
        """
        Set target joint positions for specified joint groups.

        Parameters:
            joint_positions (List[float]): Array of joint angles in radians.
            joint_groups (List[str]): Joint groups to control. Must not be empty if `joint_names` is also empty.
            joint_names (List[str]): Specific joint names, takes priority over `joint_groups`. Must not be empty if `joint_groups` is also empty.
            is_blocking (bool): Whether to block until command execution completes (optional, default: True).
            speed_rad_s (float): Maximum joint speed in rad/s (optional, default: 0.2).
            timeout_s (float): Maximum blocking wait time in seconds (optional, default: 15.0).

        Returns:
            ControlStatus: Execution result status.
        """
    def set_suction_cup_command(
        self, end_effector: str, activate: bool
    ) -> ControlStatus:
        """
        Set suction cup command.

        Parameters:
            end_effector (str): Suction cup name, e.g. "left_suction_cup" or "right_suction_cup".
            activate (bool): Whether to activate the suction cup.

        Returns:
            ControlStatus: Command sending result.
        """
    def set_volume(self, volume: typing.SupportsFloat) -> bool:
        """
        Set system global volume value.

        Parameters:
            volume (float): Target volume value, range 0.0 to 100.0.

        Returns:
            bool: Returns the volume setting result. True indicates the volume was set successfully, False indicates the volume setting failed.
        """
    def start_controller(self, group_name: str = "all") -> ControlStatus:
        """
        Start a controller for a specific joint group.

        Starts the specified controller for the given joint group. This puts the
        controller in an active state where it can send commands to the joints.
        This is the opposite operation of stop_controller.

        Parameters:
            group_name (str): Name of the joint group (default: "all").

        Returns:
            ControlStatus: Result of the operation.
        """
    def start_microphone_stream_input(
        self,
        callback: typing.Callable,
        chunk_size: typing.SupportsInt = 2560,
        use_raw_audio: bool = False,
    ) -> str:
        """
        Start microphone streaming audio input.

        Parameters:
            callback (callable): Audio data callback function with signature: void(dict audio_data).
                                The audio_data dict fields (see AudioData):
                                - 'header' (dict): Message header.
                                    - 'timestamp_ns' (int): Data acquisition timestamp (nanoseconds since epoch).
                                    - 'frame_id' (str): Stream or source frame identifier.
                                - 'type' (str): Audio data type identifier. Possible values:
                                    - 'waken_up': Wake-up event; format is 'json'; data is a UTF-8 JSON string.
                                    - 'denoise_chunk': Denoised audio chunk; format is 'pcm'; data is PCM binary.
                                    - 'vad_begin': VAD start marker; data is empty.
                                    - 'vad_chunk': Audio during VAD; format is 'pcm'; data is PCM binary.
                                    - 'vad_end': VAD end marker; data is empty.
                                - 'format' (str): How to interpret 'data':
                                    - 'pcm': 16000 Hz, 16-bit, mono PCM.
                                    - 'json': UTF-8 encoded JSON text.
                                - 'data' (bytes): Binary payload. For 'pcm', each 80 ms chunk is 2560 bytes;
                                  for 'json', length varies or may be empty for marker-only messages.
            chunk_size (int): Audio data chunk size in bytes, default value 2560. Dynamic configuration not supported yet
            use_raw_audio (bool): Whether to use raw audio, default false. Dynamic configuration not supported yet.

        Returns:
            str: Stream ID used to identify the audio input stream.
        """
    def stop_audio_stream_output(self, stream_id: str = "") -> None:
        """
        Stop the specified audio output stream or all active audio output streams playback.

        Parameters:
            stream_id (str): Audio output stream ID to stop. Empty string means stop all active audio output streams (optional, default: "").

        Returns:
            None
        """
    def stop_base(self) -> ControlStatus:
        """
        Stop base motion.

        Parameters:
            None

        Returns:
            ControlStatus: Command sending result.
        """
    def stop_controller(self, group_name: str = "all") -> ControlStatus:
        """
        Stop a controller for a specific joint group.

        Stops the specified controller for the given joint group. This puts the
        controller in a stopped state where it no longer sends commands to the joints.
        This is the opposite operation of start_controller.

        Parameters:
            group_name (str): Name of the joint group (default: "all").

        Returns:
            ControlStatus: Result of the operation.
        """
    def stop_microphone_stream_input(self, stream_id: str = "") -> None:
        """
        Stop the specified microphone streaming audio input.

        Parameters:
            stream_id (str): Audio input stream ID to stop. Empty string stops all active streams (optional, default: "").

        Returns:
            None
        """
    def stop_trajectory_execution(self) -> ControlStatus:
        """
        Stop all currently executing trajectories.

        Parameters:
            None

        Returns:
            ControlStatus: Command sending result.
        """
    def switch_controller(self, controller_name: str) -> ControlStatus:
        """
        Switch controller for a specific joint group.

        Parameters:
            controller_name (str): Controller name, for example "chassis_pose_ctrl".

        Returns:
            ControlStatus: Result of the operation.
        """
    def wait_for_shutdown(self) -> None:
        """
        Wait for all modules to finish shutting down.

        Blocks until all background threads (middleware, sensor callbacks, WBC loops)
        have stopped gracefully. Must be called after request_shutdown() and before destroy().

        Parameters:
            None

        Returns:
            None
        """
    def write_audio_stream_output(self, audio_chunk: str, stream_id: str = "") -> bool:
        """
        Write PCM format audio data chunk to audio output stream for real-time playback.

        Parameters:
            audio_chunk (bytes or str): Audio data chunk in PCM format (16000 Hz, 16-bit little-endian), single channel.
            stream_id (str): Audio stream ID to distinguish different audio sources. Empty string means use default stream (optional, default: "").

        Returns:
            bool: True if audio data has been successfully written and playback task issued, False if write failed.
        """

    def zero_whole_body_and_base(
        self,
        base_zero_pose: Pose,
        is_blocking: bool = True,
        leg_head_speed_rad_s: typing.SupportsFloat = 0.2,
        leg_head_timeout_s: typing.SupportsFloat = 15.0,
        params: Parameter = None,
    ) -> tuple[MotionStatus, ControlStatus]:
        """
        One-key zero: move whole-body joints to zero and base to zero pose.
        """

    def zero_whole_body_and_base(
        self,
        frame_id: str = "odom",
        reference_frame_id: str = "odom",
        is_blocking: bool = True,
        leg_head_speed_rad_s: typing.SupportsFloat = 0.2,
        leg_head_timeout_s: typing.SupportsFloat = 15.0,
        params: Parameter = None,
    ) -> tuple[MotionStatus, ControlStatus]:
        """
        One-key zero: move whole-body joints to zero and base (x,y,yaw) to zero with frames.

        Parameters:
            frame_id (str): Frame id ("base_link"/"odom"/"map"). Default "odom".
            reference_frame_id (str): Reference frame id ("odom"/"map"). Default "odom".
            is_blocking (bool): Whether to block on joint zeroing (optional, default: True).
            leg_head_speed_rad_s (float): Leg/head joint speed limit in rad/s (optional, default: 0.2).
            leg_head_timeout_s (float): Leg/head blocking timeout in seconds (optional, default: 15.0).
            params (Parameter | None): Motion planning parameters (optional, default: None).
        """

class GripperState:
    """
    Gripper state information
    """
    def __init__(self) -> None: ...
    @property
    def effort(self) -> float:
        """
        Gripper torque (newton-meters)
        """
    @effort.setter
    def effort(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def is_moving(self) -> bool:
        """
        Whether currently moving
        """
    @is_moving.setter
    def is_moving(self, arg0: bool) -> None: ...
    @property
    def joint_positions(self) -> list[float]:
        """
        Joint positions array
        """
    @joint_positions.setter
    def joint_positions(
        self, arg0: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def velocity(self) -> float:
        """
        Gripper velocity (meters/second)
        """
    @velocity.setter
    def velocity(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def width(self) -> float:
        """
        Gripper width (meters)
        """
    @width.setter
    def width(self, arg0: typing.SupportsFloat) -> None: ...

class GroupCommand:
    """
    Group-space trajectory point
    """
    def __init__(self) -> None: ...
    @property
    def joint_commands(self) -> list[JointCommand]:
        """
        Joint commands at this point
        """
    @joint_commands.setter
    def joint_commands(self, arg0: collections.abc.Sequence[JointCommand]) -> None: ...
    @property
    def time_from_start_s(self) -> float:
        """
        Time from trajectory start in seconds
        """
    @time_from_start_s.setter
    def time_from_start_s(self, arg0: typing.SupportsFloat) -> None: ...

class Header:
    """
    Message header
    """
    def __init__(self) -> None: ...
    @property
    def frame_id(self) -> str:
        """
        Frame ID
        """
    @frame_id.setter
    def frame_id(self, arg0: str) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds since epoch)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...

class IKSolverConfig:
    def __init__(self) -> None: ...
    def get_col_aware_ik_joint_limit_bias(self) -> float: ...
    def get_col_aware_ik_timeout(self) -> float: ...
    def get_enable_collision_check_log(self) -> bool: ...
    def get_rotation_eps(self) -> typing.Annotated[list[float], "FixedSize(3)"]: ...
    def get_seed_type(self) -> SeedType: ...
    def get_translation_eps(self) -> typing.Annotated[list[float], "FixedSize(3)"]: ...
    def print(self) -> None: ...
    def set_col_aware_ik_joint_limit_bias(self, bias: typing.SupportsFloat) -> None: ...
    def set_col_aware_ik_timeout(self, timeout: typing.SupportsFloat) -> None: ...
    def set_enable_collision_check_log(self, enable: bool) -> None: ...
    def set_rotation_eps(
        self,
        eps: typing.Annotated[
            collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"
        ],
    ) -> None: ...
    def set_seed_type(self, type: SeedType) -> None: ...
    def set_translation_eps(
        self,
        eps: typing.Annotated[
            collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"
        ],
    ) -> None: ...

class ImuData:
    """
    IMU data
    """
    def __init__(self) -> None: ...
    @property
    def accel(self) -> Vector3:
        """
        Acceleration Vector3
        """
    @accel.setter
    def accel(self, arg0: Vector3) -> None: ...
    @property
    def gyro(self) -> Vector3:
        """
        Gyroscope Vector3
        """
    @gyro.setter
    def gyro(self, arg0: Vector3) -> None: ...
    @property
    def magnet(self) -> Vector3:
        """
        Magnetometer Vector3
        """
    @magnet.setter
    def magnet(self, arg0: Vector3) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...

class JointCommand:
    """
    Single joint command object
    """
    def __init__(self) -> None: ...
    @property
    def acceleration(self) -> float:
        """
        - `acceleration` (`float`): Joint acceleration
        """
    @acceleration.setter
    def acceleration(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def effort(self) -> float:
        """
        - `effort` (`float`): Joint torque (N·m)
        """
    @effort.setter
    def effort(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def position(self) -> float:
        """
        - `position` (`float`): Joint target position (radians)
        """
    @position.setter
    def position(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def velocity(self) -> float:
        """
        - `velocity` (`float`): Joint velocity (radians/second)
        """
    @velocity.setter
    def velocity(self, arg0: typing.SupportsFloat) -> None: ...

class JointState:
    joint_name: str
    def __init__(self) -> None: ...
    @property
    def acceleration(self) -> float: ...
    @acceleration.setter
    def acceleration(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def current(self) -> float: ...
    @current.setter
    def current(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def effort(self) -> float: ...
    @effort.setter
    def effort(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def position(self) -> float: ...
    @position.setter
    def position(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def timestamp_ns(self) -> int: ...
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def velocity(self) -> float: ...
    @velocity.setter
    def velocity(self, arg0: typing.SupportsFloat) -> None: ...

class JointStateMessage:
    """
    Joint state message
    """
    def __init__(self) -> None: ...
    @property
    def joint_state_vec(self) -> list[JointState]:
        """
        Joint state list
        """
    @joint_state_vec.setter
    def joint_state_vec(self, arg0: collections.abc.Sequence[JointState]) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...

class JointStates(RobotStates):
    def __init__(self) -> None: ...
    def get_type(self) -> RobotStatesType: ...
    def set_joint(self, index: typing.SupportsInt, val: typing.SupportsInt) -> None: ...
    def set_joint_positions(
        self, joints: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    @property
    def joint_positions(self) -> list[float]: ...
    @joint_positions.setter
    def joint_positions(
        self, arg0: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...

class KinematicsBoundary:
    def __init__(self) -> None: ...
    def get_acc_lower_limit(self) -> list[float]: ...
    def get_acc_upper_limit(self) -> list[float]: ...
    def get_chain_name(self) -> str: ...
    def get_jerk_lower_limit(self) -> list[float]: ...
    def get_jerk_upper_limit(self) -> list[float]: ...
    def get_lower_limit(self) -> list[float]: ...
    def get_upper_limit(self) -> list[float]: ...
    def get_vel_lower_limit(self) -> list[float]: ...
    def get_vel_upper_limit(self) -> list[float]: ...
    def print(self) -> None: ...
    def set_acc_lower_limit(
        self, limits: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    def set_acc_upper_limit(
        self, limits: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    def set_chain_name(self, name: str) -> None: ...
    def set_jerk_lower_limit(
        self, limits: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    def set_jerk_upper_limit(
        self, limits: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    def set_lower_limit(
        self, limits: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    def set_upper_limit(
        self, limits: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    def set_vel_lower_limit(
        self, limits: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    def set_vel_upper_limit(
        self, limits: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...

class LidarData:
    """
    LiDAR point cloud data
    """
    def __init__(self) -> None: ...
    @property
    def data(self) -> list[int]:
        """
        Point cloud binary data
        """
    @data.setter
    def data(self, arg0: collections.abc.Sequence[typing.SupportsInt]) -> None: ...
    @property
    def fields(self) -> list[PointField]:
        """
        Point field description list
        """
    @fields.setter
    def fields(self, arg0: collections.abc.Sequence[PointField]) -> None: ...
    @property
    def header(self) -> Header:
        """
        Message header
        """
    @header.setter
    def header(self, arg0: Header) -> None: ...
    @property
    def height(self) -> int:
        """
        Point cloud height
        """
    @height.setter
    def height(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def is_bigendian(self) -> bool:
        """
        Whether big-endian
        """
    @is_bigendian.setter
    def is_bigendian(self, arg0: bool) -> None: ...
    @property
    def is_dense(self) -> bool:
        """
        Whether dense
        """
    @is_dense.setter
    def is_dense(self, arg0: bool) -> None: ...
    @property
    def point_step(self) -> int:
        """
        Bytes per point
        """
    @point_step.setter
    def point_step(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def row_step(self) -> int:
        """
        Bytes per row
        """
    @row_step.setter
    def row_step(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def width(self) -> int:
        """
        Point cloud width
        """
    @width.setter
    def width(self, arg0: typing.SupportsInt) -> None: ...

class LineTrajCheckPrimitive:
    def __init__(self) -> None: ...
    def get_cylinder_prim_radius(self) -> float: ...
    def get_line_check_primitive_type(self) -> PrimitiveType: ...
    def get_line_prim_curvature(self) -> float: ...
    def print(self) -> None: ...
    def set_cylinder_prim_radius(self, radius: typing.SupportsFloat) -> None: ...
    def set_line_check_primitive_type(self, type: PrimitiveType) -> None: ...
    def set_line_prim_curvature(self, curvature: typing.SupportsFloat) -> None: ...

class LogLevel:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | TRACE | Trace level |
    | DEBUG | Debug level |
    | INFO | Info level |
    | WARN | Warning level |
    | ERROR | Error level |
    | CRITICAL | Critical level |
    """

    CRITICAL: typing.ClassVar[LogLevel]  # value = <LogLevel.CRITICAL: 5>
    DEBUG: typing.ClassVar[LogLevel]  # value = <LogLevel.DEBUG: 1>
    ERROR: typing.ClassVar[LogLevel]  # value = <LogLevel.ERROR: 4>
    INFO: typing.ClassVar[LogLevel]  # value = <LogLevel.INFO: 2>
    TRACE: typing.ClassVar[LogLevel]  # value = <LogLevel.TRACE: 0>
    WARN: typing.ClassVar[LogLevel]  # value = <LogLevel.WARN: 3>
    __members__: typing.ClassVar[
        dict[str, LogLevel]
    ]  # value = {'TRACE': <LogLevel.TRACE: 0>, 'DEBUG': <LogLevel.DEBUG: 1>, 'INFO': <LogLevel.INFO: 2>, 'WARN': <LogLevel.WARN: 3>, 'ERROR': <LogLevel.ERROR: 4>, 'CRITICAL': <LogLevel.CRITICAL: 5>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class MachineType:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | G1 | G1 machine type |
    | S1 | S1 machine type |
    """

    G1: typing.ClassVar[MachineType]  # value = <MachineType.G1: 0>
    S1: typing.ClassVar[MachineType]  # value = <MachineType.S1: 1>
    __members__: typing.ClassVar[
        dict[str, MachineType]
    ]  # value = {'G1': <MachineType.G1: 0>, 'S1': <MachineType.S1: 1>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class MotionPlanConfig:
    def __init__(self) -> None: ...
    def create_collision_check_option(self) -> CollisionCheckOption: ...
    def create_ik_solver_config(self) -> IKSolverConfig: ...
    def create_line_traj_check_primitive(self) -> LineTrajCheckPrimitive: ...
    def create_sampler_config(self) -> SamplerConfig: ...
    def create_trajectory_feasibility_check_option(
        self,
    ) -> TrajectoryFeasibilityCheckOption: ...
    def create_trajectory_plan_config(self) -> TrajectoryPlanConfig: ...
    def get_collision_check_option(self) -> CollisionCheckOption: ...
    def get_collision_check_option_ref(self) -> CollisionCheckOption: ...
    def get_feasibility_boundary(self) -> list[KinematicsBoundary]: ...
    def get_hard_joint_limit(self) -> list[KinematicsBoundary]: ...
    def get_ik_joint_limit(self) -> list[KinematicsBoundary]: ...
    def get_ik_solver_config(self) -> IKSolverConfig: ...
    def get_ik_solver_config_ref(self) -> IKSolverConfig: ...
    def get_line_traj_check_primitive(self) -> LineTrajCheckPrimitive: ...
    def get_line_traj_check_primitive_ref(self) -> LineTrajCheckPrimitive: ...
    def get_revert_ik_joint_limit(self) -> bool: ...
    def get_revert_ik_joint_limit_chains(self) -> list[str]: ...
    def get_sampler_config(self) -> SamplerConfig: ...
    def get_sampler_config_ref(self) -> SamplerConfig: ...
    def get_sampler_joint_limit(self) -> list[KinematicsBoundary]: ...
    def get_trajectory_feasibility_check_option(
        self,
    ) -> TrajectoryFeasibilityCheckOption: ...
    def get_trajectory_feasibility_check_option_ref(
        self,
    ) -> TrajectoryFeasibilityCheckOption: ...
    def get_trajectory_plan_config(self) -> TrajectoryPlanConfig: ...
    def get_trajectory_plan_config_ref(self) -> TrajectoryPlanConfig: ...
    def get_update_time(self) -> int: ...
    def print(self) -> None: ...
    def set_collision_check_option(self, option: CollisionCheckOption) -> None: ...
    def set_feasibility_boundary(
        self, boundary: collections.abc.Sequence[KinematicsBoundary]
    ) -> None: ...
    def set_hard_joint_limit(
        self, boundary: collections.abc.Sequence[KinematicsBoundary]
    ) -> None: ...
    def set_ik_joint_limit(
        self, boundary: collections.abc.Sequence[KinematicsBoundary]
    ) -> None: ...
    def set_ik_solver_config(self, config: IKSolverConfig) -> None: ...
    def set_line_traj_check_primitive(
        self, primitive: LineTrajCheckPrimitive
    ) -> None: ...
    def set_revert_ik_joint_limit(self, flag: bool) -> None: ...
    def set_revert_ik_joint_limit_chains(
        self, chains: collections.abc.Sequence[str]
    ) -> None: ...
    def set_sampler_config(self, config: SamplerConfig) -> None: ...
    def set_sampler_joint_limit(
        self, boundary: collections.abc.Sequence[KinematicsBoundary]
    ) -> None: ...
    def set_trajectory_feasibility_check_option(
        self, option: TrajectoryFeasibilityCheckOption
    ) -> None: ...
    def set_trajectory_plan_config(self, config: TrajectoryPlanConfig) -> None: ...
    def set_update_time(self, t: typing.SupportsInt) -> None: ...

class MotionStatus:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | SUCCESS |  |
    | TIMEOUT |  |
    | FAULT |  |
    | INVALID_INPUT |  |
    | INIT_FAILED |  |
    | IN_PROGRESS |  |
    | STOPPED_UNREACHED |  |
    | DATA_FETCH_FAILED |  |
    | PUBLISH_FAIL |  |
    | COMM_DISCONNECTED |  |
    | STATUS_NUM |  |
    | UNSUPPORTED_FUNCRION |  |
    """

    COMM_DISCONNECTED: typing.ClassVar[
        MotionStatus
    ]  # value = <MotionStatus.COMM_DISCONNECTED: 9>
    DATA_FETCH_FAILED: typing.ClassVar[
        MotionStatus
    ]  # value = <MotionStatus.DATA_FETCH_FAILED: 7>
    FAULT: typing.ClassVar[MotionStatus]  # value = <MotionStatus.FAULT: 2>
    INIT_FAILED: typing.ClassVar[MotionStatus]  # value = <MotionStatus.INIT_FAILED: 4>
    INVALID_INPUT: typing.ClassVar[
        MotionStatus
    ]  # value = <MotionStatus.INVALID_INPUT: 3>
    IN_PROGRESS: typing.ClassVar[MotionStatus]  # value = <MotionStatus.IN_PROGRESS: 5>
    PUBLISH_FAIL: typing.ClassVar[
        MotionStatus
    ]  # value = <MotionStatus.PUBLISH_FAIL: 8>
    STATUS_NUM: typing.ClassVar[MotionStatus]  # value = <MotionStatus.STATUS_NUM: 10>
    STOPPED_UNREACHED: typing.ClassVar[
        MotionStatus
    ]  # value = <MotionStatus.STOPPED_UNREACHED: 6>
    SUCCESS: typing.ClassVar[MotionStatus]  # value = <MotionStatus.SUCCESS: 0>
    TIMEOUT: typing.ClassVar[MotionStatus]  # value = <MotionStatus.TIMEOUT: 1>
    UNSUPPORTED_FUNCRION: typing.ClassVar[
        MotionStatus
    ]  # value = <MotionStatus.UNSUPPORTED_FUNCRION: 11>
    __members__: typing.ClassVar[
        dict[str, MotionStatus]
    ]  # value = {'SUCCESS': <MotionStatus.SUCCESS: 0>, 'TIMEOUT': <MotionStatus.TIMEOUT: 1>, 'FAULT': <MotionStatus.FAULT: 2>, 'INVALID_INPUT': <MotionStatus.INVALID_INPUT: 3>, 'INIT_FAILED': <MotionStatus.INIT_FAILED: 4>, 'IN_PROGRESS': <MotionStatus.IN_PROGRESS: 5>, 'STOPPED_UNREACHED': <MotionStatus.STOPPED_UNREACHED: 6>, 'DATA_FETCH_FAILED': <MotionStatus.DATA_FETCH_FAILED: 7>, 'PUBLISH_FAIL': <MotionStatus.PUBLISH_FAIL: 8>, 'COMM_DISCONNECTED': <MotionStatus.COMM_DISCONNECTED: 9>, 'STATUS_NUM': <MotionStatus.STATUS_NUM: 10>, 'UNSUPPORTED_FUNCRION': <MotionStatus.UNSUPPORTED_FUNCRION: 11>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class NavigationTaskSnapshot:
    msg: str
    status: NavigationTaskStatus
    task_id: str
    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...

class NavigationTaskStatus:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | UNKNOWN |  |
    | RUNNING |  |
    | SUCCESS |  |
    | FAILED |  |
    | INTERRUPTED |  |
    | OCCUPIED |  |
    | COLLISION |  |
    | CLOSE_TO_OBSTACLE |  |
    """

    CLOSE_TO_OBSTACLE: typing.ClassVar[
        NavigationTaskStatus
    ]  # value = <NavigationTaskStatus.CLOSE_TO_OBSTACLE: 7>
    COLLISION: typing.ClassVar[
        NavigationTaskStatus
    ]  # value = <NavigationTaskStatus.COLLISION: 6>
    FAILED: typing.ClassVar[
        NavigationTaskStatus
    ]  # value = <NavigationTaskStatus.FAILED: 3>
    INTERRUPTED: typing.ClassVar[
        NavigationTaskStatus
    ]  # value = <NavigationTaskStatus.INTERRUPTED: 4>
    OCCUPIED: typing.ClassVar[
        NavigationTaskStatus
    ]  # value = <NavigationTaskStatus.OCCUPIED: 5>
    RUNNING: typing.ClassVar[
        NavigationTaskStatus
    ]  # value = <NavigationTaskStatus.RUNNING: 1>
    SUCCESS: typing.ClassVar[
        NavigationTaskStatus
    ]  # value = <NavigationTaskStatus.SUCCESS: 2>
    UNKNOWN: typing.ClassVar[
        NavigationTaskStatus
    ]  # value = <NavigationTaskStatus.UNKNOWN: 0>
    __members__: typing.ClassVar[
        dict[str, NavigationTaskStatus]
    ]  # value = {'UNKNOWN': <NavigationTaskStatus.UNKNOWN: 0>, 'RUNNING': <NavigationTaskStatus.RUNNING: 1>, 'SUCCESS': <NavigationTaskStatus.SUCCESS: 2>, 'FAILED': <NavigationTaskStatus.FAILED: 3>, 'INTERRUPTED': <NavigationTaskStatus.INTERRUPTED: 4>, 'OCCUPIED': <NavigationTaskStatus.OCCUPIED: 5>, 'COLLISION': <NavigationTaskStatus.COLLISION: 6>, 'CLOSE_TO_OBSTACLE': <NavigationTaskStatus.CLOSE_TO_OBSTACLE: 7>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class OdomData:
    """
    Odometry data
    """
    def __init__(self) -> None: ...
    @property
    def angular_velocity(self) -> typing.Annotated[list[float], "FixedSize(3)"]:
        """
        Angular velocity [wx, wy, wz] (radians/second)
        """
    @angular_velocity.setter
    def angular_velocity(
        self,
        arg0: typing.Annotated[
            collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"
        ],
    ) -> None: ...
    @property
    def linear_velocity(self) -> typing.Annotated[list[float], "FixedSize(3)"]:
        """
        Linear velocity [vx, vy, vz] (meters/second)
        """
    @linear_velocity.setter
    def linear_velocity(
        self,
        arg0: typing.Annotated[
            collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"
        ],
    ) -> None: ...
    @property
    def orientation(self) -> typing.Annotated[list[float], "FixedSize(4)"]:
        """
        Orientation quaternion [x, y, z, w]
        """
    @orientation.setter
    def orientation(
        self,
        arg0: typing.Annotated[
            collections.abc.Sequence[typing.SupportsFloat], "FixedSize(4)"
        ],
    ) -> None: ...
    @property
    def position(self) -> typing.Annotated[list[float], "FixedSize(3)"]:
        """
        Position [x, y, z] (meters)
        """
    @position.setter
    def position(
        self,
        arg0: typing.Annotated[
            collections.abc.Sequence[typing.SupportsFloat], "FixedSize(3)"
        ],
    ) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...

class Parameter:
    actuate_type: ...
    is_blocking: bool
    is_check_collision: bool
    is_direct_execute: bool
    is_tool_pose: bool
    reference_frame: str

    def __init__(self) -> None: ...
    def __init__(
        self,
        direct_execute: bool,
        blocking: bool,
        timeout: typing.SupportsFloat,
        actuate: str,
        tool_pose: bool,
        check_collision: bool,
        frame: str = "base_link",
    ) -> None: ...
    def __repr__(self) -> str: ...
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
    def joint_state(self) -> dict[str, list[float]]: ...
    @joint_state.setter
    def joint_state(
        self,
        arg0: collections.abc.Mapping[
            str, collections.abc.Sequence[typing.SupportsFloat]
        ],
    ) -> None: ...
    @property
    def timeout_second(self) -> float: ...
    @timeout_second.setter
    def timeout_second(self, arg0: typing.SupportsFloat) -> None: ...

class PerceptionModule:
    """

    Perception module type

    Members:

    | Enum Value | Description |
    | --- | --- |
    | LIGHT_STEREO | Lightweight stereo depth |
    | FOUNDATION_STEREO | High-precision stereo depth |
    """

    FOUNDATION_STEREO: typing.ClassVar[
        PerceptionModule
    ]  # value = <PerceptionModule.FOUNDATION_STEREO: 0>
    LIGHT_STEREO: typing.ClassVar[
        PerceptionModule
    ]  # value = <PerceptionModule.LIGHT_STEREO: 1>
    __members__: typing.ClassVar[
        dict[str, PerceptionModule]
    ]  # value = {'LIGHT_STEREO': <PerceptionModule.LIGHT_STEREO: 1>, 'FOUNDATION_STEREO': <PerceptionModule.FOUNDATION_STEREO: 0>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class Point:
    def __init__(self) -> None: ...
    def __init__(
        self,
        x: typing.SupportsFloat = 0.0,
        y: typing.SupportsFloat = 0.0,
        z: typing.SupportsFloat = 0.0,
    ) -> None: ...
    @property
    def x(self) -> float: ...
    @x.setter
    def x(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def y(self) -> float: ...
    @y.setter
    def y(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def z(self) -> float: ...
    @z.setter
    def z(self, arg0: typing.SupportsFloat) -> None: ...

class Point2d:
    def __init__(self) -> None: ...
    def __init__(
        self, x: typing.SupportsFloat = 0.0, y: typing.SupportsFloat = 0.0
    ) -> None: ...
    @property
    def x(self) -> float: ...
    @x.setter
    def x(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def y(self) -> float: ...
    @y.setter
    def y(self, arg0: typing.SupportsFloat) -> None: ...

class PointField:
    """
    Point cloud field description information
    """
    def __init__(self) -> None: ...
    @property
    def count(self) -> int:
        """
        Number of field elements
        """
    @count.setter
    def count(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def datatype(self) -> ...:
        """
        Data type (DataType enum)
        """
    @datatype.setter
    def datatype(self, arg0: ...) -> None: ...
    @property
    def name(self) -> str:
        """
        Field name, e.g., x, y, z, intensity, rgb
        """
    @name.setter
    def name(self, arg0: str) -> None: ...
    @property
    def offset(self) -> int:
        """
        Byte offset of field in a single point
        """
    @offset.setter
    def offset(self, arg0: typing.SupportsInt) -> None: ...

class PointFieldDataType:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | UNKNOWN |  |
    | INT8 |  |
    | UINT8 |  |
    | INT16 |  |
    | UINT16 |  |
    | INT32 |  |
    | UINT32 |  |
    | FLOAT32 |  |
    | FLOAT64 |  |
    """

    FLOAT32: typing.ClassVar[
        PointFieldDataType
    ]  # value = <PointFieldDataType.FLOAT32: 7>
    FLOAT64: typing.ClassVar[
        PointFieldDataType
    ]  # value = <PointFieldDataType.FLOAT64: 8>
    INT16: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.INT16: 3>
    INT32: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.INT32: 5>
    INT8: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.INT8: 1>
    UINT16: typing.ClassVar[
        PointFieldDataType
    ]  # value = <PointFieldDataType.UINT16: 4>
    UINT32: typing.ClassVar[
        PointFieldDataType
    ]  # value = <PointFieldDataType.UINT32: 6>
    UINT8: typing.ClassVar[PointFieldDataType]  # value = <PointFieldDataType.UINT8: 2>
    UNKNOWN: typing.ClassVar[
        PointFieldDataType
    ]  # value = <PointFieldDataType.UNKNOWN: 0>
    __members__: typing.ClassVar[
        dict[str, PointFieldDataType]
    ]  # value = {'UNKNOWN': <PointFieldDataType.UNKNOWN: 0>, 'INT8': <PointFieldDataType.INT8: 1>, 'UINT8': <PointFieldDataType.UINT8: 2>, 'INT16': <PointFieldDataType.INT16: 3>, 'UINT16': <PointFieldDataType.UINT16: 4>, 'INT32': <PointFieldDataType.INT32: 5>, 'UINT32': <PointFieldDataType.UINT32: 6>, 'FLOAT32': <PointFieldDataType.FLOAT32: 7>, 'FLOAT64': <PointFieldDataType.FLOAT64: 8>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class Pose:
    orientation: Quaternion
    position: Point

    def __init__(self) -> None: ...
    def __init__(
        self,
        pos: collections.abc.Sequence[typing.SupportsFloat],
        quat: collections.abc.Sequence[typing.SupportsFloat],
    ) -> None: ...
    def __init__(self, vec: collections.abc.Sequence[typing.SupportsFloat]) -> None: ...

class Pose2d:
    position: Point2d

    def __init__(self) -> None: ...
    def __init__(self, vec: collections.abc.Sequence[typing.SupportsFloat]) -> None: ...
    def __init__(
        self,
        x: typing.SupportsFloat = 0.0,
        y: typing.SupportsFloat = 0.0,
        theta: typing.SupportsFloat = 0.0,
    ) -> None: ...
    @property
    def theta(self) -> float: ...
    @theta.setter
    def theta(self, arg0: typing.SupportsFloat) -> None: ...

class PoseState(RobotStates):
    frame_id: str
    pose: Pose
    reference_frame: str
    def __init__(self) -> None: ...
    def get_type(self) -> RobotStatesType: ...

class PrimitiveType:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | LINE |  |
    | CYLINDER |  |
    """

    CYLINDER: typing.ClassVar[PrimitiveType]  # value = <PrimitiveType.CYLINDER: 1>
    LINE: typing.ClassVar[PrimitiveType]  # value = <PrimitiveType.LINE: 0>
    __members__: typing.ClassVar[
        dict[str, PrimitiveType]
    ]  # value = {'LINE': <PrimitiveType.LINE: 0>, 'CYLINDER': <PrimitiveType.CYLINDER: 1>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class Quaternion:
    def __init__(self) -> None: ...
    def __init__(
        self,
        x: typing.SupportsFloat = 0.0,
        y: typing.SupportsFloat = 0.0,
        z: typing.SupportsFloat = 0.0,
        w: typing.SupportsFloat = 1.0,
    ) -> None: ...
    @property
    def w(self) -> float: ...
    @w.setter
    def w(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def x(self) -> float: ...
    @x.setter
    def x(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def y(self) -> float: ...
    @y.setter
    def y(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def z(self) -> float: ...
    @z.setter
    def z(self, arg0: typing.SupportsFloat) -> None: ...

class RgbData:
    """
    Compressed image data
    """
    def __init__(self) -> None: ...
    @property
    def data(self) -> bytes:
        """
        Compressed binary data
        """
    @property
    def format(self) -> str:
        """
        Image format
        """
    @format.setter
    def format(self, arg0: str) -> None: ...
    @property
    def header(self) -> Header:
        """
        Message header
        """
    @header.setter
    def header(self, arg0: Header) -> None: ...

class RobotStates:
    chain_name: str

    def __init__(self) -> None: ...
    def __init__(
        self,
        chain: str,
        whole_joint: collections.abc.Sequence[typing.SupportsFloat],
        base_pose: Pose,
    ) -> None: ...
    def get_type(self) -> RobotStatesType: ...
    def set_base_state(self, base_pose: Pose) -> None: ...
    def set_whole_body_joint(
        self, joint_positions: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    @property
    def base_state(self) -> list[float]: ...
    @base_state.setter
    def base_state(
        self, arg0: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...
    @property
    def whole_body_joint(self) -> list[float]: ...
    @whole_body_joint.setter
    def whole_body_joint(
        self, arg0: collections.abc.Sequence[typing.SupportsFloat]
    ) -> None: ...

class RobotStatesType:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | POSE |  |
    | JOINT |  |
    | ROBOT_STATES |  |
    """

    JOINT: typing.ClassVar[RobotStatesType]  # value = <RobotStatesType.JOINT: 1>
    POSE: typing.ClassVar[RobotStatesType]  # value = <RobotStatesType.POSE: 0>
    ROBOT_STATES: typing.ClassVar[
        RobotStatesType
    ]  # value = <RobotStatesType.ROBOT_STATES: 2>
    __members__: typing.ClassVar[
        dict[str, RobotStatesType]
    ]  # value = {'POSE': <RobotStatesType.POSE: 0>, 'JOINT': <RobotStatesType.JOINT: 1>, 'ROBOT_STATES': <RobotStatesType.ROBOT_STATES: 2>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class SUCTION_ACTION_STATE:
    """

    Suction cup action state enumeration

    Members:

    | Enum Value | Description |
    | --- | --- |
    | IDLE | Not sucking |
    | SUCKING | Currently sucking |
    | SUCCESS | Suction successful |
    | FAILED | Suction failed |
    """

    FAILED: typing.ClassVar[
        SUCTION_ACTION_STATE
    ]  # value = <SUCTION_ACTION_STATE.FAILED: 3>
    IDLE: typing.ClassVar[
        SUCTION_ACTION_STATE
    ]  # value = <SUCTION_ACTION_STATE.IDLE: 0>
    SUCCESS: typing.ClassVar[
        SUCTION_ACTION_STATE
    ]  # value = <SUCTION_ACTION_STATE.SUCCESS: 2>
    SUCKING: typing.ClassVar[
        SUCTION_ACTION_STATE
    ]  # value = <SUCTION_ACTION_STATE.SUCKING: 1>
    __members__: typing.ClassVar[
        dict[str, SUCTION_ACTION_STATE]
    ]  # value = {'IDLE': <SUCTION_ACTION_STATE.IDLE: 0>, 'SUCKING': <SUCTION_ACTION_STATE.SUCKING: 1>, 'SUCCESS': <SUCTION_ACTION_STATE.SUCCESS: 2>, 'FAILED': <SUCTION_ACTION_STATE.FAILED: 3>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class SamplerConfig:
    def __init__(self) -> None: ...
    def get_interpolate(self) -> bool: ...
    def get_interpolation_cnt(self) -> int: ...
    def get_max_planning_time(self) -> float: ...
    def get_max_simplification_time(self) -> float: ...
    def get_simplify(self) -> bool: ...
    def get_state_check_resolution(self) -> float: ...
    def get_state_check_type(self) -> StateCheckType: ...
    def get_termination_condition_type(self) -> TerminationConditionType: ...
    def print(self) -> None: ...
    def set_interpolate(self, enable: bool) -> None: ...
    def set_interpolation_cnt(self, cnt: typing.SupportsInt) -> None: ...
    def set_max_planning_time(self, time: typing.SupportsFloat) -> None: ...
    def set_max_simplification_time(self, time: typing.SupportsFloat) -> None: ...
    def set_simplify(self, enable: bool) -> None: ...
    def set_state_check_resolution(self, resolution: typing.SupportsFloat) -> None: ...
    def set_state_check_type(self, type: StateCheckType) -> None: ...
    def set_termination_condition_type(
        self, type: TerminationConditionType
    ) -> None: ...

class SeedType:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | RANDOM_SEED |  |
    | RANDOM_PROGRESSIVE_SEED |  |
    | USER_DEFINED_SEED |  |
    """

    RANDOM_PROGRESSIVE_SEED: typing.ClassVar[
        SeedType
    ]  # value = <SeedType.RANDOM_PROGRESSIVE_SEED: 1>
    RANDOM_SEED: typing.ClassVar[SeedType]  # value = <SeedType.RANDOM_SEED: 0>
    USER_DEFINED_SEED: typing.ClassVar[
        SeedType
    ]  # value = <SeedType.USER_DEFINED_SEED: 2>
    __members__: typing.ClassVar[
        dict[str, SeedType]
    ]  # value = {'RANDOM_SEED': <SeedType.RANDOM_SEED: 0>, 'RANDOM_PROGRESSIVE_SEED': <SeedType.RANDOM_PROGRESSIVE_SEED: 1>, 'USER_DEFINED_SEED': <SeedType.USER_DEFINED_SEED: 2>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class SensorType:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | HEAD_LEFT_CAMERA | Head left camera |
    | HEAD_RIGHT_CAMERA | Head right camera |
    | LEFT_ARM_CAMERA | Left arm camera |
    | RIGHT_ARM_CAMERA | Right arm camera |
    | LEFT_ARM_DEPTH_CAMERA | Left arm depth camera |
    | RIGHT_ARM_DEPTH_CAMERA | Right arm depth camera |
    | LEFT_ARM_INFRA_CAMERA_1 | Left arm infrared camera 1 |
    | LEFT_ARM_INFRA_CAMERA_2 | Left arm infrared camera 2 |
    | RIGHT_ARM_INFRA_CAMERA_1 | Right arm infrared camera 1 |
    | RIGHT_ARM_INFRA_CAMERA_2 | Right arm infrared camera 2 |
    | BASE_ULTRASONIC | Base ultrasonic sensor |
    | CHASSIS_IMU | Chassis LiDAR IMU |
    | BASE_LIDAR | Base LiDAR |
    | TORSO_IMU | Torso IMU |
    | LIDAR_IMU | LiDAR IMU |
    | LEFT_FRONT_SURROUND_CAMERA | Left front surround color camera |
    | RIGHT_FRONT_SURROUND_CAMERA | Right front surround color camera |
    | LEFT_REAR_SURROUND_CAMERA | Left rear surround color camera |
    | RIGHT_REAR_SURROUND_CAMERA | Right rear surround color camera |
    """

    BASE_LIDAR: typing.ClassVar[SensorType]  # value = <SensorType.BASE_LIDAR: 10>
    BASE_ULTRASONIC: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.BASE_ULTRASONIC: 19>
    CHASSIS_IMU: typing.ClassVar[SensorType]  # value = <SensorType.CHASSIS_IMU: 16>
    HEAD_LEFT_CAMERA: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.HEAD_LEFT_CAMERA: 0>
    HEAD_RIGHT_CAMERA: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.HEAD_RIGHT_CAMERA: 1>
    LEFT_ARM_CAMERA: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.LEFT_ARM_CAMERA: 2>
    LEFT_ARM_DEPTH_CAMERA: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.LEFT_ARM_DEPTH_CAMERA: 4>
    LEFT_ARM_INFRA_CAMERA_1: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.LEFT_ARM_INFRA_CAMERA_1: 6>
    LEFT_ARM_INFRA_CAMERA_2: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.LEFT_ARM_INFRA_CAMERA_2: 7>
    LEFT_FRONT_SURROUND_CAMERA: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.LEFT_FRONT_SURROUND_CAMERA: 20>
    LEFT_REAR_SURROUND_CAMERA: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.LEFT_REAR_SURROUND_CAMERA: 22>
    LIDAR_IMU: typing.ClassVar[SensorType]  # value = <SensorType.LIDAR_IMU: 18>
    RIGHT_ARM_CAMERA: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.RIGHT_ARM_CAMERA: 3>
    RIGHT_ARM_DEPTH_CAMERA: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.RIGHT_ARM_DEPTH_CAMERA: 5>
    RIGHT_ARM_INFRA_CAMERA_1: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.RIGHT_ARM_INFRA_CAMERA_1: 8>
    RIGHT_ARM_INFRA_CAMERA_2: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.RIGHT_ARM_INFRA_CAMERA_2: 9>
    RIGHT_FRONT_SURROUND_CAMERA: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.RIGHT_FRONT_SURROUND_CAMERA: 21>
    RIGHT_REAR_SURROUND_CAMERA: typing.ClassVar[
        SensorType
    ]  # value = <SensorType.RIGHT_REAR_SURROUND_CAMERA: 23>
    TORSO_IMU: typing.ClassVar[SensorType]  # value = <SensorType.TORSO_IMU: 17>
    __members__: typing.ClassVar[
        dict[str, SensorType]
    ]  # value = {'HEAD_LEFT_CAMERA': <SensorType.HEAD_LEFT_CAMERA: 0>, 'HEAD_RIGHT_CAMERA': <SensorType.HEAD_RIGHT_CAMERA: 1>, 'LEFT_ARM_CAMERA': <SensorType.LEFT_ARM_CAMERA: 2>, 'RIGHT_ARM_CAMERA': <SensorType.RIGHT_ARM_CAMERA: 3>, 'LEFT_ARM_DEPTH_CAMERA': <SensorType.LEFT_ARM_DEPTH_CAMERA: 4>, 'RIGHT_ARM_DEPTH_CAMERA': <SensorType.RIGHT_ARM_DEPTH_CAMERA: 5>, 'LEFT_ARM_INFRA_CAMERA_1': <SensorType.LEFT_ARM_INFRA_CAMERA_1: 6>, 'LEFT_ARM_INFRA_CAMERA_2': <SensorType.LEFT_ARM_INFRA_CAMERA_2: 7>, 'RIGHT_ARM_INFRA_CAMERA_1': <SensorType.RIGHT_ARM_INFRA_CAMERA_1: 8>, 'RIGHT_ARM_INFRA_CAMERA_2': <SensorType.RIGHT_ARM_INFRA_CAMERA_2: 9>, 'BASE_ULTRASONIC': <SensorType.BASE_ULTRASONIC: 19>, 'CHASSIS_IMU': <SensorType.CHASSIS_IMU: 16>, 'BASE_LIDAR': <SensorType.BASE_LIDAR: 10>, 'TORSO_IMU': <SensorType.TORSO_IMU: 17>, 'LIDAR_IMU': <SensorType.LIDAR_IMU: 18>, 'LEFT_FRONT_SURROUND_CAMERA': <SensorType.LEFT_FRONT_SURROUND_CAMERA: 20>, 'RIGHT_FRONT_SURROUND_CAMERA': <SensorType.RIGHT_FRONT_SURROUND_CAMERA: 21>, 'LEFT_REAR_SURROUND_CAMERA': <SensorType.LEFT_REAR_SURROUND_CAMERA: 22>, 'RIGHT_REAR_SURROUND_CAMERA': <SensorType.RIGHT_REAR_SURROUND_CAMERA: 23>, 'HEAD_LIDAR': <SensorType.HEAD_LIDAR: 11>, 'BACK_LIDAR': <SensorType.BACK_LIDAR: 12>, 'CHASSIS_LIDAR': <SensorType.CHASSIS_LIDAR: 13>, 'HEAD_IMU': <SensorType.HEAD_IMU: 14>, 'BACK_IMU': <SensorType.BACK_IMU: 15>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class SingoriXTarget:
    """
    SDK mirror of a SingoriX target
    """
    def __init__(self) -> None: ...
    @property
    def header(self) -> Header:
        """
        Message header
        """
    @header.setter
    def header(self, arg0: Header) -> None: ...
    @property
    def target_group_trajectory_map(self) -> dict[str, TargetGroupTrajectory]:
        """
        Joint-space trajectory map
        """
    @target_group_trajectory_map.setter
    def target_group_trajectory_map(
        self, arg0: collections.abc.Mapping[str, TargetGroupTrajectory]
    ) -> None: ...
    @property
    def target_task_trajectory_map(self) -> dict[str, TargetTaskTrajectory]:
        """
        Task-space trajectory map
        """
    @target_task_trajectory_map.setter
    def target_task_trajectory_map(
        self, arg0: collections.abc.Mapping[str, TargetTaskTrajectory]
    ) -> None: ...

class StateCheckType:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | EUCLIDEAN_DISTANCE |  |
    | RADIAN_DISTANCE |  |
    """

    EUCLIDEAN_DISTANCE: typing.ClassVar[
        StateCheckType
    ]  # value = <StateCheckType.EUCLIDEAN_DISTANCE: 0>
    RADIAN_DISTANCE: typing.ClassVar[
        StateCheckType
    ]  # value = <StateCheckType.RADIAN_DISTANCE: 1>
    __members__: typing.ClassVar[
        dict[str, StateCheckType]
    ]  # value = {'EUCLIDEAN_DISTANCE': <StateCheckType.EUCLIDEAN_DISTANCE: 0>, 'RADIAN_DISTANCE': <StateCheckType.RADIAN_DISTANCE: 1>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class SuctionCupState:
    """
    Suction cup state information
    """
    def __init__(self) -> None: ...
    @property
    def action_state(self) -> SUCTION_ACTION_STATE:
        """
        Current suction cup action state (SUCTION_ACTION_STATE enum)
        """
    @action_state.setter
    def action_state(self, arg0: SUCTION_ACTION_STATE) -> None: ...
    @property
    def activation(self) -> bool:
        """
        Whether currently sucking
        """
    @activation.setter
    def activation(self, arg0: bool) -> None: ...
    @property
    def pressure(self) -> float:
        """
        Current pressure (Pa)
        """
    @pressure.setter
    def pressure(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...

class SyncedObservation:
    """
    Synchronized multi-sensor observation payload
    """
    def __init__(self) -> None: ...
    @property
    def depth_data_map(self) -> dict[SensorType, DepthData]:
        """
        Timestamp-aligned depth frames
        """
    @depth_data_map.setter
    def depth_data_map(
        self, arg0: collections.abc.Mapping[SensorType, DepthData]
    ) -> None: ...
    @property
    def joint_state(self) -> JointStateMessage:
        """
        Nearest-neighbor joint sample for anchor timestamp (JointStateMessage | None)
        """
    @joint_state.setter
    def joint_state(self, arg0: JointStateMessage) -> None: ...
    @property
    def rgb_data_map(self) -> dict[SensorType, RgbData]:
        """
        Timestamp-aligned RGB frames
        """
    @rgb_data_map.setter
    def rgb_data_map(
        self, arg0: collections.abc.Mapping[SensorType, RgbData]
    ) -> None: ...

class TargetConfig:
    """
    Common target configuration
    """
    def __init__(self) -> None: ...
    @property
    def target_data(self) -> int:
        """
        Target data bitmask
        """
    @target_data.setter
    def target_data(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def target_id(self) -> str:
        """
        Target identifier
        """
    @target_id.setter
    def target_id(self, arg0: str) -> None: ...
    @property
    def target_priority(self) -> int:
        """
        Target priority
        """
    @target_priority.setter
    def target_priority(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def target_sampling(self) -> TargetSampling:
        """
        Sampling strategy
        """
    @target_sampling.setter
    def target_sampling(self, arg0: TargetSampling) -> None: ...
    @property
    def target_ts(self) -> Timestamp:
        """
        Target timestamp
        """
    @target_ts.setter
    def target_ts(self, arg0: Timestamp) -> None: ...
    @property
    def target_type(self) -> int:
        """
        Target type bitmask
        """
    @target_type.setter
    def target_type(self, arg0: typing.SupportsInt) -> None: ...

class TargetGroupTrajectory:
    """
    Target trajectory for a joint group
    """
    def __init__(self) -> None: ...
    @property
    def group_commands(self) -> list[GroupCommand]:
        """
        Trajectory points
        """
    @group_commands.setter
    def group_commands(self, arg0: collections.abc.Sequence[GroupCommand]) -> None: ...
    @property
    def joint_names(self) -> list[str]:
        """
        Joint names
        """
    @joint_names.setter
    def joint_names(self, arg0: collections.abc.Sequence[str]) -> None: ...
    @property
    def target_config(self) -> TargetConfig:
        """
        Target configuration
        """
    @target_config.setter
    def target_config(self, arg0: TargetConfig) -> None: ...

class TargetSampling:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | TARGET_SAMPLING_DEFAULT | Default sampling strategy |
    | TARGET_SAMPLING_DIRECT_PASS | Direct pass-through |
    | TARGET_SAMPLING_LINEAR_INTERPOLATE | Linear interpolation |
    | TARGET_SAMPLING_TRAPEZOIDAL_PROFILE | Trapezoidal profile |
    | TARGET_SAMPLING_S_CURVE_PROFILE | S-curve profile |
    | TARGET_SAMPLING_CUBIC_SPLINES | Cubic splines |
    | TARGET_SAMPLING_QUINTIC_SPLINES | Quintic splines |
    | TARGET_SAMPLING_B_SPLINES | B-splines |
    | TARGET_SAMPLING_CUSTOM | Custom sampling |
    """

    TARGET_SAMPLING_B_SPLINES: typing.ClassVar[
        TargetSampling
    ]  # value = <TargetSampling.TARGET_SAMPLING_B_SPLINES: 7>
    TARGET_SAMPLING_CUBIC_SPLINES: typing.ClassVar[
        TargetSampling
    ]  # value = <TargetSampling.TARGET_SAMPLING_CUBIC_SPLINES: 5>
    TARGET_SAMPLING_CUSTOM: typing.ClassVar[
        TargetSampling
    ]  # value = <TargetSampling.TARGET_SAMPLING_CUSTOM: 15>
    TARGET_SAMPLING_DEFAULT: typing.ClassVar[
        TargetSampling
    ]  # value = <TargetSampling.TARGET_SAMPLING_DEFAULT: 0>
    TARGET_SAMPLING_DIRECT_PASS: typing.ClassVar[
        TargetSampling
    ]  # value = <TargetSampling.TARGET_SAMPLING_DIRECT_PASS: 1>
    TARGET_SAMPLING_LINEAR_INTERPOLATE: typing.ClassVar[
        TargetSampling
    ]  # value = <TargetSampling.TARGET_SAMPLING_LINEAR_INTERPOLATE: 2>
    TARGET_SAMPLING_QUINTIC_SPLINES: typing.ClassVar[
        TargetSampling
    ]  # value = <TargetSampling.TARGET_SAMPLING_QUINTIC_SPLINES: 6>
    TARGET_SAMPLING_S_CURVE_PROFILE: typing.ClassVar[
        TargetSampling
    ]  # value = <TargetSampling.TARGET_SAMPLING_S_CURVE_PROFILE: 4>
    TARGET_SAMPLING_TRAPEZOIDAL_PROFILE: typing.ClassVar[
        TargetSampling
    ]  # value = <TargetSampling.TARGET_SAMPLING_TRAPEZOIDAL_PROFILE: 3>
    __members__: typing.ClassVar[
        dict[str, TargetSampling]
    ]  # value = {'TARGET_SAMPLING_DEFAULT': <TargetSampling.TARGET_SAMPLING_DEFAULT: 0>, 'TARGET_SAMPLING_DIRECT_PASS': <TargetSampling.TARGET_SAMPLING_DIRECT_PASS: 1>, 'TARGET_SAMPLING_LINEAR_INTERPOLATE': <TargetSampling.TARGET_SAMPLING_LINEAR_INTERPOLATE: 2>, 'TARGET_SAMPLING_TRAPEZOIDAL_PROFILE': <TargetSampling.TARGET_SAMPLING_TRAPEZOIDAL_PROFILE: 3>, 'TARGET_SAMPLING_S_CURVE_PROFILE': <TargetSampling.TARGET_SAMPLING_S_CURVE_PROFILE: 4>, 'TARGET_SAMPLING_CUBIC_SPLINES': <TargetSampling.TARGET_SAMPLING_CUBIC_SPLINES: 5>, 'TARGET_SAMPLING_QUINTIC_SPLINES': <TargetSampling.TARGET_SAMPLING_QUINTIC_SPLINES: 6>, 'TARGET_SAMPLING_B_SPLINES': <TargetSampling.TARGET_SAMPLING_B_SPLINES: 7>, 'TARGET_SAMPLING_CUSTOM': <TargetSampling.TARGET_SAMPLING_CUSTOM: 15>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class TargetTaskTrajectory:
    """
    Target trajectory for task-space control
    """
    def __init__(self) -> None: ...
    @property
    def group_names(self) -> list[str]:
        """
        Related group names
        """
    @group_names.setter
    def group_names(self, arg0: collections.abc.Sequence[str]) -> None: ...
    @property
    def joint_names(self) -> list[str]:
        """
        Related joint names
        """
    @joint_names.setter
    def joint_names(self, arg0: collections.abc.Sequence[str]) -> None: ...
    @property
    def subtask_names(self) -> list[str]:
        """
        Subtask names
        """
    @subtask_names.setter
    def subtask_names(self, arg0: collections.abc.Sequence[str]) -> None: ...
    @property
    def target_config(self) -> TargetConfig:
        """
        Target configuration
        """
    @target_config.setter
    def target_config(self, arg0: TargetConfig) -> None: ...
    @property
    def task_commands(self) -> list[TaskCommand]:
        """
        Trajectory points
        """
    @task_commands.setter
    def task_commands(self, arg0: collections.abc.Sequence[TaskCommand]) -> None: ...

class TaskCommand:
    """
    Task-space trajectory point
    """
    def __init__(self) -> None: ...
    @property
    def subtask_commands(self) -> list[FrameTriad]:
        """
        Subtask commands at this point
        """
    @subtask_commands.setter
    def subtask_commands(self, arg0: collections.abc.Sequence[FrameTriad]) -> None: ...
    @property
    def time_from_start_s(self) -> float:
        """
        Time from trajectory start in seconds
        """
    @time_from_start_s.setter
    def time_from_start_s(self, arg0: typing.SupportsFloat) -> None: ...

class TaskHandle:
    msg: str
    request_sent: bool
    task_id: str
    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...

class TerminationConditionType:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | TIMEOUT |  |
    | TIMEOUT_AND_EXACT_SOLUTION |  |
    """

    TIMEOUT: typing.ClassVar[
        TerminationConditionType
    ]  # value = <TerminationConditionType.TIMEOUT: 0>
    TIMEOUT_AND_EXACT_SOLUTION: typing.ClassVar[
        TerminationConditionType
    ]  # value = <TerminationConditionType.TIMEOUT_AND_EXACT_SOLUTION: 1>
    __members__: typing.ClassVar[
        dict[str, TerminationConditionType]
    ]  # value = {'TIMEOUT': <TerminationConditionType.TIMEOUT: 0>, 'TIMEOUT_AND_EXACT_SOLUTION': <TerminationConditionType.TIMEOUT_AND_EXACT_SOLUTION: 1>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class Timestamp:
    """
    High-precision timestamp
    """
    def __init__(self) -> None: ...
    @property
    def nanosec(self) -> int:
        """
        Nanoseconds
        """
    @nanosec.setter
    def nanosec(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def sec(self) -> int:
        """
        Seconds
        """
    @sec.setter
    def sec(self, arg0: typing.SupportsInt) -> None: ...

class Trajectory:
    """
    Trajectory object. Note: joint_groups and joint_names must not both be empty. joint_names takes precedence.
    """
    def __init__(self) -> None: ...
    @property
    def joint_groups(self) -> list[str]:
        """
        List of joint group names. Recommended: use semantic groups like ["left_arm", "right_arm"].
        """
    @joint_groups.setter
    def joint_groups(self, arg0: collections.abc.Sequence[str]) -> None: ...
    @property
    def joint_names(self) -> list[str]:
        """
        List of joint names. Takes precedence over joint_groups if both are set.
        """
    @joint_names.setter
    def joint_names(self, arg0: collections.abc.Sequence[str]) -> None: ...
    @property
    def points(self) -> list[TrajectoryPoint]:
        """
        List of trajectory points (TrajectoryPoint list)
        """
    @points.setter
    def points(self, arg0: collections.abc.Sequence[TrajectoryPoint]) -> None: ...

class TrajectoryControlStatus:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | INVALID_INPUT | Input parameters do not meet requirements |
    | RUNNING | Currently running |
    | COMPLETED | Reached target position |
    | STOPPED_UNREACHED | Stopped but not reached target |
    | ERROR | Error occurred, cannot continue execution |
    | DATA_FETCH_FAILED | Failed to fetch execution data |
    """

    COMPLETED: typing.ClassVar[
        TrajectoryControlStatus
    ]  # value = <TrajectoryControlStatus.COMPLETED: 2>
    DATA_FETCH_FAILED: typing.ClassVar[
        TrajectoryControlStatus
    ]  # value = <TrajectoryControlStatus.DATA_FETCH_FAILED: 5>
    ERROR: typing.ClassVar[
        TrajectoryControlStatus
    ]  # value = <TrajectoryControlStatus.ERROR: 4>
    INVALID_INPUT: typing.ClassVar[
        TrajectoryControlStatus
    ]  # value = <TrajectoryControlStatus.INVALID_INPUT: 0>
    RUNNING: typing.ClassVar[
        TrajectoryControlStatus
    ]  # value = <TrajectoryControlStatus.RUNNING: 1>
    STOPPED_UNREACHED: typing.ClassVar[
        TrajectoryControlStatus
    ]  # value = <TrajectoryControlStatus.STOPPED_UNREACHED: 3>
    __members__: typing.ClassVar[
        dict[str, TrajectoryControlStatus]
    ]  # value = {'INVALID_INPUT': <TrajectoryControlStatus.INVALID_INPUT: 0>, 'RUNNING': <TrajectoryControlStatus.RUNNING: 1>, 'COMPLETED': <TrajectoryControlStatus.COMPLETED: 2>, 'STOPPED_UNREACHED': <TrajectoryControlStatus.STOPPED_UNREACHED: 3>, 'ERROR': <TrajectoryControlStatus.ERROR: 4>, 'DATA_FETCH_FAILED': <TrajectoryControlStatus.DATA_FETCH_FAILED: 5>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class TrajectoryFeasibilityCheckOption:
    def __init__(self) -> None: ...
    def get_disable_collision_check(self) -> bool: ...
    def get_disable_joint_limit_check(self) -> bool: ...
    def get_disable_velocity_feasibility_check(self) -> bool: ...
    def print(self) -> None: ...
    def set_disable_collision_check(self, disable: bool) -> None: ...
    def set_disable_joint_limit_check(self, disable: bool) -> None: ...
    def set_disable_velocity_feasibility_check(self, disable: bool) -> None: ...

class TrajectoryPlanConfig:
    def __init__(self) -> None: ...
    def get_min_move_time(self) -> float: ...
    def get_move_line_intermediate_point(self) -> float: ...
    def get_way_point_plan_expected_time(self) -> float: ...
    def print(self) -> None: ...
    def set_min_move_time(self, time: typing.SupportsFloat) -> None: ...
    def set_move_line_intermediate_point(self, value: typing.SupportsFloat) -> None: ...
    def set_way_point_plan_expected_time(self, time: typing.SupportsFloat) -> None: ...

class TrajectoryPoint:
    """
    Single trajectory point object
    """
    def __init__(self) -> None: ...
    @property
    def joint_command_vec(self) -> list[JointCommand]:
        """
        - `joint_command_vec` (`List[JointCommand]`): List of specific joint commands to execute
        """
    @joint_command_vec.setter
    def joint_command_vec(
        self, arg0: collections.abc.Sequence[JointCommand]
    ) -> None: ...
    @property
    def time_from_start_second(self) -> float:
        """
        - `time_from_start_second` (`float`): Time from trajectory start (seconds)
        """
    @time_from_start_second.setter
    def time_from_start_second(self, arg0: typing.SupportsFloat) -> None: ...

class Twist:
    """
    Six-dimensional twist command
    """
    def __init__(self) -> None: ...
    @property
    def angular(self) -> Vector3:
        """
        Angular velocity vector
        """
    @angular.setter
    def angular(self, arg0: Vector3) -> None: ...
    @property
    def linear(self) -> Vector3:
        """
        Linear velocity vector
        """
    @linear.setter
    def linear(self, arg0: Vector3) -> None: ...

class UltrasonicData:
    """
    Ultrasonic sensor data
    """
    def __init__(self) -> None: ...
    @property
    def distance(self) -> float:
        """
        Distance (meters)
        """
    @distance.setter
    def distance(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def timestamp_ns(self) -> int:
        """
        Timestamp (nanoseconds)
        """
    @timestamp_ns.setter
    def timestamp_ns(self, arg0: typing.SupportsInt) -> None: ...

class UltrasonicType:
    """

    Members:

    | Enum Value | Description |
    | --- | --- |
    | FRONT_LEFT | Front left |
    | FRONT_RIGHT | Front right |
    | RIGHT_LEFT | Right left |
    | RIGHT_RIGHT | Right right |
    | BACK_LEFT | Back left |
    | BACK_RIGHT | Back right |
    | LEFT_LEFT | Left left |
    | LEFT_RIGHT | Left right |
    """

    BACK_LEFT: typing.ClassVar[UltrasonicType]  # value = <UltrasonicType.BACK_LEFT: 4>
    BACK_RIGHT: typing.ClassVar[
        UltrasonicType
    ]  # value = <UltrasonicType.BACK_RIGHT: 5>
    FRONT_LEFT: typing.ClassVar[
        UltrasonicType
    ]  # value = <UltrasonicType.FRONT_LEFT: 0>
    FRONT_RIGHT: typing.ClassVar[
        UltrasonicType
    ]  # value = <UltrasonicType.FRONT_RIGHT: 1>
    LEFT_LEFT: typing.ClassVar[UltrasonicType]  # value = <UltrasonicType.LEFT_LEFT: 6>
    LEFT_RIGHT: typing.ClassVar[
        UltrasonicType
    ]  # value = <UltrasonicType.LEFT_RIGHT: 7>
    RIGHT_LEFT: typing.ClassVar[
        UltrasonicType
    ]  # value = <UltrasonicType.RIGHT_LEFT: 2>
    RIGHT_RIGHT: typing.ClassVar[
        UltrasonicType
    ]  # value = <UltrasonicType.RIGHT_RIGHT: 3>
    __members__: typing.ClassVar[
        dict[str, UltrasonicType]
    ]  # value = {'FRONT_LEFT': <UltrasonicType.FRONT_LEFT: 0>, 'FRONT_RIGHT': <UltrasonicType.FRONT_RIGHT: 1>, 'RIGHT_LEFT': <UltrasonicType.RIGHT_LEFT: 2>, 'RIGHT_RIGHT': <UltrasonicType.RIGHT_RIGHT: 3>, 'BACK_LEFT': <UltrasonicType.BACK_LEFT: 4>, 'BACK_RIGHT': <UltrasonicType.BACK_RIGHT: 5>, 'LEFT_LEFT': <UltrasonicType.LEFT_LEFT: 6>, 'LEFT_RIGHT': <UltrasonicType.LEFT_RIGHT: 7>}
    def __eq__(self, other: typing.Any) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: typing.SupportsInt) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: typing.Any) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: typing.SupportsInt) -> None: ...
    def __str__(self) -> str: ...
    @property
    def name(self) -> str: ...
    @property
    def value(self) -> int: ...

class Vector3:
    """
    Three-dimensional vector
    """
    def __init__(self) -> None: ...
    @property
    def x(self) -> float:
        """
        X coordinate
        """
    @x.setter
    def x(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def y(self) -> float:
        """
        Y coordinate
        """
    @y.setter
    def y(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def z(self) -> float:
        """
        Z coordinate
        """
    @z.setter
    def z(self, arg0: typing.SupportsFloat) -> None: ...

class WBCException(Exception):
    pass

class Waypoint:
    params: WaypointParams
    pose: Pose
    def __init__(self, pose: Pose, params: WaypointParams = ...) -> None: ...

class WaypointParams:
    def __init__(self) -> None: ...
    @property
    def acceleration_scale(self) -> float: ...
    @acceleration_scale.setter
    def acceleration_scale(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def arrival_orientation_threshold(self) -> float: ...
    @arrival_orientation_threshold.setter
    def arrival_orientation_threshold(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def arrival_position_threshold_x(self) -> float: ...
    @arrival_position_threshold_x.setter
    def arrival_position_threshold_x(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def arrival_position_threshold_y(self) -> float: ...
    @arrival_position_threshold_y.setter
    def arrival_position_threshold_y(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def jerk_scale(self) -> float: ...
    @jerk_scale.setter
    def jerk_scale(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def velocity_scale(self) -> float: ...
    @velocity_scale.setter
    def velocity_scale(self, arg0: typing.SupportsFloat) -> None: ...

class Wrench:
    """
    Six-dimensional wrench command
    """
    def __init__(self) -> None: ...
    @property
    def force(self) -> Vector3:
        """
        Force vector
        """
    @force.setter
    def force(self, arg0: Vector3) -> None: ...
    @property
    def torque(self) -> Vector3:
        """
        Torque vector
        """
    @torque.setter
    def torque(self, arg0: Vector3) -> None: ...

def check_motion_status(status: MotionStatus) -> str:
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

def create_parameter(
    direct_execute: bool,
    blocking: bool,
    timeout: typing.SupportsFloat,
    actuate: str,
    tool_pose: bool,
    check_collision: bool,
    frame: str = "base_link",
) -> Parameter:
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

CLOSE_TO_OBSTACLE: (
    NavigationTaskStatus  # value = <NavigationTaskStatus.CLOSE_TO_OBSTACLE: 7>
)
COLLISION: NavigationTaskStatus  # value = <NavigationTaskStatus.COLLISION: 6>
COMM_DISCONNECTED: MotionStatus  # value = <MotionStatus.COMM_DISCONNECTED: 9>
CYLINDER: PrimitiveType  # value = <PrimitiveType.CYLINDER: 1>
DATA_FETCH_FAILED: MotionStatus  # value = <MotionStatus.DATA_FETCH_FAILED: 7>
EUCLIDEAN_DISTANCE: StateCheckType  # value = <StateCheckType.EUCLIDEAN_DISTANCE: 0>
FAILED: NavigationTaskStatus  # value = <NavigationTaskStatus.FAILED: 3>
FAULT: MotionStatus  # value = <MotionStatus.FAULT: 2>
FOUNDATION_STEREO: PerceptionModule  # value = <PerceptionModule.FOUNDATION_STEREO: 0>
INIT_FAILED: MotionStatus  # value = <MotionStatus.INIT_FAILED: 4>
INTERRUPTED: NavigationTaskStatus  # value = <NavigationTaskStatus.INTERRUPTED: 4>
INVALID_INPUT: MotionStatus  # value = <MotionStatus.INVALID_INPUT: 3>
IN_PROGRESS: MotionStatus  # value = <MotionStatus.IN_PROGRESS: 5>
JOINT: RobotStatesType  # value = <RobotStatesType.JOINT: 1>
LIGHT_STEREO: PerceptionModule  # value = <PerceptionModule.LIGHT_STEREO: 1>
LINE: PrimitiveType  # value = <PrimitiveType.LINE: 0>
OCCUPIED: NavigationTaskStatus  # value = <NavigationTaskStatus.OCCUPIED: 5>
POSE: RobotStatesType  # value = <RobotStatesType.POSE: 0>
PUBLISH_FAIL: MotionStatus  # value = <MotionStatus.PUBLISH_FAIL: 8>
RADIAN_DISTANCE: StateCheckType  # value = <StateCheckType.RADIAN_DISTANCE: 1>
RANDOM_PROGRESSIVE_SEED: SeedType  # value = <SeedType.RANDOM_PROGRESSIVE_SEED: 1>
RANDOM_SEED: SeedType  # value = <SeedType.RANDOM_SEED: 0>
ROBOT_STATES: RobotStatesType  # value = <RobotStatesType.ROBOT_STATES: 2>
RUNNING: NavigationTaskStatus  # value = <NavigationTaskStatus.RUNNING: 1>
STATUS_NUM: MotionStatus  # value = <MotionStatus.STATUS_NUM: 10>
STOPPED_UNREACHED: MotionStatus  # value = <MotionStatus.STOPPED_UNREACHED: 6>
SUCCESS: NavigationTaskStatus  # value = <NavigationTaskStatus.SUCCESS: 2>
TARGET_DATA_DEFAULT: int = 255
TARGET_DATA_FRAME_POSE: int = 16
TARGET_DATA_FRAME_TWIST: int = 32
TARGET_DATA_FRAME_WRENCH: int = 64
TARGET_DATA_JOINT_ACCELERATION: int = 4
TARGET_DATA_JOINT_EFFORT: int = 8
TARGET_DATA_JOINT_POSITION: int = 1
TARGET_DATA_JOINT_VELOCITY: int = 2
TARGET_DATA_NONE: int = 0
TARGET_TYPE_APPEND: int = 8
TARGET_TYPE_CLEAR: int = 2
TARGET_TYPE_DEFAULT: int = 255
TARGET_TYPE_NONE: int = 0
TARGET_TYPE_OVERRIDE: int = 10
TARGET_TYPE_PREPENDNOW: int = 4
TARGET_TYPE_PROVERRIDE: int = 14
TARGET_TYPE_TOUCH: int = 1
TIMEOUT: TerminationConditionType  # value = <TerminationConditionType.TIMEOUT: 0>
TIMEOUT_AND_EXACT_SOLUTION: TerminationConditionType  # value = <TerminationConditionType.TIMEOUT_AND_EXACT_SOLUTION: 1>
UNKNOWN: NavigationTaskStatus  # value = <NavigationTaskStatus.UNKNOWN: 0>
UNSUPPORTED_FUNCRION: MotionStatus  # value = <MotionStatus.UNSUPPORTED_FUNCRION: 11>
USER_DEFINED_SEED: SeedType  # value = <SeedType.USER_DEFINED_SEED: 2>
