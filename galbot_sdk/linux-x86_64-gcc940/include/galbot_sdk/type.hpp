#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

namespace galbot {
namespace sdk {
namespace g1 {

// 导航状态
enum class NavigationStatus {
  SUCCESS,          // 执行成功
  FAIL,             // 执行失败
  TIMEOUT,          // 执行超时
  INVALID_INPUT,    // 输入参数不符合要求
  MODE_ERR,         // 模式错误不支持
  COMM_ERR,         // 通信错误
  WAIT_INITIALIZED  // 等待初始化完成中,暂不可用
};

// 执行状态
enum class ControlStatus {
  SUCCESS,            // 执行成功
  TIMEOUT,            // 执行超时
  FAULT,              // 发生故障无法继续执行
  INVALID_INPUT,      // 输入参数不符合要求
  INIT_FAILED,        // 内部通讯组件创建失败
  IN_PROGRESS,        // 正在运动中但未到位
  STOPPED_UNREACHED,  // 已停止但未到达目标
  DATA_FETCH_FAILED,  // 数据获取失败
  PUBLISH_FAIL,       // 数据发送失败
  COMM_DISCONNECTED,  // 连接失败
  STATUS_NUM          // 状态枚举总数量
};

// 执行状态
enum class SensorStatus {
  SUCCESS,            // 执行成功
  TIMEOUT,            // 执行超时
  FAULT,              // 发生故障无法继续执行
  INVALID_INPUT,      // 输入参数不符合要求
  INIT_FAILED,        // 内部通讯组件创建失败
  IN_PROGRESS,        // 正在运动中但未到位
  STOPPED_UNREACHED,  // 已停止但未到达目标
  DATA_FETCH_FAILED,  // 数据获取失败
  PUBLISH_FAIL,       // 数据发送失败
  COMM_DISCONNECTED,  // 连接失败
  STATUS_NUM          // 状态枚举总数量
};

// 执行状态
enum class MotionStatus {
  SUCCESS,              // 执行成功
  TIMEOUT,              // 执行超时
  FAULT,                // 发生故障无法继续执行
  INVALID_INPUT,        // 输入参数不符合要求
  INIT_FAILED,          // 内部通讯组件创建失败
  IN_PROGRESS,          // 正在运动中但未到位
  STOPPED_UNREACHED,    // 已停止但未到达目标
  DATA_FETCH_FAILED,    // 数据获取失败
  PUBLISH_FAIL,         // 数据发送失败
  COMM_DISCONNECTED,    // 连接失败
  STATUS_NUM,           // 状态枚举总数量
  UNSUPPORTED_FUNCRION  // 暂不支持功能
};

// 轨迹执行状态
enum class TrajectoryControlStatus {
  INVALID_INPUT,      // 输入参数不符合要求
  RUNNING,            // 正在运行中
  COMPLETED,          // 运行到位
  STOPPED_UNREACHED,  // 已停止未到达
  ERROR,              // 发生故障无法继续执行
  DATA_FETCH_FAILED,  // 执行数据获取失败
  STATUS_NUM          // 状态枚举总数量
};

// 关节组
enum class JointGroup {
  HEAD,               // 头部
  LEFT_ARM,           // 左臂
  RIGHT_ARM,          // 右臂
  LEG,                // 腿部
  CHASSIS,            // 底盘
  LEFT_GRIPPER,       // 左夹爪
  RIGHT_GRIPPER,      // 右夹爪
  LEFT_SUCTION_CUP,   // 左吸盘
  RIGHT_SUCTION_CUP,  // 右吸盘
  JOINT_GROUP_NUM     // 关节组枚举总数量
};

enum class SensorType {
  HEAD_LEFT_CAMERA,        // 头部左相机
  HEAD_RIGHT_CAMERA,       // 头部右相机
  LEFT_ARM_CAMERA,         // 左臂相机
  RIGHT_ARM_CAMERA,        // 右臂相机
  LEFT_ARM_DEPTH_CAMERA,   // 左臂深度相机
  RIGHT_ARM_DEPTH_CAMERA,  // 右臂深度相机
  BASE_LIDAR,              // 底盘激光雷达
  TORSO_IMU,               // 躯干imu
  BASE_ULTRASONIC,         // 超声波传感器
  SENSOR_NUM               // 传感器枚举值总数量
};

// 底盘超声波传感器探头枚举（8个方向）
enum class UltrasonicType {
  FRONT_LEFT,   // 前左
  FRONT_RIGHT,  // 前右
  RIGHT_LEFT,   // 右左
  RIGHT_RIGHT,  // 右右
  BACK_LEFT,    // 后左
  BACK_RIGHT,   // 后右
  LEFT_LEFT,    // 左左
  LEFT_RIGHT,   // 左右
  ULTRASONIC_NUM
};

// 力传感器枚举
enum class GalbotOneFoxtrotSensor {
  LEFT_WRIST_FORCE,   // 左腕力传感器
  RIGHT_WRIST_FORCE,  // 右腕力传感器
  FORCE_NUM,          // 力传感器枚举值总数量
};

struct DeviceInfo {
  std::string model;             // 设备型号
  std::string serial_number;     // 序列号
  std::string firmware_version;  // 系统固件版本
  std::string hardware_version;  // 硬件版本
  std::string manufacturer;      // 制造商
};

struct UltrasonicData {
  int64_t timestamp;  // 时间戳，单位秒
  double distance;    // 距离，单位米
};

// 运动链类型
enum class ChainType { HEAD, LEFT_ARM, RIGHT_ARM, LEG, TORSO, CHAIN_NUM };

// 单个关节指令
struct JointCommand {
  double position;      // 位置命令（弧度）
  double velocity;      // 速度命令（弧度/秒）
  double acceleration;  // 加速度命令
  double effort;        // 力矩命令（牛米）
};

// 单个轨迹点
struct TrajectoryPoint {
  double time_from_start_second;                // 目标到达时间，值为距离轨迹开始执行时间，单位秒
  std::vector<JointCommand> joint_command_vec;  // 具体执行关节指令
};

// 轨迹
struct Trajectory {
  std::vector<TrajectoryPoint> points;
  std::vector<std::string> joint_groups;
  std::vector<std::string> joint_names;
};

struct Error {
  std::string commpent;     // 故障模块
  uint64_t error_code;      // 故障代码
  std::string description;  // 错误描述

  Error(std::string commpent_input, int error_code_input, std::string description_input)
      : commpent(std::move(commpent_input)), error_code(error_code_input), description(std::move(description_input)) {}
};

struct ErrorInfo {
  int64_t timestamp_ns;
  std::vector<Error> error_vec;
};

struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose {
  Point position;
  Quaternion orientation;
  Pose() = default;

  template <typename T>
  Pose(const T& pos, const T& quat) {  // quat顺序xyzw
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

enum class ActuateType { ACTUATE_WITH_CHAIN_ONLY, ACTUATE_WITH_TORSO, ACTUATE_WITH_LEG, ACTUATE_TYPE_NUM };

enum class SeedType { RANDOM_SEED, RANDOM_PROGRESSIVE_SEED, USER_DEFINED_SEED, SEED_TYPE_NUM };

enum ReferenceFrame { FRAME_WORLD, FRAME_BASE };

// todo 末端frame，参考frame
struct PlannerConfig {
  bool is_direct_execute = false;                                        // 规划后是否执行轨迹
  bool is_blocking = false;                                              // 是否同步等待轨迹执行到位或规划完成
  double timeout_second = 20;                                            // 最大等待时长（秒）
  ActuateType actuate_type = ActuateType::ACTUATE_WITH_CHAIN_ONLY;       // 规划类型，是否带腰或腿部规划
  bool is_tool_pose = false;                                             // 是否为工具末端位姿
  bool is_relative_pose = false;                                         // 是否为相对点位
  bool is_check_collision = true;                                        // 是否检查碰撞
  std::string reference_frame = "base_link";                             // 参考坐标系
  std::unordered_map<JointGroup, std::vector<double>> joint_state = {};  // 初始关节状态
  bool move_line = false;
};

struct PlanTaskResult {
  std::string task_id;

  bool success;
  int error_code;
  std::string error_message;

  // 轨迹结果
  struct Trajectory {
    std::vector<std::vector<double>> joint_positions;
    std::vector<double> timestamps;
  } trajectory;

  // IK 结果
  std::unordered_map<std::string, std::vector<double>> ik_result;

  // FK 结果
  std::unordered_map<std::string, Pose> fk_result;

  // 碰撞结果（可选）
  std::vector<double> collision_result;
};

struct JointState {
  double position;      // 关节当前位置（弧度）
  double velocity;      // 关节当前速度（弧度/秒）
  double acceleration;  // 关节当前加速度（弧度/秒²）
  double effort;        // 关节力矩（牛米）
  double current;       // 电机电流（安培）
  JointState() = default;
  JointState(double position_input, double velocity_input, double acceleration_input, double effort_input,
             double current_input)
      : position(position_input),
        velocity(velocity_input),
        acceleration(acceleration_input),
        effort(effort_input),
        current(current_input) {}
};

struct JointStateMessage {
  int64_t timestamp_ns;
  std::vector<JointState> joint_state_vec;
};

struct TransformMessage {
  int64_t timestamp_ns;
  Point translation;
  Quaternion rotation;
};

struct Vector3 {
  double x;
  double y;
  double z;
};
// 六维力数据
struct EffortInfo {
  int64_t timestamp_ns;
  Vector3 force;   // 力
  Vector3 torque;  // 力矩
};

// 力传感器数据
struct ForceData {
  double timestamp_s;
  Vector3 force;
  Vector3 torque;
};

// IMU数据
struct ImuData {
  int64_t timestamp_ns;
  Vector3 accel;   // 加速度
  Vector3 gyro;    // 陀螺仪
  Vector3 magnet;  // 磁力计
};

// 里程计数据
struct OdomData {
  int64_t timestamp;                  // 时间戳（秒）
  std::array<double, 3> position;     // 位置 [x, y, z]（单位：m）
  std::array<double, 4> orientation;  // 姿态四元数 [x, y, z, w]（单位：无）

  /** 暂时无法获取
  std::array<double, 3> linear_velocity; // 线速度 [vx, vy, vz]（单位：m/s）
  std::array<double, 3> angular_velocity; // 角速度 [wx, wy, wz]（单位：rad/s）
   */
};

// 夹爪状态
struct GripperState {
  int64_t timestamp_ns;
  double width;                         // 夹爪宽度（m）
  double velocity;                      // 夹爪速度（m/s）
  double effort;                        // 夹爪夹持力（N）
  bool is_moving = false;               // 是否正在运动
  std::vector<double> joint_positions;  // 关节位置
};

enum class SUCTION_ACTION_STATE {
  suction_action_idle,     // 没有吸取
  suction_action_sucking,  // 正在吸取中
  suction_action_success,  // 压力变小 则吸取成功
  suction_action_failed,   // 吸取过程没有释放 失败
};

struct SuctionCupState {
  int64_t timestamp_ns;
  bool activation;  // 当前是否正在吸
  double pressure;  // 当前压力（pa）
  SUCTION_ACTION_STATE action_state;
};

/**
 * @brief 时间戳结构体
 *
 * 用于表示高精度的时间点，通常包含秒和纳秒两部分。
 * 对应于 ROS 2 标准中的 builtin_interfaces/Time 或 std_msgs/Header 中的 stamp。
 */
struct Timestamp {
  /**
   * @brief 秒 (Seconds)
   *
   * 自 UNIX Epoch (1970-01-01 00:00:00 UTC) 以来的秒数。
   */
  int64_t sec;

  /**
   * @brief 纳秒 (Nanoseconds)
   *
   * 秒内的纳秒部分，范围通常为 [0, 999,999,999]。
   */
  uint32_t nanosec;
};

/**
 * @brief 消息头结构体
 *
 * 用于记录数据产生的时间和参考坐标系
 */
struct Header {
  /**
   * @brief 数据采集的时间戳
   */
  Timestamp stamp;

  /**
   * @brief 坐标系 ID (Frame ID)
   *
   * 标识点云数据是在哪个坐标系下采集的，例如 "lidar_link" 或 "map"
   */
  std::string frame_id;
};

/**
 * @brief RGB/彩色图像数据结构体
 *
 * 用于存储压缩后的普通图像（如 JPEG, PNG）。
 * 通常用于可视化的摄像头数据。
 */
struct RgbData {
  /**
   * @brief 消息头
   * 包含采集时间戳和相机坐标系 ID。
   */
  Header header;

  /**
   * @brief 图像格式
   * 例如: "jpeg", "png", "bgr8; jpeg compressed bgr8"
   */
  std::string format;

  /**
   * @brief 压缩的图像数据
   * 二进制数据流。
   */
  std::vector<uint8_t> data;

  /**
   * @brief 将内部存储的压缩数据解码为 OpenCV Mat 对象
   *
   * 该函数使用 cv::imdecode 对二进制流进行解码。
   *
   * @note
   * 1. 默认解码为 BGR 格式（OpenCV 标准），而非 RGB。
   * 2. 如果数据为空或解码失败，将返回 nullptr。
   *
   * @return std::shared_ptr<cv::Mat>
   *         成功时返回指向解码后图像的智能指针；
   *         失败时返回 nullptr。
   */
  std::shared_ptr<cv::Mat> convert_to_cv2_mat();
};

/**
 * @brief 深度图像数据结构体
 *
 * 用于存储压缩后的深度图像。
 * 包含额外的 depth_scale 用于恢复真实的深度数值。
 */
struct DepthData {
  /**
   * @brief 消息头
   */
  Header header;

  uint32_t height;

  uint32_t width;

  /**
   * @brief 图像格式
   * 例如: "16UC1; compressedDepth png"
   */
  std::string format;

  /**
   * @brief 压缩的深度数据
   */
  std::vector<uint8_t> data;

  /**
   * @brief 深度缩放比例/量化因子
   *
   * 用于将压缩或量化的像素值还原为物理距离（通常单位为毫米或米）。
   */
  uint32_t depth_scale;

  std::shared_ptr<cv::Mat> convert_to_cv2_mat();
};

/**
 * @brief 点云字段描述符
 *
 * 这个消息保存了 PointCloud2 消息格式中一个点条目的描述。
 * 它定义了数据在二进制 blob 中的布局方式。
 */
struct PointField {
  /**
   * @brief 数据类型枚举
   *
   * 定义了点云数据中数值的存储类型，对应字节大小和解析方式。
   */
  enum DataType : uint8_t {
    UNKNOWN = 0,
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8
  };

  /**
   * @brief 字段名称
   *
   * 常见的字段名包括:
   * - "x", "y", "z": 笛卡尔坐标
   * - "intensity": 反射强度
   * - "rgb": 颜色信息
   * - "ring": 线束索引
   */
  std::string name;

  /**
   * @brief 字节偏移量
   *
   * 该字段在单个点的结构体起始位置的字节偏移量。
   * 例如：若 x 是 float32 (4字节)，则 y 的偏移量通常为 4。
   */
  uint32_t offset;

  /**
   * @brief 数据类型
   *
   * 引用上方的 DataType 枚举，指明该字段的数据格式。
   */
  DataType datatype;

  /**
   * @brief 元素数量
   *
   * 该字段包含多少个元素。通常对于标量（如 x, intensity）为 1。
   */
  uint32_t count;
};

/**
 * @brief 激光雷达数据结构体
 *
 * 这是一个通用的 N 维点集合结构，与 ROS 2 的 sensor_msgs/msg/PointCloud2 对应。
 * 核心数据存储在 data 字节数组中，通过 fields 描述如何解析。
 */
struct LidarData {
  /**
   * @brief 消息头
   *
   * 包含时间戳和坐标系信息，用于时间同步和坐标变换。
   */
  Header header;

  /**
   * @brief 点云的高度
   *
   * - 如果是无序点云 (Unordered Cloud)，height 为 1。
   * - 如果是有序点云 (Ordered Cloud，如来自深度相机或类似图像结构的雷达)，
   *   height 表示行数。
   */
  uint32_t height;

  /**
   * @brief 点云的宽度
   *
   * - 如果是无序点云，width 表示点云中点的总数。
   * - 如果是有序点云，width 表示一行的点数。
   */
  uint32_t width;

  /**
   * @brief 字段描述列表
   *
   * 描述了二进制 data 数据中，每个点包含哪些通道（x, y, z, ...）及其布局。
   */
  std::vector<PointField> fields;

  /**
   * @brief 大端序标识
   *
   * 如果数据是 Big Endian 格式，则为 true；否则为 Little Endian（通常为
   * false）。
   */
  bool is_bigendian;

  /**
   * @brief 点步长 (Point Step)
   *
   * 单个点占用的总字节数。
   * 计算方式通常为所有 fields 占用字节数的总和（需考虑内存对齐）。
   */
  uint32_t point_step;

  /**
   * @brief 行步长 (Row Step)
   *
   * 一行数据占用的总字节数。
   * 计算公式: row_step = point_step * width
   */
  uint32_t row_step;

  /**
   * @brief 点云二进制数据
   *
   * 存储实际点数据的二进制 Blob。
   * 数组大小应为: row_step * height
   */
  std::vector<uint8_t> data;

  /**
   * @brief 稠密数据标识
   *
   * - true: 数据中没有无效点（如 NaN 或 Inf）。
   * - false: 数据中可能包含无效点。
   */
  bool is_dense;
};

/**
 * @brief 感兴趣区域 (ROI)
 *
 * 对应 ROS 2 的 sensor_msgs/msg/RegionOfInterest
 * 用于指定图像中的一个子窗口。
 */
struct RegionOfInterest {
  /**
   * @brief ROI 最左侧像素的 x 坐标
   *
   * 0 表示 ROI 包含图像左边缘。
   */
  uint32_t x_offset;

  /**
   * @brief ROI 最顶端像素的 y 坐标
   *
   * 0 表示 ROI 包含图像上边缘。
   */
  uint32_t y_offset;

  /**
   * @brief ROI 的高度 (像素)
   */
  uint32_t height;

  /**
   * @brief ROI 的宽度 (像素)
   */
  uint32_t width;

  /**
   * @brief 是否进行矫正
   *
   * - true: 表示应该从该 ROI 计算出一个独特的矫正后 ROI。
   * - false: 通常表示捕获的是全分辨率图像，或者不应用 ROI 逻辑。
   */
  bool do_rectify;
};

/**
 * @brief 相机标定信息
 *
 * 对应 ROS 2 的 sensor_msgs/msg/CameraInfo。
 * 包含相机的内参、畸变参数以及外参等信息。
 */
struct CameraInfo {
  /**
   * @brief 消息头
   * 包含时间戳和坐标系 ID (例如 "camera_optical_frame")。
   */
  Header header;

  /**
   * @brief 图像高度 (像素)
   * 标定时的图像分辨率高度。
   */
  uint32_t height;

  /**
   * @brief 图像宽度 (像素)
   * 标定时的图像分辨率宽度。
   */
  uint32_t width;

  /**
   * @brief 畸变模型
   *
   * 通常是 "plumb_bob" (径向和切向畸变)。
   */
  std::string distortion_model;

  /**
   * @brief 畸变参数 (D)
   *
   * 大小取决于畸变模型。对于 "plumb_bob"，通常有 5 个参数: (k1, k2, t1, t2,
   * k3)。
   */
  std::vector<double> d;

  /**
   * @brief 内参矩阵 (K)
   *
   * 3x3 矩阵，行优先存储 (Row-major)。
   * [fx  0 cx]
   * [ 0 fy cy]
   * [ 0  0  1]
   * 用于将相机坐标系下的 3D 点投影到 2D 像素坐标。
   */
  std::array<double, 9> k;

  /**
   * @brief 旋转/矫正矩阵 (R)
   *
   * 3x3 矩阵，行优先存储。
   * 仅用于立体相机 (Stereo)，用于将两个相机的图像平面旋转到共面（行对齐）。
   * 对于单目相机，通常为单位矩阵。
   */
  std::array<double, 9> r;

  /**
   * @brief 投影矩阵 (P)
   *
   * 3x4 矩阵，行优先存储。
   * [fx'  0  cx' Tx]
   * [ 0  fy' cy' Ty]
   * [ 0   0   1   0]
   * 用于将 3D 点投影到矫正后的图像坐标系。
   */
  std::array<double, 12> p;

  /**
   * @brief X 方向的合并像素 (Binning)
   *
   * 0 或 1 表示不进行合并。
   */
  uint32_t binning_x;

  /**
   * @brief Y 方向的合并像素 (Binning)
   */
  uint32_t binning_y;

  /**
   * @brief 感兴趣区域 (ROI)
   *
   * 原始全分辨率图像中的子窗口。
   */
  RegionOfInterest roi;

  // ==========================================
  // 下面是 Proto 定义中包含但标准 ROS 2 CameraInfo 中没有的字段
  // ==========================================

  /**
   * @brief 相机类型
   */
  std::string camera_type;

  /**
   * @brief 变换矩阵 T
   */
  std::vector<double> T;
};

}  // namespace g1
}  // namespace sdk
}  // namespace galbot
