#pragma once

#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "type.hpp"

namespace galbot {
namespace sdk {
namespace g1 {

class GalbotRobot {
 public:
  /**
   * @brief 获取全身控制单例对象
   *
   * @return 全身控制单例对象
   */
  static GalbotRobot& get_instance() {
    static GalbotRobot instance;
    return instance;
  }

  /**
   * @brief 初始化传感器接口依赖资源
   *
   * @param enable_sensor_set
   * 传感器枚举set，为降低资源消耗，填入set中的传感器才支持数据读取
   * @return true 初始化成功
   * @return false 初始化失败
   */
  bool init(const std::unordered_set<SensorType>& enable_sensor_set = {});

  /**
   * @brief 设置关节指令
   *
   * @param joint_commands 关节指令数组
   * @param joint_groups 要控制的关节组，默认为全身关节
   * @param joint_names 要控制的关节，该参数优先级高于joint_groups参数
   * @return ControlStatus 指令发送结果
   */
  ControlStatus set_joint_commands(const std::vector<JointCommand>& joint_commands,
                                   const std::vector<std::string>& joint_groups = {},
                                   const std::vector<std::string>& joint_names = {});
  /**
   * @brief 设置关节指令
   *
   * @param joint_commands 关节指令数组
   * @param joint_groups 要控制的关节组，默认为全身关节
   * @param joint_names 要控制的关节，该参数优先级高于joint_groups参数
   * @return ControlStatus 指令发送结果
   */
  ControlStatus set_joint_commands(const std::vector<JointCommand>& joint_commands,
                                   const std::vector<JointGroup>& joint_groups = {},
                                   const std::vector<std::string>& joint_names = {});
  /**
   * @brief 设置指定关节组的目标关节位置。
   *
   * @param joint_positions 关节角度数组（rad）
   * @param joint_groups 要控制的关节组，默认为全身关节
   * @param joint_names 要控制的关节，该参数优先级高于joint_groups参数
   * @param is_blocking 是否阻塞等待指令执行
   * @param speed_rad_s 关节运行最大速度
   * @param timeout_s 阻塞等待最大时间（秒），超时无论是否执行完毕将直接返回
   * @return ControlStatus 执行结果状态
   */
  ControlStatus set_joint_positions(const std::vector<double>& joint_positions,
                                    const std::vector<JointGroup>& joint_groups = {},
                                    const std::vector<std::string>& joint_names = {}, const bool is_blocking = true,
                                    const double speed_rad_s = 0.2, const double timeout_s = 15);
  /**
   * @brief 设置指定关节组的目标关节位置。
   *
   * @param joint_positions 关节角度数组（rad）
   * @param joint_groups 要控制的关节组，默认为全身关节
   * @param joint_names 要控制的关节，该参数优先级高于joint_groups参数
   * @param is_blocking 是否阻塞等待指令执行
   * @param speed_rad_s 关节运行最大速度
   * @param timeout_s 阻塞等待最大时间（秒），超时无论是否执行完毕将直接返回
   * @return ControlStatus 执行结果状态
   */
  ControlStatus set_joint_positions(const std::vector<double>& joint_positions,
                                    const std::vector<std::string>& joint_groups = {},
                                    const std::vector<std::string>& joint_names = {}, const bool is_blocking = true,
                                    const double speed_rad_s = 0.2, const double timeout_s = 15);
  /**
   * @brief 获取指定关节组轨迹执行状态
   *
   * @param joint_groups 指定关节组
   * @return std::vector<TrajectoryControlStatus> 轨迹执行状态
   */
  std::vector<TrajectoryControlStatus> check_trajectory_execution_status(std::vector<std::string> joint_groups);
  /**
   * @brief 获取指定关节组轨迹执行状态
   *
   * @param joint_groups 指定关节组
   * @return std::vector<TrajectoryControlStatus> 轨迹执行状态
   */
  std::vector<TrajectoryControlStatus> check_trajectory_execution_status(std::vector<JointGroup> joint_groups);
  /**
   * @brief 执行轨迹数据
   *
   * @param trajectory 轨迹数据
   * @param is_blocking 是否阻塞等待轨迹执行
   * @return ControlStatus 轨迹执行/发送结果
   */
  ControlStatus execute_joint_trajectory(const Trajectory& trajectory, bool is_blocking = true);
  /**
   * @brief 设置吸盘指令
   *
   * @param end_effector 吸盘枚举值
   * @param activate 是否激活
   * @return ControlStatus 指令发送结果
   */
  ControlStatus set_suction_cup_command(JointGroup end_effector, bool activate);
  /**
   * @brief 设置夹爪的指令
   *
   * @param end_effector 指定夹爪
   * @param width_m 目标夹爪开合宽度，单位(米)
   * @param velocity_mps 夹爪运动速度，单位(米/秒)
   * @param effort 夹爪力矩，单位牛米
   * @param is_blocking 是否阻塞等待动作完成
   * @return ControlStatus 指令执行/发送结果
   */
  ControlStatus set_gripper_command(JointGroup end_effector, double width_m, double velocity_mps = 0.03,
                                    double effort = 30, bool is_blocking = true);
  /**
   * @brief 获取夹爪状态
   *
   * @param end_effector 夹爪枚举
   * @return std::shared_ptr<GripperState> 夹爪状态
   */
  std::shared_ptr<GripperState> get_gripper_state(const JointGroup end_effector);
  /**
   * @brief 获取吸盘状态
   *
   * @param end_effector 吸盘枚举
   * @return std::shared_ptr<SuctionCupState> 吸盘状态
   */
  std::shared_ptr<SuctionCupState> get_suction_cup_state(const JointGroup end_effector);

  /**
   * @brief 获取关节位置
   *
   * @param joint_groups 要获取的关节组信息，为空获取全身关节
   * @param joint_names 要获取的关节信息，该参数优先级高于joint_groups参数
   * @return std::vector<double> 关节角度数组
   */
  std::vector<double> get_joint_positions(const std::vector<std::string>& joint_groups,
                                          const std::vector<std::string>& joint_names);

  /**
   * @brief 获取关节位置
   *
   * @param joint_groups 要获取的关节组信息，为空获取全身关节
   * @param joint_names 要获取的关节信息，该参数优先级高于joint_groups参数
   * @return std::vector<double> 关节角度数组
   */
  std::vector<double> get_joint_positions(const std::vector<JointGroup>& joint_groups,
                                          const std::vector<std::string>& joint_names = {});
  /**
   * @brief 获取机器人关节名称
   *
   * @param only_active_joint 只获取可活动关节
   * @param joint_groups 关节组字符串数组，将只获取对应关节组关节名称
   * @return std::vector<std::string> 关节名称数组
   */
  std::vector<std::string> get_joint_names(bool only_active_joint = true,
                                           const std::vector<std::string>& joint_groups = {});
  /**
   * @brief 获取机器人关节名称
   *
   * @param only_active_joint 只获取可活动关节
   * @param joint_groups 关节组枚举数组，将只获取对应关节组关节名称
   * @return std::vector<std::string> 关节名称数组
   */
  std::vector<std::string> get_joint_names(bool only_active_joint = true,
                                           const std::vector<JointGroup>& joint_groups = {});

  /**
   * @brief 获取关节实时状态
   *
   * @param joint_groups 要获取信息的关节组，默认为全身关节
   * @param joint_names 要获取信息的关节，该参数优先级高于joint_groups参数
   * @return std::vector<JointState> 关节状态数据
   */
  std::vector<JointState> get_joint_states(const std::vector<std::string>& joint_group_vec,
                                           const std::vector<std::string>& joint_names_vec = {});
  /**
   * @brief 获取关节实时状态
   *
   * @param joint_groups 要获取信息的关节组，默认为全身关节
   * @param joint_names 要获取信息的关节，该参数优先级高于joint_groups参数
   * @return std::vector<JointState> 关节状态数据
   */
  std::vector<JointState> get_joint_states(const std::vector<JointGroup>& joint_group,
                                           const std::vector<std::string>& joint_names = {});

  /**
   * @brief 设置底盘速度指令
   *
   * @param linear_velocity  线速度指令，单位 m/s，顺序为 {vx, vy, vz}
   * @param angular_velocity 角速度指令，单位 rad/s，顺序为 {wx, wy, wz}
   * @return ControlStatus 指令发送结果
   */
  ControlStatus set_base_velocity(const std::array<double, 3>& linear_velocity,
                                  const std::array<double, 3>& angular_velocity);
  /**
   * @brief 停止底盘运动
   *
   * @return ControlStatus 指令发送结果
   */
  ControlStatus stop_base();
  /**
   * @brief 停止目前执行所有轨迹
   *
   * @return ControlStatus 指令发送结果
   */
  ControlStatus stop_trajectory_execution();

  /**
   * @brief 获取imu数据
   *
   * @param imu_type 传感器枚举值
   * @return std::shared_ptr<ImuData> imu数据
   */
  std::shared_ptr<ImuData> get_imu_data(SensorType imu_type);

  /**
   * @brief 获取里程计信息
   *
   * @return std::shared_ptr<OdomData> 里程计数据，包含位置、姿态、线速度、角速度和时间戳
   */
  std::shared_ptr<OdomData> get_odom();

  /**
   * @brief 获取指定相机最新图像数据
   *
   * @param camera 指定相机
   * @return SensorStatus 执行结果
   * @return std::shared_ptr<RgbData> 图像数据
   */
  std::shared_ptr<RgbData> get_rgb_data(const SensorType rgb_camera);

  /**
   * @brief 获取指定相机最新深度图像数据
   *
   * @param camera 指定相机
   * @return SensorStatus 执行结果
   * @return std::shared_ptr<DepthData> 深度图像数据
   */
  std::shared_ptr<DepthData> get_depth_data(const SensorType depth_camera);
  /**
   * @brief 获取最新雷达数据
   *
   * @return SensorStatus 执行结果
   * @return std::shared_ptr<galbot::sensor_proto::PointCloud2> 雷达数据
   */
  std::shared_ptr<LidarData> get_lidar_data(const SensorType lidar);

  /**
   * @brief 获取指定超声波传感器的距离数据
   *
   * @param ultrasonic_type 超声波传感器探头枚举（8个方向之一）
   * @return std::shared_ptr<UltrasonicData> 超声波数据，获取失败返回nullptr
   */
  std::shared_ptr<UltrasonicData> get_ultrasonic_data(const UltrasonicType ultrasonic_type);

  /**
   * @brief 查询坐标系变换（TF） author yuxia
   *
   * @param target_frame   目标坐标系
   * @param source_frame   源坐标系
   * @param timestamp_ns   期望的变换时间戳（ns）。传入 0 表示获取最新可用 TF
   * @param timeout_ms     查询超时时间（ms）
   *
   * @return std::pair<std::vector<double>, int64_t>  如果获取失败 则返回空向量
   */
  std::pair<std::vector<double>, int64_t> get_transform(const std::string& target_frame,
                                                        const std::string& source_frame, int64_t timestamp_ns = 0,
                                                        int64_t timeout_ms = 100);

  /**
   * @brief 获取力传感器数据
   *
   * @param sensor_type 力传感器枚举值
   * @return std::shared_ptr<ForceData> 力传感器数据
   */
  std::shared_ptr<ForceData> get_force_sensor_data(const GalbotOneFoxtrotSensor sensor_type);

  /**
   * @brief 判断整体系统是否处于运行状态
   *
   * @return true 处于运行状态
   * @return false 已捕获到退出信号，准备shutdown
   */
  bool is_running();
  /**
   * @brief 主动发出SIGINT信号
   *
   */
  void request_shutdown();
  /**
   * @brief 持续睡眠，直到收到退出信号
   *
   */
  void wait_for_shutdown();
  /**
   * @brief 清理系统与中间件相关资源
   *
   */
  void destroy();
  /**
   * @brief 注册退出函数，将在收到退出信号时执行该函数
   *
   * @param exit_function 收到退出信号时执行函数
   */
  void register_exit_callback(std::function<void()> exit_function);

 private:
  GalbotRobot() = default;

  GalbotRobot(const GalbotRobot&) = delete;
  GalbotRobot& operator=(const GalbotRobot&) = delete;
  GalbotRobot(GalbotRobot&&) = delete;
  GalbotRobot& operator=(GalbotRobot&&) = delete;
};

}  // namespace g1
}  // namespace sdk
}  // namespace galbot