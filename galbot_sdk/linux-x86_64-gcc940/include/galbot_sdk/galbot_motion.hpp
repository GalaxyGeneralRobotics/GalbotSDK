#pragma once

#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "motion_plan_config.hpp"
#include "type.hpp"

namespace galbot {
namespace sdk {
namespace g1 {
// 全身规划map
static std::unordered_map<std::string, ActuateType> g_actuate_type_map = {
    {"with_chain_only", ActuateType::ACTUATE_WITH_CHAIN_ONLY},
    {"with_torso", ActuateType::ACTUATE_WITH_TORSO},
    {"with_leg", ActuateType::ACTUATE_WITH_LEG},
    {"type_num", ActuateType::ACTUATE_TYPE_NUM}};

static std::unordered_map<MotionStatus, std::string> status_string_map_ = {
    {MotionStatus::SUCCESS, "SUCCESS: 执行成功"},
    {MotionStatus::TIMEOUT, "TIMEOUT: 执行超时"},
    {MotionStatus::FAULT, "FAULT: 发生故障无法继续执行"},
    {MotionStatus::INVALID_INPUT, "INVALID_INPUT: 输入参数不符合要求"},
    {MotionStatus::INIT_FAILED, "INIT_FAILED: 内部通讯组件创建失败"},
    {MotionStatus::IN_PROGRESS, "IN_PROGRESS: 正在运动中但未到位"},
    {MotionStatus::STOPPED_UNREACHED, "STOPPED_UNREACHED: 已停止但未到达目标"},
    {MotionStatus::DATA_FETCH_FAILED, "DATA_FETCH_FAILED: 数据获取失败"},
    {MotionStatus::PUBLISH_FAIL, "PUBLISH_FAIL: 数据发送失败"},
    {MotionStatus::COMM_DISCONNECTED, "COMM_DISCONNECTED: 连接失败"},
    {MotionStatus::STATUS_NUM, "STATUS_NUM: 状态枚举总数量"},
    {MotionStatus::UNSUPPORTED_FUNCRION, "UNSUPPORTED_FUNCRION: 暂不支持功能"}};

class Parameter : public PlannerConfig {
 public:
  Parameter() = default;
  Parameter(bool direct_execute, bool blocking, double timeout, std::string actuate, bool tool_pose,
            bool check_collision, const std::string& frame = "base_link",
            std::unordered_map<std::string, std::vector<double>> custom_joint_state = {}) {
    is_direct_execute = direct_execute;
    is_blocking = blocking;
    timeout_second = timeout;
    actuate_type = g_actuate_type_map[actuate];
    is_tool_pose = tool_pose;
    is_check_collision = check_collision;
    // for (const auto &pair: custom_joint_state) {
    //     const auto &key = pair.first;
    //     const auto &double_vec = pair.second;
    //     joint_state[g_custom_joint_group_map[key]] = double_vec;
    // }
  }
  // set
  void setDirectExecute(bool direct_execute) { is_direct_execute = direct_execute; }
  void setBlocking(bool blocking) { is_blocking = blocking; }
  void setTimeout(double timeout) { timeout_second = timeout; }
  void setActuate(const std::string& actuate) { actuate_type = g_actuate_type_map[actuate]; }
  void setToolPose(bool tool_pose) { is_tool_pose = tool_pose; }
  void setCheckCollision(bool check_collision) { is_check_collision = check_collision; }
  void setReferenceFrame(const std::string& frame) { reference_frame = frame; }
  // void setJointState(const std::string &key, const std::vector<double>
  // &joint_values) {
  //     // joint_state[g_custom_joint_group_map[key]] = joint_values;
  // }
  void setMoveLine(bool flag) { move_line = flag; }
  // get
  bool getDirectExecute() const { return is_direct_execute; }
  bool getBlocking() const { return is_blocking; }
  double getTimeout() const { return timeout_second; }
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
  bool getToolPose() const { return is_tool_pose; }
  bool getCheckCollision() const { return is_check_collision; }
  std::string getReferenceFrame() const { return reference_frame; }
  // std::vector<double> getJointState(const std::string &key) const {
  //     auto it = joint_state.find(g_custom_joint_group_map.at(key));
  //     if (it != joint_state.end()) {
  //         return it->second;
  //     }
  //     return {};
  // }
  bool isMoveLine() { return move_line; }
};
static std::shared_ptr<Parameter> default_param = std::make_shared<Parameter>();
class RobotStates {
 public:
  enum class Type { POSE, JOINT, ROBOT_STATES };
  virtual Type getType() const { return Type::ROBOT_STATES; };
  std::string chain_name = "";           // 单链
  std::vector<double> whole_body_joint;  // 全身关节角
  std::vector<double> base_state;        // 底盘位姿（x,y,z,rx,ry,rz,rw)

  RobotStates() = default;
  void setWholeBodyJoint(const std::vector<double>& joint_positions) { whole_body_joint = joint_positions; }

  void setBaseState(const Pose& base_pose) {
    base_state = {base_pose.position.x,    base_pose.position.y,    base_pose.position.z,   base_pose.orientation.x,
                  base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w};
  }

  RobotStates(const std::string& chain, const std::vector<double>& whole_joint, const Pose& base_pose)
      : chain_name(chain), whole_body_joint(whole_joint) {
    base_state = {base_pose.position.x,    base_pose.position.y,    base_pose.position.z,   base_pose.orientation.x,
                  base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w};
  }
};

class PoseState : public RobotStates {
 public:
  Type getType() const override { return Type::POSE; }
  std::string frame_id = "EndEffector";       // "EndEffector", "Camera"
  std::string reference_frame = "base_link";  // "base_link", "world"
  //   Pose start_pose; //如果是单点规划，则只包含一个目标位姿
  Pose pose;

  //   std::unordered_map<std::string, std::vector<std::vector<double>>>
  //       multi_chain_pose_configs; // 多链
};

class JointStates : public RobotStates {
 public:
  Type getType() const override { return Type::JOINT; }
  //   std::vector<double> start_joint_positions; //
  //   如果是单点规划，则只包含一个目标关节角
  std::vector<double> joint_positions;
  void set_joint_positions(const std::vector<double>& joints) { joint_positions = joints; }
  void set_joint(int index, int val) {
    if (joint_positions.size() > index && index >= 0) {
      joint_positions[index] = val;
    }
  }
};

class GalbotMotion {
 public:
  /**
   * @brief 获取GalbotMotion单例对象
   *
   * @return GalbotMotion& GalbotMotion单例对象
   */
  static GalbotMotion& get_instance() {
    static GalbotMotion instance;
    return instance;
  }

  /**
   * @brief 初始化系统接口依赖资源
   *
   * @return true 初始化成功
   * @return false 初始化失败
   */
  bool init();

  bool is_valid();

  /**
   * @brief 正运动学求解
   * @param  target_frame  目标关节link名称
   * @param  joint_state 链（组）初始关节状态
   * @param  reference_frame 目标参考坐标系: world/base_link
   * @param  param 配置参数
   * @return  执行状态, 求解结果(x,y,z,rx,ry,rz,rw)
   */
  std::tuple<MotionStatus, std::vector<double>> forward_kinematics(
      const std::string& target_frame, const std::string& reference_frame = "base_link",
      const std::unordered_map<std::string, std::vector<double>>& joint_state = {},
      std::shared_ptr<Parameter> params = default_param);
  /**
   * @brief 正运动学求解
   *
   * @param target_frame 目标关节link名称
   * @param reference_robot_states 全身关节状态,
   * RobotStates类型，为空时，使用当前关节状态
   * @param reference_frame  目标参考坐标系: world/base_link
   * @param params
   * @return std::tuple<MotionStatus, std::vector<double>>
   */
  std::tuple<MotionStatus, std::vector<double>> forward_kinematics_by_state(
      const std::string& target_frame, const std::shared_ptr<RobotStates>& reference_robot_states = nullptr,
      const std::string& reference_frame = "base_link", std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief 逆运动学求解
   *
   * @param target_pose 目标位姿
   * @param chain_names 链名，可指定手臂+腿或手臂+腰组合
   * @param target_frame  目标位姿所在坐标系: EndEffector/Camera
   * @param reference_frame   目标参考坐标系: world/base_link
   * @param initial_joint_positions 参考链关节状态，为空时，使用当前关节状态
   * @param enable_collision_check    求解时考虑碰撞
   * @param params    可选参数
   * @return 状态+求解链关节值
   */
  std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<double>>> inverse_kinematics(
      const std::vector<double>& target_pose, const std::vector<std::string>& chain_names,
      const std::string& target_frame = "EndEffector", const std::string& reference_frame = "base_link",
      const std::unordered_map<std::string, std::vector<double>>& initial_joint_positions = {},
      const bool& enable_collision_check = true, std::shared_ptr<Parameter> params = default_param);
  /**
   * @brief   逆运动学求解
   *
   * @param target_pose 目标位姿
   * @param chain_names 链名，可指定手臂+腿或手臂+腰组合
   * @param target_frame 目标位姿所在坐标系: EndEffector/Camera
   * @param reference_frame 目标参考坐标系: world/base_link
   * @param reference_robot_states 全身关节状态,
   * RobotStates类型，为空时，使用当前关节状态
   * @param enable_collision_check 求解时考虑碰撞
   * @param params 可选参数
   * @return 状态+求解链关节值
   */
  std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<double>>> inverse_kinematics_by_state(
      const std::vector<double>& target_pose, const std::vector<std::string>& chain_names,
      const std::string& target_frame = "EndEffector", const std::string& reference_frame = "base_link",
      const std::shared_ptr<RobotStates>& reference_robot_states = nullptr, const bool& enable_collision_check = true,
      std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief
   * 获取左右手臂末端执行器位姿，从TF中获取，要求end_effector_frame在urdf模型中定义
   *
   * @param end_effector_frame 目标link
   * @param reference_frame   参考坐标系
   * @return 执行状态 + 求解结果(x,y,z,rx,ry,rz,rw)
   */
  std::tuple<MotionStatus, std::vector<double>> get_end_effector_pose(const std::string& end_effector_frame,
                                                                      const std::string& reference_frame = "base_link");
  /**
   * @brief 获取左右臂末端位姿，从TF中获取
   *
   * @param chain_name 左右臂链: left_arm/right_arm
   * @param frame_id  末端坐标系: EndEffector/Camera
   * @param reference_frame 目标参考坐标系: world/base_link
   * @return 执行状态 + 求解结果(x,y,z,rx,ry,rz,rw)
   */
  std::tuple<MotionStatus, std::vector<double>> get_end_effector_pose_on_chain(
      const std::string& chain_name,
      const std::string frame_id = "EndEffector",  // "EndEffector", "Camera"
      const std::string& reference_frame = "base_link");

  /**
   * @brief 设置末端执行器运动至目标位姿（直线）
   *
   * @param target_pose   目标笛卡尔位姿
   * @param end_effector_frame   left_arm/right_arm, 和chain_name一致
   * @param reference_frame  目标参考坐标系: world/base_link
   * @param reference_robot_states 全身关节状态,
   * RobotStates类型，为空时，使用当前关节状态
   * @param enable_collision_check 是否考虑碰撞
   * @param is_blocking   是否阻塞等待运动完成
   * @param timeout
   * 阻塞等待超时时间，设置值小于0且is_blocking为true时，使用params中超时参数
   * @param params    可选参数
   * @return MotionStatus
   */
  MotionStatus set_end_effector_pose(const std::vector<double>& target_pose, const std::string& end_effector_frame,
                                     const std::string& reference_frame = "base_link",
                                     std::shared_ptr<RobotStates> reference_robot_states = nullptr,
                                     const bool& enable_collision_check = true, const bool& is_blocking = true,
                                     const double& timeout = -1.0, std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief 单链规划
   *
   * @param target    目标链状态，可选PoseState/JointStates,不可为其他类型或为空
   * @param start     链起始状态（规划起始点），可为空或JointStates
   * @param reference_robot_states
   * 全身参考状态，为空时，将使用当前全身状态，start不为空时，对应链关节状态将被覆盖为start的关节值
   * @param enable_collision_check    是否考虑碰撞
   * @param params    可选参数
   * @return 执行状态 + 规划轨迹
   */
  std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>> motion_plan(
      std::shared_ptr<RobotStates> target, std::shared_ptr<RobotStates> start = nullptr,
      std::shared_ptr<RobotStates> reference_robot_states = nullptr, bool enable_collision_check = true,
      std::shared_ptr<Parameter> params = default_param);

  /**
   * @brief 单链多路点规划
   *
   * @param target
   * 用于表明路点类型，type为PoseState/JointStates，并指定chain_name
   * @param targets   规划路点
   * @param start     链起始状态（规划起始点），可为空或JointStates
   * @param reference_robot_states
   * 全身参考状态，为空时，将使用当前全身状态，start不为空时，对应链关节状态将被覆盖为start的关节值
   * @param enable_collision_check 是否考虑碰撞
   * @param params 可选参数
   * @return 执行状态 + 规划轨迹
   */
  std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>>
  motion_plan_multi_waypoints(std::shared_ptr<RobotStates> target, std::vector<std::vector<double>> targets,
                              std::shared_ptr<RobotStates> start = nullptr,
                              std::shared_ptr<RobotStates> reference_robot_states = nullptr,
                              bool enable_collision_check = true, std::shared_ptr<Parameter> params = default_param);

   std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>>
   motion_plan_multi_waypoints(std::unordered_map<std::shared_ptr<RobotStates>, std::vector<std::vector<double>>> targets, std::vector<std::shared_ptr<RobotStates>> start = {},
                              std::shared_ptr<RobotStates> reference_robot_states = nullptr,
                              bool enable_collision_check = true, std::shared_ptr<Parameter> params = default_param);


  /**
   * @brief 检测全身状态是否碰撞
   *
   * @param start 待检测的全身状态队列，RobotStates类型为RobotStates/单链JointStates
   * @param enable_collision_check
   * true时，检测自碰撞及与环境碰撞情况，false时，仅进行自碰撞检测
   * @param params    可选参数
   * @return 执行状态 + 检测结果
   */
  std::tuple<MotionStatus, std::vector<bool>> check_collision(const std::vector<std::shared_ptr<RobotStates>>& start,
                                                              bool enable_collision_check = true,
                                                              std::shared_ptr<Parameter> params = default_param);
  /**
   * @brief 附加工具
   *
   * @param chain 装载工具的链，一般为左右臂: left_arm/right_arm
   * @param tool  装载的工具名称
   * @return MotionStatus
   */
  MotionStatus attach_tool(const std::string& chain, const std::string& tool);
  /**
   * @brief 卸载工具
   *
   * @param chain 装载工具的链，一般为左右臂: left_arm/right_arm
   * @return MotionStatus
   */
  MotionStatus detach_tool(const std::string& chain);
  /**
   * @brief 加载碰撞体至环境中
   *
   * @param obstacle_id 碰撞体id，唯一标识符号，不可重复，用于移除对应碰撞体
   * @param obstacle_type 碰撞体类型
   * @param pose  碰撞体加载位置与姿态
   * @param scale 几何体尺寸，box: 长/宽/高，sphere: 半径/-/-，cylinder:
   * 半径/高度/-
   * @param key   mesh/point_cloud: 数据文件路径，depth_image:
   * 相机类型，front_head/left_arm/right_arm
   * @param target_frame  加载目标坐标系: world/base_link/chain名称
   * @param ee_frame
   * target_frame为chain时，可指定目标坐标系为末端(ee_base)/相机(camera_base/camera_object)
   * @param reference_joint_positions
   * 加载时机器人全身关节状态，为空时，使用当前全身状态
   * @param reference_base_pose
   * 加载时机器人在map坐标系下位姿，为空时，使用当前定位
   * @param ignore_collision_link_names   加载碰撞体忽略与指定link不作碰撞检测
   * @param safe_margin   碰撞体表面与机器人的安全距离，小于该值时，认为碰撞
   * @param resolution    部分类型碰撞体加载时精度
   * @return MotionStatus
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
   * @brief 移除指定碰撞体
   *
   * @param obstacle_id
   * @return MotionStatus
   */
  MotionStatus remove_obstacle(const std::string& obstacle_id);
  /**
   * @brief 移除所有加载的碰撞体
   *
   * @return MotionStatus
   */
  MotionStatus clear_obstacle();
  /**
   * @brief 加载碰撞体固接于机器人上
   *
   * @param obstacle_id 碰撞体id，唯一标识符号，不可重复，用于移除对应碰撞体
   * @param obstacle_type 碰撞体类型
   * @param pose  碰撞体加载位置与姿态
   * @param scale 几何体尺寸，box: 长/宽/高，sphere: 半径/-/-，cylinder:
   * 半径/高度/-
   * @param key   mesh/point_cloud: 数据文件路径，depth_image:
   * 相机类型，front_head/left_arm/right_arm
   * @param target_frame  加载目标坐标系: world/base_link/chain名称
   * @param ee_frame
   * target_frame为chain时，可指定目标坐标系为末端(ee_base)/相机(camera_base/camera_object)
   * @param reference_joint_positions
   * 加载时机器人全身关节状态，为空时，使用当前全身状态
   * @param reference_base_pose
   * 加载时机器人在map坐标系下位姿，为空时，使用当前定位
   * @param ignore_collision_link_names   加载碰撞体忽略与指定link不作碰撞检测
   * @param safe_margin   碰撞体表面与机器人的安全距离，小于该值时，认为碰撞
   * @param resolution    部分类型碰撞体加载时精度
   * @return MotionStatus
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
   * @brief 移除股接于机器人上的碰撞体
   *
   * @param obstacle_id
   * @return MotionStatus
   */
  MotionStatus detach_target_object(const std::string& obstacle_id);

  MotionStatus set_motion_plan_config(std::shared_ptr<MotionPlanConfig> config);
  std::tuple<MotionStatus, MotionPlanConfig> get_motion_plan_config();

  /**
   * @brief 获取机器人连杆的名称列表。
   * @param only_end_effector 是否仅返回末端执行器连杆。默认为false，获取所有连杆（包括中间连杆和基座连杆）。
   * @return 连杆名称列表
   */
  std::vector<std::string> get_link_names(bool only_end_effector = false);

 public:
  const std::set<std::string>& getSupportLinks();
  const std::set<std::string>& getSupportChains();
  const std::set<std::string>& getSupportFrame();
  const std::set<std::string>& getSupportEEFrame();
  const std::set<std::string>& getSupportToolList();
  std::set<std::string> getSupportObstacleType();
  std::vector<std::string> getBuiltObjectList();

  bool isLinkNameValid(const std::string& value, bool throw_exception = false);
  bool isChainNameValid(const std::string& value, bool throw_exception = false);
  bool isFrameNameValid(const std::string& value, bool throw_exception = false);
  bool isEEFrameValid(const std::string& value, bool throw_exception = false);
  bool isToolNameValid(const std::string& value, bool throw_exception = false);
  bool isObstacleTypeValid(const std::string& value, bool throw_exception = false);
  bool isWholeBodyStateValid(const std::vector<double>& value, bool throw_exception = false);
  bool isPose7dValid(const std::vector<double>& value, bool throw_exception = false);
  bool isChainJointStateValid(const std::unordered_map<std::string, std::vector<double>>& value,
                              bool throw_exception = false);

  int getRobotDof() { return robot_dof_; }
  RobotStates getRobotStates();
  std::vector<double> getWholeBodyState();
  std::vector<double> getChassisState();
  std::unordered_map<std::string, std::vector<double>> getChainJointState();
  std::unordered_map<std::string, std::vector<double>> getChainPoseState(const std::string& frame = "base_link");
  bool replace_joint_state(const std::string& chain_name, const std::vector<double>& chain_joint,
                           std::vector<double>& whole_body_joint);

  std::string status_to_string(MotionStatus status);

 private:
  /**
   * @brief chain_names检查,输出actuate_type
   */
  std::tuple<std::string, std::string> validateAndGetChainType(const std::vector<std::string>& chain_names);

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
  MotionStatus add_object(const std::string& attached_frame, const std::string& obstacle_id,
                          const std::string& obstacle_type, const std::vector<double>& pose,
                          const std::array<double, 3>& scale = {}, const std::string& key = "",
                          const std::string& target_frame = "world", const std::string& ee_frame = "ee_base",
                          const std::vector<double>& reference_joint_positions = {},
                          const std::vector<double>& reference_base_pose = {},
                          const std::vector<std::string>& ignore_collision_link_names = {},
                          const double& safe_margin = 0, const double& resolution = 0.01);

  MotionStatus remove_object(const std::string& attached_frame, const std::string& obstacle_id);

  MotionStatus clear_object(const std::string& attached_frame);

 private:
  GalbotMotion() = default;
  GalbotMotion(const GalbotMotion&) = delete;
  GalbotMotion& operator=(const GalbotMotion&) = delete;
  GalbotMotion(GalbotMotion&&) = delete;
  GalbotMotion& operator=(GalbotMotion&&) = delete;

  int robot_dof_ = 21;
};

}  // namespace g1
}  // namespace sdk
}  // namespace galbot