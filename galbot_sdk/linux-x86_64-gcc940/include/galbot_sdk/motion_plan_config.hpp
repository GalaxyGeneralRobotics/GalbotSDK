#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>
namespace galbot {
namespace sdk {
namespace g1 {

// KinematicsBoundary 结构体的接口
struct KinematicsBoundary {
  // 原始成员变量（私有化）
 private:
  std::string chain_name = "";           // 链名称
  std::vector<double> lower_limit;       // 关节下限位
  std::vector<double> upper_limit;       // 关节上限位
  std::vector<double> vel_lower_limit;   // 关节速度下限位
  std::vector<double> vel_upper_limit;   // 关节速度上限位
  std::vector<double> acc_lower_limit;   // 关节加速度下限位
  std::vector<double> acc_upper_limit;   // 关节加速度上限位
  std::vector<double> jerk_lower_limit;  // 关节加加速度下限位
  std::vector<double> jerk_upper_limit;  // 关节加加速度上限位

 public:
  // setter 方法
  void set_chain_name(const std::string& name) { chain_name = name; }
  void set_lower_limit(const std::vector<double>& limits) { lower_limit = limits; }
  void set_upper_limit(const std::vector<double>& limits) { upper_limit = limits; }
  void set_vel_lower_limit(const std::vector<double>& limits) { vel_lower_limit = limits; }
  void set_vel_upper_limit(const std::vector<double>& limits) { vel_upper_limit = limits; }
  void set_acc_lower_limit(const std::vector<double>& limits) { acc_lower_limit = limits; }
  void set_acc_upper_limit(const std::vector<double>& limits) { acc_upper_limit = limits; }
  void set_jerk_lower_limit(const std::vector<double>& limits) { jerk_lower_limit = limits; }
  void set_jerk_upper_limit(const std::vector<double>& limits) { jerk_upper_limit = limits; }

  // getter 方法
  const std::string& get_chain_name() const { return chain_name; }
  const std::vector<double>& get_lower_limit() const { return lower_limit; }
  const std::vector<double>& get_upper_limit() const { return upper_limit; }
  const std::vector<double>& get_vel_lower_limit() const { return vel_lower_limit; }
  const std::vector<double>& get_vel_upper_limit() const { return vel_upper_limit; }
  const std::vector<double>& get_acc_lower_limit() const { return acc_lower_limit; }
  const std::vector<double>& get_acc_upper_limit() const { return acc_upper_limit; }
  const std::vector<double>& get_jerk_lower_limit() const { return jerk_lower_limit; }
  const std::vector<double>& get_jerk_upper_limit() const { return jerk_upper_limit; }

  void print() const;
};

// SamplerConfig 结构体的接口
struct SamplerConfig {
  enum StateCheckType { EUCLIDEAN_DISTANCE, RADIAN_DISTANCE };
  enum TerminationConditionType { TIMEOUT = 0, TIMEOUT_AND_EXACT_SOLUTION = 1 };

 private:
  StateCheckType state_check_type_;
  double state_check_resolution_ = 0.01;
  bool interpolate_ = true;
  int interpolation_cnt_ = 0;
  bool simplify_ = true;
  double max_simplification_time_ = -1.0;
  TerminationConditionType termination_condition_type_ = TIMEOUT_AND_EXACT_SOLUTION;
  double max_planning_time_ = 2.0;

 public:
  // setter 方法
  void set_state_check_type(StateCheckType type) { state_check_type_ = type; }
  void set_state_check_resolution(double resolution) { state_check_resolution_ = resolution; }
  void set_interpolate(bool enable) { interpolate_ = enable; }
  void set_interpolation_cnt(int cnt) { interpolation_cnt_ = cnt; }
  void set_simplify(bool enable) { simplify_ = enable; }
  void set_max_simplification_time(double time) { max_simplification_time_ = time; }
  void set_termination_condition_type(TerminationConditionType type) { termination_condition_type_ = type; }
  void set_max_planning_time(double time) { max_planning_time_ = time; }

  // getter 方法
  StateCheckType get_state_check_type() const { return state_check_type_; }
  double get_state_check_resolution() const { return state_check_resolution_; }
  bool get_interpolate() const { return interpolate_; }
  int get_interpolation_cnt() const { return interpolation_cnt_; }
  bool get_simplify() const { return simplify_; }
  double get_max_simplification_time() const { return max_simplification_time_; }
  TerminationConditionType get_termination_condition_type() const { return termination_condition_type_; }
  double get_max_planning_time() const { return max_planning_time_; }

  void print() const;
};

// TrajectoryPlanConfig 结构体的接口
struct TrajectoryPlanConfig {
 private:
  double min_move_time_ = 0.0;
  double move_line_intermediate_point_ = 50.0;
  double way_point_plan_expected_time_ = 0.5;

 public:
  // setter 方法
  void set_min_move_time(double time) { min_move_time_ = time; }
  void set_move_line_intermediate_point(double value) { move_line_intermediate_point_ = value; }
  void set_way_point_plan_expected_time(double time) { way_point_plan_expected_time_ = time; }

  // getter 方法
  double get_min_move_time() const { return min_move_time_; }
  double get_move_line_intermediate_point() const { return move_line_intermediate_point_; }
  double get_way_point_plan_expected_time() const { return way_point_plan_expected_time_; }

  void print() const;
};

// IKSolverConfig 结构体的接口
struct IKSolverConfig {
  enum SeedType {
    RANDOM_SEED,
    RANDOM_PROGRESSIVE_SEED,
    USER_DEFINED_SEED,
  };

 private:
  double col_aware_ik_timeout_ = 10.0;  // ms
  SeedType seed_type_ = RANDOM_PROGRESSIVE_SEED;
  double col_aware_ik_joint_limit_bias_ = 0.001;
  std::array<double, 3> translation_eps_ = {0.0, 0.0, 0.0};
  std::array<double, 3> rotation_eps_ = {0.0, 0.0, 0.0};
  bool enable_collision_check_log_ = false;

 public:
  // setter 方法
  void set_col_aware_ik_timeout(double timeout) { col_aware_ik_timeout_ = timeout; }
  void set_seed_type(SeedType type) { seed_type_ = type; }
  void set_col_aware_ik_joint_limit_bias(double bias) { col_aware_ik_joint_limit_bias_ = bias; }
  void set_translation_eps(const std::array<double, 3>& eps) { translation_eps_ = eps; }
  void set_rotation_eps(const std::array<double, 3>& eps) { rotation_eps_ = eps; }
  void set_enable_collision_check_log(bool enable) { enable_collision_check_log_ = enable; }

  // getter 方法
  double get_col_aware_ik_timeout() const { return col_aware_ik_timeout_; }
  SeedType get_seed_type() const { return seed_type_; }
  double get_col_aware_ik_joint_limit_bias() const { return col_aware_ik_joint_limit_bias_; }
  const std::array<double, 3>& get_translation_eps() const { return translation_eps_; }
  const std::array<double, 3>& get_rotation_eps() const { return rotation_eps_; }
  bool get_enable_collision_check_log() const { return enable_collision_check_log_; }

  void print() const;
};

// CollisionCheckOption 结构体的接口
struct CollisionCheckOption {
 private:
  bool disable_self_collision_check_ = false;
  bool disable_env_collision_check_ = false;

 public:
  // setter 方法
  void set_disable_self_collision_check(bool disable) { disable_self_collision_check_ = disable; }
  void set_disable_env_collision_check(bool disable) { disable_env_collision_check_ = disable; }

  // getter 方法
  bool get_disable_self_collision_check() const { return disable_self_collision_check_; }
  bool get_disable_env_collision_check() const { return disable_env_collision_check_; }

  void print() const;
};

// TrajectoryFeasibilityCheckOption 结构体的接口
struct TrajectoryFeasibilityCheckOption {
 private:
  bool disable_collision_check_ = false;
  bool disable_joint_limit_check_ = false;
  bool disable_velocity_feasibility_check_ = false;

 public:
  // setter 方法
  void set_disable_collision_check(bool disable) { disable_collision_check_ = disable; }
  void set_disable_joint_limit_check(bool disable) { disable_joint_limit_check_ = disable; }
  void set_disable_velocity_feasibility_check(bool disable) { disable_velocity_feasibility_check_ = disable; }

  // getter 方法
  bool get_disable_collision_check() const { return disable_collision_check_; }
  bool get_disable_joint_limit_check() const { return disable_joint_limit_check_; }
  bool get_disable_velocity_feasibility_check() const { return disable_velocity_feasibility_check_; }

  void print() const;
};

// LineTrajCheckPrimitive 结构体的接口
struct LineTrajCheckPrimitive {
  enum PrimitiveType {
    LINE,
    CYLINDER,
  };

 private:
  PrimitiveType line_check_primitive_type_ = CYLINDER;
  double cylinder_prim_radius_ = 0.02;
  double line_prim_curvature_ = 0.02;

 public:
  // setter 方法
  void set_line_check_primitive_type(PrimitiveType type) { line_check_primitive_type_ = type; }
  void set_cylinder_prim_radius(double radius) { cylinder_prim_radius_ = radius; }
  void set_line_prim_curvature(double curvature) { line_prim_curvature_ = curvature; }

  // getter 方法
  PrimitiveType get_line_check_primitive_type() const { return line_check_primitive_type_; }
  double get_cylinder_prim_radius() const { return cylinder_prim_radius_; }
  double get_line_prim_curvature() const { return line_prim_curvature_; }

  void print() const;
};

// MotionPlanConfig 类的完整接口
class MotionPlanConfig {
 private:
  int64_t update_nsec_ = 0;
  std::shared_ptr<SamplerConfig> sampler_config_ = nullptr;
  std::shared_ptr<TrajectoryPlanConfig> traj_plan_config_ = nullptr;
  std::shared_ptr<IKSolverConfig> ik_solver_config_ = nullptr;
  std::shared_ptr<CollisionCheckOption> collision_check_option_ = nullptr;
  std::shared_ptr<TrajectoryFeasibilityCheckOption> traj_feasibility_check_option_ = nullptr;
  std::vector<KinematicsBoundary> feasibility_boundary_;
  std::shared_ptr<LineTrajCheckPrimitive> line_traj_check_primitive_ = nullptr;
  std::vector<KinematicsBoundary> ik_joint_limit_;
  std::vector<KinematicsBoundary> sampler_joint_limit_;
  std::vector<KinematicsBoundary> hard_joint_limit_;
  bool revert_ik_joint_limit_ = false;
  std::vector<std::string> revert_ik_joint_limit_chains_;

 public:
  MotionPlanConfig() = default;

  void set_update_time(int64_t t) { update_nsec_ = t; }
  int64_t get_update_time() { return update_nsec_; }

  // 创建并获取各个配置对象的方法
  std::shared_ptr<SamplerConfig> create_sampler_config() {
    if (!sampler_config_)
      sampler_config_ = std::make_shared<SamplerConfig>();
    return sampler_config_;
  }

  std::shared_ptr<TrajectoryPlanConfig> create_trajectory_plan_config() {
    if (!traj_plan_config_)
      traj_plan_config_ = std::make_shared<TrajectoryPlanConfig>();
    return traj_plan_config_;
  }

  std::shared_ptr<IKSolverConfig> create_ik_solver_config() {
    if (!ik_solver_config_)
      ik_solver_config_ = std::make_shared<IKSolverConfig>();
    return ik_solver_config_;
  }

  std::shared_ptr<CollisionCheckOption> create_collision_check_option() {
    if (!collision_check_option_)
      collision_check_option_ = std::make_shared<CollisionCheckOption>();
    return collision_check_option_;
  }

  std::shared_ptr<TrajectoryFeasibilityCheckOption> create_trajectory_feasibility_check_option() {
    if (!traj_feasibility_check_option_)
      traj_feasibility_check_option_ = std::make_shared<TrajectoryFeasibilityCheckOption>();
    return traj_feasibility_check_option_;
  }

  std::shared_ptr<LineTrajCheckPrimitive> create_line_traj_check_primitive() {
    if (!line_traj_check_primitive_)
      line_traj_check_primitive_ = std::make_shared<LineTrajCheckPrimitive>();
    return line_traj_check_primitive_;
  }

  // setter 方法
  void set_sampler_config(const std::shared_ptr<SamplerConfig>& config) { sampler_config_ = config; }
  void set_trajectory_plan_config(const std::shared_ptr<TrajectoryPlanConfig>& config) { traj_plan_config_ = config; }
  void set_ik_solver_config(const std::shared_ptr<IKSolverConfig>& config) { ik_solver_config_ = config; }
  void set_collision_check_option(const std::shared_ptr<CollisionCheckOption>& option) {
    collision_check_option_ = option;
  }
  void set_trajectory_feasibility_check_option(const std::shared_ptr<TrajectoryFeasibilityCheckOption>& option) {
    traj_feasibility_check_option_ = option;
  }
  void set_feasibility_boundary(const std::vector<KinematicsBoundary>& boundary) { feasibility_boundary_ = boundary; }
  void set_line_traj_check_primitive(const std::shared_ptr<LineTrajCheckPrimitive>& primitive) {
    line_traj_check_primitive_ = primitive;
  }
  void set_ik_joint_limit(const std::vector<KinematicsBoundary>& boundary) { ik_joint_limit_ = boundary; }
  void set_sampler_joint_limit(const std::vector<KinematicsBoundary>& boundary) { sampler_joint_limit_ = boundary; }
  void set_hard_joint_limit(const std::vector<KinematicsBoundary>& boundary) { hard_joint_limit_ = boundary; }
  void set_revert_ik_joint_limit(bool flag) { revert_ik_joint_limit_ = flag; }
  //! TODO: 预留
  void set_revert_ik_joint_limit_chains(const std::vector<std::string>& chains) {
    revert_ik_joint_limit_ = !chains.empty();
    revert_ik_joint_limit_chains_ = chains;
  }

  // getter 方法（返回智能指针）
  std::shared_ptr<SamplerConfig> get_sampler_config() const { return sampler_config_; }
  std::shared_ptr<TrajectoryPlanConfig> get_trajectory_plan_config() const { return traj_plan_config_; }
  std::shared_ptr<IKSolverConfig> get_ik_solver_config() const { return ik_solver_config_; }
  std::shared_ptr<CollisionCheckOption> get_collision_check_option() const { return collision_check_option_; }
  std::shared_ptr<TrajectoryFeasibilityCheckOption> get_trajectory_feasibility_check_option() const {
    return traj_feasibility_check_option_;
  }
  std::shared_ptr<LineTrajCheckPrimitive> get_line_traj_check_primitive() const { return line_traj_check_primitive_; }

  const std::vector<KinematicsBoundary>& get_feasibility_boundary() const { return feasibility_boundary_; }
  std::vector<KinematicsBoundary>& get_feasibility_boundary() { return feasibility_boundary_; }

  const std::vector<KinematicsBoundary>& get_ik_joint_limit() const { return ik_joint_limit_; }
  std::vector<KinematicsBoundary>& get_ik_joint_limit() { return ik_joint_limit_; }

  const std::vector<KinematicsBoundary>& get_sampler_joint_limit() const { return sampler_joint_limit_; }
  std::vector<KinematicsBoundary>& get_sampler_joint_limit() { return sampler_joint_limit_; }

  const std::vector<KinematicsBoundary>& get_hard_joint_limit() const { return hard_joint_limit_; }
  std::vector<KinematicsBoundary>& get_hard_joint_limit() { return hard_joint_limit_; }

  const std::vector<std::string>& get_revert_ik_joint_limit_chains() const { return revert_ik_joint_limit_chains_; }
  std::vector<std::string>& get_revert_ik_joint_limit_chains() { return revert_ik_joint_limit_chains_; }

  bool get_revert_ik_joint_limit() { return revert_ik_joint_limit_; }

  // getter 方法（返回引用，如果为空则创建新对象）
  SamplerConfig& get_sampler_config_ref() {
    if (!sampler_config_)
      sampler_config_ = std::make_shared<SamplerConfig>();
    return *sampler_config_;
  }

  TrajectoryPlanConfig& get_trajectory_plan_config_ref() {
    if (!traj_plan_config_)
      traj_plan_config_ = std::make_shared<TrajectoryPlanConfig>();
    return *traj_plan_config_;
  }

  IKSolverConfig& get_ik_solver_config_ref() {
    if (!ik_solver_config_)
      ik_solver_config_ = std::make_shared<IKSolverConfig>();
    return *ik_solver_config_;
  }

  CollisionCheckOption& get_collision_check_option_ref() {
    if (!collision_check_option_)
      collision_check_option_ = std::make_shared<CollisionCheckOption>();
    return *collision_check_option_;
  }

  TrajectoryFeasibilityCheckOption& get_trajectory_feasibility_check_option_ref() {
    if (!traj_feasibility_check_option_)
      traj_feasibility_check_option_ = std::make_shared<TrajectoryFeasibilityCheckOption>();
    return *traj_feasibility_check_option_;
  }

  LineTrajCheckPrimitive& get_line_traj_check_primitive_ref() {
    if (!line_traj_check_primitive_)
      line_traj_check_primitive_ = std::make_shared<LineTrajCheckPrimitive>();
    return *line_traj_check_primitive_;
  }

  void print() const;
};

}  // namespace g1
}  // namespace sdk
}  // namespace galbot