#pragma once

#include "type.hpp"

namespace galbot {
namespace sdk {
namespace g1 {

class GalbotNavigation {
 public:
  /**
   * @brief 获取NavigationImpl单例对象
   *
   * @return NavigationImpl& NavigationImpl单例对象
   */
  static GalbotNavigation& get_instance() {
    static GalbotNavigation instance;
    return instance;
  }

  /**
   * @brief 初始化NavigationImpl接口依赖资源
   *
   * @return true 初始化成功
   * @return false 初始化失败
   */
  bool init();

  // /**
  //  * @brief 保存地图,仅在部署模式有效
  //  *
  //  * @return NavigationStatus 执行结果
  //  */
  // NavigationStatus save_map();

  /**
   * @brief 重定位
   * @param init_pose 当前在map系下的位姿, 供重定位参考
   *
   * @return NavigationStatus 执行结果
   */
  NavigationStatus relocalize(const Pose& init_pose);

  /**
   * @brief 是否处于定位状态
   *
   * @return bool: true 有定位状态  false - 定位丢失状态
   */
  bool is_localized();

  /**
   * @brief 获取当前底盘位姿
   *
   * @return Pose 当前位姿
   */
  Pose get_current_pose();

  /**
   * @brief 导航至目标点位
   * @param goal_pose 目标点位
   * @param enable_collision_check 启用障碍物检测
   * @param is_blocking 是否阻塞,默认false,false时下发导航指令后函数即返回发布状态,
   * true时等待到达目标或失败后再返回执行状态
   * @param timeout 最大等待时间
   * @param omni_plan 是否支持全向运动
   *
   * @return NavigationStatus 执行结果
   */
  NavigationStatus navigate_to_goal(const Pose& goal_pose, bool enable_collision_check = true, bool is_blocking = false,
                                    float timeout = 8, bool omni_plan = true);
  /**
   * @brief 在odom系下导航到相对点位,不启用障碍物检测,支持全向运动
   * @param goal_pose 目标点位相对当前机器人底盘坐标
   * @param is_blocking 是否阻塞,默认true,false时下发导航指令后函数即返回发布状态,
   * true时等待到达目标或失败后再返回执行状态
   * @param timeout 最大等待时间
   *
   * @return NavigationStatus 执行结果
   */
  NavigationStatus move_straight_to(const Pose& goal_pose, bool is_blocking = true, float timeout = 8);

  /**
   * @brief 停止当前导航
   *
   * @return NavigationStatus 执行结果
   */
  NavigationStatus stop_navigation();

  /**
   * @brief 检查map坐标系下起点到终点是否有可达路径
   * @param goal_pose 目标点位
   * @param start_pose 起始点位
   *
   * @return 检测结果, true为存在路径, false为不存在
   */
  bool check_path_reachability(const Pose& goal_pose, const Pose& start_pose);

  /**
   * @brief 检查是否已到达目的地
   *
   * @return result 检查结果, true为已到达, false为未到达
   */

  bool check_goal_arrival();

 private:
  GalbotNavigation() = default;
  GalbotNavigation(const GalbotNavigation&) = delete;
  GalbotNavigation& operator=(const GalbotNavigation&) = delete;
  GalbotNavigation(GalbotNavigation&&) = delete;
  GalbotNavigation& operator=(GalbotNavigation&&) = delete;
};

}  // namespace g1
}  // namespace sdk
}  // namespace galbot