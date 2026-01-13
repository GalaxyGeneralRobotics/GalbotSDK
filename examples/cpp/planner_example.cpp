#include <array>
#include <chrono>
#include <iomanip>
#include <ios>
#include <iostream>
#include <memory>
#include <ostream>
#include <sstream>
#include <thread>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"
#include "motion_plan_config.hpp"
#include "type.hpp"

// 添加的命令帮助表结构
struct CommandInfo {
  std::string cmd;
  std::string description;
  std::string parameters;
  std::string example;
};
using namespace galbot::sdk::g1;
// 命令帮助表
const std::vector<CommandInfo> COMMAND_TABLE = {
    {"i", "终端打印当前机器人状态及sdk支持字符串", "无参数", "i"},
    {"cfg", "embosa日志打印mps配置, 需从日志查看", "无参数", "cfg"},
    {"fk", "前向运动学", "link_name", "fk left_arm_end_effector_mount_link"},
    {"ik", "逆向运动学", "链名数量 链名列表 7维位姿", "ik 2 left_arm right_arm 1.0 0.0 0.5 0.0 0.0 0.0 1.0"},
    {"mp", "运动规划",
     "0/1(笛卡尔/关节) true/false(立即执行) 链名 目标位姿/关节 "
     "true/false(碰撞检测)",
     "mp 0 true left_arm 1.0 0.0 0.5 0.0 0.0 0.0 1.0 true"},
    {"mpmw", "多点运动规划",
     "0/1(笛卡尔/关节) true/false(立即执行) 链名 目标位姿/关节 路径点数量 "
     "路径点位姿",
     "mpmw 0 true left_arm 1.0 0.0 0.5 0.0 0.0 0.0 1.0 2 ..."},
    {"mpmcmw", "多链多点运动规划",
     "0/1(笛卡尔/关节) true/false(立即执行) 链名数量 链名列表 目标位姿/关节 路径点数量 "
     "路径点位姿",
     "mpmcmw 0 true 2 left_arm right_arm 1.0 0.0 0.5 0.0 0.0 0.0 1.0 2 ..."},
    {"gp", "获取末端位姿", "末端坐标系", "gp left_arm_end_effector"},
    {"sp", "设置末端位姿", "末端坐标系 7维位姿", "sp left_arm_end_effector 1.0 0.0 0.5 0.0 0.0 0.0 1.0"},
    {"cc", "碰撞检测", "无参数", "cc"},
    {"get_tools/gt", "得到支持的工具列表", "工具列表", "gt"},

    {"add_box/ab", "示例: 添加BOX碰撞体到环境", "碰撞体名称(唯一字符串)", "ab"},
    {"add_sphere/as", "示例: 添加SPHERE碰撞体到环境", "碰撞体名称(唯一字符串)", "as"},
    {"get_built_obj/gbo", "示例: 获取当前环境中的碰撞体列表", "无参数", "gbo"},
    {"rm_obj/ro", "示例: 移除添加至环境的碰撞体", "碰撞体名称", "ro"},
    {"clear_obj/co", "示例: 清除所有添加至环境的碰撞体", "无参数", "co"},
    {"abtc", "示例: 添加BOX碰撞体到机器人链末端", "碰撞体名称(唯一字符串)", "abtc"},
    {"abtb", "示例: 添加BOX碰撞体到机器人base_link", "碰撞体名称(唯一字符串)", "abtb"},
    {"do", "示例: 移除添加至机器人的碰撞体", "碰撞体名称", "do"},
    {"sikl", "示例: 设置指定链IK软限位及还原IK软限位", "无参数", "sikl"},
    {"at", "示例: 添加工具到指定链", "链名 工具名", "at left_arm tool_name"},
    {"dt", "示例: 移除指定链的工具", "链名", "dt left_arm"},
    {"help", "显示帮助", "无参数", "help"},
    {"exit/q", "退出程序", "无参数", "exit"},
    {"gln", "获取连杆名称列表", "是否仅获取末端执行器连杆 (1/0)", "gln"}};

// 辅助函数声明
void printCommandHelp();
bool getPoseInput(std::vector<double>& pose, int expected_size);
void printMotionPlanResult(const std::pair<galbot::sdk::g1::MotionStatus,
                                           std::unordered_map<std::string, std::vector<std::vector<double>>>>& result);
void getChainNames(std::vector<std::string>& chain_names);

void printInfo() {
  auto& sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  auto robot_state = sdk.getRobotStates();
  auto whole_body_joint = sdk.getWholeBodyState();
  auto base_state = sdk.getChassisState();
  auto chain_state = sdk.getChainJointState();
  auto support_chains = sdk.getSupportChains();
  auto support_links = sdk.getSupportLinks();
  auto support_tools = sdk.getSupportToolList();
  auto support_frame = sdk.getSupportFrame();
  auto support_ee_frame = sdk.getSupportEEFrame();
  auto support_obstacle = sdk.getSupportObstacleType();
  std::cout << "\n========================================" << std::endl;
  std::cout << "robot state:\nbody: ";
  for (auto& joint : whole_body_joint) {
    std::cout << std::setprecision(4) << joint << ", ";
  }
  std::cout << "\nbase: ";
  for (auto& p : base_state) {
    std::cout << std::setprecision(4) << p << ", ";
  }
  std::cout << "\n--------- ";
  for (auto& chain : chain_state) {
    std::cout << "\n" << chain.first << ": ";
    for (auto& j : chain.second) {
      std::cout << std::setprecision(4) << j << ", ";
    }
  }
  std::cout << "\n--------- \nsupport chains: ";
  for (auto& val : support_chains) {
    std::cout << val << ", ";
  }
  std::cout << "\n--------- \nsupport links: ";
  for (auto& val : support_links) {
    std::cout << val << ", ";
  }
  std::cout << "\n--------- \nsupport tools: ";
  for (auto& val : support_tools) {
    std::cout << val << ", ";
  }
  std::cout << "\n--------- \nsupport frame: ";
  for (auto& val : support_frame) {
    std::cout << val << ", ";
  }
  std::cout << "\n--------- \nsupport ee frame: ";
  for (auto& val : support_ee_frame) {
    std::cout << val << ", ";
  }
  std::cout << "\n--------- \nsupport obstacle type: ";
  for (auto& val : support_obstacle) {
    std::cout << val << ", ";
  }
}
void testAddBox(const std::string& name) {
  auto& sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  std::array<double, 3> scale{2.0, 2.0, 4.0};
  std::vector<double> transform{0, 0, 0, 0, 0, 0, 1};
  auto ret = sdk.add_obstacle(name, "box", transform, scale, "", "world");
  std::cout << "ret = " << sdk.status_to_string(ret) << std::endl;
}
void testAddSphere(const std::string& name) {
  auto& sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  std::array<double, 3> scale{1.0, 0, 0};
  std::vector<double> transform{1.0, 0, 1, 0, 0, 0, 1};
  auto ret = sdk.add_obstacle(name, "sphere", transform, scale, "", "world");
  std::cout << "ret = " << sdk.status_to_string(ret) << std::endl;
}
void testGetObjects() {
  auto &sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  auto obj_list = sdk.getBuiltObjectList();
  std::cout << "obstacle list: ";
  for (auto &name : obj_list) {
    std::cout << name << ", ";
  }
  std::cout << std::endl;
}

void testRemoveObject(const std::string &name) {
  auto &sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  if (name == "") {
    auto ret = sdk.clear_obstacle();
    std::cout << "ret = " << sdk.status_to_string(ret) << std::endl;
    return;
  }
  auto ret = sdk.remove_obstacle(name);
  std::cout << "ret = " << sdk.status_to_string(ret) << std::endl;
}
void testAddTool(const std::string& chain, const std::string& tool) {
  auto& sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  auto ret = sdk.attach_tool(chain, tool);
  std::cout << "ret = " << sdk.status_to_string(ret) << std::endl;
}
void testDetachTool(const std::string& chain) {
  auto& sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  auto ret = sdk.detach_tool(chain);
  std::cout << "ret = " << sdk.status_to_string(ret) << std::endl;
}
void testAttachBoxToChain(const std::string& name) {
  auto& sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  std::array<double, 3> scale{1.0, 1.0, 2.0};
  std::vector<double> transform{1.0, 0, 1, 0, 0, 0, 1};
  auto ret = sdk.attach_target_object(name, "box", transform, scale, "", "left_arm", "ee_base");
  std::cout << "ret = " << sdk.status_to_string(ret) << std::endl;
}
void testAttachBoxToBaseLink(const std::string& name) {
  auto& sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  std::array<double, 3> scale{2.0, 2.0, 4.0};
  std::vector<double> transform{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  auto ret = sdk.attach_target_object(name, "box", transform, scale, "", "base_link");
  std::cout << "ret = " << sdk.status_to_string(ret) << std::endl;
}
void testdetachObject(const std::string& name) {
  auto& sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  auto ret = sdk.detach_target_object(name);
  std::cout << "ret = " << sdk.status_to_string(ret) << std::endl;
}
void testSetSoftIKLimit() {
  auto& sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  auto config_ret = sdk.get_motion_plan_config();
  std::cout << "获取配置返回状态: " << sdk.status_to_string(std::get<0>(config_ret)) << std::endl;
  if (std::get<0>(config_ret) != galbot::sdk::g1::MotionStatus::SUCCESS) {
    return;
  }
  auto config = std::get<1>(config_ret);
  config.print();
  auto ik_limit = config.get_ik_joint_limit();
  std::string chain_name = "left_arm";
  std::vector<double> upper_limit, lower_limit;
  // 测试数据
  std::vector<double> test_upper_limit{1, 1, 1, 1, 1, 1, 1};
  std::vector<double> test_lower_limit{0, 0, 0, 0, 0, 0, 0};
  for (auto& limit : ik_limit) {
    if (limit.get_chain_name() == chain_name) {
      upper_limit = limit.get_upper_limit();
      lower_limit = limit.get_lower_limit();
    }
  }
  std::cout << "chain: " << chain_name << "\nupper: ";
  for (auto& j : upper_limit) {
    std::cout << j << ", ";
  }
  std::cout << "\nlower: ";
  for (auto& j : lower_limit) {
    std::cout << j << ", ";
  }
  // 测试: 设置对应chain IK软限位
  KinematicsBoundary test_bound;
  test_bound.set_chain_name(chain_name);
  test_bound.set_upper_limit(test_upper_limit);
  test_bound.set_lower_limit(test_lower_limit);
  std::shared_ptr<MotionPlanConfig> motion_config = std::make_shared<MotionPlanConfig>();
  motion_config->set_ik_joint_limit({test_bound});
  auto ret = sdk.set_motion_plan_config(motion_config);
  std::cout << "设置配置返回状态: " << sdk.status_to_string(ret) << std::endl;
  if (ret != galbot::sdk::g1::MotionStatus::SUCCESS) {
    return;
  }
  // 辅助: 设置后打印对应chain软限位检查是否正确
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  config_ret = sdk.get_motion_plan_config();
  std::cout << "获取配置返回状态: " << sdk.status_to_string(std::get<0>(config_ret)) << std::endl;
  if (std::get<0>(config_ret) != galbot::sdk::g1::MotionStatus::SUCCESS) {
    return;
  }
  config = std::get<1>(config_ret);
  config.print();
  ik_limit = config.get_ik_joint_limit();
  for (auto& limit : ik_limit) {
    if (limit.get_chain_name() == chain_name) {
      test_upper_limit = limit.get_upper_limit();
      test_lower_limit = limit.get_lower_limit();
    }
  }
  std::cout << "after set config: " << std::endl;
  std::cout << "chain: " << chain_name << "\nupper: ";
  for (auto& j : test_upper_limit) {
    std::cout << j << ", ";
  }
  std::cout << "\nlower: ";
  for (auto& j : test_lower_limit) {
    std::cout << j << ", ";
  }
  std::cout << std::endl;

  // 测试: 还原IK软限位
  motion_config = std::make_shared<MotionPlanConfig>();
  motion_config->set_revert_ik_joint_limit(true);
  ret = sdk.set_motion_plan_config(motion_config);
  std::cout << "设置配置返回状态: " << sdk.status_to_string(ret) << std::endl;
  if (ret != galbot::sdk::g1::MotionStatus::SUCCESS) {
    return;
  }
}

void getSupportToolList() {
  auto &sdk = galbot::sdk::g1::GalbotMotion::get_instance();
    auto ret = sdk.getSupportToolList();
    std::stringstream ss;
    for (auto tool_name : ret)
    {
        ss << "{" << tool_name << "}, ";
    }
    std::cout << ss.str().c_str() << std::endl;
}

int main(int argc, char *argv[]) {
  // 获取wbc_interface实例
  auto& planner_instance = galbot::sdk::g1::GalbotMotion::get_instance();
  // 获取galbot_robot实例
  auto& base_instance = galbot::sdk::g1::GalbotRobot::get_instance();
  // 获取galbot_navigation实例
  auto& navi_instance = galbot::sdk::g1::GalbotNavigation::get_instance();

  // 初始化wbc全身控制接口
  if (!planner_instance.init()) {
    std::cout << "wbc_interface init fail" << std::endl;
    return -1;
  }

  // 初始化系统接口相关资源
  if (!base_instance.init()) {
    std::cout << "base_instance init fail" << std::endl;
    return -1;
  }

  // 初始化导航服务接口，获取定位信息
  if (!navi_instance.init()) {
    std::cout << "base_instance init fail" << std::endl;
    return -1;
  }

  std::cout << "\n========================================" << std::endl;
  std::cout << "     Galbot 机器人规划程序已启动" << std::endl;
  std::cout << "========================================\n" << std::endl;
  printCommandHelp();

  std::string cmd;
  bool running = true;

  std::map<std::string, std::vector<double>> chain_state_map{
      {"leg", {0.4992, 1.499, 1, 0, -0.0004314}},
      {"head", {0, 0.009898}},
      {"left_arm", {-2, 1.6, 0.6001, 1.7, 0, 0.8, 0}},
      {"right_arm", {2, -1.6, -0.5999, -1.7, 0, -0.7999, 0}},
  };
  std::vector<double> whole_body_joint(planner_instance.getRobotDof());
  for (auto& chain : chain_state_map) {
    planner_instance.replace_joint_state(chain.first, chain.second, whole_body_joint);
  }
  while (running && base_instance.is_running()) {
    std::cout << "\n>>> 请输入命令 (输入 'help' 查看帮助): ";
    std::cin >> cmd;

    if (cmd == "help") {
      printCommandHelp();
      continue;
    }
    if (cmd == "i") {
      printInfo();
      continue;
    }
    if (cmd == "config" || cmd == "cfg") {
      std::cout << "get config: " << std::endl;
      auto config_ret = planner_instance.get_motion_plan_config();
      if (std::get<0>(config_ret) != galbot::sdk::g1::MotionStatus::SUCCESS) {
        std::cout << "get config failed." << std::endl;
      }
      std::get<1>(config_ret).print();
      continue;
    }
    if (cmd == "add_box" || cmd == "ab") {
      std::string obj_name;
      std::cout << "请输入id字符串: ";
      std::cin >> obj_name;
      testAddBox(obj_name);
      continue;
    }
    if (cmd == "add_sphere" || cmd == "as") {
      std::string obj_name;
      std::cout << "请输入id字符串: ";
      std::cin >> obj_name;
      testAddSphere(obj_name);
      continue;
    }
    if (cmd == "rm_obj" || cmd == "ro") {
      std::string obj_name;
      std::cout << "请输入id字符串: ";
      std::cin >> obj_name;
      testRemoveObject(obj_name);
      continue;
    }
    if (cmd == "clear_obj" || cmd == "co") {
      testRemoveObject("");
      continue;
    }
    if (cmd == "attach_box_to_chain" || cmd == "abtc") {
      std::string obj_name;
      std::cout << "请输入id字符串: ";
      std::cin >> obj_name;
      testAttachBoxToChain(obj_name);
      continue;
    }
    if (cmd == "attach_box_to_baselink" || cmd == "abtb") {
      std::string obj_name;
      std::cout << "请输入id字符串: ";
      std::cin >> obj_name;
      testAttachBoxToBaseLink(obj_name);
      continue;
    }
    if (cmd == "detach_obj" || cmd == "do") {
      std::string obj_name;
      std::cout << "请输入id字符串: ";
      std::cin >> obj_name;
      testdetachObject(obj_name);
      continue;
    }
    if (cmd == "set_ik_limit" || cmd == "sikl") {
      testSetSoftIKLimit();
      continue;
    }
    if (cmd == "attach_tool" || cmd == "at") {
      std::string chain_name;
      std::string tool_name;
      std::cout << "请输入链名: ";
      std::cin >> chain_name;
      std::cout << "请输入工具名: ";
      std::cin >> tool_name;
      testAddTool(chain_name, tool_name);
      continue;
    }
    if (cmd == "detach_tool" || cmd == "dt") {
      std::string chain_name;
      std::cout << "请输入链名: ";
      std::cin >> chain_name;
      testDetachTool(chain_name);
      continue;
    }

    if (cmd == "fk") {
      std::string link_name;
      std::cout << "请输入 link_name (例如: left_arm_end_effector_mount_link "
                   "或 right_arm_end_effector_mount_link): ";
      std::cin >> link_name;

      auto res = planner_instance.forward_kinematics(link_name);
      std::cout << "前向运动学结果: " << planner_instance.status_to_string(std::get<0>(res)) << "\n";
      if (std::get<0>(res) == MotionStatus::SUCCESS) {
        std::cout << "正解结果: ";
        for (double value : std::get<1>(res)) {
          std::cout << value << " ";
        }
        std::cout << std::endl;
      }
    } else if (cmd == "ik") {
      std::vector<std::string> chain_names;
      getChainNames(chain_names);

      std::vector<double> target_pose;
      if (!getPoseInput(target_pose, 7)) {
        continue;
      }

      auto ik_result = planner_instance.inverse_kinematics(target_pose, chain_names);
      std::cout << "逆向运动学结果: " << planner_instance.status_to_string(std::get<0>(ik_result)) << "\n";
      if (std::get<1>(ik_result).empty()) {
        std::cout << "逆向运动学结果为空!" << std::endl;
      } else {
        std::cout << "\n逆向运动学结果:" << std::endl;
        for (const auto& pair_ : std::get<1>(ik_result)) {
          const auto& chain = pair_.first;
          const auto& angles = pair_.second;
          std::cout << "  链条 '" << chain << "': ";
          for (size_t i = 0; i < angles.size(); ++i) {
            std::cout << angles[i] << (i != angles.size() - 1 ? ", " : "");
          }
          std::cout << std::endl;
        }
      }

    } else if (cmd == "mp") {
      std::cout << "请选择规划类型:\n  0 - 笛卡尔空间\n  1 - 关节空间\n选择: ";
      int choice;
      std::cin >> choice;

      std::cout << "是否立即执行 (true/false): ";
      std::string is_direct_execute_str;
      std::cin >> is_direct_execute_str;
      bool is_direct_execute = (is_direct_execute_str == "true");

      auto params = std::make_shared<galbot::sdk::g1::Parameter>();
      params->is_direct_execute = is_direct_execute;

      std::tuple<galbot::sdk::g1::MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>>
          result;

      if (choice == 0) {  // 笛卡尔空间
        auto target_pose_ptr = std::make_shared<galbot::sdk::g1::PoseState>();
        std::cout << "请输入链名: ";
        std::cin >> target_pose_ptr->chain_name;

        std::cout << "请输入目标位姿 (x y z qx qy qz qw): ";
        std::vector<double> pose_input;
        if (!getPoseInput(pose_input, 7)) {
          continue;
        }
        target_pose_ptr->pose = pose_input;

        std::cout << "是否启用环境碰撞检测 (true/false): ";
        std::string enable_collision_check;
        std::cin >> enable_collision_check;

        std::cout << (enable_collision_check == "true" ? "执行路径规划..." : "执行轨迹规划...") << std::endl;
        result =
            planner_instance.motion_plan(target_pose_ptr, nullptr, nullptr, (enable_collision_check == "true"), params);
      } else {  // 关节空间
        auto target_joint_ptr = std::make_shared<galbot::sdk::g1::JointStates>();
        std::cout << "请输入链名: ";
        std::cin >> target_joint_ptr->chain_name;
        std::vector<double> joint_input;
        std::cout << "请输入目标关节角度: ";
        if (!getPoseInput(joint_input, 7)) {
          continue;
        }
        target_joint_ptr->joint_positions = joint_input;

        std::cout << "是否启用环境碰撞检测 (true/false): ";
        std::string enable_collision_check;
        std::cin >> enable_collision_check;

        std::cout << (enable_collision_check == "true" ? "执行路径规划..." : "执行轨迹规划...") << std::endl;
        result = planner_instance.motion_plan(target_joint_ptr, nullptr, nullptr, (enable_collision_check == "true"),
                                              params);
      }

      printMotionPlanResult({std::get<0>(result), std::get<1>(result)});

    } else if (cmd == "mpmw") {
      std::cout << "请选择规划类型:\n  0 - 笛卡尔空间\n  1 - 关节空间\n选择: ";
      int choice;
      std::cin >> choice;

      std::cout << "是否立即执行 (true/false): ";
      std::string is_direct_execute_str;
      std::cin >> is_direct_execute_str;
      bool is_direct_execute = (is_direct_execute_str == "true");

      auto params = std::make_shared<galbot::sdk::g1::Parameter>();
      params->is_direct_execute = is_direct_execute;

      std::tuple<galbot::sdk::g1::MotionStatus, std::unordered_map<std::string, std::vector<std::vector<double>>>>
          result;

      if (choice == 0) {  // 笛卡尔空间
        auto target_pose_ptr = std::make_shared<galbot::sdk::g1::PoseState>();
        std::cout << "请输入链名: ";
        std::cin >> target_pose_ptr->chain_name;

        // std::cout << "请输入目标位姿 (x y z qx qy qz qw): ";
        // std::vector<double> pose_input;
        // if (!getPoseInput(pose_input, 7)) {
        //     continue;
        // }
        // target_pose_ptr->pose = pose_input;

        int n;
        std::cout << "请输入路径点数量: ";
        std::cin >> n;

        std::vector<std::vector<double>> waypoint_poses(n);
        for (int i = 0; i < n; ++i) {
          std::cout << "路径点 " << i + 1 << " (x y z qx qy qz qw): ";
          if (!getPoseInput(waypoint_poses[i], 7)) {
            continue;
          }
        }

        std::cout << "是否启用环境碰撞检测 (true/false): ";
        std::string enable_collision_check;
        std::cin >> enable_collision_check;

        std::cout << "执行多点路径规划..." << std::endl;
        result = planner_instance.motion_plan_multi_waypoints(target_pose_ptr, waypoint_poses, nullptr, nullptr,
                                                              (enable_collision_check == "true"), params);

      } else {  // 关节空间
        auto target_joint_ptr = std::make_shared<galbot::sdk::g1::JointStates>();
        std::cout << "请输入链名: ";
        std::cin >> target_joint_ptr->chain_name;

        // std::cout << "请输入目标关节角度 (7个值): ";
        // if (!getPoseInput(target_joint_ptr->joint_positions, 7)) {
        //     continue;
        // }

        int n;
        std::cout << "请输入路径点数量: ";
        std::cin >> n;

        std::vector<std::vector<double>> waypoint_joints(n);
        for (int i = 0; i < n; ++i) {
          std::cout << "路径点 " << i + 1 << " (7个关节角度): ";
          if (!getPoseInput(waypoint_joints[i], 7)) {
            continue;
          }
        }

        std::cout << "是否启用环境碰撞检测 (true/false): ";
        std::string enable_collision_check;
        std::cin >> enable_collision_check;

        std::cout << "执行多点关节空间规划..." << std::endl;
        result = planner_instance.motion_plan_multi_waypoints(target_joint_ptr, waypoint_joints, nullptr, nullptr,
                                                              (enable_collision_check == "true"), params);
      }

      printMotionPlanResult({std::get<0>(result), std::get<1>(result)});

    } else if (cmd == "gp") {
      std::string end_effector;
      std::cout << "请输入左臂/右臂末端 (left_arm/right_arm): ";
      std::cin >> end_effector;

      auto result = planner_instance.get_end_effector_pose_on_chain(end_effector);
      std::cout << "获取末端位姿返回状态: " << planner_instance.status_to_string(std::get<0>(result)) << "\n";
      if (std::get<0>(result) == MotionStatus::SUCCESS) {
        std::cout << "\n末端位姿: ";
        for (auto v : std::get<1>(result)) {
          std::cout << v << " ";
        }
      }

    } else if (cmd == "sp") {
      std::string end_effector;
      std::cout << "请输入末端坐标系 "
                   "(left_arm/right_arm): ";
      std::cin >> end_effector;

      std::vector<double> target_pose;
      std::cout << "请输入目标位姿 (x y z qx qy qz qw): ";
      if (!getPoseInput(target_pose, 7)) {
        continue;
      }

      auto result = planner_instance.set_end_effector_pose(target_pose, end_effector);
      std::cout << "设置末端位姿返回状态: " << planner_instance.status_to_string(result) << "\n";

    } else if (cmd == "cc") {
      auto target_pose_ptr = std::make_shared<galbot::sdk::g1::RobotStates>();
      std::cout << "请输入当前机器人关节角度 (21个值): ";
      if (!getPoseInput(target_pose_ptr->whole_body_joint, 21)) {
        continue;
      }
      std::vector<std::shared_ptr<galbot::sdk::g1::RobotStates>> cc_list;
      cc_list.push_back(target_pose_ptr);
      auto result = planner_instance.check_collision(cc_list);
      std::cout << "碰撞检测返回状态: " << planner_instance.status_to_string(std::get<0>(result)) << "\n";
      if (std::get<0>(result) == MotionStatus::SUCCESS) {
        std::cout << "碰撞检测结果: ";
        for (auto v : std::get<1>(result)) {
          std::cout << v << " ";
        }
        std::cout << "\n检测状态: " << static_cast<int>(std::get<0>(result)) << std::endl;
      }
    } else if (cmd == "get_tools" || cmd == "gt") {
        std::cout << "当前机器人所支持的工具列表: " << "\n";
        getSupportToolList();
    } else if (cmd == "get_link_names" || cmd == "gln") {
      int only_end_effector = 0;
      auto link_names = planner_instance.getSupportLinks();
      std::cout << "获取连杆名称列表: " << std::endl;
      for (auto link_name : link_names) {
        std::cout << link_name << std::endl;
      }
    } else if (cmd == "exit" || cmd == "q") {
      std::cout << "\n程序正在退出..." << std::endl;
      running = false;
    } else {
      std::cout << "未知命令: " << cmd << "，请输入 'help' 查看可用命令" << std::endl;
    }
  }
  base_instance.request_shutdown();
  base_instance.wait_for_shutdown();
  base_instance.destroy();
  return 0;
}

// 打印命令帮助表
void printCommandHelp() {
  std::cout << "\n================ 命令帮助表 =================" << std::endl;
  std::cout << std::left << std::setw(8) << "命令" << std::setw(15) << "描述" << std::setw(30) << "参数"
            << std::setw(40) << "示例" << std::endl;
  std::cout << std::string(93, '-') << std::endl;

  for (const auto& cmd : COMMAND_TABLE) {
    std::cout << std::left << std::setw(8) << cmd.cmd << std::setw(15) << cmd.description << std::setw(30)
              << cmd.parameters << std::setw(40) << cmd.example << std::endl;
  }
  std::cout << "=============================================\n" << std::endl;
}

// 获取位姿输入
bool getPoseInput(std::vector<double>& pose, int expected_size) {
  pose.clear();
  std::cout << "请输入 " << expected_size << " 个数值 (用空格分隔): ";

  for (int i = 0; i < expected_size; ++i) {
    double value;
    if (!(std::cin >> value)) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "输入错误，请重新输入" << std::endl;
      return false;
    }
    pose.push_back(value);
  }
  // 清除可能的换行符
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return true;
}

// 打印运动规划结果
void printMotionPlanResult(const std::pair<galbot::sdk::g1::MotionStatus,
                                           std::unordered_map<std::string, std::vector<std::vector<double>>>>& result) {
  auto& sdk = galbot::sdk::g1::GalbotMotion::get_instance();
  std::cout << "\n运动规划结果:" << sdk.status_to_string(result.first) << std::endl;
  std::cout << "==========================================" << std::endl;

  for (const auto& pair_ : result.second) {
    const auto& key = pair_.first;
    const auto& vec_of_vec = pair_.second;
    std::cout << "关键点: " << key << std::endl;
    std::cout << "向量数量: " << vec_of_vec.size() << std::endl;

    for (size_t i = 0; i < vec_of_vec.size(); ++i) {
      std::cout << "  向量 " << i << " (大小: " << vec_of_vec[i].size() << "): [";

      for (size_t j = 0; j < vec_of_vec[i].size(); ++j) {
        std::cout << vec_of_vec[i][j];
        if (j < vec_of_vec[i].size() - 1) {
          std::cout << ", ";
        }
      }
      std::cout << "]" << std::endl;
    }
    std::cout << std::endl;
  }
}

// 获取链名列表
void getChainNames(std::vector<std::string>& chain_names) {
  chain_names.clear();
  int n;
  std::cout << "请输入链名数量: ";
  std::cin >> n;
  std::cin.ignore();

  std::cout << "请输入 " << n << " 个链名:" << std::endl;
  for (int i = 0; i < n; ++i) {
    std::string name;
    std::cout << "链名 " << i + 1 << ": ";
    std::getline(std::cin, name);
    chain_names.push_back(name);
  }
}