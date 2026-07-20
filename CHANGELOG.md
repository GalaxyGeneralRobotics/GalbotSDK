# Changelog / 更新日志

All notable changes to Galbot SDK will be documented in this file.  
所有重要的 Galbot SDK 更改都将记录在此文件中。

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.9.1] - 2026-06-26

### Added / 新增
- Added S1 C++ and Python examples for Jacobian computation APIs (`get_jacobian`, `get_jacobian_by_state`). / 新增 S1 雅可比矩阵计算接口（`get_jacobian`、`get_jacobian_by_state`）的 C++ 和 Python 示例
- Added perception functionality for S1. / 新增 S1 感知功能
- Added get Infrared image data of an arm camera. / 添加获取手臂摄像头的红外图像数据
- Added support for Linker Hand L20 Dexterous Hand. / 新增 Linker Hand L20 灵巧手

### Changed / 更改
- Improved documentation descriptions. / 完善部分文档说明
- PC deployment add one-click configuration of embosa IP. / PC端部署增加一键配置Embosa IP

### Fixed / 修复
- Fixed issues caused by an incorrect torso zero position. / 修复 torso 零位不正确引起的相关问题
- Fixed the issue where the `set_base_pose` interface could not move. / 修复 `set_base_pose` 接口不能运动的问题

## [1.9.0] - 2026-06-10

### Added / 新增
- Added navigation bounding-box filtering APIs (`add_bounding_box`, `remove_bounding_box`, `get_bounding_box`) so the fusion service can dynamically remove points belonging to carried boxes. / 新增导航箱子过滤接口（`add_bounding_box`、`remove_bounding_box`、`get_bounding_box`），支持 fusion 服务动态抠除搬运箱子对应的点云
- Added attached-box collision APIs (`attach_box_to_link`, `detach_box_from_link`) so navigation can account for collisions between carried boxes and the environment. / 新增箱子挂载与解除挂载接口（`attach_box_to_link`、`detach_box_from_link`），支持导航服务识别搬运箱子与环境之间的碰撞关系
- Added chassis navigation APIs (`navigate_along_trajectory`, `navigate_through_waypoints`, `set_navigation_target`, `navigate_with_velocity`) and task status query APIs (`get_navigation_target_status`, `get_navigation_status`), enabling smoothed trajectory navigation, sequential waypoint navigation, real-time target updates, velocity-based navigation commands, and task progress tracking by task ID. / 新增底盘导航接口（`navigate_along_trajectory`、`navigate_through_waypoints`、`set_navigation_target`、`navigate_with_velocity`）及任务状态查询接口（`get_navigation_target_status`、`get_navigation_status`），支持平滑轨迹导航、多路点顺序导航、动态目标点实时更新、速度控制导航，以及通过 task ID 查询任务进度
- Added navigation dynamic configuration APIs (`set_navigation_velocity_limit`, `set_navigation_kinematics_limits`, `set_navigation_timeout`, `set_navigation_arrival_threshold`, `dump_navigation_configs`) for updating velocity, acceleration, jerk, timeout, and arrival-threshold settings through PNS. / 新增导航动态配置接口（`set_navigation_velocity_limit`、`set_navigation_kinematics_limits`、`set_navigation_timeout`、`set_navigation_arrival_threshold`、`dump_navigation_configs`），支持通过 PNS 更新速度、加速度、加加速度、超时与到位阈值配置
- Add get synchronized observation aligned by camera timestamp APIs (`get_synced_observation`) / 获取按相机时间戳对齐的同步观测数据接口 (`get_synced_observation`)
- G1 adds chassis_imu and lidar_imu / G1增加chassis_imu, lidar_imu

### Changed / 更改
- S1 JointGroup retrieves the corresponding joint group information based on the robot hardware version, improving compatibility across different S1 hardware revisions. / S1 JointGroup根据机器人硬件版本获取对应的关节组信息，提升不同 S1 硬件版本之间的兼容性

### Fixed / 修复
- Fixed issues related to control interfaces / 修复控制接口相关问题

## [1.8.1] - 2026-05-25

### Added / 新增
- Added Jacobian computation interfaces in the motion module (get_jacobian, get_jacobian_by_state), with corresponding C++/Python examples. / 在 motion 模块新增雅可比计算接口（get_jacobian、get_jacobian_by_state），并提供对应 C++/Python 示例
- Added support for VMware deployment environments. / 适配 VMware 虚拟机部署环境
- Added version and compatibility check CLI tools. / 新增版本检查和兼容性检查命令行工具

### Changed / 更改
- Optimized SDK installation, deployment, and FAQ documentation. / 优化 SDK 安装、部署和 FAQ 文档
- Reorganized installation documentation into separate topics for system requirements, PC setup, robot deployment, Docker deployment, terminology, and version compatibility. / 将安装文档拆分为系统要求、PC 配置、机器人部署、Docker 部署、术语说明和版本兼容性等独立主题

### Fixed / 修复
- Fixed a critical execution bug in set_joint_commands_batch and execute_joint_trajectory: when body joints and grippers were specified together, command data could be parsed incorrectly and cause unintended robot motion. / 修复 set_joint_commands_batch 和 execute_joint_trajectory 的严重执行问题：同时指定身体关节和夹爪时，命令数据可能解析错误并导致机器人意外运动

---

## [1.8.0] - 2026-05-08

### Added / 新增
- Added support for WSL2 and Docker environments. / 适配WSL2与docker环境使用
- Added support for dexterous hand control and status retrieval. / 支持灵巧手控制与状态获取功能
- Added a real-time end-effector position control interface. / 增加末端位置实时控制接口
- Added a check script to verify robot environment and SDK version compatibility. / 提供检查脚本，用于检测机器人环境与SDK版本是否匹配
- Added a post-installation SDK validation program. / 增加安装SDK后的验证程序

## [1.7.1] - 2026-04-29

### Added / 新增
- Added camera interface adaptation for heavy-duty S1 robots equipped with head depth cameras. / 针对搭载头部深度相机的重载 S1 机器人，完善相机接口适配与支持

## [1.7.0] - 2026-04-17

### Added / 新增
- Added support for the S1 robot. / 适配重载 S1 机器人，部分接口因硬件差异暂未在 S1 上支持

### Changed / 更改
- Python singleton initialization no longer requires calling `get_instance()`; e.g., `GalbotRobot.get_instance()` is now simply `GalbotRobot()`. / Python 单例初始化不再需要调用 `get_instance()`，例如 `GalbotRobot.get_instance()` 更改为 `GalbotRobot()`
- C++ singleton initialization via `get_instance()` now requires a `MachineType` argument (`MachineType::G1` or `MachineType::S1`). / C++ 单例初始化调用 `get_instance()` 时需传入 `MachineType::G1` 或 `MachineType::S1`
- C++ namespace changed from `galbot::sdk::g1` to `galbot::sdk`. / C++ 命名空间从 `galbot::sdk::g1` 更改为 `galbot::sdk`
- `type.hpp` renamed to `galbot_sdk_type.hpp`. / `type.hpp` 更名为 `galbot_sdk_type.hpp`
- `ControllerName` is now robot-specific: use `G1ControllerName` for G1 and `S1ControllerName` for S1. / `ControllerName` 根据机器人型号区分，G1 使用 `G1ControllerName`，S1 使用 `S1ControllerName`
- `JointGroup` is now robot-specific: use `G1JointGroup` for G1 and `S1JointGroup` for S1. / `JointGroup` 根据机器人型号区分，G1 使用 `G1JointGroup`，S1 使用 `S1JointGroup`

## [1.6.2] - 2026-03-26

### Added / 新增
- Added a new perception module with support for high-precision stereo depth estimation model. / 新增感知模块，增加高精双目深度估计模型调用接口

## [1.6.1] - 2026-03-18

**注意变更：set_joint_commands接口为适应高频控制需求，将不再对起始位置点进行插值，会在目标时间点快速到达指定位置。在使用时目标角度和当前角度差距建议不要过大，防止机器人运动过快发生事故**

**Note: To meet high-frequency control requirements, the set_joint_commands interface no longer performs interpolation from the starting position and will drive the joints to the specified target position as quickly as possible at the target time. When using this interface for the first time, it is recommended that the difference between the target angle and the current angle not be too large, to avoid excessively fast motion and potential accidents.**

### Changed / 更改
- Fix high-frequency call issue in set_joint_commands function, add time_from_start_s parameter as the expected arrival time.
  The current version does not support torque, velocity, or acceleration settings for head, leg, and arm joint_command. /
  set_joint_commands函数修复高频调用问题，增加time_from_start_s参数，作为期望到达时间。
  当前版本头、腿、手臂关节joint_command暂不支持力矩、速度、加速度设置。

- Update is_moving logic in get_gripper_state: if the gripper does not move within a specified time, is_moving will be set to false /
  get_gripper_state中is_moving字段判断逻辑改为，指定时间内夹爪未移动，is_moving变为false

- Add default value for frame_id parameter in set_base_pose function (default: odom frame), without affecting existing usage /
  set_base_pose函数frame_id参数增加默认值为odom坐标系，不影响原有调用方式

---

## [1.6.0] - 2026-03-06

### Added / 新增
- Added volume and audio management interfaces, power and device information retrieval interfaces / 增加音量、音频管理接口，电源、设备信息获取接口
- Provided user log printing interface and SDK log information retrieval interface / 提供用户日志打印接口与SDK日志信息获取接口
- Add real-time joint trajectory function, control mode management function, one-click homing, robot link query interface, and sensor calibration parameter interface. / 增加关节实时轨迹函数、控制模式管理函数、一键回零、机器人link获取、传感器标定参数接口
- Added scenario-level sample programs / 增加情景级示例程序

### Changed / 更改
- Add a duration parameter to the chassis velocity control interface (without affecting existing usage) / 底盘速度控制接口增加持续时间参数（不影响原有调用方式）

---

## [1.5.1] - 2026-01-16

### Added / 新增
- Added interface to retrieve joint group names / 增加获取关节组名称接口

### Changed / 更改
- Optimized SDK documentation overall / 优化 SDK 文档整体内容

---

## [1.5.0] - 2026-01-10

### Added / 新增
- Improved environment compatibility: supports Python 3.8-3.14 and Ubuntu 20-24 / 提升环境兼容性：支持 Python 3.8-3.14 和 Ubuntu 20-24
- Added contextual examples / 增加情境性example
- Added access to six-dimensional force, odometry, and ultrasonic sensor data / 增加六维力、里程计和超声波传感器数据的获取接口
- Added tools and obstacle management functionality to planning / 增加规划工具和障碍物管理功能

### Changed / 更改
- Optimized SDK package structure / 优化 SDK 包结构

---

## [1.4.2] - 2025-12-25

### Changed / 更改
- Optimized overall SDK installation process / 优化 SDK 安装流程
- Added supplementary C++ example documentation / 补充C++ 示例文档

### Fixed / 修复
- Fixed issues related to control planning / 修复控制规划相关问题

---

## [1.4.1] - 2025-12-19

### Added / 新增
- Completed Python interfaces for planning / 完善规划的 Python 接口

### Changed / 更改
- Updated API documentation / 更新 API 文档
- Improved documentation generation process / 改进文档生成流程

---

## [1.4.0] - 2025-12-16

### Added / 新增
- Introduced MkDocs documentation system / 引入 MkDocs 文档系统
- Added motion planning-related interfaces / 增加运动规划相关接口
- Added control and sensor example programs / 增加控制与传感器示例程序

### Changed / 更改
- Refactored motion planning backend / 重构运动规划底层
- Standardized geometry type naming / 规范几何类型命名

---

## [1.3.2] - 2025-12-14

### Fixed / 修复
- Fixed issues related to control interfaces / 修复控制接口相关问题

---

## [1.3.1] - 2025-12-12

### Added / 新增
- Provided Python interfaces for planning/navigation / 提供规划/导航的 Python 接口
- Added API documentation generation / 增加 API 文档生成

---

## [1.3.0] - 2025-12-09

### Added / 新增
- Provided Python interfaces for control and sensors / 提供控制与传感器的 Python 接口

### Changed / 更改
- Standardized timeout parameter types / 规范超时参数类型

### Fixed / 修复
- Fixed issues related to planning and control / 修复规划与控制相关问题
- Fixed pybind compilation issues on x86 platforms / 修复 x86 平台 pybind 编译问题

---

## [1.2.0] - 2025-12-05

### Changed / 更改
- Updated control interface forms to improve usability / 更新控制接口形式以提升易用性

---

## [1.1.0] - 2025-12-01

### Added / 新增
- Added basic planning capabilities / 增加基础规划能力
- Added basic navigation capabilities / 增加基础导航能力

---

## [1.0.0] - 2025-11-21

### Added / 新增
- Added basic control capabilities / 增加基础控制能力
- Added basic sensor data reception / 增加基础传感器数据接收
