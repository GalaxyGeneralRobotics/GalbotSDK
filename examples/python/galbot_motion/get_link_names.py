import time
import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotMotion, GalbotRobot

# 获取 GalbotMotion 单例并初始化
motion = GalbotMotion.get_instance()
robot = GalbotRobot.get_instance()

if motion.init():
    print("GalbotMotion 初始化成功")
else:
    print("GalbotMotion 初始化失败")
if robot.init():
    print("GalbotRobot 初始化成功")
else:
    print("GalbotRobot 初始化失败")

# 程序立即启动，稍等数据就绪时间
time.sleep(2)

try:
    # 获取所有连杆名称
    all_link_names = motion.get_link_names(only_end_effector=False)
    print(f"\n所有连杆名称 (共 {len(all_link_names)} 个):")
    for i, link_name in enumerate(all_link_names, 1):
        print(f"  {i}. {link_name}")

    # 只获取末端执行器连杆名称
    ee_link_names = motion.get_link_names(only_end_effector=True)
    print(f"\n末端执行器连杆名称 (共 {len(ee_link_names)} 个):")
    for i, link_name in enumerate(ee_link_names, 1):
        print(f"  {i}. {link_name}")

    # 示例：使用连杆名称进行前向运动学计算
    if ee_link_names:
        print(f"\n使用末端执行器连杆 '{ee_link_names[0]}' 进行前向运动学计算...")
        success, fk_result = motion.forward_kinematics(ee_link_names[0])
        if success == gm.MotionStatus.SUCCESS:
            print(f"前向运动学结果: {fk_result}")
        else:
            print(f"前向运动学计算失败: {success}")
except Exception as e:
    print(f"❌ 获取连杆名称异常: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()