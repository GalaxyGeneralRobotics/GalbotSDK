import time
from galbot_sdk.g1 import GalbotRobot

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init()
# 程序立即启动，稍等数据就绪时间
time.sleep(1)
print("初始化成功")

# 使用关节组名称获取关节位置，为空默认返回所有关节
joint_group_names = ["left_arm"]
# 仅获取可活动关节
only_active_joint = True
ret = robot.get_joint_names(only_active_joint, joint_group_names)
print("Left joint names:")
for i, name in enumerate(ret):
    print(f"{i + 1}: {name}")

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')