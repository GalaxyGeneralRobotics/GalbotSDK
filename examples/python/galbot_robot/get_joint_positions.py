import time
from galbot_sdk.g1 import GalbotRobot

def print_joint_positions(joint_positions):
    print(f"pos count is {len(joint_positions)}")
    for pos in joint_positions:
        print(pos)

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init()
# 程序立即启动，稍等数据就绪时间
time.sleep(1)
print("初始化成功")

# 使用关节组名称获取关节位置，为空默认返回所有关节
joint_group_names = ["left_arm"]
ret = robot.get_joint_positions(joint_group_names, [])
print("左臂关节位置：")
print_joint_positions(ret)
# 获取指定关节位置，如果填充将覆盖关节组输入
joint_names = ["left_arm_joint1", "left_arm_joint2"]
state_ret = robot.get_joint_positions([], joint_names)
print("左臂1、2关节位置:")
print_joint_positions(state_ret)

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')