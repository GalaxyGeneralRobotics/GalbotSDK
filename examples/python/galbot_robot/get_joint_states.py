import time
from galbot_sdk.g1 import GalbotRobot

def print_joint_states(joint_states):
    """
    joint_state_vec: List of JointState, 每个对象具有 position, velocity, acceleration, effort, current
    """
    for js in joint_states:
        print(f" : position = {js.position} , velocity = {js.velocity} "
            f", acceleration = {js.acceleration} , effort = {js.effort} , current = {js.current}")

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init()
# 程序立即启动，稍等数据就绪时间
time.sleep(1)
print("初始化成功")
# 使用关节组名称获取关节状态，为空默认返回所有关节
joint_group_names = ["left_arm"]
ret = robot.get_joint_states(joint_group_names, [])
print_joint_states(ret)

# 获取指定关节状态，如果填充将覆盖关节组输入
joint_names = ["left_arm_joint1", "left_arm_joint2"]
state_ret = robot.get_joint_states([], joint_names)
print_joint_states(state_ret)

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')