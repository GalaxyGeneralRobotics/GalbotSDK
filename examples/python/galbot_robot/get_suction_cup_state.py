import time
from galbot_sdk.g1 import GalbotRobot, JointGroup

def print_suction_cup_state(suction_cup_state):
    """
    suction_cup_state: 对象，包含 timestamp_ns, pressure, activation, action_state
    """
    group_name = joint_group.name
    print(f"Timestamp (ns): {suction_cup_state.timestamp_ns}")
    print(
        f"pressure {suction_cup_state.pressure} "
        f"activation {suction_cup_state.activation} "
        f"action state {int(suction_cup_state.action_state)}"
    )

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init()

# 程序立即启动，稍等数据就绪时间
time.sleep(1)
print("初始化成功")

# 设置吸盘所属的关节组（右吸盘）
joint_group = JointGroup.RIGHT_SUCTION_CUP

# 获取吸盘状态
suction_cup_state = robot.get_suction_cup_state(joint_group)

if suction_cup_state is None:
    print("get suction cup error")
else:
    print("右吸盘状态：")
    print_suction_cup_state(suction_cup_state)

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')