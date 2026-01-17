from galbot_sdk.g1 import GalbotRobot, JointGroup, ControlStatus
import time

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init()
time.sleep(1)
print('初始化成功')

# 设置吸盘所属的关节组（右吸盘）
joint_group = JointGroup.RIGHT_SUCTION_CUP

# 是否激活吸盘
activate = True  # True：激活吸盘，False：关闭吸盘

# 发送吸盘控制指令
status = robot.set_suction_cup_command(
    joint_group,
    activate
)

# 检查执行结果
if status != ControlStatus.SUCCESS:
    print("设置吸盘失败")
else:
    print("设置吸盘成功")

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')