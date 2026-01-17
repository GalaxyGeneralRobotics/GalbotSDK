import time
from galbot_sdk.g1 import GalbotRobot, JointGroup, ControlStatus

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init()
print('初始化成功')
# 程序立即启动，稍等数据就绪时间
time.sleep(2)

# 设置左夹爪宽度为0.1m，运行速度为0.05m，力矩为10N，将阻塞等待夹爪运行到位
status = robot.set_gripper_command(
    JointGroup.LEFT_GRIPPER, 0.1, 0.05, 10, True
)

if status != ControlStatus.SUCCESS:
    print("设置夹爪失败")
else:
    print('设置夹爪成功')

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')