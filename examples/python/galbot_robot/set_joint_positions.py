import time
from galbot_sdk.g1 import GalbotRobot, ControlStatus

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init()
print('初始化成功')

# 程序立即启动，稍等数据就绪时间
time.sleep(2)

# 设置头部两个关节角度为0.2，0.2，阻塞等待动作执行到位，最大超时时间为10s
joint_pos = [0.2, 0.2]
# 设置头部关节组，如为空将默认填写全身关节
joint_groups = ["head"]
# 是否阻塞等待关节运行到位
is_blocking = True
# 限制关节最大运行速度为0.1rad/s
max_speed = 0.1
# 阻塞等待最大时间
timeout_s = 10

status = robot.set_joint_positions(
    joint_pos, joint_groups, [], is_blocking, max_speed, timeout_s
)

if status != ControlStatus.SUCCESS:
    print("关节角设置失败")
else:
    print('关节角设置成功')

time.sleep(1)

# 使用特定关节名称进行控制，该参数将覆盖joint_groups关节组参数
joint_names = ["head_joint1", "head_joint2"]
joint_pos = [0.0, 0.0]

status = robot.set_joint_positions(
    joint_pos, [], joint_names, is_blocking, max_speed, timeout_s
)

if status != ControlStatus.SUCCESS:
    print("关节角设置失败")
else:
    print('关节角设置成功')

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')