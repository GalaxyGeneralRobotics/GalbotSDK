from galbot_sdk.g1 import GalbotRobot, ControlStatus
import time

# 获取 GalbotRobot 的单例
robot = GalbotRobot.get_instance()
robot.init()
time.sleep(1)
print("初始化成功")

# 设置底盘速度
linear_velocity = [0.1, 0.0, 0.0]  # 前进 0.5 m/s
angular_velocity = [0.0, 0.0, 0.1]  # 旋转 0.1 rad/s
# 注意：将以指定速度运行，直到下发停止运动指令
status = robot.set_base_velocity(linear_velocity, angular_velocity)

if status == ControlStatus.SUCCESS:
    print("底盘速度设置成功。")
else:
    print("设置底盘速度失败。")

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')