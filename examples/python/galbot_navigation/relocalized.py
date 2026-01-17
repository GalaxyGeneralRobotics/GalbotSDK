from galbot_sdk.g1 import GalbotNavigation, GalbotRobot
import numpy as np
import time

nav = GalbotNavigation.get_instance()
nav.init()
robot = GalbotRobot.get_instance()
robot.init()

init_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

# 尝试确保定位成功
while not nav.is_localized():
    nav.relocalize(init_pose)
    time.sleep(0.5)

print("当前位姿:", nav.get_current_pose())

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')