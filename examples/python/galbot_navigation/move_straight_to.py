from galbot_sdk.g1 import GalbotNavigation, GalbotRobot
import numpy as np
import time

nav = GalbotNavigation.get_instance()
nav.init()
robot = GalbotRobot.get_instance()
robot.init()

target = np.array([0.2, 0.0, 0.0, 0, 0, 0.0, 1.0])

nav.move_straight_to(target, is_blocking=False, timeout=10)
time.sleep(1.0)
nav.stop_navigation()

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')