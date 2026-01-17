from galbot_sdk.g1 import GalbotNavigation, GalbotRobot
import numpy as np
import time

nav = GalbotNavigation.get_instance()
nav.init()
robot = GalbotRobot.get_instance()
robot.init()

goal = np.array([0.5, 0.0, 0.0, 0, 0, 0.0, 1.0])

nav.navigate_to_goal(goal, enable_collision_check=True, is_blocking=False, timeout=20)
while not nav.check_goal_arrival():
    print("正在导航...")
    time.sleep(0.5)

print("已到达目标")

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')