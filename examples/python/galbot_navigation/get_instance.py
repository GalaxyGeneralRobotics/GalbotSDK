from galbot_sdk.g1 import GalbotNavigation, GalbotRobot
import numpy as np

# 初始化系统与导航模块
robot = GalbotRobot.get_instance()
robot.init()

nav = GalbotNavigation.get_instance()
nav.init()

print("GalbotNavigation 已初始化:", nav is not None)

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')