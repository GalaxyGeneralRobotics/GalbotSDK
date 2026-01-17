from galbot_sdk.g1 import GalbotNavigation, GalbotRobot
import numpy as np

nav = GalbotNavigation.get_instance()
nav.init()
robot = GalbotRobot.get_instance()
robot.init()

start = nav.get_current_pose()
goal = np.array([1.0, 1.0, 0.0, 0, 0, 0.4794255, 0.8775826])

if nav.check_path_reachability(goal, start):
    status = nav.navigate_to_goal(goal, enable_collision_check=True, is_blocking=True, timeout=30)
    print("navigate_to_goal 返回状态:", status)
    print("是否到达:", nav.check_goal_arrival())
else:
    print("路径不可达或不安全")
    
# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')