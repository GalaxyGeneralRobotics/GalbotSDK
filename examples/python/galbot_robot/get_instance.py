from galbot_sdk.g1 import GalbotRobot
import time

# 获取 GalbotRobot 的单例
robot = GalbotRobot.get_instance()

state = robot.init()
if not state:
    print("初始化失败")
else:
    print("初始化成功")

while robot.is_running():
    # 业务逻辑
    time.sleep(1)
    break

# 发出退出信号退出程序
robot.request_shutdown()
# 等待进入退出状态
robot.wait_for_shutdown()
# SDK相关资源释放
robot.destroy()