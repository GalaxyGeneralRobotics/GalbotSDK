from galbot_sdk.g1 import GalbotMotion, GalbotRobot

# 获取 GalbotMotion 单例并初始化
motion = GalbotMotion.get_instance()
robot = GalbotRobot.get_instance()

if motion.init():
    print("GalbotMotion 初始化成功")
else:
    print("GalbotMotion 初始化失败")

if robot.init():
    print("GalbotRobot 初始化成功")
else:
    print("GalbotRobot 初始化失败")

# 仍然可以通过 GalbotRobot 管理机器人生命周期
robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()