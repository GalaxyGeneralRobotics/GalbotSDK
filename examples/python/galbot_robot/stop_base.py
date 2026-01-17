from galbot_sdk.g1 import GalbotRobot, ControlStatus

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init()
time.sleep(1)
print("初始化成功")

# 发送停止底盘运动指令
while True:
    status = robot.stop_base()
    # 检查执行结果
    if status == ControlStatus.SUCCESS:
        print("停止底盘运动成功")
        break
    print("停止底盘失败，重试中...")

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')