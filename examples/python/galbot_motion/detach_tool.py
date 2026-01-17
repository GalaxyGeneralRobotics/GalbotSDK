import time
import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotMotion, GalbotRobot

# 获取 GalbotMotion 单例并初始化
motion = GalbotMotion.get_instance()
robot = GalbotRobot.get_instance()

def printStatus(status):
        if(status == gm.MotionStatus.SUCCESS):
            print("执行结果: SUCCESS, 执行成功")
        elif(status == gm.MotionStatus.TIMEOUT):
            print("执行结果: TIMEOUT, 执行超时")
        elif(status == gm.MotionStatus.FAULT):
            print("执行结果: FAULT, 发生故障无法继续执行")
        elif(status == gm.MotionStatus.INVALID_INPUT):
            print("执行结果: INVALID_INPUT, 输入参数不符合要求")
        elif(status == gm.MotionStatus.INIT_FAILED):
            print("执行结果: INIT_FAILED, 内部通讯组件创建失败")
        elif(status == gm.MotionStatus.IN_PROGRESS):
            print("执行结果: IN_PROGRESS, 正在运动中但未到位")
        elif(status == gm.MotionStatus.STOPPED_UNREACHED):
            print("执行结果: STOPPED_UNREACHED, 已停止但未到达目标")
        elif(status == gm.MotionStatus.DATA_FETCH_FAILED):
            print("执行结果: DATA_FETCH_FAILED, 数据获取失败")
        elif(status == gm.MotionStatus.PUBLISH_FAIL):
            print("执行结果: PUBLISH_FAIL, 数据发送失败")
        elif(status == gm.MotionStatus.COMM_DISCONNECTED):
            print("执行结果: COMM_DISCONNECTED, 连接失败")

if motion.init():
    print("GalbotMotion 初始化成功")
else:
    print("GalbotMotion 初始化失败")
if robot.init():
    print("GalbotRobot 初始化成功")
else:
    print("GalbotRobot 初始化失败")

# 程序立即启动，稍等数据就绪时间
time.sleep(2)

# 1. 卸载工具
try:
    chain_name = "left_arm"
    status = motion.detach_tool(
        chain=chain_name
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "卸载工具失败"
    print(f"✅ 卸载工具成功")
except Exception as e:
    print(f"❌ 卸载工具异常: {e}") 

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()