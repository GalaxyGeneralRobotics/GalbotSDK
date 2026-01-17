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
time.sleep(1)

chain_pose_baselink = {
    "leg": [0.0596,-0.0000,1.0327,0.5000,0.5003,0.4997,0.5000],
    "head": [0.0599,0.0002,1.4098,-0.7072,0.0037,0.0037,0.7069],
    "left_arm": [0.1267,0.2342,0.7356,0.0220,0.0127,0.0343,0.9991],
    "right_arm": [0.1267,-0.2345,0.7358,-0.0225,0.0126,-0.0343,0.9991]
}
custom_param = gm.Parameter()
target_frame = "EndEffector"
reference_frame = "base_link"
target_chain = "left_arm"
# 场景1：基础版
try:
    end_ee_link = "left_arm_end_effector_mount_link"
    status, pose = motion.get_end_effector_pose(
        end_effector_frame=end_ee_link,
        reference_frame=reference_frame
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "获取末端位姿失败"
    print(f"✅ 基础版获取末端位姿成功: {pose}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ 基础版获取末端位姿异常: {e}")

# 场景2：指定链名 + 自定义frame
try:
    status, pose = motion.get_end_effector_pose_on_chain(
        chain_name=target_chain,
        frame_id=target_frame,
        reference_frame=reference_frame
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "获取末端位姿失败"
    print(f"✅ 指定链名获取末端位姿成功: {pose}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ 指定链名获取末端位姿异常: {e}")

end_effector_frame="left_arm"
reference_frame = "base_link"
try:
    status = motion.set_end_effector_pose(
        target_pose=chain_pose_baselink[end_effector_frame],
        end_effector_frame=end_effector_frame,
        reference_frame=reference_frame,
        enable_collision_check=False,
        is_blocking=False,
        timeout=5.0,
        params=custom_param
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "设置末端位姿失败"
    print(f"✅ 设置末端位姿成功: status={status}")
except Exception as e:
    print(f"❌ 设置末端位姿异常: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()