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

chain_joints = {
    "leg": [0.4992,1.4991,1.0005,0.0000,-0.0004],
    "head": [0.0000,0.0],
    "left_arm": [1.9999,-1.6000,-0.5999,-1.6999,0.0000,-0.7999,0.0000],
    "right_arm": [-2.0000,1.6001,0.6001,1.7000,0.0000,0.8000,0.0000]
}
chain_pose_baselink = {
    "leg": [0.0596,-0.0000,1.0327,0.5000,0.5003,0.4997,0.5000],
    "head": [0.0599,0.0002,1.4098,-0.7072,0.0037,0.0037,0.7069],
    "left_arm": [0.1267,0.2342,0.7356,0.0220,0.0127,0.0343,0.9991],
    "right_arm": [0.1267,-0.2345,0.7358,-0.0225,0.0126,-0.0343,0.9991]
}
whole_body_joint = [
    num for key in ["leg", "head", "left_arm", "right_arm"]
    for num in chain_joints[key]
]
base_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
custom_param = gm.Parameter()

# 场景1：笛卡尔空间多路点规划（PoseState目标）
try:
    # 构造目标位姿
    target_pose_state = gm.PoseState()
    target_pose_state.chain_name = "left_arm"

    # 构造路点（3个中间位姿）
    waypoint_poses = [
        [0.1267,0.2342,0.7356,0.0220,0.0127,0.0343,0.9991],
        [0.2267,0.2342,0.7356,0.0220,0.0127,0.0343,0.9991],
        [0.3267,0.2342,0.7356,0.0220,0.0127,0.0343,0.9991],
        [0.4267,0.2342,0.7356,0.0220,0.0127,0.0343,0.9991],
    ]

    status, traj = motion.motion_plan_multi_waypoints(
        target=target_pose_state,
        waypoint_poses=waypoint_poses,
        enable_collision_check=False,
        params=custom_param
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "笛卡尔多点单链规划失败"
    if traj != {}:
        print(f"✅ 笛卡尔路点单链规划规划成功: 轨迹点数={len(traj[target_pose_state.chain_name])}")
        time.sleep(0.8)
    else:
        print(f"⚠️ 返回状态为SUCCESS，轨迹为空，可能已到达，检查目标值与当前状态是否一致或在误差范围内")
except Exception as e:
    print(f"❌ 笛卡尔多点运动规划异常: {e}")

# 场景2：关节空间多路点规划（JointStates目标）
try:
    # 构造目标位姿
    target_joint = gm.JointStates()
    target_joint.chain_name = "left_arm"

    # 构造路点（3个中间位姿）
    waypoints = [
        [0.1267,0.2342,0.7356,0.0220,0.0127,0.0343,0.9991],
        [0.2267,0.4342,0.7356,0.0220,0.0127,0.0343,0.9991],
        [0.3267,0.6342,0.7356,0.0220,0.0127,0.0343,0.9991],
        [0.4267,0.8342,0.7356,0.0220,0.0127,0.0343,0.9991]
    ]

    status, traj = motion.motion_plan_multi_waypoints(
        target=target_joint,
        waypoint_poses=waypoints,
        enable_collision_check=False,
        params=custom_param
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "笛卡尔多点单链规划失败"
    if traj != {}:
        print(f"✅ 关节路点单链规划规划成功: 轨迹点数={len(traj[target_pose_state.chain_name])}")
    else:
        print(f"⚠️ 返回状态为SUCCESS，轨迹为空，可能已到达，检查目标值与当前状态是否一致或在误差范围内")
except Exception as e:
    print(f"❌ 关节空间多点运动规划异常: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()