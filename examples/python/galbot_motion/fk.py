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
reference_frame = "base_link"
target_frame = "EndEffector"
target_chain = "left_arm"
one_chain = [target_chain]
chain_with_torso = [target_chain, "torso"]
error_chains = [target_chain, "torso", "head"]
# 场景1：单链逆解
try:
    status, joint_map = motion.inverse_kinematics(
        target_pose=chain_pose_baselink[target_chain],
        chain_names=one_chain,
        target_frame=target_frame,
        reference_frame=reference_frame,
        enable_collision_check=False  # 关闭碰撞检测加速测试
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "逆运动学求解失败"
    print(f"✅ 基础版逆运动学成功: 关节角={joint_map}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ 基础版逆运动学异常: {e}")

# 场景2：手臂链+腰部逆解
try:
    status, joint_map = motion.inverse_kinematics(
        target_pose=chain_pose_baselink[target_chain],
        chain_names=chain_with_torso,
        target_frame=target_frame,
        reference_frame=reference_frame,
        enable_collision_check=False  # 关闭碰撞检测加速测试
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "逆运动学求解失败"
    print(f"✅ 自定义初始关节逆运动学成功: 关节角={joint_map}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ 自定义初始关节逆运动学异常: {e}")

# 场景3：非法链组合
try:
    status, joint_map = motion.inverse_kinematics(
        target_pose=chain_pose_baselink[target_chain],
        chain_names=error_chains,
        target_frame=target_frame,
        reference_frame=reference_frame,
        enable_collision_check=False  # 关闭碰撞检测加速测试
    )
    printStatus(status)
    assert status == gm.MotionStatus.INVALID_INPUT, "逆运动学求解失败"
    print(f"✅ 非法链组合输入检测成功")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ 自定义初始关节逆运动学异常: {e}")

# 场景4：使用参考关节
try:
    # initial_joint_positions可指定链关节作为逆解参考值，未指定链关节使用全身关节补全
    status, joint_map = motion.inverse_kinematics(
        target_pose=chain_pose_baselink[target_chain],
        chain_names=one_chain,
        target_frame=target_frame,
        reference_frame=reference_frame,
        initial_joint_positions=chain_joints,
        enable_collision_check=False  # 关闭碰撞检测加速测试
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "逆运动学求解失败"
    print(f"✅ 自定义初始关节逆运动学成功: 关节角={joint_map}")
    time.sleep(0.8)
except Exception as e:
    print(f"❌ 自定义初始关节逆运动学异常: {e}")

# 场景5：使用RobotStates
try:
    ref_robot_state = gm.RobotStates()
    ref_robot_state.chain_name = target_chain
    ref_robot_state.whole_body_joint = whole_body_joint
    ref_robot_state.base_state = base_state
    target_frame = "EndEffector"
    reference_frame = "base_link"
    status, joint_map = motion.inverse_kinematics_by_state(
        target_pose=chain_pose_baselink[target_chain],
        chain_names=one_chain,
        target_frame=target_frame,
        reference_frame=reference_frame,
        reference_robot_states=ref_robot_state
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "逆运动学求解失败"
    print(f"✅ 基于RobotStates逆运动学成功: 关节角={joint_map}")
except Exception as e:
    print(f"❌ 基于RobotStates逆运动学异常: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()