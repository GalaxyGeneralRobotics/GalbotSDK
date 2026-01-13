"""
注意：运行本示例时，请确认机器人的运动控制服务`/data/galbot/bin/service_motion_plan`、
    机器人状态发布服务`/data/galbot/bin/robot_state_publish`、
    以及手眼标定发布服务`/data/galbot/bin/eyehand_calib_publish`已加载;
"""
try:
    import galbot_sdk.g1 as gm
    from galbot_sdk.g1 import GalbotMotion, GalbotRobot, ControlStatus
except ImportError:
    print("import galbot_sdk failed, please install it first or check if it is in the PYTHONPATH")
    exit(1)

import time
import numpy as np
from typing import Sequence, Dict

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

def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.array(q, dtype=np.float64)
    return q / np.linalg.norm(q)

def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """
    计算四元数的共轭

    params:
    q (np.ndarray): 输入四元数 [x, y, z, w]

    return:
    np.ndarray: 四元数的共轭 [x, y, z, w]
    """
    qx, qy, qz, qw = q
    return np.array([-qx, -qy, -qz, qw])

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    计算两个四元数的乘积

    params:
    q1 (np.ndarray): 第一个四元数 [x, y, z, w]
    q2 (np.ndarray): 第二个四元数 [x, y, z, w]

    return:
    np.ndarray: 两个四元数的乘积 [x, y, z, w]
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def orientation_error_angle(A: np.ndarray, B: np.ndarray) -> float:
    """
    计算两个四元数之间的旋转角度误差（弧度）

    params:
    A (np.ndarray): 第一个四元数 [x, y, z, w]
    B (np.ndarray): 第二个四元数 [x, y, z, w]

    return:
    float: 旋转角度误差（弧度）
    """
    qA = quat_normalize(A[3:7])
    qB = quat_normalize(B[3:7])

    q_err = quat_multiply(qB, quat_conjugate(qA))
    q_err = quat_normalize(q_err)

    # 数值稳定
    qw = np.clip(q_err[3], -1.0, 1.0)

    angle = 2 * np.arccos(qw)
    return angle  # 单位：弧度


def calculate_error(pose1: Sequence[float], pose2: Sequence[float]) -> Dict[str, float]:
    """
    计算两个位姿之间的位置误差和旋转误差（弧度）

    params:
    pose1, pose2: [x, y, z, qx, qy, qz, qw]

    return:
    dict: 包含位置误差（米）和旋转误差（弧度）的字典
    """
    A, B = np.array(pose1), np.array(pose2)
    pos_err = np.linalg.norm(A[:3] - B[:3])
    rot_err = orientation_error_angle(A, B)

    return {
        "position_error_norm": pos_err,
        "orientation_error_rad": rot_err,
        "orientation_error_deg": np.degrees(rot_err)
    }

def check_robot_safety():
    """检查机器人是否安全"""
    # 提示注意事项
    print("⚠️  注意: 1.请确保机器人的急停按钮已放开; 2.请确保机器人前后左右无遮挡，避免发生意外情况。")
    while True:
        key = input("请确认机器人急停按钮已放开，且无遮挡，是否继续（y/n）...")
        if key == 'y':
            print("用户确认，继续运行...")
            break
        elif key == 'n':
            print("用户未确认，程序退出...")
            exit(1)
        else:
            print("输入错误，请输入'y'或'n'")

def main():
    check_robot_safety()
    try:
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

        # 程序立即启动，稍等数据就绪时间
        time.sleep(1)

        # 定义目标位姿
        chain_pose_baselink = {
            "leg": [0.0596,-0.0000,1.0327,0.5000,0.5003,0.4997,0.5000],
            "head": [0.0599,0.0002,1.4098,-0.7072,0.0037,0.0037,0.7069],
            "left_arm": [0.1267,0.2342,0.7356,0.0220,0.0127,0.0343,0.9991],
            "right_arm": [0.1267,-0.2345,0.7358,-0.0225,0.0126,-0.0343,0.9991]
        }
        # 定义目标链名、目标位姿、参考位姿、末端链接
        target_frame = "EndEffector"
        reference_frame = "base_link"
        target_chain = "left_arm"
        end_link = "left_arm_end_effector_mount_link"

        # 1. 获取当前left_arm末端位姿
        try:
            status, original_pose = motion.get_end_effector_pose_on_chain(
                chain_name=target_chain,
                frame_id=target_frame,
                reference_frame=reference_frame
            )
            assert status == gm.MotionStatus.SUCCESS, "获取末端位姿失败"
            print(f"✅ 当前{target_chain}末端位姿: {original_pose}")
            time.sleep(0.8)
        except Exception as e:
            print(f"❌ 指定链名获取末端位姿异常: {e}")

        # 2. 根据目标位姿IK求解关节角度，并验证求解结果
        # 2.1 通过IK求解目标位姿的关节角度joint_angles_ik
        try:
            status, joint_angles_ik = motion.inverse_kinematics(
                target_pose=chain_pose_baselink[target_chain],
                chain_names=[target_chain],
                target_frame=target_frame,
                reference_frame=reference_frame,
                enable_collision_check=False # 禁用碰撞检测
            )
            assert status == gm.MotionStatus.SUCCESS, "IK求解失败"
            print(f"✅ 目标{target_chain}IK求解成功 joint_angles_ik: {joint_angles_ik}")
            time.sleep(1)
        except Exception as e:
            print(f"❌ IK求解异常: {e}")

        # 2.2 通过设置关节组角度joint_angles_ik将末端位姿设置为目标位姿tgt_pose_ik
        try:
            status = robot.set_joint_positions(
                joint_angles_ik[target_chain], 
                [target_chain], 
                [], 
                True,
                0.1,
                20.0,
            )
            assert status == ControlStatus.SUCCESS, "设置关节组角度失败"
            print(f"✅ 设置{target_chain}关节组角度成功.")
            time.sleep(1)
        except Exception as e:
            print(f"❌ 设置{target_chain}关节组角度异常: {e}")

        # 2.3 验证设置的关节组角度是否与求解的角度一致
        try:
            status, tgt_pose_ik = motion.get_end_effector_pose_on_chain(
                chain_name=target_chain,
                frame_id=target_frame,
                reference_frame=reference_frame
            )
            assert status == gm.MotionStatus.SUCCESS, "获取末端位姿失败"
            print(f"✅ 获取{target_chain}末端位姿成功: {tgt_pose_ik}")
            time.sleep(1)

            error = calculate_error(tgt_pose_ik, chain_pose_baselink[target_chain])
            print(f"末端位姿误差: {error}")
        except Exception as e:
            print(f"❌ 获取{target_chain}末端位姿异常: {e}")

        # 2.4 通过FK求解关节组角度joint_angles_ik对应的末端位姿tgt_pose_fk是否和目标位姿tgt_pose_ik一致
        try:
            status, tgt_pose_fk = motion.forward_kinematics(
                target_frame=end_link,
                reference_frame=reference_frame,
                joint_state=joint_angles_ik,
                params=gm.Parameter()
            )
            assert status == gm.MotionStatus.SUCCESS, "FK求解失败"
            print(f"✅ 目标{target_chain}FK求解成功: {tgt_pose_fk}")
            time.sleep(1)

            error = calculate_error(tgt_pose_fk, chain_pose_baselink[target_chain])
            print(f"FK求解误差: {error}")
        except Exception as e:
            print(f"❌ FK求解异常: {e}")

        time.sleep(3)
        print()

        # 3. 通过设置末端位姿恢复到原始位姿
        # 3.1 设置末端位姿恢复到原始位姿
        try:
            status = motion.set_end_effector_pose(
                target_pose=original_pose,
                end_effector_frame=target_chain,
                reference_frame=reference_frame,
                enable_collision_check=False,
                is_blocking=True,
                timeout=5.0,
                params=gm.Parameter()
            )
            assert status == gm.MotionStatus.SUCCESS, "设置末端位姿失败"
            print(f"✅ 设置末端位姿成功: status={status}")
            time.sleep(1)
        except Exception as e:
            print(f"❌ 设置{target_chain}末端位姿异常: {e}")

        # 3.2 获取末端位姿并验证是否恢复到原始位姿
        try:
            status, original_pose_rec = motion.get_end_effector_pose_on_chain(
                chain_name=target_chain,
                frame_id=target_frame,
                reference_frame=reference_frame
            )
            assert status == gm.MotionStatus.SUCCESS, "获取末端位姿失败"
            print(f"✅ 获取{target_chain}末端位姿成功: {original_pose_rec}")
            time.sleep(0.8)
            
            error = calculate_error(original_pose_rec, original_pose)
            print(f"恢复末端位姿误差: {error}")
        except Exception as e:
            print(f"❌ 设置末端位姿异常: {e}")
    
    except Exception as e:
        print(f"❌ 主程序异常: {e}")
    finally:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()

if __name__=="__main__":
    main()