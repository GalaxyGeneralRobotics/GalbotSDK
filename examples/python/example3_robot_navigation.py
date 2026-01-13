"""
注意：运行本示例时，请确认机器人的导航功能`/data/galbot/bin/service_navigation_plan`已加载;
"""
try:
    from galbot_sdk.g1  import GalbotNavigation, GalbotRobot
except ImportError:
    print("import galbot_sdk failed, please install it first or check if it is in the PYTHONPATH")
    exit(1)

import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from typing import Sequence, Dict

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

def local_pose_to_global(start_pose: Sequence[float], local_pose: Sequence[float]):
    """将本地位姿local_pose转换为全局位姿

    Args:
        start_pose (Sequence[float]): 起始位姿，[x, y, z, qx, qy, qz, qw]
        local_pose (Sequence[float]): 本地位姿，[x, y, z, qx, qy, qz, qw]

    Returns:
        _type_: _description_
    """
    start_mat = np.eye(4)
    start_mat[:3, :3] = R.from_quat([start_pose[3], start_pose[4], start_pose[5], start_pose[6]]).as_matrix()
    start_mat[:3, 3] = [start_pose[0], start_pose[1], start_pose[2]]

    local_mat = np.eye(4)
    local_mat[:3, :3] = R.from_quat([local_pose[3], local_pose[4], local_pose[5], local_pose[6]]).as_matrix()
    local_mat[:3, 3] = [local_pose[0], local_pose[1], local_pose[2]]

    global_mat = start_mat @ local_mat

    return global_mat[:3, 3].tolist() + R.from_matrix(global_mat[:3, :3]).as_quat().tolist()

def demo_square_move(robot: GalbotRobot, nav: GalbotNavigation):
    try:
        start_pose = nav.get_current_pose()
    except Exception as e:
        print(f"获取当前位姿失败: {e}")
        return
    
    # 前向0.5m，左转90度
    local_pose = [0.5, 0.0, 0.0, 0.0, 0.0, 0.707, 0.707] 
    
    try:
        # 每次向前移动0.5m，左转90度，重复4次，形成一个正方形
        for _ in range(4):
            # 计算目标点位姿
            cur_pose = nav.get_current_pose()
            goal_pose = local_pose_to_global(cur_pose, local_pose)
            
            # 检查路径是否可达
            if nav.check_path_reachability(goal_pose, cur_pose):
                # 导航到目标位姿
                retry_cnt = 3
                while True:
                    status = nav.navigate_to_goal(goal_pose, enable_collision_check=True, is_blocking=True, timeout=30)
                    time.sleep(0.5)
                    retry_cnt -= 1
                    if nav.check_goal_arrival() or retry_cnt < 0:
                        break
                    else:
                        print(f"导航失败，重试中...{retry_cnt}")
                print("navigate_to_goal 返回状态:", status)
                print("是否到达:", nav.check_goal_arrival())
            else:
                print("路径不可达或不安全")

        cur_pose = nav.get_current_pose()
        print(f"当前位姿: {cur_pose}, 和起始位姿误差: {calculate_error(cur_pose, start_pose)}")
    except Exception as e:
        print(f"导航过程中发生异常: {e}")

def move_to_original(robot: GalbotRobot, nav: GalbotNavigation):
    cur_pose = nav.get_current_pose()
    goal_pose = [0, 0, 0, 0, 0, 0, 1]
    
    try:
        if nav.check_path_reachability(goal_pose, cur_pose):
            retry_cnt = 3
            while True:
                status = nav.navigate_to_goal(goal_pose, enable_collision_check=True, is_blocking=True, timeout=30)
                time.sleep(0.5)
                retry_cnt -= 1
                if nav.check_goal_arrival() or retry_cnt < 0:
                    break
                else:
                    print(f"导航失败，重试中...{retry_cnt}")
            print("navigate_to_goal 返回状态:", status)
            print("是否到达:", nav.check_goal_arrival())
        else:
            print("路径不可达或不安全")
    except Exception as e:
        print(f"导航过程中发生异常: {e}")

def check_robot_safety():
    """检查机器人是否安全"""
    # 提示注意事项
    print("⚠️  注意: 1.请确保机器人的急停按钮已放开; 2.请确保机器人前后左右无遮挡，避免发生意外情况; 3.请确保机器人周围空旷，无障碍物。")
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
        # 获取机器人实例
        robot = GalbotRobot.get_instance()
        # 获取导航实例
        nav = GalbotNavigation.get_instance()
        
        # 初始化机器人
        if robot.init():
            print("机器人初始化成功")
        else:
            print("机器人初始化失败")
        # 初始化导航
        if nav.init():  
            print("导航初始化成功")
        else:
            print("导航初始化失败")
        
        # 等待数据准备
        time.sleep(1)
        
        # 检查初始定位状态
        is_localized = nav.is_localized()
        if not is_localized:
            print("定位失败，尝试重新定位: 请将机器人移动到地图原点！")
        time.sleep(3)
        while not is_localized:
            nav.relocalize([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
            time.sleep(0.5)
            is_localized = nav.is_localized()

        # square_move
        demo_square_move(robot, nav)

    except Exception as e:
        print(f"发生异常: {e}")
    finally:
        robot.request_shutdown()
        robot.wait_for_shutdown()
        robot.destroy()

if __name__ == "__main__":
    main()
