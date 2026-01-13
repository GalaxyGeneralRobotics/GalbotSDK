"""
注意：运行本示例时，请确认机器人的运动控制服务`/data/galbot/bin/service_motion_plan`、
    机器人状态发布服务`/data/galbot/bin/robot_state_publish`、
    导航服务`/data/galbot/bin/service_navigation_plan`
    以及手眼标定发布服务`/data/galbot/bin/eyehand_calib_publish`已加载;
"""
try:
    import galbot_sdk.g1 as gm
    from galbot_sdk.g1  import GalbotNavigation, GalbotRobot, GalbotMotion, JointGroup, SensorType
except ImportError:
    print("import galbot_sdk failed, please install it first or check if it is in the PYTHONPATH")
    exit(1)

import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from typing import Sequence
import cv2

def decode_compressed_image(compressed_image, camera_info={}):
    """
    decode CompressedImage image

    Params:
        compressed_image: image dict, keys:[header, format, data, "depth_scale"]

    Returns:
        numpy.ndarray: decoded image
    """
    image_data = compressed_image["data"]
    if compressed_image["format"] == "rgb8":
        return decode_rgb_image(image_data)
    elif compressed_image["format"] == "16UC1":
        return decode_depth_image(image_data, compressed_image["depth_scale"], camera_info)
    else:
        raise ValueError(f"Unsupport data format: {compressed_image['format']}")

def decode_rgb_image(image_data):
    """decode rgb image"""
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Fail to Decode RGB Image")
    return img

def decode_depth_image(image_data, depth_scale, camera_info):
    """decode depth image"""
    depth_img = np.frombuffer(image_data, dtype=np.uint16).copy()

    if not camera_info:
        depth_img = depth_img.reshape((720, 1280))
    else:
        depth_img = depth_img.reshape((camera_info["height"], camera_info["width"]))
    depth_img = depth_img.astype(np.float32) / depth_scale

    return depth_img

def print_gripper_state(joint_group, gripper_state):
    """
    joint_group: JointGroup 枚举
    gripper_state: 对象，包含 timestamp_ns, width, velocity, effort, is_moving
    """
    print(f"Timestamp (ns): {gripper_state.timestamp_ns}")
    print(
        f"width {gripper_state.width} "
        f"velocity {gripper_state.velocity} "
        f"effort {gripper_state.effort} "
        f"is moving {gripper_state.is_moving}"
    )

def get_navigation_pose(object_goal_pose: Sequence[float], motion: GalbotMotion, arm: str = "left_arm"):
    """
    获取导航目标位姿

    Args:
        object_goal_pose (Sequence[float]): 目标位姿[x, y, z, qx, qy, qz, qw]
        motion (GalbotMotion): 运动控制实例
        arm (str, optional): 末端执行器名称. Defaults to "left_arm".

    Returns:
        Sequence[float]: 导航目标位姿[x, y, z, qx, qy, qz, qw]
    """
    assert arm in ["left_arm", "right_arm"], "arm must be left_arm or right_arm"

    try:
        status, ee_pose_in_base = motion.get_end_effector_pose(
            end_effector_frame=f"{arm}_end_effector_mount_link",
            reference_frame="base_link"
        )
        if status != gm.MotionStatus.SUCCESS:
            print(f"获取末端位姿失败: status={status}")
            offset_y = ee_pose_in_base[1]
        else:
            print(f"获取末端位姿成功: pose={ee_pose_in_base}")
            offset_y = 0.3

        # 底盘位姿先设置成和目标位姿在同一个z坐标上
        base_goal_pose_mat = np.eye(4)
        base_goal_pose_mat[:3, :3] = R.from_quat(object_goal_pose[3:]).as_matrix()
        base_goal_pose_mat[:3, 3] = np.array([object_goal_pose[0], object_goal_pose[1], 0])

        # 根据底盘和相机的相对位置，相机导航目标往局部坐标的后方移动0.6m，留出给相机的观察空间
        base_goal_pose_mat = base_goal_pose_mat @ np.array([[1,0,0,-0.6],[0,1,0,-offset_y],[0,0,1,0],[0,0,0,1]])
        print(base_goal_pose_mat)

        base_goal_pose_quat = R.from_matrix(base_goal_pose_mat[:3, :3]).as_quat()
        base_goal_pose_pos = base_goal_pose_mat[:3, 3]

        return base_goal_pose_pos.tolist() + base_goal_pose_quat.tolist()
    
    except Exception as e:
        print("获取导航目标位姿失败:", e)
    

def navigation_to_goal(nav: GalbotNavigation, goal_pose: Sequence[float], retry_cnt: int = 3):
    """
    导航到目标位姿

    Args:
        nav (GalbotNavigation): 导航实例
        goal_pose (Sequence[float]): 目标位姿[x, y, z, qx, qy, qz, qw]
        retry_cnt (int, optional): 重试次数. Defaults to 3.
    """
    try:
        cur_pose = nav.get_current_pose()
        print(f"当前位姿: {cur_pose}")
        if nav.check_path_reachability(goal_pose, cur_pose):
            retry_cnt = 3
            while True:
                status = nav.navigate_to_goal(goal_pose, enable_collision_check=True, is_blocking=True, timeout=20)
                time.sleep(0.5)
                retry_cnt -= 1
                if nav.check_goal_arrival() or retry_cnt < 0:
                    break
                else:
                    print(f"导航失败: status={status}, 重试中: {retry_cnt}")
            print("navigate_to_goal 返回状态:", status)
            print("是否到达:", nav.check_goal_arrival())
        else:
            print("路径不可达或不安全")
    except Exception as e:
        print(f"导航过程中发生异常: {e}")

def lift_camera_up(motion: GalbotMotion, target_pose: Sequence[float], target_chain: str, reference_frame: str):
    """
    相机抬起到目标高度

    Args:
        motion (GalbotMotion): 运动控制实例
        target_pose (Sequence[float]): 目标位姿[x, y, z, qx, qy, qz, qw]
        target_chain (str): 末端执行器名称
        reference_frame (str): 参考坐标系
    """
    try:
        retry_cnt = 3
        while True:
            status, cur_ee_pose = motion.get_end_effector_pose_on_chain(
                chain_name=target_chain,
                frame_id="EndEffector",
                reference_frame=reference_frame
            )
            time.sleep(0.5)
            retry_cnt -= 1

            if status == gm.MotionStatus.SUCCESS or retry_cnt < 0:
                print(f"当前末端位姿: {cur_ee_pose}")
                break
            else:
                print(f"获取末端位姿失败: status={status}, 重试中: {retry_cnt}")
        
        tgt_ee_pose = cur_ee_pose.copy()
        tgt_ee_pose[2] = target_pose[2] - 0.1
        print(f"目标末端位姿: {tgt_ee_pose}")

        retry_cnt = 3
        while True:
            status = motion.set_end_effector_pose(
                target_pose=tgt_ee_pose,
                end_effector_frame=target_chain,
                reference_frame=reference_frame,
                enable_collision_check=False,
                is_blocking=True,
                timeout=5.0,
                params=gm.Parameter()
            )
            time.sleep(0.5)
            retry_cnt -= 1
            if status == gm.MotionStatus.SUCCESS or retry_cnt < 0:
                print(f"✅ 设置末端位姿成功: status={status}")
                break
            else:
                print(f"设置末端位姿失败: status={status}，重试中: {retry_cnt}")
    except Exception as e:
        print(f"❌ 抬起相机失败: {e}")

def detect_target(img: np.ndarray, depth_img: np.ndarray) -> Sequence[float]:
    """
    检测目标函数. 输入rgb图像和深度图像, 输出目标位姿.

    Args:
        img (np.ndarray): rgb图像
        depth_img (np.ndarray): 深度图像

    Returns:
        Sequence[float]: 目标位姿[x, y, z, qx, qy, qz, qw]
    """
    try:
        
        # 假设检测到目标位姿为[-0.05, -0.1, 0.12, 0.0, 0.0, 0.0, 1.0]
        # 表示目标在相机前方0.12m，左侧0.05m，高度0.1m，方向为相机默认方向
        default_pose = [-0.05, -0.1, 0.12, 0.0, 0.0, 0.0, 1.0]
        
        return default_pose
    except Exception as e:
        print(f"检测目标异常: {e}")
        return None

def pose_camera_to_base(robot: GalbotRobot, pose_camera: Sequence[float]) -> Sequence[float]:
    """
    将相机位姿转换到底盘坐标系

    Args:
        robot (GalbotRobot): 机器人实例
        pose_camera (Sequence[float]): 相机位姿[x, y, z, qx, qy, qz, qw]

    Returns:
        Sequence[float]: 底盘位姿[x, y, z, qx, qy, qz, qw]
    """
    source_frame="left_arm_camera_color_optical_frame"
    target_frame="base_link"
    base_to_cam = robot.get_transform(target_frame, source_frame)[0]
    if base_to_cam is None:
        print("获取相机到底盘变换失败")
        return None
    else:
        print("base_to_cam: ", base_to_cam)

    base_to_cam_mat = np.eye(4)
    base_to_cam_mat[:3, :3] = R.from_quat(base_to_cam[3:]).as_matrix()
    base_to_cam_mat[:3, 3] = np.array(base_to_cam[:3])
    
    pose_camera_mat = np.eye(4)
    pose_camera_mat[:3, :3] = R.from_quat(pose_camera[3:]).as_matrix()
    pose_camera_mat[:3, 3] = np.array(pose_camera[:3])
    
    pose_base_mat = base_to_cam_mat @ pose_camera_mat[:, 3:]
    print("pose_base_mat: ", pose_base_mat)
    
    return pose_base_mat.flatten()[:3].tolist() + [0, 0, 0, 1]

def detect_object(robot: GalbotRobot, arm: str = "left_arm"):
    try:
        # 获取相机图像数据
        if arm == "left_arm":
            rgb_image_data = robot.get_rgb_data(SensorType.LEFT_ARM_CAMERA)
            depth_data = robot.get_depth_data(SensorType.LEFT_ARM_DEPTH_CAMERA)
        elif arm == "right_arm":
            rgb_image_data = robot.get_rgb_data(SensorType.RIGHT_ARM_CAMERA)
            depth_data = robot.get_depth_data(SensorType.RIGHT_ARM_DEPTH_CAMERA)
        else:
            raise ValueError("arm must be left_arm or right_arm")

        # 解码图像数据
        if not rgb_image_data:
            print("No rgb image data!")
        else:
            print("get rgb image suceess")
            img = decode_compressed_image(rgb_image_data)
        
        if not depth_data:
            print("No depth_data!")
        else:
            depth_img = decode_compressed_image(depth_data)
            print("get depth data suceess")

        # 检测目标
        object_pose_camera = detect_target(img, depth_img)
        if object_pose_camera is None:
            print("检测目标失败")
            return None
        else:
            print(f"object_pose_camera: {object_pose_camera}")
        
        # 计算目标位姿在底盘坐标系下的位姿
        object_pose_base = pose_camera_to_base(robot, object_pose_camera)
        print(f"目标位姿在底盘坐标系下的位姿: {object_pose_base}")
    
    except Exception as e:
        print(f"检测目标异常: {e}")

    return object_pose_base

def check_robot_safety():
    """检查机器人是否安全"""
    # 提示注意事项
    print("⚠️  注意: 1.请确保机器人的急停按钮已放开; 2.请确保机器人前后左右无遮挡，避免发生意外情况。3.请确保机器人周围空旷，无障碍物。")
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

def pick_and_place(robot: GalbotRobot, 
                   nav: GalbotNavigation, 
                   motion: GalbotMotion, 
                   object_pose_base: Sequence[float], 
                   target_chain: str, 
                   reference_frame: str):
    try:
        # 打开左夹爪
        # 设置左夹爪宽度为0.1m，运行速度为0.05m，力矩为10N，将阻塞等待夹爪运行到位
        status = robot.set_gripper_command(
            JointGroup.LEFT_GRIPPER, 0.1, 0.05, 10, True
        )
        time.sleep(0.5)

        print("object_pose_base: ", object_pose_base)
        
        # 伸手到目标位置
        retry_cnt = 3
        while True:
            status = motion.set_end_effector_pose(
                target_pose=object_pose_base,
                end_effector_frame=target_chain,
                reference_frame=reference_frame,
                enable_collision_check=False,
                is_blocking=True,
                timeout=5.0,
                params=gm.Parameter()
            )
            time.sleep(1)
            retry_cnt -= 1
            if status == gm.MotionStatus.SUCCESS or retry_cnt < 0:
                break
            else:
                print(f"设置末端位姿失败: status={status}, 重试次数: {retry_cnt}")
        
        assert status == gm.MotionStatus.SUCCESS, "设置末端位姿失败"
        print(f"✅ 设置末端位姿成功: status={status}")

        # 夹爪闭合取物
        status = robot.set_gripper_command(
            JointGroup.LEFT_GRIPPER, 0.02, 0.05, 10, True
        )
        time.sleep(0.5)

        # 回归到初始位置
        navigation_to_goal(nav, [0, 0, 0, 0, 0, 0, 1])
        time.sleep(2)

        # 松手放开目标
        status = robot.set_gripper_command(
            JointGroup.LEFT_GRIPPER, 0.1, 0.05, 10, True
        )
        time.sleep(0.5)
    except Exception as e:
        print(f"pick_and_place 过程中发生异常: {e}")
        return None


def main():
    check_robot_safety()
    try:
        # 获取机器人实例
        robot = GalbotRobot.get_instance()
        # 获取GalbotMotion实例
        motion = GalbotMotion.get_instance()
        # 获取导航实例
        nav = GalbotNavigation.get_instance()

        # 获取左臂的RGB图像和深度图像，右臂的深度图像，底座的激光雷达数据，躯干的IMU数据
        enable_sensor_set = {SensorType.LEFT_ARM_CAMERA, # 左臂深度相机
                            SensorType.LEFT_ARM_DEPTH_CAMERA, # 左臂RGB相机
                            SensorType.BASE_LIDAR, # 底座激光雷达
                            SensorType.TORSO_IMU} # 躯干IMU传感器

        # 初始化机器人
        if robot.init(enable_sensor_set):
            print("GalbotRobot 初始化成功")
        else:
            print("GalbotRobot 初始化失败")
        if motion.init():
            print("GalbotMotion 初始化成功")
        else:
            print("GalbotMotion 初始化失败")
        if nav.init():
            print("GalbotNavigation 初始化成功")
        else:
            print("GalbotNavigation 初始化失败")

        # 程序立即启动，稍等数据就绪时间
        time.sleep(1)

        # 计算出导航目标位姿
        object_goal_pose = [-1, 0.33, 0.90, 0, 0, 1, 0]
        base_goal_pose = [0, 0, 0, 0, 0, 0, 1]
        base_goal_pose = get_navigation_pose(object_goal_pose, motion)
        
        # 导航到目标位姿
        navigation_to_goal(nav, base_goal_pose)
        
        target_chain = "left_arm"
        reference_frame = "base_link"

        # 获取当前末端位姿，用于后续恢复
        try:
            status, original_pose = motion.get_end_effector_pose_on_chain(
                chain_name=target_chain,
                frame_id="EndEffector",
                reference_frame=reference_frame
            )
            assert status == gm.MotionStatus.SUCCESS, "获取末端位姿失败"
            print(f"✅ 获取{target_chain}末端位姿成功: {original_pose}")
            time.sleep(1)
        except Exception as e:
            print(f"❌ 获取{target_chain}末端位姿异常: {e}")

        # 抬起相机
        lift_camera_up(motion, object_goal_pose, "left_arm", "base_link")

        # 检测目标
        object_pose_base = detect_object(robot, arm="left_arm")
        if object_pose_base is None:
            print("检测目标失败")
            return None
        else:
            print(f"检测到目标位姿: {object_pose_base}")
        
        # 抓取并放回初始位置
        pick_and_place(robot, nav, motion, object_pose_base, "left_arm", "base_link")

        # 取到目标后，姿态回正
        try:
            time.sleep(2)
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
            
    except Exception as e:
        print(f"出现异常: {e}")
    finally:
        # 主动发出SIGINT退出信号
        robot.request_shutdown()
        # 等待进入shutdown状态
        robot.wait_for_shutdown()
        # 进行SDK资源释放
        robot.destroy()
        print('资源释放成功')

if __name__ == "__main__":
    main()