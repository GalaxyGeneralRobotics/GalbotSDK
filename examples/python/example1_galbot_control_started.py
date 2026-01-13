"""
注意：运行本示例时，请确认机器人的`急停按钮`已放开;
"""
import time

try:
    from galbot_sdk.g1  import GalbotRobot, ControlStatus
except ImportError:
    print("import galbot_sdk failed, please install it first or check if it is in the PYTHONPATH")
    exit(1)


def demo_heart_pose(robot: GalbotRobot,
                    joint_group_names: list,
                    position_seq: list,
                    is_blocking: bool,
                    max_speed: float,
                    timeout_s: float,
                    retry_count: int = 3
                    ):
    """
    机器人比心动作演示函数

    参数:
    robot (GalbotRobot): GalbotRobot 实例
    joint_group_names (list): 要控制的关节组名称列表
    position_seq (list): 要设置的关节组角度序列列表
    is_blocking (bool): 是否阻塞式设置角度
    max_speed (float): 最大速度
    timeout_s (float): 超时时间（秒）
    retry_count (int, optional): 重试次数，默认3次
    """

    # 获取当前关节组角度，用于后续恢复
    original_pos = robot.get_joint_positions(joint_group_names, [])
    print(f"关节组{joint_group_names}的当前角度: {original_pos}")

    # 开始比心动作
    pos_idx = 0
    print("开始比心动作...")
    while True:
        time.sleep(1)
        pos = position_seq[pos_idx]
        control_status = robot.set_joint_positions(
            pos, joint_group_names, [], is_blocking, max_speed, timeout_s
        )

        # 若设置失败，重试3次
        retry_cnt = retry_count
        while control_status != ControlStatus.SUCCESS and retry_cnt > 0:
            print(f"设置关节组{joint_group_names}角度失败, 重试中 {retry_cnt}...")
            retry_cnt = retry_cnt - 1
            time.sleep(1)
            control_status = robot.set_joint_positions(
                pos, joint_group_names, [], is_blocking, max_speed, timeout_s
            )
        # 若成功，切换到下一个姿态序列
        if control_status == ControlStatus.SUCCESS:
            print(f"设置关节组{joint_group_names}角度成功")
            pos_idx = pos_idx + 1
        # 若失败，跳出循环
        else:
            print(f"设置关节组{joint_group_names}角度失败")
            break

        # 若所有姿态序列都设置完成，跳出循环
        if pos_idx > len(position_seq) - 1:
            break

    # 获取当前关节组角度
    print("展示比心动作15秒，然后恢复原始姿态...")
    time.sleep(5)

    # 恢复原始姿态关节组角度
    control_status = robot.set_joint_positions(
        original_pos, joint_group_names, [], is_blocking, max_speed, timeout_s
    )
    # 若设置失败，重试5次
    retry_cnt = retry_count
    while control_status != ControlStatus.SUCCESS and retry_cnt > 0:
        print(f"设置关节组{joint_group_names}角度失败, 重试中 {retry_cnt}...")
        retry_cnt = retry_cnt - 1
        time.sleep(2)
        control_status = robot.set_joint_positions(
            original_pos, joint_group_names, [], is_blocking, max_speed, timeout_s
        )
    # 若成功，恢复原始姿态
    if control_status == ControlStatus.SUCCESS:
        print(f"恢复关节组{joint_group_names}角度成功")
    else:
        print(f"恢复关节组{joint_group_names}角度失败")

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
        # 获取机器人实例
        robot = GalbotRobot.get_instance()
        
        # 初始化机器人
        state = robot.init()
        if not state:
            print(f"初始化失败")
            exit(1)
        else:
            print(f"初始化成功")
            print(f"机器人是否运行: {robot.is_running()}")

        # 等待数据准备
        time.sleep(3)

        # 获取关节名称列表
        joint_names = robot.get_joint_names()
        if len(joint_names) > 0:
            print(f"关节名称列表: {joint_names}")
        else:
            print(f"获取关节名称列表失败")

        # 使用关节组名称获取关节位置，为空默认返回所有关节
        joint_group_names = ["left_arm", "right_arm"]
        # 左右臂比心动作序列
        position_seq = [
            [1.53, 0.36, -2.54, -1.80, 0.12, -0.82, 0.09, # left_arm
             -1.53, -0.36, 2.54, 1.80, -0.12, 0.82, -0.09] # right_arm
        ]
        # 是否阻塞等待关节运行到位
        is_blocking = True
        # 限制关节最大运行速度为0.1rad/s
        max_speed = 0.1
        # 阻塞等待最大时间
        timeout_s = 20

        # 进行心跳动作
        demo_heart_pose(robot, joint_group_names, position_seq,
                        is_blocking, max_speed, timeout_s)

    except Exception as e:
        print(f"发生异常: {e}")
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
