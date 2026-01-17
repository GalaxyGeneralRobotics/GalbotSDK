import time
from galbot_sdk.g1 import GalbotRobot, JointGroup

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

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init()

# 程序立即启动，稍等数据就绪时间
time.sleep(1)
print("初始化成功")

# 设置夹爪关节组（左夹爪）
joint_group = JointGroup.LEFT_GRIPPER

# 获取夹爪状态
gripper_state = robot.get_gripper_state(joint_group)

if gripper_state is None:
    print("get gripper state error")
else:
    print("左夹爪状态如下：")
    print_gripper_state(joint_group, gripper_state)

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')