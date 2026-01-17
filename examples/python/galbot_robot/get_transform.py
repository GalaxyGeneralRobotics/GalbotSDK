import time
from galbot_sdk.g1 import GalbotRobot

def print_pose(pose_vec):
    """
    pose_vec: list of floats
    """
    print("pose_vec = [" + ", ".join(str(p) for p in pose_vec) + "]")

# 获取 GalbotRobot 的单例并初始化
robot = GalbotRobot.get_instance()
robot.init()

# 程序立即启动，稍等数据就绪时间
time.sleep(1)
print("初始化成功")

# 设置目标帧和源帧
target_frame = "base_link"
source_frame = "left_arm_link1"
timestamp_ns = 0    # 0为获取最新tf变换值

# 获取坐标变换
ret_val = robot.get_transform(target_frame, source_frame)

if not ret_val[0]:
    print("get_transform error")
else:
    print("tf_timestamp_ns:", ret_val[1])
    print_pose(ret_val[0])

# 主动发出SIGINT退出信号
robot.request_shutdown()
# 等待进入shutdown状态
robot.wait_for_shutdown()
# 进行SDK资源释放
robot.destroy()
print('资源释放成功')