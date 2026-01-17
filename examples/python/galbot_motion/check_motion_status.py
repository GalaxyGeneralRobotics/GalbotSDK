from galbot_sdk.g1 import MotionStatus, check_motion_status

status_str = check_motion_status(MotionStatus.SUCCESS)
print('MotionStatus 字符串:', status_str)