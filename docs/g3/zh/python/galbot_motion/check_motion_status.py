from galbot_sdk.g3 import MotionStatus, check_motion_status

status_str = check_motion_status(MotionStatus.SUCCESS)
print('MotionStatus string:', status_str)