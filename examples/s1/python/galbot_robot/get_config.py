import time
from galbot_sdk.s1 import ConfigService, ControlStatus, GalbotRobot

def print_fields(title, status, fields):
    print(f"[{title}] status: {status}")
    if status != ControlStatus.SUCCESS:
        print("部分字段读取失败，以下为成功读取的字段：")
    for field in fields:
        print(f"  {field.key} = {field.value}")

if __name__ == "__main__":
    robot = GalbotRobot()
    robot.init()
    time.sleep(2)
    status, fields = robot.get_config(ConfigService.CONTROL, ["report_error_skip", "report_sensor_skip"])
    print_fields("Control selected fields", status, fields)
    status, fields = robot.get_config(ConfigService.CONTROL, ["report_error_skip", "report_sensor_skip"], use_default=True)
    print_fields("Control built-in defaults", status, fields)
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
