from galbot_sdk.g1 import Parameter, create_parameter

# 通过构造函数创建 Parameter 并设置选项
p = Parameter()
p.set_blocking(True)
p.set_check_collision(False)
p.set_timeout(5.0)
p.set_actuate('with_chain_only')
p.set_tool_pose(False)
p.set_reference_frame('base_link')
p.set_joint_state('left_arm', [0.0] * 7)

print('blocking:', p.get_blocking())
print('collision check:', p.get_check_collision())
print('timeout:', p.get_timeout())

# 或者使用工厂函数快速创建 Parameter
p2 = create_parameter(direct_execute=False, blocking=True, timeout=3.0, actuate='with_chain_only', tool_pose=False, check_collision=True)
print('工厂创建的 timeout:', p2.get_timeout())