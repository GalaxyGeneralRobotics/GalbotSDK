from galbot_sdk.g1 import Parameter, create_parameter, G1JointGroup

# Create Parameter via constructor and set options
p = Parameter()
p.set_blocking(True)
p.set_check_collision(False)
p.set_timeout(5.0)
p.set_actuate('with_chain_only')
p.set_tool_pose(False)
p.set_reference_frame('base_link')

p.joint_state = {
    G1JointGroup.left_arm: [0.0] * 7,
    # Can add others if needed:
    # G1JointGroup.right_arm: [0.0] * 7,
    # G1JointGroup.leg: [0.0] * 4,
}

print('blocking:', p.get_blocking())
print('collision check:', p.get_check_collision())
print('timeout:', p.get_timeout())

# Or use factory function to quickly create Parameter
p2 = create_parameter(direct_execute=False, blocking=True, timeout=3.0, actuate='with_chain_only', tool_pose=False, check_collision=True)
print('Factory-created timeout:', p2.get_timeout())