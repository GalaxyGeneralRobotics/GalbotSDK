from galbot_sdk.g3 import create_joint_state, create_pose_state, JointStates, PoseState, Pose

# Create helper objects using the factory function
js = create_joint_state()
ps = create_pose_state()

# Fill example fields
js.chain_name = 'left_arm'
js.joint_positions = [0.0] * 7

ps.chain_name = 'left_arm'
# 7D vector: x, y, z, qx, qy, qz, qw
ps.pose = Pose([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

print(type(js), js.chain_name)
print(type(ps), ps.chain_name)