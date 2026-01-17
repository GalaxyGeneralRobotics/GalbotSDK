import galbot_sdk.g1 as gm
from galbot_sdk.g1 import GalbotMotion, GalbotRobot, GalbotNavigation   
import time

# NOTE:
# - GalbotMotion currently does NOT provide real-time obstacle perception / automatic environment updates.
# - Collision checking uses self-collision + the collision world built from objects you load manually via
#   add_obstacle()/attach_target_object() (including point clouds if you load them explicitly).

motion = GalbotMotion.get_instance()
robot = GalbotRobot.get_instance()
nav = GalbotNavigation.get_instance()

def printStatus(status):
        if(status == gm.MotionStatus.SUCCESS):
            print("Result: SUCCESS")
        elif(status == gm.MotionStatus.TIMEOUT):
            print("Result: TIMEOUT")
        elif(status == gm.MotionStatus.FAULT):
            print("Result: FAULT")
        elif(status == gm.MotionStatus.INVALID_INPUT):
            print("Result: INVALID_INPUT")
        elif(status == gm.MotionStatus.INIT_FAILED):
            print("Result: INIT_FAILED")
        elif(status == gm.MotionStatus.IN_PROGRESS):
            print("Result: IN_PROGRESS")
        elif(status == gm.MotionStatus.STOPPED_UNREACHED):
            print("Result: STOPPED_UNREACHED")
        elif(status == gm.MotionStatus.DATA_FETCH_FAILED):
            print("Result: DATA_FETCH_FAILED")
        elif(status == gm.MotionStatus.PUBLISH_FAIL):
            print("Result: PUBLISH_FAIL")
        elif(status == gm.MotionStatus.COMM_DISCONNECTED):
            print("Result: COMM_DISCONNECTED")

if motion.init():
    print("GalbotMotion init OK")
else:
    print("GalbotMotion init FAILED")
if robot.init():
    print("GalbotRobot init OK")
else:
    print("GalbotRobot init FAILED")
if nav.init():
    print("GalbotNavigation init OK")
else:
    print("GalbotNavigation init FAILED")

# Wait for data to be ready.
time.sleep(3)

chain_joints = {
    "leg": [0.4992,1.4991,1.0005,0.0000,-0.0004],
    "head": [0.0000,0.0],
    "left_arm": [1.9999,-1.6000,-0.5999,-1.6999,0.0000,-0.7999,0.0000],
    "right_arm": [-2.0000,1.6001,0.6001,1.7000,0.0000,0.8000,0.0000]
}

whole_body_joint = [
    num for key in ["leg", "head", "left_arm", "right_arm"] 
    for num in chain_joints[key]
]
base_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
custom_param = gm.Parameter()

try:
    # Build RobotStates list to check.
    check_states = [gm.RobotStates() for _ in range(2)]
    check_states[0].whole_body_joint = whole_body_joint
    check_states[0].base_state = base_state

    bad_left_arm_joint = [1.99995,-1.60004,0.599905,-1.69994,0,-0.799924,0]

    check_left_arm = gm.JointStates()
    check_left_arm.chain_name = "left_arm"
    check_left_arm.joint_positions = bad_left_arm_joint
    check_states[1] = check_left_arm

    status, collision_res = motion.check_collision(
        start=check_states,
        enable_collision_check=True
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "Collision check failed"
    assert len(collision_res) == len(check_states), "Result size mismatch"
    print(f"OK: collision check finished: {collision_res} (False=no collision)")
except Exception as e:
    print(f"ERROR: collision check exception: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()