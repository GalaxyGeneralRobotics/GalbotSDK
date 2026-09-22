import time
import galbot_sdk.g3 as gm
from galbot_sdk.g3 import GalbotMotion, GalbotRobot

# Get and initialize the GalbotMotion singleton
motion = GalbotMotion()
robot = GalbotRobot()

def printStatus(status):
        if(status == gm.MotionStatus.SUCCESS):
            print("Execution result: SUCCESS, execution successful")
        elif(status == gm.MotionStatus.TIMEOUT):
            print("Execution result: TIMEOUT, execution timed out")
        elif(status == gm.MotionStatus.FAULT):
            print("Execution result: FAULT, a fault occurred and execution cannot continue")
        elif(status == gm.MotionStatus.INVALID_INPUT):
            print("Execution result: INVALID_INPUT, input parameters do not meet requirements")
        elif(status == gm.MotionStatus.INIT_FAILED):
            print("Execution result: INIT_FAILED, failed to create internal communication components")
        elif(status == gm.MotionStatus.IN_PROGRESS):
            print("Execution result: IN_PROGRESS, in motion but not yet in position")
        elif(status == gm.MotionStatus.STOPPED_UNREACHED):
            print("Execution result: STOPPED_UNREACHED, stopped but target not reached")
        elif(status == gm.MotionStatus.DATA_FETCH_FAILED):
            print("Execution result: DATA_FETCH_FAILED, failed to fetch data")
        elif(status == gm.MotionStatus.PUBLISH_FAIL):
            print("Execution result: PUBLISH_FAIL, data transmission failed")
        elif(status == gm.MotionStatus.COMM_DISCONNECTED):
            print("Execution result: COMM_DISCONNECTED, connection failed")

if motion.init():
    print("GalbotMotion initialized successfully")
else:
    print("GalbotMotion initialization failed")
if robot.init():
    print("GalbotRobot initialized successfully")
else:
    print("GalbotRobot initialization failed")

# Program started, waiting for data
time.sleep(2)

try:
    chain_name = "left_arm"
    tool_name = "suction_cup"
    status = motion.attach_tool(
        chain=chain_name,
        tool=tool_name
    )
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "load failed"
    print(f"✅ Tool attached successfully")

    # Clean up the attached tool model to avoid affecting subsequent examples.
    status = motion.detach_tool(chain=chain_name)
    printStatus(status)
    assert status == gm.MotionStatus.SUCCESS, "detach failed"
    print(f"✅ Tool detached successfully")
except Exception as e:
    print(f"❌ Tool operation exception: {e}")

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
