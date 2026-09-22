"""
WBC end-effector command example

Demonstrates the whole-body control end-effector trajectory API:
- set_end_effector_command: publish pose targets (each pose is [x,y,z,qx,qy,qz,qw])
- clear_end_effector_command: clear end-effector task commands on the WBC channel

reference_frames: omit or [] to use ``world`` for every pose; otherwise one string per pose,
typically ``world`.
"""

import sys
import os
import time

# Add pybind library path to PYTHONPATH
script_path = os.path.abspath(__file__)
sdk_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_path))))

# Add package path (for galbot_sdk module)
package_path = os.path.join(sdk_root, "pybind", "package")
# Add lib path (for .so files)
lib_path = os.path.join(sdk_root, "pybind", "output", "linux-x86_64-gcc940", "lib")

for p in [package_path, lib_path]:
    if p not in sys.path:
        sys.path.insert(0, p)

print(f"Added paths: {package_path}, {lib_path}")

from galbot_sdk.g1 import GalbotRobot


def main():
    # Get GalbotRobot singleton and initialize
    robot = GalbotRobot()
    robot.init()
    print("Initialization successful")

    time.sleep(2)

    ee_info = robot.get_wbc_end_effector_poses()# Before using this function, ensure that `using_wbc` in the configuration file is set to `true`.
    print("\nCurrent WBC end-effector poses:")
    for frame, pose in ee_info.items():
        print(f"  Frame: {frame}, Pose: {pose}")

    ree_pose = ee_info.get("ree_pose")  # keys: "ree_pose", "lee_pose", "head_pose"
    if ree_pose and len(ree_pose) >= 7:
        target_pose = list(ree_pose[:7])
        target_pose[0] -= 0.1

        # Example: absolute pose — omit reference_frames to default every pose to "world"
        robot.set_end_effector_command(
            poses=[target_pose],
            end_effector_frames=["right_arm_end_effector_mount_link"],
        )
        print('\nPublished end-effector command (default reference_frames → world)')

    time.sleep(3)

    robot.clear_end_effector_command()  # clear and return to default reference joint pose
    # robot.clear_end_effector_command(hold=True)   # clear and hold current joint pose
    
    # Send shutdown signal
    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()
    print("Resource cleanup successful")


if __name__ == "__main__":
    main()
