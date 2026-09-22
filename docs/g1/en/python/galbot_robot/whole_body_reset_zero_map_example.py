from galbot_sdk.g1 import GalbotRobot
from galbot_sdk.g1 import GalbotMotion
import time


def main():
    robot = GalbotRobot()
    motion = GalbotMotion()

    if not robot.init():
        print("GalbotRobot init failed.")
        return
    if not motion.init():
        print("GalbotMotion init failed.")
        return

    time.sleep(2)

    # Whole-body joints: leg(5) + head(2) + left_arm(7) + right_arm(7)
    whole_body_joint_1 = [
        0.25, 1.1, 0.85, 0.0, 0.0,       # leg
        0.5, 0.5,                        # head
        2.0, -1.55, -0.55, -1.7, -0.0, -0.8, 0.2,   # left_arm
        -2.0, 1.55, 0.55, 1.7, 0.0, 0.8, 0.2        # right_arm
    ]

    # Base pose command map(x, y, yaw) Note: adjust based on the robot's actual localization in the map frame
    base_x_1 = -0.4
    base_y_1 = 0.226593
    base_yaw_1 = 0.0

    # Optional frames (frame_id: base_link/odom/map, reference_frame_id: odom/map)
    frame_id = "base_link"
    reference_frame_id = "map"

    time.sleep(1)

    # reset to zero
    result = robot.zero_whole_body_and_base(
        frame_id,
        reference_frame_id,
        True,
        0.2,
        15.0,
    )
    print("Zero joint status:", result[0])
    print("Zero base status:", result[1])

    robot.request_shutdown()
    robot.wait_for_shutdown()
    robot.destroy()


if __name__ == "__main__":
    main()
