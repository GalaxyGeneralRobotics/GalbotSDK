from galbot_sdk.g3 import GalbotNavigation
from galbot_sdk.g3 import GalbotRobot


def print_boxes(boxes):
    print(f"Current bounding boxes: {len(boxes)}")
    for index, box in enumerate(boxes):
        print(f"--- Box [{index}] ---")
        print("Tag:", box["box_tag"])
        print("Parent:", box["parent_link_name"])
        print("Size:", box["box_size"])
        print("Pose:", box["box_pose"])


robot = GalbotRobot()
robot.init()

nav = GalbotNavigation()
nav.init()

# Add a box so navigation does not treat the corresponding fused points as obstacles.
box_info = {
    "box_size": [0.4, 0.3, 0.28],
    "box_pose": [0.45, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0],
    "box_tag": 1001,
    "parent_link_name": "base_link",
}
attached_box_info = {
    "box_size": [0.4, 0.3, 0.28],
    "box_pose": [0.2, 0.0, -0.2, 0.0, 0.0, 0.0, 1.0],
    "box_tag": 2001,
    "parent_link_name": "left_arm_end_effector_mount_link",
}

success, status = nav.add_bounding_box(box_info)
print("add_bounding_box:", success, status)

print_boxes(nav.get_bounding_box())

success, status = nav.attach_box_to_link(attached_box_info, [])
print("attach_box_to_link:", success, status)

success, status = nav.detach_box_from_link(2001)
print("detach_box_from_link:", success, status)

success, status = nav.remove_bounding_box(1001)
print("remove_bounding_box:", success, status)

print_boxes(nav.get_bounding_box())

robot.request_shutdown()
robot.wait_for_shutdown()
robot.destroy()
print("Resources released successfully")
