/**
 * example: add/get/remove navigation bounding boxes and attach/detach box collision objects.
 *
 * These boxes are used by navigation fusion to ignore corresponding box regions,
 * so navigation will not treat the carried/known boxes as obstacles.
 */
#include "galbot_navigation.hpp"
#include "galbot_robot.hpp"

#include <iostream>
#include <vector>

using namespace galbot::sdk;

void print_box_info(const BoxInfo& box_info) {
    std::cout << "box_tag: " << box_info.box_tag << std::endl;
    std::cout << "parent_link_name: " << box_info.parent_link_name << std::endl;
    std::cout << "box_size: [" << box_info.box_size.length_x << ", "
              << box_info.box_size.length_y << ", " << box_info.box_size.length_z << "]" << std::endl;
    std::cout << "box_pose: [" << box_info.box_pose.position.x << ", "
              << box_info.box_pose.position.y << ", " << box_info.box_pose.position.z << ", "
              << box_info.box_pose.orientation.x << ", " << box_info.box_pose.orientation.y << ", "
              << box_info.box_pose.orientation.z << ", " << box_info.box_pose.orientation.w << "]" << std::endl;
}

BoxInfo make_box_info(int box_tag, const BoxSize& box_size, const Pose& box_pose,
                      const std::string& parent_link_name) {
    BoxInfo box_info;
    box_info.box_tag = box_tag;
    box_info.box_size = box_size;
    box_info.box_pose = box_pose;
    box_info.parent_link_name = parent_link_name;
    return box_info;
}

int main() {
    auto& navigation = GalbotNavigation::get_instance(MachineType::G3);
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    if (!robot.init()) {
        std::cerr << "Base instance initialization failed!" << std::endl;
        return -1;
    }
    if (!navigation.init()) {
        std::cerr << "Navigation instance initialization failed!" << std::endl;
        return -1;
    }

    BoxSize small_box_size{0.4f, 0.3f, 0.28f};
    Pose small_box_pose(std::vector<double>{0.45, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0});
    Pose attached_box_pose(std::vector<double>{0.2, 0.0, -0.2, 0.0, 0.0, 0.0, 1.0});
    BoxInfo small_box = make_box_info(1001, small_box_size, small_box_pose, "base_link");
    BoxInfo attached_box = make_box_info(2001, small_box_size, attached_box_pose, "left_arm_end_effector_mount_link");

    std::cout << "Add navigation bounding box filters" << std::endl;
    auto add_status = navigation.add_bounding_box(small_box);
    std::cout << "add_bounding_box result: " << static_cast<int>(add_status) << std::endl;

    std::cout << "Get navigation bounding box filters" << std::endl;
    auto boxes = navigation.get_bounding_box();
    std::cout << "box count: " << boxes.size() << std::endl;
    for (const auto& box : boxes) {
        print_box_info(box);
    }

    std::cout << "Attach one box collision object to a robot link" << std::endl;
    std::vector<std::string> ignore_collision_links;
    auto attach_status = navigation.attach_box_to_link(attached_box, ignore_collision_links);
    std::cout << "attach_box_to_link result: " << static_cast<int>(attach_status) << std::endl;

    std::cout << "Detach one box collision object from its robot link" << std::endl;
    auto detach_status = navigation.detach_box_from_link(2001);
    std::cout << "detach_box_from_link result: " << static_cast<int>(detach_status) << std::endl;

    std::cout << "Remove one navigation bounding box filter" << std::endl;
    auto remove_status = navigation.remove_bounding_box(1001);
    std::cout << "remove_bounding_box result: " << static_cast<int>(remove_status) << std::endl;

    boxes = navigation.get_bounding_box();
    std::cout << "box count after remove: " << boxes.size() << std::endl;
    for (const auto& box : boxes) {
        print_box_info(box);
    }

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    std::cout << "Resources released successfully" << std::endl;
    return 0;
}
