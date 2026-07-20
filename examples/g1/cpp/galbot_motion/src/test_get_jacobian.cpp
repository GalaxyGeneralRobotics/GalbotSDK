#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "galbot_motion.hpp"
#include "galbot_robot.hpp"

using namespace galbot::sdk;

static int g_test_count = 0;
static int g_pass_count = 0;
static int g_fail_count = 0;

void print_jacobian(const std::string& label,
                    const std::tuple<MotionStatus, std::vector<std::vector<double>>>& res,
                    GalbotMotion& planner) {
    auto status = std::get<0>(res);
    const auto& matrix = std::get<1>(res);

    std::cout << "[" << label << "] Status: " << planner.status_to_string(status) << std::endl;

    if (status == MotionStatus::SUCCESS && !matrix.empty()) {
        int rows = static_cast<int>(matrix.size());
        int cols = static_cast<int>(matrix[0].size());
        std::cout << "Jacobian matrix (" << rows << "x" << cols << "):" << std::endl;
        for (int r = 0; r < rows; ++r) {
            std::cout << "  row " << r << ": ";
            for (int c = 0; c < cols; ++c) {
                printf("%10.5f ", matrix[r][c]);
            }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;
}

void run_test(const std::string& name, MotionStatus expected,
              const std::tuple<MotionStatus, std::vector<std::vector<double>>>& res,
              GalbotMotion& planner) {
    g_test_count++;
    auto status = std::get<0>(res);
    const auto& matrix = std::get<1>(res);

    bool ok = (status == expected);
    bool shape_ok = true;
    if (ok && status == MotionStatus::SUCCESS) {
        shape_ok = (!matrix.empty() && matrix[0].size() > 0);
        // Jacobian should be 6 rows (vx,vy,vz,wx,wy,wz)
        if (shape_ok) {
            shape_ok = (matrix.size() == 6);
        }
    }

    if (ok && shape_ok) {
        g_pass_count++;
        std::cout << "[PASS] " << name << std::endl;
    } else {
        g_fail_count++;
        std::cout << "[FAIL] " << name;
        if (!ok) {
            std::cout << " (expected: " << planner.status_to_string(expected)
                      << ", got: " << planner.status_to_string(status) << ")";
        }
        if (!shape_ok && status == MotionStatus::SUCCESS) {
            std::cout << " (unexpected matrix shape)";
        }
        std::cout << std::endl;
    }
    print_jacobian(name, res, planner);
}

int main() {
    auto& planner = GalbotMotion::get_instance(MachineType::G1);
    auto& robot = GalbotRobot::get_instance(MachineType::G1);

    if (!planner.init()) {
        std::cerr << "Planner initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "Planner initialized successfully!" << std::endl;

    if (!robot.init()) {
        std::cerr << "Robot initialization failed!" << std::endl;
        return -1;
    }
    std::cout << "Robot initialized successfully!" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // --- Print supported links (valid target_frame link names for get_jacobian) ---
    std::cout << "\n=== Supported links (valid target_frame for get_jacobian) ===" << std::endl;
    {
        const auto& support_links = planner.get_support_links();
        std::cout << "get_support_links() returned " << support_links.size() << " links:" << std::endl;
        for (const auto& link : support_links) {
            std::cout << "  - " << link << std::endl;
        }

        const auto ee_links = planner.get_link_names(true);
        std::cout << "get_link_names(only_end_effector=true) returned " << ee_links.size()
                  << " end-effector links:" << std::endl;
        for (const auto& link : ee_links) {
            std::cout << "  - " << link << std::endl;
        }
        std::cout << std::endl;
    }

    // Joint configurations for testing
    std::vector<double> left_arm_joints = {1.9999, -1.6000, -0.5999, -1.6999, 0.0000, -0.7999, 0.0000};
    std::vector<double> right_arm_joints = {-2.0000, 1.6001, 0.6001, 1.7000, 0.0000, 0.8000, 0.0000};

    // --- Test 1: Default parameters (current robot state) ---
    try {
        std::cout << "=== Test 1: get_jacobian with default parameters ===" << std::endl;
        auto res = planner.get_jacobian("left_arm");
        run_test("left_arm default state", MotionStatus::SUCCESS, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 1 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 2: EndEffector frame, base_link reference ---
    try {
        std::cout << "=== Test 2: EndEffector frame / base_link reference ===" << std::endl;
        auto res = planner.get_jacobian("left_arm", "EndEffector", "base_link");
        run_test("left_arm EndEffector/base_link", MotionStatus::SUCCESS, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 2 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 3: Tool frame, world reference (NOT supported -> INVALID_INPUT) ---
    try {
        std::cout << "=== Test 3: Tool frame / world reference (expected INVALID_INPUT) ===" << std::endl;
        auto res = planner.get_jacobian("left_arm", "Tool", "world");
        run_test("left_arm Tool/world", MotionStatus::INVALID_INPUT, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 3 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 4: Custom joint state ---
    try {
        std::cout << "=== Test 4: Custom joint state ===" << std::endl;
        std::unordered_map<std::string, std::vector<double>> custom_joint_state = {{"left_arm", left_arm_joints}};
        auto custom_param_ptr = std::make_shared<Parameter>();
        custom_param_ptr->set_timeout(5.0);
        auto res = planner.get_jacobian("left_arm", "EndEffector", "base_link", custom_joint_state, custom_param_ptr);
        run_test("left_arm custom joint state", MotionStatus::SUCCESS, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 4 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 5: Right arm, default state ---
    try {
        std::cout << "=== Test 5: Right arm default state ===" << std::endl;
        auto res = planner.get_jacobian("right_arm");
        run_test("right_arm default state", MotionStatus::SUCCESS, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 5 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 6: Right arm with custom joint state ---
    try {
        std::cout << "=== Test 6: Right arm custom joint state ===" << std::endl;
        std::unordered_map<std::string, std::vector<double>> custom_joint_state = {{"right_arm", right_arm_joints}};
        auto res = planner.get_jacobian("right_arm", "EndEffector", "base_link", custom_joint_state);
        run_test("right_arm custom joint state", MotionStatus::SUCCESS, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 6 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 7: Invalid chain name ---
    try {
        std::cout << "=== Test 7: Invalid chain name ===" << std::endl;
        auto res = planner.get_jacobian("invalid_chain");
        run_test("invalid_chain", MotionStatus::INVALID_INPUT, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 7 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 8: Left arm, EndEffector frame, world reference ---
    try {
        std::cout << "=== Test 8: EndEffector / world reference ===" << std::endl;
        auto res = planner.get_jacobian("left_arm", "EndEffector", "world");
        run_test("left_arm EndEffector/world", MotionStatus::SUCCESS, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 8 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 9: Left arm, Tool frame, base_link reference (NOT supported -> INVALID_INPUT) ---
    try {
        std::cout << "=== Test 9: Tool frame / base_link reference (expected INVALID_INPUT) ===" << std::endl;
        auto res = planner.get_jacobian("left_arm", "Tool", "base_link");
        run_test("left_arm Tool/base_link", MotionStatus::INVALID_INPUT, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 9 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 10: Both arms sequentially ---
    try {
        std::cout << "=== Test 10: Both arms ===" << std::endl;
        std::unordered_map<std::string, std::vector<double>> both_arms_joint;
        both_arms_joint["left_arm"] = left_arm_joints;
        both_arms_joint["right_arm"] = right_arm_joints;

        auto res_left = planner.get_jacobian("left_arm", "EndEffector", "base_link", both_arms_joint);
        run_test("both_arms left", MotionStatus::SUCCESS, res_left, planner);

        auto res_right = planner.get_jacobian("right_arm", "EndEffector", "base_link", both_arms_joint);
        run_test("both_arms right", MotionStatus::SUCCESS, res_right, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 10 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 11: Empty chain name ---
    try {
        std::cout << "=== Test 11: Empty chain name ===" << std::endl;
        auto res = planner.get_jacobian("");
        run_test("empty_chain", MotionStatus::INVALID_INPUT, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 11 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 12: All zeros joint configuration ---
    try {
        std::cout << "=== Test 12: All zeros joint configuration ===" << std::endl;
        std::vector<double> zero_joints(7, 0.0);
        std::unordered_map<std::string, std::vector<double>> zero_joint_state = {{"left_arm", zero_joints}};
        auto res = planner.get_jacobian("left_arm", "EndEffector", "base_link", zero_joint_state);
        run_test("left_arm zero joints", MotionStatus::SUCCESS, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 12 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 13: torso chain (torso shares the leg end-effector: leg_end_effector_mount_link) ---
    try {
        std::cout << "=== Test 13: torso chain / EndEffector / base_link ===" << std::endl;
        auto res = planner.get_jacobian("torso", "EndEffector", "base_link");
        run_test("torso EndEffector/base_link", MotionStatus::SUCCESS, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 13 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // --- Test 14: leg chain with joint values (EndEffector maps to leg_end_effector_mount_link;
    //     G1 leg DOF = 5, so pass five joint values) ---
    try {
        std::cout << "=== Test 14: leg chain / EndEffector / base_link (5 joints) ===" << std::endl;
        std::unordered_map<std::string, std::vector<double>> leg_joint_state = {
            {"leg", {0.0, 0.0, 0.0, 0.0, 0.0}}};
        auto res = planner.get_jacobian("leg", "EndEffector", "base_link", leg_joint_state);
        run_test("leg EndEffector/base_link", MotionStatus::SUCCESS, res, planner);
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Test 14 exception: " << e.what() << std::endl;
        g_test_count++;
        g_fail_count++;
    }

    // Summary
    std::cout << "\n========================================" << std::endl;
    std::cout << "Jacobian test summary: " << g_test_count << " tests, "
              << g_pass_count << " passed, " << g_fail_count << " failed" << std::endl;
    std::cout << "========================================\n" << std::endl;

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return (g_fail_count == 0) ? 0 : 1;
}
