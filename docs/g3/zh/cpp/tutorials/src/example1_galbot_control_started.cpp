#include <chrono>
#include <cctype>
#include <iostream>
#include <thread>
#include <vector>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

void print_list(std::ostream& output, const std::vector<std::string>& values) {
    output << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            output << ", ";
        }
        output << "'" << values[i] << "'";
    }
    output << "]";
}

void print_list(std::ostream& output, const std::vector<double>& values) {
    output << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            output << ", ";
        }
        output << values[i];
    }
    output << "]";
}

const char* control_status_to_string(ControlStatus status) {
    switch (status) {
        case ControlStatus::SUCCESS: return "SUCCESS";
        case ControlStatus::TIMEOUT: return "TIMEOUT";
        case ControlStatus::FAULT: return "FAULT";
        case ControlStatus::INVALID_INPUT: return "INVALID_INPUT";
        case ControlStatus::INIT_FAILED: return "INIT_FAILED";
        case ControlStatus::IN_PROGRESS: return "IN_PROGRESS";
        case ControlStatus::STOPPED_UNREACHED: return "STOPPED_UNREACHED";
        case ControlStatus::DATA_FETCH_FAILED: return "DATA_FETCH_FAILED";
        case ControlStatus::PUBLISH_FAIL: return "PUBLISH_FAIL";
        case ControlStatus::COMM_DISCONNECTED: return "COMM_DISCONNECTED";
        default: return "UNKNOWN";
    }
}

ControlStatus set_joint_positions_with_retry(
        GalbotRobot& robot, const std::vector<double>& positions,
        const std::vector<std::string>& joint_group_names, bool is_blocking,
        double max_speed, double timeout_s, int retry_count,
        std::chrono::seconds retry_delay) {
    ControlStatus status = robot.set_joint_positions(
        positions, joint_group_names, {}, is_blocking, max_speed, timeout_s);
    for (int retry_number = 1;
         status != ControlStatus::SUCCESS && retry_number <= retry_count;
         ++retry_number) {
        std::cerr << "Setting angles for joint group ";
        print_list(std::cerr, joint_group_names);
        std::cerr << " failed with " << control_status_to_string(status)
                  << ", retrying " << retry_number << "/" << retry_count
                  << "..." << std::endl;
        std::this_thread::sleep_for(retry_delay);
        status = robot.set_joint_positions(
            positions, joint_group_names, {}, is_blocking, max_speed, timeout_s);
    }
    return status;
}

void demo_heart_pose(GalbotRobot& robot,
                     const std::vector<std::string>& joint_group_names,
                     const std::vector<std::vector<double>>& position_seq,
                     bool is_blocking, double max_speed, double timeout_s){
    /** Get current joint group angles for subsequent restoration */
    std::vector<double> original_pos = robot.get_joint_positions(joint_group_names, {});
    std::cout << "Current angles of joint group ";
    print_list(std::cout, joint_group_names);
    std::cout << ": ";
    print_list(std::cout, original_pos);
    std::cout << std::endl;

    /** Execute heart pose sequence */
    int pos_idx = 0;
    std::cout << "Starting heart gesture..." << std::endl;
    while (pos_idx < position_seq.size()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::vector<double> pos = position_seq[pos_idx];
        ControlStatus control_status = set_joint_positions_with_retry(
            robot, pos, joint_group_names, is_blocking, max_speed, timeout_s,
            3, std::chrono::seconds(1));
        if (control_status == ControlStatus::SUCCESS) {
            std::cout << "Setting angles for joint group ";
            print_list(std::cout, joint_group_names);
            std::cout << " successful" << std::endl;
            pos_idx++;
        } else {
            std::cerr << "Setting angles for joint group ";
            print_list(std::cerr, joint_group_names);
            std::cerr << " failed: " << control_status_to_string(control_status)
                      << std::endl;
            break;
        }
    }

    /** Restore original joint positions */
    std::cout << "Showing heart gesture for 15 seconds, then restoring original pose..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(15));
    ControlStatus restore_status = set_joint_positions_with_retry(
        robot, original_pos, joint_group_names, is_blocking, max_speed, timeout_s,
        3, std::chrono::seconds(2));
    if (restore_status == ControlStatus::SUCCESS) {
        std::cout << "Restoring angles for joint group ";
        print_list(std::cout, joint_group_names);
        std::cout << " successful" << std::endl;
    } else {
        std::cerr << "Restoring angles for joint group ";
        print_list(std::cerr, joint_group_names);
        std::cerr << " failed: " << control_status_to_string(restore_status)
                  << std::endl;
    }
    
}

/* @brief Check if the robot is safe
*/
void check_robot_safety(){
    std::cout << "⚠️  Note: 1. Please ensure the robot's emergency stop button is released; 2. Please ensure there are no obstacles in front, back, left, and right of the robot to avoid unexpected situations. \n" << std::endl;

    char key;
    for(;;){
        std::cout << "Please confirm that the robot's emergency stop button is released and there are no obstacles. Continue? (y/n)...";
        std::cin >> key;

        if(std::tolower(key) == 'y'){
            std::cout << "User confirmed, continuing execution...\n" << std::endl;
            break;
        }else if(std::tolower(key) == 'n'){
            std::cout << "User not confirmed, program exiting...\n" << std::endl;
            exit(0);
        }else{
            std::cout << "Input error, please enter 'y' or 'n'\n" << std::endl;
        }
    }
}

int main(){
    check_robot_safety();
    try{
        /* Get robot instance  */
        auto& robot = GalbotRobot::get_instance(MachineType::G3); 

        /* Initialize robot */
        if (robot.init()) {
            std::cout << "Initialization successful" << std::endl;
            std::cout << "Is robot running: "
                      << (robot.is_running() ? "True" : "False") << std::endl;
        }else{
            std::cerr << "Initialization failed" << std::endl;
            robot.destroy();
            return 1;
        }

        /* Wait for data preparation */
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        
        /** Get joint names */
        std::vector<std::string> joint_names = robot.get_joint_names();
        if (joint_names.empty()) {
            std::cerr << "Failed to get list of joint names" << std::endl;
        } else {
            std::cout << "List of joint names: ";
            print_list(std::cout, joint_names);
            std::cout << std::endl;
        }

        /** Get joint positions using joint group names, empty returns all joints by default */
        std::vector<std::string> joint_group_names = {"left_arm", "right_arm"};
        std::vector<std::vector<double>> position_seq = {{
             1.0,  0.55, -2.0,  1.5, 0.0, -0.7, 0.0,  // left_arm
            -1.0, -0.55,  2.0, -1.5, 0.0,  0.7, 0.0   // right_arm
        }};
        bool is_blocking = true;
        double max_speed = 0.1;
        double timeout_s = 30;

        demo_heart_pose(robot, joint_group_names, position_seq,
                        is_blocking, max_speed, timeout_s);


        /** Actively send SIGINT exit signal to the robot */
        robot.request_shutdown();
        /** Wait to enter shutdown state */
        robot.wait_for_shutdown();
        /** Release SDK resources */
        robot.destroy();
        std::cout << "Resource release successful" << std::endl;    
    }catch(const std::exception& e){
        std::cout << "Error: " << e.what() << std::endl;
    }

    return 0;
}
