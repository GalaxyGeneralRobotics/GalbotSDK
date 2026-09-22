#include<iostream>
#include<fstream>
#include<thread>
#include<chrono>
#include <array>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include "opencv2/opencv.hpp"

#include "galbot_robot.hpp"
#include "galbot_motion.hpp"


using namespace galbot::sdk;

inline std::vector<double> quat_normalize(const std::vector<double>& q)
{
    double norm = std::sqrt(
        q[0]*q[0] + q[1]*q[1] +
        q[2]*q[2] + q[3]*q[3]
    );
    if (norm <= 0.0)
        throw std::runtime_error("Zero-norm quaternion");

    return std::vector<double>{ q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm };
}
inline std::vector<double> quat_conjugate(const std::vector<double>& q)
{
    return std::vector<double>{ -q[0], -q[1], -q[2], q[3] };
}

inline std::vector<double> quat_multiply(const std::vector<double>& q1, const std::vector<double>& q2)
{
    const double x1 = q1[0];
    const double y1 = q1[1];
    const double z1 = q1[2];
    const double w1 = q1[3];
    const double x2 = q2[0];
    const double y2 = q2[1];
    const double z2 = q2[2];
    const double w2 = q2[3];

    return std::vector<double>{
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    };
}

template <typename T>
inline T clamp(const T& v, const T& lo, const T& hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

inline double orientation_error_angle(const std::vector<double>& A, const std::vector<double>& B)
{
    std::vector<double> qA = quat_normalize({ A[3], A[4], A[5], A[6] });
    std::vector<double> qB = quat_normalize({ B[3], B[4], B[5], B[6] });

    std::vector<double> q_err = quat_multiply(qB, quat_conjugate(qA));
    q_err = quat_normalize(q_err);

    // Numerically stable
    double qw = clamp(q_err[3], -1.0, 1.0);

    return 2.0 * std::acos(qw);
}

struct PoseError
{
    double position_error_norm;   // meters
    double orientation_error_rad;
    double orientation_error_deg;
};

inline PoseError calculate_error(const std::vector<double>& pose1, const std::vector<double>& pose2)
{
    double dx = pose1[0] - pose2[0];
    double dy = pose1[1] - pose2[1];
    double dz = pose1[2] - pose2[2];

    double pos_err = std::sqrt(dx*dx + dy*dy + dz*dz);
    double rot_err = orientation_error_angle(pose1, pose2);

    return PoseError{
        pos_err,
        rot_err,
        rot_err * 180.0 / M_PI
    };
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
        auto& motion = GalbotMotion::get_instance(MachineType::G3); 

        /* Initialize robot */
        if (robot.init()) {
            std::cout << "Initialization successful" << std::endl;
            std::cout << "Is robot running: " << robot.is_running() << std::endl;
        }else{
            std::cerr << "Initialization failed" << std::endl;
        }
        
        /** Initialize motion */
        if (motion.init()) {
            std::cout << "Motion initialization successful" << std::endl;
        }else{
            std::cerr << "Motion initialization failed" << std::endl;
        }

        /* Wait for data preparation */
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        // set initial joint positions
        std::vector<std::string> joint_groups = {"leg", "head", "left_arm", "right_arm"};
        std::vector<std::string> joint_names = {};

        std::vector<double> joint_pos = {0.5, 1.5, 1.0, 0.0, 0.0, 
            0.0, 0.0,
            1.2, -1.0, -0.2, 1.4, 0.1000, -0.3, 0.0000,
            -1.2, 1.0, 0.2, -1.4, -0.1000, 0.3, 0.0000};
        bool is_block = true;
        double max_speed_rad_s = 0.1;
        double timeout_s = 30.0;

        galbot::sdk::ControlStatus joint_execution_status =
            robot.set_joint_positions(joint_pos, joint_groups, joint_names, is_block, max_speed_rad_s, timeout_s);

        if (joint_execution_status == ControlStatus::SUCCESS) {
            std::cout << "✅ Initial joint positions set successfully!" << std::endl;
        } else {
            std::cerr << "Failed to set initial joint positions!" << std::endl;
        }
        
        /** Define target pose */
        std::map<std::string, std::vector<double>> chain_pose_baselink = {
            {"leg", {0.0596,-0.0000,1.0327,0.5000,0.5003,0.4997,0.5000}},
            {"head", {0.0599,0.0002,1.4098,-0.7072,0.0037,0.0037,0.7069}},
            {"left_arm", {0.5, 0.3, 1.0, 0.31, 0.05, -0.22, 0.92}},
            {"right_arm", {0.5, -0.3, 1.0, -0.31, 0.05, -0.22, 0.92}}
        };

        std::string target_chain = "left_arm";
        std::string reference_frame = "base_link";
        std::string target_frame = "EndEffector";
        std::string end_link = "left_arm_end_effector_mount_link";

        /** 1. Get end effector pose on chain */
        std::tuple<MotionStatus, std::vector<double>> ret = motion.get_end_effector_pose_on_chain(
            target_chain, target_frame, reference_frame
        );
        if(std::get<0>(ret) == MotionStatus::SUCCESS){
            std::cout << "✅ Current " << target_chain << "end-effector pose: " << std::get<1>(ret).size() << std::endl;
            for (auto pose : std::get<1>(ret)){
                std::cout << pose << " ";
            }
            std::cout << std::endl;
        }else{
            std::cerr << "Failed to get end effector pose on chain!" << std::endl;
        }

        /** 2. Solve joint angles based on target pose IK and verify the solution 
         *  2.1 Solve joint angles joint_angles_ik for target pose through IK
        */
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        bool enable_collision_check = false;
        std::tuple<MotionStatus, std::unordered_map<std::string, std::vector<double>>> ret_ik = motion.inverse_kinematics(
            chain_pose_baselink[target_chain],
            {target_chain},
            target_frame, 
            reference_frame,
            {},
            enable_collision_check,
            std::make_shared<Parameter>()
        );
        if (std::get<0>(ret_ik) == MotionStatus::SUCCESS){
            std::cout << "✅ Target" << target_chain << "IK solving successful joint_angles_ik:" << std::endl;
            for (auto joint_angle : std::get<1>(ret_ik)[target_chain]){
                std::cout << joint_angle << " ";
            }
            std::cout << std::endl;
        }else{
            std::cerr << "IK solving failed" << std::endl;
        }

        /** 2.2 Set end-effector pose to target pose tgt_pose_ik by setting 
         * joint group angles joint_angles_ik 
        */
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ControlStatus ret2 = robot.set_joint_positions(
            std::get<1>(ret_ik)[target_chain],
            {target_chain},
            {},
            true,
            0.1,
            20.0
        );
        if(ret2 == ControlStatus::SUCCESS){
            std::cout << "✅ Target " << target_chain << "pose setting successful" << std::endl;
        }else{
            std::cerr << "Target " << target_chain << "pose setting failed" << std::endl;
        }

        /** 2.3 Verify whether the set joint group angles are consistent with the solved angles */
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::tuple<MotionStatus, std::vector<double>> ret3 = motion.get_end_effector_pose_on_chain(
            target_chain,
            target_frame,
            reference_frame
        );
        if(std::get<0>(ret3) == MotionStatus::SUCCESS){
            std::cout << "✅ Verified " << target_chain << "end-effector pose: " << std::get<1>(ret3).size() << std::endl;
            for (auto pose : std::get<1>(ret3)){
                std::cout << pose << " ";
            }
            std::cout << std::endl;
        }else{
            std::cerr << "Failed to verify end effector pose on chain!" << std::endl;
        }
        
        /** 2.4 Verify whether the end-effector pose tgt_pose_fk corresponding to joint group angles joint_angles_ik solved by FK is consistent with target pose tgt_pose_ik */
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::tuple<MotionStatus, std::vector<double>> ret_fk = motion.forward_kinematics(
            end_link,
            reference_frame,
            std::get<1>(ret_ik),
            std::make_shared<Parameter>()
        );
        if (std::get<0>(ret_fk) == MotionStatus::SUCCESS){
            std::cout << "✅ Verified " << target_chain << "end-effector pose: " << std::get<1>(ret_fk).size() << std::endl;
            for (auto pose : std::get<1>(ret_fk)){
                std::cout << pose << " ";
            }
            std::cout << std::endl;

            PoseError err = calculate_error(chain_pose_baselink[target_chain], std::get<1>(ret_fk));
            std::cout << "Position error: " << err.position_error_norm << " m" << std::endl;
            std::cout << "Orientation error: " << err.orientation_error_deg << " deg" << std::endl;
        }else{
            std::cerr << "Failed to verify end effector pose on chain!" << std::endl;
        }

        /** 3. Restore to original pose by setting end-effector pose
         *  3.1 Set end-effector pose to restore to original pose
         */
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        MotionStatus ret4 = motion.set_end_effector_pose(
            std::get<1>(ret),
            target_chain,
            reference_frame,
            nullptr,
            enable_collision_check,
            true,
            5.0,
            std::make_shared<Parameter>()
        );
        if(ret4 == MotionStatus::SUCCESS){
            std::cout << "✅ Restored " << target_chain << "end-effector pose to original pose" << std::endl;
        }else{
            std::cerr << "Failed to restore " << target_chain << "end-effector pose to original pose" << std::endl;
        }

        /** 3.2 Get end-effector pose and verify whether it has been restored to original pose */
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::tuple<MotionStatus, std::vector<double>> ret5 = motion.get_end_effector_pose_on_chain(
            target_chain,
            target_frame,
            reference_frame
        );
        if(std::get<0>(ret5) == MotionStatus::SUCCESS){
            std::cout << "✅ Verified " << target_chain << "end-effector pose: " << std::get<1>(ret5).size() << std::endl;
            for (auto pose : std::get<1>(ret5)){
                std::cout << pose << " ";
            }
            std::cout << std::endl;

            PoseError err = calculate_error(std::get<1>(ret), std::get<1>(ret5));
            std::cout << "Position error: " << err.position_error_norm << " m" << std::endl;
            std::cout << "Orientation error: " << err.orientation_error_deg << " deg" << std::endl;
        }else{
            std::cerr << "Failed to verify end effector pose on chain!" << std::endl;
        }

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
