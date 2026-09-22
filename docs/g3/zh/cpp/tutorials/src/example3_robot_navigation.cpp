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
#include "galbot_navigation.hpp"

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


/**
 * @brief Convert quaternion [x, y, z, w] to 3x3 rotation matrix
 * @param qx X component of quaternion
 * @param qy Y component of quaternion
 * @param qz Z component of quaternion
 * @param qw W component (scalar) of quaternion
 * @return 3x3 rotation matrix (CV_64F)
 */
inline cv::Mat quat_to_rotation_matrix(double qx, double qy, double qz, double qw) {
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    
    R.at<double>(0, 0) = 1 - 2*(qy*qy + qz*qz);
    R.at<double>(0, 1) = 2*(qx*qy - qw*qz);
    R.at<double>(0, 2) = 2*(qx*qz + qw*qy);
    
    R.at<double>(1, 0) = 2*(qx*qy + qw*qz);
    R.at<double>(1, 1) = 1 - 2*(qx*qx + qz*qz);
    R.at<double>(1, 2) = 2*(qy*qz - qw*qx);
    
    R.at<double>(2, 0) = 2*(qx*qz - qw*qy);
    R.at<double>(2, 1) = 2*(qy*qz + qw*qx);
    R.at<double>(2, 2) = 1 - 2*(qx*qx + qy*qy);
    
    return R;
}

/**
 * @brief Convert 3x3 rotation matrix to quaternion [x, y, z, w]
 * @param R 3x3 rotation matrix (CV_64F)
 * @return Quaternion as vector [qx, qy, qz, qw]
 */
inline std::vector<double> rotation_matrix_to_quat(const cv::Mat& R) {
    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
    double qx, qy, qz, qw;
    
    if (trace > 0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
        qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
        qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
    } else if ((R.at<double>(0, 0) > R.at<double>(1, 1)) && (R.at<double>(0, 0) > R.at<double>(2, 2))) {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2));
        qw = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
        qx = 0.25 * s;
        qy = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
        qz = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
    } else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2));
        qw = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
        qx = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
        qy = 0.25 * s;
        qz = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1));
        qw = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
        qx = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
        qy = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
        qz = 0.25 * s;
    }
    
    return std::vector<double>{qx, qy, qz, qw};
}

/**
 * @brief Convert local pose to global pose
 * 
 * Transforms a pose from local frame to global frame using the start pose as reference.
 * Mathematically: global_pose = start_pose @ local_pose (matrix multiplication)
 * 
 * @param cur_pose Current/start pose (reference frame), [x, y, z, qx, qy, qz, qw]
 * @param local_pose Local pose relative to start, [x, y, z, qx, qy, qz, qw]
 * @return Global pose result, [x, y, z, qx, qy, qz, qw]
 */
Pose local_pose_to_global(Pose cur_pose, Pose local_pose){
    // Create 4x4 transformation matrix from start pose
    cv::Mat T_start = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat R_start = quat_to_rotation_matrix(cur_pose.orientation.x, cur_pose.orientation.y, 
                                               cur_pose.orientation.z, cur_pose.orientation.w);
    R_start.copyTo(T_start(cv::Rect(0, 0, 3, 3)));
    T_start.at<double>(0, 3) = cur_pose.position.x;
    T_start.at<double>(1, 3) = cur_pose.position.y;
    T_start.at<double>(2, 3) = cur_pose.position.z;
    
    // Create 4x4 transformation matrix from local pose
    cv::Mat T_local = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat R_local = quat_to_rotation_matrix(local_pose.orientation.x, local_pose.orientation.y,
                                               local_pose.orientation.z, local_pose.orientation.w);
    R_local.copyTo(T_local(cv::Rect(0, 0, 3, 3)));
    T_local.at<double>(0, 3) = local_pose.position.x;
    T_local.at<double>(1, 3) = local_pose.position.y;
    T_local.at<double>(2, 3) = local_pose.position.z;
    
    // Global transformation = start transformation * local transformation
    cv::Mat T_global = T_start * T_local;
    
    // Extract position and orientation from result
    cv::Mat R_global = T_global(cv::Rect(0, 0, 3, 3));
    std::vector<double> quat_global = rotation_matrix_to_quat(R_global);
    
    Pose result;
    result.position.x = T_global.at<double>(0, 3);
    result.position.y = T_global.at<double>(1, 3);
    result.position.z = T_global.at<double>(2, 3);
    result.orientation.x = quat_global[0];
    result.orientation.y = quat_global[1];
    result.orientation.z = quat_global[2];
    result.orientation.w = quat_global[3];
    
    return result;
}

void demo_square_move(GalbotRobot& robot, GalbotNavigation& navi){
    Pose local_pose(std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.0, 0.707, 0.707});

    for(int i=0; i<4; ++i){
        Pose current_pose = navi.get_current_pose();
        std::cout << "Current pose: [" 
                  << current_pose.position.x << ", "
                  << current_pose.position.y << ", "
                  << current_pose.position.z << ", "
                  << current_pose.orientation.x << ", "
                  << current_pose.orientation.y << ", "
                  << current_pose.orientation.z << ", "
                  << current_pose.orientation.w << "]" << std::endl;

        Pose goal_pose = local_pose_to_global(current_pose, local_pose);
        
        if(navi.check_path_reachability(goal_pose, current_pose)){
            int retry_cnt = 3;
            for(;;){
                // navi.move_to_goal(goal_pose);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                navi.navigate_to_goal(goal_pose, true, true, 30);
                retry_cnt -= 1;
                if(navi.check_goal_arrival() || retry_cnt <= 0){
                    break;
                }else{
                    std::cout << "Navigation failed, retry count: " << retry_cnt << std::endl;
                }
            }
        }
    }
    Pose final_pose = navi.get_current_pose();
    std::cout << "Final pose: [" 
              << final_pose.position.x << ", "
              << final_pose.position.y << ", "
              << final_pose.position.z << ", "
              << final_pose.orientation.x << ", "
              << final_pose.orientation.y << ", "
              << final_pose.orientation.z << ", "
              << final_pose.orientation.w << "]" << std::endl;
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
        auto& navi = GalbotNavigation::get_instance(MachineType::G3);        

        /* Initialize robot */
        if (robot.init()) {
            std::cout << "Initialization successful" << std::endl;
            std::cout << "Is robot running: " << robot.is_running() << std::endl;
        }else{
            std::cerr << "Initialization failed" << std::endl;
        }
        /** Initialize navigation */
        if (navi.init()) {
            std::cout << "Navigation initialization successful" << std::endl;
        }else{
            std::cerr << "Navigation initialization failed" << std::endl;
        }

        /* Wait for data preparation */
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        
        /** If not localized, relocalize */
        bool is_localized = navi.is_localized();
        std::cout << "Is robot localized: " << is_localized << std::endl;
        while(!is_localized){
            NavigationStatus status = navi.relocalize(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
            std::this_thread::sleep_for(std::chrono::seconds(3));
            std::cout << "Waiting for localization..." << std::endl;
            is_localized = navi.is_localized();
        }

        /** If localized, move the robot */
        demo_square_move(robot, navi);

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
