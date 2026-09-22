#include<iostream>
#include<fstream>
#include<thread>
#include<chrono>
#include "opencv2/opencv.hpp"

#include "galbot_robot.hpp"

using namespace galbot::sdk;

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

struct Point3D {
    float x, y, z;
    uint8_t intensity;
};

std::vector<Point3D> get_xyz_points(const std::shared_ptr<LidarData>& lidar_data, bool remove_nan = false){
    std::vector<Point3D> points;
    if (!lidar_data || lidar_data->data.empty()) return points;

    /** 1. lookup the offset of x, y, z field */
    auto off_x = -1, off_y = -1, off_z = -1;
    auto off_intensity = -1;
    std::cout << "off_x" << off_x << std::endl;
    for (const auto& f: lidar_data->fields) {
        if (f.name == "x") off_x = f.offset;
        else if (f.name == "y") off_y = f.offset;
        else if (f.name == "z") off_z = f.offset;
        else if (f.name == "reflectivity") off_intensity = f.offset;
        std::cout << f.name << " " << f.datatype << " ";
    }
    std::cout << "\n";

    if (off_x == -1 || off_y == -1 || off_z == -1) {
        std::cerr << "Error: the lidar data has no required x/y/z field." << std::endl;
        return points;
    }

    /** 2. read data directly */
    uint32_t num_points = lidar_data->width * lidar_data->height;
    points.reserve(num_points);
    
    const uint8_t* raw_data = lidar_data->data.data();
    const uint32_t point_step = lidar_data->point_step;

    for (uint32_t i=0; i<num_points; ++i) {
        /** get pointer for current point */
        const uint8_t* point_ptr = raw_data + (i * point_step);
        
        /** use reinterpret_cast convert pointer type to read memory */
        /** assume points in lidar data are float32, which is most common data type */
        float x = *reinterpret_cast<const float*>(point_ptr + off_x);
        float y = *reinterpret_cast<const float*>(point_ptr + off_y);
        float z = *reinterpret_cast<const float*>(point_ptr + off_z);
        uint8_t intensity = off_intensity != -1 ? *reinterpret_cast<const uint8_t*>(point_ptr + off_intensity) : 255;
        
        if (remove_nan && (std::isnan(x) || std::isnan(y) || std::isnan(z))) {
            continue;
        }
        points.push_back(Point3D{x, y, z, intensity});
    }
    return points;
}

void save_xyz_to_pcd(const std::vector<Point3D>& points, const std::string& file_path){
    std::ofstream fs(file_path);
    if (!fs.is_open()) {
        std::cerr << "Error: cannot open file " << file_path << " for writing." << std::endl;
        return;
    }

    // PCD 0.7 Header
    fs << "# .PCD v0.7 - Point Cloud Data file format\n"
       << "VERSION 0.7\n"
       << "FIELDS x y z\n"
       << "SIZE 4 4 4\n"
       << "TYPE F F F\n"
       << "COUNT 1 1 1\n"
       << "WIDTH " << points.size() << "\n"
       << "HEIGHT 1\n"
       << "VIEWPOINT 0 0 0 1 0 0 0\n"
       << "POINTS " << points.size() << "\n"
       << "DATA ascii\n";
    
    for (const auto& p: points) {
        fs << p.x << " " << p.y << " " << p.z << "\n";
    }

    fs.close();
    std::cout << "Successfully saved " << points.size() << " points to " << file_path << std::endl;
}

int main(){
    check_robot_safety();
    try{
        /* Get robot instance  */
        auto& robot = GalbotRobot::get_instance(MachineType::G3); 
        
        /** Get left arm RGB, base LiDAR, and torso IMU data.
            G3 does not have arm-mounted depth cameras. */
        std::unordered_set<SensorType> sensor_types = {
            SensorType::LEFT_ARM_CAMERA,        /** Left arm rgb camera */
            SensorType::BASE_LIDAR,              /** Base LiDAR */
            SensorType::TORSO_IMU              /** Torso IMU */
        };

        /* Initialize robot */
        if (robot.init(sensor_types)) {
            std::cout << "Initialization successful" << std::endl;
        }else{
            std::cerr << "Initialization failed" << std::endl;
        }

        /* Wait for data preparation */
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        
        /** Get rgb image */
        std::shared_ptr<RgbData> rgb_data = robot.get_rgb_data(SensorType::LEFT_ARM_CAMERA, RgbOutputFormat::JPEG, true);
        std::shared_ptr<cv::Mat> rgb_img;
        if (rgb_data) {
            std::cout << "Get rgb image suceess" << std::endl;
            rgb_img = rgb_data->convert_to_cv2_mat();
            
            cv::imwrite("rgb_image.png", *rgb_img);
            std::cout << "RGB image saved as 'rgb_image.png'" << std::endl;
        }else{
            std::cerr << "No rgb image data!" << std::endl;
        }

        /** Get lidar data */
        std::shared_ptr<LidarData> lidar_data = robot.get_lidar_data(SensorType::BASE_LIDAR);
        if (lidar_data) {
            std::vector<Point3D> points = get_xyz_points(lidar_data);
            save_xyz_to_pcd(points, "lidar_points.pcd");
        } else {
            std::cerr << "No lidar data!" << std::endl;
        }
        
        /** Get torso imu data */
        std::shared_ptr<ImuData> imu_data = robot.get_imu_data(SensorType::TORSO_IMU);
        if (imu_data) {
            std::cout << "Get imu data suceess" << std::endl;
        } else {
            std::cerr << "No imu data!" << std::endl;
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
