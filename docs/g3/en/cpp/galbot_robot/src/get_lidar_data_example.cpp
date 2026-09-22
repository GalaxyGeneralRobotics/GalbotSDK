#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <memory>
#include <unordered_set>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

// Define point structure
struct Point3D {
    float x, y, z;
};

/**
 * Directly extract the XYZ array from LidarData via pointer operations
 */
std::vector<Point3D> get_xyz_points(const std::shared_ptr<LidarData>& cloud, bool remove_nan = false) {
    std::vector<Point3D> points;
    if (!cloud || cloud->data.empty()) return points;

    // 1. Find the offsets of x, y, z fields (offset)
    // In the Python version, fields are indexed by field name; here we precompute offsets for better performance
    int32_t off_x = -1, off_y = -1, off_z = -1;
    for (const auto& f : cloud->fields) {
        if (f.name == "x") off_x = f.offset;
        else if (f.name == "y") off_y = f.offset;
        else if (f.name == "z") off_z = f.offset;
    }

    if (off_x == -1 || off_y == -1 || off_z == -1) {
        std::cerr << "Error: point cloud data is missing required xyz fields" << std::endl;
        return points;
    }

    uint32_t num_points = cloud->width * cloud->height;
    points.reserve(num_points);

    const uint8_t* raw_data = cloud->data.data();
    uint32_t point_step = cloud->point_step;

    // 2. Read directly via pointer (core zero-copy logic)
    for (uint32_t i = 0; i < num_points; ++i) {
        // Calculate starting pointer for current point
        const uint8_t* pt_ptr = raw_data + (i * point_step);

        // Use reinterpret_cast to directly cast pointer types and read memory
        // Assume LiDAR data is float32 (F), which is the most common format
        float x = *reinterpret_cast<const float*>(pt_ptr + off_x);
        float y = *reinterpret_cast<const float*>(pt_ptr + off_y);
        float z = *reinterpret_cast<const float*>(pt_ptr + off_z);

        // Handle NaN values (corresponds to Python remove_nan logic)
        if (remove_nan) {
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
                continue;
            }
        }

        points.push_back({x, y, z});
    }

    return points;
}

/**
 * Save to a PCD file
 */
void save_xyz_to_pcd(const std::vector<Point3D>& points, const std::string& filename) {
    std::ofstream fs(filename);
    if (!fs.is_open()) {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
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

    // Data
    for (const auto& p : points) {
        fs << p.x << " " << p.y << " " << p.z << "\n";
    }

    fs.close();
    std::cout << "Saved " << points.size() << " points to " << filename << std::endl;
}

int main() {
    // Get object instance
    auto& robot = GalbotRobot::get_instance(MachineType::G3);

    // Initialize sensors; only cameras and LiDAR sensors passed during initialization can retrieve data
    std::unordered_set<SensorType> sensor_types =  {
        SensorType::BASE_LIDAR              // Chassis lidar
    };

    // Initialize system
    if (robot.init(sensor_types)) {
        std::cout << "System initialized successfully!" << std::endl;
    } else {
        std::cerr << "System initialization failed!" << std::endl;
        return -1;
    }

    // Program started, waiting for data
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Get LiDAR data
    std::shared_ptr<LidarData> lidar_data = robot.get_lidar_data(SensorType::BASE_LIDAR);
    if (lidar_data) {
        std::cout << "Lidar data retrieved successfully!" << std::endl;
        std::vector<Point3D> xyz_points = get_xyz_points(lidar_data, false);
        if (!xyz_points.empty()) {
            save_xyz_to_pcd(xyz_points, "output_xyz.pcd");
        }
    } else {
        std::cerr << "Failed to get lidar data!" << std::endl;
    }

    // Exit system and release SDK resources
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
