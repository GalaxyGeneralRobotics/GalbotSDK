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

using namespace galbot::sdk::g1;

// 定义点结构体
struct Point3D {
    float x, y, z;
};

/**
 * 直接通过指针操作从 LidarData 中提取 XYZ 数组
 */
std::vector<Point3D> get_xyz_points(const std::shared_ptr<LidarData>& cloud, bool remove_nan = false) {
    std::vector<Point3D> points;
    if (!cloud || cloud->data.empty()) return points;

    // 1. 查找 x, y, z 字段的偏移量 (offset)
    // Python 版中通过 field name 索引，这里我们预先找到偏移量以提高效率
    int32_t off_x = -1, off_y = -1, off_z = -1;
    for (const auto& f : cloud->fields) {
        if (f.name == "x") off_x = f.offset;
        else if (f.name == "y") off_y = f.offset;
        else if (f.name == "z") off_z = f.offset;
    }

    if (off_x == -1 || off_y == -1 || off_z == -1) {
        std::cerr << "错误: 点云数据缺少必要的 xyz 字段" << std::endl;
        return points;
    }

    uint32_t num_points = cloud->width * cloud->height;
    points.reserve(num_points);

    const uint8_t* raw_data = cloud->data.data();
    uint32_t point_step = cloud->point_step;

    // 2. 直接指针读取（零拷贝核心逻辑）
    for (uint32_t i = 0; i < num_points; ++i) {
        // 计算当前点的起始指针
        const uint8_t* pt_ptr = raw_data + (i * point_step);

        // 使用 reinterpret_cast 直接转换指针类型读取内存
        // 假设雷达数据是 float32 (F)，这是最常见的格式
        float x = *reinterpret_cast<const float*>(pt_ptr + off_x);
        float y = *reinterpret_cast<const float*>(pt_ptr + off_y);
        float z = *reinterpret_cast<const float*>(pt_ptr + off_z);

        // 处理 NaN (对应 Python 的 remove_nan 逻辑)
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
 * 保存到 PCD 文件
 */
void save_xyz_to_pcd(const std::vector<Point3D>& points, const std::string& filename) {
    std::ofstream fs(filename);
    if (!fs.is_open()) {
        std::cerr << "无法打开文件进行写入: " << filename << std::endl;
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
    std::cout << "已保存 " << points.size() << " 个点到 " << filename << std::endl;
}

int main() {
    // 获取对象实例
    auto& robot = GalbotRobot::get_instance();

    // 初始化传感器，为节省资源，只有初始化中传入的相机与雷达传感器可获取数据
    std::unordered_set<SensorType> sensor_types =  {
        SensorType::BASE_LIDAR              // 底盘激光雷达
    };

    // 初始化系统
    if (robot.init(sensor_types)) {
        std::cout << "系统初始化成功！" << std::endl;
    } else {
        std::cerr << "系统初始化失败！" << std::endl;
        return -1;
    }

    // 程序立即启动，稍等数据就绪时间
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 获取激光雷达数据
    std::shared_ptr<LidarData> lidar_data = robot.get_lidar_data(SensorType::BASE_LIDAR);
    if (lidar_data) {
        std::cout << "激光雷达数据获取成功！" << std::endl;
        std::vector<Point3D> xyz_points = get_xyz_points(lidar_data, false);
        if (!xyz_points.empty()) {
            save_xyz_to_pcd(xyz_points, "output_xyz.pcd");
        }
    } else {
        std::cerr << "激光雷达数据获取失败！" << std::endl;
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
