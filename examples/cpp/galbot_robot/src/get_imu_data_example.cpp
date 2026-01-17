#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

#include "galbot_robot.hpp"

using namespace galbot::sdk::g1;

void print_imu_data(const std::shared_ptr<ImuData>& imu_data) {
    if (!imu_data) {
        std::cerr << "IMU data is empty" << std::endl;
        return;
    }

    std::cout << "Timestamp (ns): " << imu_data->timestamp_ns << std::endl;

    std::cout << "Accelerometer: "
              << "x=" << imu_data->accel.x << ", "
              << "y=" << imu_data->accel.y << ", "
              << "z=" << imu_data->accel.z << std::endl;

    std::cout << "Gyroscope: "
              << "x=" << imu_data->gyro.x << ", "
              << "y=" << imu_data->gyro.y << ", "
              << "z=" << imu_data->gyro.z << std::endl;

    std::cout << "Magnetometer: "
              << "x=" << imu_data->magnet.x << ", "
              << "y=" << imu_data->magnet.y << ", "
              << "z=" << imu_data->magnet.z << std::endl;
}

int main() {
    // 获取对象实例
    auto& robot = GalbotRobot::get_instance();

    // 初始化系统
    if (robot.init()) {
        std::cout << "系统初始化成功！" << std::endl;
    } else {
        std::cerr << "系统初始化失败！" << std::endl;
        return -1;
    }

    // 程序立即启动，稍等数据就绪时间
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 获取imu数据
    std::shared_ptr<ImuData> imu_data = robot.get_imu_data(SensorType::TORSO_IMU);
    if (imu_data) {
        std::cout << "IMU数据获取成功！" << std::endl;
        print_imu_data(imu_data);
    } else {
        std::cerr << "IMU数据获取失败！" << std::endl;
    }

    // 退出系统并进行SDK资源释放
    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();

    return 0;
}
