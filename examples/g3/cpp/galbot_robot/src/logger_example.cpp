#include <iostream>

#include "galbot_robot.hpp"
#include "galbot_sdk_logger.hpp"

using namespace galbot::sdk;

int main() {
  // Get and initialize the GalbotRobot singleton
  auto& robot = GalbotRobot::get_instance(MachineType::G3);
  robot.init();
  // ==============================
  // 1. Initialize SDK logging
  // ==============================
  LoggerConfig logger_config;
  // Log generation path; if not filled, defaults to ~/galbot_sdk_log/user_log
  logger_config.path = "";
  // Log file name; if not filled, defaults to <process_name>_<current_time>_<pid>_<thread_id>.log
  logger_config.file_name = "";
  // Maximum bytes per single log file
  logger_config.file_max_size = 1024 * 1024 * 10; // 10MB
  // Maximum number of rotating log files; creates new file when full, retaining up to file_max_num files
  logger_config.file_max_num = 5;
  // Minimum log output level
  logger_config.level = LogLevel::DEBUG;
  // Whether to output to terminal; default is false
  logger_config.console_output = false;

  if (!init_galbot_sdk_logger(logger_config)) {
    std::cerr << "failed to init galbot sdk logger" << std::endl;
    return -1;
  } else {
    std::cout << "galbot sdk logger initialized successfully" << std::endl;
  }

  // ==============================
  // 2. Print logs at different levels
  // ==============================
  GALBOT_SDK_LOG_TRACE << "this is trace log";
  GALBOT_SDK_LOG_DEBUG << "this is debug log";
  GALBOT_SDK_LOG_INFO << "this is info log";
  GALBOT_SDK_LOG_WARN << "this is warning log";
  GALBOT_SDK_LOG_ERROR << "this is error log";
  GALBOT_SDK_LOG_CRITICAL << "this is critical log";

  // ==============================
  // 3. Supports chained << style
  // ==============================
  int robot_id = 3;
  GALBOT_SDK_LOG_INFO << "robot_id = " << robot_id;

  GALBOT_SDK_LOG_INFO << "example program finished";

  // Exit the system and release SDK and logger resources
  robot.request_shutdown();
  robot.wait_for_shutdown();
  robot.destroy();
  return 0;
}
