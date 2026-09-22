/**
 * @file galbot_sdk_logger.hpp
 * @brief Logging utilities for Galbot SDK
 * 
 * @author Galbot SDK Team
 * @copyright Copyright (c) 2023-2026 Galbot. All rights reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 #pragma once

#include <sstream>
#include <string>

#include "galbot_sdk_type.hpp"

namespace galbot {
namespace sdk {
/**
 * @brief Initialize the user logger for the Galbot SDK
 *
 * This function initializes the user logger with the specified configuration.
 * The logger supports asynchronous logging to files and console output.
 *
 * @param logger_config Configuration for the logger, including log level, file
 * path, file size, and other settings.
 * @return true if the logger was successfully initialized
 * @return false if the logger initialization failed
 */
bool init_galbot_sdk_logger(const LoggerConfig& logger_config);

class GalbotSdkLogStream {
 public:
  GalbotSdkLogStream(LogLevel level, const char* file, const char* func, int line)
      : level_(level), file_(file), func_(func), line_(line) {}

  template <typename T>
  GalbotSdkLogStream& operator<<(const T& v) {
    buffer_ << v;
    return *this;
  }

  GalbotSdkLogStream& operator<<(std::ostream& (*manip)(std::ostream&) ) {
    manip(buffer_);
    return *this;
  }

  ~GalbotSdkLogStream() noexcept {
    try {
      std::string s = buffer_.str();
      if (!s.empty()) {
        record_log(level_, file_, func_, line_, s.c_str());
      }
    } catch (...) {
      // 析构时不抛异常
    }
  }

 private:
  void record_log(LogLevel level, const char* file, const char* func, int line, const char* msg);

 private:
  LogLevel level_;
  const char* file_;
  const char* func_;
  int line_;
  std::ostringstream buffer_;
};

#define GALBOT_SDK_LOG_TRACE                                                                                        \
  galbot::sdk::GalbotSdkLogStream(galbot::sdk::LogLevel::TRACE,                                             \
                                      (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__), __func__, \
                                      __LINE__)

#define GALBOT_SDK_LOG_DEBUG                                                                                        \
  galbot::sdk::GalbotSdkLogStream(galbot::sdk::LogLevel::DEBUG,                                             \
                                      (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__), __func__, \
                                      __LINE__)

#define GALBOT_SDK_LOG_INFO                                                                                         \
  galbot::sdk::GalbotSdkLogStream(galbot::sdk::LogLevel::INFO,                                              \
                                      (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__), __func__, \
                                      __LINE__)

#define GALBOT_SDK_LOG_WARN                                                                                         \
  galbot::sdk::GalbotSdkLogStream(galbot::sdk::LogLevel::WARN,                                              \
                                      (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__), __func__, \
                                      __LINE__)

#define GALBOT_SDK_LOG_ERROR                                                                                        \
  galbot::sdk::GalbotSdkLogStream(galbot::sdk::LogLevel::ERROR,                                             \
                                      (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__), __func__, \
                                      __LINE__)

#define GALBOT_SDK_LOG_CRITICAL                                                                                     \
  galbot::sdk::GalbotSdkLogStream(galbot::sdk::LogLevel::CRITICAL,                                          \
                                      (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__), __func__, \
                                      __LINE__)

}  // namespace sdk
}  // namespace galbot
