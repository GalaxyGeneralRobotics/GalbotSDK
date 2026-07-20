/**
 * @file galbot_perception.hpp
 * @brief Perception interface for on-device vision algorithms.
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

#include <set>

#include "galbot_sdk_type.hpp"
#include "galbot_perception_types.hpp"

namespace galbot {
namespace sdk {

/**
 * @class GalbotPerception
 * @brief Perception module interface; obtain the singleton via get_instance(MachineType).
 *
 * @robot G1 S1
 */
class GalbotPerception {
 public:
  virtual ~GalbotPerception() = default;

  /**
   * @brief Get the singleton instance of GalbotPerception.
   * @param m Machine type (e.g. MachineType::G1 or MachineType::S1).
   * @return Reference to the singleton instance for the given machine type.
   */
  static GalbotPerception& get_instance(MachineType m);

  /**
   * @brief Initialize perception and load models for the selected modules.
   * @param enabled_modules Set of perception modules to enable.
   * @return True if every requested module loaded successfully.
   */
  virtual bool init(const std::set<PerceptionModule>& enabled_modules) = 0;

  /**
   * @brief Run a single inference for the given module.
   * @note After init, wait ~10s for models to be ready before calling run_once.
   * @param module Perception module to run.
   */
  virtual bool run_once(PerceptionModule module) = 0;

  /**
   * @brief Block until the module produces a new result, or timeout. Use with run_once to fetch the latest output.
   * @param module    Perception module.
   * @param timeout_s Timeout in seconds.
   */
  virtual bool wait_for_new_result(PerceptionModule module, double timeout_s = 5.0) = 0;

  /**
   * @brief Return the latest cached result for the module without blocking.
   * @param module Perception module.
   * @param result Output detection result.
   * @return True if a result is available, false if none.
   */
  virtual bool get_latest_result(PerceptionModule module, DetectionResult& result) = 0;

 protected:
  GalbotPerception() = default;
  GalbotPerception(const GalbotPerception&) = delete;
  GalbotPerception& operator=(const GalbotPerception&) = delete;
  GalbotPerception(GalbotPerception&&) = delete;
  GalbotPerception& operator=(GalbotPerception&&) = delete;
};

}  // namespace sdk
}  // namespace galbot
