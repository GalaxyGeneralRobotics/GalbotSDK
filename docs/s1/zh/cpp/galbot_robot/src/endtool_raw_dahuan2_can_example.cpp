#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {

constexpr uint32_t kTxCanId = 0x601;
constexpr uint32_t kRxCanId = 0x201;
constexpr uint8_t kWriteSingleRegister = 0x06;
constexpr uint8_t kReadRegisters = 0x03;
constexpr uint16_t kRegisterInit = 0x0100;
constexpr uint16_t kRegisterForce = 0x0101;
constexpr uint16_t kRegisterTargetPosition = 0x0103;
constexpr uint16_t kRegisterSpeed = 0x0104;
constexpr uint16_t kRegisterStatusBase = 0x0200;

struct Dahuan2Status {
  uint16_t init_status = 0;
  uint16_t grip_status = 0;
  uint16_t position_permille = 0;
  uint16_t speed = 0;
  int16_t current_ma = 0;
  uint16_t error = 0;
};

struct DemoState {
  std::mutex mutex;
  Dahuan2Status status;
  bool has_status = false;
  std::array<uint64_t, 2> generation_by_kind{};
  uint64_t valid_count = 0;
  uint64_t idle_snapshot_count = 0;
  uint64_t rejected_count = 0;
  uint64_t non_increasing_generation_count = 0;
};

std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE> make_request(
    uint8_t function, uint16_t address, uint16_t value) {
  std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE> frame{};
  const uint32_t frame_type = 1;
  std::memcpy(frame.data(), &frame_type, sizeof(frame_type));
  std::memcpy(frame.data() + 4, &kTxCanId, sizeof(kTxCanId));
  frame[8] = 7;  // Five protocol bytes plus the two-byte TIB overhead.
  frame[12] = function;
  frame[13] = static_cast<uint8_t>(address >> 8);
  frame[14] = static_cast<uint8_t>(address);
  frame[15] = static_cast<uint8_t>(value >> 8);
  frame[16] = static_cast<uint8_t>(value);
  return frame;
}

std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE> encode_init(bool calibrate) {
  return make_request(kWriteSingleRegister, kRegisterInit, calibrate ? 0x00A5 : 0x0001);
}

std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE> encode_status_request() {
  return make_request(kReadRegisters, kRegisterStatusBase, 6);
}

bool decode_status(const EndToolRawData& raw, EndToolSide expected_side, Dahuan2Status& status) {
  // Dahuan CAN responses may appear in either receive buffer, so do not filter raw.kind.
  if (raw.side != expected_side) {
    return false;
  }
  // The receive PDO is a snapshot.  A zero marker means no new CAN response
  // arrived in this cycle; the remaining bytes may still contain an old reply.
  if (raw.frame[12] != 0x01) {
    return false;
  }

  uint32_t frame_type = 0;
  uint32_t can_id = 0;
  std::memcpy(&frame_type, raw.frame.data(), sizeof(frame_type));
  std::memcpy(&can_id, raw.frame.data() + 4, sizeof(can_id));
  if (frame_type != 1 || can_id != kRxCanId || raw.frame[16] != kReadRegisters ||
      raw.frame[17] < 12) {
    return false;
  }

  const auto read_be16 = [&raw](std::size_t register_offset) {
    const std::size_t offset = 18 + register_offset * 2;
    return static_cast<uint16_t>((static_cast<uint16_t>(raw.frame[offset]) << 8) |
                                 raw.frame[offset + 1]);
  };
  status.init_status = read_be16(0);
  status.grip_status = read_be16(1);
  status.position_permille = read_be16(2);
  status.speed = read_be16(3);
  status.current_ma = static_cast<int16_t>(read_be16(4));
  status.error = read_be16(5);
  return true;
}

const char* error_text(uint16_t error) {
  switch (error) {
    case 0:
      return "no error";
    case 1:
      return "under voltage";
    case 2:
      return "over voltage";
    case 3:
      return "over current";
    case 4:
      return "over heat";
    case 5:
      return "motor disconnected";
    case 8:
      return "overload";
    case 11:
      return "over speed";
    case 15:
      return "startup error";
    case 32:
      return "encoder error";
    default:
      return "unknown error";
  }
}

bool parse_side(const std::string& value, EndToolSide& side) {
  if (value == "left" || value == "l" || value == "0") {
    side = EndToolSide::LEFT;
    return true;
  }
  if (value == "right" || value == "r" || value == "1") {
    side = EndToolSide::RIGHT;
    return true;
  }
  return false;
}

uint64_t steady_now_ms() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}

}  // namespace

int main(int argc, char** argv) {
  EndToolSide side = EndToolSide::LEFT;
  int position = 500;
  int speed = 50;
  int force = 50;
  int calibrate_value = 0;
  int duration_s = 10;
  int init_wait_s = 4;
  int run_init_value = 0;
  int stop_controller_value = 0;
  int restore_controller_value = 0;
  try {
    if (!parse_side(argc > 1 ? argv[1] : "left", side)) {
      std::cerr << "Side must be left/right/l/r/0/1.\n";
      return 1;
    }
    position = argc > 2 ? std::stoi(argv[2]) : position;
    speed = argc > 3 ? std::stoi(argv[3]) : speed;
    force = argc > 4 ? std::stoi(argv[4]) : force;
    calibrate_value = argc > 5 ? std::stoi(argv[5]) : calibrate_value;
    duration_s = argc > 6 ? std::stoi(argv[6]) : duration_s;
    init_wait_s = argc > 7 ? std::stoi(argv[7]) : init_wait_s;
    run_init_value = argc > 8 ? std::stoi(argv[8]) : run_init_value;
    stop_controller_value = argc > 9 ? std::stoi(argv[9]) : stop_controller_value;
    restore_controller_value = argc > 10 ? std::stoi(argv[10]) : restore_controller_value;
  } catch (const std::exception&) {
    std::cerr << "Invalid numeric argument.\n";
    return 1;
  }
  const bool calibrate = calibrate_value == 1;
  const bool run_init = calibrate || run_init_value == 1;
  if (position < 0 || position > 1000 || speed < 1 || speed > 100 || force < 20 ||
      force > 100 || (calibrate_value != 0 && calibrate_value != 1) || duration_s < 0 ||
      init_wait_s < 0 || (run_init_value != 0 && run_init_value != 1) ||
      (stop_controller_value != 0 && stop_controller_value != 1) ||
      (restore_controller_value != 0 && restore_controller_value != 1)) {
    std::cerr << "Usage: " << argv[0]
              << " [left|right] [position:0..1000] [speed:1..100] [force:20..100]"
                 " [calibrate:0|1] [duration_s] [init_wait_s]"
                 " [run_init:0|1] [stop_standard_controller:0|1]"
                 " [restore_controller:0|1]\n";
    return 1;
  }

  const std::string controller_group = side == EndToolSide::LEFT ? "left_gripper" : "right_gripper";
  auto& robot = GalbotRobot::get_instance(MachineType::S1);
  if (!robot.init()) {
    std::cerr << "FAILED: failed to initialize the S1 SDK.\n";
    return 2;
  }

  std::cout << "WARNING: raw TX has no server-side exclusive writer lock.\n";
  std::cout << "WBCS must run with endtool_raw_enabled=true.\n";
  if (stop_controller_value == 1) {
    std::cout << "Best-effort stop of " << controller_group << ": "
              << static_cast<int>(robot.stop_controller(controller_group)) << '\n';
  }

  auto last_print_ms = std::make_shared<std::atomic<uint64_t>>(0);
  auto state = std::make_shared<DemoState>();
  const uint64_t callback_handle = robot.register_endtool_raw_callback(
      [side, last_print_ms, state](const EndToolRawData& raw) {
        if (raw.side != side) {
          return;
        }
        if (raw.frame[12] != 0x01) {
          std::lock_guard<std::mutex> lock(state->mutex);
          ++state->idle_snapshot_count;
          return;
        }
        Dahuan2Status status;
        if (!decode_status(raw, side, status)) {
          std::lock_guard<std::mutex> lock(state->mutex);
          ++state->rejected_count;
          return;
        }
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          const std::size_t kind_index =
              raw.kind == EndToolRxKind::BUFFER_1KHZ ? 0U : 1U;
          const uint64_t previous_generation = state->generation_by_kind[kind_index];
          if (raw.generation != 0 && previous_generation != 0 &&
              raw.generation <= previous_generation) {
            ++state->non_increasing_generation_count;
            return;
          }
          state->generation_by_kind[kind_index] = raw.generation;
          state->status = status;
          state->has_status = true;
          ++state->valid_count;
        }
        const uint64_t now_ms = steady_now_ms();
        uint64_t previous = last_print_ms->load(std::memory_order_relaxed);
        if (now_ms - previous < 200 || !last_print_ms->compare_exchange_strong(
                                           previous, now_ms, std::memory_order_relaxed)) {
          return;
        }
        std::cout << "RX kind=" << static_cast<uint32_t>(raw.kind)
                  << " generation=" << raw.generation << " init=" << status.init_status
                  << " grip=" << status.grip_status << " position=" << status.position_permille
                  << "permille speed=" << status.speed << " current=" << status.current_ma
                  << "mA error=0x" << std::hex << status.error << std::dec << " ("
                  << error_text(status.error) << ")\n";
      });
  if (callback_handle == 0) {
    std::cerr << "FAILED: failed to register the raw RX callback.\n";
    if (restore_controller_value == 1) {
      robot.start_controller(controller_group);
    }
    robot.destroy();
    return 3;
  }

  const auto publish = [&robot, side](const auto& frame, const char* description) {
    const auto result = robot.send_endtool_raw_frame(side, frame);
    if (result != ControlStatus::SUCCESS) {
      std::cerr << "FAILED: failed to publish " << description << ": "
                << static_cast<int>(result) << '\n';
      return false;
    }
    return true;
  };
  const auto poll_for = [&publish](int seconds) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!publish(encode_status_request(), "status request")) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return true;
  };

  bool success = true;
  if (run_init) {
    success = publish(encode_init(calibrate), calibrate ? "calibration command" : "homing command");
  }
  if (success && run_init) {
    std::cout << "Polling initialization state for " << init_wait_s << " seconds...\n";
    success = poll_for(init_wait_s);
  }
  success = success && publish(make_request(kWriteSingleRegister, kRegisterSpeed, speed), "speed command");
  success = success && publish(make_request(kWriteSingleRegister, kRegisterForce, force), "force command");
  success = success && publish(
                           make_request(kWriteSingleRegister, kRegisterTargetPosition, position),
                           "position command");
  uint64_t motion_status_baseline = 0;
  if (success) {
    std::lock_guard<std::mutex> lock(state->mutex);
    // Drop any response left by initialization polling. The following active
    // reads must produce a fresh status before the target can be verified.
    state->has_status = false;
    motion_status_baseline = state->valid_count;
  }
  if (success) {
    std::cout << "Polling status for " << duration_s << " seconds...\n";
    success = poll_for(duration_s);
  }

  if (success) {
    Dahuan2Status status;
    bool has_status = false;
    uint64_t valid_count = 0;
    uint64_t idle_count = 0;
    uint64_t rejected_count = 0;
    uint64_t non_increasing_count = 0;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      status = state->status;
      has_status = state->has_status;
      valid_count = state->valid_count;
      idle_count = state->idle_snapshot_count;
      rejected_count = state->rejected_count;
      non_increasing_count = state->non_increasing_generation_count;
    }
    if (!has_status || valid_count <= motion_status_baseline) {
      std::cerr << "FAILED: Dahuan2 device verification failed: no fresh status response was "
                   "received: idle_snapshots="
                << idle_count
                << " rejected=" << rejected_count
                << " non_increasing_generations=" << non_increasing_count << '\n';
      success = false;
    } else if (status.error != 0) {
      std::cerr << "FAILED: Dahuan2 device verification failed: gripper status error=0x"
                << std::hex << status.error << std::dec
                << " (" << error_text(status.error) << ")\n";
      success = false;
    } else if (std::abs(static_cast<int>(status.position_permille) - position) > 20) {
      std::cerr << "FAILED: Dahuan2 device verification failed: position did not reach target: "
                   "requested="
                << position
                << " actual=" << status.position_permille << " tolerance=20permille\n";
      success = false;
    } else {
      std::cout << "Position reached: requested=" << position
                << " actual=" << status.position_permille
                << "permille fresh_status_count=" << valid_count << '\n';
    }
  }

  robot.unregister_endtool_raw_callback(callback_handle);
  if (restore_controller_value == 1) {
    robot.start_controller(controller_group);
  }
  robot.destroy();
  return success ? 0 : 4;
}
