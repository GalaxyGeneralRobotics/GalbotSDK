#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {

constexpr uint32_t kTxCanId = 0x70A;
constexpr uint32_t kRxCanId = 0x10A;
constexpr uint8_t kWriteCommand = 0x21;
constexpr uint8_t kAnswerCommand = 0x22;
constexpr uint8_t kReportCommand = 0x23;
constexpr uint8_t kInitFunction = 0x04;
constexpr uint8_t kMoveFunction = 0x05;
constexpr float kMinWidthMm = 6.0F;
constexpr float kMaxWidthMm = 120.0F;
constexpr float kMinVelocityMmS = 1.0F;
constexpr float kMaxVelocityMmS = 100.0F;
constexpr float kMinTorqueNm = 1.0F;
constexpr float kMaxTorqueNm = 50.0F;
constexpr auto kStatusPollInterval = std::chrono::milliseconds(100);
constexpr auto kStatusStaleInterval = std::chrono::seconds(1);
constexpr auto kInitialStatusWait = std::chrono::seconds(1);
constexpr auto kStatusPrintInterval = std::chrono::milliseconds(500);
constexpr auto kRawTransportReadyTimeout = std::chrono::seconds(2);
constexpr auto kRawTransportWarmupInterval = std::chrono::milliseconds(200);
constexpr auto kInitTimeout = std::chrono::seconds(20);
constexpr auto kInitCommandGuardInterval = std::chrono::seconds(1);
constexpr auto kMonitorDuration = std::chrono::seconds(10);
constexpr float kPositionToleranceMm = 2.0F;
constexpr float kStoppedVelocityToleranceMmS = 1.0F;

enum class DecodeResult {
  VALID,
  IDLE_SNAPSHOT,
  FRAME_TYPE,
  CAN_ID,
  COMMAND,
  PAYLOAD,
};

struct GripStatus {
  float position_mm = 0.0F;
  float velocity_mm_s = 0.0F;
  float torque_nm = 0.0F;
  uint8_t motion_state = 0;
  uint8_t error_code = 0;
  uint8_t response_command = 0;
};

struct DemoState {
  std::mutex mutex;
  std::condition_variable changed;
  GripStatus last_status;
  bool valid = false;
  uint64_t last_generation = 0;
  uint64_t last_update_ms = 0;
  uint64_t latest_raw_generation = 0;
  uint64_t idle_snapshot_count = 0;
  uint64_t frame_type_reject_count = 0;
  uint64_t can_id_reject_count = 0;
  uint64_t command_reject_count = 0;
  uint64_t payload_reject_count = 0;
  uint64_t non_increasing_generation_count = 0;
};

struct StateSnapshot {
  GripStatus status;
  bool valid = false;
  uint64_t last_generation = 0;
  uint64_t last_update_ms = 0;
  uint64_t latest_raw_generation = 0;
  uint64_t idle_snapshot_count = 0;
  uint64_t frame_type_reject_count = 0;
  uint64_t can_id_reject_count = 0;
  uint64_t command_reject_count = 0;
  uint64_t payload_reject_count = 0;
  uint64_t non_increasing_generation_count = 0;
};

uint64_t steady_now_ms() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now().time_since_epoch())
                                   .count());
}

std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE> make_can_frame(
    uint8_t command, uint8_t function, const uint8_t* payload, std::size_t payload_size) {
  std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE> frame{};
  const uint32_t frame_type = 1;
  std::memcpy(frame.data(), &frame_type, sizeof(frame_type));
  std::memcpy(frame.data() + 4, &kTxCanId, sizeof(kTxCanId));
  frame[8] = static_cast<uint8_t>(payload_size + 2);
  frame[12] = command;
  frame[13] = function;
  if (payload != nullptr && payload_size > 0) {
    std::memcpy(frame.data() + 14, payload, payload_size);
  }
  return frame;
}

std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE> encode_init() {
  return make_can_frame(kWriteCommand, kInitFunction, nullptr, 0);
}

std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE> encode_move(
    float width_mm, float velocity_mm_s, float torque_nm) {
  std::array<uint8_t, 12> payload{};
  std::memcpy(payload.data(), &width_mm, sizeof(width_mm));
  std::memcpy(payload.data() + 4, &velocity_mm_s, sizeof(velocity_mm_s));
  std::memcpy(payload.data() + 8, &torque_nm, sizeof(torque_nm));
  return make_can_frame(kWriteCommand, kMoveFunction, payload.data(), payload.size());
}

DecodeResult decode_status(const EndToolRawData& raw, EndToolSide expected_side, GripStatus& status) {
  // This gripper's CAN response is valid only in the 250 Hz passthrough buffer.
  if (raw.side != expected_side || raw.kind != EndToolRxKind::BUFFER_250HZ) {
    return DecodeResult::FRAME_TYPE;
  }

  uint32_t frame_type = 0;
  uint32_t can_id = 0;
  std::memcpy(&frame_type, raw.frame.data(), sizeof(frame_type));
  std::memcpy(&can_id, raw.frame.data() + 4, sizeof(can_id));
  if (frame_type != 1) {
    return DecodeResult::FRAME_TYPE;
  }
  // A zero CAN ID is an empty slot in the 250 Hz buffer, not a device error.
  if (can_id == 0) {
    return DecodeResult::IDLE_SNAPSHOT;
  }
  if (can_id != kRxCanId) {
    return DecodeResult::CAN_ID;
  }

  // Valid S1 EtherCAT/TIB snapshots can have byte 12 cleared. Identify the
  // device report by its CAN ID and by the command/function fields at byte 16.
  const uint8_t command = raw.frame[16];
  const uint8_t function = raw.frame[17];
  if ((command != kAnswerCommand && command != kReportCommand) || function != kMoveFunction) {
    return DecodeResult::COMMAND;
  }

  GripStatus decoded;
  decoded.error_code = raw.frame[21];
  decoded.response_command = command;
  std::memcpy(&decoded.position_mm, raw.frame.data() + 22, sizeof(decoded.position_mm));
  std::memcpy(&decoded.velocity_mm_s, raw.frame.data() + 26, sizeof(decoded.velocity_mm_s));
  std::memcpy(&decoded.torque_nm, raw.frame.data() + 30, sizeof(decoded.torque_nm));
  decoded.motion_state = raw.frame[34];
  if (!std::isfinite(decoded.position_mm) || !std::isfinite(decoded.velocity_mm_s) ||
      !std::isfinite(decoded.torque_nm) || decoded.position_mm < 0.0F ||
      decoded.position_mm > kMaxWidthMm ||
      std::fabs(decoded.velocity_mm_s) > kMaxVelocityMmS ||
      std::fabs(decoded.torque_nm) > kMaxTorqueNm || decoded.motion_state > 4) {
    // An uninitialized gripper may report 0 mm even though Move accepts 6 mm
    // as its minimum target, so only negative positions are rejected here.
    return DecodeResult::PAYLOAD;
  }

  status = decoded;
  return DecodeResult::VALID;
}

const char* error_text(uint8_t error_code) {
  switch (error_code) {
    case 0x00:
      return "no error";
    case 0x01:
      return "not enabled";
    case 0x08:
      return "overvoltage";
    case 0x09:
      return "undervoltage";
    case 0x0A:
      return "overcurrent";
    case 0x0B:
      return "MOS overheating";
    case 0x0C:
      return "motor coil overheating";
    case 0x0D:
      return "communication lost";
    case 0x0E:
      return "motor overload";
    case 0x10:
      return "locked rotor";
    case 0x11:
      return "locked rotor and not enabled";
    case 0x40:
      return "power loss after calibration";
    case 0x80:
      return "communication error";
    default:
      return "unknown error";
  }
}

StateSnapshot get_snapshot(const std::shared_ptr<DemoState>& state) {
  std::lock_guard<std::mutex> lock(state->mutex);
  StateSnapshot snapshot;
  snapshot.status = state->last_status;
  snapshot.valid = state->valid;
  snapshot.last_generation = state->last_generation;
  snapshot.last_update_ms = state->last_update_ms;
  snapshot.latest_raw_generation = state->latest_raw_generation;
  snapshot.idle_snapshot_count = state->idle_snapshot_count;
  snapshot.frame_type_reject_count = state->frame_type_reject_count;
  snapshot.can_id_reject_count = state->can_id_reject_count;
  snapshot.command_reject_count = state->command_reject_count;
  snapshot.payload_reject_count = state->payload_reject_count;
  snapshot.non_increasing_generation_count = state->non_increasing_generation_count;
  return snapshot;
}

void count_decode_result(DemoState& state, DecodeResult result) {
  switch (result) {
    case DecodeResult::IDLE_SNAPSHOT:
      ++state.idle_snapshot_count;
      break;
    case DecodeResult::FRAME_TYPE:
      ++state.frame_type_reject_count;
      break;
    case DecodeResult::CAN_ID:
      ++state.can_id_reject_count;
      break;
    case DecodeResult::COMMAND:
      ++state.command_reject_count;
      break;
    case DecodeResult::PAYLOAD:
      ++state.payload_reject_count;
      break;
    case DecodeResult::VALID:
      break;
  }
}

void print_status(const char* prefix, uint64_t generation, const GripStatus& status) {
  std::cout << prefix << " generation=" << generation << " source="
            << (status.response_command == kAnswerCommand ? "ANSWER" : "REPORT")
            << " position=" << status.position_mm << "mm velocity=" << status.velocity_mm_s
            << "mm/s torque=" << status.torque_nm
            << "Nm motion_state=" << static_cast<unsigned int>(status.motion_state)
            << " error=0x" << std::hex << static_cast<unsigned int>(status.error_code) << std::dec
            << " (" << error_text(status.error_code) << ")\n";
}

void print_rx_summary(const StateSnapshot& snapshot) {
  std::cout << "RX summary: latest_raw_generation=" << snapshot.latest_raw_generation
            << " latest_valid_generation=" << snapshot.last_generation
            << " idle_snapshots=" << snapshot.idle_snapshot_count
            << " malformed={frame_type:" << snapshot.frame_type_reject_count
            << ",can_id:" << snapshot.can_id_reject_count
            << ",command:" << snapshot.command_reject_count
            << ",payload:" << snapshot.payload_reject_count << "}"
            << " non_increasing_generations=" << snapshot.non_increasing_generation_count
            << '\n';
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

bool valid_motion_command(float width_mm, float velocity_mm_s, float torque_nm) {
  return std::isfinite(width_mm) && std::isfinite(velocity_mm_s) && std::isfinite(torque_nm) &&
         width_mm >= kMinWidthMm && width_mm <= kMaxWidthMm &&
         velocity_mm_s >= kMinVelocityMmS && velocity_mm_s <= kMaxVelocityMmS &&
         torque_nm >= kMinTorqueNm && torque_nm <= kMaxTorqueNm;
}

void print_usage(const char* program, std::ostream& output) {
  output << "Usage:\n"
         << "  " << program << " [options]\n"
         << "  " << program
         << " [left|right] [width_mm] [velocity_mm_s] [torque_nm] [init:0|1]"
            " [stop_controller:0|1] [restore_controller:0|1]\n\n"
         << "Options:\n"
         << "  -h, --help                    Show this help message.\n"
         << "  --side <left|right>           Gripper side (default: left).\n"
         << "  --width-mm <6..120>           Target opening in millimeters (default: 50).\n"
         << "  --velocity-mm-s <1..100>      Target velocity in mm/s (default: 50).\n"
         << "  --torque-nm <1..50>           Target torque in Nm (default: 20).\n"
         << "  --init                         Home/calibrate before Move.\n"
         << "  --stop-standard-controller    Best-effort stop of the standard controller.\n"
         << "  --restore-controller          Restart the standard controller on exit.\n\n"
         << "Examples:\n"
         << "  " << program << " --help\n"
         << "  " << program << " --side left --init\n"
         << "  " << program
         << " --side right --width-mm 50 --velocity-mm-s 50 --torque-nm 20 --init\n";
}

}  // namespace

int main(int argc, char** argv) {
  EndToolSide side = EndToolSide::LEFT;
  float width_mm = 50.0F;
  float velocity_mm_s = 50.0F;
  float torque_nm = 20.0F;
  int init_value = 0;
  int stop_controller_value = 0;
  int restore_controller_value = 0;
  try {
    // Named options match the Python example, while the original positional
    // form remains available for existing scripts.
    const bool named_options = argc > 1 && argv[1][0] == '-';
    if (named_options) {
      for (int index = 1; index < argc; ++index) {
        const std::string option = argv[index];
        if (option == "-h" || option == "--help") {
          print_usage(argv[0], std::cout);
          return 0;
        }

        const auto read_value = [&index, argc, argv](const std::string& name) {
          if (index + 1 >= argc) {
            throw std::invalid_argument("missing value for " + name);
          }
          return std::string(argv[++index]);
        };
        if (option == "--side") {
          const std::string value = read_value(option);
          if (!parse_side(value, side)) {
            throw std::invalid_argument("--side must be left or right");
          }
        } else if (option == "--width-mm") {
          width_mm = std::stof(read_value(option));
        } else if (option == "--velocity-mm-s") {
          velocity_mm_s = std::stof(read_value(option));
        } else if (option == "--torque-nm") {
          torque_nm = std::stof(read_value(option));
        } else if (option == "--init") {
          init_value = 1;
        } else if (option == "--stop-standard-controller") {
          stop_controller_value = 1;
        } else if (option == "--restore-controller") {
          restore_controller_value = 1;
        } else {
          throw std::invalid_argument("unknown option: " + option);
        }
      }
    } else {
      if (!parse_side(argc > 1 ? argv[1] : "left", side)) {
        throw std::invalid_argument("side must be left/right/l/r/0/1");
      }
      width_mm = argc > 2 ? std::stof(argv[2]) : width_mm;
      velocity_mm_s = argc > 3 ? std::stof(argv[3]) : velocity_mm_s;
      torque_nm = argc > 4 ? std::stof(argv[4]) : torque_nm;
      init_value = argc > 5 ? std::stoi(argv[5]) : init_value;
      stop_controller_value = argc > 6 ? std::stoi(argv[6]) : stop_controller_value;
      restore_controller_value = argc > 7 ? std::stoi(argv[7]) : restore_controller_value;
    }
  } catch (const std::exception& error) {
    std::cerr << "Argument error: " << error.what() << "\n\n";
    print_usage(argv[0], std::cerr);
    return 1;
  }
  if (!valid_motion_command(width_mm, velocity_mm_s, torque_nm) ||
      (init_value != 0 && init_value != 1) ||
      (stop_controller_value != 0 && stop_controller_value != 1) ||
      (restore_controller_value != 0 && restore_controller_value != 1)) {
    std::cerr << "Argument error: motion values are outside the documented ranges.\n\n";
    print_usage(argv[0], std::cerr);
    return 1;
  }

  const std::string controller_group = side == EndToolSide::LEFT ? "left_gripper" : "right_gripper";
  // Print the resolved values before SDK initialization so command-line
  // mistakes remain visible even when the robot connection cannot be created.
  std::cout << "Parsed options: side=" << (side == EndToolSide::LEFT ? "left" : "right")
            << " width_mm=" << width_mm << " velocity_mm_s=" << velocity_mm_s
            << " torque_nm=" << torque_nm << " init=" << std::boolalpha
            << (init_value == 1) << " stop_standard_controller="
            << (stop_controller_value == 1) << " restore_controller="
            << (restore_controller_value == 1) << std::noboolalpha << '\n';

  auto& robot = GalbotRobot::get_instance(MachineType::S1);
  if (!robot.init()) {
    std::cerr << "FAILED: failed to initialize the S1 SDK.\n";
    return 2;
  }

  std::cout << "WARNING: same-process raw TX is serialized, but there is no cross-client "
               "resource lease.\n";
  std::cout << "WBCS must run with endtool_raw_enabled=true in [robot_info.custom_params].\n";
  if (stop_controller_value == 1) {
    const auto stop_status = robot.stop_controller(controller_group);
    std::cout << "Best-effort stop of " << controller_group << ": "
              << static_cast<int>(stop_status) << '\n';
  }

  auto state = std::make_shared<DemoState>();
  const uint64_t callback_handle = robot.register_endtool_raw_callback(
      [side, state](const EndToolRawData& raw) {
        if (raw.side != side || raw.kind != EndToolRxKind::BUFFER_250HZ) {
          return;
        }

        GripStatus status;
        const DecodeResult decode_result = decode_status(raw, side, status);
        std::unique_lock<std::mutex> lock(state->mutex);
        state->latest_raw_generation = raw.generation;
        if (decode_result != DecodeResult::VALID) {
          count_decode_result(*state, decode_result);
          lock.unlock();
          state->changed.notify_all();
          return;
        }

        if (raw.generation != 0 && state->last_generation != 0 &&
            raw.generation <= state->last_generation) {
          ++state->non_increasing_generation_count;
          lock.unlock();
          state->changed.notify_all();
          return;
        }
        state->last_status = status;
        state->valid = true;
        state->last_generation = raw.generation;
        state->last_update_ms = steady_now_ms();
        lock.unlock();
        state->changed.notify_all();
      });
  if (callback_handle == 0) {
    std::cerr << "FAILED: failed to register the raw RX callback.\n";
    if (restore_controller_value == 1) {
      robot.start_controller(controller_group);
    }
    robot.destroy();
    return 3;
  }

  bool success = true;
  std::cout << "Waiting for the first valid Raw status before TX (timeout=2s)...\n";
  {
    std::unique_lock<std::mutex> lock(state->mutex);
    success = state->changed.wait_for(lock, kRawTransportReadyTimeout, [&state] {
      return state->valid;
    });
  }
  if (!success) {
    const StateSnapshot snapshot = get_snapshot(state);
    std::cerr << "FAILED: raw transport did not deliver a valid gripper status before TX; "
              << "latest_raw_generation=" << snapshot.latest_raw_generation
              << " idle_snapshots=" << snapshot.idle_snapshot_count << '\n';
    print_rx_summary(snapshot);
  } else {
    const StateSnapshot snapshot = get_snapshot(state);
    std::cout << "Raw transport ready: generation=" << snapshot.last_generation
              << "; warming up TX for 200ms...\n";
    // Receiving a valid sample proves that discovery is active. Give the
    // command writer a short additional window to match the WBCS subscriber
    // before publishing the first one-shot Init or Move frame.
    std::this_thread::sleep_for(kRawTransportWarmupInterval);
  }

  if (success && init_value == 1) {
    std::cout << "WARNING: Init performs homing/calibration; do not run it before every Move.\n";
    const uint64_t init_baseline = get_snapshot(state).last_generation;
    if (robot.send_endtool_raw_frame(side, encode_init()) != ControlStatus::SUCCESS) {
      std::cerr << "FAILED: failed to publish the gripper init frame.\n";
      success = false;
    }

    if (success) {
      // A passive Ready report from the pre-Init state can arrive immediately
      // after publication. Keep Move out of the TX path for at least one second
      // so that report cannot prematurely complete this Init attempt.
      const uint64_t ready_not_before_ms =
          steady_now_ms() +
          static_cast<uint64_t>(
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  kInitCommandGuardInterval)
                  .count());
      std::cout << "Waiting for a fresh Ready status (2) after Init "
                   "(minimum guard=1s, timeout=20s)...\n";
      const auto init_deadline = std::chrono::steady_clock::now() + kInitTimeout;
      auto last_print_at = std::chrono::steady_clock::time_point::min();
      uint8_t last_motion_state = 0;
      uint8_t last_error_code = 0;
      bool has_printed_state = false;
      bool ready = false;
      while (std::chrono::steady_clock::now() < init_deadline) {
        // Wake on a raw callback, but periodically re-check the timeout even if
        // the receive stream is silent.
        {
          std::unique_lock<std::mutex> lock(state->mutex);
          state->changed.wait_for(lock, kStatusPollInterval);
        }
        const StateSnapshot snapshot = get_snapshot(state);
        if (!snapshot.valid || snapshot.last_generation <= init_baseline) {
          continue;
        }

        const auto now = std::chrono::steady_clock::now();
        const bool state_changed = !has_printed_state ||
                                   snapshot.status.motion_state != last_motion_state ||
                                   snapshot.status.error_code != last_error_code;
        if (state_changed || now - last_print_at >= kStatusPrintInterval) {
          print_status("Init status:", snapshot.last_generation, snapshot.status);
          last_motion_state = snapshot.status.motion_state;
          last_error_code = snapshot.status.error_code;
          last_print_at = now;
          has_printed_state = true;
        }
        // State 2 is command-ready. State 4 can be a previous Move completion
        // and therefore does not prove that this Init has completed.
        if (snapshot.status.motion_state == 2 &&
            snapshot.last_update_ms >= ready_not_before_ms) {
          ready = true;
          break;
        }
      }
      if (!ready) {
        const StateSnapshot snapshot = get_snapshot(state);
        std::cerr << "FAILED: gripper did not report a fresh command-ready status after Init; "
                  << "latest_valid_generation=" << snapshot.last_generation
                  << " latest_raw_generation=" << snapshot.latest_raw_generation
                  << " idle_snapshots=" << snapshot.idle_snapshot_count << '\n';
        print_rx_summary(snapshot);
        success = false;
      }
    }
  }

  if (success) {
    // Give the passive stream a short chance to expose an uninitialized
    // device before publishing Move. No status is treated as inconclusive.
    {
      std::unique_lock<std::mutex> lock(state->mutex);
      state->changed.wait_for(lock, kInitialStatusWait, [&state] {
        return state->valid;
      });
      if (state->valid &&
          (state->last_status.error_code == 0x01 || state->last_status.motion_state == 0)) {
        std::cerr << "FAILED: gripper is not initialized (device error 0x01/state 0); "
                     "rerun this example with init=1 before sending Move.\n";
        success = false;
      }
    }
  }

  uint64_t move_baseline = 0;
  if (success) {
    move_baseline = get_snapshot(state).last_generation;
    if (robot.send_endtool_raw_frame(side, encode_move(width_mm, velocity_mm_s, torque_nm)) !=
        ControlStatus::SUCCESS) {
      std::cerr << "FAILED: failed to publish the gripper move frame.\n";
      success = false;
    }
  }

  if (success) {
    std::cout << "Move sent. Monitoring only fresh passive status reports for 10 seconds...\n";
    const auto deadline = std::chrono::steady_clock::now() + kMonitorDuration;
    auto last_sdk_print = std::chrono::steady_clock::time_point::min();
    auto last_raw_print = std::chrono::steady_clock::time_point::min();
    float last_sdk_position_mm = 0.0F;
    bool has_sdk_position = false;
    bool has_raw_state = false;
    uint8_t last_raw_motion_state = 0;
    uint8_t last_raw_error_code = 0;
    uint64_t printed_generation = move_baseline;
    bool received_after_move = false;
    bool raw_target_reached = false;
    bool sdk_target_reached = false;
    GripStatus latest_move_status;
    bool has_latest_move_status = false;

    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::unique_lock<std::mutex> lock(state->mutex);
        state->changed.wait_for(lock, kStatusPollInterval);
      }
      const StateSnapshot snapshot = get_snapshot(state);

      if (snapshot.valid && snapshot.last_generation > move_baseline &&
          snapshot.last_generation != printed_generation) {
        received_after_move = true;
        printed_generation = snapshot.last_generation;
        latest_move_status = snapshot.status;
        has_latest_move_status = true;

        const auto now = std::chrono::steady_clock::now();
        const bool state_changed = !has_raw_state ||
                                   snapshot.status.motion_state != last_raw_motion_state ||
                                   snapshot.status.error_code != last_raw_error_code;
        if (state_changed || now - last_raw_print >= kStatusPrintInterval) {
          print_status("Raw status snapshot:", snapshot.last_generation, snapshot.status);
          last_raw_motion_state = snapshot.status.motion_state;
          last_raw_error_code = snapshot.status.error_code;
          last_raw_print = now;
          has_raw_state = true;
        }

        if (snapshot.status.error_code == 0 &&
            std::fabs(snapshot.status.position_mm - width_mm) <= kPositionToleranceMm &&
            std::fabs(snapshot.status.velocity_mm_s) <= kStoppedVelocityToleranceMmS) {
          std::cout << "Move completed within tolerance: requested=" << width_mm
                    << "mm actual=" << snapshot.status.position_mm
                    << "mm (verified by Raw status).\n";
          raw_target_reached = true;
          break;
        }
      }

      const auto sdk_state = robot.get_gripper_state(controller_group);
      if (sdk_state == nullptr) {
        continue;
      }
      const float position_mm = static_cast<float>(sdk_state->width * 1000.0);
      const float velocity_mm_s = static_cast<float>(sdk_state->velocity * 1000.0);
      const auto now = std::chrono::steady_clock::now();
      if (!has_sdk_position || std::fabs(position_mm - last_sdk_position_mm) >= 0.1F ||
          now - last_sdk_print >= kStatusPrintInterval) {
        std::cout << "SDK state: position=" << position_mm << "mm velocity=" << velocity_mm_s
                  << "mm/s effort=" << sdk_state->effort
                  << " is_moving=" << std::boolalpha << sdk_state->is_moving << std::noboolalpha
                  << '\n';
        last_sdk_position_mm = position_mm;
        last_sdk_print = now;
        has_sdk_position = true;
      }
      if (std::fabs(position_mm - width_mm) <= kPositionToleranceMm && !sdk_state->is_moving) {
        std::cout << "Move completed within tolerance: requested=" << width_mm
                  << "mm actual=" << position_mm
                  << "mm (verified by get_gripper_state).\n";
        sdk_target_reached = true;
        break;
      }
    }

    const StateSnapshot snapshot = get_snapshot(state);
    if (!received_after_move) {
      std::cout << "WARNING: no valid gripper status snapshot was received after Move. "
                   "Publication success alone does not prove that the device executed the "
                   "command.\n";
    } else if (steady_now_ms() - snapshot.last_update_ms >
               static_cast<uint64_t>(
                   std::chrono::duration_cast<std::chrono::milliseconds>(kStatusStaleInterval).count())) {
      std::cout << "WARNING: the valid gripper status stream stopped; subsequent receive "
                   "buffers were idle or rejected.\n";
    }
    print_rx_summary(snapshot);

    if (!raw_target_reached && !sdk_target_reached) {
      // Raw verification is authoritative for raw-mode operation. Retain the
      // SDK state as a fallback for configurations that still publish it.
      const auto sdk_state = robot.get_gripper_state(controller_group);
      if (sdk_state != nullptr) {
        const float position_mm = static_cast<float>(sdk_state->width * 1000.0);
        sdk_target_reached = std::fabs(position_mm - width_mm) <= kPositionToleranceMm &&
                             !sdk_state->is_moving;
      }

      if (!sdk_target_reached) {
        std::cerr << "FAILED: self-developed gripper verification failed: Move did not reach "
                     "the requested width: requested="
                  << width_mm << "mm tolerance=" << kPositionToleranceMm << "mm; ";
        if (sdk_state == nullptr) {
          std::cerr << "SDK state=unavailable; ";
        } else {
          const float position_mm = static_cast<float>(sdk_state->width * 1000.0);
          std::cerr << "SDK state={position=" << position_mm
                    << "mm,error=" << std::fabs(position_mm - width_mm)
                    << "mm,is_moving=" << std::boolalpha << sdk_state->is_moving
                    << std::noboolalpha << "}; ";
        }
        if (!has_latest_move_status) {
          std::cerr << "last_raw_status=none\n";
        } else {
          std::cerr << "last_raw_status={generation=" << printed_generation
                    << ",position=" << latest_move_status.position_mm
                    << "mm,velocity=" << latest_move_status.velocity_mm_s
                    << "mm/s,motion_state="
                    << static_cast<unsigned int>(latest_move_status.motion_state)
                    << ",error=0x" << std::hex
                    << static_cast<unsigned int>(latest_move_status.error_code) << std::dec
                    << "}\n";
        }
        success = false;
      }
    }
  }

  robot.unregister_endtool_raw_callback(callback_handle);
  if (restore_controller_value == 1) {
    robot.start_controller(controller_group);
  }
  robot.destroy();
  return success ? 0 : 4;
}
