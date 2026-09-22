#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

namespace {

constexpr float kMaxAbsWrenchValue = 1.0e6F;
constexpr auto kStaleTimeout = std::chrono::milliseconds(1000);
constexpr auto kFreezeTimeout = std::chrono::milliseconds(2000);
constexpr auto kWarningInterval = std::chrono::milliseconds(1000);
constexpr auto kStatsInterval = std::chrono::seconds(5);

struct ForceReading {
  float fx = 0.0F;
  float fy = 0.0F;
  float fz = 0.0F;
  float mx = 0.0F;
  float my = 0.0F;
  float mz = 0.0F;
};

enum class DecodeResult {
  OK,
  PREFIX_MISMATCH,
  TERMINATOR_MISMATCH,
  INVALID_NUMERIC,
  ALL_ZERO,
};

struct DemoState {
  std::atomic<uint64_t> query_ok{0};
  std::atomic<uint64_t> query_fail{0};
  std::atomic<uint64_t> rx_1khz{0};
  std::atomic<uint64_t> rejected{0};
  std::atomic<uint64_t> reject_prefix{0};
  std::atomic<uint64_t> reject_terminator{0};
  std::atomic<uint64_t> reject_numeric{0};
  std::atomic<uint64_t> reject_all_zero{0};
  std::atomic<uint64_t> non_increasing_generation{0};
  std::atomic<uint64_t> duplicate_snapshots{0};
  std::atomic<uint64_t> valid{0};
  std::atomic<uint64_t> last_print_ms{0};
  std::atomic<uint64_t> last_valid_ms{0};
  std::atomic<uint64_t> last_generation{0};
  std::atomic<uint64_t> last_payload_change_ms{0};
  std::atomic<uint64_t> last_freeze_warning_ms{0};
  std::atomic<uint64_t> last_stale_warning_ms{0};
  std::mutex payload_mutex;
  std::array<uint8_t, 24> last_payload{};
  bool has_last_payload = false;
};

std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE> encode_query() {
  std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE> frame{};
  const uint32_t frame_type = 2;
  const uint32_t device_id = 233;
  const uint32_t data_size = 4;
  std::memcpy(frame.data(), &frame_type, sizeof(frame_type));
  std::memcpy(frame.data() + 4, &device_id, sizeof(device_id));
  std::memcpy(frame.data() + 8, &data_size, sizeof(data_size));
  frame[12] = 0x49;
  frame[13] = 0xAA;
  frame[14] = 0x0D;
  frame[15] = 0x0A;
  return frame;
}

DecodeResult decode_force(
    const std::array<uint8_t, END_TOOL_RAW_FRAME_SIZE>& frame, ForceReading& reading) {
  if ((frame[20] != 0x48 && frame[20] != 0x49) || frame[21] != 0xAA) {
    return DecodeResult::PREFIX_MISMATCH;
  }
  if (frame[46] != 0x0D || frame[47] != 0x0A) {
    return DecodeResult::TERMINATOR_MISMATCH;
  }

  ForceReading decoded;
  std::memcpy(&decoded.fx, frame.data() + 22, sizeof(float));
  std::memcpy(&decoded.fy, frame.data() + 26, sizeof(float));
  std::memcpy(&decoded.fz, frame.data() + 30, sizeof(float));
  std::memcpy(&decoded.mx, frame.data() + 34, sizeof(float));
  std::memcpy(&decoded.my, frame.data() + 38, sizeof(float));
  std::memcpy(&decoded.mz, frame.data() + 42, sizeof(float));

  const float values[] = {decoded.fx, decoded.fy, decoded.fz,
                          decoded.mx, decoded.my, decoded.mz};
  bool all_zero = true;
  for (const float value : values) {
    if (!std::isfinite(value) || std::fabs(value) > kMaxAbsWrenchValue) {
      return DecodeResult::INVALID_NUMERIC;
    }
    all_zero = all_zero && value == 0.0F;
  }
  if (all_zero) {
    return DecodeResult::ALL_ZERO;
  }

  reading = decoded;
  return DecodeResult::OK;
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

void count_decode_rejection(const std::shared_ptr<DemoState>& state, DecodeResult result) {
  state->rejected.fetch_add(1, std::memory_order_relaxed);
  switch (result) {
    case DecodeResult::PREFIX_MISMATCH:
      state->reject_prefix.fetch_add(1, std::memory_order_relaxed);
      break;
    case DecodeResult::TERMINATOR_MISMATCH:
      state->reject_terminator.fetch_add(1, std::memory_order_relaxed);
      break;
    case DecodeResult::INVALID_NUMERIC:
      state->reject_numeric.fetch_add(1, std::memory_order_relaxed);
      break;
    case DecodeResult::ALL_ZERO:
      state->reject_all_zero.fetch_add(1, std::memory_order_relaxed);
      break;
    case DecodeResult::OK:
      break;
  }
}

}  // namespace

int main(int argc, char** argv) {
  EndToolSide side = EndToolSide::LEFT;
  int duration_s = 10;
  int query_interval_ms = 10;
  int print_interval_ms = 100;
  try {
    if (!parse_side(argc > 1 ? argv[1] : "left", side)) {
      std::cerr << "Side must be left/right/l/r/0/1.\n";
      return 1;
    }
    duration_s = argc > 2 ? std::stoi(argv[2]) : duration_s;
    query_interval_ms = argc > 3 ? std::stoi(argv[3]) : query_interval_ms;
    print_interval_ms = argc > 4 ? std::stoi(argv[4]) : print_interval_ms;
  } catch (const std::exception&) {
    std::cerr << "Invalid numeric argument.\n";
    return 1;
  }
  if (duration_s <= 0 || query_interval_ms <= 0 || print_interval_ms <= 0) {
    std::cerr << "Usage: " << argv[0]
              << " [left|right] [duration_s>0] [query_interval_ms>0] [print_interval_ms>0]\n";
    return 1;
  }

  auto& robot = GalbotRobot::get_instance(MachineType::S1);
  if (!robot.init()) {
    std::cerr << "Failed to initialize the S1 SDK.\n";
    return 2;
  }
  std::cout << "WARNING: same-process raw TX is serialized, but there is no cross-client "
               "resource lease.\n";
  std::cout << "WBCS must run with endtool_raw_enabled=true.\n";

  auto state = std::make_shared<DemoState>();
  const uint64_t callback_handle = robot.register_endtool_raw_callback(
      [side, print_interval_ms, state](const EndToolRawData& raw) {
        if (raw.side != side || raw.kind != EndToolRxKind::BUFFER_1KHZ) {
          return;
        }
        state->rx_1khz.fetch_add(1, std::memory_order_relaxed);

        ForceReading reading;
        const DecodeResult decode_result = decode_force(raw.frame, reading);
        if (decode_result != DecodeResult::OK) {
          count_decode_rejection(state, decode_result);
          return;
        }

        uint64_t previous_generation = state->last_generation.load(std::memory_order_relaxed);
        while (true) {
          if (previous_generation != 0 && raw.generation <= previous_generation) {
            state->non_increasing_generation.fetch_add(1, std::memory_order_relaxed);
            return;
          }
          if (state->last_generation.compare_exchange_weak(
                  previous_generation, raw.generation, std::memory_order_relaxed,
                  std::memory_order_relaxed)) {
            break;
          }
        }

        const uint64_t now_ms = steady_now_ms();
        bool duplicate = false;
        {
          std::lock_guard<std::mutex> lock(state->payload_mutex);
          duplicate = state->has_last_payload &&
                      std::equal(state->last_payload.begin(), state->last_payload.end(),
                                 raw.frame.begin() + 22);
          if (!duplicate) {
            std::copy(raw.frame.begin() + 22, raw.frame.begin() + 46,
                      state->last_payload.begin());
            state->has_last_payload = true;
          }
        }
        if (!duplicate) {
          state->last_payload_change_ms.store(now_ms, std::memory_order_relaxed);
        } else {
          state->duplicate_snapshots.fetch_add(1, std::memory_order_relaxed);
          const uint64_t last_change_ms =
              state->last_payload_change_ms.load(std::memory_order_relaxed);
          uint64_t last_warning_ms =
              state->last_freeze_warning_ms.load(std::memory_order_relaxed);
          if (last_change_ms != 0 &&
              now_ms - last_change_ms >= static_cast<uint64_t>(kFreezeTimeout.count()) &&
              (last_warning_ms == 0 ||
               now_ms - last_warning_ms >= static_cast<uint64_t>(kWarningInterval.count())) &&
              state->last_freeze_warning_ms.compare_exchange_strong(
                  last_warning_ms, now_ms, std::memory_order_relaxed,
                  std::memory_order_relaxed)) {
            std::cerr << "Force payload unchanged for " << (now_ms - last_change_ms)
                      << "ms while generation advances. This may be a held TIB snapshot; an "
                         "exactly steady sensor can also produce identical bytes.\n";
          }
          return;
        }

        state->last_valid_ms.store(now_ms, std::memory_order_relaxed);
        state->valid.fetch_add(1, std::memory_order_relaxed);
        uint64_t previous_print = state->last_print_ms.load(std::memory_order_relaxed);
        if (now_ms - previous_print < static_cast<uint64_t>(print_interval_ms) ||
            !state->last_print_ms.compare_exchange_strong(
                previous_print, now_ms, std::memory_order_relaxed,
                std::memory_order_relaxed)) {
          return;
        }
        std::cout << "generation=" << raw.generation << " Fx=" << reading.fx
                  << " Fy=" << reading.fy << " Fz=" << reading.fz << "kg Mx=" << reading.mx
                  << " My=" << reading.my << " Mz=" << reading.mz << "kg*m\n";
      });
  if (callback_handle == 0) {
    std::cerr << "Failed to register the raw RX callback.\n";
    robot.destroy();
    return 3;
  }

  const auto query = encode_query();
  const auto query_interval = std::chrono::milliseconds(query_interval_ms);
  const auto started_at = std::chrono::steady_clock::now();
  const auto deadline = started_at + std::chrono::seconds(duration_s);
  auto next_query_at = started_at;
  auto next_stats_at = started_at + kStatsInterval;
  auto last_stats_at = started_at;
  uint64_t last_stats_query_count = 0;
  uint64_t last_stats_rx_count = 0;

  while (true) {
    const auto before_sleep = std::chrono::steady_clock::now();
    if (before_sleep < next_query_at) {
      std::this_thread::sleep_until(next_query_at);
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }

    const auto result = robot.send_endtool_raw_frame(side, query);
    if (result == ControlStatus::SUCCESS) {
      state->query_ok.fetch_add(1, std::memory_order_relaxed);
    } else {
      const uint64_t failures = state->query_fail.fetch_add(1, std::memory_order_relaxed) + 1;
      if (failures == 1 || failures % 100 == 0) {
        std::cerr << "Query publish failed: status=" << static_cast<int>(result)
                  << " failures=" << failures << '\n';
      }
    }

    const auto after_query = std::chrono::steady_clock::now();
    next_query_at += query_interval;
    // Never catch up with a burst after Publish blocks; doing so only increases FIFO pressure.
    if (next_query_at <= after_query) {
      next_query_at = after_query + query_interval;
    }

    const uint64_t now_ms = steady_now_ms();
    const uint64_t last_valid_ms = state->last_valid_ms.load(std::memory_order_relaxed);
    const uint64_t stale_for_ms =
        last_valid_ms == 0
            ? static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                        after_query - started_at)
                                        .count())
            : now_ms - last_valid_ms;
    uint64_t last_stale_warning_ms =
        state->last_stale_warning_ms.load(std::memory_order_relaxed);
    if (stale_for_ms >= static_cast<uint64_t>(kStaleTimeout.count()) &&
        (last_stale_warning_ms == 0 ||
         now_ms - last_stale_warning_ms >= static_cast<uint64_t>(kWarningInterval.count())) &&
        state->last_stale_warning_ms.compare_exchange_strong(
            last_stale_warning_ms, now_ms, std::memory_order_relaxed,
            std::memory_order_relaxed)) {
      std::cerr << "No fresh force response for " << stale_for_ms
                << "ms: query_ok=" << state->query_ok.load(std::memory_order_relaxed)
                << " query_fail=" << state->query_fail.load(std::memory_order_relaxed)
                << " rx_1khz=" << state->rx_1khz.load(std::memory_order_relaxed)
                << " valid=" << state->valid.load(std::memory_order_relaxed)
                << ". Check TIB, sensor power/wiring/baud rate, "
                   "endtool_raw_enabled and WBCS logs.\n";
    }

    if (after_query >= next_stats_at) {
      const uint64_t query_count = state->query_ok.load(std::memory_order_relaxed);
      const uint64_t rx_count = state->rx_1khz.load(std::memory_order_relaxed);
      const double elapsed_s = std::chrono::duration<double>(after_query - last_stats_at).count();
      std::cout << "Rate: query_hz="
                << static_cast<double>(query_count - last_stats_query_count) / elapsed_s
                << " rx_hz=" << static_cast<double>(rx_count - last_stats_rx_count) / elapsed_s
                << " fresh=" << state->valid.load(std::memory_order_relaxed)
                << " duplicates=" << state->duplicate_snapshots.load(std::memory_order_relaxed)
                << " rejects(prefix=" << state->reject_prefix.load(std::memory_order_relaxed)
                << " terminator=" << state->reject_terminator.load(std::memory_order_relaxed)
                << " numeric=" << state->reject_numeric.load(std::memory_order_relaxed)
                << " zero=" << state->reject_all_zero.load(std::memory_order_relaxed) << ")\n";
      last_stats_query_count = query_count;
      last_stats_rx_count = rx_count;
      last_stats_at = after_query;
      next_stats_at = after_query + kStatsInterval;
    }
  }

  robot.unregister_endtool_raw_callback(callback_handle);
  std::cout << "Stopped: query_ok=" << state->query_ok.load()
            << " query_fail=" << state->query_fail.load()
            << " rx_1khz=" << state->rx_1khz.load()
            << " rejected=" << state->rejected.load()
            << " (prefix=" << state->reject_prefix.load()
            << " terminator=" << state->reject_terminator.load()
            << " numeric=" << state->reject_numeric.load()
            << " zero=" << state->reject_all_zero.load()
            << " non_increasing_generation=" << state->non_increasing_generation.load() << ")"
            << " fresh=" << state->valid.load()
            << " duplicates=" << state->duplicate_snapshots.load() << '\n';
  robot.destroy();
  return 0;
}
