#include <chrono>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>
#include <thread>

#include "galbot_robot.hpp"

using namespace galbot::sdk;

// Added command help table structure
struct CommandInfo {
  std::string cmd;
  std::string description;
  std::string parameters;
  std::string example;
};

// Command help table
const std::vector<CommandInfo> COMMAND_TABLE = {
    {"start_microphone_stream_input", "Start microphone streaming audio input feature", "No parameters", "start_microphone_stream_input"},
    {"stop_microphone_stream_input", "Stop the specified microphone streaming audio input", "No parameters", "stop_microphone_stream_input"},
    {"write_audio_stream_output", "PCM audiodata audio output", "Audio file name to play (16K 16bit Mono PCM audio file path)", "write_audio_stream_output"},
    {"stop_audio_stream_output", "Stop the specified audio output stream or all active audio output streams", "No parameters", "stop_audio_stream_output"},
    {"set_volume", "Set system global volume", "Volume setting parameter (0-100)", "set_volume"},
    {"get_volume", "Get current system global volume", "No parameters", "get_volume"},
    {"help", "Show help", "No parameters", "help"},
    {"q", "Exit program", "No parameters", "q"}};

// command
void printCommandHelp() {
  std::cout << "\n================ Command Help Table =================" << std::endl;
  std::cout << std::left << std::setw(30) << "Command" << std::setw(30) << "Description" << std::setw(30) << "Parameters"
            << std::setw(30) << "example" << std::endl;
  std::cout << std::string(110, '-') << std::endl;

  for (const auto& cmd : COMMAND_TABLE) {
    std::cout << std::left << std::setw(40) << cmd.cmd << std::setw(60) << cmd.description << std::setw(40)
              << cmd.parameters << std::setw(30) << cmd.example << std::endl;
  }
  std::cout << "=============================================\n" << std::endl;
}


void save_audio_to_file(const std::string& filename, const std::vector<uint8_t>& audio_data) {
  if (audio_data.empty()) {
    return;
  }
  std::ofstream outfile(filename, std::ios::app | std::ios::binary);
  if (!outfile) {
    std::cout << "Failed to open file for writing: " << filename << std::endl;
    return;
  }
  outfile.write(reinterpret_cast<const char*>(audio_data.data()),
                static_cast<std::streamsize>(audio_data.size()));
}

void handle_audio_data(const std::shared_ptr<AudioData>& audio_data) {
  if (audio_data == nullptr) {
    return;
  }

  const int64_t timestamp_ns = audio_data->header.timestamp_ns;
  (void)timestamp_ns;

  const std::string& audio_type = audio_data->type;
  const auto data_size = audio_data->data.size();
  (void)audio_data->format;

  if (audio_type == "vad_begin") {
    std::cout << "\n[VAD_BEGIN] Voice activity detection started" << std::endl;
  } else if (audio_type == "vad_end") {
    std::cout << "[VAD_END] Voice activity detection ended" << std::endl;
  } else if (audio_type == "vad_chunk") {
    // std::cout << "[AUDIO_CHUNK] Received " << data_size << " bytes of " << audio_format
    //           << " audio data (timestamp_ns: " << timestamp_ns << ")" << std::endl;
    save_audio_to_file("mic_vad.pcm", audio_data->data);
  } else if (audio_type == "denoise_chunk") {
    // std::cout << "[DENOISE_AUDIO] Received " << data_size << " bytes of denoised audio (timestamp_ns: "
    //           << timestamp_ns << ")" << std::endl;
    save_audio_to_file("mic_denoise.pcm", audio_data->data);
  } else if (audio_type == "waken_up") {
    std::string payload(audio_data->data.begin(), audio_data->data.end());
    std::cout << "\n[WAKEN_UP] Wake-up event triggered: " << payload << std::endl;
  } else {
    std::cout << "[" << audio_type << "] Received audio data, size: " << data_size << " bytes" << std::endl;
  }
}


// Safely read numeric inputs of multiple types and handle input errors
template<typename T>
bool safe_read_number(T& value, const std::string& prompt = "") {
  if (!prompt.empty()) {
    std::cout << prompt;
  }

  std::string input;
  std::getline(std::cin, input);

  // Check input stream status
  if (std::cin.fail() && !std::cin.eof()) {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Input error, please enter a valid number." << std::endl;
    return false;
  }

  // Trim leading and trailing spaces
  input.erase(0, input.find_first_not_of(" \t"));
  input.erase(input.find_last_not_of(" \t") + 1);

  if (input.empty()) {
    std::cout << "Input error, please enter a valid number." << std::endl;
    return false;
  }

  try {
    size_t pos;
    T result;

    if (std::is_integral<T>()) {
      result = static_cast<T>(std::stoll(input, &pos));
    } else if (std::is_floating_point<T>()) {
      result = static_cast<T>(std::stold(input, &pos));
    }

    // Check whether the entire string has been converted
    if (pos != input.length()) {
      std::cout << "Input error, please enter a valid number." << std::endl;
      return false;
    }

    value = result;
    return true;
  } catch (const std::exception&) {
    std::cout << "Input error, please enter a valid number." << std::endl;
    return false;
  }
}


// Safely read filename input and handle input errors
bool safe_read_file_name(std::string& file_name, const std::string& prompt = "") {
  if (!prompt.empty()) {
    std::cout << prompt;
  }

  std::getline(std::cin, file_name);

  // Check input stream status
  if (std::cin.fail() && !std::cin.eof()) {
    std::cin.clear();
    std::cin.ignore();
  }
  return true;
}

int main(int argc, char* argv[]) {
  std::cout << "start" << std::endl;
  // Get the base_instance instance
  auto& base_instance = GalbotRobot::get_instance(MachineType::G3);

  if (!base_instance.init()) {
    std::cout << "base_instance init fail" << std::endl;
    return -1;
  }

  // Waiting for init done
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  printCommandHelp();

  std::string cmd, stream_id;
  while (base_instance.is_running()) {
    std::cout << "Enter interface name to test:";
    std::string input_line;
    std::getline(std::cin, input_line);

    // Extract the first word as the command (ignore subsequent arguments)
    std::istringstream iss(input_line);
    iss >> cmd;

    // If input is empty, continue the loop
    if (cmd.empty()) {
      continue;
    }
    if (cmd == "help") {
      printCommandHelp();
      continue;
    } else if (cmd == "start_microphone_stream_input") {
      stream_id = base_instance.start_microphone_stream_input([](const std::shared_ptr<AudioData> audio_data){
        handle_audio_data(audio_data);
      });

      std::cout << "Start microphone stream input, stream_id: "<< stream_id << std::endl;
    } else if (cmd == "stop_microphone_stream_input") {
      base_instance.stop_microphone_stream_input(stream_id);
      std::cout << "Stop microphone stream input, stream_id : " << stream_id << std::endl;
    } else if (cmd == "write_audio_stream_output") {
      std::string play_file;
      safe_read_file_name(play_file, "Please enter audio file name to play (16K 16bit Mono PCM audio file path): ");

      const auto chunk_size = 2560; // Data block size of 2560 bytes
      std::vector<char> audio_chunk(chunk_size, 0);
      std::cout << "Write audio stream data..." << std::endl;
      std::ifstream infile(play_file, std::ios::binary);
      if (!infile) {
        std::cout << "Failed to open for reading: " << play_file << std::endl;
        continue;
      } else {
        stream_id = "spk_stream_output_test";
        while (infile) {
          infile.read(audio_chunk.data(), static_cast<std::streamsize>(chunk_size));
          std::streamsize bytes_read = infile.gcount();
          if (bytes_read <= 0) break;
          std::string out_chunk(audio_chunk.data(), bytes_read);
          base_instance.write_audio_stream_output(out_chunk, stream_id);
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }
      std::cout << "Write audio stream output, stream id : " << stream_id << std::endl;
    } else if (cmd == "stop_audio_stream_output") {
      base_instance.stop_audio_stream_output(stream_id);
      std::cout << "Stop audio stream output, stream id: " << stream_id << std::endl;
    } else if (cmd == "set_volume") {
      float volume = 60;
      if (!safe_read_number(volume, "Please enter volume setting parameter (0-100): ")) {
        continue;
      }

      if (!base_instance.set_volume(volume)) {
        std::cout << "Failed to set volume: " << volume << std::endl;
        continue;
      }
      std::cout << "Set current volume: " << volume << std::endl;
    } else if (cmd == "get_volume") {
      auto volume = base_instance.get_volume();
      std::cout << "Get current volume: " << volume << std::endl;
    } else if (cmd == "q") {
      break;
    } else {
      std::cout << "Invalid input" << std::endl;
    }
  }

  base_instance.request_shutdown();
  base_instance.wait_for_shutdown();
  base_instance.destroy();

  std::cout << "proccess end" << std::endl;
  return 0;
}
