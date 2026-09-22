#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import struct
import math
import os
from galbot_sdk.g3 import GalbotRobot

# global variable
audio_callback_count = 0
last_audio_type = None

# Command help table
COMMAND_TABLE = [
    {
        "cmd": "start_microphone_stream_input",
        "description": "Start microphone streaming audio input feature",
        "parameters": "No parameters",
        "example": "start_microphone_stream_input"
    },
    {
        "cmd": "stop_microphone_stream_input",
        "description": "Stop the specified microphone streaming audio input",
        "parameters": "No parameters",
        "example": "stop_microphone_stream_input"
    },
    {
        "cmd": "write_audio_stream_output",
        "description": "PCM audiodata audio output",
        "parameters": "Enter audio file name to play (16K 16bit Mono PCM audio file path):",
        "example": "write_audio_stream_output"
    },
    {
        "cmd": "stop_audio_stream_output",
        "description": "Stop the specified audio output stream or all active audio output streams",
        "parameters": "No parameters",
        "example": "stop_audio_stream_output"
    },
    {
        "cmd": "set_volume",
        "description": "Set system global volume",
        "parameters": "Please enter volume setting parameter (0-100)",
        "example": "set_volume"
    },
    {
        "cmd": "get_volume",
        "description": "Get current system global volume",
        "parameters": "No parameters",
        "example": "get_volume"
    },
    {
        "cmd": "help",
        "description": "Show help",
        "parameters": "No parameters",
        "example": "help"
    },
    {
        "cmd": "q",
        "description": "Exit program",
        "parameters": "No parameters",
        "example": "q"
    }
]

# command
def print_command_help():
    print("\n================ Command Help Table =================")
    print(f"{'Command':<40} | {'Description':<30} | {'Parameters':<20} | {'example':<30}")
    print("-" * 130)
    for cmd in COMMAND_TABLE:
        print(
            f"{cmd['cmd']:<40} | {cmd['description']:<30} | {cmd['parameters']:<20} | {cmd['example']:<30}")
    print("=============================================\n")


# Safely read float input
def safe_read_float(prompt: str = "") -> float:
    """
    Safely get float input

    Args:
        prompt: prompt text

    Returns:
        Valid float input
    """
    if prompt:
        print(prompt, end='', flush=True)
    
    try:
        value = float(input())
        return value
    except ValueError:
        print("Input error, please enter a valid float.")
        return None
    except EOFError:
        print("\nInput ended")
        return None


# Safely read integer input
def safe_read_int(prompt: str = "") -> int:
    """
    Safely get integer input

    Args:
        prompt: prompt text

    Returns:
        Valid integer input
    """
    if prompt:
        print(prompt, end='', flush=True)
    
    try:
        value = int(input())
        return value
    except ValueError:
        print("Input error, please enter a valid integer.")
        return None
    except EOFError:
        print("\nInput ended")
        return None

# Safely read filename input and handle input errors
def safe_read_filename(prompt: str = "") -> str:
    """
    Safely get filename input

    Args:
        prompt: prompt text

    Returns:
        Valid filename input
    """
    if prompt:
        print(prompt, end='', flush=True)
    
    try:
        filename = input()
        return filename
    except ValueError:
        print("Input error, please enter a valid file name.")

# Save audio data
def save_audio_to_file(filename: str, audio_data: bytes):
    """
    Save audio data

    Args:
        filename: file name
        audio_data: audio data (bytes)
    """
    try:
        with open(filename, 'ab') as f:
            f.write(audio_data)
        # print(f"✓ Audio appended and saved to {filename}")
    except Exception as e:
        print(f"✗ Failed to save file: {e}")


# Read audio data from file
def read_audio_from_file(filename: str, chunk_size: int = 2560) -> list:
    """
    Read audio data from file

    Args:
        filename: file name
        chunk_size: chunk size (bytes)

    Returns:
        Audio data block list
    """
    try:
        audio_chunks = []
        with open(filename, 'rb') as f:
            while True:
                chunk = f.read(chunk_size)
                if not chunk:
                    break
                audio_chunks.append(chunk)
        print(f"✓ Read {len(audio_chunks)} chunks from {filename}")
        return audio_chunks
    except FileNotFoundError:
        print(f"✗ File not found: {filename}")
        return []
    except Exception as e:
        print(f"✗ Failed to read file: {e}")
        return []


# Audio callback function
def audio_callback(audio_data: dict):
    """
    Audio data callback function

    Args:
        audio_data: audio data dictionary including header, type, format, data
    """
    global audio_callback_count, last_audio_type
    
    if not audio_data:
        return

    header = audio_data.get("header", {})
    timestamp_ns = header.get("timestamp_ns", 0)

    audio_type = audio_data.get("type", "unknown")
    audio_format = audio_data.get("format", "unknown")
    data_size = len(audio_data.get("data", b""))

    audio_callback_count += 1
    last_audio_type = audio_type

    # Print audio data info
    if audio_type == "vad_begin":
        print(f"\n[VAD_BEGIN] Voice activity detection started")
        pass
    elif audio_type == "vad_end":
        print(f"[VAD_END] Voice activity detection ended")
        pass
    elif audio_type == "vad_chunk":
        # print(f"[AUDIO_CHUNK] Received {data_size} bytes of {audio_format} audio data (timestamp_ns: {timestamp_ns})")
        save_audio_to_file("mic_vad.pcm", audio_data.get("data", b""))
        pass
    elif audio_type == "denoise_chunk":
        # print(f"[DENOISE_AUDIO] Received {data_size} bytes of denoised audio (timestamp_ns: {timestamp_ns})")
        save_audio_to_file("mic_denoise.pcm", audio_data.get("data", b""))
        pass
    elif audio_type == "waken_up":
        print(f"\n[WAKEN_UP] Wake-up event triggered: ", audio_data.get("data", ""))
        pass
    else:
        print(f"[{audio_type}] Received audio data, size: {data_size} bytes")
        pass


def main():
    # Get GalbotRobot singleton
    base_instance = GalbotRobot()

    ok = base_instance.init()
    if not ok:
        print("Initialization failed")
        return -1

    # Waiting for init done
    time.sleep(5)

    print("\n========================================")
    print("     Galbot audio test tool started")
    print("========================================\n")

    print_command_help()

    microphone_stream_id = ""
    speaker_stream_id = "speaker_audio_output_test"
    while base_instance.is_running():
        print("Enter operation to execute:", end='', flush=True)
        cmd_line = input().strip()

        if not cmd_line:
            continue

        parts = cmd_line.split()
        cmd = parts[0]

        try:
            if cmd == "help":
                print_command_help()

            elif cmd == "start_microphone_stream_input":
                print("Starting microphone streaming audio input...")
                global audio_callback_count, last_audio_type
                
                microphone_stream_id = base_instance.start_microphone_stream_input(
                    callback=audio_callback,
                    chunk_size=2560,
                    use_raw_audio=False
                )
                audio_callback_count = 0
                last_audio_type = None
                print(f"✓ Microphone started successfully, stream ID: {microphone_stream_id}")

            elif cmd == "stop_microphone_stream_input":
                if microphone_stream_id is None:
                    print("No active microphone stream")
                else:
                    print(f"Stopping microphone stream (ID: {microphone_stream_id})...")
                    base_instance.stop_microphone_stream_input(microphone_stream_id)
                    print(f"✓ Microphone stopped")
                    print(f"  - Number of callbacks: {audio_callback_count}")
                    print(f"  - Last audio type: {last_audio_type}")
                    microphone_stream_id = None

            elif cmd == "write_audio_stream_output":
                audio_output_file = safe_read_filename("Enter audio file name to play (16K 16bit Mono PCM audio file path): ")
                print("Reading audio file...")
                # Read and play audio
                audio_chunks = read_audio_from_file(audio_output_file, chunk_size=2560)
                if audio_chunks:
                    print(f"Playing audio stream ID: {speaker_stream_id}")
                    
                    for i, chunk in enumerate(audio_chunks):
                        success = base_instance.write_audio_stream_output(chunk, speaker_stream_id)
                        if not success:
                            print(f"✗ Playback failed for chunk {i+1}")
                            break

                        time.sleep(0.05)

                    print(f"✓ Audio playback completed")

            elif cmd == "stop_audio_stream_output":
                print("Stopping audio output stream...")
                base_instance.stop_audio_stream_output(speaker_stream_id)
                print("✓ Audio output stream stopped")

            elif cmd == "set_volume":
                volume = safe_read_float("Please enter volume setting parameter (0-100): ")
                if volume is None or volume < 0 or volume > 100:
                    print(f"✗ Volume must be between 0-100; input value: {volume}")
                    continue

                if (not base_instance.set_volume(volume)):
                    print(f"✗ Failed to set volume({volume})")
                print(f"✓ Volume set to: {volume}%")

            elif cmd == "get_volume":
                volume = base_instance.get_volume()
                print(f"Current system volume: {volume}%")

            elif cmd == "q":
                print("Program exiting...")
                break

            else:
                print(f"Unknown command: {cmd}, enter 'help' to see available commands")

        except (ValueError, TypeError) as e:
            print(f"Parameters error: {e}")
        except Exception as e:
            print(f"Command execution exception: {e}")

    base_instance.request_shutdown()
    base_instance.wait_for_shutdown()
    base_instance.destroy()
    print("Program exited")


if __name__ == "__main__":
    main()
