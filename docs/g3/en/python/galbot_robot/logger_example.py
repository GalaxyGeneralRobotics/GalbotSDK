import galbot_sdk

cfg = {
    # Log storage directory; if empty, defaults to ~/galbot_sdk_log/user_log
    "path": "",

    # Log file name; if empty, defaults to <process_name>_<current_time>_<pid>_<thread_id>.log
    "file_name": "",

    # log bytes, log size,
    "file_max_size": 10 * 1024 * 1024,  # 10MB

    # Maximum rotating log files; oldest file is overwritten when limit exceeded
    "file_max_num": 5,

    # Whether to output logs to console; default is False
    "console_output": True,

    # Log output level, available values: debug, info, warning, error, critical
    "level": "info",
}

galbot_sdk.init_logger(cfg)

# Write log
galbot_sdk.debug("Debug example")
galbot_sdk.info("Info example")
galbot_sdk.warning("Warning example")
galbot_sdk.error("Error example")
galbot_sdk.critical("Critical example")
