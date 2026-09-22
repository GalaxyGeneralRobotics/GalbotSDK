import logging
import logging.handlers
import os
import threading
from typing import Dict, Any

# =============================
# 全局状态
# =============================

_user_logger = None
_logger_lock = threading.Lock()
_logger_inited = False

# =============================
# 日志等级映射
# =============================

LEVEL_MAP = {
    "debug": logging.DEBUG,
    "info": logging.INFO,
    "warn": logging.WARNING,
    "warning": logging.WARNING,
    "error": logging.ERROR,
    "critical": logging.CRITICAL,
}


# =============================
# 工具函数
# =============================

def parse_level(level: Any) -> int:
    if isinstance(level, str):
        return LEVEL_MAP.get(level.lower(), logging.INFO)
    if isinstance(level, int):
        return max(logging.NOTSET, min(level, logging.CRITICAL))
    return logging.INFO


def make_default_log_file() -> str:
    import sys
    import threading
    from datetime import datetime

    process_name = os.path.basename(sys.argv[0])
    process_name = os.path.splitext(process_name)[0] or "python"

    current_time = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    pid = os.getpid()
    tid = threading.get_ident()

    return f"{process_name}_{current_time}_{pid}_{tid}.log"


def _fallback_log(level: str, msg: str):
    """当用户未 init_logger 时兜底输出"""
    print(f"[galbot_sdk][{level}] {msg}")


# =============================
# 初始化日志
# =============================

def init_logger(cfg: Dict[str, Any]) -> bool:
    """
    初始化用户日志系统（线程安全，只允许初始化一次）

    cfg:
        path
        file_name
        file_max_size / file_max_size_mb
        file_max_num
        console_output
        level
    """
    global _user_logger, _logger_inited

    with _logger_lock:
        if _logger_inited:
            return True

        try:
            # -------------------------
            # 日志目录
            # -------------------------
            default_log_dir = os.path.join(
                os.path.expanduser("~"),
                "galbot_sdk_log",
                "user_log"
            )

            log_dir = cfg.get("path") or default_log_dir
            os.makedirs(log_dir, exist_ok=True)

            # -------------------------
            # 文件名
            # -------------------------
            file_name = cfg.get("file_name") or make_default_log_file()
            file_name = os.path.basename(file_name)  # 安全处理
            log_path = os.path.join(log_dir, file_name)

            # -------------------------
            # 文件大小
            # -------------------------
            file_max_size = cfg.get("file_max_size")
            if file_max_size is None and "file_max_size_mb" in cfg:
                file_max_size = int(cfg["file_max_size_mb"]) * 1024 * 1024
            if file_max_size is None:
                file_max_size = 10 * 1024 * 1024  # 默认 10MB

            # -------------------------
            # 文件数量
            # -------------------------
            file_max_num = int(cfg.get("file_max_num", 5))

            # -------------------------
            # 控制台
            # -------------------------
            console_output = bool(cfg.get("console_output", False))

            # -------------------------
            # 等级
            # -------------------------
            level = parse_level(cfg.get("level", logging.INFO))

            # -------------------------
            # 创建 logger
            # -------------------------
            logger = logging.getLogger("galbot_user_logger")
            logger.setLevel(level)
            logger.propagate = False
            logger.disabled = False

            # 清理旧 handler（第一次初始化时通常为空）
            for h in logger.handlers:
                try:
                    h.close()
                except Exception:
                    pass
            logger.handlers.clear()

            fmt = logging.Formatter(
                "[%(asctime)s] [%(levelname)s] %(filename)s:%(lineno)d: %(message)s"
            )

            # 文件 handler
            file_handler = logging.handlers.RotatingFileHandler(
                log_path,
                maxBytes=file_max_size,
                backupCount=file_max_num,
                encoding="utf-8",
            )
            file_handler.setFormatter(fmt)
            logger.addHandler(file_handler)

            # 控制台 handler
            if console_output:
                console_handler = logging.StreamHandler()
                console_handler.setFormatter(fmt)
                logger.addHandler(console_handler)

            _user_logger = logger
            _logger_inited = True
            return True

        except Exception as e:
            print(f"[galbot_sdk][init_logger] failed: {e}")
            return False


# =============================
# 对外日志接口
# =============================

def debug(msg: str):
    if _user_logger:
        _user_logger.debug(msg, stacklevel=2)
    else:
        _fallback_log("DEBUG", msg)


def info(msg: str):
    if _user_logger:
        _user_logger.info(msg, stacklevel=2)
    else:
        _fallback_log("INFO", msg)


def warning(msg: str):
    if _user_logger:
        _user_logger.warning(msg, stacklevel=2)
    else:
        _fallback_log("WARNING", msg)


def error(msg: str):
    if _user_logger:
        _user_logger.error(msg, stacklevel=2)
    else:
        _fallback_log("ERROR", msg)


def critical(msg: str):
    if _user_logger:
        _user_logger.critical(msg, stacklevel=2)
    else:
        _fallback_log("CRITICAL", msg)
