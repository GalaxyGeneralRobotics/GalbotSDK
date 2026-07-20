import sys
import importlib.util
from pathlib import Path
import platform

# SDK 版本号（构建时自动注入）
__version__ = "1.9.1"
version = __version__

from .galbot_sdk_logger import init_logger, debug, info, warning, error, critical

# 确定 Python 版本和平台架构
_python_version = f"{sys.version_info.major}{sys.version_info.minor}"

_machine = platform.machine().lower()
if _machine in ("x86_64", "amd64"):
    _arch = "x86_64-linux-gnu"
elif _machine in ("aarch64", "arm64"):
    _arch = "aarch64-linux-gnu"
else:
    raise RuntimeError(f"Unsupported architecture: {_machine}")

# 查找共享库
_so_filename = f"galbot_sdk.cpython-{_python_version}-{_arch}.so"

_current_dir = Path(__file__).parent
_so_path = _current_dir / _so_filename

# 库文件不存在时的错误处理
if not _so_path.exists():
    raise ImportError(
        "[galbot][ERROR] galbot_sdk shared library not found.\n"
        f"  Expected file : {_so_filename}\n"
        f"  Python        : {_python_version}\n"
        f"  Architecture  : {_arch}\n"
        f"  Search path   : {_current_dir}\n"
        "This usually means:\n"
        "  - Python version mismatch\n"
        "  - Architecture mismatch\n"
    )

# 动态加载库
spec = importlib.util.spec_from_file_location("galbot_sdk", _so_path)
if spec is None or spec.loader is None:
    raise ImportError(f"Failed to load galbot_sdk module from {_so_path}")

_sdk_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(_sdk_module)

# 导出库中的所有公共符号
for _name in dir(_sdk_module):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_sdk_module, _name)

# 清理临时变量
del _sdk_module, _current_dir, _python_version, _arch, _so_filename, _so_path
