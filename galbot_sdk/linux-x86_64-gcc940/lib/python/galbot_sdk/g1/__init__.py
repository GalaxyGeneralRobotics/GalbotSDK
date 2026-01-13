import sys
import importlib.util
from pathlib import Path
import platform

# 确定确定平台架构与pyhton版本
_python_version = f"{sys.version_info.major}{sys.version_info.minor}"

_machine = platform.machine().lower()
if _machine in ("x86_64", "amd64"):
    _arch = "x86_64-linux-gnu"
elif _machine in ("aarch64", "arm64"):
    _arch = "aarch64-linux-gnu"
else:
    raise RuntimeError(f"Unsupported architecture: {_machine}")

# 查找相应库
_so_filename = f"galbot_g1.cpython-{_python_version}-{_arch}.so"

_current_dir = Path(__file__).parent
_so_path = _current_dir / _so_filename
if not _so_path.exists():
    matches = list(_current_dir.glob("galbot_g1.cpython-*.so"))
    if matches:
        _so_path = matches[0]
    else:
        raise ImportError(
            f"Cannot find galbot_g1 shared library for Python {_python_version} "
            f"and arch {_arch}. Searched in: {_current_dir}"
        )

# 加载动态库
spec = importlib.util.spec_from_file_location("g1", _so_path)
if spec is None or spec.loader is None:
    raise ImportError(f"Failed to load galbot_g1 module from {_so_path}")

_g1_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(_g1_module)

for _name in dir(_g1_module):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_g1_module, _name)

# 清理临时变量
del _g1_module, _current_dir, _python_version, _arch, _so_filename, _so_path
