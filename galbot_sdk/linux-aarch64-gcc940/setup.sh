#!/bin/bash
# Use Bash to execute this script | 使用 Bash 执行此脚本

# Get the absolute path of the directory where this script is located | 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Add SDK library path to LD_LIBRARY_PATH for finding shared libraries | 将 SDK 库路径添加到 LD_LIBRARY_PATH 以查找依赖动态库
export LD_LIBRARY_PATH=$SCRIPT_DIR/lib:$LD_LIBRARY_PATH

# Add SDK Python module path to PYTHONPATH for importing Python bindings | 将 SDK Python 模块路径添加到 PYTHONPATH
export PYTHONPATH=$SCRIPT_DIR/lib/python:$PYTHONPATH