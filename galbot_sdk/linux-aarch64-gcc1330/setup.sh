#!/usr/bin/env bash
# Source this script from Bash or Zsh | 使用 Bash 或 Zsh source 此脚本

get_script_source() {
    if [ -n "${BASH_VERSION:-}" ]; then
        printf '%s\n' "${BASH_SOURCE[0]}"
        return
    fi

    if [ -n "${ZSH_VERSION:-}" ]; then
        eval 'printf "%s\n" "${(%):-%x}"'
        return
    fi

    printf '%s\n' "$0"
}

# Get the absolute path of the directory where this script is located | 获取脚本所在目录的绝对路径
SCRIPT_SOURCE="$(get_script_source)"
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT_SOURCE")" && pwd)"

# Add SDK binary path to PATH for command-line tools | 将 SDK 工具目录添加到 PATH
export PATH="$SCRIPT_DIR/bin${PATH:+:$PATH}"

# Add SDK library path to LD_LIBRARY_PATH for finding shared libraries | 将 SDK 库路径添加到 LD_LIBRARY_PATH 以查找依赖动态库
export LD_LIBRARY_PATH="$SCRIPT_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# Add SDK Python module path to PYTHONPATH for importing Python bindings | 将 SDK Python 模块路径添加到 PYTHONPATH
export PYTHONPATH="$SCRIPT_DIR/lib/python${PYTHONPATH:+:$PYTHONPATH}"
