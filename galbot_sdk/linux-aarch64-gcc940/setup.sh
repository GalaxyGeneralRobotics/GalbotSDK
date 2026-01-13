#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export LD_LIBRARY_PATH=$SCRIPT_DIR/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$SCRIPT_DIR/lib/python:$PYTHONPATH