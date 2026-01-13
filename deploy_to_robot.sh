#!/bin/bash
read -p "Enter robot user: " ROBOT_USER
read -p "Enter robot IP: " ROBOT_IP
REMOTE_DIR="/data/galbot/lib"

# 本地路径
LIB_DIR="./galbot_sdk/linux-aarch64-gcc940/lib"

# 拷贝C++库，保留软链接
rsync -avz -e ssh --links "$LIB_DIR"/libgalbot_g1_sdk.* "$ROBOT_USER@$ROBOT_IP:$REMOTE_DIR/"

# 拷贝python库
rsync -avz -e ssh --links "$LIB_DIR/python/galbot_sdk" "$ROBOT_USER@$ROBOT_IP:$REMOTE_DIR"
