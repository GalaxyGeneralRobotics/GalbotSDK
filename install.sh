#!/bin/bash
set -e

DEFAULT_PATH="/opt/galbot"

echo "=========================================="
echo " GALBOT_G1_SDK 安装程序"
echo "=========================================="
echo

read -p "install to [$DEFAULT_PATH]? (enter, or input other path): " INSTALL_PATH
if [ -z "$INSTALL_PATH" ]; then
    INSTALL_PATH="$DEFAULT_PATH"
fi

SDK_DIR="$INSTALL_PATH/galbot_sdk"

echo "Installing GALBOT_G1_SDK to $SDK_DIR"

# 安装 config 到机器人运行目录
sudo mkdir -p /data/config
sudo cp -rn ./config/. /data/config

# 创建安装目录
sudo mkdir -p "$INSTALL_PATH"

# 拷贝 SDK
sudo rm -rf "$SDK_DIR"
sudo cp -r galbot_sdk "$INSTALL_PATH/"

# 解压 toolchain
echo "Extracting toolchain..."
sudo tar xf toolchain.tar.gz -C "$SDK_DIR"


TOOLCHAIN_PATH=$(cd "$SDK_DIR" && pwd)
NEW_LINE="set(toolchain $TOOLCHAIN_PATH/toolchain/\${toolchain_target_plat})"

# 修改 example下的 cmake toolchain 路径
for f in examples/cpp/cmake/linux-aarch64-gcc940.cmake \
         examples/cpp/cmake/linux-x86_64-gcc940.cmake
do
    if [ ! -f "$f" ]; then
        echo "[ERROR] File not found: $f"
        exit 1
    fi
    sed -i "7c\\$NEW_LINE" "$f"
done

# 修改 examples/cpp/CMakeLists.txt 的 SDK 路径
SDK_LINE="set(SDK_INSTALL_PATH $INSTALL_PATH)"
if [ ! -f "examples/cpp/CMakeLists.txt" ]; then
    echo "[ERROR] examples/cpp/CMakeLists.txt not found"
    exit 1
fi
sed -i "7c\\$SDK_LINE" examples/cpp/CMakeLists.txt

echo
echo "=========================================="
echo "Installation complete!"
echo "SDK Path: $SDK_DIR"
echo
echo "Use:"
echo "  source $SDK_DIR/linux-x86_64-gcc940/setup.sh"
echo "=========================================="
