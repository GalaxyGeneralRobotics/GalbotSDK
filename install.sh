#!/bin/bash
# ============================================================================
# GALBOT SDK Installation Script
# GALBOT SDK 安装脚本
# ============================================================================
# Description / 描述:
#   This script installs the GALBOT SDK to your system with an interactive
#   and visual installation process.
#   本脚本以交互式和可视化的方式将 GALBOT SDK 安装到您的系统中。
#
# Usage / 使用方法:
#   ./install.sh
#
# Requirements / 要求:
#   - sudo privileges / sudo 权限
#   - bash shell / bash 终端
# ============================================================================

# Exit on error / 遇到错误时退出
set -e

# ============================================================================
# Color Definitions / 颜色定义
# ============================================================================
# ANSI color codes for better visual feedback / ANSI 颜色代码用于更好的视觉反馈
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color / 无颜色

# ============================================================================
# Icon Definitions / 图标定义
# ============================================================================
ICON_SUCCESS="✓"
ICON_ERROR="✗"
ICON_INFO="ℹ"
ICON_ARROW="➤"
ICON_GEAR="⚙"

# ============================================================================
# Configuration / 配置
# ============================================================================
DEFAULT_PATH="/opt/galbot"  # Default installation path / 默认安装路径
TOTAL_STEPS=7               # Total number of installation steps / 安装步骤总数
CURRENT_STEP=0              # Current step counter / 当前步骤计数器

# ============================================================================
# Helper Functions / 辅助函数
# ============================================================================

# Print a styled header / 打印样式化标题
print_header() {
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${WHITE}          GALBOT SDK Installation Program             ${CYAN}║${NC}"
    echo -e "${CYAN}║${WHITE}          GALBOT SDK 安装程序                          ${CYAN}║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# Print a progress bar / 打印进度条
print_progress() {
    local current=$1
    local total=$2
    local description_en=$3
    local description_zh=$4

    # Calculate percentage / 计算百分比
    local percent=$(( current * 100 / total ))
    local filled=$(( percent / 2 ))   # 50-width bar
    local empty=$(( 50 - filled ))

    # Build progress bar / 构建进度条
    printf "${BLUE}[${NC}"
    printf "${GREEN}%${filled}s${NC}" | tr ' ' '='
    printf "${WHITE}%${empty}s${NC}"  | tr ' ' '-'
    printf "${BLUE}]${NC}"

    # Print percentage and step info / 打印百分比和步骤信息
    printf " ${WHITE}%3d%%${NC} " "$percent"
    printf "${YELLOW}[%d/%d]${NC}\n" "$current" "$total"
    printf "   ${CYAN}${ICON_ARROW}${NC} ${description_en}\n"
    printf "   ${CYAN}${ICON_ARROW}${NC} ${description_zh}\n\n"
}


# Print a success message / 打印成功消息
print_success() {
    local msg_en=$1
    local msg_zh=$2
    echo -e "${GREEN}${ICON_SUCCESS}${NC} ${msg_en}"
    echo -e "${GREEN}${ICON_SUCCESS}${NC} ${msg_zh}"
}

# Print an error message / 打印错误消息
print_error() {
    local msg_en=$1
    local msg_zh=$2
    echo -e "${RED}${ICON_ERROR}${NC} ${msg_en}" >&2
    echo -e "${RED}${ICON_ERROR}${NC} ${msg_zh}" >&2
}

# Print an info message / 打印信息消息
print_info() {
    local msg_en=$1
    local msg_zh=$2
    echo -e "${BLUE}${ICON_INFO}${NC} ${msg_en}"
    echo -e "${BLUE}${ICON_INFO}${NC} ${msg_zh}"
}

# Execute a step with progress tracking / 执行带进度跟踪的步骤
execute_step() {
    local step_num=$1
    local desc_en=$2
    local desc_zh=$3
    shift 3
    
    CURRENT_STEP=$step_num
    print_progress "$CURRENT_STEP" "$TOTAL_STEPS" "$desc_en" "$desc_zh"
    
    # Execute the command / 执行命令
    "$@"
    
    print_success "Step completed / 步骤完成" "✓"
    echo ""
}

# ============================================================================
# Main Installation Process / 主安装流程
# ============================================================================

# Clear screen and print header / 清屏并打印标题
clear
print_header

# ============================================================================
# STEP 0: Get Installation Path / 步骤 0：获取安装路径
# ============================================================================
echo -e "${MAGENTA}${ICON_GEAR} Configuration / 配置${NC}"
echo -e "${WHITE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${YELLOW}Please specify the installation directory:${NC}"
echo -e "${YELLOW}请指定安装目录：${NC}"
echo -e "${CYAN}(Press Enter to install to the default path / 按回车键将安装到默认路径)${NC}"
echo -e "${CYAN}Default / 默认: ${WHITE}$DEFAULT_PATH${NC}"
echo ""
read -p "$(echo -e ${GREEN}➤${NC}) " INSTALL_PATH

# Use default path if empty / 如果为空则使用默认路径
if [ -z "$INSTALL_PATH" ]; then
    INSTALL_PATH="$DEFAULT_PATH"
fi

# Define SDK directory / 定义 SDK 目录
SDK_DIR="$INSTALL_PATH/galbot_sdk"

echo ""
print_info "Installation path / 安装路径: ${WHITE}$SDK_DIR${NC}" ""
echo ""
sleep 1

# ============================================================================
# STEP 1: Create Robot Configuration Directory / 步骤 1：创建机器人配置目录
# ============================================================================
execute_step 1 \
    "Creating robot configuration directory" \
    "创建机器人配置目录" \
    sudo mkdir -p /data/config

# ============================================================================
# STEP 2: Copy Configuration Files / 步骤 2：复制配置文件
# ============================================================================
execute_step 2 \
    "Copying configuration files to /data/config" \
    "复制配置文件到 /data/config" \
    sudo cp -rn ./config/. /data/config

# ============================================================================
# STEP 3: Create Installation Directory / 步骤 3：创建安装目录
# ============================================================================
execute_step 3 \
    "Creating installation directory" \
    "创建安装目录" \
    sudo mkdir -p "$INSTALL_PATH"

# ============================================================================
# STEP 4: Copy SDK Files / 步骤 4：复制 SDK 文件
# ============================================================================
step4_copy_sdk() {
    # Remove old SDK if exists / 如果存在旧的 SDK 则删除
    if [ -d "$SDK_DIR" ]; then
        print_info "Removing old SDK / 删除旧的 SDK" ""
        sudo rm -rf "$SDK_DIR"
    fi
    
    # Copy new SDK / 复制新的 SDK
    print_info "Copying SDK files (this may take a moment) / 正在复制 SDK 文件（可能需要一些时间）" ""
    sudo cp -r galbot_sdk "$INSTALL_PATH/"
}

execute_step 4 \
    "Copying SDK files" \
    "复制 SDK 文件" \
    step4_copy_sdk

# ============================================================================
# STEP 5: Extract Toolchain / 步骤 5：解压工具链
# ============================================================================
step5_extract_toolchain() {
    print_info "Extracting toolchain (this may take a while) / 正在解压工具链（可能需要较长时间）" ""
    sudo tar xf toolchain.tar.gz -C "$SDK_DIR"
}

execute_step 5 \
    "Extracting toolchain archive" \
    "解压工具链压缩包" \
    step5_extract_toolchain

# ============================================================================
# STEP 6: Update CMake Configuration / 步骤 6：更新 CMake 配置
# ============================================================================
step6_update_cmake() {
    # Get absolute toolchain path / 获取工具链绝对路径
    TOOLCHAIN_PATH=$(cd "$SDK_DIR" && pwd)
    NEW_LINE="set(toolchain $TOOLCHAIN_PATH/toolchain/\${toolchain_target_plat})"
    
    print_info "Updating CMake toolchain paths / 更新 CMake 工具链路径" ""
    
    # Update toolchain files / 更新工具链文件
    local files=(
        "examples/cpp/cmake/linux-aarch64-gcc940.cmake"
        "examples/cpp/cmake/linux-x86_64-gcc940.cmake"
    )
    
    for f in "${files[@]}"; do
        if [ ! -f "$f" ]; then
            print_error "File not found: $f" "文件未找到：$f"
            exit 1
        fi
        echo -e "   ${CYAN}→${NC} Updating / 更新: ${f}"
        sed -i "7c\\$NEW_LINE" "$f"
    done
}

execute_step 6 \
    "Updating CMake toolchain configuration" \
    "更新 CMake 工具链配置" \
    step6_update_cmake

# ============================================================================
# STEP 7: Update Examples Configuration / 步骤 7：更新示例配置
# ============================================================================
step7_update_examples() {
    SDK_LINE="set(SDK_INSTALL_PATH $INSTALL_PATH)"
    
    if [ ! -f "examples/cpp/CMakeLists.txt" ]; then
        print_error "examples/cpp/CMakeLists.txt not found" "examples/cpp/CMakeLists.txt 未找到"
        exit 1
    fi
    
    print_info "Updating examples SDK path / 更新示例 SDK 路径" ""
    echo -e "   ${CYAN}→${NC} Updating / 更新: examples/cpp/CMakeLists.txt"
    sed -i "7c\\$SDK_LINE" examples/cpp/CMakeLists.txt
}

execute_step 7 \
    "Updating examples configuration" \
    "更新示例配置" \
    step7_update_examples

# ============================================================================
# Installation Complete / 安装完成
# ============================================================================
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║${WHITE}              Installation Successful!                    ${GREEN}║${NC}"
echo -e "${GREEN}║${WHITE}              安装成功！                                    ${GREEN}║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Print installation summary / 打印安装摘要
echo -e "${CYAN}${ICON_INFO} Installation Summary / 安装摘要:${NC}"
echo -e "${WHITE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}SDK Installation Path / SDK 安装路径:${NC}"
echo -e "  ${WHITE}$SDK_DIR${NC}"
echo ""
echo -e "${YELLOW}Configuration Path / 配置路径:${NC}"
echo -e "  ${WHITE}/data/config${NC}"
echo ""

# Print usage instructions / 打印使用说明
echo -e "${CYAN}${ICON_GEAR} Next Steps / 后续步骤:${NC}"
echo -e "${WHITE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}1.${NC} Source the SDK environment / 加载 SDK 环境:"
echo -e "   ${CYAN}source $SDK_DIR/linux-x86_64-gcc940/setup.sh${NC}"
echo ""
echo -e "${GREEN}2.${NC} Explore the examples / 查看示例:"
echo -e "   ${CYAN}cd examples/python${NC}  or / 或  ${CYAN}cd examples/cpp${NC}"
echo ""
echo -e "${GREEN}3.${NC} Read the documentation / 阅读文档:"
echo -e "   ${CYAN}cd docs && python3 -m http.server 8000${NC}"
echo -e "   open ${CYAN}http://localhost:8000${NC}"
echo ""
echo -e "${WHITE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${MAGENTA}Thank you for using GALBOT SDK! / 感谢使用 GALBOT SDK！${NC}"
echo -e "${WHITE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
