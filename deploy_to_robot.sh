#!/bin/bash
# =============================================================================
# Galbot SDK Deployment Script / Galbot SDK 部署脚本
# =============================================================================

# -----------------------------------------------------------------------------
# Color Definitions / 颜色定义
# -----------------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

# -----------------------------------------------------------------------------
# Print Helpers
# -----------------------------------------------------------------------------
print_success() { echo -e "${GREEN}✓${NC} $1"; }
print_error()   { echo -e "${RED}✗${NC} $1"; }
print_warning() { echo -e "${YELLOW}⚠${NC} $1"; }
print_info()    { echo -e "${BLUE}ℹ${NC} $1"; }

# -----------------------------------------------------------------------------
# Check for sshpass / 检查 sshpass 是否已安装
# -----------------------------------------------------------------------------
check_sshpass() {
    if ! command -v sshpass >/dev/null 2>&1; then
        print_warning "sshpass not found / 未找到 sshpass"
        echo "  Ubuntu/Debian: sudo apt install sshpass"
        echo "  CentOS/RHEL:  sudo yum install sshpass"
        exit 1
    fi
}

# -----------------------------------------------------------------------------
# Banner
# -----------------------------------------------------------------------------
print_banner() {
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC}  ${BOLD}Galbot SDK Deployment Tool / Galbot SDK 部署工具${NC}              ${CYAN}║${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

print_step() {
    echo ""
    echo -e "${MAGENTA}┌─────────────────────────────────────────────────────────────────┐${NC}"
    echo -e "${MAGENTA}│${NC} ${BOLD}Step $1:${NC} $2"
    echo -e "${MAGENTA}│${NC} ${BOLD}步骤 $1:${NC} $3"
    echo -e "${MAGENTA}└─────────────────────────────────────────────────────────────────┘${NC}"
}

# =============================================================================
# Main
# =============================================================================

check_sshpass
clear
print_banner

# -----------------------------------------------------------------------------
# Step 1: User Input
# -----------------------------------------------------------------------------
print_step 1 "User Input Configuration" "用户输入配置"

read -p "Enter robot username / 请输入机器人用户名: " ROBOT_USER
[ -z "$ROBOT_USER" ] && { print_error "Username cannot be empty"; exit 1; }

read -p "Enter robot IP address / 请输入机器人 IP 地址: " ROBOT_IP
[ -z "$ROBOT_IP" ] && { print_error "IP address cannot be empty"; exit 1; }

read -s -p "Enter robot password / 请输入机器人密码: " SSHPASS
echo ""
[ -z "$SSHPASS" ] && { print_error "Password cannot be empty"; exit 1; }

REMOTE_DIR="/data/galbot/lib"
LIB_DIR="./galbot_sdk/linux-aarch64-gcc940/lib"

# -----------------------------------------------------------------------------
# SSH non-interactive options (关键修复点)
# -----------------------------------------------------------------------------
SSH_OPTS="-o StrictHostKeyChecking=no \
-o UserKnownHostsFile=/dev/null \
-o LogLevel=ERROR \
-o ConnectTimeout=10"

# -----------------------------------------------------------------------------
# Step 2: Validation
# -----------------------------------------------------------------------------
print_step 2 "Pre-deployment Validation" "部署前验证"

[ ! -d "$LIB_DIR" ] && { print_error "Local library directory not found"; exit 1; }
CPP_LIB_COUNT=$(find "$LIB_DIR" -name "libgalbot_g1_sdk.*" | wc -l)

print_success "Local directory OK"
print_info "C++ libraries: $CPP_LIB_COUNT"

# -----------------------------------------------------------------------------
# Step 3: Confirmation
# -----------------------------------------------------------------------------
print_step 3 "Deployment Confirmation" "部署确认"

echo "Target: $ROBOT_USER@$ROBOT_IP:$REMOTE_DIR"
read -p "Continue? (y/n): " -n 1 -r
echo ""
[[ ! $REPLY =~ ^[Yy]$ ]] && exit 0

# -----------------------------------------------------------------------------
# Step 4: Deploy C++ Libraries
# -----------------------------------------------------------------------------
print_step 4 "Deploying C++ Libraries" "部署 C++ 库"

sshpass -p "$SSHPASS" \
rsync -avz \
-e "ssh $SSH_OPTS" \
"$LIB_DIR"/libgalbot_g1_sdk.* \
"$ROBOT_USER@$ROBOT_IP:$REMOTE_DIR/" \
&& print_success "C++ libraries deployed" \
|| { print_error "C++ deployment failed"; exit 1; }

# -----------------------------------------------------------------------------
# Step 5: Deploy Python Library
# -----------------------------------------------------------------------------
print_step 5 "Deploying Python Library" "部署 Python 库"

if [ -d "$LIB_DIR/python/galbot_sdk" ]; then
    sshpass -p "$SSHPASS" \
    rsync -avz \
    -e "ssh $SSH_OPTS" \
    "$LIB_DIR/python/galbot_sdk" \
    "$ROBOT_USER@$ROBOT_IP:$REMOTE_DIR" \
    && print_success "Python library deployed" \
    || { print_error "Python deployment failed"; exit 1; }
else
    print_warning "Python library not found, skipping"
fi

# -----------------------------------------------------------------------------
# Step 6: Verification
# -----------------------------------------------------------------------------
print_step 6 "Post-deployment Verification" "部署后验证"

sshpass -p "$SSHPASS" \
ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" "
ls '$REMOTE_DIR'/libgalbot_g1_sdk.* >/dev/null 2>&1 && echo CPP_OK=1 || echo CPP_OK=0
[ -d '$REMOTE_DIR/galbot_sdk' ] && echo PY_OK=1 || echo PY_OK=0
"

# -----------------------------------------------------------------------------
# Done
# -----------------------------------------------------------------------------
echo ""
print_success "Deployment completed successfully / 部署完成"
echo "Target: $ROBOT_USER@$ROBOT_IP:$REMOTE_DIR"
