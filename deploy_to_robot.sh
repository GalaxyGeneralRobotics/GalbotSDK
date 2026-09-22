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
    # 判断是否 root 用户
    if [ "$(id -u)" -eq 0 ]; then
        SUDO=""
    else
        SUDO="sudo"
    fi

    if command -v sshpass >/dev/null 2>&1; then
        echo "sshpass 已安装 ✔"
        return 0
    fi

    echo "sshpass 未找到，尝试自动安装..."

    if [ -f /etc/os-release ]; then
        . /etc/os-release
        distro=${ID_LIKE:-$ID}
    else
        echo "无法识别系统类型，请手动安装 sshpass"
        exit 1
    fi

    case "$distro" in
        *debian*|*ubuntu*|*linuxmint*)
            echo "Detected Debian/Ubuntu 系列..."
            $SUDO apt-get update -y || { echo "apt update 失败，请手动安装 sshpass"; exit 1; }
            $SUDO apt-get install -y sshpass || { echo "sshpass 安装失败，请手动安装"; exit 1; }
            ;;
        *rhel*|*centos*|*rocky*|*almalinux*|*fedora*|*amazon*|*ol*)
            echo "Detected RHEL/CentOS/Fedora 系列..."
            if command -v dnf >/dev/null 2>&1; then
                $SUDO dnf install -y epel-release || echo "epel-release 已存在或安装失败"
                $SUDO dnf install -y sshpass || { echo "sshpass 安装失败，请手动安装"; exit 1; }
            else
                $SUDO yum install -y epel-release || echo "epel-release 已存在或安装失败"
                $SUDO yum install -y sshpass || { echo "sshpass 安装失败，请手动安装"; exit 1; }
            fi
            ;;
        *arch*|*manjaro*)
            echo "Detected Arch/Manjaro 系列..."
            $SUDO pacman -Sy --noconfirm sshpass || { echo "sshpass 安装失败，请手动安装"; exit 1; }
            ;;
        *)
            echo "未知的 Linux 发行版 ($distro)，请手动安装 sshpass"
            exit 1
            ;;
    esac

    # 再次确认 sshpass 是否安装成功
    if ! command -v sshpass >/dev/null 2>&1; then
        echo "sshpass 安装失败，请手动安装"
        exit 1
    fi

    echo "sshpass 安装完成 ✔"
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
REMOTE_BIN_DIR="${REMOTE_DIR}/galbot_sdk/bin"

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

# 根据目标机器架构和 Ubuntu 版本选择对应的 aarch64 SDK。
REMOTE_ARCH=$(sshpass -p "$SSHPASS" \
ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" "uname -m" 2>/dev/null | tr -d '\r')

REMOTE_UBUNTU_VERSION=$(sshpass -p "$SSHPASS" \
ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" "source /etc/os-release && echo \$VERSION_ID" 2>/dev/null | tr -d '\r')

if [ -z "$REMOTE_ARCH" ] || [ -z "$REMOTE_UBUNTU_VERSION" ]; then
    print_error "Failed to detect remote robot platform"
    exit 1
fi

TARGET_PLATFORM=""
if [ "$REMOTE_ARCH" = "aarch64" ]; then
    if [ "$REMOTE_UBUNTU_VERSION" = "18.04" ] || [ "$REMOTE_UBUNTU_VERSION" = "20.04" ]; then
        TARGET_PLATFORM="linux-aarch64-gcc940"
    elif [[ "$REMOTE_UBUNTU_VERSION" == 24.* ]] || [[ "$REMOTE_UBUNTU_VERSION" == 25.* ]]; then
        TARGET_PLATFORM="linux-aarch64-gcc1330"
    else
        print_error "Unsupported Ubuntu version on robot: $REMOTE_UBUNTU_VERSION"
        exit 1
    fi
else
    print_error "Unsupported robot architecture: $REMOTE_ARCH"
    exit 1
fi

LIB_DIR="./galbot_sdk/${TARGET_PLATFORM}/lib"

[ ! -d "$LIB_DIR" ] && { print_error "Local library directory not found: $LIB_DIR"; exit 1; }
CPP_LIB_COUNT=$(find "$LIB_DIR" -name "libgalbot_sdk.*" | wc -l)

print_success "Local directory OK"
print_info "Remote architecture: $REMOTE_ARCH"
print_info "Remote Ubuntu version: $REMOTE_UBUNTU_VERSION"
print_info "Selected target platform: $TARGET_PLATFORM"
print_info "Using local library directory: $LIB_DIR"
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
"$LIB_DIR"/libgalbot_sdk.* \
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
    "$ROBOT_USER@$ROBOT_IP:$REMOTE_DIR/" \
    && print_success "Python library deployed" \
    || { print_error "Python deployment failed"; exit 1; }
else
    print_warning "Python library not found, skipping"
fi

# -----------------------------------------------------------------------------
# Step 6: Deploy Version Check Tool
# -----------------------------------------------------------------------------
print_step 6 "Deploying Version Check Tool" "部署版本检查工具"

COMPAT_SCRIPT="./check_robot_compat.py"
WRAPPER_SCRIPT="./galbot_sdk_wrapper.sh"

if [ -f "$COMPAT_SCRIPT" ] && [ -f "$WRAPPER_SCRIPT" ]; then
    # 在机器人上创建 bin 目录
    sshpass -p "$SSHPASS" \
    ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" "mkdir -p '$REMOTE_BIN_DIR'" \
    || { print_error "Failed to create $REMOTE_BIN_DIR on robot"; exit 1; }

    # 复制 check_robot_compat.py
    sshpass -p "$SSHPASS" \
    rsync -avz \
    -e "ssh $SSH_OPTS" \
    "$COMPAT_SCRIPT" \
    "$ROBOT_USER@$ROBOT_IP:$REMOTE_BIN_DIR/" \
    || { print_error "Version check tool deployment failed"; exit 1; }

    # 复制 galbot_sdk wrapper 脚本并重命名
    sshpass -p "$SSHPASS" \
    rsync -avz \
    -e "ssh $SSH_OPTS" \
    "$WRAPPER_SCRIPT" \
    "$ROBOT_USER@$ROBOT_IP:$REMOTE_BIN_DIR/galbot_sdk" \
    && sshpass -p "$SSHPASS" ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" "chmod +x '$REMOTE_BIN_DIR/galbot_sdk'" \
    && print_success "Version check tool deployed" \
    || { print_error "Failed to deploy galbot_sdk wrapper on robot"; exit 1; }
else
    if [ ! -f "$COMPAT_SCRIPT" ]; then
        print_warning "check_robot_compat.py not found, skipping"
    fi
    if [ ! -f "$WRAPPER_SCRIPT" ]; then
        print_warning "galbot_sdk_wrapper.sh not found, skipping"
    fi
fi

# -----------------------------------------------------------------------------
# Step 7: Configure Environment Variables
# -----------------------------------------------------------------------------
print_step 7 "Configuring Environment Variables" "配置环境变量"

# 创建临时配置脚本
cat > /tmp/galbot_env_setup.sh << 'ENVEOF'
#!/bin/bash
RCFILE=~/.bashrc
SDK_BIN="/data/galbot/lib/galbot_sdk/bin"
SDK_LIB="/data/galbot/lib"
NOW=$(date "+%Y-%m-%d %H:%M:%S")

# 检查 PATH 是否已配置
if ! grep -q "export PATH=.*$SDK_BIN" "$RCFILE" 2>/dev/null; then
    echo "" >> "$RCFILE"
    echo "# Galbot SDK PATH configuration - added on $NOW" >> "$RCFILE"
    echo "export PATH=\"$SDK_BIN:\$PATH\"" >> "$RCFILE"
    PATH_ADDED=1
else
    PATH_ADDED=0
fi

# 检查 PYTHONPATH 是否已配置
if ! grep -q "export PYTHONPATH=.*$SDK_LIB" "$RCFILE" 2>/dev/null; then
    echo "" >> "$RCFILE"
    echo "# Galbot SDK PYTHONPATH configuration - added on $NOW" >> "$RCFILE"
    echo "export PYTHONPATH=\"$SDK_LIB:\$PYTHONPATH\"" >> "$RCFILE"
    PY_ADDED=1
else
    PY_ADDED=0
fi

if [ "$PATH_ADDED" -eq 1 ] || [ "$PY_ADDED" -eq 1 ]; then
    echo "Environment variables added to ~/.bashrc"
    echo "Run: source ~/.bashrc  to activate immediately"
else
    echo "Environment variables already configured in ~/.bashrc"
fi
ENVEOF

# 上传并执行配置脚本
sshpass -p "$SSHPASS" \
rsync -avz -e "ssh $SSH_OPTS" \
    /tmp/galbot_env_setup.sh \
    "$ROBOT_USER@$ROBOT_IP:/tmp/" \
    && sshpass -p "$SSHPASS" ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" "bash /tmp/galbot_env_setup.sh" \
    && rm -f /tmp/galbot_env_setup.sh

print_success "环境配置已自动添加到机器人 ~/.bashrc"
print_info "运行 'source ~/.bashrc' 或重新 SSH 登录使配置生效"

# -----------------------------------------------------------------------------
# Step 8: Verification
# -----------------------------------------------------------------------------
print_step 8 "Post-deployment Verification" "部署后验证"

sshpass -p "$SSHPASS" \
ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" "
ls '$REMOTE_DIR'/libgalbot_sdk.* >/dev/null 2>&1 && echo CPP_OK=1 || echo CPP_OK=0
[ -d '$REMOTE_DIR/galbot_sdk' ] && echo PY_OK=1 || echo PY_OK=0
[ -x '$REMOTE_BIN_DIR/galbot_sdk' ] && echo TOOL_OK=1 || echo TOOL_OK=0
"

# -----------------------------------------------------------------------------
# Done
# -----------------------------------------------------------------------------
echo ""
print_success "Deployment completed successfully / 部署完成"
echo "Target: $ROBOT_USER@$ROBOT_IP:$REMOTE_DIR"
