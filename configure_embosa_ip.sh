#!/bin/bash
# =============================================================================
# Galbot Embosa IP One-Click Configuration Script
# Deploys embosa IP config to PC (local), XCU and HPU (via sshpass) in sequence.
# Run this script on the PC side only.
# =============================================================================

set -euo pipefail

# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------
EMBOSA_CONFIG_PATH="/data/config/embosa_ip_config.json"
SYSTEM_CFG_PATH="/data/config/system.cfg"

XCU_USER="root"
HPU_USER="galbot"
HPU_PASS="gb@2023"

SSH_OPTS="-o StrictHostKeyChecking=no -o ConnectTimeout=10"

# -----------------------------------------------------------------------------
# Runtime IPs written to JSON files. PC_IP is user-configurable, while XCU_IP
# and HPU_IP are assigned from the selected machine profile.
# -----------------------------------------------------------------------------
PC_IP=""
XCU_IP=""
HPU_IP=""

# Machine-type dependent defaults and the fixed discovery endpoint.
# The discovery endpoint must not be changed by the user-entered device IPs.
# Assigned in prompt_machine_type().
MACHINE_TYPE=""
XCU_PASS=""
DEFAULT_PC_IP=""
DISCOVERY_IP=""
DISCOVERY_PORT=11888

# -----------------------------------------------------------------------------
# Help
# -----------------------------------------------------------------------------
show_help() {
    cat << EOF
Galbot Embosa IP One-Click Configuration Script

Run on PC to deploy embosa IP config to PC (local), XCU and HPU via SSH.
在 PC 端运行，一键完成 PC / XCU / HPU 三端 embosa IP 配置。

Usage / 用法:
  $(basename "$0") [-h|--help]

The script will interactively prompt for machine type and the PC IP.
脚本启动后交互选择机型并输入 PC IP 地址。

  机型 / Machine type: G-series (默认) / S1
  PC  IP: 默认 G-series=192.168.1.99，S1=192.168.100.99，直接回车使用默认值
  XCU IP: 固定 G-series=192.168.1.66，S1=192.168.100.66
  HPU IP: 固定 G-series=192.168.1.88，S1=192.168.100.88

Discovery service / 发现服务:
  G-series: 192.168.1.88:${DISCOVERY_PORT}
  S1:       192.168.100.88:${DISCOVERY_PORT}
  发现服务地址按机型固定，不会被输入的设备 IP 修改。

SSH credentials / SSH 凭据:
  XCU: ${XCU_USER} / 密码按机型区分（G-series=12345678，S1=123）
  HPU: ${HPU_USER} / ${HPU_PASS}
EOF
}

# -----------------------------------------------------------------------------
# Argument parsing
# -----------------------------------------------------------------------------
parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                echo "[ERROR] Unknown option: $1 / 未知参数: $1" >&2
                echo ""
                show_help
                exit 1
                ;;
        esac
    done
}

prompt_machine_type() {
    local choice
    while true; do
        read -r -p "请选择机型 [1] G-series (默认) [2] S1: " choice
        choice="${choice:-1}"
        case "$choice" in
            1)
                MACHINE_TYPE="G-series"
                XCU_PASS="12345678"
                DEFAULT_PC_IP="192.168.1.99"
                XCU_IP="192.168.1.66"
                HPU_IP="192.168.1.88"
                DISCOVERY_IP="192.168.1.88"
                break
                ;;
            2)
                MACHINE_TYPE="S1"
                XCU_PASS="123"
                DEFAULT_PC_IP="192.168.100.99"
                XCU_IP="192.168.100.66"
                HPU_IP="192.168.100.88"
                DISCOVERY_IP="192.168.100.88"
                break
                ;;
            *)
                echo "输入无效，请输入 1 或 2 / Invalid input, please enter 1 or 2."
                ;;
        esac
    done
    echo "[INFO] 机型 / Machine type: ${MACHINE_TYPE}"
}

prompt_pc_ip() {
    read -r -p "请输入配置的 PC IP [默认 ${DEFAULT_PC_IP}]: " PC_IP
    PC_IP="${PC_IP:-$DEFAULT_PC_IP}"
}

# -----------------------------------------------------------------------------
# Check sshpass is available
# -----------------------------------------------------------------------------
check_sshpass() {
    if ! command -v sshpass &>/dev/null; then
        echo "[ERROR] sshpass is not installed / 未检测到 sshpass，请先安装：" >&2
        echo "         sudo apt install sshpass" >&2
        exit 1
    fi
}

# -----------------------------------------------------------------------------
# Determine whether local commands need sudo
# If already root, no prefix is needed; otherwise use sudo. sudo prompts
# interactively for the password only when a privileged command actually
# runs, so the whole script does not need to be invoked with sudo.
# -----------------------------------------------------------------------------
check_local_sudo() {
    if [[ "$(id -u)" -eq 0 ]]; then
        SUDO=""
    else
        SUDO="sudo"
    fi
}

# -----------------------------------------------------------------------------
# Build the GBS 1.18 server-discovery JSON for one device.
# dsha_did identifies the platform. PC interfaces use the user-selected IP;
# XCU/HPU interfaces and the discovery endpoint come from the machine profile.
# -----------------------------------------------------------------------------
build_device_config() {
    local dsha_did="$1"
    local device_ip="$2"

    echo "{
    \"discovery\": {
        \"remote_servers\": [
            {
                \"ip\": \"${DISCOVERY_IP}\",
                \"port\": ${DISCOVERY_PORT}
            }
        ],
        \"server_config\": {
            \"listen_addresses\": [
                {
                    \"ip\": \"${DISCOVERY_IP}\",
                    \"port\": ${DISCOVERY_PORT}
                }
            ]
        }
    },
    \"discovery_mode\": \"server\",
    \"embosa_ip\": {
        \"dsha_did\": ${dsha_did},
        \"local_interface\": [
            \"${device_ip}\"
        ],
        \"meta_interface\": [
            \"${device_ip}\"
        ]
    }
}"
}

build_pc_config() {
    build_device_config 3 "$PC_IP"
}

build_xcu_config() {
    build_device_config 1 "$XCU_IP"
}

build_hpu_config() {
    build_device_config 2 "$HPU_IP"
}

# -----------------------------------------------------------------------------
# Remote helpers
# -----------------------------------------------------------------------------
ssh_run() {
    local pass="$1" user="$2" host="$3" cmd="$4"
    sshpass -p "$pass" ssh $SSH_OPTS "${user}@${host}" "$cmd"
}

ssh_write_file() {
    local pass="$1" user="$2" host="$3" remote_path="$4" content="$5"
    echo "$content" | sshpass -p "$pass" ssh $SSH_OPTS "${user}@${host}" "cat > ${remote_path}"
}

remote_fix_system_cfg() {
    local pass="$1" user="$2" host="$3"
    ssh_run "$pass" "$user" "$host" \
        "python3 -c \"import json; p='${SYSTEM_CFG_PATH}'; d=json.load(open(p)); d['enable_modify_embosa_cfg']=False; json.dump(d,open(p,'w'),indent=4)\""
}

# -----------------------------------------------------------------------------
# Deploy PC (local)
# -----------------------------------------------------------------------------
deploy_pc() {
    echo ""
    echo "========== [1/3] PC 端配置 / PC Configuration =========="

    local config
    config="$(build_pc_config)"
    echo "[INFO] Writing ${EMBOSA_CONFIG_PATH}:"
    echo "$config" | sed 's/^/  /'

    local config_dir
    config_dir="$(dirname "$EMBOSA_CONFIG_PATH")"
    if [[ ! -d "$config_dir" ]]; then
        $SUDO mkdir -p "$config_dir"
    fi

    if [[ -f "$EMBOSA_CONFIG_PATH" ]]; then
        $SUDO cp "$EMBOSA_CONFIG_PATH" "${EMBOSA_CONFIG_PATH}.bak"
        echo "[INFO] Backed up: ${EMBOSA_CONFIG_PATH}.bak"
    fi

    echo "$config" | $SUDO tee "$EMBOSA_CONFIG_PATH" > /dev/null
    echo "[OK] PC embosa config written."
}

# -----------------------------------------------------------------------------
# Deploy XCU (via sshpass)
# -----------------------------------------------------------------------------
deploy_xcu() {
    echo ""
    echo "========== [2/3] XCU 端配置 / XCU Configuration =========="

    local config
    config="$(build_xcu_config)"
    echo "[INFO] XCU embosa config:"
    echo "$config" | sed 's/^/  /'

    echo "[INFO] Fixing system.cfg on XCU (${XCU_IP})..."
    remote_fix_system_cfg "$XCU_PASS" "$XCU_USER" "$XCU_IP"
    echo "[OK] system.cfg updated on XCU."

    echo "[INFO] Writing embosa_ip_config.json on XCU..."
    ssh_write_file "$XCU_PASS" "$XCU_USER" "$XCU_IP" "$EMBOSA_CONFIG_PATH" "$config"
    echo "[OK] embosa_ip_config.json written on XCU."

    echo "[INFO] Restarting launcher on XCU..."
    ssh_run "$XCU_PASS" "$XCU_USER" "$XCU_IP" "systemctl restart launcher"
    echo "[OK] launcher restarted on XCU."
}

# -----------------------------------------------------------------------------
# Deploy HPU (via sshpass)
# -----------------------------------------------------------------------------
deploy_hpu() {
    echo ""
    echo "========== [3/3] HPU 端配置 / HPU Configuration =========="

    local config
    config="$(build_hpu_config)"
    echo "[INFO] HPU embosa config:"
    echo "$config" | sed 's/^/  /'

    echo "[INFO] Fixing system.cfg on HPU (${HPU_IP})..."
    remote_fix_system_cfg "$HPU_PASS" "$HPU_USER" "$HPU_IP"
    echo "[OK] system.cfg updated on HPU."

    echo "[INFO] Writing embosa_ip_config.json on HPU..."
    ssh_write_file "$HPU_PASS" "$HPU_USER" "$HPU_IP" "$EMBOSA_CONFIG_PATH" "$config"
    echo "[OK] embosa_ip_config.json written on HPU."

    echo "[INFO] Restarting launcher on HPU..."
    sshpass -p "$HPU_PASS" ssh -tt $SSH_OPTS "${HPU_USER}@${HPU_IP}" "echo '${HPU_PASS}' | sudo -S -p '' systemctl restart launcher"
    echo "[OK] launcher restarted on HPU."
}

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
main() {
    parse_args "$@"
    check_sshpass
    check_local_sudo
    prompt_machine_type
    prompt_pc_ip

    echo ""
    echo "[INFO] Embosa config IPs / 配置文件 IP:"
    echo "[INFO]   PC  : ${PC_IP}"
    echo "[INFO]   XCU (fixed / 固定): ${XCU_IP}"
    echo "[INFO]   HPU (fixed / 固定): ${HPU_IP}"
    echo "[INFO]   Discovery (fixed): ${DISCOVERY_IP}:${DISCOVERY_PORT}"

    deploy_pc
    deploy_xcu

    echo ""
    echo "[INFO] Waiting 3 seconds before deploying HPU... / 等待 3 秒后部署 HPU..."
    sleep 3

    deploy_hpu

    echo ""
    echo "=========================================="
    echo "[OK] All done! / 全部部署完成！"
    echo "=========================================="
}

main "$@"
