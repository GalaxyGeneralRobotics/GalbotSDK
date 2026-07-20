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

XCU_INTERNAL_IP="192.168.100.66"
HPU_INTERNAL_IP="192.168.100.88"

XCU_USER="root"
XCU_PASS="12345678"
HPU_USER="galbot"
HPU_PASS="gb@2023"

SSH_OPTS="-o StrictHostKeyChecking=no -o ConnectTimeout=10"

# -----------------------------------------------------------------------------
# Variables (embosa config IPs, written to JSON files)
# -----------------------------------------------------------------------------
PC_IP=""
XCU_IP=""
HPU_IP=""

DEFAULT_XCU_IP="192.168.1.66"
DEFAULT_HPU_IP="192.168.1.88"

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

The script will interactively prompt for all IPs.
脚本启动后交互输入各端 IP 地址。

  PC  IP: 必填 / required
  XCU IP: 默认 ${DEFAULT_XCU_IP}，直接回车使用默认值
  HPU IP: 默认 ${DEFAULT_HPU_IP}，直接回车使用默认值

SSH credentials / SSH 凭据:
  XCU: ${XCU_USER} / ${XCU_PASS}
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

prompt_config_ips() {
    while true; do
        read -r -p "请输入配置的 PC IP: " PC_IP
        if [[ -n "$PC_IP" ]]; then
            break
        fi
        echo "PC IP 不能为空，请重新输入 / PC IP cannot be empty, please try again."
    done

    read -r -p "请输入配置的 XCU IP [默认 ${DEFAULT_XCU_IP}]: " XCU_IP
    XCU_IP="${XCU_IP:-$DEFAULT_XCU_IP}"

    read -r -p "请输入配置的 HPU IP [默认 ${DEFAULT_HPU_IP}]: " HPU_IP
    HPU_IP="${HPU_IP:-$DEFAULT_HPU_IP}"
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
# Build JSON configs for each device
# XCU/HPU local_interface: include external IP only when it differs from internal
# -----------------------------------------------------------------------------
build_pc_config() {
    echo "{
    \"embosa_ip\": {
        \"local_interface\": [
            \"${PC_IP}\"
        ],
        \"peer_lists\": [
            \"${XCU_IP}\", \"${HPU_IP}\"
        ]
    }
}"
}

build_xcu_config() {
    local local_iface
    if [[ "$XCU_IP" == "$XCU_INTERNAL_IP" ]]; then
        local_iface="\"${XCU_INTERNAL_IP}\""
    else
        local_iface="\"${XCU_INTERNAL_IP}\", \"${XCU_IP}\""
    fi
    echo "{
    \"embosa_ip\": {
        \"local_interface\": [
            ${local_iface}
        ],
        \"peer_lists\": [
            \"${HPU_INTERNAL_IP}\", \"${PC_IP}\"
        ]
    }
}"
}

build_hpu_config() {
    local local_iface
    if [[ "$HPU_IP" == "$HPU_INTERNAL_IP" ]]; then
        local_iface="\"${HPU_INTERNAL_IP}\""
    else
        local_iface="\"${HPU_INTERNAL_IP}\", \"${HPU_IP}\""
    fi
    echo "{
    \"embosa_ip\": {
        \"local_interface\": [
            ${local_iface}
        ],
        \"peer_lists\": [
            \"${XCU_INTERNAL_IP}\", \"${PC_IP}\"
        ]
    }
}"
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
        mkdir -p "$config_dir"
    fi

    if [[ -f "$EMBOSA_CONFIG_PATH" ]]; then
        cp "$EMBOSA_CONFIG_PATH" "${EMBOSA_CONFIG_PATH}.bak"
        echo "[INFO] Backed up: ${EMBOSA_CONFIG_PATH}.bak"
    fi

    echo "$config" > "$EMBOSA_CONFIG_PATH"
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
    prompt_config_ips

    echo ""
    echo "[INFO] Embosa config IPs / 配置文件 IP:"
    echo "[INFO]   PC  : ${PC_IP}"
    echo "[INFO]   XCU : ${XCU_IP}"
    echo "[INFO]   HPU : ${HPU_IP}"

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
