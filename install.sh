#!/bin/bash
#
# install.sh - GALBOT_SDK 增量安装脚本
#
# 功能：
# 1. 读取 deps/*.json 获取依赖声明
# 2. 通过校验缓存压缩包的 SHA256 判断哪些依赖需要更新
# 3. 下载/复制并安装需要更新的依赖（保存到 .cache/ 目录）
# 4. 安装 SDK 核心库
# 5. 可选择运行 G1/S1欢迎自检程序
# 6. 可选择下载Python example三方库依赖、和部署SDK到机器人

# 缓存机制：
# - 所有下载的压缩包都保存在 $INSTALL_DIR/.cache/ 目录
# - 每次安装前会校验缓存文件的实际 SHA256 值
# - 如果校验失败或缓存不存在，则重新下载
#
# 用法：
#   ./install.sh [选项]
#
# 选项：
#   --platform <platform>   指定目标平台（如 linux-aarch64-gcc940）
#   --install-dir <path>    安装目录（默认 /opt/galbot）
#   --check                 仅校验版本，不下载或安装
#   --force                 强制重新下载所有依赖
#   --offline               禁止网络访问，仅使用已安装的依赖
#   -y, --yes               非交互模式，使用默认值
#   -h, --help              显示帮助信息
#

set -e

# 脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# SDK 源目录（用于本地模式查找依赖压缩包）
# 可通过环境变量 SDK_SOURCE_DIR 指定，用于本地测试
if [ -n "$SDK_SOURCE_DIR" ]; then
    # 使用环境变量指定的目录
    :
elif [ -d "$SCRIPT_DIR/deps/toolchain/releases" ] || [ -d "$SCRIPT_DIR/deps/third_party/releases" ]; then
    # 当前目录下有完整的 deps 结构（包含压缩包）
    SDK_SOURCE_DIR="$SCRIPT_DIR"
elif [ -d "$SCRIPT_DIR/../deps/toolchain/releases" ] || [ -d "$SCRIPT_DIR/../deps/third_party/releases" ]; then
    # 父目录下有完整的 deps 结构（本地测试模式，install.sh 在 output/ 下）
    SDK_SOURCE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
elif [ -d "$SCRIPT_DIR/deps" ]; then
    # 当前目录有 deps（可能只有 JSON，用于在线下载模式）
    SDK_SOURCE_DIR="$SCRIPT_DIR"
else
    SDK_SOURCE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
fi

# 默认值
DEFAULT_INSTALL_DIR="/opt/galbot"
read_sdk_version() {
    local version_file="${SCRIPT_DIR}/VERSION"
    if [ ! -f "$version_file" ] && [ -n "${SDK_SOURCE_DIR:-}" ]; then
        version_file="${SDK_SOURCE_DIR}/VERSION"
    fi
    if [ ! -f "$version_file" ]; then
        echo "unknown"
        return
    fi

    sed -n '1{s/[[:space:]]//g;p;q;}' "$version_file"
}

SDK_VERSION="$(read_sdk_version)"
EMBOSA_LOG_DIR="/userdata/log/embosa"

# 参数
INSTALL_DIR=""
TARGET_PLATFORM=""
ALL_PLATFORMS=true  # 默认安装所有平台
CHECK_ONLY=false
FORCE_INSTALL=false
OFFLINE_MODE=false
NON_INTERACTIVE=false
USE_SUDO=true

# 所有支持的平台
SUPPORTED_PLATFORMS=("linux-aarch64-gcc940" "linux-x86_64-gcc940")

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# sudo 包装函数
run_cmd() {
    if [ "$USE_SUDO" = true ]; then
        sudo "$@"
    else
        "$@"
    fi
}

cached_file_exists() {
    run_cmd test -f "$1"
}

cached_file_sha256() {
    run_cmd sha256sum "$1" | awk '{print $1}'
}

show_help() {
    cat << EOF
GALBOT_SDK 安装程序 v${SDK_VERSION}

用法: ./install.sh [选项]

选项:
  --platform <platform>   指定单个目标平台（覆盖默认的所有平台安装）
                          可选值: linux-aarch64-gcc940, linux-x86_64-gcc940
  --single-platform       只安装当前检测到的平台（覆盖默认的所有平台安装）
  --all-platforms         安装所有平台（默认行为，用于交叉编译）
  --install-dir <path>    安装目录（默认: $DEFAULT_INSTALL_DIR）
  --sdk-source <path>     SDK 源目录（用于本地测试，从此目录读取 deps/releases）
  --check                 仅校验版本，不下载或安装
  --force                 强制重新下载所有依赖
  --offline               离线模式，仅使用已安装的依赖
  --no-sudo               不使用 sudo（用于测试或无权限环境）
  -y, --yes               非交互模式，使用默认值（并跳过安装后的python依赖下载/机器人部署）
  -h, --help              显示帮助信息

环境变量:
  SDK_SOURCE_DIR          等同于 --sdk-source，用于本地测试

示例:
  ./install.sh                                    # 交互式安装（默认安装所有平台）
  ./install.sh -y                                 # 使用默认值安装（所有平台）
  ./install.sh --single-platform                 # 只安装当前检测到的平台
  ./install.sh --platform linux-x86_64-gcc940    # 只安装指定平台
  ./install.sh --install-dir /home/user/galbot   # 指定安装目录
  ./install.sh --check                            # 仅检查版本
  ./install.sh --sdk-source /path/to/sdk_source  # 本地测试，使用本地依赖
EOF
}

# 解析命令行参数
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --platform)
                TARGET_PLATFORM="$2"
                ALL_PLATFORMS=false  # 指定平台时，只安装该平台
                shift 2
                ;;
            --install-dir)
                INSTALL_DIR="$2"
                shift 2
                ;;
            --sdk-source)
                SDK_SOURCE_DIR="$2"
                shift 2
                ;;
            --all-platforms)
                ALL_PLATFORMS=true
                shift
                ;;
            --single-platform)
                ALL_PLATFORMS=false
                shift
                ;;
            --check)
                CHECK_ONLY=true
                shift
                ;;
            --force)
                FORCE_INSTALL=true
                shift
                ;;
            --offline)
                OFFLINE_MODE=true
                shift
                ;;
            -y|--yes)
                NON_INTERACTIVE=true
                shift
                ;;
            --no-sudo)
                USE_SUDO=false
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                log_error "未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done
}

# 检测系统架构
detect_platform() {
    local arch=$(uname -m)
    case $arch in
        x86_64)
            echo "linux-x86_64-gcc940"
            ;;
        aarch64|arm64)
            echo "linux-aarch64-gcc940"
            ;;
        *)
            log_error "不支持的架构: $arch"
            exit 1
            ;;
    esac
}

# 获取工具链名称
get_toolchain_name() {
    local platform=$1
    case $platform in
        linux-aarch64-gcc940)
            echo "gcc940-aarch64-ubuntu2004-gnu"
            ;;
        linux-x86_64-gcc940)
            echo "gcc940-x86_64-ubuntu2004-gnu"
            ;;
        *)
            log_error "未知平台: $platform"
            exit 1
            ;;
    esac
}

# 下载或复制文件
download_file() {
    local url=$1
    local local_path=$2
    local output_file=$3
    local expected_sha256=$4
    
    # 优先使用本地路径
    if [ -n "$local_path" ] && [ -f "$SDK_SOURCE_DIR/$local_path" ]; then
        log_info "  从本地复制: $local_path"
        cp "$SDK_SOURCE_DIR/$local_path" "$output_file"
    elif [ "$OFFLINE_MODE" = true ]; then
        log_error "  离线模式下无法获取文件"
        return 1
    elif [ -z "$url" ]; then
        # URL 为空，且本地文件不存在
        log_error "  URL 为空，且本地文件不存在: $local_path"
        return 1
    else
        log_info "  下载: $url"
        # 避免被当前 shell 中的 SDK 环境变量污染（例如 LD_LIBRARY_PATH 指向 SDK 自带 libcurl）
        # 导致系统 curl/wget 运行时出现符号版本不匹配。
        local -a clean_env_cmd=(env -u LD_LIBRARY_PATH -u PYTHONPATH)
        if command -v curl &> /dev/null; then
            "${clean_env_cmd[@]}" curl -L -o "$output_file" "$url" --progress-bar
        elif command -v wget &> /dev/null; then
            "${clean_env_cmd[@]}" wget -O "$output_file" "$url"
        else
            log_error "  未找到 curl 或 wget"
            return 1
        fi
    fi
    
    # 校验 sha256
    if [ -n "$expected_sha256" ]; then
        local actual_sha256=$(sha256sum "$output_file" | cut -d' ' -f1)
        if [ "$actual_sha256" != "$expected_sha256" ]; then
            log_error "  SHA256 校验失败!"
            log_error "    期望: $expected_sha256"
            log_error "    实际: $actual_sha256"
            run_cmd rm -f "$output_file"
            return 1
        fi
        log_success "  SHA256 校验通过"
    fi
    
    return 0
}

# 检查是否需要更新工具链
# 通过校验缓存的压缩包 sha256 来判断
needs_toolchain_update() {
    local toolchain_name=$1
    local expected_sha256=$2
    local cache_dir="$INSTALL_DIR/.cache"
    local cached_file="$cache_dir/${toolchain_name}.tar.gz"
    
    if [ "$FORCE_INSTALL" = true ]; then
        return 0
    fi
    
    # 检查缓存的压缩包是否存在
    if ! cached_file_exists "$cached_file"; then
        log_info "  缓存不存在: ${toolchain_name}.tar.gz"
        return 0
    fi
    
    # 计算缓存压缩包的实际 sha256
    log_info "  校验缓存: ${toolchain_name}.tar.gz ..."
    local actual_sha256
    actual_sha256=$(cached_file_sha256 "$cached_file")
    
    if [ "$actual_sha256" != "$expected_sha256" ]; then
        log_warn "  缓存校验失败，需要重新下载"
        log_warn "    期望: ${expected_sha256:0:16}..."
        log_warn "    实际: ${actual_sha256:0:16}..."
        return 0
    fi
    
    # 检查工具链是否已安装
    local toolchain_dir="$INSTALL_DIR/toolchain/$toolchain_name"
    if [ ! -d "$toolchain_dir" ]; then
        log_info "  工具链目录不存在，需要安装"
        return 0
    fi
    
    # 缓存存在且校验通过，无需更新
    return 1
}

# 检查是否需要更新三方库
# 通过检查库文件是否存在、校验缓存的压缩包 sha256 来判断
needs_thirdparty_update() {
    local pkg_name=$1
    local platform=$2
    local expected_sha256=$3
    local version=$4
    local pattern=$5
    local cache_dir="$INSTALL_DIR/.cache"
    local cached_file="$cache_dir/${pkg_name}-${version}-${platform}.tar.gz"
    local lib_dir="$INSTALL_DIR/galbot_sdk/$platform/lib"

    if [ "$FORCE_INSTALL" = true ]; then
        return 0
    fi

    # 检查库文件是否存在（使用 pattern 匹配）
    local lib_pattern="$pattern"
    if [ -z "$lib_pattern" ]; then
        lib_pattern="lib${pkg_name}*"
    fi

    if [ ! -d "$lib_dir" ]; then
        log_info "  lib 目录不存在，需要安装"
        return 0
    fi

    # 检查 lib 目录下是否存在匹配的库文件
    # pattern 可能包含多个模式，用 | 分隔（OR 逻辑，任意一个匹配即可）
    local lib_files=""
    if [[ "$lib_pattern" == *"|"* ]]; then
        # 多模式：逐个检查，累积结果
        local IFS='|'
        for p in $lib_pattern; do
            local matches=$(find "$lib_dir" -maxdepth 1 \( -type f -o -type l \) -name "$p" 2>/dev/null)
            lib_files="$lib_files$matches"
        done
    else
        # 单模式
        lib_files=$(find "$lib_dir" -maxdepth 1 \( -type f -o -type l \) -name "$lib_pattern" 2>/dev/null)
    fi

    if [ -z "$lib_files" ]; then
        log_info "  库文件不存在: $lib_pattern"
        return 0
    fi

    # 检查缓存的压缩包是否存在
    if ! cached_file_exists "$cached_file"; then
        return 0
    fi

    # 计算缓存压缩包的实际 sha256
    local actual_sha256
    actual_sha256=$(cached_file_sha256 "$cached_file")

    if [ "$actual_sha256" != "$expected_sha256" ]; then
        log_warn "  $pkg_name 缓存校验失败，需要重新下载"
        return 0
    fi

    # 库文件存在，缓存存在且校验通过，无需更新
    return 1
}

# 安装工具链（单平台）
install_toolchain_for_platform() {
    local platform=$1
    local toolchain_json=""
    
    if [ -f "$SCRIPT_DIR/deps/toolchain.json" ]; then
        toolchain_json="$SCRIPT_DIR/deps/toolchain.json"
    else
        log_warn "未找到 toolchain.json，跳过工具链安装"
        return 0
    fi
    
    log_info "处理工具链 (平台: $platform)..."
    
    local toolchain_name=$(get_toolchain_name "$platform")
    
    # 解析工具链信息
    local toolchain_info=$(python3 -c "
import json
data = json.load(open('$toolchain_json'))
for tc in data.get('toolchains', []):
    if tc.get('target_platform') == '$platform':
        print(json.dumps(tc))
        break
" 2>/dev/null)
    
    if [ -z "$toolchain_info" ]; then
        log_warn "未找到平台 $platform 的工具链配置"
        return 0
    fi
    
    local url=$(echo "$toolchain_info" | python3 -c "import json,sys; print(json.load(sys.stdin).get('url', ''))")
    local local_path=$(echo "$toolchain_info" | python3 -c "import json,sys; print(json.load(sys.stdin).get('local_path', ''))")
    local sha256=$(echo "$toolchain_info" | python3 -c "import json,sys; print(json.load(sys.stdin).get('sha256', ''))")
    local version=$(echo "$toolchain_info" | python3 -c "import json,sys; print(json.load(sys.stdin).get('version', ''))")
    
    # 缓存目录和缓存文件路径
    local cache_dir="$INSTALL_DIR/.cache"
    local cached_file="$cache_dir/${toolchain_name}.tar.gz"
    
    if needs_toolchain_update "$toolchain_name" "$sha256"; then
        if [ "$CHECK_ONLY" = true ]; then
            log_info "  需要更新: $toolchain_name (v$version)"
            return 0
        fi

        log_info "  安装工具链: $toolchain_name (v$version)"

        # 创建缓存目录
        run_cmd mkdir -p "$cache_dir"

        # 检查缓存文件是否已存在且校验通过，避免重复下载
        local cache_valid=false
        if cached_file_exists "$cached_file"; then
            local actual_sha256
            actual_sha256=$(cached_file_sha256 "$cached_file")
            if [ "$actual_sha256" = "$sha256" ]; then
                cache_valid=true
                log_success "  使用已有缓存: ${toolchain_name}.tar.gz"
            fi
        fi

        # 如果缓存无效，才下载
        if [ "$cache_valid" = false ]; then
            # 下载到缓存目录
            local tmp_file=$(mktemp)

            if ! download_file "$url" "$local_path" "$tmp_file" "$sha256"; then
                log_error "  工具链下载失败"
                run_cmd rm -f "$tmp_file"
                return 1
            fi

            # 移动到缓存目录
            run_cmd mv "$tmp_file" "$cached_file"
        fi

        # 解压到安装目录
        local toolchain_dir="$INSTALL_DIR/toolchain"
        run_cmd mkdir -p "$toolchain_dir"

        log_info "  解压到 $toolchain_dir/"
        run_cmd tar xzf "$cached_file" -C "$toolchain_dir" --no-same-owner

        log_success "  工具链安装完成: $toolchain_name"
    else
        log_success "  工具链校验通过，已是最新: $toolchain_name"
    fi
}

# 安装工具链（支持多平台）
install_toolchain() {
    if [ "$ALL_PLATFORMS" = true ]; then
        log_info "安装所有平台的工具链..."
        for platform in "${SUPPORTED_PLATFORMS[@]}"; do
            install_toolchain_for_platform "$platform"
            echo
        done
    else
        install_toolchain_for_platform "$TARGET_PLATFORM"
    fi
}

# 安装三方库（单平台）
install_thirdparty_for_platform() {
    local platform=$1
    local thirdparty_json=""
    
    if [ -f "$SCRIPT_DIR/deps/third_party.json" ]; then
        thirdparty_json="$SCRIPT_DIR/deps/third_party.json"
    else
        log_warn "未找到 third_party.json，跳过三方库安装"
        return 0
    fi
    
    log_info "处理三方库 (平台: $platform)..."
    
    # 获取所有包
    local packages=$(python3 -c "
import json
data = json.load(open('$thirdparty_json'))
for pkg in data.get('packages', []):
    name = pkg.get('name', '')
    version = pkg.get('version', '')
    pattern = pkg.get('pattern', '')
    platform_info = pkg.get('platforms', {}).get('$platform', {})
    if platform_info:
        url = platform_info.get('url', '')
        local_path = platform_info.get('local_path', '')
        sha256 = platform_info.get('sha256', '')
        print(f'{name}|{version}|{url}|{local_path}|{sha256}|{pattern}')
" 2>/dev/null)
    
    local lib_dir="$INSTALL_DIR/galbot_sdk/$platform/lib"
    local cache_dir="$INSTALL_DIR/.cache"
    
    while IFS='|' read -r name version url local_path sha256 pattern; do
        [ -z "$name" ] && continue
        
        local cached_file="$cache_dir/${name}-${version}-${platform}.tar.gz"
        
        if needs_thirdparty_update "$name" "$platform" "$sha256" "$version" "$pattern"; then
            if [ "$CHECK_ONLY" = true ]; then
                log_info "  需要更新: $name (v$version)"
                continue
            fi

            log_info "  安装: $name (v$version)"

            # 创建缓存目录
            run_cmd mkdir -p "$cache_dir"

            # 清理该库的旧版本缓存（保留当前版本）（列目录与 run_cmd 一致，避免读不到 root 的 .cache）
            while IFS= read -r old_cache; do
                [ -z "$old_cache" ] && continue
                if [ "$old_cache" != "$cached_file" ]; then
                    log_info "  清理旧版本缓存: $(basename "$old_cache")"
                    run_cmd rm -f "$old_cache"
                fi
            done < <(run_cmd find "$cache_dir" -maxdepth 1 -name "${name}-*-${platform}.tar.gz" 2>/dev/null)

            # 检查缓存文件是否已存在且校验通过，避免重复下载
            local cache_valid=false
            if cached_file_exists "$cached_file"; then
                local actual_sha256
                actual_sha256=$(cached_file_sha256 "$cached_file")
                if [ "$actual_sha256" = "$sha256" ]; then
                    cache_valid=true
                    log_success "  使用已有缓存: ${name}-${version}-${platform}.tar.gz"
                fi
            fi

            # 如果缓存无效，才下载
            if [ "$cache_valid" = false ]; then
                # 下载到临时目录
                local tmp_file=$(mktemp)

                if ! download_file "$url" "$local_path" "$tmp_file" "$sha256"; then
                    log_error "  三方库 $name 平台 $platform 下载/校验失败，安装中止，请检查网络后重新下载"
                    run_cmd rm -f "$tmp_file"
                    return 1
                fi

                # 移动到缓存目录
                run_cmd mv "$tmp_file" "$cached_file"
            fi

            # 清理旧版本文件（在解压前）
            # 从 JSON 中读取 pattern，如果没有则使用默认规则
            local lib_pattern="$pattern"
            if [ -z "$lib_pattern" ]; then
                # 默认模式：lib{name}*
                lib_pattern="lib${name}*"
            fi

            if [ -n "$lib_pattern" ] && [ -d "$lib_dir" ]; then
                log_info "  清理旧版本文件: $lib_pattern"
                # 查找并删除匹配的文件（包括符号链接）
                find "$lib_dir" -maxdepth 1 \( -type f -o -type l \) -name "$lib_pattern" -exec rm -f {} \; 2>/dev/null || true
            fi

            # 解压到 lib 目录
            run_cmd mkdir -p "$lib_dir"
            run_cmd tar xzf "$cached_file" -C "$lib_dir" --strip-components=1 --no-same-owner

            log_success "  $name 安装完成"
        else
            log_success "  校验通过: $name"
        fi
    done <<< "$packages"
}

# 安装三方库（支持多平台）
install_thirdparty() {
    if [ "$ALL_PLATFORMS" = true ]; then
        log_info "安装所有平台的三方库..."
        for platform in "${SUPPORTED_PLATFORMS[@]}"; do
            install_thirdparty_for_platform "$platform"
            echo
        done
    else
        install_thirdparty_for_platform "$TARGET_PLATFORM"
    fi
}

# 安装 SDK 核心库（单平台）
install_sdk_core_for_platform() {
    local platform=$1
    log_info "安装 SDK 核心库 (平台: $platform)..."
    
    local sdk_dir="$INSTALL_DIR/galbot_sdk"
    
    # 创建目录结构
    run_cmd mkdir -p "$sdk_dir/$platform/include"
    run_cmd mkdir -p "$sdk_dir/$platform/lib"
    run_cmd mkdir -p "$sdk_dir/$platform/bin"
    
    # 复制头文件
    if [ -d "$SCRIPT_DIR/galbot_sdk/$platform/include" ]; then
        log_info "  复制头文件..."
        run_cmd cp -r "$SCRIPT_DIR/galbot_sdk/$platform/include/"* "$sdk_dir/$platform/include/"
    fi
    
    # 复制 SDK 核心库
    if [ -d "$SCRIPT_DIR/galbot_sdk/$platform/lib" ]; then
        log_info "  复制 SDK 核心库..."
        
        # 只复制 SDK 核心库（三方库通过 install_thirdparty 安装）
        for f in "$SCRIPT_DIR/galbot_sdk/$platform/lib/"libgalbot_sdk.so*; do
            if [ -e "$f" ]; then
                run_cmd cp -a "$f" "$sdk_dir/$platform/lib/"
            fi
        done
    fi
    
    # 复制 bin 目录
    if [ -d "$SCRIPT_DIR/galbot_sdk/$platform/bin" ]; then
        log_info "  复制可执行文件..."
        run_cmd cp -r "$SCRIPT_DIR/galbot_sdk/$platform/bin/"* "$sdk_dir/$platform/bin/" 2>/dev/null || true
    fi
    
    # 复制 setup.sh
    if [ -f "$SCRIPT_DIR/galbot_sdk/$platform/setup.sh" ]; then
        run_cmd cp "$SCRIPT_DIR/galbot_sdk/$platform/setup.sh" "$sdk_dir/$platform/"
    fi
    
    # 复制 common.cmake
    if [ -f "$SCRIPT_DIR/galbot_sdk/common.cmake" ]; then
        run_cmd cp "$SCRIPT_DIR/galbot_sdk/common.cmake" "$sdk_dir/"
    fi
    
    # 复制 python 绑定
    if [ -d "$SCRIPT_DIR/galbot_sdk/$platform/lib/python" ]; then
        log_info "  复制 Python 绑定..."
        run_cmd cp -r "$SCRIPT_DIR/galbot_sdk/$platform/lib/python" "$sdk_dir/$platform/lib/"
    fi
    
    log_success "  SDK 核心库安装完成: $platform"
}

# 安装 SDK 核心库（支持多平台）
install_sdk_core() {
    if [ "$ALL_PLATFORMS" = true ]; then
        log_info "安装所有平台的 SDK 核心库..."
        for platform in "${SUPPORTED_PLATFORMS[@]}"; do
            install_sdk_core_for_platform "$platform"
            echo
        done
    else
        install_sdk_core_for_platform "$TARGET_PLATFORM"
    fi
}

# 安装配置文件和示例
install_extras() {
    log_info "安装配置文件和示例..."
    
    # 安装 config 到机器人运行目录（可选，仅在有权限时）
    if [ -d "$SCRIPT_DIR/config" ]; then
        if [ "$USE_SUDO" = true ]; then
            log_info "  安装配置文件到 /data/config..."
            run_cmd mkdir -p /data/config
            run_cmd cp -rn "$SCRIPT_DIR/config/"* /data/config/ 2>/dev/null || \
                run_cmd cp -r "$SCRIPT_DIR/config/"* /data/config/
        else
            log_info "  复制配置文件到安装目录..."
            run_cmd mkdir -p "$INSTALL_DIR/config"
            run_cmd cp -r "$SCRIPT_DIR/config/"* "$INSTALL_DIR/config/"
        fi
    fi
    
    # 复制 examples
    if [ -d "$SCRIPT_DIR/examples" ]; then
        log_info "  复制示例代码..."
        run_cmd mkdir -p "$INSTALL_DIR/examples"
        run_cmd cp -r "$SCRIPT_DIR/examples/"* "$INSTALL_DIR/examples/"
        
        # 更新 examples 中的路径
        update_example_paths
    fi
    
    # 复制文档
    if [ -d "$SCRIPT_DIR/docs" ]; then
        log_info "  复制文档..."
        run_cmd mkdir -p "$INSTALL_DIR/docs"
        run_cmd cp -r "$SCRIPT_DIR/docs/"* "$INSTALL_DIR/docs/"
    fi
    
    # 复制 licenses
    if [ -d "$SCRIPT_DIR/licenses" ]; then
        run_cmd mkdir -p "$INSTALL_DIR/licenses"
        run_cmd cp -r "$SCRIPT_DIR/licenses/"* "$INSTALL_DIR/licenses/"
    fi
    
    # 复制 README
    if [ -f "$SCRIPT_DIR/README.md" ]; then
        run_cmd cp "$SCRIPT_DIR/README.md" "$INSTALL_DIR/"
    fi
    
    log_success "  配置文件和示例安装完成"
}

# 安装版本检查工具到各平台 bin 目录
install_tools() {
    log_info "安装版本检查工具..."

    # 复制 check_robot_compat.py 和 wrapper
    local compat_script="$SCRIPT_DIR/check_robot_compat.py"
    local wrapper_script="$SCRIPT_DIR/galbot_sdk_wrapper.sh"
    local embosa_ip_script="$SCRIPT_DIR/configure_embosa_ip.sh"

    if [ ! -f "$compat_script" ]; then
        log_warn "未找到 check_robot_compat.py，跳过工具安装"
        return 0
    fi

    if [ ! -f "$wrapper_script" ]; then
        log_warn "未找到 galbot_sdk_wrapper.sh，跳过工具安装"
        return 0
    fi

    local platforms=()
    if [ "$ALL_PLATFORMS" = true ]; then
        platforms=("${SUPPORTED_PLATFORMS[@]}")
    else
        platforms=("$TARGET_PLATFORM")
    fi

    for platform in "${platforms[@]}"; do
        local bin_dir="$INSTALL_DIR/galbot_sdk/$platform/bin"
        run_cmd mkdir -p "$bin_dir"

        run_cmd cp "$compat_script" "$bin_dir/"
        run_cmd chmod +x "$bin_dir/check_robot_compat.py"

        # 复制 galbot_sdk wrapper
        run_cmd cp "$wrapper_script" "$bin_dir/galbot_sdk"
        run_cmd chmod +x "$bin_dir/galbot_sdk"

        # 复制 embosa IP 配置脚本
        if [ -f "$embosa_ip_script" ]; then
            run_cmd cp "$embosa_ip_script" "$bin_dir/configure_embosa_ip.sh"
            run_cmd chmod +x "$bin_dir/configure_embosa_ip.sh"
        fi

        log_success "  版本检查工具安装完成: $platform"
    done
}

# 更新示例代码中的路径
update_example_paths() {
    local sdk_path="$INSTALL_DIR/galbot_sdk"
    local toolchain_path="$sdk_path/toolchain"
    local new_line="set(toolchain $INSTALL_DIR/toolchain/\${toolchain_target_plat})"
    local sdk_line="set(SDK_INSTALL_PATH $INSTALL_DIR)"

    # 更新 cmake toolchain 路径（仅 g1/s1 的 cpp/cmake）
    local cmake_locations=(
        "$INSTALL_DIR/examples/g1/cpp/cmake"
        "$INSTALL_DIR/examples/s1/cpp/cmake"
    )
    for dir in "${cmake_locations[@]}"; do
        for f in "$dir/linux-aarch64-gcc940.cmake" "$dir/linux-x86_64-gcc940.cmake"; do
            if [ -f "$f" ]; then
                run_cmd sed -i "7c\\$new_line" "$f"
            fi
        done
    done

    # 更新 CMakeLists.txt 中的 SDK 路径（g1/s1 各自 cpp 目录）
    for cmake_file in "$INSTALL_DIR/examples/g1/cpp/CMakeLists.txt" \
                      "$INSTALL_DIR/examples/s1/cpp/CMakeLists.txt"; do
        if [ -f "$cmake_file" ]; then
            run_cmd sed -i "7c\\$sdk_line" "$cmake_file"
        fi
    done
}

# 显示安装摘要
show_summary() {
echo
echo "=========================================="
    echo " 安装完成!"
    echo "=========================================="
    echo
    echo "安装目录: $INSTALL_DIR"
    echo "SDK 版本: $SDK_VERSION"
    if [ "$ALL_PLATFORMS" = true ]; then
        echo "已安装平台: ${SUPPORTED_PLATFORMS[*]}"
    else
        echo "目标平台: $TARGET_PLATFORM"
    fi
    echo
    echo "目录结构:"
    echo "  $INSTALL_DIR/"
    echo "  ├── .cache/              # 依赖压缩包缓存（用于校验）"
    echo "  ├── toolchain/"
    if [ "$ALL_PLATFORMS" = true ]; then
        for platform in "${SUPPORTED_PLATFORMS[@]}"; do
            echo "  │   ├── $(get_toolchain_name $platform)/"
        done
    else
        echo "  │   └── $(get_toolchain_name $TARGET_PLATFORM)/"
    fi
    echo "  ├── galbot_sdk/"
    if [ "$ALL_PLATFORMS" = true ]; then
        for platform in "${SUPPORTED_PLATFORMS[@]}"; do
            echo "  │   ├── $platform/"
            echo "  │   │   ├── include/"
            echo "  │   │   ├── lib/"
            echo "  │   │   ├── bin/         # 版本检查工具"
            echo "  │   │   └── setup.sh"
        done
    else
        echo "  │   └── $TARGET_PLATFORM/"
        echo "  │       ├── include/"
        echo "  │       ├── lib/"
        echo "  │       ├── bin/         # 版本检查工具"
        echo "  │       └── setup.sh"
    fi
    echo "  ├── examples/"
    echo "  └── docs/"
    echo
    echo "使用方法:"
    echo "  # 检查 SDK 与机器人版本兼容性"
    echo "  galbot_sdk check-version [--robot-ip <IP>]"
    echo
    if [ "$ALL_PLATFORMS" = true ]; then
        for platform in "${SUPPORTED_PLATFORMS[@]}"; do
            echo "  # $platform:"
            echo "  source $INSTALL_DIR/galbot_sdk/$platform/setup.sh"
        done
    else
        echo "  source $INSTALL_DIR/galbot_sdk/$TARGET_PLATFORM/setup.sh"
    fi
    echo
    echo "=========================================="
}

welcome_verify_cpp_executable() {
    local model=$1
    local plat=$2
    echo "$INSTALL_DIR/galbot_sdk/$plat/bin/$model/galbot_robot/galbot_robot_installation_welcome_verify"
}

run_welcome_verify_g1() {
    local run_plat
    run_plat=$(detect_platform)
    local setup_sh="$INSTALL_DIR/galbot_sdk/$run_plat/setup.sh"
    local cpp_bin
    cpp_bin="$(welcome_verify_cpp_executable g1 "$run_plat")"
    log_info "本机平台: $run_plat（setup.sh 与可执行文件均来自 galbot_sdk/$run_plat/）"
    if [ ! -f "$setup_sh" ]; then
        log_error "未找到环境脚本（用于 LD_LIBRARY_PATH）: $setup_sh"
        return 1
    fi
    if [ ! -f "$cpp_bin" ] || [ ! -x "$cpp_bin" ]; then
        log_error "未找到 G1 欢迎自检可执行文件: $cpp_bin"
        return 1
    fi

    local pcm_default="$INSTALL_DIR/examples/g1/assets/audio/welcome_16k_mono.pcm"
    local pcm_path=""
    if [ -f "$pcm_default" ]; then
        pcm_path="$pcm_default"
        log_info "使用内置欢迎音频: $pcm_path"
    else
        read -r -p "未找到默认 PCM，请输入 welcome 音频绝对路径（16 kHz mono s16le .pcm）: " pcm_path || true
        pcm_path="${pcm_path//\"/}"
        pcm_path="${pcm_path#"${pcm_path%%[![:space:]]*}"}"
        pcm_path="${pcm_path%"${pcm_path##*[![:space:]]}"}"
        if [ -z "$pcm_path" ] || [ ! -f "$pcm_path" ]; then
            log_error "PCM 路径无效或文件不存在；G1 自检需要环境变量 GALBOT_WELCOME_PCM。"
            return 1
        fi
    fi

    export GALBOT_WELCOME_PCM="$pcm_path"
    log_info "已导出 GALBOT_WELCOME_PCM=$GALBOT_WELCOME_PCM"

    log_info "执行 G1 安装欢迎自检: $cpp_bin"
    set +e
    (
        source "$setup_sh"
        "$cpp_bin"
    )
    local cpp_ret=$?
    set -e

    return 0
}

run_welcome_verify_s1() {
    local run_plat
    run_plat=$(detect_platform)
    local setup_sh="$INSTALL_DIR/galbot_sdk/$run_plat/setup.sh"
    local cpp_bin
    cpp_bin="$(welcome_verify_cpp_executable s1 "$run_plat")"

    if [ ! -f "$setup_sh" ]; then
        log_error "未找到环境脚本（用于 LD_LIBRARY_PATH）: $setup_sh"
        return 1
    fi
    if [ ! -f "$cpp_bin" ] || [ ! -x "$cpp_bin" ]; then
        log_error "未找到 S1 欢迎自检可执行文件: $cpp_bin"
        return 1
    fi

    log_info "执行 S1 安装欢迎自检: $cpp_bin"
    set +e
    (
        source "$setup_sh"
        "$cpp_bin"
    )
    local cpp_ret=$?
    set -e

    return 0
}

# 自动配置本地 PC 的 PATH 环境变量
configure_path_local() {
    if [ "$CHECK_ONLY" = true ]; then
        return 0
    fi

    log_info "自动配置本地 PATH 环境变量..."

    # 获取目标用户（sudo 下用原始用户）
    local target_user="$USER"
    if [ -n "$SUDO_USER" ]; then
        target_user="$SUDO_USER"
    fi

    # 检测用户默认 shell
    local user_shell="bash"
    local user_default_shell
    user_default_shell=$(getent passwd "$target_user" | cut -d: -f7)
    if [[ "$user_default_shell" == *"/zsh" ]]; then
        user_shell="zsh"
    fi

    # 获取目标用户主目录（sudo 下使用原始用户）
    local target_home="$HOME"
    if [ -n "$SUDO_USER" ]; then
        target_home=$(getent passwd "$SUDO_USER" | cut -d: -f6)
        if [ -z "$target_home" ]; then
            target_home="$HOME"
        fi
    fi

    local rcfile
    local setup_cmd
    local install_subdir

    if [ "$ALL_PLATFORMS" = true ]; then
        # 多平台模式，自动检测当前平台写入 rc
        install_subdir=$(detect_platform)
        log_info "多平台安装模式，自动配置当前平台 PATH: $install_subdir"
    else
        install_subdir="$TARGET_PLATFORM"
    fi
    rcfile="$target_home/.${user_shell}rc"
    setup_cmd="source $INSTALL_DIR/galbot_sdk/$install_subdir/setup.sh"

    # 检查是否已配置
    if grep -q "galbot_sdk.*setup.sh" "$rcfile" 2>/dev/null; then
        log_info "PATH 配置已存在于 $rcfile，跳过"
        return 0
    fi

    # 添加配置
    echo "" >> "$rcfile"
    echo "# Galbot SDK PATH 配置 - $(date '+%Y-%m-%d %H:%M:%S')" >> "$rcfile"
    echo "$setup_cmd" >> "$rcfile"

    log_success "PATH 配置已添加到 $rcfile (用户: $target_user)"
    log_info "运行 'source $rcfile' 或重新登录使配置生效"
}

# 安装完成后询问是否运行欢迎自检（-y / NON_INTERACTIVE 时跳过）
prompt_run_installation_welcome_verify() {
    if [ "$NON_INTERACTIVE" = true ]; then
        return 0
    fi

    echo
    read -r -p "安装已完成。是否现在运行验证程序，将进行简单活动关节 [y/n]（直接回车为否）: " welcome_ans
    case "$welcome_ans" in
        [yY]|[yY][eE][sS]) ;;
        *) return 0 ;;
    esac

    echo
    log_info "运行前请完成以下准备："
    echo "  • 机器人周围空旷"
    echo "  • 如在外部 PC 中运行程序，请与机器人连接好网线，并按照文档配置本机 PC 与机器人的通讯配置文件与 IP"
    echo "  • 将机器人背后急停旋钮旋开（解除急停）"
    echo "  详细步骤请参考说明文档，本地文档查看方式如下："
    echo
    echo "    cd docs"
    echo "    python3 -m http.server 8000"
    echo
    echo "然后在浏览器中打开： http://0.0.0.0:8000/"
    echo "  （参考安装与配置章节「模式二：PC端部署」）"
    echo

    read -r -p "完成上述准备后输入 y 继续 [y/n]（直接回车或其它键取消）: " ready_ans
    case "$ready_ans" in
        [yY]) ;;
        *)
            log_warn "已取消运行检测程序。"
            return 0
            ;;
    esac

    local model_choice=""
    read -r -p "请选择机型  [1] G1  [2] S1 : " model_choice
    case "$model_choice" in
        1) run_welcome_verify_g1 ;;
        2) run_welcome_verify_s1 ;;
        *)
            log_error "无效选择，已跳过自检程序。"
            return 0
            ;;
    esac
}

prompt_yes_no() {
    local msg=$1
    local default=${2:-n}
    if [ "$NON_INTERACTIVE" = true ]; then
        if [ "$default" = y ]; then
            return 0
        fi
        return 1
    fi
    local suffix="[y/n]"
    [ "$default" = y ] && suffix="[y/n]"
    local ans=""
    read -r -p "$msg $suffix: " ans || return 1
    if [ -z "$ans" ]; then
        [ "$default" = y ] && return 0 || return 1
    fi
    case "$ans" in
        [yY]|[yY][eE][sS]) return 0 ;;
        [nN]|[nN][oO]) return 1 ;;
        *) return 1 ;;
    esac
}

# 安装后可选步骤：Python 依赖、部署到机器人
post_install_optional_steps() {
    if [ "$CHECK_ONLY" = true ]; then
        return 0
    fi

    echo
    log_info "========== 可选：安装后配置（每步可单独选择） =========="

    # ---------- 部署到机器人 ----------
    local aarch_lib="$INSTALL_DIR/galbot_sdk/linux-aarch64-gcc940/lib"
    if prompt_yes_no "是否将 SDK 库部署到机器人（若只在PC中运行SDK程序或已在机器人中安装完成可跳过）" n; then
        if [ ! -d "$aarch_lib" ]; then
            log_error "未找到机器人SDK库目录: $aarch_lib"
            log_info "请先完成带 linux-aarch64-gcc940 的安装后，再进行部署"
            log_info "手动部署：cd \"$INSTALL_DIR\" && bash \"$SCRIPT_DIR/deploy_to_robot.sh\""
        else
            log_info "即将在目录 \"$INSTALL_DIR\" 下运行部署脚本（相对路径 galbot_sdk/linux-aarch64-gcc940/lib）。"
            set +e
            (
                cd "$INSTALL_DIR" && bash "$SCRIPT_DIR/deploy_to_robot.sh"
            )
            local dep_ret=$?
            set -e
            if [ "$dep_ret" -ne 0 ]; then
                log_error "机器人部署失败（退出码 $dep_ret）。"
                log_info "可确认本机与机器人网线连接正常后，手动执行: $SCRIPT_DIR/deploy_to_robot.sh脚本部署"
            else
                log_success "机器人部署步骤已结束。"
            fi
        fi
    fi

    # ---------- Python 依赖 ----------
    if prompt_yes_no "是否安装 Python example 所需第三方依赖（耗时可能较长；若只做 C++ 开发或不需要运行 Python example 可跳过）" n; then
        log_info "运行: $SCRIPT_DIR/install_python_deps.sh"
        set +e
        source "$SCRIPT_DIR/install_python_deps.sh"
        local py_ret=$?
        set -e
        if [ "$py_ret" -ne 0 ]; then
            log_error "Python example 依赖下载失败（退出码 $py_ret）。"
            log_info "可使用 pip install 自行安装依赖包"
            prompt_yes_no "是否继续后续可选步骤？" y || return 0
        else
            log_success "Python 依赖下载已完成。"
        fi
    fi

    log_info "========== 可选步骤结束 =========="
}

#######################################
# 主程序
#######################################

main() {
    echo "=========================================="
    echo " GALBOT_SDK 安装程序 v${SDK_VERSION}"
    echo "=========================================="
    echo
    
    # 解析参数
    parse_args "$@"
    
    # 检测/确认平台
    if [ "$ALL_PLATFORMS" = true ]; then
        log_info "将安装所有支持的平台: ${SUPPORTED_PLATFORMS[*]}"
        # 设置一个默认平台用于某些单平台操作（如示例路径更新）
        if [ -z "$TARGET_PLATFORM" ]; then
            TARGET_PLATFORM=$(detect_platform)
        fi
        log_info "当前检测平台（用于示例路径）: $TARGET_PLATFORM"
    else
        # 单平台模式
        if [ -z "$TARGET_PLATFORM" ]; then
            TARGET_PLATFORM=$(detect_platform)
            log_info "检测到平台: $TARGET_PLATFORM"
        else
            log_info "指定平台: $TARGET_PLATFORM"
        fi
    fi
    
    # 确认安装目录
    if [ -z "$INSTALL_DIR" ]; then
        if [ "$NON_INTERACTIVE" = true ]; then
            INSTALL_DIR="$DEFAULT_INSTALL_DIR"
        else
            read -p "安装目录 [$DEFAULT_INSTALL_DIR]: " INSTALL_DIR
            INSTALL_DIR="${INSTALL_DIR:-$DEFAULT_INSTALL_DIR}"
        fi
    fi
    
    log_info "安装目录: $INSTALL_DIR"
    log_info "目标平台: $TARGET_PLATFORM"
    echo
    
    # 创建安装与日志目录
    run_cmd mkdir -p "$INSTALL_DIR"
    run_cmd mkdir -p "$EMBOSA_LOG_DIR"
    run_cmd chmod 777 "$EMBOSA_LOG_DIR"
    
    # 安装工具链
    install_toolchain
    echo
    
    # 安装三方库
    install_thirdparty
    echo
    
    if [ "$CHECK_ONLY" = true ]; then
        log_info "检查完成（--check 模式，未进行实际安装）"
        exit 0
    fi
    
    # 安装 SDK 核心库
    install_sdk_core
    echo
    
    # 安装配置文件和示例
    install_extras
    echo

    # 安装版本检查工具
    install_tools
    echo

    # 自动配置 PATH (仅 PC 端)
    configure_path_local
    echo

    # 显示摘要
    show_summary
    echo

    # 运行欢迎程序
    prompt_run_installation_welcome_verify

    # 可选：下载Python 依赖、部署到机器人
    post_install_optional_steps
}

# 运行主程序
main "$@"
