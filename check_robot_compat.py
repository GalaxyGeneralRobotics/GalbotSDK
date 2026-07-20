#!/usr/bin/env python3
"""
Galbot SDK 版本兼容性检查工具

功能:
    检查当前 SDK 版本与机器人系统版本(GBS)是否兼容，提供升级/降级建议。

版本对应关系:
    SDK 1.5.x ~ 1.7.x  →  机器人 GBS 1.15.x
    SDK 1.8.x          →  机器人 GBS 1.16.x

SDK 版本获取优先级:
    1. 用户手动指定 (--sdk-version)
    2. 从 galbot_sdk 包动态获取 (__version__)
    3. 若无法自动获取，则提示用户通过 --sdk-version 手动指定

机器人版本获取优先级:
    1. 用户手动指定 (--robot-version, 优先级最高)
    2. 读取本地 system.cfg 文件 (CUR_VERSION)
    3. 通过 SSH 读取远程 system.cfg（默认: galbot@192.168.1.88，有线直连 Orin）
    4. 通过 SDK 连接读取 (firmware_version, 耗时较长，最后尝试)
    5. 自动探测 (依次尝试上述方法)

使用方法:
    # 默认方式：自动检测 SDK 和机器人版本
    galbot_sdk check-version

    # 离线验证：手动指定版本
    galbot_sdk check-version --sdk-version 1.8.0 --robot-version GBS_1.16.0

    可选参数:
      --sdk-version      指定 SDK 版本号
      --robot-version    指定机器人版本号
      --source           版本来源: auto(默认), local_cfg, sdk, ssh_cfg
      --robot-ip         机器人 IP (默认: 192.168.1.88)
      --user             远程用户名 (默认: galbot)
      --system-cfg       system.cfg 文件路径
      --list             查看完整版本对照表
      --json             JSON 格式输出

退出码:
    0 - 版本兼容
    1 - 版本不匹配
    2 - 检查过程出错

示例输出 (匹配):
    ==================================================
        Galbot SDK 版本兼容性检查
    ==================================================
      SDK 版本:         1.8.0
      SDK 版本来源:      python:galbot_sdk.__version__
      要求 GBS 版本:    >= GBS_1.16.0
      机器人当前版本:   GBS_1.16.0
      版本获取方式:      sdk:get_device_information.firmware_version
    --------------------------------------------------
      检查结果:        MATCH
    --------------------------------------------------
    ✅ 版本兼容，可直接使用当前组合。
       机器人版本 GBS_1.16.0 满足 SDK 1.8.0 的要求（最低 GBS_1.16.0）

示例输出 (不匹配 — SDK 过高，机器人版本偏低):
    ❌ 版本不匹配
       机器人版本: GBS_1.15.0
       SDK 1.8.0 要求: GBS_1.16.x

    ⚠️  当前 SDK 版本与机器人系统版本不匹配，
        可能导致 SDK 部分功能异常或无法使用。

    👉 解决方案（按推荐优先级）：
       1. 【推荐】升级机器人系统到 GBS_1.16.x 或更高
          → 获得最新功能、性能优化和安全更新
       2. 【备选】降级 SDK 到 1.5.0~1.7.99
          → 仅当需要使用特定机器人版本时使用

示例输出 (不匹配 — 机器人版本过高，SDK 版本偏低):
    ❌ 版本不匹配
       机器人版本: GBS_1.17.0
       SDK 1.8.0 要求: GBS_1.16.x

    ⚠️  当前 SDK 版本与机器人系统版本不匹配，
        可能导致 SDK 部分功能异常或无法使用。

    👉 解决方案（按推荐优先级）：
       1. 【推荐】升级 SDK 到 1.9.0~1.9.99（与机器人 GBS_1.17.0 匹配）
          → 获得最新功能、性能优化和安全更新
"""

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

DEFAULT_CFG_PATH = "/data/config/system.cfg"

# 有线直连默认网络配置
# PC: 192.168.1.99, XCU: 192.168.1.66, Orin: 192.168.1.88
DEFAULT_SSH_HOST = "192.168.1.88"
DEFAULT_SSH_USER = "galbot"

EXIT_MATCH = 0
EXIT_MISMATCH = 1
EXIT_ERROR = 2

# SDK 连接超时时间（秒）
SDK_CONNECTION_TIMEOUT = 20

# SDK 版本与 GBS 版本兼容性映射表
# 格式: (SDK 最小版本, SDK 最大版本, [兼容的 GBS 版本列表])
# 示例: ("1.8.0", "1.8.99", ["GBS_1.16.x", "GBS_1.17.x"]) 表示 SDK 1.8.x 兼容 GBS 1.16.x 和 1.17.x
VERSION_COMPATIBILITY_MAP = [
    ("1.5.0", "1.7.99", ["GBS_1.15.x"]),
    ("1.8.0", "1.8.99", ["GBS_1.16.x"]),
    ("1.9.0", "1.9.99", ["GBS_1.17.x"]),
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="检查机器人版本与 SDK 依赖版本兼容性（默认优先通过 SDK 连接读取）。"
    )
    parser.add_argument("--json", action="store_true", help="输出 JSON")
    parser.add_argument(
        "--list", action="store_true", help="显示完整的 SDK 与 GBS 版本兼容性对照表"
    )
    parser.add_argument(
        "--source",
        choices=["auto", "sdk", "local_cfg", "ssh_cfg"],
        default="auto",
        help="机器人版本来源：auto(默认)、local_cfg、sdk、ssh_cfg",
    )
    parser.add_argument(
        "--system-cfg", default=DEFAULT_CFG_PATH, help="system.cfg 路径"
    )
    parser.add_argument(
        "--robot-ip",
        default=DEFAULT_SSH_HOST,
        help=f"机器人 IP（默认: {DEFAULT_SSH_HOST}）",
    )
    parser.add_argument(
        "--user",
        default=DEFAULT_SSH_USER,
        help=f"SSH 用户名（默认: {DEFAULT_SSH_USER}）",
    )
    parser.add_argument(
        "--port", type=int, default=22, help="SSH 端口（仅 ssh_cfg 使用）"
    )
    parser.add_argument(
        "--robot-version",
        help="手工指定机器人版本（用于离线验证，优先级最高）",
    )
    parser.add_argument(
        "--sdk-version",
        help="手工指定 SDK 版本（如 1.8.0），默认自动检测",
    )
    return parser.parse_args()


def normalize_version(ver):
    m = re.search(r"(\d+(?:\.\d+)*)", ver or "")
    if not m:
        raise ValueError(f"无法解析版本号: {ver}")
    return [int(x) for x in m.group(1).split(".")]


def version_ge(lhs, rhs):
    l_parts = normalize_version(lhs)
    r_parts = normalize_version(rhs)
    length = max(len(l_parts), len(r_parts))
    l_parts.extend([0] * (length - len(l_parts)))
    r_parts.extend([0] * (length - len(r_parts)))
    return l_parts >= r_parts


def version_in_range(ver, min_ver, max_ver):
    """检查版本是否在 [min_ver, max_ver] 范围内"""
    return version_ge(ver, min_ver) and version_ge(max_ver, ver)


def detect_sdk_version(user_specified_version=None):
    """
    智能检测 SDK 版本号，按优先级尝试：
    1. 用户手动指定 (--sdk-version)
    2. 直接 import galbot_sdk 包读取 __version__
    3. 无法自动获取时提示用户通过 --sdk-version 手动指定

    返回: (version, source_description, warning_message_or_None)
    """
    # 优先级1: 用户手动指定
    if user_specified_version:
        return user_specified_version, "manual:--sdk-version", None

    # 优先级2: 直接 import galbot_sdk 包读取 __version__
    import_error = None
    try:
        import galbot_sdk

        if hasattr(galbot_sdk, "__version__"):
            return galbot_sdk.__version__, "python:galbot_sdk.__version__", None
    except Exception as exc:
        import_error = exc

    detail = (
        f"导入 galbot_sdk 失败: {import_error}"
        if import_error
        else "galbot_sdk 包未提供 __version__"
    )
    raise RuntimeError(
        "无法自动获取当前 Python 环境中的 SDK 版本。\n"
        f"原因: {detail}\n"
        "请通过 --sdk-version 手动指定实际安装的 SDK 版本，例如:\n"
        "  galbot_sdk check-version --sdk-version 1.8.0\n"
        "或在离线校验时同时指定机器人版本:\n"
        "  galbot_sdk check-version --sdk-version 1.8.0 --robot-version GBS_1.16.0"
    )


def get_required_gbs_versions(sdk_version):
    """根据 SDK 版本从兼容性映射表中查找对应的所有兼容 GBS 版本"""
    for sdk_min, sdk_max, gbs_list in VERSION_COMPATIBILITY_MAP:
        if version_in_range(sdk_version, sdk_min, sdk_max):
            return gbs_list
    raise ValueError(
        f"当前 SDK 版本 {sdk_version} 不在已知兼容性列表中，"
        f"请运行 --list 查看支持的版本范围，或手动指定 --sdk-version"
    )


def get_matching_sdk_versions(robot_version):
    """根据机器人版本查找匹配的 SDK 版本范围"""
    robot_ver_num = normalize_version(robot_version)
    robot_major_minor = f"{robot_ver_num[0]}.{robot_ver_num[1]}"

    matching = []
    for sdk_min, sdk_max, gbs_list in VERSION_COMPATIBILITY_MAP:
        for gbs_ver in gbs_list:
            gbs_ver_num = normalize_version(gbs_ver)
            gbs_major_minor = f"{gbs_ver_num[0]}.{gbs_ver_num[1]}"
            if robot_major_minor == gbs_major_minor:
                matching.append((sdk_min, sdk_max, gbs_list))
                break  # 找到匹配即可，避免重复添加
    return matching


def get_matching_gbs_versions(sdk_version):
    """根据 SDK 版本查找匹配的所有 GBS 版本"""
    for sdk_min, sdk_max, gbs_list in VERSION_COMPATIBILITY_MAP:
        if version_in_range(sdk_version, sdk_min, sdk_max):
            return gbs_list
    return None


def format_version_list():
    """格式化输出完整的版本兼容性列表"""
    lines = ["=== SDK 与 GBS 版本兼容性对照表 ==="]
    lines.append(f"{'SDK 版本范围':<20} {'对应 GBS 版本':<30}")
    lines.append("-" * 50)
    for sdk_min, sdk_max, gbs_list in VERSION_COMPATIBILITY_MAP:
        gbs_str = ", ".join(gbs_list)
        lines.append(f"{sdk_min} - {sdk_max:<10} {gbs_str:<30}")
    return "\n".join(lines)


def get_robot_version_from_sdk():
    robot = None
    last_error = None
    for module in ("galbot_sdk.g1", "galbot_sdk.s1"):
        try:
            mod = __import__(module, fromlist=["GalbotRobot"])
            robot_cls = getattr(mod, "GalbotRobot")
            robot = robot_cls()
            robot.init()
            # 等待设备信息就绪，最多等待 SDK_CONNECTION_TIMEOUT 秒
            wait_time = 0
            device_info = None
            while wait_time < SDK_CONNECTION_TIMEOUT:
                device_info = robot.get_device_information() or {}
                fw = str(device_info.get("firmware_version", "")).strip()
                if fw:
                    return fw, "sdk:get_device_information.firmware_version"
                time.sleep(0.5)
                wait_time += 0.5
            raise RuntimeError("获取 firmware_version 超时")
        except Exception as exc:  # pylint: disable=broad-except
            last_error = exc
        finally:
            if robot is not None:
                try:
                    robot.request_shutdown()
                    robot.wait_for_shutdown()
                    robot.destroy()
                except Exception:
                    pass
            robot = None
    raise RuntimeError(
        f"无法通过 SDK 获取机器人版本: {last_error}\n\n"
        f"💡 提示：请 SSH 登录机器人后手动查询版本（密码: gb@2023）：\n"
        f"   ssh galbot@<机器人IP>\n"
        f"   cat /data/config/system.cfg | grep CUR_VERSION\n\n"
        f"   获取版本号后运行: galbot_sdk check-version --robot-version <版本号>"
    )


def get_robot_version_from_local_cfg(cfg_path):
    cfg = json.loads(Path(cfg_path).read_text(encoding="utf-8"))
    cur = str(cfg.get("CUR_VERSION", "")).strip()
    if not cur:
        raise RuntimeError("system.cfg 中缺少 CUR_VERSION")
    return cur, f"local_cfg:{cfg_path}"


def get_robot_version_from_ssh_cfg(host, user, port, cfg_path):
    # 应用有线直连默认值，并提示用户当前使用的连接信息
    using_default_user = not user
    user = user or DEFAULT_SSH_USER
    using_default_host = not host

    if using_default_host or using_default_user:
        notes = []
        if using_default_host:
            notes.append(f"host={host} (默认有线直连 Orin)")
        else:
            notes.append(f"host={host}")
        if using_default_user:
            notes.append(f"user={user} (默认)")
        else:
            notes.append(f"user={user}")
        print(f"🔗 SSH 连接: {', '.join(notes)}", file=sys.stderr)
    else:
        print(f"🔗 SSH 连接: host={host}, user={user}", file=sys.stderr)

    # 构建 SSH 命令（带超时，防止网络不通时卡住）
    cmd = [
        "ssh",
        "-o",
        "ConnectTimeout=3",
        "-o",
        "ConnectionAttempts=1",
        "-o",
        "StrictHostKeyChecking=no",
        "-p",
        str(port),
        f"{user}@{host}",
        f"cat '{cfg_path}'",
    ]
    try:
        proc = subprocess.run(
            cmd, check=False, capture_output=True, text=True, timeout=8
        )
    except subprocess.TimeoutExpired:
        raise RuntimeError(
            f"SSH 连接超时（{host}:{port}），请检查网络连接或机器人是否在线"
        )
    if proc.returncode != 0:
        # 如果是认证失败，提示用户输入密码（ssh 会自动提示）
        if "Permission denied" in proc.stderr or "Authentication failed" in proc.stderr:
            raise RuntimeError(
                f"SSH 认证失败，请检查用户名/密码或使用 SSH 密钥认证。\n"
                f"提示：可先手动测试连接: ssh {user}@{host}"
            )
        raise RuntimeError(proc.stderr.strip() or "ssh 执行失败")
    cfg = json.loads(proc.stdout)
    cur = str(cfg.get("CUR_VERSION", "")).strip()
    if not cur:
        raise RuntimeError("远程 system.cfg 中缺少 CUR_VERSION")
    return cur, f"ssh_cfg:{user}@{host}:{port}{cfg_path}"


def resolve_robot_version(args):
    if args.robot_version:
        return args.robot_version.strip(), "manual:--robot-version"

    # 如果指定了 --robot-ip（非默认值），优先使用 SSH 方式（无论 auto 或 ssh_cfg）
    if args.robot_ip != DEFAULT_SSH_HOST:
        return get_robot_version_from_ssh_cfg(
            args.robot_ip, args.user, args.port, args.system_cfg
        )

    if args.source == "sdk":
        return get_robot_version_from_sdk()
    if args.source == "local_cfg":
        return get_robot_version_from_local_cfg(args.system_cfg)
    if args.source == "ssh_cfg":
        return get_robot_version_from_ssh_cfg(
            args.robot_ip, args.user, args.port, args.system_cfg
        )

    # auto: 优先 local_cfg，其次 ssh_cfg（有线直连默认），最后 sdk
    errors = []

    # 优先级1: 本地 system.cfg
    try:
        return get_robot_version_from_local_cfg(args.system_cfg)
    except Exception as exc:
        errors.append(f"本地 system.cfg: {exc}")

    # 优先级2: SSH 远程读取（使用默认或用户指定的 robot_ip/user）
    try:
        return get_robot_version_from_ssh_cfg(
            args.robot_ip, args.user, args.port, args.system_cfg
        )
    except Exception as exc:
        errors.append(f"SSH 远程: {exc}")

    # 优先级3: SDK 连接（最后尝试，可能耗时较长）
    print(
        "🔄 尝试使用 SDK 连接获取...提示：SDK初始化需要一定时间，请耐心等待",
        file=sys.stderr,
    )
    try:
        return get_robot_version_from_sdk()
    except Exception as exc:
        errors.append(f"SDK 连接: {exc}")

    # 所有自动获取方式都失败了，提供详细的错误信息和手动查询指引
    error_details = "\n   ".join(f"{i + 1}. {e}" for i, e in enumerate(errors))
    raise RuntimeError(
        f"无法自动获取机器人版本号，尝试过的方法：\n   {error_details}\n\n"
        f"👉 解决方案（按推荐优先级）：\n"
        f"   1. 【推荐】使用 SSH 方式获取（密码: gb@2023）：\n"
        f"      galbot_sdk check-version\n"
        f"      或指定不同 IP: galbot_sdk check-version --robot-ip <机器人IP>\n"
        f"   2. 手动查询机器人版本：\n"
        f"      SSH 登录机器人后执行: cat /data/config/system.cfg | grep CUR_VERSION\n"
        f"   3. 获取版本号后，使用 --robot-version 参数运行：\n"
        f"      galbot_sdk check-version --robot-version <版本号>\n\n"
        f"💡 SDK 与机器人版本对应关系：\n"
        f"   运行 --list 查看完整对照表"
    )


def detect_mismatch_direction(sdk_version, robot_version, gbs_list):
    # pylint: disable=unused-argument
    # sdk_version 参数保留用于未来扩展，当前逻辑基于 gbs_list 判断
    """
    判断版本不匹配的方向，并返回解决方案。

    返回: (direction, matching_sdks)
    direction:
        "sdk_too_new"   - SDK 版本过高（机器人版本低于 SDK 要求）
                          解决方案: 升级机器人 或 降级 SDK
        "robot_too_new"  - 机器人版本过高（机器人版本高于 SDK 已知的兼容版本，
                          但机器人本身支持更高版本的 SDK）
                          解决方案: 仅升级 SDK
        "unknown"        - 机器人版本完全超出已知范围，无法判断
                          解决方案: 升级 SDK 到最新版本
    matching_sdks: 能匹配当前机器人版本的 SDK 版本范围列表
    """
    robot_ver_num = normalize_version(robot_version)
    robot_major_minor = (
        robot_ver_num[0],
        robot_ver_num[1] if len(robot_ver_num) > 1 else 0,
    )

    # 提取 SDK 要求的 GBS 版本中，最大的主次版本号
    max_gbs_in_sdk = (0, 0)
    for gbs_ver in gbs_list:
        gbs_ver_num = normalize_version(gbs_ver)
        major_minor = (gbs_ver_num[0], gbs_ver_num[1] if len(gbs_ver_num) > 1 else 0)
        if major_minor > max_gbs_in_sdk:
            max_gbs_in_sdk = major_minor

    if robot_major_minor < max_gbs_in_sdk:
        # 机器人版本低于 SDK 要求 → SDK 过高
        # 返回能匹配机器人版本的 SDK 版本范围，用于降级建议
        matching_sdks = get_matching_sdk_versions(robot_version)
        return "sdk_too_new", matching_sdks
    elif robot_major_minor > max_gbs_in_sdk:
        # 机器人版本高于 SDK 已知的兼容版本
        # → 检查机器人版本是否有其他已知 SDK 版本可以匹配
        matching_sdks = get_matching_sdk_versions(robot_version)
        if matching_sdks:
            # 机器人版本有对应的 SDK 版本（只是当前 SDK 版本低了）
            return "robot_too_new", matching_sdks
        else:
            # 机器人版本完全超出已知范围
            return "unknown", None
    else:
        # robot_major_minor == max_gbs_in_sdk，理论上不应走到这里
        # （如果相等 check_version_compatibility 应该已返回 True）
        return "unknown", None


def guide_text(match, cur_version, sdk_version, gbs_list, show_detail=True):
    """
    生成版本兼容性检查的引导文本。

    兼容时: 提示版本匹配，可直接使用。
    不匹配时: 先警告兼容性风险，然后根据不匹配方向给出针对性建议:
      - SDK 过高（机器人版本低）: 升级机器人 或 降级 SDK
      - 机器人过高（SDK 版本低）: 仅升级 SDK（不推荐降级机器人）
      - 未知: 建议升级 SDK 到最新版本
    """
    gbs_str = ", ".join(gbs_list)
    if match:
        guide = "✅ 版本兼容，可直接使用当前组合。"
        if show_detail:
            guide += f"\n   SDK {sdk_version} 与机器人 {cur_version} 匹配（兼容 GBS: {gbs_str}）"
        return guide

    direction, matching_sdks = detect_mismatch_direction(
        sdk_version, cur_version, gbs_list
    )

    guide = "❌ 版本不匹配\n"
    guide += f"   机器人版本: {cur_version}\n"
    guide += f"   SDK {sdk_version} 要求: {gbs_str}\n\n"
    guide += "⚠️  当前 SDK 版本与机器人系统版本不匹配，\n"
    guide += "    可能导致 SDK 部分功能异常或无法使用。\n\n"
    guide += "👉 解决方案（按推荐优先级）：\n"

    if direction == "sdk_too_new":
        # SDK 版本高于机器人，机器人需要追赶
        guide += f"   1. 【推荐】升级机器人系统到 {gbs_list[0]} 或更高\n"
        guide += "      → 获得最新功能、性能优化和安全更新\n"
        if matching_sdks:
            sdk_ranges = ", ".join(
                [f"{min_v}~{max_v}" for min_v, max_v, _ in matching_sdks]
            )
            guide += f"   2. 【备选】降级 SDK 到 {sdk_ranges}\n"
        else:
            guide += f"   2. 【备选】降级 SDK 到与机器人 {cur_version} 兼容的版本\n"
        guide += "      → 仅当需要使用特定机器人版本时使用\n"

    elif direction == "robot_too_new" and matching_sdks:
        # 机器人版本过高，SDK 需要升级
        sdk_ranges = ", ".join(
            [f"{min_v}~{max_v}" for min_v, max_v, _ in matching_sdks]
        )
        guide += (
            f"   1. 【推荐】升级 SDK 到 {sdk_ranges}（与机器人 {cur_version} 匹配）\n"
        )
        guide += "      → 获得最新功能、性能优化和安全更新\n"
        # 不建议降级机器人，因为降级会丢失功能

    else:
        # direction == "unknown" 或其他边界情况
        guide += "   1. 【推荐】升级 SDK 到最新版本\n"
        guide += "      → 获得最新功能、性能优化和安全更新\n"
        if gbs_list:
            guide += f"   2. 【备选】确认机器人系统版本为 {gbs_list[0]}\n"
            guide += "      → 如需使用特定 SDK 版本，请确保机器人版本匹配\n"

    guide += "\n💡 查看完整版本对照表请运行: galbot_sdk check-version --list"
    return guide


def check_version_compatibility(sdk_version, robot_version):
    """
    检查 SDK 版本与机器人版本是否兼容

    每个 SDK 版本范围可对应多个 GBS 版本：
    - SDK 1.5.x ~ 1.7.x  →  [GBS_1.15.x]
    - SDK 1.8.x          →  [GBS_1.16.x]（或 [GBS_1.16.x, GBS_1.17.x]）
    - SDK 1.9.x          →  [GBS_1.17.x]

    返回: (match, gbs_list)
    """
    for sdk_min, sdk_max, gbs_list in VERSION_COMPATIBILITY_MAP:
        if version_in_range(sdk_version, sdk_min, sdk_max):
            # 提取机器人版本的主次版本号
            robot_ver_num = normalize_version(robot_version)
            robot_major_minor = (
                robot_ver_num[0],
                robot_ver_num[1] if len(robot_ver_num) > 1 else 0,
            )

            # 检查机器人版本是否在任意一个兼容的 GBS 版本范围内
            for gbs_ver in gbs_list:
                gbs_ver_num = normalize_version(gbs_ver)
                gbs_major_minor = (
                    gbs_ver_num[0],
                    gbs_ver_num[1] if len(gbs_ver_num) > 1 else 0,
                )
                if robot_major_minor == gbs_major_minor:
                    return True, gbs_list

            # 没有匹配的 GBS 版本
            return False, gbs_list

    raise ValueError(
        f"当前 SDK 版本 {sdk_version} 不在已知兼容性列表中，"
        f"请运行 --list 查看支持的版本范围，或手动指定 --sdk-version"
    )


def main():
    args = parse_args()

    # 显示版本兼容性对照表
    if args.list:
        print(format_version_list())
        sys.exit(EXIT_MATCH)

    try:
        # 智能检测 SDK 版本
        sdk_version, sdk_version_source, sdk_warning = detect_sdk_version(
            args.sdk_version
        )

        # 检查版本兼容性
        cur_version, robot_version_source = resolve_robot_version(args)
        match, gbs_list = check_version_compatibility(sdk_version, cur_version)
        status = "MATCH" if match else "MISMATCH"
        guide = guide_text(match, cur_version, sdk_version, gbs_list, show_detail=True)
        payload = {
            "sdk_version": sdk_version,
            "sdk_version_source": sdk_version_source,
            "required_robot_system_versions": gbs_list,
            "robot_system_version": cur_version,
            "robot_version_source": robot_version_source,
            "status": status,
            "match": match,
            "guide": guide,
            "compatibility_map": VERSION_COMPATIBILITY_MAP,
        }
        if args.json:
            print(json.dumps(payload, ensure_ascii=False, indent=2))
        else:
            # 显示 SDK 版本检测警告（如果有）
            if sdk_warning:
                print(sdk_warning)
                print()

            gbs_display = ", ".join(gbs_list)
            print("=" * 50)
            print("    Galbot SDK 版本兼容性检查")
            print("=" * 50)
            print(f"  SDK 版本:         {sdk_version}")
            print(f"  SDK 版本来源:      {sdk_version_source}")
            print(f"  要求 GBS 版本:    {gbs_display}")
            print(f"  机器人当前版本:   {cur_version}")
            print(f"  版本获取方式:      {robot_version_source}")
            print("-" * 50)
            print(f"  检查结果:        {status}")
            print("-" * 50)
            print(guide)
        sys.exit(EXIT_MATCH if match else EXIT_MISMATCH)
    except Exception as exc:  # pylint: disable=broad-except
        err = {
            "status": "ERROR",
            "error": str(exc),
            "hint": "请检查 SDK 连通性、system.cfg 路径或 SSH 参数。",
        }
        if args.json:
            print(json.dumps(err, ensure_ascii=False, indent=2), file=sys.stderr)
        else:
            print(f"ERROR: {err['error']}", file=sys.stderr)
            print(f"HINT: {err['hint']}", file=sys.stderr)
        sys.exit(EXIT_ERROR)


if __name__ == "__main__":
    main()
