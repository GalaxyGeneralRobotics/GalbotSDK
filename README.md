# Galbot SDK

![Version](https://img.shields.io/badge/version-1.10.0-blue.svg)
![Robot Model](https://img.shields.io/badge/Robot-G1%2FS1%2FG3-red.svg)
![GBS Version](https://img.shields.io/badge/GBS-1.18-green.svg)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20--24-orange.svg)
![Python](https://img.shields.io/badge/Python-3.8--3.14-yellow.svg)

**[中文](#中文文档) | [English](#english-docs)**

---

## 中文文档

欢迎使用 Galbot SDK（Galbot机器人软件开发套件）！

本 SDK 支持 C++ 和 Python，提供完整的 API 参考、使用教程和示例代码。

### 源码下载

```bash
git clone https://github.com/GalaxyGeneralRobotics/GalbotSDK.git
```

### Jetson MMAPI 安装

Orin 和 Thor 的 MMAPI 以 `deps/jetson-mmapi-orin.tar.gz` 和
`deps/jetson-mmapi-thor.tar.gz` 交付。继续使用原来的 `install.sh` 安装命令即可；
离线包使用 `install_offline.sh`。安装脚本自动检查并解压对应平台依赖到
`<安装目录>/deps/`（默认 `/opt/galbot/deps/`），保留库的软链接。
无需手动解压内部依赖，也无需 Git LFS；下载目录不会被展开的大文件修改。
编译示例前请先完成安装，使用安装后的 SDK 目录。旧版未压缩的 MMAPI 目录仍兼容。

### 📌 版本匹配

**重要**：安装前请确认您的机器人版本！

#### 当前版本

- **最新 SDK**: V1.10.0
- **发布日期**: 2026-09-22
- **对应机器人环境版本**: V1.18
- **维护状态**: ✅ 当前维护版本

更多版本历史请参阅 [CHANGELOG.md](CHANGELOG.md)

### 📚 查看完整文档

#### 🌐 方法一：启动本地文档服务器（推荐）

在 SDK 根目录执行：

```bash
cd docs
python3 -m http.server 8000
```

然后在浏览器中打开：**http://localhost:8000/{model}/zh/**（`{model}` 为您的机器人型号，例如 G1 对应 **http://localhost:8000/g1/zh/**）

#### 📄 方法二：直接打开文件

用浏览器打开文件：`docs/{model}/zh/index.html`（`{model}` 为您的机器人型号，例如 `docs/g1/zh/index.html`）



---

## English Docs

Welcome to Galbot SDK (Galbot Robot Software Development Kit)!

This SDK supports C++ and Python, providing complete API reference, tutorials, and example code.

### Source Code Download

```bash
git clone https://github.com/GalaxyGeneralRobotics/GalbotSDK.git
```
### Jetson MMAPI installation

Orin and Thor MMAPI dependencies are shipped as `deps/jetson-mmapi-orin.tar.gz`
and `deps/jetson-mmapi-thor.tar.gz`. Use the existing `install.sh` command, or
`install_offline.sh` for offline packages. The installer checks and extracts the
selected dependencies into `<install-dir>/deps/` (default: `/opt/galbot/deps/`),
preserving library symlinks without changing the downloaded package directory.
No manual dependency extraction or Git LFS is needed. Install the SDK before
building examples and use the installed SDK directory. Older packages containing
uncompressed MMAPI directories remain supported.

### 📌 Version Compatibility

**Important**: Please confirm your robot version before installation!

#### Current Version

- **Latest SDK**: V1.10.0
- **Release Date**: 2026-09-22
- **Compatible Robot Version**: V1.18
- **Maintenance Status**: ✅ Currently Maintained

For more version history, please refer to [CHANGELOG.md](CHANGELOG.md)

### 📚 View Complete Documentation

#### 🌐 Method 1: Start Local Documentation Server (Recommended)

Execute in the SDK root directory:

```bash
cd docs
python3 -m http.server 8000
```

Then open in your browser: **http://localhost:8000/{model}/en/** (`{model}` is your robot model, e.g. for G1: **http://localhost:8000/g1/en/**)

#### 📄 Method 2: Open File Directly

Open the file in your browser: `docs/{model}/en/index.html` (`{model}` is your robot model, e.g. `docs/g1/en/index.html`)
