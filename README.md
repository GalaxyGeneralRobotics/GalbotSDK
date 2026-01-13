# SDK README

## 源码下载

```bash
git clone https://github.com/GalaxyGeneralRobotics/GalbotSDK.git
cd GalbotSDK
git lfs pull
```
请确保安装了git-lfs, 如Ubuntu/Debian下可用如下命令安装:
```bash
sudo apt-get install git-lfs
```

## 版本匹配

**重要**：安装前请确认您的机器人版本！！！

### 当前版本
- **最新SDK**: V1.5.0
- **发布日期**: 2026-01-15
- **对应机器人版本**: V1.13.9
- **版本对应表**: 参照 [版本说明](VERSION.md)

## 安装说明

**安装SDK到本地**
```bash
./install.sh
```
根据提示选择安装目录，默认的路径为/opt/galbot  
如果需要安装到机器人，首先ssh登录到机器人上

**远程部署SDK库到机器人**
```bash
./deploy_to_robot.sh
```
根据提示输入机器人IP和用户，并用密码验证传输

## SDK 文档
文档查看方式：
```bash
cd docs
python3 -m http.server 8000
```
打开http://localhost:8000/ 在本地浏览器中访问文档