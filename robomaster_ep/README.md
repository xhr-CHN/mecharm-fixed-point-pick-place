# RoboMaster EP 控制脚本

本目录保存 RoboMaster EP 的电脑端 Python 控制脚本。它与 `实验二` 原有的 Isaac Sim/ROS 2 仿真代码分开，直接通过机器人 Wi-Fi 使用 RoboMaster SDK 控制真实 EP。

## 已验证环境

- Windows PowerShell
- Conda 环境：`robomaster`
- Python 3.8
- RoboMaster SDK：`0.1.1.68`
- RoboMaster EP 固件版本：`01.01.1150`
- 连接方式：电脑直连机器人 Wi-Fi，`conn_type="ap"`
- 通信方式：`proto_type="tcp"`
- SDK 本地构建目录：`C:\Users\xhr\RoboMaster-SDK-fixed`

SDK 构建目录没有复制进 GitHub，因为它包含约 202 MB 的编译产物、DLL、安装包和缓存。重新部署时应从官方 SDK 源码构建，并准备 CMake 3.31.x 和 Visual Studio C++ Build Tools。

## 脚本

- `test_ep_connection.py`：测试 SDK 连接和机器人版本
- `test_ep_arm.py`：测试机械臂小幅运动
- `test_ep_gripper.py`：测试夹爪开合和状态订阅
- `test_chassis_rotate.py`：测试麦轮原地旋转
- `test_camera.py`：测试摄像头图传
- `simple_grab.py`：订阅夹爪状态，确认连续收到 `closed` 后返回初始姿态
- `simple_grab_time_check.py`：以闭合耗时辅助判断夹爪状态
- `pick_rotate_place.py`：慢速抓取、旋转、放置流程
- `pick_rotate_place_5_cycles.py`：连续 5 次抓取、搬运和放置
- `pick_rotate_place_5_cycles_retry.py`：连续 5 次成功抓取、失败重试、50 Hz 夹爪订阅、持续摄像头图传和 `E` 键安全退出

## 运行方式

先连接机器人 Wi-Fi，再激活环境：

```powershell
conda activate robomaster
$env:RM_SDK_ROOT = "C:\Users\xhr\RoboMaster-SDK-fixed"
```

Python 3.8 在 Windows 下需要显式加入 SDK 的 FFmpeg 和 Opus DLL 目录。以摄像头测试为例：

```powershell
python -c "import os; root=os.environ['RM_SDK_ROOT']; keep=[os.add_dll_directory(os.path.join(root,'lib','libmedia_codec','src','ffmpeg-dll')), os.add_dll_directory(os.path.join(root,'lib','libmedia_codec','src','opus-dll'))]; exec(open(r'E:\机器人集成小组项目\实验二\robomaster_ep\scripts\test_camera.py',encoding='utf-8').read())"
```

把命令中的 `test_camera.py` 换成其他脚本即可。

## 安全注意事项

- 第一次运行运动脚本时，机械臂和底盘周围必须留出空间。
- 机械臂的 `y=0` 是 SDK 坐标中的低位边界，实际使用前应先在 RoboMaster App 中完成机械臂校准。
- 夹爪的 `closed` 只表示夹爪闭合状态，不等于确认夹住了物体；需要摄像头或额外力/压力传感器做抓取确认。
- 运行电脑端 Python 时，不要同时让其他设备控制同一台机器人。
