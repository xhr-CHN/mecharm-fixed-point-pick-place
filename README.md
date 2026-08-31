# 实验二：机械臂定点抓取

本目录用于 mechArm 270 Pi 自适应夹爪的固定点抓取实验。仿真运行在 Windows Isaac Sim 5.1.0，任务节点运行在 WSL 2 Ubuntu 22.04 / ROS 2 Humble。

当前已建立：

- 官方 mechArm 270 Pi 自适应夹爪 URDF 与网格；
- `mecharm_pick_place` ROS 2 Python 包；
- 固定点抓取状态机；
- 任务状态、五次结果和关节轨迹记录；
- Isaac Sim 5.1 启动和 ROS 2 适配脚本；
- 单元测试和 ROS 2 Launch 入口。

## 目录

```text
config/                 仿真参数
docs/                   设计与实验记录
scripts/                Windows/WSL 启动与环境检查
simulation/             Isaac Sim 脚本、场景和官方模型
src/mecharm_pick_place/  ROS 2 功能包
tests/                  不依赖 ROS 的单元测试
results/                轨迹、结果、日志和视频
report/                 实验报告
```

## 初次构建

在 WSL 中执行：

```bash
cd /mnt/e/机器人集成小组项目/实验二
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

模型导入与场景标定完成后，从 Windows PowerShell 运行：

```powershell
$env:ISAAC_SIM_ROOT = "C:\isaacsim"
.\scripts\run_simulation.ps1
```

启动一次五轮抓取：

```bash
ros2 service call /mecharm/start_task std_srvs/srv/Trigger {}
```

仿真坐标当前是安全的初始建议值，必须在 Isaac Sim 中核对桌面高度、工具坐标系与可达性后再用于验收。

## 模型来源

模型取自 Elephant Robotics 官方 `mycobot_ros2` 仓库 Humble 分支：

```text
https://github.com/elephantrobotics/mycobot_ros2
mycobot_description/urdf/mecharm_270_pi
mycobot_description/urdf/adaptive_gripper
```

许可证保存在 `simulation/urdf/mycobot_description/LICENSE`。

