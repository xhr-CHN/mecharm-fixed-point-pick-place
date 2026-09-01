# 实验二：机械臂定点抓取

本目录用于 mechArm 270 Pi 自适应夹爪的固定点抓取实验。仿真运行在 Windows Isaac Sim 5.1.0，任务节点运行在 WSL 2 Ubuntu 22.04 / ROS 2 Humble。

当前已建立：

- 官方 mechArm 270 Pi 自适应夹爪 URDF 与网格；
- `mecharm_pick_place` ROS 2 Python 包；
- 固定点抓取状态机；
- 任务状态、五次结果和关节轨迹记录；
- Isaac Sim 5.1 启动和 ROS 2 适配脚本；
- Isaac Sim 内置的 120 Hz 平滑定点抓取演示；
- 自适应夹爪 Mimic 联动、有限角度和抓取连接修复；
- 单元测试和 ROS 2 Launch 入口。
- MoveIt 2 碰撞规划和标准 `FollowJointTrajectory` 执行链路。

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
sudo apt-get update
sudo apt-get install -y ros-humble-moveit ros-humble-moveit-configs-utils ros-humble-control-msgs ros-humble-trajectory-msgs ros-humble-xacro
cd /mnt/e/机器人集成小组项目/实验二
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

本项目固定使用 WSL `Ubuntu-22.04`。Windows 和 WSL 两侧均使用 `ROS_DOMAIN_ID=0`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`ROS_LOCALHOST_ONLY=0`。

## MoveIt 2 仿真（当前主流程）

先在 Windows PowerShell 执行：

```powershell
cd "E:\机器人集成小组项目\实验二"
.\scripts\start_moveit_sim.ps1
```

Isaac 场景打开后，在 WSL 执行：

```bash
cd /mnt/e/机器人集成小组项目/实验二
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
ros2 launch mecharm_moveit_config simulation_moveit.launch.py use_rviz:=true run_task:=false
```

确认 `/joint_states` 和 `/mecharm_controller/follow_joint_trajectory` 后，将 `run_task` 改为 `true` 启动一次完整抓取。详细步骤见 `docs/testing.md`。

模型导入与场景标定完成后，从 Windows PowerShell 运行（脚本默认使用本机已验证的 `E:\AIRobotic\isaac-sim`）：

```powershell
.\scripts\run_simulation.ps1
```

如需使用其他 Isaac Sim 安装目录，可先设置：

```powershell
$env:ISAAC_SIM_ROOT = "D:\path\to\isaac-sim"
```

启动一次五轮抓取：

```bash
ros2 service call /mecharm/start_task std_srvs/srv/Trigger {}
```

仿真坐标当前是安全的初始建议值，必须在 Isaac Sim 中核对桌面高度、工具坐标系与可达性后再用于验收。

## Isaac Sim 场景与旧版内置演示

首次生成或模型修改后，在 Windows PowerShell 中执行：

```powershell
& ".\scripts\start_isaac.ps1" `
  -ProjectRoot "E:\机器人集成小组项目\实验二" `
  -RebuildScene `
  -BuildSceneOnly
```

随后打开 `simulation/scenes/mecharm_pick_place.usd`，在 Isaac Sim 的 Script Editor 中执行：

```python
exec(open(r"E:\机器人集成小组项目\实验二\simulation\isaac\auto_demo_in_app.py", encoding="utf-8").read())
```

该脚本只保留作模型调试，不再作为实验主流程。主流程由 MoveIt 2 规划完整轨迹，通过 ROS 2 action 逐点执行。

## 模型来源

模型取自 Elephant Robotics 官方 `mycobot_ros2` 仓库 Humble 分支：

```text
https://github.com/elephantrobotics/mycobot_ros2
mycobot_description/urdf/mecharm_270_pi
mycobot_description/urdf/adaptive_gripper
```

许可证保存在 `simulation/urdf/mycobot_description/LICENSE`。
