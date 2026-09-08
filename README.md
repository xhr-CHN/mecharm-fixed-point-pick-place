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

项目使用独立镜像 `mecharm-exp2-ros2:humble`。Dockerfile 会安装 ROS 2 Humble、MoveIt 2、Fast DDS 工具和 colcon；不依赖比赛镜像或容器。为规避 Docker BuildKit 对中文上下文路径的会话编码问题，启动脚本只把 `docker/Dockerfile` 和 `docker/entrypoint.sh` 暂存到 `%TEMP%\mecharm-exp2-docker-build` 后构建，源码仍从原实验二目录挂载。

所有 ROS 2 Humble/MoveIt 2 节点都运行在单个 `moveit` 容器中，并使用容器本机 ROS 图。Windows Isaac Sim 不再加入跨系统 DDS；它通过项目自带的 TCP 8765 关节适配层与容器交换 `/joint_states` 和 `/mecharm/joint_target`。WSL 不承载运行时 ROS 图。

首次使用时，需要在管理员 PowerShell 中运行 `scripts/enable_ros2_firewall.ps1`，为 Isaac Sim 开放 TCP 8765。旧的 Domain 44 UDP 防火墙规则会由脚本移除。

## MoveIt 2 仿真（当前主流程）

启动前先关闭代理软件的 TUN 模式。TUN 会拦截 ROS 2 使用的 UDP/DDS 发现流量，典型现象是 Isaac Sim 和 RViz 都能打开，但 RViz 的 Global Status 为 Error、机械臂不显示、任务不执行。若刚关闭 TUN，请先在 Windows PowerShell 执行一次：

```powershell
wsl --shutdown
```

先在 Windows PowerShell 执行：

```powershell
cd "E:\机器人集成小组项目\实验二"
.\scripts\start_moveit_sim.ps1
```

脚本会构建并启动项目自己的 `moveit` 服务和 Isaac Sim，并打印工作区构建命令与完整任务命令。等待 Isaac 场景完全加载且时间轴开始运行后执行打印出的命令。首轮使用 `use_rviz:=false`，直接在 Isaac 中观察完整抓取。

容器首次创建或代码修改后构建：

```powershell
docker compose -f ".\docker-compose.yml" exec moveit bash -lc "source /opt/ros/humble/setup.bash && colcon --log-base /opt/mecharm_ws/log build --base-paths /workspace/mecharm_exp2/simulation/urdf/mycobot_description /workspace/mecharm_exp2/src --build-base /opt/mecharm_ws/build --install-base /opt/mecharm_ws/install --symlink-install"
```

当前推荐的直接完整抓取（不启动 MoveIt/OMPL）：

```powershell
docker compose -f ".\docker-compose.yml" exec moveit bash -lc "source /opt/ros/humble/setup.bash && source /opt/mecharm_ws/install/setup.bash && ros2 launch mecharm_pick_place direct_pick_place.launch.py project_root:=/workspace/mecharm_exp2"
```

该流程复用实验二已经验证的关节路点和平滑插值，直接在 Isaac 中完成抓取。MoveIt 碰撞模型保留为后续校准项。详细步骤见 `docs/testing.md`。

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

## 实验三环形分类仿真

实验三在本地复用本实验的 Elephant Robotics mechArm 270 Pi 和自适应夹爪。机械臂位于桌面中央，网球和铅笔共 6 个物体沿机械臂周围的六边形环形网格分布。仿真分类入口为 `src/mecharm_pick_place/launch/experiment3_sorting.launch.py`，配置为 `config/experiment3_sorting.yaml`。

重建环形场景：

```powershell
& '.\scripts\start_isaac.ps1' -ProjectRoot 'E:\机器人集成小组项目\实验二' -Experiment3 -RebuildScene -BuildSceneOnly
```

ROS 2 启动命令、检测话题、异常场景和验收标准见 `docs/testing.md`。实验三结果只写入 `results/experiment3/`，本项目不执行 GitHub 同步。

## 模型来源

模型取自 Elephant Robotics 官方 `mycobot_ros2` 仓库 Humble 分支：

```text
https://github.com/elephantrobotics/mycobot_ros2
mycobot_description/urdf/mecharm_270_pi
mycobot_description/urdf/adaptive_gripper
```

许可证保存在 `simulation/urdf/mycobot_description/LICENSE`。
