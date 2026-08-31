# 实验二：机械臂定点抓取设计

## 1. 目标与范围

本实验使用 Windows 原生 Isaac Sim 5.1.0 完成 mechArm 270 Pi 与自适应夹爪的定点抓取仿真，并通过 WSL 2 中 Ubuntu 22.04、ROS 2 Humble 运行平台无关的任务逻辑。目标物位置固定，不使用相机、YOLO 或任何视觉定位功能。

本阶段只实现仿真。真机连接、串口、厂家驱动和 Jetson 工作空间在仿真验收通过后另行确认。仿真必须覆盖完整抓取流程、连续五次测试、不可达目标测试、轨迹与结果记录。

## 2. 已确认环境

- 机械臂：Elephant Robotics mechArm 270 Pi。
- 末端执行器：自适应夹爪。
- 仿真器：Windows 原生 Isaac Sim 5.1.0。
- ROS 2：WSL 2 Ubuntu 22.04 中的 ROS 2 Humble。
- 已有经验：先前项目包含 Isaac Sim 内运行的 ROS 2 Bridge 脚本，可参考其扩展启用、DDS 配置和状态发布方式。
- 模型现状：本机和 WSL 尚未发现 mechArm 的 URDF、Xacro 或 USD 文件。

## 3. 方案选择

### 3.1 采用方案

采用“ROS 2 任务状态机 + Isaac Sim 仿真适配层”结构：

1. WSL 中的 ROS 2 节点管理抓取步骤、超时、重试和结果记录。
2. Windows Isaac Sim 内的 Python 适配脚本接收标准 ROS 2 指令，使用 Isaac Sim 5.1.0 的机械臂运动学与 Articulation 控制接口驱动模型。
3. 仿真适配层把关节状态、运动结果、逆解失败、限位错误和夹爪结果发布回 ROS 2。
4. 仿真与未来真机使用相同的高层任务接口；真机阶段只更换设备适配层和参数文件。

### 3.2 未采用方案

- 仅使用预设关节角：实现简单，但无法可靠验证笛卡尔目标不可达和无逆解，不满足异常验收重点。
- 在 WSL 中搭建完整 MoveIt 2 执行链并转发到 Isaac Sim：规划能力完整，但初始集成工作量较大，会同时引入 MoveIt 控制器映射、Action 执行器和跨系统 DDS 三类问题。当前固定点实验优先采用 Isaac 原生运动学；后续若碰撞规划不足，再单独引入 MoveIt 2。
- 完全在 Isaac Sim 内编写任务：可以运行，但任务逻辑难以直接迁移到真机，也不符合任务逻辑与设备适配分离原则。

## 4. 目录设计

所有实验二新增内容均位于 `E:\机器人集成小组项目\实验二`：

```text
实验二/
├── README.md
├── docs/
│   ├── design.md
│   ├── simulation_notes.md
│   └── real_robot_notes.md
├── config/
│   ├── simulation.yaml
│   └── real_robot.yaml
├── src/
│   └── mecharm_pick_place/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/mecharm_pick_place
│       ├── launch/
│       │   ├── simulation.launch.py
│       │   └── real_robot.launch.py
│       ├── mecharm_pick_place/
│       │   ├── __init__.py
│       │   ├── task_node.py
│       │   ├── recorder_node.py
│       │   ├── state_machine.py
│       │   ├── task_config.py
│       │   └── result_types.py
│       └── test/
├── simulation/
│   ├── isaac/
│   │   ├── start_simulation.py
│   │   ├── mecharm_bridge.py
│   │   └── grasp_attachment.py
│   ├── urdf/
│   ├── scenes/
│   └── models/
├── scripts/
│   ├── check_simulation_environment.ps1
│   └── run_simulation.ps1
├── results/
│   ├── simulation/
│   ├── real_robot/
│   ├── logs/
│   ├── trajectories/
│   └── videos/
├── tests/
└── report/
    ├── main.tex
    ├── figures/
    └── build/
```

ROS 2 的 `build/`、`install/`、`log/`，Python 缓存，Isaac Sim 临时缓存和报告构建产物不得提交 Git。根目录 `.gitignore` 本阶段不修改；实验二先使用自身的 `.gitignore`，如以后确需修改根目录规则，必须另行取得确认。

## 5. 模型与场景

模型来源使用 Elephant Robotics 官方 `mycobot_ros2` Humble 分支中的 mechArm 270 Pi 自适应夹爪 URDF。实施时只引入该型号所需的 URDF、网格与许可证说明，不下载或复制与本实验无关的其他机器人资源。

导入流程为：

1. 保存官方来源 URL、分支和取得日期。
2. 使用 Isaac Sim 5.1.0 URDF Importer 导入带自适应夹爪的 mechArm 270 Pi。
3. 固定底座，保留可动关节，不合并需要单独控制的夹爪关节。
4. 检查全部机械臂关节、夹爪关节、关节方向、限位、碰撞体和惯性参数。
5. 将验证后的模型保存为实验二场景引用的 USD。

关节名称、末端链接名称和夹爪关节名称必须从实际导入模型中读取并记录，不在设计阶段凭空指定。

仿真场景包含：

- 固定底座上的 mechArm 270 Pi；
- 自适应夹爪；
- 有碰撞体的桌面；
- 质量不超过 100 g 的标准目标物；
- 固定取物点 A 和固定放置点 B 的可视标记；
- 世界坐标系、机械臂基坐标系和末端工具坐标系。

## 6. ROS 2 包与节点

ROS 2 包名为 `mecharm_pick_place`。

### 6.1 `pick_place_task_node`

职责：加载任务参数，运行抓取状态机，发送目标位姿与夹爪命令，监督超时和错误，并发布任务状态。节点不直接调用 Isaac Sim API，也不包含真机串口代码。

### 6.2 `pick_place_recorder_node`

职责：订阅任务状态、运动结果和关节状态，保存每次尝试的开始时间、结束时间、成功或失败、失败原因、轨迹和参数快照。

### 6.3 `isaac_mecharm_bridge`

该组件运行在 Isaac Sim 进程内，不作为 WSL 中的普通 ROS 2 包入口。职责包括：

- 绑定导入后的机械臂 Articulation；
- 接收目标位姿与夹爪命令；
- 求解逆运动学并拒绝无解目标；
- 检查关节目标是否处于 URDF 限位内；
- 平滑执行关节目标；
- 发布关节状态和执行结果；
- 在夹爪闭合且目标物处于允许距离内时建立仿真附着；
- 在释放阶段解除附着；
- 报告碰撞、超时和夹爪失败。

## 7. ROS 2 接口

优先使用标准 ROS 2 消息，避免为固定点实验引入不必要的自定义接口。

| 名称 | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/mecharm/target_pose` | `geometry_msgs/msg/PoseStamped` | 任务节点 → 适配层 | 发送末端目标位姿 |
| `/mecharm/joint_target` | `sensor_msgs/msg/JointState` | 任务节点 → 适配层 | 发送 home 或已验证安全姿态的关节目标 |
| `/mecharm/gripper_command` | `std_msgs/msg/Float64` | 任务节点 → 适配层 | 发送归一化夹爪开度，0 为闭合、1 为打开 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 适配层 → ROS 2 | 发布当前关节状态 |
| `/mecharm/motion_result` | `std_msgs/msg/String` | 适配层 → 任务节点 | 发布结构化 JSON 运动结果 |
| `/mecharm/task_status` | `std_msgs/msg/String` | 任务节点 → 记录节点 | 发布结构化 JSON 状态 |
| `/mecharm/task_result` | `std_msgs/msg/String` | 任务节点 → 记录节点 | 发布单次任务最终结果 |
| `/mecharm/start_task` | `std_srvs/srv/Trigger` | 外部 → 任务节点 | 启动一次完整抓取 |
| `/mecharm/stop_task` | `std_srvs/srv/Trigger` | 外部 → 任务节点 | 请求安全停止 |

字符串消息中的 JSON 字段固定为 `attempt_id`、`state`、`success`、`error_code`、`message` 和 `timestamp`。实施时为序列化和解析编写单元测试，防止记录节点依赖自由格式文本。

## 8. 抓取状态机

正常路径：

```text
IDLE
→ HOME
→ ABOVE_PICK
→ DESCEND_PICK
→ CLOSE_GRIPPER
→ ATTACH_OBJECT
→ LIFT_PICK
→ ABOVE_PLACE
→ DESCEND_PLACE
→ OPEN_GRIPPER
→ DETACH_OBJECT
→ RETREAT
→ HOME
→ SUCCESS
```

每次状态转换必须等待适配层明确返回成功。不得仅依靠固定 `sleep` 时间推进状态。

异常路径：

```text
任意执行状态
→ ERROR
→ STOP_COMMAND
→ SAFE_POSITION（仅在安全且可规划时）
→ FAILED
```

任务进入 `FAILED` 后不得自动重启；下一次尝试必须由服务调用显式启动。

## 9. 参数分离

`config/simulation.yaml` 保存：

- 坐标系名称；
- home 关节姿态；
- 取物点 A；
- 放置点 B；
- 安全高度；
- 末端朝向；
- 速度和加速度比例；
- 位置、姿态和夹爪容差；
- 规划与执行超时；
- 夹爪开合值；
- 允许附着距离；
- 连续测试次数，固定为 5；
- 最低成功次数，固定为 4。

坐标数值必须在模型导入和场景测量后写入，第一次有效配置需要同时保存场景截图和坐标系说明。未来的 `real_robot.yaml` 使用相同字段结构，但当前仿真阶段不填写未经验证的真机值。

## 10. 异常与安全处理

| 异常 | 检测位置 | 处理 |
|---|---|---|
| 目标不可达或无逆解 | Isaac 运动学求解 | 不发送关节命令，返回 `NO_IK` |
| 关节超限 | 适配层执行前 | 拒绝执行，返回 `JOINT_LIMIT` |
| 规划或运动失败 | 适配层 | 停止当前命令，返回 `MOTION_FAILED` |
| 控制超时 | 任务节点 | 请求停止，返回 `TIMEOUT` |
| 夹爪控制失败 | 适配层 | 不附着目标物，返回 `GRIPPER_FAILED` |
| 目标物距离过远 | 附着组件 | 拒绝附着，返回 `OBJECT_OUT_OF_RANGE` |
| 明显碰撞 | PhysX 接触监测 | 停止任务，返回 `COLLISION` |
| ROS 2 通信中断 | 两端心跳超时 | 停止关节目标更新，任务失败 |

仿真停止必须是保持当前位置或进入经验证的安全姿态，不得在错误发生后盲目回零。

## 11. 启动方式

`src/mecharm_pick_place/launch/simulation.launch.py` 是 ROS 2 的统一入口，由 `setup.py` 安装到包的 share 目录。它负责启动任务节点、记录节点和环境检查，并通过 Windows 互操作调用实验二的 PowerShell 启动脚本。PowerShell 脚本使用配置的 Isaac Sim 安装路径启动 `simulation/isaac/start_simulation.py`。

Isaac Sim 安装目录不得写死在源代码中，使用环境变量 `ISAAC_SIM_ROOT` 或启动参数提供。启动流程必须等待以下条件全部成立后才允许开始任务：

1. Isaac Sim 场景加载完成；
2. 机械臂 Articulation 和夹爪绑定成功；
3. ROS 2 Bridge 已连接；
4. `/joint_states` 持续发布；
5. 任务节点和记录节点就绪。

## 12. 测试与验收

### 12.1 自动化测试

- 配置字段、数值范围和坐标系校验；
- 状态机正常转换；
- 失败后不继续执行后续抓取步骤；
- 超时进入安全停止；
- JSON 状态和结果格式稳定；
- 五次测试汇总及 80% 成功率计算；
- 非法关节目标和不可达目标的错误映射。

### 12.2 仿真测试顺序

1. 验证模型导入、关节方向和夹爪开合。
2. 验证 home 姿态。
3. 分别验证 A 上方、A、B 上方和 B。
4. 验证无目标物的完整空跑。
5. 验证夹取、附着、移动和释放。
6. 执行一次完整正常任务。
7. 连续执行五次并要求至少四次成功。
8. 将目标设到工作空间外，确认返回 `NO_IK` 且机械臂安全停止。
9. 检查碰撞、关节限位、轨迹、日志和参数快照。

### 12.3 结果文件

每组验收生成独立时间戳目录，至少包含：

- `summary.csv`：五次结果及成功率；
- `events.jsonl`：逐状态事件；
- `trajectory.csv`：带时间戳的关节轨迹；
- `parameters.yaml`：本次参数快照；
- `errors.log`：异常信息；
- 演示视频或其文件说明。

## 13. 实验一复用边界

可参考实验一的 ROS 2 Python 包结构、Launch 组织、参数文件、日志格式、分阶段提交和 LaTeX 报告模板。报告模板必须复制到实验二后再编辑。

不得复制或计入实验二成果的内容包括：数据集、YOLO 训练与推理代码、模型文件、检测消息、摄像头节点、识别图片、训练曲线、识别视频和目标检测结果。

## 14. Git 与操作边界

- 当前 Git 根目录保持为 `E:\机器人集成小组项目`，不在实验二内部创建嵌套 `.git`。
- 所有新增和修改仅发生在 `实验二/`。
- 不修改根目录 README、`.gitignore` 或 `.gitattributes`，除非另行取得确认。
- 不改写历史，不执行破坏性清理。
- 提交和推送均需用户明确确认。
- GitHub 仓库命名候选为 `mecharm-fixed-point-pick-place`；远程仓库的创建和关联不属于本设计文档写入步骤。

## 15. 完成定义

仿真阶段完成需要同时满足：

1. 一个 ROS 2 Launch 入口可以启动完整仿真链路；
2. 固定点抓取流程完整执行；
3. 五次连续抓取至少成功四次；
4. 不发生明显碰撞和关节超限；
5. 不可达目标会停止并输出明确错误；
6. 轨迹、五次结果、参数快照和异常日志均可追溯；
7. README、仿真说明和演示材料齐全；
8. 未引入任何视觉定位功能或实验一检测成果。
