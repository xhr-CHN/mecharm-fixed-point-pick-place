# 仿真联调与验收

## 启动

当前流程使用实验二专用 Docker 镜像和 Compose 项目，不依赖比赛镜像或容器。所有 ROS 2 与 MoveIt 节点都在 `moveit` 服务内部通信；Windows Isaac 通过 TCP 8765 关节适配层发送实测状态并接收目标，不再使用跨系统 DDS。

Docker 镜像使用 `%TEMP%\mecharm-exp2-docker-build` 作为纯英文构建上下文，避免中文项目路径触发 BuildKit `x-docker-expose-session-sharedkey` 编码错误。该目录只包含 Dockerfile 和入口脚本。

Windows PowerShell：

```powershell
cd "E:\机器人集成小组项目\实验二"
# 首次使用时，在“以管理员身份运行”的 PowerShell 中执行一次：
.\scripts\enable_ros2_firewall.ps1
.\scripts\start_moveit_sim.ps1
```

首次使用时，以管理员身份运行更新后的防火墙脚本，开放 Isaac `kit.exe` 的 TCP 8765。Isaac 控制台应显示 `Isaac TCP joint bridge listening on 0.0.0.0:8765`；启动 MoveIt 后应显示 `Isaac TCP joint bridge connected`。

首次创建或代码修改后，在 PowerShell 构建容器工作区：

```powershell
docker compose -f ".\docker-compose.yml" exec moveit bash -lc "source /opt/ros/humble/setup.bash && colcon --log-base /opt/mecharm_ws/log build --base-paths /workspace/mecharm_exp2/simulation/urdf/mycobot_description /workspace/mecharm_exp2/src --build-base /opt/mecharm_ws/build --install-base /opt/mecharm_ws/install --symlink-install"
```

等待 Isaac 场景加载并播放后，在 PowerShell 启动直接完整抓取：

```powershell
docker compose -f ".\docker-compose.yml" exec moveit bash -lc "source /opt/ros/humble/setup.bash && source /opt/mecharm_ws/install/setup.bash && ros2 launch mecharm_pick_place direct_pick_place.launch.py project_root:=/workspace/mecharm_exp2"
```

预期：终端出现 `DIRECT_PICK_PLACE_START`，随后依次经过抓取、抬升、放置和回零阶段，最终出现 `DIRECT_PICK_PLACE_SUCCESS`。该流程不启动 MoveIt、OMPL 或 RViz。

## 运动前检查

```powershell
docker compose exec moveit bash -lc "source /opt/ros/humble/setup.bash && ros2 topic echo /joint_states --once"
docker compose exec moveit bash -lc "source /opt/ros/humble/setup.bash && ros2 action list | grep '^/mecharm_controller/follow_joint_trajectory$'"
```

必须同时看到六个机械臂关节和 `gripper_controller`，并能看到上述 action。检查通过后停止 launch，再执行完整任务：

```powershell
docker compose exec moveit bash -lc "source /opt/ros/humble/setup.bash && source /opt/mecharm_ws/install/setup.bash && cd /workspace/mecharm_exp2 && ros2 launch mecharm_moveit_config simulation_moveit.launch.py project_root:=/workspace/mecharm_exp2 use_rviz:=false run_task:=true"
```

完整任务成功时，日志依次出现 `GROUP_READY`、`STATE_READY`、`SCENE_READY`、`HOME_START`、`HOME_DONE`，最终出现 `PICK_PLACE_SUCCESS`。若只停在 `You can start planning now!` 且没有 `GROUP_READY`，说明任务节点未被 DDS 发现；优先检查 TUN 是否关闭和三个环境变量是否已清除。

`run_task:=true` 时，任务节点会延迟 12 秒启动，给 `move_group` 和控制器留出初始化时间。先出现 `You can start planning now!`、随后出现 `TASK_NODE_STARTING` 属于正常顺序。RViz 报 `/recognize_objects not available` 可以忽略，该实验未使用物体识别 action。

自适应夹爪左右两侧的 `gripper_left1/left2` 和 `gripper_right1/right2` 是正常机构重叠对，已仅在 MoveIt SRDF 中禁用自碰撞检查；Isaac 的真实物理碰撞保持不变。

当前“先动起来”阶段设置 `disable_collision_checking: true`：MoveIt 不加载桌面/方块碰撞体，并允许机器人内部碰撞。关节限位、速度、加速度、轨迹执行和 Isaac 真实物理仍启用。模型碰撞几何校准后将该参数改回 `false`。

官方 adaptive-gripper DAE 只有铰接孔，没有独立插销网格。场景构建时会在四个外露铰点生成无碰撞金属圆柱，并验证对应四个 PhysX RevoluteJoint 的 body0/body1 连接。插销只补视觉，转动约束由 RevoluteJoint 提供，避免实体插销与连杆碰撞后卡死。

官方 URDF 因树结构限制省略了 `gripper_left2↔gripper_left1` 和 `gripper_right2↔gripper_right1` 两个闭环销轴。Isaac 场景构建会将外侧夹指关节改为被动关节，并在 DAE 实测孔位处创建左右两个 PhysX loop RevoluteJoint，使内侧连杆通过闭环真实带动外侧夹指。

构建时会显式清除 URDF 导入器留在外侧被动关节上的默认驱动属性（`drive:angular:physics:*`）。这类属性不是以 DriveAPI 形式写入，`RemoveAPI` 不会删除；若残留，外侧夹指会被弹簧拉回零位并与闭环约束顶牛，表现为一侧夹指松软、无法出力。

PhysX 5.1 对“同一个 reference joint 的多个 mimic follower”支持不可靠：`left2/right2/right3` 若都 mimic `gripper_controller`，右侧整侧会被静默忽略、收不到开合指令。因此场景不再导入 URDF mimic，三根内连杆（`left2/right2/right3`）都改为显式 angular drive；TCP 桥收到 `gripper_controller` 指令时在软件层按 `left2=+master`、`right2=right3=-master` 同步镜像，外侧夹指仍由闭环真实带动。

两个 loop RevoluteJoint 设置 `excludeFromArticulation=true`，作为树形 articulation 之外的闭环约束参与 PhysX 求解；articulation 的位置和速度迭代数均设为 64，减少受力时销孔分离和外侧夹指松动。

靠近夹指端的两个可见插销挂在 `gripper_left1/right1` 外侧夹指上，位置使用 loop joint 的外侧局部锚点；这样不会被内侧 `left2/right2` 连杆遮挡，并与夹指一起运动。

Isaac 使用 `MECHARM_SELF_COLLISION=selective`：开启 articulation self-collision，但过滤机械臂相邻连杆、机械臂与夹爪官方重叠网格、以及除最终左右夹指以外的夹爪内部机构对。`gripper_left1` 与 `gripper_right1` 的碰撞保持启用，使最终夹指可以物理接触。

抓取顺序固定为：初始慢速打开夹爪，回到初始位，移动到物体正上方预抓取高度（`z=0.125 m`），下降到合爪高度（`z=0.065 m`，高于物体顶面 `0.050 m`），在不再继续下降的情况下慢速合拢，随后抬升。放置侧同样下降到 `z=0.065 m`（与合爪高度一致，此时被夹物体已回到桌面高度）后再打开释放。夹爪速度为 `0.12 rad/s`，每次开合前等待 `1.0 s`、开合后等待 `1.5 s`。物理附着距离为 `0.08 m`，且只有在夹爪合拢后才会吸附。

若规划失败、关节状态超过 0.5 秒未更新或最终误差超过 0.02 rad，控制器会中止任务并保持当前实测位置。

## 验收

场景每次恢复初始状态后运行一次，共五次。至少四次完成抓取、搬运、释放和回零；全程不得触碰桌面、越过关节限位、明显振荡或让方块吸附时跳变。
