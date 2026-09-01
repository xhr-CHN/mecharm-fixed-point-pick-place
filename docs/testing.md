# 仿真联调与验收

## 启动

Windows PowerShell：

```powershell
cd "E:\机器人集成小组项目\实验二"
# 首次使用时，在“以管理员身份运行”的 PowerShell 中执行一次：
.\scripts\enable_ros2_firewall.ps1
.\scripts\start_moveit_sim.ps1
```

WSL：

```bash
cd /mnt/e/机器人集成小组项目/实验二
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
ros2 launch mecharm_moveit_config simulation_moveit.launch.py use_rviz:=true run_task:=false
```

## 运动前检查

```bash
ros2 topic echo /joint_states --once
ros2 action list | grep '^/mecharm_controller/follow_joint_trajectory$'
```

必须同时看到六个机械臂关节和 `gripper_controller`，并能看到上述 action。检查通过后停止 launch，再执行完整任务：

```bash
ros2 launch mecharm_moveit_config simulation_moveit.launch.py use_rviz:=true run_task:=true
```

若规划失败、关节状态超过 0.5 秒未更新或最终误差超过 0.02 rad，控制器会中止任务并保持当前实测位置。

## 验收

场景每次恢复初始状态后运行一次，共五次。至少四次完成抓取、搬运、释放和回零；全程不得触碰桌面、越过关节限位、明显振荡或让方块吸附时跳变。
