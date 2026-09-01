param(
    [string]$ProjectRoot = 'E:\机器人集成小组项目\实验二'
)

$ErrorActionPreference = 'Stop'
$isaacLauncher = Join-Path $ProjectRoot 'scripts\start_isaac.ps1'
if (-not (Test-Path -LiteralPath $isaacLauncher -PathType Leaf)) {
    throw "Isaac launcher not found: $isaacLauncher"
}
if (-not (Test-Path -LiteralPath 'E:\AIRobotic\isaac-sim\python.bat' -PathType Leaf)) {
    throw 'Isaac Sim 5.1 Python launcher not found at E:\AIRobotic\isaac-sim\python.bat'
}

$env:ROS_DOMAIN_ID = '0'
$env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
Start-Process powershell.exe -WindowStyle Normal -ArgumentList @(
    '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $isaacLauncher,
    '-ProjectRoot', $ProjectRoot
)

$linuxRoot = '/mnt/e/机器人集成小组项目/实验二'
$command = "cd '$linuxRoot' && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 && ros2 launch mecharm_moveit_config simulation_moveit.launch.py use_rviz:=true run_task:=false"
Write-Host 'Isaac Sim is starting. Run this in WSL after the scene opens:'
Write-Host $command
