param(
    [string]$ProjectRoot = ''
)

$ErrorActionPreference = 'Stop'

if (-not $ProjectRoot) {
    $ProjectRoot = Split-Path -Parent $PSScriptRoot
}

$isaacLauncher = Join-Path $ProjectRoot 'scripts\start_isaac.ps1'
if (-not (Test-Path -LiteralPath $isaacLauncher -PathType Leaf)) {
    throw "Isaac launcher not found: $isaacLauncher"
}
if (-not (Test-Path -LiteralPath 'E:\AIRobotic\isaac-sim\python.bat' -PathType Leaf)) {
    throw 'Isaac Sim 5.1 Python launcher not found at E:\AIRobotic\isaac-sim\python.bat'
}

$env:ROS_DOMAIN_ID = '0'
$env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
$env:ROS_LOCALHOST_ONLY = '0'

Start-Process powershell.exe -WindowStyle Normal -ArgumentList @(
    '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $isaacLauncher,
    '-ProjectRoot', $ProjectRoot
)

[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$driveLetter = $ProjectRoot.Substring(0, 1).ToLower()
$relativePath = $ProjectRoot.Substring(3).Replace('\', '/')
$linuxRoot = "/mnt/$driveLetter/$relativePath"
$command = "cd '$linuxRoot' && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 && ros2 launch mecharm_moveit_config simulation_moveit.launch.py use_rviz:=true run_task:=true"

Write-Host ''
Write-Host 'Isaac Sim is starting. Wait until the scene is fully loaded and running, then run this in WSL:'
Write-Host ''
Write-Host $command
Write-Host ''
