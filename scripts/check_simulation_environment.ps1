$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$required = @(
    (Join-Path $projectRoot 'config\simulation.yaml'),
    (Join-Path $projectRoot 'simulation\isaac\start_simulation.py'),
    (Join-Path $projectRoot 'simulation\urdf\mycobot_description\urdf\mecharm_270_pi\mecharm_270_pi_adaptive_gripper.urdf')
)

foreach ($path in $required) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Required experiment file is missing: $path"
    }
}

$isaacRoot = $env:ISAAC_SIM_ROOT
if (-not $isaacRoot) {
    $isaacRoot = 'E:\AIRobotic\isaac-sim'
}
if (-not (Test-Path -LiteralPath (Join-Path $isaacRoot 'python.bat'))) {
    throw "Isaac Sim Python launcher not found: $isaacRoot\python.bat"
}
$isaacVersion = Get-Content -LiteralPath (Join-Path $isaacRoot 'VERSION') -ErrorAction Stop
if (-not $isaacVersion.StartsWith('5.1.0')) {
    throw "Expected Isaac Sim 5.1.0, found: $isaacVersion"
}

wsl.exe -d Ubuntu-22.04 -u xhr -- bash -lc 'source /opt/ros/humble/setup.bash && test "$(printenv ROS_DISTRO)" = humble && printenv ROS_DISTRO'
if ($LASTEXITCODE -ne 0) {
    throw 'ROS 2 Humble check failed in WSL.'
}

Write-Host "Isaac Sim $isaacVersion found at $isaacRoot"
Write-Host 'Simulation environment files and WSL ROS 2 check passed.'
