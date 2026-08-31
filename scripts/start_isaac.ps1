param(
    [Parameter(Mandatory = $true)]
    [string]$ProjectRoot,
    [switch]$Headless,
    [switch]$BuildSceneOnly,
    [switch]$RebuildScene,
    [switch]$SmokeTest
)

$ErrorActionPreference = 'Stop'

$isaacRoot = $env:ISAAC_SIM_ROOT
if (-not $isaacRoot) {
    $isaacRoot = 'E:\AIRobotic\isaac-sim'
}

$pythonBat = Join-Path $isaacRoot 'python.bat'
$entrypoint = Join-Path $ProjectRoot 'simulation\isaac\start_simulation.py'
$bridgeLib = Join-Path $isaacRoot 'exts\isaacsim.ros2.bridge\humble\lib'
$rosLogDir = Join-Path $ProjectRoot 'log\ros2'

if (-not (Test-Path -LiteralPath $pythonBat -PathType Leaf)) {
    throw "Isaac Sim Python launcher not found: $pythonBat"
}
if (-not (Test-Path -LiteralPath $entrypoint -PathType Leaf)) {
    throw "Experiment entry point not found: $entrypoint"
}

$env:ISAAC_SIM_ROOT = $isaacRoot
$env:ROS_DISTRO = 'humble'
$env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
$env:PATH = "$bridgeLib;$env:PATH"
$env:ROS_LOG_DIR = $rosLogDir
New-Item -ItemType Directory -Path $rosLogDir -Force | Out-Null

$arguments = @($entrypoint, '--project-root', $ProjectRoot)
if ($Headless) {
    $arguments += '--headless'
}
if ($BuildSceneOnly) {
    $arguments += '--build-scene-only'
}
if ($RebuildScene) {
    $arguments += '--rebuild-scene'
}
if ($SmokeTest) {
    $arguments += '--smoke-test'
}

& $pythonBat @arguments
exit $LASTEXITCODE
