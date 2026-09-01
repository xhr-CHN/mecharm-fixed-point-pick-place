param(
    [string]$ProjectRoot = ''
)

$ErrorActionPreference = 'Stop'

if (-not $ProjectRoot) {
    $ProjectRoot = Split-Path -Parent $PSScriptRoot
}

$isaacLauncher = Join-Path $ProjectRoot 'scripts\start_isaac.ps1'
$composeFile = Join-Path $ProjectRoot 'docker-compose.yml'
if (-not (Test-Path -LiteralPath $isaacLauncher -PathType Leaf)) {
    throw "Isaac launcher not found: $isaacLauncher"
}
if (-not (Test-Path -LiteralPath $composeFile -PathType Leaf)) {
    throw "Docker Compose file not found: $composeFile"
}
if (-not (Test-Path -LiteralPath 'E:\AIRobotic\isaac-sim\python.bat' -PathType Leaf)) {
    throw 'Isaac Sim 5.1 Python launcher not found at E:\AIRobotic\isaac-sim\python.bat'
}

function Test-DockerReady {
    cmd.exe /d /c 'docker info --format "{{.ServerVersion}}" >nul 2>nul'
    return $LASTEXITCODE -eq 0
}

if (-not (Test-DockerReady)) {
    $dockerDesktop = 'C:\Program Files\Docker\Docker\Docker Desktop.exe'
    if (-not (Test-Path -LiteralPath $dockerDesktop -PathType Leaf)) {
        throw 'Docker Desktop is not running and its launcher was not found.'
    }
    Write-Host 'Starting Docker Desktop...'
    Start-Process -FilePath $dockerDesktop -WindowStyle Hidden
    for ($attempt = 0; $attempt -lt 30 -and -not (Test-DockerReady); $attempt++) {
        Start-Sleep -Seconds 2
    }
}
if (-not (Test-DockerReady)) {
    throw 'Docker Desktop did not become ready within 60 seconds.'
}

$env:DOCKER_CLIENT_TIMEOUT = '600'
$env:COMPOSE_HTTP_TIMEOUT = '600'
$baseImage = 'osrf/ros:humble-desktop-full'
cmd.exe /d /c "docker image inspect $baseImage >nul 2>nul"
if ($LASTEXITCODE -ne 0) {
    for ($pullAttempt = 1; $pullAttempt -le 5; $pullAttempt++) {
        Write-Host "Pulling official ROS base image (attempt $pullAttempt of 5)..."
        docker pull $baseImage
        if ($LASTEXITCODE -eq 0) {
            break
        }
        if ($pullAttempt -lt 5) {
            Write-Host 'Docker Hub authentication timed out; downloaded layers are preserved. Retrying...'
            Start-Sleep -Seconds 5
        }
    }
    cmd.exe /d /c "docker image inspect $baseImage >nul 2>nul"
    if ($LASTEXITCODE -ne 0) {
        throw "Unable to pull official base image after five attempts: $baseImage"
    }
}

Write-Host 'Building the clean experiment-two Docker image from an ASCII-only staging path...'
$asciiBuildRoot = Join-Path $env:TEMP 'mecharm-exp2-docker-build'
$asciiDockerDir = Join-Path $asciiBuildRoot 'docker'
New-Item -ItemType Directory -Path $asciiDockerDir -Force | Out-Null
Copy-Item -LiteralPath (Join-Path $ProjectRoot 'docker\Dockerfile') `
    -Destination (Join-Path $asciiDockerDir 'Dockerfile') -Force
Copy-Item -LiteralPath (Join-Path $ProjectRoot 'docker\entrypoint.sh') `
    -Destination (Join-Path $asciiDockerDir 'entrypoint.sh') -Force

docker build `
    --pull=false `
    --tag mecharm-exp2-ros2:humble `
    --file (Join-Path $asciiDockerDir 'Dockerfile') `
    $asciiBuildRoot
if ($LASTEXITCODE -ne 0) {
    throw 'Docker image build failed. Review the build output above.'
}

Write-Host 'Starting the clean experiment-two MoveIt service...'
docker compose -f $composeFile up -d --no-build --remove-orphans moveit
if ($LASTEXITCODE -ne 0) {
    throw 'Docker Compose failed. Review the build output above.'
}

$env:MECHARM_TRANSPORT = 'tcp'
$env:MECHARM_TCP_BIND = '0.0.0.0'
$env:MECHARM_TCP_PORT = '8765'
Remove-Item Env:ROS_DISCOVERY_SERVER -ErrorAction SilentlyContinue
Remove-Item Env:ROS_SUPER_CLIENT -ErrorAction SilentlyContinue
Remove-Item Env:FASTRTPS_DEFAULT_PROFILES_FILE -ErrorAction SilentlyContinue
Remove-Item Env:CYCLONEDDS_URI -ErrorAction SilentlyContinue

Start-Process powershell.exe -WindowStyle Normal -ArgumentList @(
    '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $isaacLauncher,
    '-ProjectRoot', $ProjectRoot
)

[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$buildCommand = "docker compose -f `"$composeFile`" exec moveit bash -lc `"source /opt/ros/humble/setup.bash && colcon --log-base /opt/mecharm_ws/log build --base-paths /workspace/mecharm_exp2/simulation/urdf/mycobot_description /workspace/mecharm_exp2/src --build-base /opt/mecharm_ws/build --install-base /opt/mecharm_ws/install --symlink-install`""
$command = "docker compose -f `"$composeFile`" exec moveit bash -lc `"source /opt/ros/humble/setup.bash && source /opt/mecharm_ws/install/setup.bash && cd /workspace/mecharm_exp2 && ros2 launch mecharm_moveit_config simulation_moveit.launch.py project_root:=/workspace/mecharm_exp2 use_rviz:=false run_task:=true`""

Write-Host ''
Write-Host 'Clean Docker MoveIt runtime is running with container-local ROS 2.'
Write-Host 'Isaac Sim is starting with the TCP joint bridge on port 8765.'
Write-Host 'Run this once after the image is built or source code changes:'
Write-Host ''
Write-Host $buildCommand
Write-Host ''
Write-Host 'Wait until the Isaac scene is fully loaded and playing, then run:'
Write-Host ''
Write-Host $command
Write-Host ''
