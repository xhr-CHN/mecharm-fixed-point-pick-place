$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$linuxRoot = wsl.exe -d Ubuntu-22.04 -u xhr -- wslpath -a "$projectRoot"
if ($LASTEXITCODE -ne 0) {
    throw 'Failed to resolve the experiment directory in WSL.'
}
$linuxRoot = $linuxRoot.Trim()

wsl.exe -d Ubuntu-22.04 -u xhr -- bash -lc "source /opt/ros/humble/setup.bash && source '$linuxRoot/install/setup.bash' && ros2 launch mecharm_pick_place simulation.launch.py"
exit $LASTEXITCODE

