# Run once from an Administrator PowerShell.
$ErrorActionPreference = 'Stop'
$ruleName = 'Isaac Sim 5.1 mechArm TCP Bridge'
$oldRuleName = 'Isaac Sim 5.1 ROS2 DDS Domain 44'
$program = 'E:\AIRobotic\isaac-sim\kit\python\kit.exe'

if (-not (Test-Path -LiteralPath $program -PathType Leaf)) {
    throw "Isaac Sim process not found: $program"
}

netsh advfirewall firewall delete rule name="$oldRuleName" | Out-Null
netsh advfirewall firewall delete rule name="$ruleName" | Out-Null
netsh advfirewall firewall add rule `
    name="$ruleName" `
    dir=in `
    action=allow `
    protocol=TCP `
    localport=8765 `
    remoteip=127.0.0.1,172.16.0.0/12 `
    program="$program" `
    profile=any `
    enable=yes

if ($LASTEXITCODE -ne 0) {
    throw 'Firewall rule creation failed. Open PowerShell as Administrator and run this script again.'
}

Write-Host 'Enabled Isaac Sim mechArm TCP bridge port 8765 for Docker private addresses.'
