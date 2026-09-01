# Run once from an Administrator PowerShell.
$ErrorActionPreference = 'Stop'
$ruleName = 'Isaac Sim 5.1 ROS2 DDS Domain 0'
$program = 'E:\AIRobotic\isaac-sim\kit\python\kit.exe'

if (-not (Test-Path -LiteralPath $program -PathType Leaf)) {
    throw "Isaac Sim process not found: $program"
}

netsh advfirewall firewall delete rule name="$ruleName" | Out-Null
netsh advfirewall firewall add rule `
    name="$ruleName" `
    dir=in `
    action=allow `
    protocol=UDP `
    localport=7400-7500 `
    remoteip=LocalSubnet `
    program="$program" `
    profile=any `
    enable=yes

if ($LASTEXITCODE -ne 0) {
    throw 'Firewall rule creation failed. Open PowerShell as Administrator and run this script again.'
}
