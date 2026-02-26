$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$pidFile = Join-Path $projectRoot ".mqtt\mosquitto.pid"

if (-not (Test-Path $pidFile)) {
    Write-Output "PID file not found: $pidFile"
    Write-Output "If needed, stop manually: Stop-Process -Name mosquitto"
    exit 0
}

$pidText = (Get-Content $pidFile -Raw).Trim()
$tmpPid = 0
if (-not [int]::TryParse($pidText, [ref]$tmpPid)) {
    Write-Output "Invalid PID value in ${pidFile}: '$pidText'"
    exit 1
}

$pidValue = $tmpPid
$proc = Get-Process -Id $pidValue -ErrorAction SilentlyContinue
if (-not $proc) {
    Write-Output "Process PID=$pidValue already stopped."
    Remove-Item $pidFile -Force -ErrorAction SilentlyContinue
    exit 0
}

Stop-Process -Id $pidValue -Force
Start-Sleep -Milliseconds 300
Remove-Item $pidFile -Force -ErrorAction SilentlyContinue
Write-Output "Mosquitto stopped. PID=$pidValue"
