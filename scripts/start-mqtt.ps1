$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$mosquittoExe = "C:\Program Files\mosquitto\mosquitto.exe"
$configPath = Join-Path $projectRoot ".mqtt\mosquitto.conf"
$dataPath = Join-Path $projectRoot ".mqtt\data"
$logPath = Join-Path $projectRoot ".mqtt\log"
$pidFile = Join-Path $projectRoot ".mqtt\mosquitto.pid"

if (-not (Test-Path $mosquittoExe)) {
    throw "Mosquitto not found: $mosquittoExe"
}

New-Item -ItemType Directory -Force -Path $dataPath, $logPath | Out-Null

if (-not (Test-Path $configPath)) {
    @"
listener 1883 0.0.0.0
allow_anonymous true
persistence true
persistence_location ./.mqtt/data/
log_dest file ./.mqtt/log/mosquitto.log
pid_file ./.mqtt/mosquitto.pid
"@ | Set-Content -Path $configPath -Encoding ascii
}

if (Test-Path $pidFile) {
    $pidText = (Get-Content $pidFile -Raw).Trim()
    $tmpPid = 0
    if ([int]::TryParse($pidText, [ref]$tmpPid)) {
        $existingProc = Get-Process -Id $tmpPid -ErrorAction SilentlyContinue
        if ($existingProc) {
            Write-Output "Mosquitto already running. PID=$tmpPid"
            exit 0
        }
    }
}

$proc = Start-Process -FilePath $mosquittoExe -ArgumentList @("-c", $configPath, "-v") -WorkingDirectory $projectRoot -PassThru
Start-Sleep -Seconds 1
if ($proc.HasExited) {
    throw "Mosquitto exited immediately. Check .mqtt/log/mosquitto.log"
}
Write-Output "Mosquitto started. PID=$($proc.Id), config=$configPath"
