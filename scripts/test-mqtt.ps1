$ErrorActionPreference = "Stop"

$subExe = "C:\Program Files\mosquitto\mosquitto_sub.exe"
$pubExe = "C:\Program Files\mosquitto\mosquitto_pub.exe"

if (-not (Test-Path $subExe) -or -not (Test-Path $pubExe)) {
    throw "mosquitto_pub/sub not found in C:\Program Files\mosquitto"
}

$topic = "test/codex"
$payload = "mqtt_ok"

$job = Start-Job -ScriptBlock {
    & $using:subExe -h 127.0.0.1 -p 1883 -t $using:topic -C 1 -W 5
}
Start-Sleep -Seconds 1
& $pubExe -h 127.0.0.1 -p 1883 -t $topic -m $payload | Out-Null
$result = Receive-Job -Job $job -Wait
Remove-Job $job

if ($result -ne $payload) {
    throw "MQTT test failed. Expected '$payload', got '$result'"
}

Write-Output "MQTT test passed: $result"
