# Install arduino-cli into tools/
# Run once from PowerShell: cd tools; .\install_arduino_cli.ps1

$ErrorActionPreference = "Stop"
$ToolsDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$CliExe = Join-Path $ToolsDir "arduino-cli.exe"

if (Test-Path $CliExe) {
    Write-Host "arduino-cli already installed at $CliExe"
    & $CliExe version
    exit 0
}

Write-Host "Downloading arduino-cli..."
$ZipPath = Join-Path $ToolsDir "arduino-cli.zip"
Invoke-WebRequest -Uri "https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip" -OutFile $ZipPath
Expand-Archive -Path $ZipPath -DestinationPath $ToolsDir -Force
Remove-Item $ZipPath

Write-Host "Configuring arduino-cli to use existing Arduino15 packages..."
$ConfigDir = Join-Path $env:USERPROFILE ".arduino-cli"
New-Item -ItemType Directory -Force -Path $ConfigDir | Out-Null

$Config = @"
board_manager:
  additional_urls: []
daemon:
  port: "50051"
directories:
  data: C:\Users\$env:USERNAME\AppData\Local\Arduino15
  downloads: C:\Users\$env:USERNAME\AppData\Local\Arduino15\staging
  user: A:\Code_and_Data\Arduino\sketchbook
logging:
  file: ""
  format: text
  level: warn
metrics:
  addr: :9090
  enabled: true
output:
  no_color: false
updater:
  enable_notification: false
"@

$Config | Set-Content (Join-Path $ConfigDir "arduino-cli.yaml")

Write-Host ""
Write-Host "Done! Testing..."
& $CliExe version
& $CliExe board listall esp32s3 2>$null | Select-String "waveshare|esp32s3" | head -5
Write-Host ""
Write-Host "arduino-cli is ready. Run flash.bat to compile and upload firmware."
