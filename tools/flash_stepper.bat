@echo off
:: Compile and upload the STEPPER firmware (ESP32_Stepper_Firmware) in one step.
:: Usage: flash_stepper.bat [COM port]   (default: COM3)
::
:: IMPORTANT: Close Arduino IDE serial monitor and SpectrumBoi before running.
:: Only one program can use the COM port at a time.

set PORT=%1
if "%PORT%"=="" set PORT=COM3

set TOOLS_DIR=%~dp0
set CLI=%TOOLS_DIR%arduino-cli.exe
set SKETCH=A:\Code_and_Data\Spectrometry\OpenHyperspectral\firmware\ESP32_Stepper_Firmware
set LIB_SERIALTRANSFER=A:\Code_and_Data\Arduino\sketchbook\libraries\SerialTransfer
set LIB_TMCSTEPPER=A:\Code_and_Data\Arduino\sketchbook\libraries\TMCStepper
set FQBN=esp32:esp32:esp32s3:UploadSpeed=921600,USBMode=hwcdc,CDCOnBoot=cdc,MSCOnBoot=default,DFUOnBoot=default,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=4M,PartitionScheme=default,DebugLevel=none,PSRAM=disabled,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default

if not exist "%CLI%" (
    echo ERROR: arduino-cli.exe not found at %CLI%
    echo Run tools\install_arduino_cli.ps1 first.
    exit /b 1
)

echo === Compiling and uploading STEPPER firmware to %PORT% ===
echo (Close Arduino IDE serial monitor and SpectrumBoi if upload fails with "port busy")
echo.

"%CLI%" compile --upload --port %PORT% --fqbn "%FQBN%" ^
    --library "%LIB_SERIALTRANSFER%" ^
    --library "%LIB_TMCSTEPPER%" ^
    "%SKETCH%"

if errorlevel 1 (
    echo.
    echo FAILED. If error is "port busy" - close serial monitor and retry.
    exit /b 1
)

echo.
echo === Done! Stepper firmware flashed to %PORT% ===
