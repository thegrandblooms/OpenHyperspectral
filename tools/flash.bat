@echo off
:: Compile and upload ESP32 firmware in one step.
:: After flashing, run: py -3.9 tools\motor_test.py --preset baseline
:: Usage: flash.bat [COM port]   (default: COM3)
::
:: IMPORTANT: Close Arduino IDE serial monitor and SpectrumBoi before running.
:: Only one program can use the COM port at a time.

set PORT=%1
if "%PORT%"=="" set PORT=COM3

set TOOLS_DIR=%~dp0
set CLI=%TOOLS_DIR%arduino-cli.exe
set SKETCH=A:\Code_and_Data\Spectrometry\OpenHyperspectral\firmware\ESP32_MCU_Firmware
set LIB_SIMPLEFOC=A:\Code_and_Data\Arduino\sketchbook\libraries\Simple_FOC
set LIB_SERIALTRANSFER=A:\Code_and_Data\Arduino\sketchbook\libraries\SerialTransfer
set LIB_MT6701=A:\Code_and_Data\Spectrometry\OpenHyperspectral\firmware\libraries\MT6701
set LIB_SIMPLEFOC_DRIVERS=A:\Code_and_Data\Spectrometry\OpenHyperspectral\firmware\libraries\SimpleFOCDrivers_Calibrated
set FQBN=esp32:esp32:esp32s3:UploadSpeed=921600,USBMode=hwcdc,CDCOnBoot=cdc,MSCOnBoot=default,DFUOnBoot=default,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=4M,PartitionScheme=default,DebugLevel=none,PSRAM=disabled,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default

if not exist "%CLI%" (
    echo ERROR: arduino-cli.exe not found at %CLI%
    echo Run tools\install_arduino_cli.ps1 first.
    exit /b 1
)

echo === Compiling and uploading to %PORT% ===
echo (Close Arduino IDE serial monitor and SpectrumBoi if upload fails with "port busy")
echo.

"%CLI%" compile --upload --port %PORT% --fqbn "%FQBN%" ^
    --library "%LIB_SIMPLEFOC%" ^
    --library "%LIB_SERIALTRANSFER%" ^
    --library "%LIB_MT6701%" ^
    "%SKETCH%"

if errorlevel 1 (
    echo.
    echo FAILED. If error is "port busy" - close serial monitor and retry.
    exit /b 1
)

echo.
echo === Done! Firmware flashed to %PORT% ===
