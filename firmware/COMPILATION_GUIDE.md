# ESP32 Firmware Compilation Guide

## Overview

The firmware uses the **Arduino framework on ESP32-S3**, not bare-metal ESP-IDF. This is intentional because:

1. **SimpleFOC requires Arduino framework** - It's built on Arduino's HAL
2. **Easier development** - Arduino framework simplifies peripheral access
3. **Standard practice** - Most ESP32 projects use Arduino framework via Arduino IDE or PlatformIO
4. **`#include <Arduino.h>` is correct** - This is the Arduino API for ESP32

## Required Libraries

The firmware uses the **MT6701 library** included in this repository, plus external libraries:

### 1. MT6701 Encoder Library (Included)

**IMPORTANT**: Do NOT install "MT6701-arduino" from Arduino Library Manager! That is a different library with an incompatible API.

The MT6701 library is located in `firmware/libraries/MT6701/` and is **automatically available** to the firmware via symlink.

**Setup Instructions**:

On **Windows**:
1. **DO NOT** install MT6701 from Library Manager (uninstall if you already did)
2. Copy our custom library:
   - Source: `firmware/libraries/MT6701/`
   - Destination: `C:\Users\<YourName>\Documents\Arduino\libraries\MT6701\`
   - Or use the symlink method below if you have admin/developer mode

On **Linux/macOS**:
1. The symlink `firmware/ESP32_MCU_Firmware/libraries` → `../libraries` should work automatically
2. If compilation fails, copy to: `~/Arduino/libraries/MT6701/`

**Symlink Setup (Windows - requires admin/developer mode)**:
```cmd
cd firmware\ESP32_MCU_Firmware
mklink /D libraries ..\libraries
```

**Symlink Setup (Linux/macOS)**:
```bash
cd firmware/ESP32_MCU_Firmware
ln -s ../libraries libraries
```

**Verification**: After setup, Arduino should find our library at compilation. You should see:
```
Using library MT6701 at version 1.0.0 in folder: <path>/firmware/libraries/MT6701
```

**NOT**:
```
Using library MT6701-arduino at version 1.0.3  # WRONG LIBRARY!
```

### 2. SimpleFOC (Install via Library Manager)

Install these through Arduino Library Manager:

1. **SimpleFOC** (>= 2.3.0)
   - Motor control with FOC algorithm
   - Encoder support
   - PID controllers

2. **SerialTransfer** for Arduino
   - Binary serial communication
   - Install via Library Manager: "SerialTransfer"

3. **ESP32 Board Support**
   - In Arduino IDE: File → Preferences → Additional Boards Manager URLs
   - Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Then: Tools → Board → Boards Manager → Search "ESP32" → Install

## Board Configuration

In Arduino IDE, select:
- **Board**: "ESP32S3 Dev Module"
- **USB CDC On Boot**: "Enabled" (for serial communication)
- **Flash Size**: "16MB (128Mb)"
- **Partition Scheme**: "Default 4MB with spiffs"
- **Upload Speed**: "921600"

## Pin Configuration

The firmware is now located in `firmware/ESP32_MCU_Firmware/`.

Edit `firmware/ESP32_MCU_Firmware/config.h` before compiling:

```cpp
// Motor driver pins (SimpleFOC Mini)
#define MOTOR_PWM_A      10   // Phase A PWM
#define MOTOR_PWM_B      11   // Phase B PWM
#define MOTOR_PWM_C      12   // Phase C PWM
#define MOTOR_ENABLE     13   // Enable pin

// MT6701 Encoder pins (I2C)
#define ENCODER_SDA      47   // I2C SDA
#define ENCODER_SCL      48   // I2C SCL
#define ENCODER_I2C_ADDR 0x06 // MT6701 I2C address

// Motor parameters (IMPORTANT: Match your motor!)
#define POLE_PAIRS       7     // Count magnets, divide by 2
#define ENCODER_PPR      16384 // MT6701: 14-bit = 16384 counts
#define VOLTAGE_PSU      12.0  // Your power supply voltage
#define CURRENT_LIMIT    1.0   // Safe current limit (amps)
```

## Compilation Fixes Applied

### 1. MT6701 Encoder Integration
**Updated**: Now uses native MT6701 library instead of SimpleFOC's generic I2C sensor
- Library location: `firmware/libraries/MT6701/`
- Features: 14-bit resolution, field strength monitoring, zero calibration
- Accessed via symlink in `firmware/ESP32_MCU_Firmware/libraries/`

### 2. Pin Definitions
**Fixed**: Changed from ESP-IDF style (`GPIO_NUM_10`) to Arduino style (`10`)
- Arduino framework uses plain integers for pins
- ESP-IDF uses `gpio_num_t` enum

### 3. Encoder Interface
**Updated**: Replaced quadrature encoder with MT6701 I2C encoder
- No interrupt handlers needed (I2C communication)
- SimpleFOC wrapper class provides compatibility
- Automatic field strength validation
**Fixed**: Corrected buffer access
- Changed `transfer.packet.rxBuff` → `transfer.rxBuff`
- Changed `transfer.packet.txBuff` → `transfer.txBuff`
- Changed `transfer.sendData()` → `transfer.sendDatum()`

### 5. Removed Unused Code
**Fixed**: Removed `g_motor_controller` global pointer (not needed)

## Expected Warnings (Safe to Ignore)

You may see these warnings during compilation:

1. **"pragma message" warnings from SimpleFOC**
   - These are informational, not errors
   - SimpleFOC reports detected board and features

2. **Unused variable warnings**
   - Debug code may have unused variables
   - Safe to ignore or set `DEBUG_MOTOR = false`

## Known Issues and Solutions

### Issue: "MT6701.h: No such file or directory"
**Solution**: Copy our custom MT6701 library to your Arduino libraries folder.
- Source: `firmware/libraries/MT6701/`
- Destination: `~/Arduino/libraries/MT6701/` (Linux/macOS) or `C:\Users\<YourName>\Documents\Arduino\libraries\MT6701\` (Windows)

### Issue: "no matching function for call to 'MT6701::MT6701(uint8_t&)'"
**Solution**: You have the wrong MT6701 library installed!
1. **Uninstall "MT6701-arduino"** from Library Manager if installed
2. **Use our custom library** from `firmware/libraries/MT6701/`
3. Copy it to your Arduino libraries folder (see above)
4. Restart Arduino IDE
5. Verify compilation uses: `Using library MT6701 at version 1.0.0`

### Issue: "SimpleFOC.h: No such file"
**Solution**: Install SimpleFOC library via Library Manager

### Issue: "SerialTransfer.h: No such file"
**Solution**: Install SerialTransfer library via Library Manager

### Issue: "Encoder was not declared"
**Solution**: This error should no longer occur with MT6701 integration
- Old firmware used SimpleFOC's Encoder class (quadrature)
- New firmware uses MT6701Sensor wrapper class

### Issue: Compilation takes a long time
**Expected**: First compilation of SimpleFOC is slow (~2-3 minutes)
- SimpleFOC has many template instantiations
- Subsequent compilations are faster

### Issue: Upload fails / Port not found
**Solutions**:
- Press and hold BOOT button during upload
- Check USB cable (needs data lines, not just power)
- Install CH340 or CP2102 drivers (depending on board)

## Motor Calibration Required

After first upload, motor MUST be calibrated:

```python
from motor_control import MotorController

motor = MotorController('/dev/ttyUSB0')  # or COM3 on Windows
motor.connect()
motor.calibrate()  # This aligns motor phases with encoder
```

**What calibration does**:
1. Runs motor through electrical rotation
2. Aligns encoder zero with motor electrical zero
3. Verifies encoder is responding correctly
4. Stores calibration in motor object (not persistent)

**Note**: Calibration is NOT saved to EEPROM - run on each power-up

## Testing After Upload

### 1. Serial Monitor Test
Open Serial Monitor (115200 baud), you should see:
```
=== OpenHyperspectral Motor Controller ===
Firmware Version: 1.0.0
Board: ESP32-S3-Touch-LCD-2 (Waveshare)
Communication initialized
Motor controller initialized
System ready!
Waiting for commands...
```

### 2. Python Test
```bash
python motor_control/controller.py /dev/ttyUSB0
```

Expected output:
```
==== TESTING MOTOR CONTROLLER ON /dev/ttyUSB0 ====

Connecting to motor controller...
✅ CONNECTION SUCCESSFUL

Testing ping command...
✅ PING SUCCESSFUL - Echo value: 0x87654321

Testing status command...
✅ STATUS SUCCESSFUL
```

## PID Tuning

Default PID values are conservative. You may need to tune for your motor:

### Position Control (P_angle)
```cpp
#define PID_P_POSITION   20.0   // Higher = stiffer position hold
#define PID_I_POSITION   0.0    // Usually keep at 0
#define PID_D_POSITION   0.1    // Damping
```

### Velocity Control (PID_velocity)
```cpp
#define PID_P_VELOCITY   0.5    // Higher = faster response
#define PID_I_VELOCITY   10.0   // Removes steady-state error
#define PID_D_VELOCITY   0.0    // Usually keep at 0
```

### Tuning Tips
1. Start with low P, increase until oscillation
2. Back off P by 30%
3. Add I slowly to remove steady-state error
4. Add D if oscillation persists

## Troubleshooting Compilation

### Linker Errors
If you see undefined reference errors:
1. Make sure all `.cpp` files are in same folder as `.ino`
2. Arduino automatically compiles all files in sketch folder
3. Check SimpleFOC is installed correctly

### Template Errors
SimpleFOC uses heavy templating:
- Errors can be cryptic
- Usually indicate version mismatch
- Try SimpleFOC version 2.3.0 or 2.3.1

### Memory Issues
If compilation fails with memory errors:
- ESP32-S3 should have plenty of flash/RAM
- Check partition scheme is correct
- Reduce `DEBUG_MOTOR` and `DEBUG_COMM` output

## Performance Expectations

With correct compilation:
- **Loop frequency**: ~1000 Hz (1ms per iteration)
- **FOC frequency**: ~1000 Hz
- **Serial latency**: ~10-20ms for command processing
- **Position accuracy**: ~0.1 radians with good tuning

## Next Steps After Successful Compilation

1. **Upload firmware** to ESP32-S3
2. **Wire motor and encoder** per pin configuration
3. **Run calibration** via Python controller
4. **Test basic movement** with example scripts
5. **Tune PID parameters** for your specific motor
6. **Integrate with camera** for synchronized scanning
