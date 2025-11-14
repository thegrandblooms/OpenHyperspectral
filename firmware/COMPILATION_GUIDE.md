# ESP32 Firmware Compilation Guide

## Overview

The firmware uses the **Arduino framework on ESP32-S3**, not bare-metal ESP-IDF. This is intentional because:

1. **SimpleFOC requires Arduino framework** - It's built on Arduino's HAL
2. **Easier development** - Arduino framework simplifies peripheral access
3. **Standard practice** - Most ESP32 projects use Arduino framework via Arduino IDE or PlatformIO
4. **`#include <Arduino.h>` is correct** - This is the Arduino API for ESP32

## Required Libraries

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

Edit `firmware/motor_firmware/config.h` before compiling:

```cpp
// Motor driver pins
#define MOTOR_PWM_A      10   // Phase A PWM
#define MOTOR_PWM_B      11   // Phase B PWM
#define MOTOR_PWM_C      12   // Phase C PWM
#define MOTOR_ENABLE     13   // Enable pin

// Encoder pins
#define ENCODER_A        14   // Channel A
#define ENCODER_B        15   // Channel B

// Motor parameters (IMPORTANT: Match your motor!)
#define POLE_PAIRS       7     // Count magnets, divide by 2
#define ENCODER_PPR      2048  // Pulses per revolution
#define VOLTAGE_PSU      12.0  // Your power supply voltage
#define CURRENT_LIMIT    1.0   // Safe current limit (amps)
```

## Compilation Fixes Applied

### 1. Pin Definitions
**Fixed**: Changed from ESP-IDF style (`GPIO_NUM_10`) to Arduino style (`10`)
- Arduino framework uses plain integers for pins
- ESP-IDF uses `gpio_num_t` enum

### 2. Encoder Interrupts
**Fixed**: Added proper ISR handlers for SimpleFOC encoder
```cpp
// Added global encoder pointer and ISR functions
Encoder* g_encoder = nullptr;

void doA() { if (g_encoder) g_encoder->handleA(); }
void doB() { if (g_encoder) g_encoder->handleB(); }
```

### 3. SerialTransfer API
**Fixed**: Corrected buffer access
- Changed `transfer.packet.rxBuff` → `transfer.rxBuff`
- Changed `transfer.packet.txBuff` → `transfer.txBuff`
- Changed `transfer.sendData()` → `transfer.sendDatum()`

### 4. Removed Unused Code
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

### Issue: "SimpleFOC.h: No such file"
**Solution**: Install SimpleFOC library via Library Manager

### Issue: "SerialTransfer.h: No such file"
**Solution**: Install SerialTransfer library via Library Manager

### Issue: "Encoder was not declared"
**Solution**: Make sure SimpleFOC library is installed correctly

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
