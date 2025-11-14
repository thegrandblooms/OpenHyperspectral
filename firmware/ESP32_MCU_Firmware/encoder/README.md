# MT6701 Encoder Driver

This directory contains the MT6701 14-bit magnetic rotary encoder driver.

## Files

- `MT6701.h` - Header file with class definition
- `MT6701.cpp` - Implementation

## Integration

These files are automatically compiled by Arduino IDE as part of the `ESP32_MCU_Firmware` sketch. No separate library installation is required.

## Usage

```cpp
#include "encoder/MT6701.h"

MT6701 encoder(0x06);  // I2C address
encoder.begin(&Wire);
float angle = encoder.readAngleRadians();
```

## Features

- 14-bit resolution (16,384 positions per revolution)
- I2C interface (address 0x06)
- Configurable zero position
- Direction control
- Magnetic field strength monitoring
- Non-volatile EEPROM configuration

## SimpleFOC Integration

The `motor_control.h` file includes `MT6701Sensor` wrapper class that adapts this driver to work with SimpleFOC's `Sensor` interface.
