# MT6701 Arduino Library

Arduino library for the MT6701 14-bit magnetic rotary position sensor.

## Features

- 14-bit resolution (16,384 positions per revolution)
- I2C interface (default address 0x06)
- Angle reading in raw counts, radians, or degrees
- Zero position calibration
- Direction control (clockwise/counter-clockwise)
- Magnetic field strength monitoring
- Simple Arduino Wire library integration

## Hardware

- **Sensor:** MT6701 14-bit Magnetic Encoder
- **Interface:** I2C
- **Address:** 0x06 (default)
- **Voltage:** 3.3V
- **Resolution:** 16,384 positions/revolution

## Installation

### Arduino IDE
1. Copy the `MT6701` folder to your Arduino `libraries/` directory
2. Restart Arduino IDE
3. Include in your sketch: `#include <MT6701.h>`

### PlatformIO
Add to `platformio.ini`:
```ini
lib_deps =
    file://../libraries/MT6701
```

## Usage

### Basic Example

```cpp
#include <Wire.h>
#include <MT6701.h>

MT6701 encoder;

void setup() {
    Serial.begin(115200);

    Wire.begin(47, 48);  // SDA=GPIO47, SCL=GPIO48 for ESP32-S3
    Wire.setClock(400000);  // 400 kHz

    if (encoder.begin(&Wire)) {
        Serial.println("MT6701 initialized!");
    } else {
        Serial.println("MT6701 not found!");
    }
}

void loop() {
    uint16_t raw = encoder.readRawAngle();
    float radians = encoder.readAngleRadians();
    float degrees = encoder.readAngleDegrees();

    Serial.print("Raw: ");
    Serial.print(raw);
    Serial.print(" | Radians: ");
    Serial.print(radians, 4);
    Serial.print(" | Degrees: ");
    Serial.println(degrees, 2);

    delay(100);
}
```

### Advanced Features

```cpp
// Check if magnetic field is good
if (encoder.isFieldGood()) {
    Serial.println("Magnetic field strength is good");
} else {
    uint8_t status = encoder.readFieldStatus();
    if (status & MT6701_FIELD_TOO_STRONG) {
        Serial.println("Warning: Magnetic field too strong");
    }
    if (status & MT6701_FIELD_TOO_WEAK) {
        Serial.println("Warning: Magnetic field too weak");
    }
}

// Set current position as zero
encoder.setZeroToCurrent();

// Set specific angle as zero (in radians)
encoder.setZero(1.57);  // Set π/2 as zero

// Set rotation direction
encoder.setDirection(true);  // true = clockwise

// Read zero position setting
float zero = encoder.readZeroPosition();
Serial.print("Zero position: ");
Serial.println(zero);
```

## API Reference

### Initialization

- `MT6701(uint8_t address = 0x06)` - Constructor
- `bool begin(TwoWire *wire = &Wire)` - Initialize encoder

### Reading Functions

- `uint16_t readRawAngle()` - Read raw 14-bit angle (0-16383)
- `float readAngleRadians()` - Read angle in radians (0-2π)
- `float readAngleDegrees()` - Read angle in degrees (0-360)
- `uint8_t readFieldStatus()` - Read magnetic field status
- `bool isFieldGood()` - Check if field strength is good

### Configuration Functions

- `bool setZeroToCurrent()` - Set current position as zero
- `bool setZero(float angle)` - Set zero position (radians)
- `bool setDirection(bool clockwise)` - Set rotation direction
- `float readZeroPosition()` - Read zero position setting
- `bool isClockwise()` - Check if direction is clockwise

### Utility Functions

- `bool testConnection()` - Test I2C communication
- `uint8_t getAddress()` - Get I2C address

## Based On

This library is based on [I-AM-ENGINEER/MT6701-driver](https://github.com/I-AM-ENGINEER/MT6701-driver) and adapted for Arduino/ESP32 compatibility.

## License

MIT License
