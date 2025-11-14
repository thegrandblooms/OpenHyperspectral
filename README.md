# OpenHyperspectral

An Open Source and Modular **1D Line-Scanning Hyperspectral Imaging System** with synchronized motor control.

## Overview

OpenHyperspectral is a line-scanning hyperspectral system that combines:
- **ESP32-S3-Touch-LCD-2** (Waveshare) for BLDC motor control with SimpleFOC
- **DRV8313** SimpleFOC motor driver board v1
- **MT6701** 14-bit magnetic encoder for precise position feedback
- **USB Serial Protocol** for PC-motor synchronization
- **Mightex Monochrome Camera** for hyperspectral line capture
- **SpectrumBoi UI** for preview and control
- **Python Main Program** orchestrating scanning, capture, and processing

## Architecture

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed system architecture.

```
Main Computer
     │
     ├─── USB ───> Mightex Monochrome Camera (line capture)
     │
     └─── USB ───> ESP32-S3-Touch-LCD-2
                        │
                        ├─── PWM ───> DRV8313 ───> BLDC Motor
                        │
                        └─── I2C/ABZ ───> MT6701 Encoder
```

## Features

### Motor Control
- **SimpleFOC BLDC Control**: Smooth, precise motor control with field-oriented control
- **DRV8313 Driver**: SimpleFOC motor driver board v1 for 3-phase BLDC motors
- **MT6701 Encoder**: 14-bit absolute position sensing via I2C or ABZ interface
- **Position Synchronization**: USB serial protocol for exact position-based triggering
- **Multiple Control Modes**: Position, velocity, and torque control
- **Real-time Feedback**: High-resolution encoder position tracking
- **Safety Features**: Current limiting, emergency stop, calibration

### 1D Line Scanning
- **Linear Scanning**: Single-axis motor control for line-by-line acquisition
- **Position-Based Triggering**: Camera captures at precise motor positions
- **Data Cube Construction**: Build hyperspectral cubes from line scans
- **SpectrumBoi Integration**: Real-time preview and control UI
- **Data Organization**: Automatic file organization with metadata

### Communication
- **Binary Protocol**: Efficient SerialTransfer-based communication
- **Async Position Monitoring**: Non-blocking position notification system
- **Error Handling**: Robust error detection and retry logic
- **Status Reporting**: Comprehensive system status queries

## Hardware Setup

### ESP32-S3-Touch-LCD-2 (Waveshare)

**Motor Driver Connections:**
- GPIO10 → Motor Phase A (PWM)
- GPIO11 → Motor Phase B (PWM)
- GPIO12 → Motor Phase C (PWM)
- GPIO13 → Motor Enable

**Encoder Connections:**
- GPIO14 → Encoder Channel A
- GPIO15 → Encoder Channel B
- GPIO16 → Encoder Index (optional)

**Display:** Built-in ST7789 LCD (pre-configured)

### Motor and Encoder Configuration

Edit `firmware/motor_firmware/config.h` to match your hardware:

```cpp
// Motor driver pins (Arduino framework style)
#define MOTOR_PWM_A      10         // GPIO10 - Phase A
#define MOTOR_PWM_B      11         // GPIO11 - Phase B
#define MOTOR_PWM_C      12         // GPIO12 - Phase C
#define MOTOR_ENABLE     13         // GPIO13 - Enable

// MT6701 Encoder pins (ABZ interface)
#define ENCODER_A        14         // GPIO14 - Channel A
#define ENCODER_B        15         // GPIO15 - Channel B
#define ENCODER_I        -1         // Index not used

// Motor parameters
#define POLE_PAIRS       7          // Motor pole pairs (magnets / 2)
#define ENCODER_PPR      2048       // Encoder pulses per revolution
#define VOLTAGE_PSU      12.0       // Power supply voltage (V)
#define CURRENT_LIMIT    1.0        // Maximum current (A)
```

**Note**: MT6701 can be used in ABZ (incremental) or I2C (absolute) mode. Default config uses ABZ.

## Software Installation

### Prerequisites

**Arduino IDE:**
- ESP32 board support (ESP32-S3)
- SimpleFOC library (>= 2.3.0)
- SerialTransfer library
- LVGL (>= 8.3.0) - if using display

**Python:**
```bash
pip install pySerialTransfer numpy opencv-python
```

### Firmware Upload

1. Open `firmware/motor_firmware/motor_firmware.ino` in Arduino IDE
2. Select **ESP32-S3 Dev Module** as board
3. Configure your motor parameters in `config.h`
4. Upload to ESP32

### Python Setup

```bash
cd OpenHyperspectral
pip install -e .
```

## Usage

### Basic Motor Control

```python
from motor_control import MotorController

# Connect to motor controller
motor = MotorController('/dev/ttyUSB0')  # or 'COM3' on Windows
motor.connect()

# Calibrate motor (first time only)
motor.calibrate()

# Enable and move to position
motor.enable()
motor.move_to(1.57)  # Move to π/2 radians (90 degrees)

# Wait for position reached
position = motor.wait_for_position(sequence_id)
print(f"Reached position: {position:.4f} rad")

# Cleanup
motor.disable()
motor.disconnect()
```

### Camera Integration (1D Line Scan)

```python
from motor_control import MotorController
from mightex_driver import Camera
import numpy as np

# Initialize systems
motor = MotorController('/dev/ttyUSB0')
camera = Camera()

motor.connect()
camera.connect()

# Storage for hyperspectral data cube
data_cube = []

# Synchronized line capture function
def capture_line(position, seq_id, data):
    print(f"Capturing line at position {position:.4f} rad")
    line_image = camera.capture()  # Capture one line
    data_cube.append({
        'position': position,
        'sequence': seq_id,
        'data': line_image
    })

# Set position callback
motor.set_position_callback(capture_line)

# 1D linear scan (move through positions)
start_position = 0.0
end_position = 6.28  # One full rotation
num_lines = 100

positions = np.linspace(start_position, end_position, num_lines)

for i, pos in enumerate(positions):
    seq_id = motor.queue_movement(pos)
    motor.wait_for_position(seq_id)
    print(f"Line {i+1}/{num_lines} captured")

# Save hyperspectral data cube
np.save('hyperspectral_cube.npy', data_cube)

# Cleanup
motor.disconnect()
camera.disconnect()
```

## Testing

### Test Motor Communication

```bash
python motor_control/controller.py /dev/ttyUSB0
```

This runs a comprehensive test suite including:
- Connection and ping test
- Status query
- Motor calibration
- Position movement
- Home command

## Calibration

Motor calibration is required on first use:

```python
motor = MotorController('/dev/ttyUSB0')
motor.connect()
motor.calibrate()
```

## Troubleshooting

### Motor not moving
- Check power supply voltage (12V recommended)
- Verify motor phase connections
- Ensure motor is calibrated: `motor.calibrate()`
- Check current limit setting

### Encoder not working
- Verify encoder connections (A, B channels)
- Check encoder power (usually 5V)
- Test encoder independently

### Communication timeouts
- Check USB cable quality
- Reduce baud rate in `config.h` (try 57600)
- Add delay after connect: `time.sleep(3)`

## Directory Structure

```
OpenHyperspectral/
├── firmware/                  # ESP32 firmware
│   └── motor_firmware/
│       ├── motor_firmware.ino
│       ├── config.h
│       ├── commands.h
│       ├── communication.h/.cpp
│       └── motor_control.h/.cpp
├── motor_control/            # Python motor control
│   ├── __init__.py
│   └── controller.py
├── camera/                   # Camera interface
│   ├── camera_ui.py
│   └── camera_viewer.py
├── mightex_driver/          # Camera driver
│   └── camera.py
└── references/              # Reference implementations
    ├── HyperMicro/
    └── esp-32_motor_control/
```

## Hardware Components

- **ESP32-S3-Touch-LCD-2** (Waveshare): Main controller with 2.1" LCD display
- **DRV8313**: SimpleFOC motor driver board v1 (3-phase BLDC driver)
- **MT6701**: 14-bit magnetic encoder (I2C or ABZ interface)
- **BLDC Motor**: 3-phase brushless DC motor (user-specified)
- **Mightex Monochrome Camera**: Hyperspectral line imaging camera
- **Power Supply**: 12V for motor driver (recommended)

## References

This project integrates components from:
- **HyperMicro**: USB serial synchronization protocol (references/HyperMicro)
- **ESP-32 Motor Control**: Modular driver architecture (references/esp-32_motor_control)
- **SimpleFOC**: BLDC motor control library
- **SpectrumBoi**: Spectrometer UI and visualization

## License

See [LICENSE](LICENSE) file.
