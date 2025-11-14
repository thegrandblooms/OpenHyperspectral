# OpenHyperspectral

An Open Source and Modular Hyperspectral Camera with synchronized motor control for scanning applications.

## Overview

OpenHyperspectral integrates:
- **ESP32-S3-Touch-LCD-2** (Waveshare) for BLDC motor control with SimpleFOC
- **USB Serial Protocol** for PC-motor synchronization
- **Mightex Camera** for hyperspectral image capture
- **Python Controller** for high-level scanning and data collection

## Architecture

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed system architecture.

```
PC (Python)                    ESP32-S3                    Hardware
┌──────────────┐              ┌────────────┐              ┌──────────┐
│   Camera     │              │  SimpleFOC │──────────────│   BLDC   │
│   Control    │              │   Motor    │              │  Motor   │
└──────────────┘              │  Control   │              └──────────┘
       │                      └────────────┘
       │                             │
┌──────────────┐              ┌────────────┐
│    Motor     │◄──USB Serial─│   Serial   │
│  Controller  │   Protocol   │    Comm    │
└──────────────┘              └────────────┘
       │
┌──────────────┐
│   Scanner    │
│   (Grid/     │
│   Spiral)    │
└──────────────┘
```

## Features

### Motor Control
- **SimpleFOC BLDC Control**: Smooth, precise motor control with field-oriented control
- **Position Synchronization**: USB serial protocol for exact position-based triggering
- **Multiple Control Modes**: Position, velocity, and torque control
- **Real-time Feedback**: Encoder-based position tracking
- **Safety Features**: Current limiting, emergency stop, calibration

### Scanning
- **Grid Patterns**: Regular grid scanning for systematic coverage
- **Spiral Patterns**: Efficient spiral scanning starting from center
- **Camera Synchronization**: Automatic image capture at each motor position
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

### Motor Configuration

Edit `firmware/motor_firmware/config.h` to match your motor:

```cpp
#define POLE_PAIRS       7          // Motor pole pairs
#define ENCODER_PPR      2048        // Encoder pulses per revolution
#define VOLTAGE_PSU      12.0        // Power supply voltage
#define CURRENT_LIMIT    1.0         // Maximum current (A)
```

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

### Camera Integration

```python
from motor_control import MotorController
from mightex_driver import Camera

# Initialize both systems
motor = MotorController('/dev/ttyUSB0')
camera = Camera()

motor.connect()
camera.connect()

# Synchronized capture function
def synchronized_capture(position, seq_id, data):
    print(f"Capturing at position {position:.4f} rad")
    image = camera.capture()
    filename = f"scan_{seq_id:04d}_{position:.4f}.tiff"
    camera.save_image(image, filename)

# Set position callback
motor.set_position_callback(synchronized_capture)

# Move to positions
for i, pos in enumerate(scan_positions):
    seq_id = motor.queue_movement(pos)
    motor.wait_for_position(seq_id)

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

## References

This project integrates components from:
- **HyperMicro**: USB serial synchronization protocol (references/HyperMicro)
- **ESP-32 Motor Control**: Modular driver architecture (references/esp-32_motor_control)
- **SimpleFOC**: BLDC motor control library

## License

See [LICENSE](LICENSE) file.
