# OpenHyperspectral Architecture

## System Overview

OpenHyperspectral is a hyperspectral imaging system that synchronizes a camera with motorized scanning via USB communication. The system combines:

1. **ESP32-S3-Touch-LCD-2** (Waveshare) - Motor control with display
2. **SimpleFOC** - BLDC motor control library
3. **USB Serial Protocol** - Synchronization between PC and ESP32
4. **Mightex Camera** - Hyperspectral image capture
5. **Python Controller** - High-level scanning and data collection

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        PC/Computer                               │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  camera_viewer.py / camera_ui.py                           │ │
│  │  - Mightex camera interface                                │ │
│  │  - Image capture and display                               │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  motor_controller.py (based on HyperMicro controller)      │ │
│  │  - USB serial communication                                │ │
│  │  - Position tracking and callbacks                         │ │
│  │  - Movement queue management                               │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  scanner.py (based on HyperMicro scanner)                  │ │
│  │  - Grid/spiral scanning patterns                           │ │
│  │  - Camera + motor synchronization                          │ │
│  │  - Data organization and metadata                          │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────┼──────────────────────────────────────┘
                           │ USB Serial (SerialTransfer Protocol)
┌──────────────────────────┴──────────────────────────────────────┐
│              ESP32-S3-Touch-LCD-2 (Waveshare)                    │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  motor_firmware.ino                                        │ │
│  │  ┌──────────────────────────────────────────────────────┐ │ │
│  │  │  Serial Communication Module (HyperMicro protocol)   │ │ │
│  │  │  - Command processing                                │ │ │
│  │  │  - Position notifications                            │ │ │
│  │  │  - Status reporting                                  │ │ │
│  │  └──────────────────────────────────────────────────────┘ │ │
│  │  ┌──────────────────────────────────────────────────────┐ │ │
│  │  │  Motor Control Module (SimpleFOC)                    │ │ │
│  │  │  - FOC algorithm                                     │ │ │
│  │  │  - Position/velocity control                         │ │ │
│  │  │  - Encoder feedback                                  │ │ │
│  │  └──────────────────────────────────────────────────────┘ │ │
│  │  ┌──────────────────────────────────────────────────────┐ │ │
│  │  │  Display Module (LVGL + ST7789)                      │ │ │
│  │  │  - Status display                                    │ │ │
│  │  │  - Manual control interface                          │ │ │
│  │  │  - Touch/encoder input                               │ │ │
│  │  └──────────────────────────────────────────────────────┘ │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  SimpleFOC Driver                                          │ │
│  │  - Motor driver abstraction                                │ │
│  │  - Encoder interface                                       │ │
│  │  - Current sensing (optional)                              │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────┼──────────────────────────────────────┘
                           │
                    ┌──────┴──────┐
                    │  BLDC Motor  │
                    │  + Encoder   │
                    └─────────────┘
```

## Communication Protocol

### Command Structure (Binary Protocol via SerialTransfer)

**Commands (PC → ESP32):**
- `CMD_MOVE_TO (0x01)` - Move to absolute position
- `CMD_SET_SPEED (0x03)` - Set movement speed
- `CMD_SET_ACCEL (0x04)` - Set acceleration
- `CMD_STOP (0x05)` - Emergency stop
- `CMD_HOME (0x06)` - Set current position as home
- `CMD_ENABLE (0x07)` - Enable motor
- `CMD_DISABLE (0x08)` - Disable motor
- `CMD_GET_STATUS (0x09)` - Request status
- `CMD_PING (0x0A)` - Connection test
- `CMD_SET_MODE (0x0B)` - Set control mode
- `CMD_SET_CURRENT_LIMIT (0x0C)` - Set FOC current limit

**Responses (ESP32 → PC):**
- `RESP_OK (0x81)` - Command acknowledged
- `RESP_ERROR (0x82)` - Command failed
- `RESP_POSITION_REACHED (0x84)` - Target position reached
- `RESP_PING (0x86)` - Ping response

### Position Synchronization

1. PC sends `CMD_MOVE_TO` with target position and sequence ID
2. ESP32 starts movement and responds with `RESP_OK`
3. When position is reached, ESP32 sends `RESP_POSITION_REACHED` with sequence ID and actual position
4. PC triggers camera capture upon receiving position notification
5. Image is saved with sequence ID for data organization

## Motor Control Architecture

### SimpleFOC Integration

```cpp
// Motor and sensor objects
BLDCMotor motor = BLDCMotor(pole_pairs);
BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, enable);
Encoder encoder = Encoder(encoderA, encoderB, ppr);

// Control modes
- Position control: For precise grid scanning
- Velocity control: For continuous rotation
- Open-loop control: For testing without encoder
```

### Control Flow

1. **Initialization**: Setup SimpleFOC motor, driver, encoder
2. **Command Reception**: Parse SerialTransfer packets
3. **Motion Execution**: Use SimpleFOC position/velocity control
4. **Position Monitoring**: Check if target reached
5. **Notification**: Send position reached message to PC

## Directory Structure

```
OpenHyperspectral/
├── firmware/                      # ESP32 firmware
│   ├── motor_firmware/
│   │   ├── motor_firmware.ino    # Main firmware file
│   │   ├── config.h              # Hardware configuration
│   │   ├── commands.h            # Command definitions
│   │   ├── communication.h/.cpp  # Serial protocol
│   │   ├── motor_control.h/.cpp  # SimpleFOC integration
│   │   └── display.h/.cpp        # LVGL display
│   └── libraries/                # Required libraries
│       ├── SimpleFOC/
│       ├── SerialTransfer/
│       └── LVGL/
├── motor_control/                 # Python motor control
│   ├── __init__.py
│   ├── controller.py             # Motor controller class
│   ├── scanner.py                # Scanning patterns
│   └── calibration.py            # Motor calibration tools
├── camera/                        # Camera interface (existing)
│   ├── camera_ui.py
│   ├── camera_viewer.py
│   └── camera_streaming.py
├── mightex_driver/               # Camera driver (existing)
│   ├── __init__.py
│   └── camera.py
├── examples/                      # Usage examples
│   ├── basic_motor_control.py
│   ├── grid_scan.py
│   └── synchronized_capture.py
└── tests/                         # Test scripts
    ├── test_motor_communication.py
    ├── test_camera_sync.py
    └── test_full_scan.py
```

## Key Features

### From HyperMicro
- ✅ Binary serial communication protocol
- ✅ Position-based synchronization
- ✅ Grid and spiral scanning patterns
- ✅ Async position monitoring with threading
- ✅ Movement queue management
- ✅ Data organization and metadata

### From ESP32 Motor Control
- ✅ Modular driver architecture
- ✅ Display with LVGL
- ✅ Manual control modes
- ✅ Encoder input
- ✅ Sleep/wake power management
- ✅ Acceleration profiles

### New for OpenHyperspectral
- ✅ SimpleFOC BLDC motor control
- ✅ FOC current limiting
- ✅ Encoder feedback for position
- ✅ Camera + motor synchronization
- ✅ Hyperspectral data acquisition
- ✅ Real-time preview with motor position overlay

## Implementation Phases

### Phase 1: Basic Motor Control (Current Phase)
- [ ] ESP32 firmware with SimpleFOC
- [ ] Serial communication protocol
- [ ] Python motor controller
- [ ] Basic position control

### Phase 2: Display Integration
- [ ] LVGL display with motor status
- [ ] Manual control interface
- [ ] Touch/encoder input

### Phase 3: Camera Synchronization
- [ ] Position callbacks in Python
- [ ] Camera trigger on position reached
- [ ] Data organization system

### Phase 4: Advanced Scanning
- [ ] Grid scanning implementation
- [ ] Spiral scanning patterns
- [ ] Automated calibration
- [ ] Real-time preview

## Dependencies

### ESP32 Firmware
- Arduino framework
- SimpleFOC library (>= 2.3.0)
- SerialTransfer library
- LVGL (>= 8.3.0)
- ST7789 display driver

### Python Software
- pySerialTransfer
- numpy
- opencv-python (for preview)
- mightex-camera-sdk
- matplotlib (for visualization)

## Configuration

### Hardware Pin Mapping (ESP32-S3)
```cpp
// Motor driver pins (3-phase PWM)
#define MOTOR_PWM_A  GPIO_NUM_10
#define MOTOR_PWM_B  GPIO_NUM_11
#define MOTOR_PWM_C  GPIO_NUM_12
#define MOTOR_ENABLE GPIO_NUM_13

// Encoder pins
#define ENCODER_A    GPIO_NUM_14
#define ENCODER_B    GPIO_NUM_15
#define ENCODER_I    GPIO_NUM_16  // Index (optional)

// Display pins (built-in on Waveshare board)
// Already configured by Display_ST7789.h

// Serial communication
#define SERIAL_BAUD  115200
```

### Motor Configuration
```cpp
// BLDC motor parameters
#define POLE_PAIRS   7          // Motor pole pairs
#define ENCODER_PPR  2048       // Encoder pulses per revolution
#define VOLTAGE_PSU  12.0       // Power supply voltage
#define CURRENT_LIMIT 1.0       // Current limit in amps

// Motion parameters
#define MAX_VELOCITY 100.0      // Max velocity (rad/s)
#define MAX_ACCELERATION 50.0   // Max acceleration (rad/s²)
```

## Notes

- The ESP32-S3 has sufficient processing power for FOC algorithm at high frequencies
- SimpleFOC provides smooth motion control with encoder feedback
- Serial communication uses binary protocol for efficiency
- Position synchronization ensures camera captures at exact motor positions
- Display provides feedback and manual control when not connected to PC
