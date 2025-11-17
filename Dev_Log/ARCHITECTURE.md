# OpenHyperspectral Architecture

## System Overview

OpenHyperspectral is a **1D line-scanning hyperspectral imaging system** that synchronizes a monochrome camera with a single-motor scanning mechanism via USB communication. The system combines:

1. **ESP32-S3-Touch-LCD-2** (Waveshare) - Motor control with display
2. **SimpleFOC** - BLDC motor control library with DRV8313 driver
3. **MT6701** - 14-bit magnetic encoder for position feedback
4. **USB Serial Protocol** - Synchronization between PC and ESP32
5. **Mightex Monochrome Camera** - Hyperspectral line capture
6. **SpectrumBoi** - Spectrometer and camera preview UI
7. **Python Main Program** - Orchestrates scanning, capture, and processing

## Hardware Architecture

```
Main Computer
     │
     ├─── USB ───> Mightex Monochrome Camera
     │
     └─── USB ───> ESP32-S3-Touch-LCD-2
                        │
                        ├─── 3-Phase PWM + Enable ───> DRV8313 SimpleFOC Driver
                        │                                      │
                        │                                      └──> BLDC Motor
                        │
                        └─── I2C/ABZ Interface ───> MT6701 14-bit Encoder
                                                         │
                                                         └──> Motor Shaft
```

## Software Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Main Program (Python)                        │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  SpectrumBoi UI Module                                     │ │
│  │  - Spectrometer preview                                    │ │
│  │  - Camera live view                                        │ │
│  │  - Scan progress monitoring                                │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Camera Capture Module                                     │ │
│  │  - Mightex camera interface                                │ │
│  │  - Line image acquisition                                  │ │
│  │  - Triggered capture on position events                    │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Motor Control Module (controller.py)                      │ │
│  │  - USB serial communication                                │ │
│  │  - Position tracking and callbacks                         │ │
│  │  - 1D linear scan coordination                             │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Image Storage Module                                      │ │
│  │  - Data file organization                                  │ │
│  │  - Metadata management                                     │ │
│  │  - Position-indexed storage                                │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Image Processing Pipeline                                 │ │
│  │  - Line-by-line processing                                 │ │
│  │  - Real-time or batch processing                           │ │
│  │  - Data cube construction                                  │ │
│  └────────────────────────────────────────────────────────────┘ │
│                              │                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Data Cube Visualization & Analysis UI                     │ │
│  │  - 3D hyperspectral data visualization                     │ │
│  │  - Spectral analysis tools                                 │ │
│  │  - Export and reporting                                    │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────┼──────────────────────────────────────┘
                           │ USB Serial (SerialTransfer Protocol)
┌──────────────────────────┴──────────────────────────────────────┐
│              ESP32-S3-Touch-LCD-2 (Waveshare)                    │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  motor_firmware.ino                                        │ │
│  │  ┌──────────────────────────────────────────────────────┐ │ │
│  │  │  Serial Communication Module                         │ │ │
│  │  │  - Command processing                                │ │ │
│  │  │  - Position notifications                            │ │ │
│  │  │  - Status reporting                                  │ │ │
│  │  └──────────────────────────────────────────────────────┘ │ │
│  │  ┌──────────────────────────────────────────────────────┐ │ │
│  │  │  Motor Control Module (SimpleFOC)                    │ │ │
│  │  │  - FOC algorithm                                     │ │ │
│  │  │  - Position control (1D linear)                      │ │ │
│  │  │  - MT6701 encoder feedback                           │ │ │
│  │  └──────────────────────────────────────────────────────┘ │ │
│  │  ┌──────────────────────────────────────────────────────┐ │ │
│  │  │  Display Module (LVGL + ST7789)                      │ │ │
│  │  │  - Status display                                    │ │ │
│  │  │  - Manual control interface                          │ │ │
│  │  │  - Touch/encoder input                               │ │ │
│  │  └──────────────────────────────────────────────────────┘ │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────┼──────────────────────────────────────┘
                           │ 3-phase PWM
                    ┌──────┴──────┐
                    │   DRV8313   │
                    │SimpleFOC Drv│
                    └──────┬──────┘
                           │
                    ┌──────┴──────┐
                    │  BLDC Motor  │
                    │  + MT6701    │
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
BLDCMotor motor = BLDCMotor(7);  // 7 pole pairs (Mitoot 2804)
BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, enable);
MT6701Sensor encoder = MT6701Sensor(0x06);  // Custom I2C wrapper

// Control modes
- Position control: For precise 1D linear scanning
- Velocity control: For continuous scanning motion
- Torque control: For force-limited applications
```

### MT6701 I2C Calibration

**Problem**: SimpleFOC's automatic `initFOC()` calibration fails with MT6701 I2C sensors due to I2C timing (too slow for SimpleFOC's movement detection algorithm).

**Solution**: Manual calibration using `setPhaseVoltage()` at known electrical angles:

1. Apply voltage at 270° electrical → motor physically aligns
2. Wait 700ms for settling
3. Read encoder position (motor is at known electrical angle)
4. Repeat at 0° electrical
5. Calculate `zero_electric_angle` and `sensor_direction` from measurements
6. Set calibration values in SimpleFOC
7. Call `initFOC()` → skips alignment, returns success

**Why this works**:
- Uses **static fields** (not rotation) → motor doesn't oscillate
- Long settling times → motor fully stops before reading
- No movement detection required → I2C speed irrelevant

**Commands**:
- `align` - Diagnostic test (verifies motor holds positions before calibration)
- `calibrate` - Run manual calibration

### Control Flow

1. **Initialization**: Setup SimpleFOC motor, driver, encoder
2. **Manual Calibration**: Calculate FOC calibration constants
3. **Command Reception**: Parse SerialTransfer packets
4. **Motion Execution**: Use SimpleFOC position/velocity control
5. **Position Monitoring**: Check if target reached (using absolute encoder)
6. **Notification**: Send position reached message to PC

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
│   │   └── display.h/.cpp        # LVGL display (future)
│   └── COMPILATION_GUIDE.md      # Compilation instructions
├── motor_control/                 # Python motor control
│   ├── __init__.py
│   ├── controller.py             # Motor controller class
│   └── calibration.py            # Motor calibration tools (future)
├── camera/                        # Camera interface (existing)
│   ├── camera_ui.py
│   ├── camera_viewer.py
│   └── camera_streaming.py
├── mightex_driver/               # Camera driver (existing)
│   ├── __init__.py
│   └── camera.py
├── spectrumboi/                   # SpectrumBoi UI (existing)
│   └── ...
├── examples/                      # Usage examples (future)
│   ├── basic_motor_control.py
│   ├── line_scan.py
│   └── synchronized_capture.py
└── tests/                         # Test scripts (future)
    ├── test_motor_communication.py
    ├── test_camera_sync.py
    └── test_line_scan.py
```

## Key Features

### From HyperMicro
- ✅ Binary serial communication protocol
- ✅ Position-based synchronization
- ✅ Async position monitoring with threading
- ✅ Movement queue management
- ✅ Data organization and metadata

### From ESP32 Motor Control
- ✅ Modular driver architecture
- ✅ Display with LVGL (planned)
- ✅ Manual control modes (planned)
- ✅ Encoder input
- ✅ Acceleration profiles

### OpenHyperspectral Specific
- ✅ SimpleFOC BLDC motor control with DRV8313
- ✅ MT6701 14-bit encoder integration
- ✅ FOC current limiting
- ✅ 1D line-scanning motor control
- ✅ Camera + motor synchronization
- ✅ Hyperspectral data acquisition
- ✅ SpectrumBoi UI integration
- ✅ Data cube construction and visualization

## Implementation Phases

### Phase 1: Basic Motor Control ✅ (Current Phase)
- ✅ ESP32 firmware with SimpleFOC
- ✅ Serial communication protocol
- ✅ Python motor controller
- ✅ Basic position control
- ✅ MT6701 I2C encoder integration
- ✅ Manual calibration for MT6701 (bypasses SimpleFOC auto-calibration issues)

### Phase 2: Camera Synchronization
- [ ] Position callbacks in Python
- [ ] Mightex camera trigger on position reached
- [ ] 1D line scan coordination
- [ ] Data organization system

### Phase 3: SpectrumBoi UI Integration
- [ ] Motor control integration
- [ ] Live camera preview
- [ ] Scan progress visualization
- [ ] Real-time spectral preview

### Phase 4: Data Pipeline
- [ ] Image storage module
- [ ] Processing pipeline
- [ ] Data cube construction
- [ ] Visualization and analysis UI

### Phase 5: Display Integration (Optional)
- [ ] LVGL display with motor status
- [ ] Manual control interface
- [ ] Touch/encoder input

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

### Hardware Pin Mapping (ESP32-S3-Touch-LCD-2)
```cpp
// MT6701 Encoder (I2C interface - ABSOLUTE POSITION)
#define ENCODER_SDA  47         // GPIO47 - I2C data
#define ENCODER_SCL  48         // GPIO48 - I2C clock
// I2C address: 0x06 (MT6701 default)

// SimpleFOC Motor Driver (DRV8313)
#define MOTOR_EN     15         // GPIO15 - Enable
#define MOTOR_IN1    13         // GPIO13 - Phase 1 PWM
#define MOTOR_IN2    11         // GPIO11 - Phase 2 PWM
#define MOTOR_IN3    12         // GPIO12 - Phase 3 PWM

// Optional monitoring/control
#define MOTOR_FAULT  14         // GPIO14 - nFT (fault detection)
#define MOTOR_RESET  9          // GPIO9  - nRT (driver reset)

// Display (built-in on Waveshare board)
// ST7789 LCD 240×320 via SPI (managed by board support package)
// Touch: CST816D via I2C

// Serial communication (USB CDC)
#define SERIAL_BAUD  115200
```

**Pin Clustering**: Pins are physically grouped for clean wiring:
- **Encoder cluster** (top right): GPIO47/48 + 3V3 + GND
- **Motor driver cluster** (middle right): GPIO15,13,11,12,14,9 + GND

### Motor Configuration
```cpp
// BLDC motor parameters (Mitoot 2804 100kv Gimbal Motor)
#define POLE_PAIRS   7          // Motor pole pairs (Mitoot 2804: 7 pole pairs)
#define ENCODER_PPR  16384      // MT6701 14-bit resolution (2^14 = 16384 counts/rev)
#define VOLTAGE_PSU  12.0       // Power supply voltage (V)
#define CURRENT_LIMIT 1.0       // Current limit (A) - gimbal motors are low current

// Motion parameters for hyperspectral scanning
#define MAX_VELOCITY 100.0      // Max velocity (deg/s) - slow for precision
#define MAX_ACCELERATION 50.0   // Max acceleration (deg/s²)
#define POSITION_TOLERANCE 0.5  // Position reached tolerance (degrees)
```

**Motor Specifications**:
- **Type**: Mitoot 2804 100kv Brushless Gimbal Motor
- **Pole pairs**: 7 (14 magnets)
- **Resistance**: High (>10Ω) - designed for gimbal applications
- **KV rating**: 100kv (low speed, high torque)
- **Driver**: SimpleFOC Mini v1 (DRV8313-based, 2A continuous/phase)

**Encoder Specifications**:
- **Type**: MT6701 14-bit Absolute Magnetic Encoder
- **Interface**: I2C (address 0x06)
- **Resolution**: 14-bit (16384 positions/revolution = 0.022° precision)
- **Update rate**: ~50-100Hz via I2C (sufficient for gimbal motors)
- **Features**: Absolute position (no homing required), field strength monitoring

## Notes

### Hardware
- ESP32-S3 has sufficient processing power for FOC algorithm at high frequencies (~1kHz)
- DRV8313 is SimpleFOC motor driver board v1 (3-phase BLDC driver)
- MT6701 provides 14-bit absolute position sensing (can use ABZ or I2C interface)
- Waveshare ESP32-S3-Touch-LCD-2 has built-in 2.1" ST7789 LCD display

### Software
- SimpleFOC provides smooth motion control with encoder feedback
- Serial communication uses binary protocol (SerialTransfer) for efficiency
- Position synchronization ensures camera captures at exact motor positions
- SpectrumBoi UI provides real-time preview and control
- 1D line scanning builds up hyperspectral data cube line-by-line

### Scanning Strategy
- Single motor controls 1D linear scan position
- Motor moves to position → notifies PC → PC triggers camera capture
- Each position corresponds to one line in the final hyperspectral data cube
- Data cube: (spatial lines, spectral channels, wavelengths)
