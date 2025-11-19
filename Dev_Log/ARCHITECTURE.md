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

1. **Diagnostic test** - Apply voltage at 4 electrical angles (0°, 90°, 180°, 270°) to verify motor response
2. **Calibration** - Use 90° and 270° electrical positions (best mechanical separation)
3. Wait 700ms at each position for settling
4. Read encoder position at each angle
5. Validate motor moved between calibration points
6. Calculate `zero_electric_angle` and `sensor_direction` from measurements
7. Set calibration values in SimpleFOC
8. Call `initFOC()` → skips alignment, returns success

**Why this works**:
- Uses **static fields** (not rotation) → motor doesn't oscillate
- Long settling times → motor fully stops before reading
- No movement detection required → I2C speed irrelevant
- Uses angles with maximum mechanical separation (~336° for Mitoot 2804)

**Commands**:
- `c` / `calibrate` - Run unified calibration (includes diagnostic + calibration)
- `align` - Standalone diagnostic test (4 positions, verifies motor response)
- `phase_test` - Hardware diagnostic (6 positions, tests all three motor phases)

**Calibration Flow**:
```
Step 1: Diagnostic Test
  → Test motor at 0°, 90°, 180°, 270° electrical
  → Verify motor responds to all positions

Step 2: Calculate Calibration Values
  → Use 90° and 270° positions (best separation)
  → Validate movement occurred
  → Calculate zero_electric_angle and sensor_direction

Step 3: Initialize FOC
  → Call initFOC() with preset values
  → Motor ready for operation
```

### Hardware Diagnostics & Troubleshooting

**Phase Test System** (`phase_test` command):
- Tests motor at 6 electrical angles (0°, 60°, 120°, 180°, 240°, 300°)
- Verifies all three motor phases (A, B, C) are functional
- Monitors driver fault pin (nFT) throughout test
- Identifies specific phase failures

**Expected Results** (with functional hardware):
- Motor moves to 6 distinct mechanical positions
- Each position ~8.6° apart (60° electrical ÷ 7 pole pairs)
- Fault pin remains HIGH (OK) throughout

**Failure Patterns**:
- **Only 2-3 positions**: One motor phase not working → Check wiring, driver output, motor winding
- **No movement**: Driver not powered, enable pin failed, or all phases disconnected
- **Fault pin LOW**: Overcurrent, thermal shutdown, or power supply issue

**Common Issues**:
1. **Motor winding damage**: Caused by mounting screws too long puncturing windings
   - Symptom: Only 2-3 distinct positions in phase test
   - Diagnosis: Measure phase-to-phase resistance (should be ~10Ω for Mitoot 2804)
   - Solution: Replace motor, verify screw length before assembly

2. **Driver phase failure**: One DRV8313 output channel not working
   - Symptom: Failure pattern stays with GPIO when motor wires swapped
   - Diagnosis: Measure voltage at driver outputs with multimeter
   - Solution: Replace driver or use different GPIO pins

3. **Wiring issues**: Loose connection or broken wire
   - Symptom: Intermittent failures, works sometimes
   - Diagnosis: Wiggle wires during phase test
   - Solution: Re-seat connections, check continuity

**Troubleshooting Workflow**:
```
1. Run phase_test
2. If failures detected:
   a. Note which angles fail
   b. Swap motor wires M1 ↔ M2
   c. Run phase_test again
   d. If failure moves → motor issue
      If failure stays → driver/GPIO issue
3. Measure resistances and voltages
4. Replace faulty component
```

### MCU Control Flow

1. **Initialization**: Setup SimpleFOC motor, driver, encoder
2. **Manual Calibration**: Calculate FOC calibration constants
3. **Command Reception**: Parse SerialTransfer packets
4. **Motion Execution**: Use SimpleFOC position/velocity control
5. **Position Monitoring**: Check if target reached (using absolute encoder)
6. **Notification**: Send position reached message to PC

## PC-Side Motor Controller Architecture

### Overview

The PC-side controller implements a **stop-and-verify workflow** optimized for hyperspectral imaging where position accuracy is critical and settling time >> I2C read time.

### Core Workflow

```python
For each scan position:
    1. Send move command to MCU
    2. Wait for movement completion
    3. Verify position via encoder (retry if error > tolerance)
    4. Wait for settling (vibration damping)
    5. Trigger camera capture
    6. Store image with position metadata
    7. Continue to next position
```

### Position Verification Strategy

**Why encoder-aware scanning matters**:
- SimpleFOC's `shaft_angle` may drift from actual encoder position
- I2C encoder provides absolute position (no cumulative error)
- Each capture must be verified against encoder, not just motor controller state

**Move-and-Verify Implementation**:
```python
def move_and_verify(target_angle, tolerance=0.5, max_retries=3):
    for attempt in range(max_retries):
        # Send move command to MCU
        mcu.move_to_position(target_angle)

        # Wait for MCU to report movement complete
        wait_for_movement_complete()

        # Verify actual encoder position
        actual_angle = mcu.get_encoder_position()
        error = abs(actual_angle - target_angle)

        if error <= tolerance:
            return actual_angle  # Success

        # Position error - retry
        log_warning(f"Position error {error}°, retrying")

    raise PositionError(f"Failed to reach {target_angle}°")
```

### Scan Session Management

**Metadata Per Image**:
- Commanded angle (target position)
- Actual encoder angle (verified position)
- Position error (difference)
- Timestamp
- Image index in scan
- Exposure settings

**Error Recovery**:
- Position errors → Retry move up to N times
- Communication timeout → Reconnect and resume
- Motor fault → Emergency stop, report to user
- Camera failure → Skip frame or abort scan

**Workflow Configuration**:
```python
class ScanConfig:
    position_tolerance_deg = 0.5      # Max position error
    settling_time_ms = 500            # Vibration damping wait
    max_move_retries = 3              # Retry attempts

    # Scan parameters
    start_angle = 0.0
    end_angle = 360.0
    increment = 0.5                   # Angular increment per capture
```

**See `Dev_Log/system_architecture_scope.md`** for complete PC controller design and implementation details.

## Directory Structure

```
OpenHyperspectral/
├── firmware/                           # ESP32 firmware
│   └── ESP32_MCU_Firmware/
│       ├── ESP32_MCU_Firmware.ino     # Main firmware file
│       ├── config.h                    # Hardware configuration
│       ├── commands.h                  # Command definitions
│       ├── communication.h/.cpp        # Serial protocol
│       ├── motor_control.h/.cpp        # SimpleFOC integration & calibration
│       ├── tests.h/.cpp                # Test & diagnostic system
│       ├── encoder.h/.cpp              # MT6701 encoder wrapper
│       └── display.h/.cpp              # LVGL display (future)
├── Dev_Log/                            # Development documentation
│   ├── dev_log_1.md                    # Initial SimpleFOC integration
│   ├── dev_log_2.md                    # Motor diagnostics & hardware issue
│   ├── system_architecture_scope.md    # PC controller & test flow scope
│   ├── ARCHITECTURE.md                 # This file
│   └── SIMPLEFOC_DIAGNOSTIC.md         # SimpleFOC troubleshooting notes
├── motor_control/                      # Python motor control (planned)
│   ├── __init__.py
│   ├── controller.py                   # Motor controller class
│   └── scan.py                         # Scan session management
├── camera/                             # Camera interface (existing)
│   ├── camera_ui.py
│   ├── camera_viewer.py
│   └── camera_streaming.py
├── mightex_driver/                     # Camera driver (existing)
│   ├── __init__.py
│   └── camera.py
├── spectrumboi/                        # SpectrumBoi UI (existing)
│   └── ...
├── examples/                           # Usage examples (future)
│   ├── basic_motor_control.py
│   ├── line_scan.py
│   └── synchronized_capture.py
└── tests/                              # Python test scripts (future)
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

### Phase 1: MCU Firmware & Calibration ✅ (Complete - Blocked on Hardware)
- ✅ ESP32 firmware with SimpleFOC
- ✅ Serial communication protocol (binary + ASCII)
- ✅ Basic position control
- ✅ MT6701 I2C encoder integration
- ✅ Manual calibration for MT6701 (bypasses SimpleFOC auto-calibration issues)
- ✅ Unified calibration workflow (diagnostic + calibration in one command)
- ✅ Phase test diagnostics (hardware validation)
- ✅ Comprehensive test system (tests.h/tests.cpp)
- ⏸️ **Blocked**: Waiting for replacement motor (current motor damaged by mounting screws)

**Next**: Implement modular test flow system (TestResult-based architecture)

### Phase 2: PC-Side Motor Controller 🔄 (In Planning)
- [ ] `HyperspectralController` class implementation
- [ ] Move-and-verify workflow with encoder confirmation
- [ ] Position error detection and retry logic
- [ ] Serial communication (integrate with MCU binary protocol)
- [ ] Scan session management and metadata tracking
- [ ] Error handling and recovery strategies

**Scope**: See `Dev_Log/system_architecture_scope.md` for detailed design

### Phase 3: Camera Synchronization
- [ ] Camera trigger integration (hardware or software)
- [ ] Position-based capture triggering
- [ ] Image + position metadata association
- [ ] 1D line scan coordination
- [ ] Settling time and vibration handling
- [ ] Data organization system

### Phase 4: MCU Test Flow System
- [ ] Implement `TestResult` structure
- [ ] Convert existing tests to standardized format
- [ ] Implement `runCalibrationFlow()` decision tree
- [ ] Auto-diagnostic triggering on test failures
- [ ] PID auto-tuning integration
- [ ] Position control validation

**Scope**: See `Dev_Log/system_architecture_scope.md` for detailed design

### Phase 5: SpectrumBoi UI Integration
- [ ] Motor control integration
- [ ] Live camera preview
- [ ] Scan progress visualization
- [ ] Real-time spectral preview

### Phase 6: Data Pipeline
- [ ] Image storage module
- [ ] Processing pipeline
- [ ] Data cube construction
- [ ] Visualization and analysis UI

### Phase 7: Display Integration (Optional)
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

## Current Status

### Hardware Validation (as of 2025-11-19)

**Confirmed Working:**
- ✅ ESP32-S3 GPIO outputs (all motor control pins functional)
- ✅ DRV8313 driver (all 3 phase outputs operational)
- ✅ MT6701 encoder (I2C communication, absolute position reading)
- ✅ Driver fault monitoring (nFT pin on GPIO14)
- ✅ Power supply and enable circuitry

**Current Issue:**
- ❌ Mitoot 2804 motor windings damaged
  - Cause: M2.5 mounting screws ~1mm too long, punctured copper windings
  - Diagnosis: Phase test showed only 2-3 positions (phase failure pattern)
  - Wire swap test confirmed motor issue (not driver)
  - Resistance test: Open circuit between all phase pairs
  - **Action**: Replacement motor ordered

**Hardware Checklist for Motor Replacement:**
1. Measure mounting hole depth before selecting screws
2. Use screws at least 1-2mm shorter than hole depth
3. Run `phase_test` immediately after installation to verify all phases
4. Run `calibrate` to complete FOC setup
5. Proceed with PC controller integration

### Development Readiness

**MCU Firmware:** Ready for testing (blocked on replacement motor)
- Calibration system complete and validated
- Phase diagnostics implemented
- Test system framework in place

**PC Controller:** Design complete, ready for implementation
- Architecture scoped in `system_architecture_scope.md`
- Stop-and-verify workflow defined
- Error recovery strategies planned

**Next Development Phase:** Implement modular test flow system while waiting for hardware

## Lessons Learned

### Hardware Assembly Best Practices

1. **Always verify screw length before assembly**
   - Measure mounting hole depth with calipers
   - Select screws at least 1-2mm shorter
   - Better too short than risking component damage

2. **Test immediately after hardware changes**
   - Run diagnostics after any mechanical assembly
   - Catch issues before they cascade
   - Phase test is quick (<1 min) and comprehensive

3. **Systematic troubleshooting pays off**
   - Phase test → Identify symptom (only 2 positions)
   - Wire swap → Isolate component (motor vs driver)
   - Resistance test → Confirm damage
   - Physical inspection → Find root cause

### Software Architecture Insights

4. **Comprehensive diagnostics are essential**
   - Simple alignment test showed symptoms
   - Detailed phase test isolated root cause
   - Fault monitoring ruled out driver issues
   - Modular tests enable rapid diagnosis

5. **Manual calibration works well**
   - Using angles with good mechanical separation critical
   - 90° and 270° electrical worked when 0° and 270° didn't
   - Motor orientation affects which angles have best separation

6. **Absolute encoders enable robust workflows**
   - No homing required (absolute position on power-up)
   - Position verification catches drift/errors
   - Essential for stop-and-verify scanning

## Notes

### Hardware
- ESP32-S3 has sufficient processing power for FOC algorithm at high frequencies (~1kHz)
- DRV8313 is SimpleFOC motor driver board v1 (3-phase BLDC driver)
- MT6701 provides 14-bit absolute position sensing (can use ABZ or I2C interface)
- Waveshare ESP32-S3-Touch-LCD-2 has built-in 2.1" ST7789 LCD display
- **IMPORTANT**: Always verify mounting screw length to avoid damaging motor windings

### Software
- SimpleFOC provides smooth motion control with encoder feedback
- Serial communication uses binary protocol (SerialTransfer) for efficiency
- Position synchronization ensures camera captures at exact motor positions
- SpectrumBoi UI provides real-time preview and control
- 1D line scanning builds up hyperspectral data cube line-by-line
- Comprehensive diagnostic system enables rapid hardware troubleshooting

### Scanning Strategy
- Single motor controls 1D linear scan position
- Motor moves to position → notifies PC → PC triggers camera capture
- Each position corresponds to one line in the final hyperspectral data cube
- Data cube: (spatial lines, spectral channels, wavelengths)
- Stop-and-verify workflow ensures position accuracy via encoder confirmation
