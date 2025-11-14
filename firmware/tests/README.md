# OpenHyperspectral Firmware Test Suite

Comprehensive testing suite for the OpenHyperspectral motor controller firmware. Each test can be run independently to verify specific subsystems without requiring the full hardware setup.

## 📋 Test Overview

| Test Name | Purpose | Hardware Required | Duration |
|-----------|---------|-------------------|----------|
| **motor_test** | Test motor spinning (open-loop, no encoder) | Motor + Driver | ~50 seconds |
| **encoder_test** | Continuous encoder data streaming | Encoder only | Continuous |
| **i2c_scanner_test** | Scan I2C bus for devices | None | ~1 second |
| **driver_fault_test** | Test driver enable/disable & fault detection | Driver only | Interactive |
| **communication_test** | Test binary serial protocol | None | Interactive |
| **loop_timing_test** | Verify 1000 Hz control loop timing | None | Continuous |
| **integration_test** | Full system test with closed-loop control | Motor + Encoder + Driver | ~1 minute |

## 🚀 Quick Start

### Upload a Test

1. Open Arduino IDE or PlatformIO
2. Navigate to `firmware/tests/<test_name>/<test_name>.ino`
3. Select board: **ESP32-S3-Touch-LCD-2 (Waveshare)** or **ESP32-S3-DevKitC**
4. Upload the sketch
5. Open Serial Monitor at **115200 baud**
6. Follow on-screen instructions

### Example: Running Motor Test

```bash
# Using Arduino CLI
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware/tests/motor_test/motor_test.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32s3 firmware/tests/motor_test/motor_test.ino

# Open serial monitor
arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

## 📝 Detailed Test Descriptions

---

### 1. Motor Test (Open-Loop)

**File:** `motor_test/motor_test.ino`

**Purpose:** Verify motor can spin without encoder feedback using open-loop voltage control.

**What It Tests:**
- SimpleFOC driver initialization
- Motor phase connections
- Power supply connectivity
- Bidirectional rotation
- Speed control response

**Hardware Required:**
- ESP32-S3 board
- SimpleFOC Mini v1 driver (DRV8313)
- Mitoot 2804 BLDC motor
- 12V power supply

**Test Sequence:**
1. Phase 1: Slow clockwise (5 rad/s) - 10 seconds
2. Phase 2: Medium clockwise (10 rad/s) - 10 seconds
3. Phase 3: Stop - 5 seconds
4. Phase 4: Slow counter-clockwise (-5 rad/s) - 10 seconds
5. Phase 5: Medium counter-clockwise (-10 rad/s) - 10 seconds
6. Phase 6: Final stop - 5 seconds

**Expected Output:**
```
[TEST] Phase 1: Slow clockwise rotation (5 rad/s)
       You should see the motor spinning slowly clockwise
[HEARTBEAT] Uptime: 10s | Test phase: 0 | Target: 5.00 rad/s
...
```

**Troubleshooting:**
- Motor doesn't spin → Check power supply, GND connection, motor phases
- Motor stutters → Check phase wire connections, verify power supply voltage
- Driver gets hot → Reduce test voltage, check for short circuits

---

### 2. Encoder Test (Continuous Streaming)

**File:** `encoder_test/encoder_test.ino`

**Purpose:** Continuously read and stream encoder position/velocity data.

**What It Tests:**
- MT6701 encoder I2C communication
- Position accuracy (14-bit resolution)
- Velocity calculation
- Full rotation tracking
- Communication reliability

**Hardware Required:**
- ESP32-S3 board
- MT6701 14-bit magnetic encoder
- Magnet (attached to motor shaft)

**Commands:**
- `h` - Show help
- `s` - Start streaming
- `p` - Pause streaming
- `r` - Reset statistics
- `i` - Show encoder info
- `c` - Show statistics

**Output Format:**
```
[12.34s] Pos: 2.4567 rad | Vel: 1.23 rad/s | Raw: 6234 | Rev: 2
```

**Fields:**
- **Pos** - Current angle (0 to 2π radians)
- **Vel** - Angular velocity (rad/s)
- **Raw** - Raw encoder count (0-16383)
- **Rev** - Full rotations completed

**Test Procedure:**
1. Upload sketch
2. Manually rotate motor shaft slowly
3. Verify position changes smoothly
4. Rotate faster to test velocity calculation
5. Check for consistent readings

**Expected Behavior:**
- Position updates at 100 Hz
- Smooth position changes
- No sudden jumps (indicates communication errors)
- Velocity proportional to rotation speed

---

### 3. I2C Scanner Test

**File:** `i2c_scanner_test/i2c_scanner_test.ino`

**Purpose:** Scan I2C bus and identify connected devices.

**What It Tests:**
- I2C bus functionality
- Device detection
- Wiring verification
- I2C speed compatibility

**Hardware Required:**
- ESP32-S3 board
- (Optional) I2C devices to detect

**Known Devices:**
- `0x06` - MT6701 Encoder
- `0x15` - CST816D Touch Controller (on ESP32-S3-Touch-LCD-2)
- `0x6B` - QMI8658 IMU (on ESP32-S3-Touch-LCD-2)

**Commands:**
- `h` - Show help
- `s` - Scan I2C bus
- `f` - Set fast mode (400 kHz)
- `n` - Set standard mode (100 kHz)
- `i` - Show I2C configuration
- `d` - Detect specific address

**Expected Output:**
```
     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
     ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─
00:                -- 06 -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- 15 -- -- -- -- -- -- -- -- -- --
...

Detected Devices:
  • 0x06 - MT6701 (14-bit Magnetic Encoder) ✓ ENCODER DETECTED!
  • 0x15 - CST816D (Touch Controller)
```

---

### 4. Driver Fault Test

**File:** `driver_fault_test/driver_fault_test.ino`

**Purpose:** Test motor driver enable/disable and fault detection.

**What It Tests:**
- Driver enable pin functionality
- Fault pin monitoring (nFT)
- Driver reset functionality
- Over-current protection
- Driver state management

**Hardware Required:**
- ESP32-S3 board
- SimpleFOC Mini v1 driver

**Commands:**
- `h` - Show help
- `e` - Enable driver
- `d` - Disable driver
- `r` - Reset driver (pulse nRT pin)
- `s` - Show driver status
- `f` - Check fault pin
- `a` - Run automatic test sequence

**Test Sequence (Auto Mode):**
1. Disable driver
2. Enable driver
3. Disable again
4. Reset driver
5. Enable after reset
6. Check fault status

**Fault Detection:**
The test continuously monitors the nFT pin (active LOW) and will alert if:
- Overcurrent condition
- Short circuit
- Over-temperature
- Supply voltage issue

---

### 5. Communication Test

**File:** `communication_test/communication_test.ino`

**Purpose:** Test binary serial protocol (SerialTransfer library).

**What It Tests:**
- SerialTransfer packet integrity
- PING/PONG communication
- Command/response handling
- Communication latency
- Error handling

**Hardware Required:**
- ESP32-S3 board
- USB connection to PC

**How to Test:**

**Option 1: Using Python Controller**
```bash
cd motor_control
python controller.py /dev/ttyUSB0
```

**Option 2: Manual Testing**
Send binary PING command:
- Command ID: `0x0A`
- Payload: 4-byte echo value
- Expected: PING response with same echo value

**Statistics:**
- Packets received/sent
- Error count
- Average latency (microseconds)
- Success rate

**Commands:**
- `s` - Show detailed statistics

---

### 6. Loop Timing Test

**File:** `loop_timing_test/loop_timing_test.ino`

**Purpose:** Verify 1000 Hz control loop maintains stable timing.

**What It Tests:**
- Loop frequency accuracy
- Timing jitter
- CPU utilization
- Timing violations
- Performance under load

**Hardware Required:**
- ESP32-S3 board

**Target Performance:**
- **Frequency:** 1000 Hz
- **Period:** 1000 μs (±50 μs acceptable)
- **CPU Usage:** <50%
- **Violations:** <1%

**Commands:**
- `h` - Show help
- `s` - Show detailed statistics
- `r` - Reset statistics
- `d` - Show timing distribution
- `i` - Show system information

**Output:**
```
[HEARTBEAT] Loops: 10000 | Avg: 123.4 μs | CPU: 12.3% | Violations: 0.05%
```

**Timing Distribution:**
Shows histogram of loop execution times to identify timing anomalies.

**What Good Results Look Like:**
- Average loop time: 100-200 μs
- CPU usage: <30%
- Timing violations: <0.1%
- Consistent timing (low jitter)

---

### 7. Integration Test (Full System)

**File:** `integration_test/integration_test.ino`

**Purpose:** Test complete system with motor + encoder + closed-loop control.

**What It Tests:**
- Motor calibration
- Closed-loop position control
- FOC algorithm performance
- Positioning accuracy
- System integration

**Hardware Required:**
- ESP32-S3 board
- SimpleFOC Mini v1 driver
- Mitoot 2804 BLDC motor
- MT6701 encoder
- 12V power supply

**Test Procedure:**

1. **Upload and Start**
   ```
   Upload sketch → Open Serial Monitor (115200 baud)
   ```

2. **Calibrate Motor**
   ```
   Type: c
   Motor will spin during calibration (10-20 seconds)
   Wait for "CALIBRATION SUCCESSFUL!"
   ```

3. **Run Automated Test**
   ```
   Type: t
   Tests 7 positions: 0, π/2, π, 3π/2, 2π, π, 0
   Each position tested for accuracy and settle time
   ```

4. **Review Results**
   ```
   Type: r
   Shows pass/fail for each test
   Average error, max error, settle times
   ```

**Commands:**
- `h` - Show help
- `c` - Run calibration (required before testing)
- `e` - Enable motor
- `d` - Disable motor
- `t` - Run automated test sequence
- `s` - Show current status
- `r` - Show test results
- `m <angle>` - Move to specific angle (rad)

**Test Success Criteria:**
- All positions reached within 5 seconds
- Position error < 0.1 rad (~5.7°)
- No oscillation or instability
- Smooth motion between positions

**Example Output:**
```
Test | Target (rad) | Actual (rad) | Error (rad) | Time (ms) | Result
──────────────────────────────────────────────────────────────────────
  1  |    0.00     |    0.01     |   0.0100   |   1234   | PASS
  2  |    1.57     |    1.58     |   0.0050   |   2345   | PASS
  3  |    3.14     |    3.15     |   0.0080   |   2567   | PASS
...

Tests Passed: 7 / 7
Average Error: 0.0067 rad (0.38 deg)
Maximum Error: 0.0100 rad (0.57 deg)
```

---

## 🔧 Troubleshooting

### Common Issues

**1. Upload Failed**
```
Error: Failed to connect to ESP32
```
**Solution:**
- Hold BOOT button while uploading
- Check USB cable connection
- Verify correct COM port selected
- Try different USB port

**2. Encoder Not Detected**
```
[ERROR] MT6701 encoder not found!
```
**Solution:**
- Check wiring: 3.3V, GND, SDA (GPIO47), SCL (GPIO48)
- Verify encoder has power (measure 3.3V at VDD pin)
- Try different I2C speed (send 'n' for standard mode)
- Check for loose connections

**3. Motor Doesn't Spin**
```
Motor enabled but no rotation
```
**Solution:**
- Check power supply is on and providing 12V
- Verify GND connection between ESP32 and driver
- Check motor phase wires are connected
- Verify driver LED is on
- Try motor_test first (open-loop)

**4. Timing Violations**
```
[WARNING] Timing violation: 1500 μs (500 μs over target)
```
**Solution:**
- Reduce debug output (set DEBUG_SERIAL to false)
- Close serial monitor (it adds overhead)
- Check CPU frequency is 240 MHz
- Disable other running tasks

**5. Communication Errors**
```
[ERROR] SerialTransfer error: -1
```
**Solution:**
- Check baud rate is 115200
- Close other programs using serial port
- Try different USB cable (some are charge-only)
- Reset ESP32 and reconnect

---

## 📊 Interpreting Results

### Motor Test
✅ **Good:** Smooth rotation in both directions, consistent speed
❌ **Bad:** Stuttering, no rotation, overheating

### Encoder Test
✅ **Good:** Smooth position changes, velocity matches rotation speed
❌ **Bad:** Jumps in position, communication errors, stuck readings

### Integration Test
✅ **Good:** <0.1 rad error, <3s settle time, no oscillation
❌ **Bad:** Large errors, timeout, oscillation, instability

---

## 🎯 Recommended Test Sequence

For a new hardware setup, run tests in this order:

1. **i2c_scanner_test** - Verify encoder is detected
2. **driver_fault_test** - Verify driver can be enabled
3. **encoder_test** - Verify encoder reads correctly
4. **motor_test** - Verify motor can spin (open-loop)
5. **loop_timing_test** - Verify control loop timing
6. **integration_test** - Full system test
7. **communication_test** - Test PC communication (optional)

---

## 🔬 Advanced Testing

### Performance Benchmarking

Run `loop_timing_test` for extended periods to measure stability:
```bash
# Run for 1 hour, log output
arduino-cli monitor -p /dev/ttyUSB0 > timing_log.txt
```

### Endurance Testing

Run `integration_test` repeatedly to test reliability:
```arduino
// Modify integration_test.ino loop():
if (calibrated && millis() % 60000 == 0) {
    runTestSequence();  // Run every minute
}
```

### Custom Test Positions

Modify `integration_test.ino` to test specific positions:
```arduino
float test_positions[] = {0.0, 0.5, 1.0, 1.5, 2.0};  // Custom angles
```

---

## 📞 Support

If tests fail consistently:

1. Check **hardware_primer.txt** in `references/` folder
2. Review **motor_firmware/config.h** for correct pin assignments
3. Verify power supply voltage (should be 12V)
4. Check motor resistance (~10Ω for Mitoot 2804)
5. Ensure encoder magnet is properly aligned (2-3mm from sensor)

For issues or questions, refer to:
- Main firmware: `firmware/motor_firmware/`
- Configuration: `firmware/motor_firmware/config.h`
- Hardware reference: `references/hardware_primer.txt`

---

## 📝 Test Development

### Adding New Tests

To create a new test:

1. Create folder: `firmware/tests/my_test/`
2. Create sketch: `firmware/tests/my_test/my_test.ino`
3. Use this template:

```arduino
#include <Arduino.h>

#define HEARTBEAT_INTERVAL 10000

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) delay(10);

    Serial.println("My Test - Starting...");
    // Your initialization
}

void loop() {
    // Your test logic

    // Heartbeat every 10 seconds
    static unsigned long last_heartbeat = 0;
    if (millis() - last_heartbeat >= HEARTBEAT_INTERVAL) {
        last_heartbeat = millis();
        Serial.println("[HEARTBEAT] Test running...");
    }
}
```

4. Update this README with test description

---

## ✅ Test Checklist

Use this checklist to verify hardware:

- [ ] I2C scanner detects encoder (0x06)
- [ ] Driver can be enabled/disabled without faults
- [ ] Encoder position changes when shaft rotated
- [ ] Motor spins in open-loop mode
- [ ] Control loop maintains 1000 Hz
- [ ] Motor calibration completes successfully
- [ ] Closed-loop position control works
- [ ] Position error < 0.1 rad
- [ ] No timing violations during operation
- [ ] Binary protocol communication works

If all tests pass, your system is ready for normal operation! 🎉

---

**Last Updated:** 2025-11-14
**Firmware Version:** 1.0.0
**Board:** ESP32-S3-Touch-LCD-2 (Waveshare)
