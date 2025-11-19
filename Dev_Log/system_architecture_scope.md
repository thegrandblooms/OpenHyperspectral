# OpenHyperspectral System Architecture Scope

**Date:** 2025-11-19
**Purpose:** Define architecture for PC-side motor controller and MCU-side calibration/test system
**Status:** Planning Phase

---

## System Overview

The OpenHyperspectral system consists of two interconnected subsystems:

1. **MCU Firmware (ESP32-S3):** Real-time motor control, encoder reading, calibration/diagnostics
2. **PC Controller (Python):** High-level scan orchestration, image acquisition, data management

This document scopes the responsibilities, interfaces, and implementation strategy for both.

---

# Part 1: PC-Side Motor Controller (controller.py)

## Purpose

The PC-side controller orchestrates hyperspectral scanning by coordinating motor movements with camera captures. It operates in a **stop-and-verify workflow** where position accuracy is critical.

## Core Responsibilities

### 1. Scan Planning & Execution

**Scan Definition:**
- Define angular scan range (start_angle, end_angle)
- Set angular increment (e.g., 0.5° per capture)
- Calculate total number of capture points
- Generate scan trajectory

**Execution Flow:**
```
For each scan position:
    1. Send move command to MCU
    2. Wait for movement completion
    3. Verify position reached target (encoder confirmation)
    4. Trigger camera capture
    5. Wait for image acquisition
    6. Store image with position metadata
    7. Repeat until scan complete
```

### 2. Position Awareness & Verification

**Encoder Monitoring:**
- Request encoder position from MCU before/after each move
- Compare commanded position vs. actual encoder reading
- Detect position errors exceeding tolerance (e.g., ±0.5°)
- Log position deltas for each capture

**Error Handling:**
- If position error detected → retry movement
- If repeated failures → abort scan and report error
- Track position drift over full scan duration

**Position Metadata:**
- Store commanded angle, actual encoder angle, timestamp per image
- Enable post-processing position correction if needed

### 3. Communication Protocol

**Serial/USB Interface:**
- Binary protocol via SerialTransfer library (already implemented in MCU)
- ASCII command fallback for debugging
- Timeout handling for communication failures

**Key Commands to MCU:**
- `MOVE_TO_POSITION(angle)` - Absolute position command
- `GET_ENCODER_POSITION()` - Request current encoder reading
- `GET_STATUS()` - Query motor state (IDLE/MOVING/ERROR)
- `ENABLE_MOTOR()` / `DISABLE_MOTOR()` - Motor control
- `EMERGENCY_STOP()` - Immediate halt

**Expected Responses:**
- Position acknowledgments with encoder readings
- Movement completion notifications
- Error codes if move fails

### 4. Camera Integration

**Trigger Interface:**
- GPIO trigger to camera (if hardware trigger supported)
- Software trigger via camera API (SDK)
- Configurable pre-capture delay for vibration settling

**Image Management:**
- Capture image at each scan position
- Save with metadata: `image_{angle}_{timestamp}.tif`
- Associate angle/position with each frame

**Timing:**
- Account for camera exposure time
- Account for image transfer time
- Ensure motor is stationary before exposure

### 5. Data Management

**Scan Session:**
```python
class ScanSession:
    - scan_id: Unique identifier
    - start_time: Timestamp
    - parameters: Scan range, increment, etc.
    - images: List of captured images with metadata
    - positions: Commanded vs. actual position log
    - errors: Any position/movement errors encountered
```

**Metadata Per Image:**
```json
{
    "image_index": 0,
    "commanded_angle": 90.0,
    "encoder_angle": 90.12,
    "position_error": 0.12,
    "timestamp": "2025-11-19T10:30:45.123",
    "exposure_time_ms": 100,
    "settled": true
}
```

### 6. Scan Workflow Example

```python
class HyperspectralController:
    def run_scan(self, start_angle, end_angle, increment):
        """Execute a hyperspectral scan"""

        # 1. Initialize
        self.mcu.enable_motor()
        scan_session = ScanSession(start_angle, end_angle, increment)

        # 2. Move to start position
        self.move_and_verify(start_angle)

        # 3. Execute scan
        current_angle = start_angle
        while current_angle <= end_angle:
            # Move to position
            self.move_and_verify(current_angle)

            # Wait for settling (vibration damping)
            time.sleep(self.settling_time)

            # Verify position again after settling
            encoder_pos = self.mcu.get_encoder_position()
            position_error = abs(encoder_pos - current_angle)

            if position_error > self.tolerance:
                # Retry move
                self.move_and_verify(current_angle)
                encoder_pos = self.mcu.get_encoder_position()
                position_error = abs(encoder_pos - current_angle)

                if position_error > self.tolerance:
                    raise ScanError(f"Position error {position_error}° exceeds tolerance")

            # Capture image
            image = self.camera.capture()
            metadata = {
                'commanded': current_angle,
                'actual': encoder_pos,
                'error': position_error,
                'timestamp': time.time()
            }
            scan_session.add_image(image, metadata)

            # Next position
            current_angle += increment

        # 4. Complete scan
        self.mcu.disable_motor()
        scan_session.save()
        return scan_session

    def move_and_verify(self, target_angle, max_retries=3):
        """Move to angle and verify position reached"""
        for attempt in range(max_retries):
            # Send move command
            self.mcu.move_to_position(target_angle)

            # Wait for movement completion
            while self.mcu.get_status() == 'MOVING':
                time.sleep(0.1)

            # Verify position
            actual_angle = self.mcu.get_encoder_position()
            error = abs(actual_angle - target_angle)

            if error <= self.tolerance:
                return actual_angle

            # Retry if position error too large
            logging.warning(f"Position error {error}°, retrying (attempt {attempt+1})")

        raise PositionError(f"Failed to reach {target_angle}° after {max_retries} attempts")
```

## Configuration Parameters

```python
class ScanConfig:
    # Position control
    position_tolerance_deg = 0.5      # Max acceptable position error
    settling_time_ms = 500            # Wait time after move for vibrations to damp
    max_move_retries = 3              # Retry attempts for failed moves

    # Communication
    serial_port = "/dev/ttyUSB0"      # MCU serial port
    serial_baud = 115200              # Baud rate
    command_timeout_ms = 5000         # Timeout for MCU responses

    # Camera
    camera_exposure_ms = 100          # Camera exposure time
    pre_capture_delay_ms = 100        # Additional delay before capture

    # Scan defaults
    default_scan_increment_deg = 0.5  # Default angular increment
    default_scan_range = (0, 360)     # Default full rotation
```

## Error Handling & Recovery

**Error Categories:**
1. **Communication errors** - MCU not responding
2. **Position errors** - Motor can't reach target position
3. **Motor errors** - Fault condition, overcurrent, etc.
4. **Camera errors** - Capture failed, trigger timeout

**Recovery Strategies:**
- Communication errors → Retry with exponential backoff, reconnect if needed
- Position errors → Retry move, reduce velocity if needed, abort if persistent
- Motor errors → Emergency stop, report to user, require manual intervention
- Camera errors → Retry capture, skip frame if critical

## Dependencies

**Python Libraries:**
- `pyserial` - Serial communication with MCU
- `numpy` - Position calculations, data handling
- Camera SDK (manufacturer-specific)
- `json` - Metadata serialization
- `logging` - Error and event logging

**Existing Code:**
- Reference `controller.py` for initial implementation
- Integrate with existing SerialTransfer protocol in MCU firmware

---

# Part 2: MCU Calibration & Test Flow System

## Purpose

Implement a **modular, decision-tree based calibration and testing system** that automatically diagnoses issues, runs appropriate tests, and guides the user to a functional motor system.

## Design Principles

1. **Modular** - Each test/calibration is an independent function
2. **Composable** - Tests can be chained in different sequences
3. **Self-diagnosing** - Failed tests trigger appropriate diagnostics
4. **Clear reporting** - Pass/fail status with actionable next steps
5. **Reconfigurable** - Easy to add/remove/reorder tests

## Test Architecture

### Test Function Signature

All tests follow a standard interface:

```cpp
struct TestResult {
    bool passed;                    // Did test pass?
    String test_name;               // Name of test
    String message;                 // Human-readable result
    uint8_t error_code;             // Error code if failed (0 = pass)
    float* data;                    // Optional test data (measurements, etc.)
};

typedef TestResult (*TestFunction)(MotorController&);
```

### Test Registry

```cpp
struct TestDefinition {
    const char* name;               // Test name
    const char* description;        // What this test does
    TestFunction function;          // Function to execute
    TestDefinition* on_pass;        // Next test if passed
    TestDefinition* on_fail;        // Next test if failed (diagnostic)
};
```

## Calibration/Test Decision Tree

```
START
  ↓
[1] Encoder Communication Test
  ├─ PASS → [2]
  └─ FAIL → [D1] I2C Scan Diagnostic → ABORT

[2] Manual Motor Calibration (includes 4-position alignment test)
  ├─ PASS → [3]
  └─ FAIL → [D2] Phase Test Diagnostic
             ├─ All phases OK → [D3] Encoder alignment check → Report
             └─ Phase failure → Report broken phase → ABORT

[3] PID Auto-Tuning
  ├─ PASS → [4]
  └─ FAIL → [D4] Reduce velocity/acceleration, retry → [3] or Report

[4] Position Control Validation
  ├─ PASS → COMPLETE ✓
  └─ FAIL → [D5] Check PID gains, retry → [3]

COMPLETE: Motor system ready for operation
```

## Test Modules

### Core Tests

#### 1. Encoder Communication Test
```cpp
TestResult testEncoderCommunication(MotorController& mc) {
    // Check I2C communication with MT6701
    // Read encoder position
    // Verify reasonable value (0-360°)
    // Check field status (magnetic field OK)

    if (encoder_readable && field_good) {
        return {true, "Encoder Communication", "MT6701 responding, field OK", 0};
    } else {
        return {false, "Encoder Communication", "Cannot read encoder", ERR_ENCODER_COMM};
    }
}
```

#### 2. Manual Motor Calibration
```cpp
TestResult testMotorCalibration(MotorController& mc) {
    // This is our existing runManualCalibration()
    // Already includes 4-position diagnostic test
    // Calculates zero_electric_angle and sensor_direction
    // Calls initFOC()

    bool success = mc.runManualCalibration();

    if (success) {
        return {true, "Motor Calibration", "FOC initialized successfully", 0};
    } else {
        return {false, "Motor Calibration", "Calibration failed", ERR_CALIBRATION_FAILED};
    }
}
```

#### 3. PID Auto-Tuning
```cpp
TestResult testPIDTuning(MotorController& mc) {
    // Run PID auto-tuning
    // Test motor response
    // Find optimal P, I, D gains

    bool success = mc.autoTunePID(true);  // Already implemented

    if (success) {
        return {true, "PID Tuning", "Optimal PID parameters found", 0};
    } else {
        return {false, "PID Tuning", "Auto-tune failed or unstable", ERR_PID_TUNING};
    }
}
```

#### 4. Position Control Validation
```cpp
TestResult testPositionControl(MotorController& mc) {
    // Enable motor
    // Move to 3 test positions (90°, 180°, 270°)
    // Verify position reached within tolerance
    // Check velocity control, settling time

    mc.enable();
    float test_positions[] = {90.0, 180.0, 270.0};

    for (float target : test_positions) {
        mc.moveToPosition(target);
        delay(3000);  // Wait for movement

        float actual = mc.getEncoderDegrees();
        float error = abs(actual - target);

        if (error > POSITION_TOLERANCE_DEG) {
            return {false, "Position Control", "Position error too large", ERR_POSITION_ERROR};
        }
    }

    mc.disable();
    return {true, "Position Control", "Position accuracy validated", 0};
}
```

### Diagnostic Tests

#### D1. I2C Scan Diagnostic
```cpp
TestResult diagnosticI2CScan(MotorController& mc) {
    // Scan I2C bus
    // Report all devices found
    // Check if MT6701 (0x06) present

    // Use existing scanI2C() function
    scanI2C();

    return {false, "I2C Diagnostic", "Check I2C scan output above", ERR_I2C_NO_ENCODER};
}
```

#### D2. Phase Test Diagnostic
```cpp
TestResult diagnosticPhaseTest(MotorController& mc) {
    // Run our new testDriverPhases() function
    // Identify which phase(s) failed
    // Report specific failure

    mc.testDriverPhases();  // Already implemented

    // Analyze results and return
    return {false, "Phase Diagnostic", "See phase test results above", ERR_PHASE_FAILURE};
}
```

#### D3. Encoder Alignment Check
```cpp
TestResult diagnosticEncoderAlignment(MotorController& mc) {
    // Apply voltage at known angle
    // Read encoder position
    // Check if encoder moves when motor moves
    // Verify encoder mounted correctly on shaft

    mc.motor.enable();
    mc.motor.setPhaseVoltage(6.0, 0, 0);
    delay(1000);
    float pos1 = mc.getEncoderDegrees();

    mc.motor.setPhaseVoltage(6.0, 0, PI);
    delay(1000);
    float pos2 = mc.getEncoderDegrees();

    mc.motor.disable();

    float movement = abs(pos2 - pos1);
    if (movement < 5.0) {
        return {false, "Encoder Alignment", "Encoder not tracking motor movement", ERR_ENCODER_NOT_TRACKING};
    } else {
        return {true, "Encoder Alignment", "Encoder tracking motor correctly", 0};
    }
}
```

## Implementation in tests.cpp / tests.h

### tests.h
```cpp
//=============================================================================
// TEST SYSTEM
//=============================================================================

// Test result structure
struct TestResult {
    bool passed;
    String test_name;
    String message;
    uint8_t error_code;
};

// Error codes
#define ERR_NONE                    0
#define ERR_ENCODER_COMM            1
#define ERR_I2C_NO_ENCODER          2
#define ERR_CALIBRATION_FAILED      3
#define ERR_PHASE_FAILURE           4
#define ERR_ENCODER_NOT_TRACKING    5
#define ERR_PID_TUNING              6
#define ERR_POSITION_ERROR          7

//=============================================================================
// CORE CALIBRATION & TEST FUNCTIONS
//=============================================================================

// Main test flow - runs full calibration/diagnostic sequence
TestResult runCalibrationFlow(MotorController& motorControl);

// Core tests (return TestResult)
TestResult testEncoderCommunication(MotorController& mc);
TestResult testMotorCalibration(MotorController& mc);
TestResult testPIDTuning(MotorController& mc);
TestResult testPositionControl(MotorController& mc);

// Diagnostic tests (run when core tests fail)
TestResult diagnosticI2CScan(MotorController& mc);
TestResult diagnosticPhaseTest(MotorController& mc);
TestResult diagnosticEncoderAlignment(MotorController& mc);

// Utility functions
void printTestResult(const TestResult& result);
```

### tests.cpp
```cpp
TestResult runCalibrationFlow(MotorController& motorControl) {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║              Calibration & Test Sequence                       ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("\nThis will run a complete calibration and diagnostic sequence.");
    Serial.println("Tests will run automatically with diagnostics on failure.");
    Serial.println("─────────────────────────────────────────────────────────────────\n");

    // Step 1: Encoder Communication
    Serial.println("\n[1/4] Testing Encoder Communication...");
    TestResult result = testEncoderCommunication(motorControl);
    printTestResult(result);

    if (!result.passed) {
        Serial.println("\n[DIAGNOSTIC] Running I2C scan...");
        diagnosticI2CScan(motorControl);
        return result;  // Abort flow
    }

    // Step 2: Motor Calibration
    Serial.println("\n[2/4] Running Motor Calibration...");
    result = testMotorCalibration(motorControl);
    printTestResult(result);

    if (!result.passed) {
        Serial.println("\n[DIAGNOSTIC] Running phase test to identify issue...");
        diagnosticPhaseTest(motorControl);

        Serial.println("\n[DIAGNOSTIC] Checking encoder alignment...");
        diagnosticEncoderAlignment(motorControl);

        return result;  // Abort flow
    }

    // Step 3: PID Tuning
    Serial.println("\n[3/4] Running PID Auto-Tuning...");
    result = testPIDTuning(motorControl);
    printTestResult(result);

    if (!result.passed) {
        Serial.println("\n[DIAGNOSTIC] PID tuning failed.");
        Serial.println("Try reducing velocity and acceleration limits.");
        return result;  // Could retry with different parameters
    }

    // Step 4: Position Control Validation
    Serial.println("\n[4/4] Validating Position Control...");
    result = testPositionControl(motorControl);
    printTestResult(result);

    if (!result.passed) {
        Serial.println("\n[DIAGNOSTIC] Position control failed.");
        Serial.println("Consider re-running PID tuning or checking mechanical load.");
        return result;
    }

    // All tests passed!
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║              ✓ CALIBRATION COMPLETE                            ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("\nMotor system is calibrated and ready for operation.");
    Serial.println("You can now:");
    Serial.println("  • Enable motor with 'e' command");
    Serial.println("  • Move to positions with 'm <angle>' command");
    Serial.println("  • Start PC-side scanning operations");
    Serial.println("");

    return {true, "Calibration Flow", "All tests passed", ERR_NONE};
}

void printTestResult(const TestResult& result) {
    if (result.passed) {
        Serial.print("  ✓ PASS: ");
    } else {
        Serial.print("  ✗ FAIL: ");
    }
    Serial.println(result.message);
}
```

## Command Integration

Add new command to main firmware:

```cpp
else if (command == "calibrate_flow" || command == "cf") {
    runCalibrationFlow(motorControl);
}
```

## Extensibility

### Adding New Tests

To add a new test to the flow:

1. **Define test function** in tests.cpp:
```cpp
TestResult testNewFeature(MotorController& mc) {
    // Test implementation
    bool success = /* test logic */;
    return {success, "New Feature", "Test result message", error_code};
}
```

2. **Add to test sequence** in `runCalibrationFlow()`:
```cpp
Serial.println("\n[X/Y] Testing New Feature...");
result = testNewFeature(motorControl);
printTestResult(result);
if (!result.passed) {
    // Run diagnostic if needed
    return result;
}
```

3. **Add diagnostic** if needed:
```cpp
TestResult diagnosticNewFeature(MotorController& mc) {
    // Diagnostic implementation
}
```

### Reconfiguring Test Flow

The test flow is just a sequence of function calls, making it trivial to:
- **Reorder tests** - Change sequence in `runCalibrationFlow()`
- **Skip tests** - Comment out or add conditional logic
- **Add branches** - Add different paths based on test results
- **Parallel tests** - Run multiple diagnostics simultaneously

Example alternative flow:
```cpp
// Quick flow - skip PID tuning
TestResult runQuickCalibration(MotorController& mc) {
    testEncoderCommunication(mc);
    testMotorCalibration(mc);
    testPositionControl(mc);  // Skip PID auto-tune
}
```

---

## Integration & Testing Plan

### Phase 1: MCU Test Flow Implementation
1. Implement `TestResult` structure
2. Convert existing tests to TestResult format
3. Implement `runCalibrationFlow()` with decision tree
4. Test with functional motor (post-replacement)
5. Verify diagnostics trigger correctly on failures

### Phase 2: PC Controller Development
1. Implement basic `HyperspectralController` class
2. Implement `move_and_verify()` function
3. Test motor communication and position verification
4. Add camera integration (mock camera initially)
5. Test full scan sequence with mock data

### Phase 3: Integration Testing
1. Run full calibration flow on MCU
2. Verify motor ready for PC control
3. Execute test scans from PC
4. Validate position accuracy throughout scan
5. Test error recovery (simulated position errors)

### Phase 4: Production Readiness
1. Add configuration file support (scan parameters)
2. Implement data logging and metadata management
3. Add progress reporting and UI feedback
4. Performance optimization (scan speed vs. accuracy)
5. Long-duration testing (thermal drift, repeatability)

---

## Success Criteria

### MCU Test Flow
- ✓ All tests return clear pass/fail status
- ✓ Failed tests automatically trigger appropriate diagnostics
- ✓ User receives actionable next steps on failure
- ✓ Complete calibration flow takes <2 minutes
- ✓ New tests can be added in <30 lines of code

### PC Controller
- ✓ Position accuracy <0.5° throughout scan
- ✓ Successful error recovery on communication failures
- ✓ Complete 360° scan in <10 minutes (depends on increment)
- ✓ Metadata correctly associated with all images
- ✓ System can run unattended for full scan duration

---

## File Structure

```
firmware/ESP32_MCU_Firmware/
├── tests.h              # Test system definitions, TestResult struct
├── tests.cpp            # Test flow implementation, diagnostics
├── motor_control.h      # Motor controller class (existing)
└── motor_control.cpp    # Motor control functions (existing)

controller/
├── controller.py        # PC-side motor controller
├── config.py            # Configuration parameters
├── scan.py              # Scan session management
├── camera.py            # Camera interface abstraction
└── metadata.py          # Metadata handling
```

---

## Questions for Refinement

1. **Scan speed priorities:** What's more important - scan speed or position accuracy?
2. **Camera trigger:** Hardware trigger available or software only?
3. **Scan patterns:** Only linear 1D rotation, or also back-and-forth, multi-revolution?
4. **Error tolerance:** How should system handle single position failures in middle of scan?
5. **Data format:** Preferred output format for images and metadata (HDF5, TIFF+JSON, other)?
6. **Real-time preview:** Should PC controller show live preview during scan?

---

This scope provides a complete architectural foundation for both the MCU test system and PC controller. Both are designed to be modular, extensible, and production-ready. Let me know if you'd like me to start implementing either system!
