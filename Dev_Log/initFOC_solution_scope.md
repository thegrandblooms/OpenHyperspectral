# initFOC() Solution Scope - MT6701 I2C Calibration

**Status**: Research Complete
**Date**: 2025-11-17
**Branch**: `claude/scope-initfoc-fix-01UcUgzPqPJPUY4Q26dbMSZ5`

---

## Executive Summary

**Root Cause**: SimpleFOC's automatic `initFOC()` calibration **does not work reliably with MT6701 I2C sensors**. This is a known, documented issue in the SimpleFOC community (GitHub issue #172).

**Solution**: Implement **manual calibration** to find `zero_electric_angle` and `sensor_direction`, then pass these values to `initFOC()` to skip the problematic automatic alignment.

**Why This Isn't Reinventing the Wheel**: We're following SimpleFOC's documented approach for absolute encoders where automatic calibration fails. This is the **standard workaround** recommended by the SimpleFOC community.

---

## Application Requirements: Hyperspectral Line Scanning

This motor control system is designed for **hyperspectral imaging**, which has specific requirements that influence our technical decisions:

### Position Verification Loop

**Requirement**: Before each spectral capture, the system must:
1. Command motor to target angle
2. Read encoder to verify position
3. If position error > tolerance → auto-correct
4. Re-verify position
5. Only proceed with image capture when position is confirmed

**Implications**:
- ✅ **Absolute encoder is critical** - no incremental encoders (lose position on power-off)
- ✅ **Position accuracy >> speed** - better to be slow and accurate than fast and wrong
- ✅ **Stop-and-verify operation** - not continuous motion
- ✅ **I2C speed is acceptable** - position verification happens during settling time

### Scan Pattern

**Requirement**: Hundreds to thousands of position verifications per hyperspectral scan:
- Move to angle θ₁ → verify → capture → move to θ₂ → verify → capture → ...
- Each position must be confirmed before sensor readout
- Feedback system automatically corrects drift

**Implications**:
- ✅ **Absolute position on every read** - I2C provides this
- ✅ **Gimbal motor = perfect match** - designed for precise positioning, not speed
- ✅ **FOC control loop at 50-100Hz is sufficient** - motor settles slowly anyway
- ⚠️ **If verification is too slow**, consider analog output (100x faster, still absolute)

### Why I2C Works for This Application

Unlike high-speed motor control (robotics, drones, etc.) where 1kHz+ update rates are critical:

**Your application**:
- Motor moves → **settles for 500ms** → verify position → **capture image (100-1000ms)** → repeat
- The **settling time >> I2C read time** (500ms vs 10-20ms)
- Position verification happens during the settling period (essentially "free")
- Absolute encoder ensures position is never lost between captures

**Conclusion**: I2C's "slowness" is irrelevant for your use case. You're limited by mechanical settling time and image capture time, not sensor read speed.

---

## Research Findings

### 1. SimpleFOC MT6701 I2C Support Status

From the official Arduino-FOC-drivers repository (2025):

```
⚠️ work in progress... SSI driver is working. I2C not yet complete.
```

**Key Limitation**:
> "the I2C output of this sensor is probably too slow for high performance motor control"

**Current State**:
- ✅ SSI/SPI driver: WORKING
- ⚠️ I2C driver: INCOMPLETE (work in progress)
- ✅ ABZ output: Works with standard SimpleFOC encoder classes
- ✅ Analog output: Works with standard SimpleFOC analog sensor class

### 2. Known Issue: initFOC() Fails with MT6701

**GitHub Issue #172**: "initFOC with MT6701Sensor get wrong pole pair value and electrical zero offset"

**Symptoms**:
- Estimated pole pairs: ~18.76 (actual: 7 in our case, 11 in issue example)
- Wrong electrical zero offset
- Motor draws excessive power (5W vs 1.5W with correct calibration)
- Motor oscillates between poles instead of smooth rotation

**Confirmed Workaround**:
> "manually tested the motor's electrical zero offset... this significantly reduced power consumption and improved motor performance"

### 3. Why Custom MT6701 Wrapper Is Necessary

Our custom `MT6701Sensor` class (wrapping the MT6701 library from `firmware/libraries/MT6701`) is **necessary** because:
- SimpleFOC's official I2C driver is incomplete
- We need full control over I2C communication
- Our wrapper successfully reads encoder position (this is confirmed working)

**What Works**:
- ✅ I2C communication with MT6701
- ✅ Reading absolute position (radians, degrees, raw counts)
- ✅ Velocity calculation
- ✅ Field strength monitoring
- ✅ Integration with SimpleFOC via `Sensor` interface

**What Doesn't Work**:
- ❌ SimpleFOC's automatic `initFOC()` calibration
- ❌ Automatic detection of motor movement during alignment
- ❌ Both `needsSearch=0` and `needsSearch=1` paths fail

---

## Solution: Manual Calibration

SimpleFOC documentation explicitly supports manual calibration for absolute encoders:

> "For absolute sensors such as magnetic sensors, once you have the motor's zero electrical offset and sensor direction, you can set these values to avoid the alignment procedure"

### How Manual Calibration Works

Based on SimpleFOC's `find_sensor_offset_and_direction.ino` example and `BLDCMotor::alignSensor()` source code:

1. **Apply Known Electrical Angle**
   ```cpp
   motor.setPhaseVoltage(voltage_align, 0, _3PI_2);  // Apply voltage at 270° electrical
   delay(700);  // Wait for motor to settle
   ```

2. **Read Sensor Position**
   ```cpp
   sensor.update();
   float mechanical_angle = sensor.getSensorAngle();  // Read where motor actually is
   ```

3. **Calculate Zero Electric Angle**
   ```cpp
   // The motor is now at electrical angle = _3PI_2 (270°)
   // The sensor reads mechanical angle
   // zero_electric_angle is the offset between them
   zero_electric_angle = _electricalAngle(mechanical_angle, pole_pairs) - _3PI_2;
   zero_electric_angle = _normalizeAngle(zero_electric_angle);
   ```

4. **Determine Sensor Direction**
   - Apply voltage sweeps and monitor sensor response
   - If sensor increases when motor rotates forward → CW
   - If sensor decreases when motor rotates forward → CCW

5. **Use Calibration Values**
   ```cpp
   motor.zero_electric_angle = 2.15;  // Example value (will be different for each setup)
   motor.sensor_direction = Direction::CW;  // or CCW
   motor.initFOC();  // Now skips alignment and returns SUCCESS
   ```

---

## Implementation Plan

### Phase 1: Create Manual Calibration Routine

**File**: `motor_control.cpp` - Add new function `runManualCalibration()`

**Steps**:
1. Enable motor
2. Apply voltage at multiple known electrical angles (_3PI_2, 0, PI_2, PI)
3. Read sensor position at each angle
4. Calculate average zero_electric_angle
5. Determine sensor_direction from angle progression
6. Validate results (check for consistency)
7. Return calibration values

**Advantages**:
- ✅ Works with MT6701 I2C (uses only sensor reading, not movement detection)
- ✅ Deterministic (same process every time)
- ✅ Can be run once and saved to NVS
- ✅ Standard SimpleFOC approach for absolute encoders

### Phase 2: Save Calibration to NVS

**File**: Create `calibration_storage.cpp`

**Purpose**: Save/load calibration values to/from ESP32 NVS (Non-Volatile Storage)

**Features**:
- Save `zero_electric_angle`, `sensor_direction` after successful calibration
- Load on boot and skip manual calibration if values exist
- Add command to clear/reset calibration (force re-calibration)

### Phase 3: Fallback to Auto-Calibration (Optional)

**Purpose**: Try manual calibration first, fall back to auto if manual fails

**Logic**:
```cpp
// Try manual calibration
if (!runManualCalibration()) {
    // Fall back to auto-calibration (current approach)
    return runAutoCalibration();
}
```

---

## Alternative Solutions (For Future Consideration)

### Alternative 1: Switch to MT6701 SSI Interface

**Pros**:
- ✅ SimpleFOC SSI driver is complete and working
- ✅ Faster than I2C (~10x)
- ✅ Built-in CRC error checking (better noise immunity)
- ✅ Better EMI resistance near motors

**Cons**:
- ❌ Requires hardware rewiring (3 wires: CS, CLK, DO)
- ❌ Uses SPI pins (may conflict with other peripherals)
- ❌ MT6701 reportedly struggles sharing SPI bus

**Verdict**: Consider for future hardware revision, not current firmware fix.

### Alternative 2: Use MT6701 ABZ Output

**Pros**:
- ✅ Works with standard SimpleFOC encoder class
- ✅ Faster than I2C
- ✅ Standard SimpleFOC calibration may work better

**Cons**:
- ❌ Requires hardware rewiring (3 wires: A, B, Z)
- ❌ Loses absolute position benefit on startup
- ❌ Need to use I2C to initialize absolute position anyway

**Verdict**: Defeats the purpose of using an absolute encoder.

### Alternative 3: Use Hybrid Approach

**Concept**:
- Use MT6701 ABZ output for real-time FOC control (fast)
- Use I2C only on startup to read absolute position
- Best of both worlds: absolute position + fast incremental updates

**Pros**:
- ✅ Fast control loop
- ✅ Absolute position on boot
- ✅ Standard SimpleFOC encoder support

**Cons**:
- ❌ Requires hardware rewiring
- ❌ More complex initialization
- ❌ Need to configure MT6701 ABZ resolution (via I2C)

**Verdict**: Interesting for future, but manual calibration solves the problem now.

---

## MT6701 Communication Protocol Comparison

The MT6701 supports **five different output modes**. Here's how they compare for your hyperspectral imaging application:

### Protocol Specifications

| Protocol | Speed | Resolution | Absolute Position? | Wiring | SimpleFOC Support |
|----------|-------|------------|-------------------|--------|-------------------|
| **I2C** (current) | 50-100 Hz | 14-bit (0.022°) | ✅ Yes | 2 wires (SDA, SCL) | ⚠️ Incomplete driver |
| **SSI/SPI** | 5-10 kHz | 14-bit (0.022°) | ✅ Yes | 3 wires (CS, CLK, MISO) | ✅ Working driver |
| **Analog** | 10-20 kHz | 12-bit* (0.088°) | ✅ Yes | 1 wire (Vout) | ✅ Standard class |
| **PWM** | Variable | 14-bit | ✅ Yes | 1 wire | ⚠️ No standard driver |
| **ABZ** | Very fast | Configurable | ❌ **No** (incremental) | 3 wires (A, B, Z) | ✅ Standard class |

\* Analog resolution limited by ESP32 12-bit ADC, not MT6701 output

### Detailed Protocol Analysis

#### I2C (Current Implementation)

**How it works**:
- ESP32 requests angle via I2C bus → MT6701 responds with 14-bit position
- Transaction time: ~10-20ms per read
- Update rate: 50-100 Hz

**For your application**:
- ✅ **Sufficient for FOC** - gimbal motors + stop-and-verify pattern
- ✅ **14-bit resolution** - best available precision
- ✅ **No hardware changes needed**
- ✅ **Settling time >> read time** - I2C "slowness" is irrelevant
- ❌ Only blocker: `initFOC()` calibration (solved by manual calibration)

**Verdict**: **Recommended - stick with this + manual calibration**

---

#### SSI/SPI

**How it works**:
- ESP32 clocks data out via SPI bus
- Includes 6-bit CRC for error detection
- Transaction time: ~100-200μs
- Update rate: 5-10 kHz

**For your application**:
- ✅ **100x faster than I2C** - but you don't need this speed
- ✅ **14-bit resolution** - same as I2C
- ✅ **CRC error checking** - better EMI immunity
- ✅ **SimpleFOC driver complete**
- ❌ **Requires rewiring** - 3 pins (major hardware change)
- ❌ **MT6701 doesn't share SPI bus well** - conflicts with display/SD card

**Verdict**: Overkill for your application. Only consider if you have EMI issues with I2C.

---

#### Analog Output (Pin 3)

**How it works**:
- MT6701 outputs voltage proportional to angle: 0-360° → 0-VDD
- ESP32 ADC reads voltage
- Transaction time: ~10-50μs
- Update rate: 10-20 kHz

**Key insight**: **STILL ABSOLUTE!** Voltage = angle on power-up.

**For your application**:
- ✅ **200x faster than I2C** - but you don't need this
- ✅ **Absolute position** - no homing required
- ✅ **SimpleFOC `MagneticSensorAnalog` class works**
- ✅ **Minimal hardware change** - add 1 wire to ADC pin
- ⚠️ **12-bit resolution** - 0.088° vs I2C's 0.022° (still excellent for your use)
- ⚠️ **ADC noise** - ±1-2 LSB typical (still sub-degree accuracy)

**Resolution impact**:
- I2C 14-bit: 16384 counts/360° = **0.022° resolution**
- ADC 12-bit: 4096 counts/360° = **0.088° resolution**
- For 1° angular steps in hyperspectral scan: **11 ADC counts per step** (plenty!)

**Verdict**: **Best upgrade path if I2C proves inadequate**. Easy to add, maintains absolute position.

---

#### ABZ (Incremental Encoder)

**How it works**:
- Quadrature pulses (A/B) count rotations
- Z pulse marks index (once per revolution)
- Interrupt-driven counting

**For your application**:
- ✅ **Very fast**
- ✅ **SimpleFOC encoder class works**
- ❌ **NOT ABSOLUTE** - loses position on power-off
- ❌ **Must home on startup** - rotate to find index
- ❌ **Defeats purpose of absolute encoder**

**Verdict**: ❌ **Do not use** - your application requires absolute position.

---

### Hardware Accessibility Check

From your hardware setup:
- ✅ **I2C**: Currently wired (GPIO 47/48) → **Working**
- ❓ **SSI/SPI**: Would need 3 GPIOs (CS, CLK, MISO) → **Requires rewiring**
- ❓ **Analog**: MT6701 Pin 3 available? → **Check your module**
- ❓ **ABZ**: Would need 3 GPIOs (A, B, Z) → **Not recommended anyway**

**Action item**: If you have access to MT6701 Pin 3 (analog output), it's easy to add a jumper wire to any ESP32 ADC-capable GPIO as a backup option.

---

### Protocol Decision for Hyperspectral Imaging

**Your requirements**:
1. Absolute position (no homing on startup) → ❌ Eliminates ABZ
2. Position verification before each capture → ✅ I2C is sufficient
3. Hundreds/thousands of verifications per scan → ✅ Speed not critical (limited by settling time)
4. Automatic correction for drift → ✅ Absolute encoder required

**Conclusion**:
- **Use I2C** with manual calibration (Phase 1)
- **Keep analog as backup** (Phase 2 if needed)
- **Ignore SSI/ABZ** unless you redesign hardware

---

## Recommended Approach

**Phase 1 (Immediate)**: Implement manual calibration
- Create `runManualCalibration()` function
- Use SimpleFOC's `setPhaseVoltage()` to apply known angles
- Calculate `zero_electric_angle` and `sensor_direction`
- Test with your hardware
- Document the calibration values

**Phase 2 (Short-term)**: Position verification loop for hyperspectral imaging
- Implement `moveToPositionAndVerify()` with automatic correction
- Confirmed position check before allowing image capture
- Retry logic if position drifts
- Return success/failure status

**Phase 3 (Short-term)**: Add NVS storage
- Save calibration to flash
- Skip calibration on subsequent boots
- Add command to force re-calibration

**Phase 4 (Optional)**: Consider analog output upgrade
- If I2C proves inadequate (unlikely)
- Add 1 wire from MT6701 Pin 3 to ESP32 ADC pin
- Still maintains absolute position
- 200x faster than I2C

---

## Code References from SimpleFOC

### Manual Calibration Pattern

From SimpleFOC's `BLDCMotor::alignSensor()` (src/BLDCMotor.cpp):

```cpp
// Step 1: Apply voltage at known electrical angle
setPhaseVoltage(voltage_sensor_align, 0, _3PI_2);
delay(700);  // Wait for motor to settle

// Step 2: Read sensor
sensor->update();

// Step 3: Calculate zero_electric_angle
zero_electric_angle = _electricalAngle(sensor_direction * sensor->getAngle(), pole_pairs);
zero_electric_angle = _normalizeAngle(zero_electric_angle - _3PI_2 - zero_electric_offset);
```

### Skip Calibration Pattern

From SimpleFOC documentation and examples:

```cpp
// Set calibration values BEFORE initFOC()
motor.zero_electric_angle = 2.15;  // rad (YOUR VALUE WILL BE DIFFERENT)
motor.sensor_direction = Direction::CW;  // or CCW

// Now initFOC() skips alignment
int result = motor.initFOC();
// result should be 1 (success)
```

### Position Verification Loop (For Hyperspectral Imaging)

From application requirements - confirms position before image capture:

```cpp
/**
 * Move to target position and verify arrival with automatic correction
 * @param target_deg Target position in degrees
 * @param tolerance_deg Position tolerance in degrees (e.g., 0.5°)
 * @param max_retries Maximum correction attempts (default 5)
 * @return true if position confirmed within tolerance
 */
bool MotorController::moveToPositionAndVerify(float target_deg,
                                                float tolerance_deg,
                                                int max_retries) {
    if (!motor_enabled || !motor_calibrated) {
        return false;
    }

    // Initial move command
    moveToPosition(target_deg);  // Existing SimpleFOC move

    // Wait for settling (tuned for your motor/load)
    delay(500);  // Adjust based on motor characteristics

    // Verify position with absolute encoder (not SimpleFOC shaft_angle)
    int retries = 0;
    while (retries < max_retries) {
        // Read absolute position from MT6701
        encoder.update();
        float actual_deg = encoder.getDegrees();
        float error = abs(actual_deg - target_deg);

        // Check if within tolerance
        if (error < tolerance_deg) {
            // Position confirmed!
            if (DEBUG_MOTOR) {
                Serial.print("[VERIFY] Position confirmed: ");
                Serial.print(actual_deg, 2);
                Serial.print("° (target: ");
                Serial.print(target_deg, 2);
                Serial.print("°, error: ");
                Serial.print(error, 3);
                Serial.println("°)");
            }
            return true;
        }

        // Position error detected - apply correction
        if (DEBUG_MOTOR) {
            Serial.print("[VERIFY] Position error: ");
            Serial.print(error, 2);
            Serial.print("° - retrying (attempt ");
            Serial.print(retries + 1);
            Serial.print("/");
            Serial.print(max_retries);
            Serial.println(")");
        }

        // Command correction
        moveToPosition(target_deg);
        delay(200);  // Shorter delay for corrections
        retries++;
    }

    // Failed to reach position after max retries
    if (DEBUG_MOTOR) {
        encoder.update();
        Serial.print("[VERIFY] FAILED - final position: ");
        Serial.print(encoder.getDegrees(), 2);
        Serial.print("° (target: ");
        Serial.print(target_deg, 2);
        Serial.println("°)");
    }
    return false;
}

/**
 * Hyperspectral scan loop example
 */
void runHyperspectralScan(float start_deg, float end_deg, float step_deg) {
    Serial.println("[SCAN] Starting hyperspectral scan...");

    int successful_captures = 0;
    int failed_positions = 0;

    for (float angle = start_deg; angle <= end_deg; angle += step_deg) {
        // Move to angle and verify
        if (moveToPositionAndVerify(angle, 0.5)) {  // 0.5° tolerance
            // Position confirmed - safe to capture
            Serial.print("[SCAN] Capturing at ");
            Serial.print(angle, 1);
            Serial.println("°...");

            // TODO: Trigger hyperspectral camera capture here
            // captureSpectralImage();

            successful_captures++;
        } else {
            // Position verification failed
            Serial.print("[SCAN] Skipping ");
            Serial.print(angle, 1);
            Serial.println("° - position verification failed");
            failed_positions++;
        }

        // Optional: Add delay between captures for sensor processing
        delay(100);
    }

    Serial.print("[SCAN] Complete - ");
    Serial.print(successful_captures);
    Serial.print(" captures, ");
    Serial.print(failed_positions);
    Serial.println(" failures");
}
```

**Key features**:
1. ✅ Uses **absolute encoder** for verification (not SimpleFOC's internal state)
2. ✅ **Automatic retry** with correction if position drifts
3. ✅ **Confirmed position** before proceeding with image capture
4. ✅ **Fail-safe** - returns false if position cannot be achieved
5. ✅ **Application-aware** - designed for stop-and-verify hyperspectral scanning

---

## Testing Plan

### Test 1: Verify Manual Calibration Values

1. Run manual calibration routine
2. Print `zero_electric_angle` and `sensor_direction`
3. Manually inspect motor alignment (should hold position firmly)
4. Rotate motor by hand, verify encoder reads correctly
5. Compare calculated electrical angle vs expected

### Test 2: Verify initFOC() Success

1. Set calibration values
2. Call `initFOC()`
3. Verify return value == 1
4. Check that `motor.shaft_angle` updates during rotation
5. Verify smooth motor rotation (no pole oscillation)

### Test 3: Verify FOC Control

1. Enable motor
2. Command position movement
3. Observe smooth rotation
4. Check current draw (should be ~1.5W idle, not 5W)
5. Verify position control accuracy

### Test 4: Verify NVS Storage

1. Run calibration, save to NVS
2. Reboot ESP32
3. Load calibration from NVS
4. Skip manual calibration
5. Verify motor still works correctly

---

## Success Criteria

### Must Have ✅
- [ ] `motor.initFOC()` returns 1 (success)
- [ ] `motor.shaft_angle` tracks encoder position
- [ ] Motor rotates smoothly (no oscillation between poles)
- [ ] Position control works (can command movements)
- [ ] Current draw is normal (~1.5W idle, not 5W)

### Should Have 📋
- [ ] Calibration saved to NVS
- [ ] Automatic calibration skip on subsequent boots
- [ ] Command to force re-calibration
- [ ] Diagnostic output for calibration process

### Nice to Have 🎯
- [ ] Multiple calibration runs averaged for accuracy
- [ ] Calibration validation (detect bad calibration)
- [ ] Auto-detect if calibration has drifted
- [ ] Temperature-compensated calibration (if needed)

---

## Next Steps

1. **Review this scope document** - Confirm approach before implementation
2. **Implement manual calibration** - Start with Phase 1
3. **Test on hardware** - Validate calibration values
4. **Add NVS storage** - Phase 2 for persistence
5. **Document final values** - Create reference for future setups

---

## Key Takeaways

1. **You're not alone** - This is a documented SimpleFOC + MT6701 issue
2. **You're not reinventing** - Manual calibration is the standard SimpleFOC approach
3. **Your encoder works perfectly** - The problem is purely in initFOC() alignment
4. **The fix is well-documented** - SimpleFOC provides examples and patterns
5. **I2C is slow but usable** - Many users successfully run FOC with I2C encoders at lower speeds (gimbal motors are perfect for this)

---

## References

- SimpleFOC Docs: https://docs.simplefoc.com/bldcmotor
- GitHub Issue #172: https://github.com/simplefoc/Arduino-FOC/issues/172
- MT6701 Driver: https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/encoders/mt6701
- Community Discussion: https://community.simplefoc.com/t/mt6701-magnetic-position-encoder-support/2618
