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

## Recommended Approach

**Phase 1 (Immediate)**: Implement manual calibration
- Create `runManualCalibration()` function
- Use SimpleFOC's `setPhaseVoltage()` to apply known angles
- Calculate `zero_electric_angle` and `sensor_direction`
- Test with your hardware
- Document the calibration values

**Phase 2 (Short-term)**: Add NVS storage
- Save calibration to flash
- Skip calibration on subsequent boots
- Add command to force re-calibration

**Phase 3 (Long-term)**: Consider SSI hardware upgrade
- If I2C proves too slow for your application
- Only if you see control loop performance issues
- Requires hardware redesign

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
