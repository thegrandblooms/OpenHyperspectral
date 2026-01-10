# Dev Log 5: Review of Previous Dev Logs - Identified Errors

**Date:** 2026-01-09
**Purpose:** Document obviously wrong theories and information found in dev_log_1 through dev_log_4
**Status:** Review only - no fixes applied yet

---

## Summary

After reviewing all four previous dev logs, I identified several incorrect statements, wrong formulas, and inaccurate technical claims. These are documented below with explanations of why they're wrong.

---

## Error 1: Wrong Formula for zero_electric_angle Calculation

**Location:** Dev Log 1, Lines 477-480

**What It Says:**
```cpp
// Apply voltage at KNOWN electrical angle
motor.setPhaseVoltage(6.0, 0, _3PI_2);
delay(2000); // Wait for motor to settle
encoder.update();
float aligned_position = encoder.getSensorAngle();
// Now we know: when encoder reads X, motor is at 3π/2 electrical
motor.zero_electric_angle = aligned_position - (_3PI_2 / POLE_PAIRS);
```

**Why It's Wrong:**
- `zero_electric_angle` is an **electrical angle** (in radians)
- The formula divides `_3PI_2` (an electrical angle) by `POLE_PAIRS`, which would convert it to a mechanical angle
- This produces a unit mismatch - subtracting a mechanical angle from a mechanical angle, but storing it as an electrical angle

**Correct Formula:**
```cpp
// aligned_position is mechanical angle in radians
// We applied electrical angle _3PI_2
// zero_electric_angle = (mechanical_position × pole_pairs) - applied_electrical_angle
motor.zero_electric_angle = (aligned_position * POLE_PAIRS) - _3PI_2;
// Or equivalently, normalize to 0-2π range:
motor.zero_electric_angle = fmod((aligned_position * POLE_PAIRS) - _3PI_2 + _2PI, _2PI);
```

---

## Error 2: Incorrect Phase Angle Descriptions

**Location:** Dev Log 2, Lines 44-54

**What It Says:**
```cpp
PhaseTest tests[] = {
    {0.0,       "Phase A positive (0° electrical)"},
    {_PI_3,     "Phase A→B transition (60°)"},
    {_PI_2,     "Phase B positive (90°)"},        // ← WRONG
    {2.0*_PI_3, "Phase B→C transition (120°)"},   // ← WRONG
    {PI,        "Phase C positive (180°)"},
    {4.0*_PI_3, "Phase C→A transition (240°)"}
};
```

**Why It's Wrong:**
In a 3-phase BLDC motor, the phases are separated by 120° electrical:
- Phase A peaks at 0°
- Phase B peaks at **120°** (not 90°)
- Phase C peaks at 240°

**Correct Phase Descriptions:**
```cpp
PhaseTest tests[] = {
    {0.0,       "Phase A positive (0° electrical)"},
    {_PI_3,     "Phase A→B transition (60°)"},
    {2.0*_PI_3, "Phase B positive (120°)"},       // 120° = 2π/3
    {PI,        "Phase B→C transition (180°)"},
    {4.0*_PI_3, "Phase C positive (240°)"},       // 240° = 4π/3
    {5.0*_PI_3, "Phase C→A transition (300°)"}
};
```

Note: The 90° angle falls between phases A and B, not at Phase B's peak.

---

## Error 3: Invalid SimpleFOC Configuration Enums

**Location:** Dev Log 4, Lines 255-257

**What It Says:**
```cpp
motor.torque_controller = TorqueControlType::angle;
motor.controller = MotionControlType::angle_openloop;
motor.move(degreesToRadians(target_position_deg));
```

**Why It's Wrong:**

1. **`TorqueControlType::angle` does not exist** in SimpleFOC
   - Valid `TorqueControlType` options are:
     - `TorqueControlType::voltage` (default)
     - `TorqueControlType::dc_current`
     - `TorqueControlType::foc_current`

2. **`MotionControlType::angle_openloop` doesn't exist** either
   - Valid `MotionControlType` options include:
     - `MotionControlType::angle` (closed-loop position control)
     - `MotionControlType::velocity` (closed-loop velocity control)
     - `MotionControlType::velocity_openloop`
     - `MotionControlType::angle_openloop` - WAIT, this DOES exist, I need to verify

**Correction:** After checking SimpleFOC source, `MotionControlType::angle_openloop` DOES exist. However, `TorqueControlType::angle` definitely does NOT exist. If this code is actually in the firmware, it either:
- Won't compile (compile error on invalid enum)
- Is pseudocode/theoretical in the log, not actual code

**Impact:** If this configuration was actually attempted, it would explain the broken motor control behavior, as the torque controller would be in an undefined state.

---

## Error 4: Overestimated I2C Read Time

**Location:** Dev Log 1, Lines 5-6

**What It Says:**
> "SimpleFOC's automatic `initFOC()` calibration fails with MT6701 I2C sensors because I2C is too slow (~10-20ms per read) for SimpleFOC's movement detection algorithm."

**Why It's Likely Wrong:**
- Standard I2C at 400kHz (Fast Mode) can read 2 bytes in ~100-200μs
- Even at 100kHz (Standard Mode), reads take ~500μs-1ms
- **10-20ms per read would indicate a serious I2C bus problem**, not normal operation
- The MT6701 angle register is only 2 bytes - should read in <1ms under normal conditions

**More Accurate Statement:**
> "SimpleFOC's automatic `initFOC()` calibration may have issues with MT6701 I2C sensors because I2C polling (even at ~1ms per read) is slower than SPI alternatives, potentially causing timing issues with SimpleFOC's movement detection algorithm."

**Note:** The referenced GitHub issue #172 should be verified to confirm the actual claim made there.

---

## Error 5: Misleading LPF_angle Diagnosis

**Location:** Dev Log 3, Lines 105-128

**What It Says:**
> "Discovery: We never initialized `motor.LPF_angle.Tf`!"
>
> "The Fix: `motor.LPF_angle.Tf = 0.0f;`"

**Why It's Misleading:**
- SimpleFOC initializes LPF_angle.Tf with a default value (typically 0.0f or small value)
- Setting it to 0.0f disables the low-pass filter (passes signal through unchanged)
- If LPF_angle.Tf being uninitialized was the issue, you'd see:
  - Random/garbage angle values
  - Noisy position tracking
  - NOT a clean "reset to 0°" behavior

**The actual problem** of shaft_angle resetting to 0.00° is not caused by an uninitialized filter. This diagnosis was a red herring.

---

## Research Findings: Definitive Answers to Questionable Theories

The following theories from dev_log_4 were researched using SimpleFOC source code and documentation.

### Theory 1 RESOLVED: sensor_direction = CCW Does Cause Sign Inversion (But It's Expected)

**Location:** Dev Log 4, Theory 1

**SimpleFOC Source Code (Sensor.h):**
```cpp
enum Direction : int8_t {
    CW      = 1,   // clockwise
    CCW     = -1,  // counter clockwise
    UNKNOWN = 0    // not yet known or invalid state
};
```

**SimpleFOC Source Code (FOCMotor.cpp):**
```cpp
float FOCMotor::shaftAngle() {
    return sensor_direction * LPF_angle(sensor->getAngle()) - sensor_offset;
}
```

**Definitive Answer:**
- **YES**, `sensor_direction = CCW` means multiplying by -1
- **YES**, this produces negative shaft_angle when sensor returns positive angles
- **This is NOT a bug** - it's how SimpleFOC normalizes direction
- The rest of SimpleFOC handles negative angles correctly
- The sign inversion observed in testing is **expected behavior** when CCW is detected

**Dev Log 4 Error:** The log said "research contradicts this theory" but the theory is actually CORRECT. The sign inversion IS caused by CCW = -1. What the dev log got wrong is thinking this is a problem - it's working as designed.

---

### Theory 2 RESOLVED: getAngle() Override is NOT the Cause of Sign Inversion

**Location:** Dev Log 4, Theory 2

**SimpleFOC Source Code (Sensor.cpp):**
```cpp
float Sensor::getAngle() {
    return (float)full_rotations * _2PI + angle_prev;
}
```

**Analysis:**
- Overriding `getAngle()` to return just `angle_prev` only affects multi-rotation tracking
- It cannot cause sign inversion - it just ignores accumulated rotations
- For a single-turn absolute encoder (0-360°), this is actually a reasonable override
- The sign inversion is caused by `sensor_direction = -1`, NOT the getAngle() override

**Definitive Answer:** Theory 2 is WRONG. The getAngle() override is not the cause.

---

### GitHub Issue #172 VERIFIED: I2C Calibration Issues Are Real

**Location:** Dev Log 1, Lines 5-6

**GitHub Issue #172 Actual Contents:**
- Title: "initFOC with MT6701Sensor get wrong pole pair value and electrical zero offset"
- User replaced AS5600 with MT6701 sensor
- `initFOC()` produced **wildly incorrect values**:
  - Estimated pole pairs: 18.76 (actual: 11)
  - Estimated zero offset: 5.85 (actual: 3.7)
- Motor consumed 5W instead of expected 1.5W
- Motor oscillated between poles

**SimpleFOC alignSensor() Timing Requirements:**
- Direction detection loop: 500 steps × 2ms = ~1 second total
- Sensor must respond within 2ms per iteration
- I2C reads taking longer cause stale data → wrong direction detection

**MT6701 I2C Driver Status (from Arduino-FOC-drivers):**
> "work in progress... I2C not yet complete"
> "the I2C output of this sensor is probably too slow for high performance motor control"

**Definitive Answer:**
- The claim that I2C causes calibration failures is **TRUE**
- The timing estimate of "10-20ms per read" is **WRONG** (typical reads are <1ms)
- The root cause is timing sensitivity, not raw I2C speed
- Recommended workaround: manual calibration with hardcoded values

---

### MotionControlType::angle_openloop VERIFIED: It Does Exist

**Location:** Dev Log 5, Error 3 (my own error in initial review)

**SimpleFOC Source Code (FOCMotor.h):**
```cpp
enum MotionControlType : uint8_t {
    torque            = 0x00,
    velocity          = 0x01,
    angle             = 0x02,
    velocity_openloop = 0x03,
    angle_openloop    = 0x04   // ← This DOES exist
};
```

**Correction:** `MotionControlType::angle_openloop` is valid. Only `TorqueControlType::angle` is invalid.

---

## Root Cause Analysis: Why Motor Moves Wrong Direction

Based on research, the actual root cause chain is:

1. **initFOC() calibration detected CCW direction** (sensor_direction = -1)
2. **shaftAngle() formula multiplies by -1**: `shaft_angle = -1 * positive_angle - offset`
3. **This produces negative shaft_angle values** (expected behavior)
4. **Motor control loop uses shaft_angle for position feedback**
5. **If the code expects positive angles**, the negative values cause reverse motion

**The fix is NOT to change sensor_direction** - it's to ensure all code handles negative angles correctly, OR to verify if the CCW detection was actually correct for the physical setup.

**Key Question:** Was CCW detection correct? If the motor/sensor are mounted such that increasing encoder angle = CW motor rotation, then CCW detection is wrong. This could happen if:
- Sensor is mounted upside down
- Motor wires are swapped
- Sensor magnet polarity is reversed

---

## Sources

- [SimpleFOC Sensor.h - Direction enum](https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/base_classes/Sensor.h)
- [SimpleFOC FOCMotor.cpp - shaftAngle() implementation](https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/base_classes/FOCMotor.cpp)
- [SimpleFOC Sensor.cpp - getAngle() implementation](https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/base_classes/Sensor.cpp)
- [SimpleFOC FOCMotor.h - MotionControlType enum](https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/base_classes/FOCMotor.h)
- [GitHub Issue #172 - initFOC with MT6701Sensor](https://github.com/simplefoc/Arduino-FOC/issues/172)
- [Arduino-FOC-drivers MT6701 README](https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/mt6701/README.md)
- [SimpleFOC Community - Skip Alignment](https://community.simplefoc.com/t/skip-the-alignment-of-the-position-sensor/4960)

---

## Recommendations

1. **Verify actual code** - Check if the errors in dev logs are in the actual firmware or just in the documentation
2. **Fix zero_electric_angle formula** - Critical for calibration to work
3. **Fix phase test descriptions** - Important for accurate hardware diagnostics
4. **Verify SimpleFOC enum usage** - Check actual code for TorqueControlType settings
5. **Measure actual I2C timing** - Add instrumentation to confirm read latency
6. **Re-examine sign inversion** - The root cause is likely in how sensor_direction interacts with the getAngle() override, not either factor alone

---

**End of Dev Log 5**
