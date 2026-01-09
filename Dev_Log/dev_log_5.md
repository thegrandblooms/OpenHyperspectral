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

## Questionable Claims (Not Definitively Wrong, But Suspect)

### Claim A: getAngle() Override Causes Sign Inversion
**Location:** Dev Log 4, Theory 2

The log claims overriding `getAngle()` to return `angle_prev` (ignoring full_rotations) caused perfect sign inversion. This is suspicious because:
- Ignoring full_rotations should only affect multi-rotation tracking
- It shouldn't cause sign inversion on a single-turn encoder
- The sign inversion is more likely caused by sensor_direction = CCW being applied

### Claim B: sensor_direction = CCW Causes Problems
**Location:** Dev Log 4, Theory 1

The log speculates CCW direction might cause sign inversion, then says "research contradicts this." However, the formula:
```cpp
shaft_angle = sensor_direction * LPF_angle(sensor->getAngle()) - sensor_offset
```
WILL produce negative angles if `sensor_direction = -1` and the base angle is positive. This isn't a "bug" - it's expected behavior. The real question is whether the rest of the code handles this correctly.

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
