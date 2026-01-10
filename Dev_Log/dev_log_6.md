# Dev Log 6: Review and Todo List Creation

**Date:** 2026-01-10
**Purpose:** Comprehensive review of dev logs 1-5 and creation of prioritized todo list
**Status:** Planning phase - identifying critical fixes needed

---

## Session Overview

This session focused on reviewing all previous dev logs (1-5) to identify errors, inconsistencies, and create a comprehensive action plan. Dev Log 5 provided critical analysis identifying multiple errors in previous logs.

---

## Key Findings from Dev Log 5 Review

Dev Log 5 conducted a thorough review of logs 1-4 and identified **5 major errors** that need correction:

### Error 1: Wrong zero_electric_angle Formula ⚠️ CRITICAL

**Location:** Dev Log 1, Lines 477-480

**Problem:**
```cpp
motor.zero_electric_angle = aligned_position - (_3PI_2 / POLE_PAIRS);
```

This formula divides an electrical angle by pole pairs, creating a unit mismatch:
- `aligned_position` is a mechanical angle (radians)
- `_3PI_2` is an electrical angle (radians)
- Dividing electrical by pole pairs converts to mechanical
- Subtracting mechanical from mechanical, but storing as electrical angle

**Correct Formula:**
```cpp
motor.zero_electric_angle = (aligned_position * POLE_PAIRS) - _3PI_2;
// Or with normalization:
motor.zero_electric_angle = fmod((aligned_position * POLE_PAIRS) - _3PI_2 + _2PI, _2PI);
```

**Impact:** Critical for calibration to work correctly

---

### Error 2: Incorrect Phase Angle Descriptions

**Location:** Dev Log 2, Lines 44-54

**Problem:**
Phase descriptions claim:
- Phase B positive at 90° (actually 120°)
- Phase B→C transition at 120° (actually 180°)

**Why It's Wrong:**
In 3-phase BLDC, phases are separated by 120° electrical:
- Phase A peaks at 0°
- Phase B peaks at 120° (not 90°)
- Phase C peaks at 240°

**Correct Descriptions:**
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

**Impact:** Important for accurate hardware diagnostics

---

### Error 3: Invalid SimpleFOC Configuration Enum

**Location:** Dev Log 4, Lines 255-257

**Problem:**
```cpp
motor.torque_controller = TorqueControlType::angle;  // DOES NOT EXIST
```

**Valid TorqueControlType Options:**
- `TorqueControlType::voltage` (default)
- `TorqueControlType::dc_current`
- `TorqueControlType::foc_current`

**Note:** `MotionControlType::angle_openloop` DOES exist and is valid

**Impact:** If this code is in actual firmware, it won't compile or will cause undefined behavior

---

### Error 4: Overestimated I2C Read Time

**Location:** Dev Log 1, Lines 5-6

**Claim:**
> "I2C is too slow (~10-20ms per read)"

**Reality:**
- Standard I2C at 400kHz: ~100-200μs for 2-byte read
- Even at 100kHz: ~500μs-1ms
- 10-20ms would indicate serious I2C bus problem

**More Accurate Statement:**
> "I2C polling (even at ~1ms per read) is slower than SPI alternatives, potentially causing timing issues with SimpleFOC's movement detection algorithm."

**Impact:** Misleading diagnosis, though I2C timing IS still a factor in calibration issues

---

### Error 5: Misleading LPF_angle Diagnosis

**Location:** Dev Log 3, Lines 105-128

**Claim:**
> "Discovery: We never initialized `motor.LPF_angle.Tf`!"
> "The Fix: `motor.LPF_angle.Tf = 0.0f;`"

**Why It's Misleading:**
- SimpleFOC initializes LPF_angle.Tf with default value
- Setting to 0.0f disables the low-pass filter (passes signal through unchanged)
- An uninitialized filter would cause random/noisy angles, NOT clean "reset to 0°" behavior

**Reality:** The shaft_angle reset to 0° problem has a different root cause

**Impact:** Red herring that didn't solve the actual problem

---

## Critical Research Findings

### Finding 1: CCW Sign Inversion is Expected Behavior

**From SimpleFOC Source Code:**
```cpp
// Sensor.h
enum Direction : int8_t {
    CW      = 1,   // clockwise
    CCW     = -1,  // counter clockwise
    UNKNOWN = 0
};

// FOCMotor.cpp
float FOCMotor::shaftAngle() {
    return sensor_direction * LPF_angle(sensor->getAngle()) - sensor_offset;
}
```

**Key Insight:**
- When `sensor_direction = CCW`, it multiplies by -1
- This produces negative shaft_angle when sensor returns positive angles
- **This is NOT a bug** - it's how SimpleFOC normalizes direction
- The rest of SimpleFOC handles negative angles correctly

**Implication:** The observed sign inversion in Dev Log 4 is expected behavior, not a problem to fix

---

### Finding 2: I2C Calibration Issues Are Real

**GitHub Issue #172 Verified:**
- User replaced AS5600 with MT6701 I2C sensor
- `initFOC()` produced wildly incorrect values
- Motor consumed 5W instead of 1.5W
- Motor oscillated between poles

**SimpleFOC Timing Requirements:**
- Direction detection: 500 steps × 2ms = ~1 second
- Sensor must respond within 2ms per iteration
- I2C reads taking longer cause stale data → wrong direction detection

**MT6701 I2C Driver Status:**
> "work in progress... I2C not yet complete"
> "the I2C output of this sensor is probably too slow for high performance motor control"

**Recommendation:** Manual calibration with hardcoded values

---

## Current System State

### What's Working ✅
- Encoder reading (MT6701 I2C communication perfect)
- Basic motor power (motor responds to voltage commands)
- System integration (firmware compiles and runs)
- Manual calibration implementation (with formula fix needed)

### What's Broken ❌
- Motor oscillates/hunts instead of settling at target position
- Position control unstable
- Calibration may have wrong zero_electric_angle due to formula error

### Uncertainty 🤔
- Root cause of oscillation/hunting behavior unclear
- PID parameters may need tuning
- Interaction between CCW direction and control loop

---

## Action Items

See `Dev_Log/todo.md` for complete prioritized todo list.

**High Priority:**
1. Fix zero_electric_angle formula (critical math error)
2. Correct documentation errors in previous logs
3. Investigate motor oscillation/hunting root cause
4. Verify PID tuning parameters
5. Check firmware for invalid SimpleFOC enum usage

**Medium Priority:**
6. Measure actual I2C read timing
7. Handle CCW direction correctly in software
8. Test manual calibration with corrected formula
9. Remove/fix getAngle() override if present

---

## Next Steps

1. **Correct dev log documentation** - Fix errors identified in logs 1-4
2. **Investigate oscillation** - Root cause analysis of hunting behavior
3. **Test corrected calibration** - Apply fixed zero_electric_angle formula
4. **PID tuning** - Verify stability parameters

---

## References

- Dev Log 5: Comprehensive error review
- SimpleFOC GitHub Issue #172: MT6701 calibration issues
- SimpleFOC Source Code: Sensor.h, FOCMotor.cpp
- Arduino-FOC-drivers: MT6701 README

---

**End of Dev Log 6**
