# Dev Log 4: SimpleFOC Integration Debugging - Sign Inversion Issue

**Date:** 2026-01-08 - 2026-01-09
**Session Focus:** Debugging shaft_angle sign inversion and tracking failures
**Status:** ⚠️ BROKEN - shaft_angle shows sign inversion during movement, motor moves wrong direction

---

## Executive Summary

**CURRENT STATUS:** Motor control is fundamentally broken. After implementing SimpleFOC's correct sensor pattern and attempting to fix wraparound tracking, shaft_angle now shows perfect sign inversion of the encoder reading during movement, causing the motor to move in the wrong direction.

**⚠️ UPDATE (from Dev Log 5 analysis):** The sign inversion is actually **expected SimpleFOC behavior** when `sensor_direction = CCW`. The formula `shaft_angle = sensor_direction * angle - offset` intentionally produces negative angles when CCW=-1. This is working as designed - SimpleFOC handles negative angles correctly. The real issue is likely in application code or control loop interaction with these negative angles.

**Observable Facts (NOT theories):**
- ❌ shaft_angle = -encoder_reading during movement (perfect sign inversion) → **Actually expected with CCW**
- ❌ Motor moves backwards when commanded forward
- ❌ When motor disabled: shaft_angle = 0.00° while encoder shows actual position
- ❌ AT_TARGET always shows 0.00° even when manually moving encoder
- ✅ Manual shaft_angle assignment works (before movement starts)
- ✅ Encoder reads correctly throughout (I2C working fine)

---

## Session Work Summary

### 1. Researched Correct SimpleFOC Sensor Pattern

**Finding:** ALL SimpleFOC magnetic sensor implementations follow the same pattern:
- ✅ Only implement `getSensorAngle()`
- ❌ Do NOT override `update()` - let base class handle it

**Sources Verified:**
- Official Arduino-FOC-drivers: `MagneticSensorMT6701SSI`
- SmartKnob production code: `MT6701Sensor`
- All SimpleFOC sensors: `MagneticSensorI2C`, `MagneticSensorSPI`, `AS5600`, `AS5048A`
- SimpleFOC documentation explicitly states: "implement the getSensorAngle() method"

**Exception:** Only `HallSensor` overrides `update()` (interrupt-driven, not absolute encoder)

### 2. Refactored MT6701Sensor Implementation

**Previous Approach (WRONG):**
```cpp
// Overrode update() to cache values, then called Sensor::update()
void MT6701Sensor::update() {
    cached_raw_count = encoder.readRawAngle();
    cached_radians = degreesToRadians(rawToDegrees(cached_raw_count));
    Sensor::update();  // Called base class
}

float MT6701Sensor::getSensorAngle() {
    return cached_radians;  // Return cached
}
```

**New Approach (CORRECT):**
```cpp
// Do NOT override update() - removed entirely
// Let base class Sensor::update() handle tracking

float MT6701Sensor::getSensorAngle() {
    // Read I2C directly and return angle
    cached_raw_count = encoder.readRawAngle();
    cached_radians = degreesToRadians(rawToDegrees(cached_raw_count));
    return cached_radians;
}
```

**What Base Class Does:**
```cpp
// From SimpleFOC Sensor.cpp
void Sensor::update() {
    float val = getSensorAngle();  // Calls our implementation
    if (val < 0) return;  // Error handling
    angle_prev_ts = _micros();
    float d_angle = val - angle_prev;
    // Wraparound detection
    if(abs(d_angle) > (0.8f*_2PI))  // If angle jumped >5 radians
        full_rotations += (d_angle > 0) ? -1 : 1;
    angle_prev = val;
}
```

**Commit:** `c1963f5` - "Refactor MT6701Sensor to follow correct SimpleFOC pattern"

---

## Observed Behavior (Facts Only)

### Test 1: After Wraparound Fix (getAngle() override)

**Command:** Move +30° from current position (261.39° → ~291°)

**Before Movement:**
```
Encoder (abs): 261.39° | shaft_angle: 261.39° ✓
(Manual assignment: motor.shaft_angle = encoder reading)
```

**During Movement:**
```
Encoder (abs): 111.18° | shaft_angle: -111.18° ❌ SIGN INVERSION
Encoder (abs): 147.30° | shaft_angle: -147.30° ❌ SIGN INVERSION
```

**Actual Motor Movement:**
- Motor moved BACKWARDS: 261° → 111° (moved -150°)
- Expected: Move forwards +30° to ~291°
- Opposite direction from command

**Pattern:** shaft_angle = -1 × encoder_reading (perfect negation)

### Test 2: Motor Disabled State

**Observation:**
```
Encoder (abs): 287.23° | shaft_angle: 0.00° | Target: 0.00°
AT_TARGET: 0.00° (always, even when manually moving encoder)
```

**When Motor Disabled:**
- shaft_angle stuck at 0.00°
- Encoder reads actual position correctly
- AT_TARGET always reports 0.00° regardless of encoder position

### Test 3: Manual Position Assignment

**Code:**
```cpp
motor.shaft_angle = degreesToRadians(encoder_degrees);
```

**Result:** Works correctly - shaft_angle matches encoder immediately after assignment

**BUT:** Once motor.loopFOC() and motor.move() are called, sign inversion occurs

### Motor Calibration Output

**From user's calibration log:**
```
Sensor direction: CCW
Zero electric angle: 4.89 rad
Motor initialized successfully
```

**Known Value:** sensor_direction = Direction::CCW

---

## Top 5 Theories (Based on Research)

### Theory 1: sensor_direction = CCW Causes Sign Inversion in SimpleFOC Formula

**SimpleFOC Formula (from source code):**
```cpp
shaft_angle = sensor_direction * LPF_angle(sensor->getAngle()) - sensor_offset
```

**Known:** User's calibration shows `sensor_direction = CCW`

**Hypothesis:** If Direction::CCW = -1 as a multiplier, formula becomes:
```cpp
shaft_angle = (-1) * positive_angle - sensor_offset
shaft_angle = -positive_angle - sensor_offset
```

This would explain the perfect sign inversion: shaft_angle = -111.18° when encoder = 111.18°

**Research Findings:**
- ❌ NO evidence this causes problems in working systems
- ❌ Both CW and CCW used successfully in production code
- ❌ No discussions about avoiding CCW or negative angles
- ❌ Official SimpleFOC examples auto-detect direction without issues

**Confidence:** LOW - Research contradicts this theory

**✅ CORRECTION (from Dev Log 5 research):** This theory is actually CORRECT! SimpleFOC source code confirms:
- `Direction::CCW = -1` (defined in Sensor.h)
- Formula: `shaft_angle = sensor_direction * LPF_angle(sensor->getAngle()) - sensor_offset`
- When `sensor_direction = CCW`, this DOES produce negative shaft_angle values
- **This is NOT a bug** - it's how SimpleFOC normalizes motor direction
- SimpleFOC's control loops handle negative angles correctly
- The sign inversion observed is **expected behavior** when CCW direction is detected
- The issue isn't the sign inversion itself, but ensuring all application code handles negative angles properly

### Theory 2: getAngle() Override Breaks SimpleFOC's Internal State

**What We Changed:**
```cpp
float MT6701Sensor::getAngle() override {
    return angle_prev;  // Ignore full_rotations
}
```

**Hypothesis:** SimpleFOC's BLDCMotor may be calling both:
- `sensor->getAngle()` - which we override
- Reading `sensor.angle_prev` directly - which bypasses our override

This could create conflicting state where:
- Our override returns positive angle
- Internal calculations use base class formula with full_rotations
- Result: sign inversion or other weird behavior

**Research Findings:**
- ⚠️ Very few SimpleFOC sensors override getAngle()
- ⚠️ Standard pattern is to ONLY override getSensorAngle()
- ⚠️ getAngle() is meant for BLDCMotor to call, not to override

**Confidence:** MEDIUM - Violates standard pattern

### Theory 3: sensor_offset Not Applied Correctly During Calibration

**SimpleFOC Formula:**
```cpp
shaft_angle = sensor_direction * angle - sensor_offset
```

**Hypothesis:** sensor_offset may be:
- Set incorrectly during calibration
- Applied with wrong sign
- Not being subtracted when we override getAngle()

If sensor_offset is wrong, it could cause:
- Position errors
- Direction errors
- Sign inversion when combined with sensor_direction

**Research Findings:**
- ⚠️ sensor_offset is set during motor.initFOC()
- ⚠️ Calibration process auto-detects sensor_direction
- ⚠️ May not work correctly with our getAngle() override

**Confidence:** MEDIUM - Could interact badly with our override

### Theory 4: Manual shaft_angle Assignment Bypasses SimpleFOC State

**What We Do:**
```cpp
motor.shaft_angle = degreesToRadians(encoder_degrees);
```

**Hypothesis:** Directly writing motor.shaft_angle:
- Doesn't update internal SimpleFOC state variables
- Doesn't update velocity calculation state
- Doesn't update PID controller state
- First loopFOC() call uses stale state, causing wrong calculations

SimpleFOC may have internal variables like:
- Last position for velocity calculation
- PID integral/derivative terms
- Target tracking state

**Research Findings:**
- ❌ No examples found of directly setting motor.shaft_angle
- ⚠️ Standard approach is to let SimpleFOC manage shaft_angle entirely
- ⚠️ May need to call motor.sensor->update() after manual assignment

**Confidence:** MEDIUM - Not a standard SimpleFOC pattern

### Theory 5: motor.move() Expects Relative Motion, Not Absolute Position

**What We Do:**
```cpp
motor.torque_controller = TorqueControlType::angle;  // ⚠️ ERROR: This enum doesn't exist!
motor.controller = MotionControlType::angle_openloop;
motor.move(degreesToRadians(target_position_deg));  // Absolute position
```

**⚠️ CORRECTION (from Dev Log 5):** The above code contains an error. `TorqueControlType::angle` does NOT exist in SimpleFOC. Valid options are `voltage`, `dc_current`, or `foc_current`. The actual firmware code correctly uses `TorqueControlType::voltage` (verified in motor_control.cpp:357). This was theoretical/example code, not actual implementation.

**Hypothesis:** motor.move() in angle mode might expect:
- Relative change from current position
- NOT absolute target position
- Or position relative to sensor_offset

This would explain:
- Motor moving wrong direction
- Position errors
- Control loop fighting against encoder

**Research Findings:**
- ⚠️ SimpleFOC docs unclear about absolute vs relative
- ⚠️ Most examples use angle mode with absolute positions
- ✓ Official examples show: `motor.move(target_angle)` as absolute

**Confidence:** LOW - Examples suggest absolute positions are correct

---

## What We Need to Know

**Critical Missing Information:**
1. Actual numeric value of Direction::CCW (is it -1 or something else?)
2. What sensor_offset value was set during calibration?
3. What does motor.shaft_angle equal when motor is running (during loopFOC)?
4. What does angle_prev equal vs what getAngle() returns?
5. What does SimpleFOC's internal shaft_angle calculation actually produce?

**Next Step:** Add diagnostic logging to capture these exact values during a test run, rather than continuing to guess based on incomplete information.

---

## Code Changes This Session

### Correct SimpleFOC Pattern Implementation
```cpp
// Removed update() override entirely
// Only override getSensorAngle() - standard pattern

float MT6701Sensor::getSensorAngle() {
    cached_raw_count = encoder.readRawAngle();
    cached_degrees = rawToDegrees(cached_raw_count);
    cached_radians = degreesToRadians(cached_degrees);
    return cached_radians;
}
```

### Attempted Wraparound Fix (MADE THINGS WORSE)
```cpp
// Added getAngle() override to prevent full_rotations tracking
float MT6701Sensor::getAngle() override {
    return angle_prev;  // Ignore full_rotations for single-turn encoder
}
```

**Result:** Caused perfect sign inversion - shaft_angle = -encoder_reading

---

## Pattern Discovered: One Week of Confident Wrong Solutions

### What Went Wrong
1. **Week 1:** Assumed I2C failure → Added cache → Didn't fix problem
2. **Week 1:** Assumed need to call base Sensor::update() → Wrong pattern
3. **Week 1:** Assumed wraparound tracking was issue → Added getAngle() override → Made it WORSE
4. **Week 1:** Assumed CCW causes negative angles → Research found NO evidence

### User Feedback
> "please be less confident in your answers, it's been about a week where we've been confidently wrong"

### Required Approach Going Forward
- Build 3 systematic models with 98% confidence:
  1. What other people are doing that works
  2. What we are doing that doesn't work
  3. What our test signals actually do
- Verify theories against working implementations
- Use diagnostic logging to gather FACTS before proposing solutions
- Stop guessing and start measuring

---

## Git Commits

1. `84e9b47` - "Fix critical I2C failure handling - prevent shaft_angle reset to 0°"
   - Added _last_valid_angle cache (encoder was reading fine, this didn't help)

2. `f498b08` - "Fix SimpleFOC sensor integration - call base class Sensor::update()"
   - Wrong approach - not the standard SimpleFOC pattern

3. `c1963f5` - "Refactor MT6701Sensor to follow correct SimpleFOC pattern - don't override update()"
   - Correct pattern, but revealed ~313° tracking errors

4. `9e6e3fc` - "Add Dev Log 4: Document wraparound tracking issue"
   - Created this log (first version)

5. `e2684e3` - "Fix wraparound tracking for single-turn absolute encoder - override getAngle()"
   - Added getAngle() override to ignore full_rotations
   - **MADE THINGS WORSE - caused sign inversion**

6. (Not yet committed) - "Update Dev Log 4 with sign inversion observations and theories"
   - This update - documenting actual behavior vs theories

---

## References

### SimpleFOC Source Code Analyzed
- `Sensor::update()` - Base class wraparound tracking logic
- `Sensor::getAngle()` - Returns angle_prev + full_rotations × 2π
- `BLDCMotor::loopFOC()` - Calls sensor->getAngle() to update shaft_angle
- Formula: `shaft_angle = sensor_direction * LPF_angle(getAngle()) - sensor_offset`

### Working Implementations Researched
- SmartKnob (uses custom implementation, not directly comparable)
- Arduino-FOC-drivers MT6701 (I2C driver marked "incomplete")
- AS5600, AS5048A (other single-turn absolute encoders)
- Multiple SimpleFOC forum posts (both CW and CCW work fine)

### Key Finding
NO working implementation found with sign inversion issue when using sensor_direction = CCW

---

**End of Dev Log 4**
**Status:** BLOCKED - Need diagnostic data to build evidence-based understanding
