# SimpleFOC Implementation Review - Critical Issues Found

**Date:** 2026-01-08
**Reviewer:** Claude Code
**Status:** 🔴 MULTIPLE CRITICAL MISTAKES IDENTIFIED

---

## Executive Summary

After reviewing the dev logs and cross-referencing with SimpleFOC documentation and community resources, **several fundamental misunderstandings** about SimpleFOC's architecture have led to unnecessary complexity and potential bugs. The main issue is **not** with SimpleFOC or the MT6701 sensor, but with **incorrect implementation patterns**.

---

## Critical Mistakes

### 1. ❌ WRONG: "I2C is too slow for SimpleFOC" (Dev Log 1)

**Your Claim:**
> "SimpleFOC's automatic initFOC() calibration fails with MT6701 I2C sensors because I2C is too slow (~10-20ms per read)"

**Reality:**
- SimpleFOC **successfully supports I2C magnetic sensors** - it's a well-documented feature
- The [MagneticSensorI2C documentation](https://docs.simplefoc.com/magnetic_sensor_i2c) shows I2C sensors are fully supported
- I2C at 400kHz is fast enough for FOC loops (community reports ~10kHz loop rates)
- The real issue is **I2C bus lockups due to EMI**, not inherent speed limitations

**Evidence:**
- [SimpleFOC Community Discussion](https://community.simplefoc.com/t/bldcmotor-loopfoc-does-not-return-when-i2c-bus-locks-on-b-g431b-esc1/3577): "BLDCMotor.loopFOC() does not return when I2C bus locks" - the problem is EMI causing bus errors, not speed
- [GitHub Issue #172](https://github.com/simplefoc/Arduino-FOC/issues/172): The MT6701 calibration issue was caused by **hardware alignment problems** (magnet distance/centering), not I2C speed

**Recommendation:**
- Check for EMI from motor cables affecting I2C signals
- Verify magnet alignment (distance and centering are critical)
- Add I2C bus error detection/recovery instead of avoiding I2C entirely

---

### 2. ❌ WRONG: Manual `shaft_angle` Synchronization

**Your Code (motor_control.cpp:872, 944):**
```cpp
// CRITICAL FIX: Sync shaft_angle with sensor
encoder.update();
motor.shaft_angle = encoder.getSensorAngle();  // Force sync!
```

**Why This Is Wrong:**
- SimpleFOC **automatically manages `shaft_angle`** through `loopFOC()`
- Manually setting `shaft_angle` fights SimpleFOC's internal state machine
- This is a **workaround for a deeper problem**, not a fix

**What SimpleFOC Actually Does:**
From [BLDCMotor.cpp source](https://github.com/simplefoc/Arduino-FOC/blob/master/src/BLDCMotor.cpp):
```cpp
void BLDCMotor::loopFOC() {
    // update sensor - do this even in open-loop mode
    if (sensor) sensor->update();
    // ... SimpleFOC automatically updates shaft_angle from sensor
}
```

**Root Cause:**
When you provide `zero_electric_angle` to `initFOC()` (skipping alignment), SimpleFOC **does not** initialize `shaft_angle` from the sensor. This is a known quirk, but the solution is **not** to manually sync it everywhere - instead, initialize it **once** after successful `initFOC()`.

**Correct Pattern:**
```cpp
// After initFOC() succeeds (only once, not repeatedly!)
if (motor.initFOC(zero_electric_offset, sensor_dir) == 1) {
    motor.shaft_angle = motor.shaftAngle();  // Use SimpleFOC's method
}
```

---

### 3. ❌ MISLEADING: "LPF_angle.Tf = 0.0f" is a "CRITICAL FIX"

**Your Code (motor_control.cpp:381):**
```cpp
// CRITICAL FIX: Initialize angle low-pass filter (prevents shaft_angle reset bug)
motor.LPF_angle.Tf = 0.0f;  // No filtering for absolute encoders
```

**Reality:**
- Setting `LPF_angle.Tf = 0.0` is **the DEFAULT behavior** in SimpleFOC
- From [SimpleFOC LPF documentation](https://docs.simplefoc.com/low_pass_filter): "LPF_angle.Tf will in most cases remain equal to 0"
- This is **not a fix** - it's just explicitly setting the default value

**Why It's Not Solving Your Problem:**
- Your shaft_angle reset issue persists even with this "fix" (Dev Log 3)
- The real problem is elsewhere (see #4 below)

---

### 4. ❌ WRONG: Home Offset Coordinate System Layer

**Your Code (motor_control.cpp:273, 982-993, 1346):**
```cpp
float home_offset_rad;  // Software layer offset

void setHome() {
    home_offset_rad = current_sensor_angle;  // Store offset
}

void moveToPosition(float position_deg) {
    // user_position = absolute_position - home_offset
    motor.move(degreesToRadians(target_position_deg) + home_offset_rad);
}
```

**Why This Is Wrong:**
SimpleFOC examples for absolute encoders **DO NOT use home offsets** ([angle_control.ino](https://github.com/simplefoc/Arduino-FOC/blob/master/examples/motion_control/position_motion_control/magnetic_sensor/angle_control/angle_control.ino)):
- Absolute encoders retain position after power-off - no homing needed
- Work directly in absolute positions (0-360°)
- Home offsets add unnecessary coordinate system complexity

**Your Dev Log 3 Finding:**
> "SimpleFOC examples don't use sensor_offset for absolute encoders - work directly in absolute positions"

**You identified this as a problem but then kept the home offset code!** (Merge conflict on lines 263-274 shows it was removed then re-added)

**Recommendation:**
- Remove `home_offset_rad` entirely
- Work in absolute positions only
- If users want "home" positioning, handle it in the application layer, not motor control

---

### 5. ❌ WRONG: Merge Conflict in Production Code

**Your Code (motor_control.cpp:263-274):**
```cpp
<<<<<<< HEAD
      target_position_deg(0.0f) {
=======
      target_position_deg(0.0f),
      target_velocity_deg_s(0.0f),
      // ... more initializers
      home_offset_rad(0.0f) {
>>>>>>> parent of 4b2e193 (Strip out coordinate system complexity)
```

**Critical Issue:**
This code **will not compile**. The merge conflict markers are still in the source file.

**What This Reveals:**
- Code was not tested after the "fix"
- The simplification attempt (removing home_offset) was reverted by a bad merge
- You're running **different code** than what's in the repository

---

### 6. ❌ WRONG: Redundant Manual `encoder.update()` Calls

**Your Code (motor_control.cpp:1336):**
```cpp
void MotorController::update() {
    // NOTE: SimpleFOC automatically calls sensor->update() inside loopFOC()
    // Do NOT call encoder.update() manually here - it breaks the control loop!
    motor.loopFOC();
}
```

**Good:** This comment is correct! SimpleFOC's `loopFOC()` calls `sensor->update()` automatically.

**Bad:** But you're still calling `encoder.update()` manually in many places:
- Line 978 (setHome) - unnecessary
- Line 1272 (isAtTarget) - **THIS IS THE BUG!**

**The Shaft Angle Reset Bug (Dev Log 3):**

Your `isAtTarget()` function calls `encoder.update()` manually:
```cpp
bool MotorController::isAtTarget() {
    encoder.update();  // ← WRONG! This races with loopFOC()
    float current_position_deg = encoder.getDegrees();
}
```

**What Happens:**
1. `update()` loop calls `motor.loopFOC()` → sensor updates, shaft_angle calculated
2. `isAtTarget()` calls `encoder.update()` → sensor reads **new position**
3. SimpleFOC's `shaft_angle` now based on **old sensor data**
4. Next `loopFOC()` sees sensor jumped → thinks motor teleported → resets tracking

**Solution:**
```cpp
bool MotorController::isAtTarget() {
    // Do NOT call encoder.update() - loopFOC() already did it!
    // Use motor.shaft_angle, which is already synchronized
    float current_position_deg = radiansToDegrees(motor.shaft_angle);
    float position_error_deg = abs(current_position_deg - target_position_deg);
    return (position_error_deg < position_tolerance_deg);
}
```

---

### 7. ❌ ARCHITECTURAL ISSUE: Distrust of SimpleFOC Tracking

**Your Code Pattern:**
```cpp
float getAbsolutePositionDeg() {
    // "THIS IS TRUTH" - bypass SimpleFOC
    return encoder.getDegrees();
}

float getCurrentPositionDeg() {
    // "WARNING: This may lag or drift!"
    return radiansToDegrees(motor.shaft_angle);
}
```

**Problem:**
You're treating SimpleFOC's `shaft_angle` as untrustworthy and reading the encoder directly for "truth". This suggests:
- You don't trust SimpleFOC's sensor integration
- You're working **against** SimpleFOC instead of **with** it
- Your manual encoder reads race with SimpleFOC's internal reads

**Correct Approach:**
- Trust `motor.shaft_angle` as the position source during motor operation
- SimpleFOC is designed to track position accurately - if it's not working, fix the **root cause**, don't bypass it
- Direct encoder reads should only be used for diagnostics, not control decisions

---

## Root Cause Analysis: Why `shaft_angle` Resets to Zero

Based on Dev Log 3, your symptom is:
> "After enable(): shaft_angle = 298.48° ✓
> During movement: shaft_angle = 0.00° ❌"

**The REAL Problem:**

1. **Race condition between manual `encoder.update()` calls and SimpleFOC's internal updates**
2. **Manual `shaft_angle` synchronization fighting SimpleFOC's tracking**
3. **Home offset coordinate transformation confusing the control loop**

**Why It Manifests:**
```cpp
// Your update loop:
void update() {
    motor.loopFOC();  // SimpleFOC reads sensor, updates shaft_angle
    motor.move(...);  // SimpleFOC calculates error, applies voltage

    if (isAtTarget()) {  // ← YOU ARE HERE
        encoder.update();  // ← Reads sensor AGAIN (new position!)
        // SimpleFOC's shaft_angle is now STALE (based on old reading)
        // Next loopFOC() sees huge position jump, loses tracking
    }
}
```

---

## Correct SimpleFOC Pattern for Absolute Encoders

Here's how SimpleFOC **should** be used with absolute I2C encoders:

```cpp
// 1. SETUP - ONE TIME
void setup() {
    sensor.init();
    motor.linkSensor(&sensor);
    driver.init();
    motor.linkDriver(&driver);

    // Configure motor (voltage/current/velocity limits, PID, etc.)
    motor.voltage_limit = 6.0;
    motor.velocity_limit = degreesToRadians(360);
    motor.PID_velocity.P = 0.5;
    motor.P_angle.P = 20.0;
    // Note: LPF_angle.Tf defaults to 0, no need to set it

    motor.init();

    // Manual calibration (if auto-calibration fails)
    motor.voltage_sensor_align = 6.0;
    motor.zero_electric_angle = calculateZeroElectricAngle();  // Your method is fine
    motor.sensor_direction = calculateDirection();

    // Initialize FOC with known calibration
    if (motor.initFOC(motor.zero_electric_angle, motor.sensor_direction) == 1) {
        // SUCCESS - SimpleFOC now tracking position
        motor.enable();
    }
}

// 2. LOOP - EVERY ITERATION
void loop() {
    // SimpleFOC does EVERYTHING - don't interfere!
    motor.loopFOC();  // Updates sensor, calculates FOC, applies voltages
    motor.move(target_angle_rad);  // Position control

    // NO manual encoder.update() calls!
    // NO manual shaft_angle modifications!
    // Just trust SimpleFOC to do its job

    // Read position from motor.shaft_angle (already updated by loopFOC)
    float current_pos = motor.shaft_angle;

    // Check if at target using SIMPLEFOC's data
    if (abs(current_pos - target_angle_rad) < tolerance) {
        // Target reached
    }
}
```

**Key Principles:**
1. **Single source of truth:** `motor.shaft_angle` (updated by loopFOC)
2. **No manual sensor reads** during motor operation
3. **No coordinate system translations** (work in absolute radians)
4. **Trust SimpleFOC's integration** - if it's broken, fix the root cause

---

## Recommendations

### Immediate Actions

1. **Fix the merge conflict** (motor_control.cpp:263-274) - the code won't compile

2. **Remove home offset implementation entirely:**
   - Delete `home_offset_rad` member variable
   - Remove coordinate transformations in `moveToPosition()`, `isAtTarget()`, etc.
   - Work in absolute positions only (0-2π radians)

3. **Fix the race condition in `isAtTarget()`:**
   ```cpp
   bool MotorController::isAtTarget() {
       // Use SimpleFOC's shaft_angle (already updated by loopFOC)
       float current_rad = motor.shaft_angle;
       float error_rad = abs(current_rad - degreesToRadians(target_position_deg));
       return (error_rad < degreesToRadians(position_tolerance_deg));
   }
   ```

4. **Remove manual `shaft_angle` synchronization:**
   - Keep the ONE initialization after `initFOC()` succeeds
   - Remove all other manual `motor.shaft_angle = ...` assignments

5. **Simplify encoder interface:**
   - Only expose `getEncoderDegrees()` for diagnostics
   - Use `motor.shaft_angle` for all control decisions

### Hardware Checks

6. **Check for I2C EMI issues:**
   - Move I2C wires away from motor power cables
   - Add pull-up resistors to SDA/SCL if not present
   - Use shielded cables if possible
   - Consider reducing I2C speed if 400kHz is unstable

7. **Verify MT6701 sensor alignment:**
   - Magnet must be **precisely centered** over sensor
   - Distance should be within sensor spec (~0.5-3mm for most)
   - Check magnetic field strength register (you're already doing this - good!)

### Testing Strategy

8. **Start with minimal example:**
   ```cpp
   void loop() {
       motor.loopFOC();
       motor.move(0);  // Just try to hold position 0

       Serial.print("shaft_angle: ");
       Serial.println(motor.shaft_angle);
       delay(100);
   }
   ```
   If `shaft_angle` tracks position correctly in this minimal test, your issue is in the added complexity.

9. **Test without home offset:**
   - Command absolute positions (e.g., move to π radians = 180°)
   - Verify motor moves to correct position
   - Check `motor.shaft_angle` stays synchronized

---

## Technical Debt to Address

From Dev Log 3:
> "Still have home_offset code in multiple places"
> "Complex diagnostic functions comparing 3 coordinate systems"
> "PID auto-tuner code (~500 LOC) we're not using"
> "Multiple control modes (velocity, torque) we're not testing"

**Recommendation:**
After fixing the immediate bugs, do an aggressive simplification pass:
- Remove unused features (velocity mode, torque mode if not needed)
- Remove PID auto-tuner if not used
- Simplify to single coordinate system (absolute positions)
- Remove diagnostic code that compares multiple position sources

**Principle:** Keep only what you actually use and test.

---

## Sources & References

### SimpleFOC Documentation
- [Magnetic Sensor I2C](https://docs.simplefoc.com/magnetic_sensor_i2c)
- [Position Control Loop](https://docs.simplefoc.com/angle_loop)
- [Low Pass Filter](https://docs.simplefoc.com/low_pass_filter)
- [FOC Algorithm Implementation](https://docs.simplefoc.com/foc_implementation)
- [Sensor Support](https://docs.simplefoc.com/sensor_support)

### SimpleFOC Community Discussions
- [MT6701 I2C Integration](https://community.simplefoc.com/t/magnetic-sensor-mt6701-i2c-integration/4919)
- [I2C Bus Lockup Issues](https://community.simplefoc.com/t/bldcmotor-loopfoc-does-not-return-when-i2c-bus-locks-on-b-g431b-esc1/3577)
- [MagneticSensor I2C Communication](https://community.simplefoc.com/t/magneticsensor-i2c-communication-implementation/44)
- [shaft_angle Always Zero](https://community.simplefoc.com/t/sensor-getangle-works-correctly-but-motor-shaft-angle-is-always-zero-mot-init-foc-failed-as-a-result/5357)

### GitHub Issues
- [MT6701 Calibration Issue #172](https://github.com/simplefoc/Arduino-FOC/issues/172)
- [BLDCMotor.cpp Source](https://github.com/simplefoc/Arduino-FOC/blob/master/src/BLDCMotor.cpp)
- [Angle Filtering Issue #302](https://github.com/simplefoc/Arduino-FOC/issues/302)

### SimpleFOC Examples
- [Angle Control Example](https://github.com/simplefoc/Arduino-FOC/blob/master/examples/motion_control/position_motion_control/magnetic_sensor/angle_control/angle_control.ino)
- [Bluepill Position Control](https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/Bluepill_examples/magnetic_sensor/bluepill_position_control/bluepill_position_control.ino)

---

## Bottom Line

Your hardware is likely **fine**. Your manual calibration approach is **correct**. The problems are:

1. ❌ **Race conditions** from manual encoder reads during motor operation
2. ❌ **Coordinate system complexity** from unnecessary home offset layer
3. ❌ **Fighting SimpleFOC** instead of working with its architecture
4. ❌ **Merge conflict** preventing compilation

**Fix these patterns, and your motor control will work.**

The SimpleFOC library is mature and well-tested with I2C absolute encoders. Trust its architecture, follow its patterns, and keep your implementation simple.
