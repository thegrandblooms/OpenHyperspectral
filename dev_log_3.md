# Dev Log 3: Debugging SimpleFOC Position Control with Absolute Encoders

**Session Date:** 2026-01-08
**Branch:** `claude/debug-pid-motor-test-JQlTi`
**Status:** 🔴 UNRESOLVED - shaft_angle still resets to 0° during movement

---

## Problem Statement

Motor position control completely broken:
- Motor moves slightly (~40-50°) when commanded
- SimpleFOC `shaft_angle` resets to **0.00°** during movement
- Encoder reads correctly (absolute position 0-360°)
- Position tracking completely lost after first `motor.move()` call

**Critical Symptom:**
```
After enable():     shaft_angle = 298.48° ✓
After setHome():    shaft_angle = 298.48° ✓
After move(30°):    shaft_angle = 298.48° ✓
During movement:    shaft_angle = 0.00°    ❌ RESET TO ZERO!
```

---

## Troubleshooting Timeline

### Attempt #1: Software Offset Layer (FAILED)

**Hypothesis:** SimpleFOC's `sensor_offset` breaks with absolute encoders when wrapping 0°/360°

**Implementation:**
```cpp
class MotorController {
    float home_offset_rad;  // Software layer offset
};

void setHome() {
    home_offset_rad = encoder.getSensorAngle();  // Store in our code
    // Don't touch motor.sensor_offset
}

void moveToPosition(float user_deg) {
    motor.move(degreesToRadians(user_deg) + home_offset_rad);
}
```

**Result:** ❌ FAILED
- shaft_angle still reset to 0° during movement
- Created 3 coordinate systems (encoder absolute, SimpleFOC absolute, user relative)
- Added unnecessary complexity (~50 LOC)

**Conclusion:** Not a sensor_offset issue - something else resetting shaft_angle

---

### Attempt #2: Research SimpleFOC Examples

**Findings from official SimpleFOC code:**

1. **[angle_control.ino](https://github.com/simplefoc/Arduino-FOC/blob/master/examples/motion_control/position_motion_control/magnetic_sensor/angle_control/angle_control.ino):**
   - Does NOT use `sensor_offset`
   - Does NOT implement "home" positioning
   - Works directly in absolute positions (0-2π radians)
   - Simple pattern: `sensor.init()` → `motor.initFOC()` → `motor.move(target)`

2. **[bluepill_position_control.ino](https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/Bluepill_examples/magnetic_sensor/bluepill_position_control/bluepill_position_control.ino):**
   - Same pattern - no sensor_offset, no home offset
   - Absolute encoders don't need homing (retain position after power-off)

3. **[BLDCMotor.cpp initFOC()](https://github.com/simplefoc/Arduino-FOC/blob/master/src/BLDCMotor.cpp):**
   - After sensor alignment: `shaft_angle = shaftAngle();`
   - Comment says: "added the shaft_angle update"
   - SimpleFOC DOES initialize shaft_angle from sensor

4. **shaftAngle() calculation:**
   ```cpp
   float FOCMotor::shaftAngle() {
       return sensor_direction * LPF_angle(sensor->getAngle()) - sensor_offset;
   }
   ```
   - Uses `LPF_angle` (low-pass filter) on position
   - We initialize `LPF_velocity` but never `LPF_angle`!

**Key Insight:** Official examples never modify `sensor_offset` and work in absolute coordinates only.

---

### Attempt #3: Strip Out Complexity

**Action:** Removed software offset layer, reverted to SimpleFOC standard pattern

**Changes:**
- Removed `home_offset_rad` member variable
- Removed coordinate translation code
- Made `setHome()` a no-op (just logs current position)
- Work only in absolute positions (0-360°)

**Diff:** -50 LOC, removed 3rd coordinate system

**Result:** ❌ Still didn't fix shaft_angle reset issue

---

### Attempt #4: Initialize LPF_angle Filter (HYPOTHESIS)

**Discovery:** We never initialized `motor.LPF_angle.Tf`!

**Our code before fix:**
```cpp
void begin() {
    // ... setup ...
    motor.LPF_velocity.Tf = PID_LPF_VELOCITY;  // ✓ Initialized
    // motor.LPF_angle.Tf = ???                // ✗ NEVER SET!
}
```

**The Fix:**
```cpp
void begin() {
    motor.LPF_angle.Tf = 0.0f;  // No filtering for absolute encoders
}
```

**Commit:** `b1d7f3e` - "CRITICAL FIX: Initialize motor.LPF_angle.Tf to prevent shaft_angle reset"

**Result:** ⚠️ INCOMPLETE - shaft_angle STILL resets to 0° during movement!

---

## Current Status: Still Broken

**Latest test output:**
```
After enable():        shaft_angle = 298.48° ✓
After setHome():       shaft_angle = 298.48° ✓
After move(30°):       shaft_angle = 298.48° ✓ (initial)
During movement:       shaft_angle = 0.00°    ❌
Encoder reads:         342.16° ✓ (moved 44° - motor IS moving)
```

**What's working:**
- ✅ Encoder reads absolute position correctly
- ✅ Motor physically moves (~44° in this test)
- ✅ SimpleFOC initFOC() initializes shaft_angle correctly
- ✅ shaft_angle stays synchronized until motor.move() is called

**What's broken:**
- ❌ shaft_angle resets to 0.00° during motor.loopFOC() or motor.move()
- ❌ Position tracking completely lost
- ❌ Motor can't reach target (thinks it's at 0° when it's at 342°)

---

## Hypotheses for Next Session

### Hypothesis A: LPF_angle initialization timing
- Maybe LPF_angle.Tf needs to be set BEFORE motor.init()?
- Or after motor.initFOC()?
- Check initialization order

### Hypothesis B: Something resetting shaft_angle in update loop
```cpp
void update() {
    motor.loopFOC();  // ← Does this reset shaft_angle?
    motor.move(target);  // ← Or this?
}
```

### Hypothesis C: Home offset code still active
- Old setHome() code might still be setting motor.sensor_offset
- Need to verify what's actually running on device
- Check if we have stale compiled code

### Hypothesis D: SimpleFOC angle wrapping issue
- When encoder wraps past 0°/360°, SimpleFOC might reset
- Motor moved from 298° to 342° (crossed 360°/0° boundary? No, actually didn't)
- But shaft_angle went to 0° anyway

### Hypothesis E: Missing sensor update
- SimpleFOC might need sensor->update() called manually
- Our code comment says "Do NOT call encoder.update() manually"
- But maybe we need to?

---

## Key Code Locations

**Motor initialization:** `motor_control.cpp:266-431` (begin() function)
**FOC calibration:** `motor_control.cpp:433-460` (calibrate() function)
**Update loop:** `motor_control.cpp:1294-1354` (update() function)
**Position control:** `motor_control.cpp:1321-1327` (motor.move() call)

**Critical config:**
```cpp
// motor_control.cpp:363-370
motor.P_angle.P = PID_P_POSITION;      // 20.0
motor.P_angle.I = PID_I_POSITION;      // 0.0
motor.P_angle.D = PID_D_POSITION;      // 0.0
motor.P_angle.output_ramp = PID_RAMP_POSITION;
motor.P_angle.limit = degreesToRadians(MAX_VELOCITY_DEG);
motor.LPF_angle.Tf = 0.0f;  // ← THE FIX (didn't work yet)
```

---

## Research Sources

- [SimpleFOC Magnetic Sensor I2C Docs](https://docs.simplefoc.com/magnetic_sensor_i2c)
- [SimpleFOC Angle Control Example](https://github.com/simplefoc/Arduino-FOC/blob/master/examples/motion_control/position_motion_control/magnetic_sensor/angle_control/angle_control.ino)
- [SimpleFOC Bluepill Position Control](https://github.com/simplefoc/Arduino-FOC/blob/master/examples/hardware_specific_examples/Bluepill_examples/magnetic_sensor/bluepill_position_control/bluepill_position_control.ino)
- [SimpleFOC BLDCMotor.cpp Source](https://github.com/simplefoc/Arduino-FOC/blob/master/src/BLDCMotor.cpp)

---

## Session End Status

**Confirmed:**
- ✅ Motor physically moves (encoder shows 44° movement)
- ✅ Encoder reads correctly (absolute position tracking works)
- ✅ `motor.LPF_angle.Tf = 0.0f` is initialized in code
- ❌ shaft_angle still resets to 0.00° during movement
- ⚠️  home_offset code still active (merge conflict brought it back)

**The fix didn't work.** Initializing LPF_angle.Tf alone was not sufficient.

## Next Steps for Future Session

1. **Remove home_offset layer completely**
   - This is masking the real issue
   - SimpleFOC examples don't use home offsets
   - Work directly in absolute positions only

2. **Check SimpleFOC source code**
   - Where does shaft_angle get reset?
   - Look at loopFOC() and move() implementations
   - Check if there's a known issue with I2C sensors

3. **Try minimal SimpleFOC example**
   - Strip to bare bones: sensor.init(), motor.initFOC(), motor.move()
   - Remove all our custom code
   - See if basic example works

4. **Consider asking SimpleFOC community**
   - Post on SimpleFOC forum with test results
   - This specific symptom (shaft_angle → 0° during movement) might be known

---

## Lessons Learned

1. **SimpleFOC examples don't use sensor_offset for absolute encoders** - work directly in absolute positions
2. **Coordinate system translation adds complexity** - only add if truly needed
3. **Uninitialized filter parameters can break tracking** - check ALL SimpleFOC object initializations
4. **Motor moving ≠ position control working** - motor can move but SimpleFOC can lose tracking
5. **Need better debugging** - should log SimpleFOC internal state during movement

---

## Technical Debt Created

- Still have home_offset code in multiple places (setHome, moveToPosition, isAtTarget, etc.)
- Complex diagnostic functions comparing 3 coordinate systems
- PID auto-tuner code (~500 LOC) we're not using
- Multiple control modes (velocity, torque) we're not testing
- State machine complexity we don't need

**Recommendation:** After fixing position control, do aggressive simplification pass.
