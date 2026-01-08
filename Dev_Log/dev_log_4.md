# Dev Log 4: SimpleFOC Integration Debugging - Wraparound Tracking Issue

**Date:** 2026-01-08
**Session Focus:** Fixing MT6701 sensor integration with SimpleFOC after implementing correct sensor pattern
**Status:** ⚠️ PARTIAL SUCCESS - Motor moves but tracking fails at 0°/360° boundary

---

## Executive Summary

**MAJOR PROGRESS:** Motor is now responding to position commands and moving dynamically! Multiple movements observed during test sequence. However, SimpleFOC's position tracking breaks during movement, showing ~360° discrepancy between encoder reading and shaft_angle. This causes oscillation and overshoot as the control loop fights incorrect position feedback.

**Current Behavior:**
- ✅ Motor moves multiple times (jerks back and forth)
- ✅ Initial tracking works: `shaft_angle` matches `encoder` at rest
- ❌ Tracking breaks during movement: 313° error appears
- ❌ Motor overshoots target by ~44° (moved 74° instead of 30°)
- ❌ Control loop oscillates trying to correct phantom position error

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

## Test Results Analysis

### Test Sequence: 30° Position Move

**Command:** Move from 255.59° → 285.59° (absolute, +30°)

**Initial State (GOOD):**
```
Encoder (abs): 255.67° | SimpleFOC shaft_angle (abs): 255.67° | Target: 0.00°
Tracking error (abs): 0.00° ✓
```

**During Movement (BROKEN):**
```
Sample 1:
  Encoder: 336.53° | shaft_angle: 23.47° | Target: 285.59°
  Tracking error: 313.07° ❌

Sample 2:
  Encoder: 351.74° | shaft_angle: 8.26° | Target: 285.59°
  Tracking error: 343.48° ❌

Sample 3:
  Encoder: 298.65° | shaft_angle: 61.35° | Target: 285.59°
  Tracking error: 237.30° ❌

Sample 4:
  Encoder: 336.97° | shaft_angle: 23.03° | Target: 285.59°
  Tracking error: 313.95° ❌
```

**Final State:**
```
Encoder: 329.77° | shaft_angle: 26.89° | Target: 285.59°
Tracking error: 302.87° ❌

Moved: 74.18° (expected ~30°)
Error: 44.18° (expected <3°)
```

### Observed Motor Behavior

**Physical Movement:**
- Motor jerks forward
- Jerks backward
- Pauses briefly
- Jerks forward again (different pattern)
- Repeats oscillation ~5-6 times
- Eventually stops (timeout)

**Interpretation:** Motor is hunting/oscillating because SimpleFOC thinks the position is ~300° away from where the encoder says it actually is. The control loop is trying to correct a phantom position error.

---

## Root Cause Analysis

### Hypothesis: Wraparound Tracking Error

**The Pattern:**
```
encoder: 336.53° | shaft_angle: 23.47°
Difference: 336.53 - 23.47 = 313.06°

But also: 360° - 313.06° = 46.94°
```

This suggests `shaft_angle` is on the opposite side of the 0°/360° boundary from the encoder reading.

**Possible Cause:** SimpleFOC's `Sensor::update()` wraparound detection logic:
```cpp
if(abs(d_angle) > (0.8f*_2PI))  // If change >~288°
    full_rotations += (d_angle > 0) ? -1 : 1;
```

For a single-turn absolute encoder:
- `full_rotations` should ALWAYS be 0
- Wraparound should be handled differently
- When encoder goes 359° → 1°, this is NOT a full rotation - it's just crossing the boundary

**Hypothesis:** The base class is incrementing `full_rotations` when the motor crosses 0°/360°, causing `shaft_angle` to be calculated incorrectly:
```cpp
// Likely in BLDCMotor.cpp
shaft_angle = sensor.getAngle();  // Gets angle_prev from base class
// angle_prev includes full_rotations offset!
```

### Why This Manifests Now

**Previous implementations:**
- Overrode `update()` → bypassed base class wraparound tracking
- Never called base class `update()` correctly → `full_rotations` stayed at 0
- Broken in other ways, but accidentally avoided this bug

**Current implementation:**
- Follows correct SimpleFOC pattern
- Base class `update()` runs properly
- Wraparound detection triggers when crossing 0°/360°
- For absolute single-turn encoder, this is INCORRECT behavior

---

## Evidence From Logs

### Velocity Calculation Insanity
```
Shaft Velocity: -341422.63°/s
```

This is physically impossible. Calculated from incorrect `angle_prev` due to `full_rotations` error.

### Position Jump Pattern
```
shaft_angle sequence: 23.47° → 8.26° → 61.35° → 23.03°
encoder sequence: 336.53° → 351.74° → 298.65° → 336.97°
```

Both are oscillating, but shaft_angle is consistently ~300° behind encoder (i.e., ~60° ahead when accounting for wraparound).

### Heartbeat Shows Correct Tracking After Movement
```
[HEARTBEAT] MT6701 Encoder: Pos=30.4° | SimpleFOC: Pos=30.4° (diff=0.0°) ✓
```

After motor stops and position settles, tracking is correct again. This suggests the issue is specifically during boundary crossing while motor is active.

---

## Key Insights

### 1. Motor Hardware Works ✅
- Open-loop movement confirmed
- Driver phases responding
- Encoder reading correctly
- I2C communication stable during movement

### 2. SimpleFOC Integration Partially Works ✅
- Sensor linked to motor
- loopFOC() running
- Position commands accepted
- Control loop active (though fighting phantom error)

### 3. Absolute Encoder Needs Special Handling ❌
SimpleFOC's base `Sensor` class is designed for:
- Incremental encoders (track full rotations)
- Multi-turn absolute encoders (track full rotations)

Our MT6701 is:
- Single-turn absolute encoder
- Does NOT track full rotations
- Position wraps at 0°/360° boundary
- `full_rotations` should ALWAYS be 0

---

## Next Steps

### Option A: Override getAngle() Instead of update()

SimpleFOC's `Sensor` base class has:
```cpp
float getAngle() {
    return angle_prev + full_rotations * _2PI;
}
```

We could override this to ignore `full_rotations`:
```cpp
float MT6701Sensor::getAngle() override {
    return angle_prev;  // Ignore full_rotations
}
```

**Pros:**
- Minimal change
- Keeps base class update() logic
- Only affects angle calculation

**Cons:**
- Still using base class update() which modifies full_rotations
- May have side effects we don't understand

### Option B: Override update() But Handle Wraparound Correctly

```cpp
void MT6701Sensor::update() override {
    float val = getSensorAngle();
    if (val < 0) return;
    angle_prev_ts = _micros();
    // DO NOT track full rotations for single-turn absolute encoder
    // Just store the angle directly
    angle_prev = val;
    // full_rotations stays at 0
}
```

**Pros:**
- Full control over tracking logic
- Explicitly handles single-turn absolute encoder case
- Clear intent

**Cons:**
- Goes against SimpleFOC pattern (only HallSensor does this)
- Duplicates base class code
- May miss future SimpleFOC updates

### Option C: Disable Wraparound Detection

Modify base class call to prevent wraparound tracking:
```cpp
void MT6701Sensor::update() override {
    // Save current state
    int32_t saved_rotations = full_rotations;

    // Call base class
    Sensor::update();

    // Restore rotation count (always 0 for single-turn)
    full_rotations = 0;
}
```

**Pros:**
- Uses base class logic for most things
- Only overrides the problematic behavior
- Easy to understand

**Cons:**
- Hacky
- Relies on implementation details
- May break with SimpleFOC updates

### Option D: Research How Others Handle This

Search for:
- Other single-turn absolute encoder implementations
- SimpleFOC configuration options for absolute encoders
- Whether there's a `needsAbsoluteTracking()` or similar flag

---

## Questions for Next Session

1. **How do other SimpleFOC absolute encoders prevent full_rotations tracking?**
   - AS5600 (12-bit, single-turn)
   - AS5048A (14-bit, single-turn)
   - Do they override update() or getAngle()?

2. **Is there a SimpleFOC configuration for single-turn absolute encoders?**
   - Sensor class flags?
   - Motor controller settings?

3. **Why does tracking work at rest but fail during movement?**
   - Is it only when crossing 0°/360°?
   - Or is it high-speed I2C read latency causing stale readings?

4. **What's the correct relationship between:**
   - `angle_prev` (from Sensor base class)
   - `shaft_angle` (from BLDCMotor)
   - `encoder reading` (ground truth)

---

## Commits This Session

1. `84e9b47` - Fix critical I2C failure handling - prevent shaft_angle reset to 0°
   - Added `_last_valid_angle` cache to MT6701.cpp
   - Return cached value on I2C failure instead of 0

2. `f498b08` - Fix SimpleFOC sensor integration - call base class Sensor::update()
   - Added `Sensor::update()` call to MT6701Sensor::update()
   - This was WRONG - not the standard pattern

3. `c1963f5` - Refactor MT6701Sensor to follow correct SimpleFOC pattern
   - **REMOVED update() override entirely**
   - Moved I2C read into getSensorAngle()
   - Let base class handle all tracking
   - **This revealed the wraparound tracking bug**

---

## Technical Debt & Improvements

### Immediate
- [ ] Fix wraparound tracking for single-turn absolute encoder
- [ ] Test position moves that cross 0°/360° boundary explicitly
- [ ] Verify `full_rotations` stays at 0 during operation

### Short-term
- [ ] Research how AS5600/AS5048A handle this in SimpleFOC
- [ ] Add boundary crossing test to motor_test sequence
- [ ] Log `full_rotations` value in diagnostics

### Long-term
- [ ] Consider contributing MT6701 driver to Arduino-FOC-drivers
- [ ] Document absolute encoder integration patterns for SimpleFOC
- [ ] Add wraparound handling configuration to MotorController

---

## Success Metrics

**Current Status:** 40% - Motor moves, tracking partially works

**Next Milestone:** 70% - Motor reaches target position within tolerance
- Fix wraparound tracking
- Achieve <3° position error
- Eliminate oscillation

**Final Goal:** 100% - Production-ready motor control
- Reliable position control
- Smooth movements
- Repeatable accuracy
- Multi-turn capability (if needed)

---

## Lessons Learned

### ✅ What Worked
1. **Research-driven debugging** - Examining other implementations revealed correct pattern
2. **Following SimpleFOC conventions** - Removed custom update() override
3. **Systematic testing** - Test sequence caught the wraparound issue clearly
4. **Detailed logging** - Tracking error diagnostics made the problem obvious

### ❌ What Didn't Work
1. **Assuming SimpleFOC base class works for all absolute encoders** - It's designed for incremental/multi-turn
2. **Following patterns blindly** - Standard pattern revealed edge case for single-turn encoders
3. **Not testing boundary crossings explicitly** - Would have caught this sooner

### 🎓 Key Insight
**SimpleFOC's Sensor base class assumes sensors either:**
- Track full rotations (incremental encoders)
- Benefit from rotation tracking (multi-turn absolute encoders)

**Single-turn absolute encoders are a special case:**
- Position wraps at 0°/360° boundary
- `full_rotations` should ALWAYS be 0
- Wraparound detection logic doesn't apply
- Need special handling or override

This is likely why the official MT6701 I2C driver in Arduino-FOC-drivers is marked "incomplete" - they probably hit this same issue!

---

## References

### SimpleFOC Source Code
- `Sensor::update()` - [Arduino-FOC/src/common/base_classes/Sensor.cpp](https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/base_classes/Sensor.cpp)
- `BLDCMotor::loopFOC()` - [Arduino-FOC/src/BLDCMotor.cpp](https://github.com/simplefoc/Arduino-FOC/blob/master/src/BLDCMotor.cpp)
- `MagneticSensorI2C` - [Arduino-FOC/src/sensors/MagneticSensorI2C.cpp](https://github.com/simplefoc/Arduino-FOC/blob/master/src/sensors/MagneticSensorI2C.cpp)

### Other Implementations
- SmartKnob MT6701 - [scottbez1/smartknob](https://github.com/scottbez1/smartknob/blob/master/firmware/src/mt6701_sensor.cpp)
- Arduino-FOC-drivers MT6701 - [simplefoc/Arduino-FOC-drivers](https://github.com/simplefoc/Arduino-FOC-drivers/tree/master/src/encoders/mt6701)

### Documentation
- [SimpleFOC Sensor Support](https://docs.simplefoc.com/sensor_support)
- [SimpleFOC Generic Sensor](https://docs.simplefoc.com/generic_sensor)

---

**End of Dev Log 4**
**Next Session:** Fix single-turn absolute encoder wraparound tracking
