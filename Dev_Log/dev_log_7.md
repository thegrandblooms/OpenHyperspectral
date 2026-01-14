# Dev Log 7: SimpleFOC Position Control Deep Dive

**Date:** 2026-01-14
**Purpose:** Compare with SmartKnob implementation, debug motor movement issues, investigate position control behavior
**Status:** Active debugging - Motor still not moving reliably despite fixes

---

## Session Overview

This session focused on comparing our SimpleFOC position control implementation with SmartKnob's approach and debugging why the motor fails to move to commanded positions. Initial hypothesis was angle wraparound bug, but deeper investigation reveals the issue is more fundamental.

---

## Initial Problem Statement

Motor exhibiting multiple failure modes:
1. **Not moving at all** - Commands like `m 50`, `m 100`, `m 150` produce no movement
2. **Moving backwards** - `m 100` from 89° moved to 79° (wrong direction!)
3. **Stopping early** - `m 50` from 103° moved to 89° instead of 50°
4. **Rapid oscillation** - Motor vibrates/hunts before settling (when it moves at all)

**Key Diagnostic Data:**
```
Command: m 100 (from 89.38°)
Expected: Move +11° to 100°
Actual: Moved -10° to 79.63° (BACKWARDS!)

Command: m 150 (from 79.63°)
Expected: Move +70° to 150°
Actual: No movement at all
```

---

## Success #1: Serial Output Optimization

**Problem:** Debug output too verbose (~18 lines per diagnostic), making patterns hard to see

**Solution:** Condensed `logMotorState()` from 18 lines to 2 lines:
```
Before (18 lines):
[DIAG] After enable()
  Encoder (abs): 274.66° | SimpleFOC shaft_angle (abs): 274.66° ...
  Enabled: Y | FOC enabled: Y | Has sensor: Y | Calibrated: Y
  Tracking error (abs): 0.02° (should be <5° for good tracking)
  --- SimpleFOC Internals ---
  Target angle (shaft_angle_sp): 304.66° (5.3179 rad)
  ...

After (2 lines):
[DIAG] After enable() | Enc:274.7° FOC:274.7° Tgt:304.7° | En:Y Cal:Y | TrkErr:0.02°
       PIDerr:30.0° VelCmd:0.0°/s VelAct:0.0°/s | Vq:6.00V Vd:0.00V
```

**Status:** ✅ Successful - Much easier to spot patterns

**Files Modified:** `tests.cpp:21-62` (logMotorState), `tests.cpp:402-536` (runMotorTest)

---

## Investigation #1: SimpleFOC Usage Verification

**Question:** Are we using SimpleFOC correctly as a "servo"?

### Comparison with SmartKnob

| Aspect | SmartKnob | OpenHyperspectral | Verdict |
|--------|-----------|-------------------|---------|
| **Control Mode** | `MotionControlType::torque` | `MotionControlType::angle` | Different but both valid |
| **motor.move()** | Torque value (manual) | Target angle (automatic) | Different approaches |
| **Position PID** | Manual calculation | SimpleFOC built-in | We use SimpleFOC's servo mode |
| **Angle Normalization** | Manual before control | None (SimpleFOC handles it?) | Potential issue |

**Key Insight:** SmartKnob uses **manual torque control** with custom PID, while we use **SimpleFOC's automatic position control**. Both are valid patterns, but SimpleFOC's angle mode has different requirements.

### SimpleFOC Documentation Verification

**Our Implementation:**
```cpp
// Control loop structure (motor_control.cpp:1398-1443)
motor.loopFOC();                              // FOC commutation (fast)
motor.move(degreesToRadians(target_position_deg));  // Position control
```

**PID Parameters:**
```cpp
// Position P controller
motor.P_angle.P = 20.0;           // ✅ Matches SimpleFOC example (20)
motor.P_angle.I = 0.0;
motor.P_angle.D = 0.0;

// Velocity PI controller
motor.PID_velocity.P = 0.2;       // ✅ Matches SimpleFOC example (0.2)
motor.PID_velocity.I = 20.0;      // ✅ Matches SimpleFOC example (20)

// Limits
motor.voltage_limit = 6.0;        // ✅ Standard for gimbal motors
motor.velocity_limit = 10.0 rad/s; // ✅ Reasonable for position control
```

**Verdict:** ✅ **We ARE using SimpleFOC correctly** - Control structure and parameters match official examples

---

## Discovery #1: Angle Wraparound Bug Hypothesis

**User provided critical data showing backwards movement:**

```
Command: m 100 (from 89°)
Result: Motor moved to 79° (backwards!)

Command: m 50 (from 103°)
Result: Motor moved to 89°, stopped early

Command: m 0 (from ~68°)
Result: Motor moved and oscillated rapidly around target
```

**Hypothesis:** SimpleFOC's position PID doesn't normalize angle errors for absolute encoders.

**Root Cause Analysis:**

SimpleFOC calculates position error as:
```cpp
// SimpleFOC internal (BLDCMotor.cpp)
error = shaft_angle_sp - shaft_angle;  // NO NORMALIZATION!
```

This works for **incremental encoders** where `shaft_angle` can be multi-turn (0° → 360° → 720° → ...), but **breaks for absolute encoders** operating in 0-360° range.

**Example Problem:**
```
Current: 350°, Target: 10°
Raw error: 10° - 350° = -340°
SimpleFOC interprets: Go -340° (long way around, backwards!)
Correct: +20° (short way, forward)
```

---

## Fix Attempt #1: Angle Normalization

**Solution:** Normalize target angle to be within ±180° of current position before passing to SimpleFOC.

**Implementation (motor_control.cpp:1428-1443):**
```cpp
// CRITICAL: SimpleFOC's position PID doesn't normalize angle errors!
// We must normalize the target to be within ±π of current position
float current_rad = motor.shaft_angle;
float target_rad = degreesToRadians(target_position_deg);

// Normalize error to [-π, +π] (shortest path)
float error_rad = target_rad - current_rad;
while (error_rad > PI) error_rad -= TWO_PI;
while (error_rad < -PI) error_rad += TWO_PI;

// Calculate normalized target that's closest to current position
float normalized_target_rad = current_rad + error_rad;
motor.move(normalized_target_rad);
```

**Expected Result:** Motor should always take shortest path, no more backwards movement

**Commit:** `496568c` - "Fix angle wraparound bug in SimpleFOC position control"

---

## Current State: Fix Didn't Fully Work

**Test Results After Normalization Fix:**

### Test 1: Motor Diagnostic Test (30° movement)
```
Start: 10.4° → Target: 40.4°
Expected: Move +30°
Actual: Moved to 13.4° (only +3°)

Diagnostics:
[DIAG] Moving | Enc:13.4° FOC:13.4° Tgt:40.4° | En:Y Cal:Y | TrkErr:0.00°
       PIDerr:27.0° VelCmd:540.3°/s VelAct:0.0°/s | Vq:6.00V Vd:0.00V
```

**Critical Observation:**
- ✅ PID error calculated correctly: 27.0° (40.4° - 13.4°)
- ✅ Velocity command generated: 540.3°/s (very high, position PID working)
- ✅ Voltage maxed out: 6.00V (velocity PID trying hard)
- ❌ **Actual velocity: 0.0°/s** (motor not moving!)

**This is the smoking gun** - The control algorithms are working correctly, but something prevents the motor from physically moving.

### Test 2: Manual Position Commands

```
Command: m 50 (from 13.36°)
Result: No movement, stuck at 13.36°

Command: m 100 (from 13.36°)
Result: No movement, stuck at 13.36°

Command: m 0 (from 13.36°)
Result: ✅ SUCCESS! Moved to 0.44° with rapid oscillation
```

**Pattern:** Motor sometimes moves (like `m 0`), sometimes doesn't (like `m 50`, `m 100`). When it does move, it oscillates rapidly before settling.

---

## Critical Diagnostic Analysis

### The Velocity Paradox

**From latest logs:**
```
PIDerr: 27.0°         ← Position PID sees 27° error
VelCmd: 540.3°/s      ← Position PID commands high velocity
VelAct: 0.0°/s        ← Motor reports ZERO velocity
Vq: 6.00V             ← Voltage controller outputs MAX voltage
```

**This means:**
1. ✅ Position PID is working (generates velocity command)
2. ✅ Velocity PID is working (generates voltage command)
3. ✅ FOC commutation is working (applies voltage to phases)
4. ❌ **Motor physically not moving despite voltage**

### Sensor Tracking is Perfect

```
[SENSOR] Raw: 608, Raw°: 13.36°, Filtered°: 13.36°
shaft_angle: 13.36° | sensor_direction: CW(+1)
Encoder: 13.36° | SimpleFOC: 13.36° | TrkErr: 0.00°
```

- ✅ Sensor reads correctly
- ✅ SimpleFOC tracking sensor perfectly
- ✅ No I2C communication issues
- ✅ Cartesian filtering working

### When m 0 DID Work

```
Command: m 0 (from 13.36°)
shaft_angle: 1.82° → 0.44°
Result: MOVED with rapid oscillation
Position reached notifications: 0.27°, 0.03°, -0.18°, 0.03°, -0.31°...
[AT_TARGET] Current: 0.44°, Target: 0.00°, Error: 0.44°, Vel: 0.00°/s
```

Motor successfully moved from 13° to 0°, but oscillated ±0.5° around target. **This proves motor CAN move when conditions are right.**

---

## Competing Theories

### Theory 1: Electrical Angle Incorrect ⚠️ LIKELY
**Hypothesis:** `zero_electric_angle` calculation during calibration is wrong or inverted

**Evidence:**
- Motor moves for some commands but not others
- When it moves, sometimes goes wrong direction initially
- Velocity PID commands voltage, but motor doesn't respond
- Suggests commutation applying torque in wrong direction or zero torque

**Test:** Check zero_electric_angle value during calibration, verify against SmartKnob's pattern

### Theory 2: Velocity PID Instability ⚠️ POSSIBLE
**Hypothesis:** Velocity PID gains too aggressive, causing motor to fight itself

**Evidence:**
- Rapid oscillation when motor does move (m 0 command)
- VelCmd: 540°/s is VERY high (motor can't accelerate that fast)
- Motor might be generating opposing torques rapidly

**Parameters:**
```cpp
PID_velocity.P = 0.2   // Standard for gimbal
PID_velocity.I = 20.0  // High integral gain
```

**Test:** Reduce velocity PID gains to 50% and retest

### Theory 3: Static Friction / Cogging ❌ UNLIKELY
**Hypothesis:** Motor can't overcome static friction with 6V

**Why Unlikely:**
- Motor DOES move for `m 0` command with same voltage
- Gimbal motors have low cogging torque
- 6V is standard operating voltage for this motor
- User confirms "VERY CONFIDENT" not mechanical/electronic issue

### Theory 4: Current Control Mode Active ⚠️ INVESTIGATE
**Hypothesis:** SimpleFOC might be in current control mode, but we have no current sensor

**Evidence:**
- Motor not responding to voltage commands
- No current sensor configured in code
- If SimpleFOC expects current feedback but gets none, control loop breaks

**Check:** Verify `motor.torque_controller` setting (should be `TorqueControlType::voltage`)

---

## Remaining Questions

### 1. Why does motor move for some commands but not others?
- `m 0` worked (13° → 0°)
- `m 50` failed (13° → no movement)
- `m 100` failed (13° → no movement)

**Hypothesis:** Electrical angle alignment only correct in certain mechanical angle ranges?

### 2. Why does VelCmd generate 540°/s but VelAct stays 0°/s?
- Position PID generates velocity command
- Velocity PID generates voltage
- But motor doesn't move

**Hypothesis:** Velocity calculation broken, or commutation not applying torque correctly

### 3. Why rapid oscillation when motor does move?
- `m 0` command showed oscillation: 0.27°, 0.03°, -0.18°, 0.03°, -0.31°...
- Suggests control loop unstable or fighting mechanical resonance

**Hypothesis:** PID gains too high, or LPF needed on velocity

---

## Files Modified This Session

### tests.cpp
- **Lines 21-62:** Condensed `logMotorState()` to 2-line format
- **Lines 402-536:** Condensed `runMotorTest()` to compact output
- **Status:** ✅ Working well, easier to read diagnostics

### motor_control.cpp
- **Lines 1428-1443:** Added angle normalization before `motor.move()`
- **Purpose:** Fix wraparound bug for absolute encoders
- **Status:** ⚠️ Helped with direction, but motor still not moving reliably

---

## Next Steps for Investigation

### High Priority
1. **Verify electrical angle calculation** - Check `zero_electric_angle` value during calibration
2. **Test voltage mode explicitly** - Ensure `motor.torque_controller = TorqueControlType::voltage`
3. **Check phase outputs during movement** - Use oscilloscope or logic analyzer to verify PWM signals
4. **Reduce velocity PID gains** - Try P=0.1, I=10.0 to reduce oscillation

### Medium Priority
5. **Test at different starting positions** - See if motor moves at some angles but not others
6. **Compare with SmartKnob calibration** - Verify our calibration matches their pattern exactly
7. **Add velocity filtering** - Implement LPF on shaft_velocity to reduce noise

### Low Priority
8. **Measure actual motor current** - Verify motor is receiving power
9. **Test with higher voltage limit** - Try 9V to rule out insufficient torque
10. **Profile SimpleFOC internals** - Add debug output for velocity PID calculations

---

## Key Learnings

### 1. SimpleFOC Usage is Correct
We verified our implementation matches SimpleFOC examples:
- Control loop structure correct
- PID parameters appropriate for gimbal motors
- Sensor integration following SimpleFOC patterns

### 2. Absolute Encoder Wraparound is Real
SimpleFOC's position PID doesn't normalize angles for 0-360° encoders. Our normalization fix helps with direction, but doesn't solve underlying movement issue.

### 3. Diagnostic Output Matters
Condensing serial output from 18 lines to 2 lines made patterns MUCH easier to spot. The compact format revealed the "VelCmd high but VelAct zero" pattern immediately.

### 4. Root Cause is Deeper Than Software Control
The control algorithms are working:
- Position error calculated correctly
- Velocity command generated appropriately
- Voltage applied to motor

But motor doesn't move. This suggests:
- Electrical angle miscalculation (commutation wrong direction)
- Or hardware/driver issue (despite user confidence)
- Or SimpleFOC configuration mismatch

---

## Code Quality Notes

### Good Patterns
- Angle normalization helper correctly handles wraparound
- Compact diagnostic output preserves all information
- SimpleFOC integration follows official examples

### Areas for Improvement
- Need better understanding of why motor moves sometimes but not others
- Velocity PID may need tuning (high gains causing oscillation)
- Should add safety checks for unreasonable velocity commands (540°/s too high)

---

## References

- [SimpleFOC Position Control Documentation](https://docs.simplefoc.com/angle_loop)
- [SimpleFOC Position Control Example](https://github.com/simplefoc/Arduino-FOC/blob/master/examples/motion_control/position_motion_control/magnetic_sensor/angle_control/angle_control.ino)
- [SmartKnob Motor Control Implementation](https://github.com/scottbez1/smartknob/blob/master/firmware/src/motor_task.cpp)
- Previous dev logs 1-6 for historical context

---

## Session Status: INCOMPLETE

**What Works:**
- ✅ Sensor tracking (perfect accuracy)
- ✅ Serial output (much clearer)
- ✅ Control algorithms (PID calculations correct)
- ✅ Angle normalization (prevents wraparound errors)

**What Doesn't Work:**
- ❌ Consistent motor movement (works sometimes, fails other times)
- ❌ Velocity response (motor reports 0°/s despite voltage commands)
- ❌ Smooth settling (rapid oscillation when motor does move)

**Mystery:**
Why does `m 0` work but `m 50` and `m 100` don't? All use same control loop, same PID, same voltage. The only difference is target angle value.

**Hypothesis for Next Session:**
Electrical angle calculation is correct only at certain mechanical angles (where calibration was performed), but incorrect at other angles. This would explain position-dependent movement.

---

**End of Dev Log 7**
