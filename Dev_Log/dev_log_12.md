# Dev Log 11: Architecture Cleanup & Oscillation Diagnosis

**Date:** 2026-02-05
**Branch:** `claude/debug-motor-open-loop-qtf1A`
**Status:** Partial fix applied, oscillation issue ongoing

---

## Session Summary

This session began with T4 (Open-Loop Test) failing and evolved into a deep investigation of the motor control architecture. We identified and fixed a fundamental dual-read bug, but oscillation issues persist under certain conditions.

---

## Issue 1: T4 Open-Loop Test Failure

**Symptom:** Motor moved -2.8° when +30° was expected.

**Root Cause:** `velocity_limit = 10 rad/s` (573°/s) was far too aggressive for a gimbal motor in open-loop mode. Gimbal motors have high phase resistance (~10Ω) and can only realistically achieve 2-5 rad/s in open-loop.

**Fix Applied:**
- Reduced `MAX_VELOCITY_DEG` from 573 to 180 in config.h
- Added `motor.velocity_limit = 2.0` specifically for T4 test in tests.cpp
- T4 now passes (moved 48-51°)

---

## Issue 2: T5 Oscillation After T4 Fix

**Symptom:** After T4 started passing, T5 (Position Control) showed constant `[SYNC]` tracking errors of 5-11° with motor hunting/oscillating rapidly.

**Root Cause Identified: Dual-Read Bug**

The `update()` function was calling `getSensorAngle()` **twice per iteration**:
1. First call: inside `motor.loopFOC()` via SimpleFOC's internal `sensor->update()`
2. Second call: explicitly by our custom `continuous_position_rad` tracking code

Each call did a fresh I2C read AND advanced the Cartesian filter state. This created **two divergent position tracking systems**:

```
READ #1 (loopFOC)  →  SimpleFOC uses this for shaft_angle
READ #2 (our code) →  We used this for target calculation

Target computed from READ #2, but PID error computed against READ #1
= Systematically biased error signal = Oscillation
```

**Fix Applied:** Stripped the entire custom tracking layer:
- Removed `continuous_position_rad`, `prev_encoder_rad` member variables
- Removed second `getSensorAngle()` call
- Removed SYNC mechanism
- Now trust SimpleFOC's built-in `full_rotations` counter for continuous angle tracking
- `resetRotationTracking()` after calibration ensures `full_rotations` starts clean

**Result:** `update()` went from 85 lines to 25 lines. Single sensor read, single tracking system.

---

## Issue 3: Persistent Oscillation (Ongoing)

**Symptom:** Even after the dual-read fix, oscillation persists:
1. Small, rapid oscillation after moves - eventually settles after several seconds
2. Larger oscillation when load/weight distribution changes - never settles

**Diagnosis: Velocity Integral Windup**

With `PID_I_VELOCITY = 20.0`, the velocity integral accumulates aggressively:
- Under load, motor can't achieve commanded velocity
- Integral accumulates (20 V/s per rad/s of velocity error)
- When motor finally moves, wound-up integral causes overshoot
- Error reverses, integral unwinds, causes reverse overshoot
- Cycle repeats indefinitely

**Key Insight:** The D-term (damping) we added didn't help. This confirms the oscillation is from **bad inputs** (integral windup), not PID tuning of the proportional/derivative terms.

**Attempted Fixes:**
- Reduced `PID_I_VELOCITY` from 20 to 5 to 2 to 0
- Increased `PID_P_VELOCITY` from 0.2 to 0.5 to compensate
- Oscillation reduced but not eliminated

---

## Systemic Issue Identified

**The deeper problem:** SimpleFOC's angle mode uses cascaded PID (Position → Velocity → Voltage). But **without current sensing**, the velocity loop is essentially open-loop with respect to torque. It's guessing what voltage to apply to achieve a velocity, which is inherently unstable under varying load.

Many successful gimbal projects skip the velocity loop entirely in voltage mode:
```cpp
voltage = Kp * (target - position) - Kd * velocity  // Direct PD control
```

This avoids the cascaded instability and integral windup problems.

---

## Files Modified This Session

| File | Changes |
|------|---------|
| `config.h` | Reduced velocity limits, added D-term to position PID |
| `tests.cpp` | Added velocity_limit save/restore for T4 |
| `motor_control.cpp` | **Major:** Removed dual-read, stripped custom tracking layer |
| `motor_control.h` | Removed `continuous_position_rad`, `prev_encoder_rad`, `MAX_TRACKING_ERROR_DEG` |
| `Dev_Log/t4_t5_signal_flow.md` | Created Mermaid diagrams of signal flow |
| `Dev_Log/todo_motor_architecture_cleanup.md` | Created naming/reorganization TODO |

---

## Plan for Next Session

### Priority 1: Data Streaming Framework

Build encoder position streaming to enable visualization and future scanning workflow.

**ESP32 Side:**
```
Command: `stream on` / `stream off`
Output: CSV lines at 100Hz
Format: timestamp_ms,position_deg,velocity_deg_s
Markers: SCAN_START,timestamp and SCAN_END,timestamp
```

**Desktop Side:**
- Python script with pyserial to read serial
- Log to CSV file with timestamps
- Real-time matplotlib plot for debugging
- Video correlation: match encoder timestamps to video frames

**Why This Helps:**
1. Visualize oscillation pattern → diagnose frequency/amplitude → identify root cause
2. Build foundation for scanning workflow (correlate encoder position with video frames)
3. Enable data-driven PID tuning

### Priority 2: Diagnose Oscillation with Data

Once we can visualize, determine:
- **Low frequency (1-5 Hz):** PID tuning issue
- **High frequency (>20 Hz):** Electrical/commutation issue
- **Grows over time:** Integral windup
- **Constant amplitude:** Limit cycle (gain too high)

### Priority 3: Consider Architecture Change

If PID tuning continues to be fragile, consider:

**Option A: Torque Mode with External Position Control**
- Run SimpleFOC in torque/voltage mode (very stable)
- Implement simple PD position controller ourselves
- No velocity loop, no integral windup

**Option B: Motion Profiling**
- Instead of step commands, generate smooth S-curve trajectories
- Reduces stress on PID controller
- Prevents overshoot by design

**Option C: Auto-Tuning Script**
- Characterize system response (step response, frequency response)
- Calculate optimal PID values mathematically
- More robust than manual tuning

---

## Current PID Configuration

```cpp
// Position PID
PID_P_POSITION   = 20.0   // Proportional
PID_I_POSITION   = 0.0    // Integral (disabled)
PID_D_POSITION   = 1.0    // Derivative (damping)

// Velocity PID
PID_P_VELOCITY   = 0.5    // Proportional (increased from 0.2)
PID_I_VELOCITY   = 2.0    // Integral (reduced from 20.0)
PID_D_VELOCITY   = 0.0    // Derivative

// Limits
MAX_VELOCITY_DEG = 180.0  // ~30 RPM (reduced from 573)
VOLTAGE_LIMIT    = 6.0V   // Gimbal motor safe limit
```

---

## Key Learnings

1. **Dual sensor reads = divergent tracking = oscillation.** Always use a single source of truth for position.

2. **SimpleFOC's `full_rotations` is reliable** after calling `resetRotationTracking()` post-calibration. The custom tracking layer was unnecessary.

3. **Velocity integral is dangerous** on voltage-mode gimbal motors. The motor can't reliably track velocity without current control, so the integral winds up.

4. **Can't tune away architectural problems.** The dual-read bug couldn't be fixed with PID tuning. Neither can the cascaded velocity loop instability under varying load.

5. **Visualization is essential.** Flying blind with oscillation makes it impossible to diagnose. Need data streaming to see what's actually happening.

---

## Commits This Session

```
a80826b Strip custom tracking layer, trust SimpleFOC full_rotations for position
f599511 Add architecture cleanup TODO: naming confusion and dual-read bug plan
3bc3a55 Add D-term damping (D=1.0) to position PID to reduce oscillation
d1a647f Add T4→T5 signal flow diagrams documenting dual-read oscillation bug
aeb50a1 Fix open-loop test failure by reducing velocity limits for gimbal motor
```

---

## Next Session Checklist

- [ ] Implement encoder streaming on ESP32 (`stream on/off` command)
- [ ] Create Python script for serial logging and real-time plotting
- [ ] Capture oscillation data and visualize
- [ ] Based on visualization, decide: tune PID vs change architecture
- [ ] If changing architecture: implement torque mode + external PD control
- [ ] Test scanning workflow: record encoder positions during video capture
