# Dev Log 10: Motor Control Working - PID Tuning Next

**Date:** 2024-01-20
**Status:** Motor Control Functional, Ready for PID Optimization
**Previous:** dev_log_9.md (Motor breakthrough, velocity fix)

## Summary

This session achieved **fully functional motor position control**. The motor now responds correctly to move commands, moves in the right direction, and holds position. Several critical bugs were fixed and the system is ready for PID tuning optimization.

## Major Fixes This Session

### 1. Velocity Calculation Fix
**Problem:** `motor.shaft_velocity` always returned 0, breaking PID damping.

**Root Cause:** Our `getVelocity()` override was broken - `previous_degrees` was set equal to `cached_degrees` in the same function call, so velocity = 0 always.

**Fix:** Removed the broken `getVelocity()` override. SimpleFOC's base class correctly calculates velocity using separate `angle_prev` and `vel_angle_prev` tracking.

### 2. Direction Verification in Calibration
**Problem:** Calibration sometimes produced `zero_electric_angle` that made motor go backward.

**Root Cause:** The 225° offset we added wasn't always correct - it varied based on motor position.

**Fix:** Added automatic direction verification after calibration:
1. Apply voltage using `setPhaseVoltage()` (reliable, doesn't cause wild spinning)
2. Check which direction motor actually moved
3. If backward, add 180° to `zero_electric_angle`
4. If weak, try alternative offset and compare

### 3. Rotation Tracking Corruption
**Problem:** `motor.shaft_angle` accumulated to crazy values (-2228°, -13076°) causing erratic behavior.

**Root Cause:** Direction test and diagnostic tests corrupted `full_rotations` counter.

**Fix:**
- Reset rotation tracking AFTER direction test
- Use `setPhaseVoltage()` instead of FOC loop for direction test (prevents wild spinning)
- `getPosition()` now normalizes to 0-360° using `fmod()`

### 4. Notification Spam
**Problem:** Position reached notifications firing repeatedly due to motor oscillation.

**Root Cause:** Flag reset when motor left target zone (during oscillation), not when new target set.

**Fix:** Track `last_target_position` and only reset notification flag when target actually changes.

### 5. Duplicate AT_TARGET Logs
**Problem:** Confusing paired logs showing both normalized and raw angles.

**Root Cause:** Two places printing `[AT_TARGET]` - one normalized, one raw.

**Fix:** Removed duplicate debug from `update()`, kept only the normalized output in `isAtTarget()`.

### 6. Binary Protocol Gibberish
**Problem:** `~���` characters cluttering serial monitor.

**Root Cause:** SerialTransfer binary packets meant for host application.

**Fix:** Added `ENABLE_BINARY_PROTOCOL` flag in config.h (default: false for debugging).

### 7. Target Normalization
**Problem:** `m 720` or `m -90` could cause unexpected behavior.

**Fix:** `moveToPosition()` now normalizes input to 0-360° range.

## Current Motor Behavior

After all fixes, motor control works well:

```
> c
[CALIBRATION] Starting...
  Movement: 23.0° Dir:CCW ZeroAngle:231.1° DirTest:-0.2° [WEAK]
  Initializing FOC... ✓
✓ Calibration complete

> e
✓ Motor enabled at -242.1°

> m 50
Moving to position: 50.00°
[AT_TARGET] Current: 49.57°, Target: 50.00°, Error: 0.43°, Vel: 0.55°/s
Position reached notification sent: seq=0, encoder pos=50.01°

> m 100
[AT_TARGET] Current: 99.58°, Target: 100.00°, Error: 0.42°, Vel: 0.55°/s
Position reached notification sent: seq=0, encoder pos=100.02°
```

**Working:**
- Motor moves to commanded positions
- Correct direction (no more backward movement)
- Position reported correctly (0-360° normalized)
- Velocity calculation working
- Single notification per move

**Remaining Issues:**
- ~0.1-0.5° steady-state error
- Slight "settling back" after reaching target (PID overshoot)
- Direction test sometimes shows [WEAK] (doesn't detect direction reliably)

## Current PID Configuration

From `config.h`:
```cpp
// Position PID
#define PID_P_POSITION           15.0    // Proportional gain
#define PID_I_POSITION           0.0     // Integral gain (currently ZERO!)
#define PID_D_POSITION           0.5     // Derivative gain
#define PID_RAMP_POSITION_DEG    400.0   // Output ramp (deg/s)

// Velocity PID
#define PID_P_VELOCITY           0.5
#define PID_I_VELOCITY           5.0
#define PID_D_VELOCITY           0.0
#define PID_RAMP_VELOCITY        1000.0
#define PID_LPF_VELOCITY         0.02    // Low-pass filter time constant

// Tolerances
#define POSITION_TOLERANCE_DEG   0.5     // Target reached threshold
#define VELOCITY_THRESHOLD_DEG   0.57    // Velocity threshold for "stopped"
```

## Next Steps: PID Tuning

### Goals
1. **Reduce steady-state error** from ~0.4° to <0.1°
2. **Eliminate overshoot/settling** - reach target and stay
3. **Maintain fast response** - don't sacrifice speed for precision

### Approach
1. **Add integral term** (`PID_I_POSITION`) to eliminate steady-state error
2. **Tune P gain** - may need adjustment with integral active
3. **Tune D gain** - add damping to reduce overshoot
4. **Adjust velocity LPF** - balance noise rejection vs response

### New PID Tuning Script Requirements
The old PID tuner is outdated. New script should:

1. **Step Response Test**
   - Command known step (e.g., +30°)
   - Measure: rise time, overshoot, settling time, steady-state error
   - Plot response curve

2. **Frequency Response Test**
   - Oscillate between two positions at increasing frequency
   - Find bandwidth and phase margin

3. **Auto-Tuning Algorithm**
   - Ziegler-Nichols or relay feedback method
   - Find ultimate gain and period
   - Calculate optimal P, I, D values

4. **Parameter Sweep**
   - Test range of P, I, D values
   - Score each combination on: accuracy, speed, stability
   - Find Pareto-optimal settings

### Test Sequence for Manual Tuning
```
c           # Calibrate
e           # Enable
m 0         # Go to 0°
m 30        # Step +30° - measure response
m 0         # Step -30° - measure response
m 90        # Larger step
m 0         # Return
```

Observe:
- How fast does it reach target?
- How much overshoot?
- Does it oscillate?
- What's the final error?

## Files Modified This Session

| File | Changes |
|------|---------|
| motor_control.cpp | Direction verification, rotation tracking reset, target normalization, removed duplicate debug |
| motor_control.h | Removed broken getVelocity() override |
| config.h | Added ENABLE_BINARY_PROTOCOL flag |
| communication.h | Conditional binary protocol output |
| ESP32_MCU_Firmware.ino | Fixed notification debouncing logic |
| tests.cpp | Diagnostic test improvements |

## Architecture Notes

### SimpleFOC Control Loop
```
                     Position PID          Velocity PID         FOC
target_deg ──►[P,I,D]──► velocity_cmd ──►[P,I,D]──► Vq ──►[setPhaseVoltage]──► Motor
                 ▲                            ▲                      │
                 │                            │                      │
            shaft_angle ◄─────────── getSensorAngle() ◄──────────────┘
                                    (from encoder)
```

### Key Variables
- `motor.shaft_angle` - Accumulated angle (can be negative, >360°)
- `motor.shaft_velocity` - Calculated by SimpleFOC base class
- `motor.zero_electric_angle` - Calibration offset
- `motor.sensor_direction` - CW or CCW
- `target_position_deg` - Our target (0-360° normalized)

### Encoder Details
- MT6701 via I2C
- 14-bit resolution: 16384 counts/revolution
- Angular resolution: 360°/16384 = 0.022°/count
- Cartesian filtering for smooth wraparound handling

## Conclusion

Motor control is now **functional and reliable**. The core issues (velocity calculation, direction verification, rotation tracking) are solved.

The remaining work is **optimization** - tuning PID parameters to achieve the precision and response time needed for the hyperspectral imaging application.

Next session: Create new PID tuning script and optimize motor performance.
