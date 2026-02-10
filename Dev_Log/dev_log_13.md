# Dev Log 13: Setpoint Ramping, T4/T5 Reliability & Oscillation Investigation

**Date:** 2026-02-10
**Branch:** `claude/debug-motor-t4-utNjI`
**Status:** T4 reliable, T5 mostly passing, position-dependent oscillation still under investigation

---

## Session Summary

This session began with T4 (Open-Loop Test) failing intermittently (~1 in 15 pass rate) and T5 (Position Control) oscillating wildly. We systematically fixed T4, then addressed T5 through setpoint ramping (motion profiling), PID tuner improvements, and oscillation detection. By the end, all 5 tests pass reliably and motor torque is preserved — but a position-dependent settling issue at certain angles (notably m90) remains unsolved.

---

## Issue 1: T4 Open-Loop Unreliability

**Symptom:** T4 passed only ~1 in 15 runs. Motor stalled at small angles (e.g., 8.6°).

**Root Causes (three separate bugs):**

1. **Loop rate too low:** T4 used `delay(10)` = 100Hz. Open-loop commutation needs electrical angle steps small enough for the motor to track. At 100Hz with 7 pole pairs: step = 1.0 × 0.01 × 7 = 0.07 rad = 4° electrical per step. At higher velocities this is too large. Fix: `delay(1)` = 1kHz.

2. **`loopFOC()` during open-loop:** `loopFOC()` overwrites `shaft_angle` with the sensor reading every iteration, preventing the open-loop controller from freely advancing the field angle. The field stops advancing and the motor stalls. Fix: Don't call `loopFOC()` during T4.

3. **`angle_openloop` vs `velocity_openloop`:** `angle_openloop` stops advancing the field once the internal angle reaches the target (~260ms for 30°), leaving a static field for the remaining 1.7s. `velocity_openloop` keeps the field rotating continuously at the requested rate. Fix: Use `velocity_openloop` mode.

**Result:** T4 now passes reliably with 50-60° of movement in the 2s window.

---

## Issue 2: T5 Oscillation From Step Inputs

**Symptom:** T5 commanded +30° step, motor oscillated violently, PID tuner reduced I_vel from 10→0.6, killing holding torque.

**Root Cause:** Step inputs to the PID. Jumping the target 30° instantly creates a massive error signal. The cascaded PID (position → velocity → voltage) overshoots, reverses, and oscillates. The PID tuner's solution — crushing I_vel — treats the symptom but destroys the motor's ability to hold position against external forces.

**The insight:** Fix the input, not the PID. This is a solved problem in industrial motor control.

**Fix Applied: Setpoint Ramping (Slew Rate Limiter)**

Standard industrial practice — ramp the PID target gradually instead of step inputs:

```cpp
// In motor_control.h:
float ramp_position_deg;                               // Current ramped setpoint
unsigned long last_ramp_time_us;                        // Timestamp for dt
static constexpr float SLEW_RATE_DEG_S = 120.0f;       // Max ramp rate (°/s)

// In motor_control.cpp update():
float ramp_error = target_position_deg - ramp_position_deg;
// Proportional deceleration: speed = min(cruise, distance × gain)
float ramp_speed = fminf(SLEW_RATE_DEG_S, fabsf(ramp_error) * 20.0f);
float max_step = ramp_speed * dt_s;
// ... advance ramp_position_deg toward target
```

Key design decisions:
- **120°/s slew rate** — fast enough for responsive moves, slow enough to avoid excitation
- **Proportional deceleration** — ramp speed scales with remaining distance (gain=20.0 matches P_pos). Creates a trapezoidal velocity profile. Motor arrives at target with near-zero velocity, preventing overshoot
- **PID uses ramped setpoint** — PID only ever sees 3-6° error, stays in its linear region. Full gains preserved = full torque

**Result:** PID tuner now reports "No adjustment needed", I_vel stays at 10.0, motor has full holding torque.

---

## Issue 3: T5 Loop Rate Mismatch

**Symptom:** T5 passed the PID tuner but still failed oscillation check.

**Root Cause:** T5 ran at 100Hz (`delay(10)`) while the PID tuner ran at 1kHz (`delay(1)`). At 100Hz, the effective PID gains are ~10x larger (larger dt between updates = larger integral accumulation, larger derivative spikes). Gains that are stable at 1kHz become unstable at 100Hz.

**Fix:** Changed T5 to `delay(1)` matching the tuner's loop rate.

---

## Issue 4: T5 Oscillation Detection

**Symptom:** T5 was passing even when the motor was clearly oscillating.

**Fix:** Added `countOscillations()` helper that counts velocity sign reversals exceeding a threshold (±20°/s) over a time window. T5 checks for oscillation in a 500ms window after the move completes. More than 3 reversals = fail.

```cpp
int countOscillations(MotorController& mc, BLDCMotor& motor,
                      unsigned long duration_ms, float threshold_deg_s);
```

---

## Issue 5: Position-Dependent Oscillation at m90 (UNRESOLVED)

**Symptom:** After all the above fixes, all 5 tests pass and m0/m180 are stable. But m90 shows sustained ±1-2° oscillation that never damps. The motor reaches the target but doesn't settle cleanly.

**Investigation so far:**

- **Electrical angle theory:** 90° mechanical × 7 pole pairs = 630° = 270° electrical. At this angle, two phases have equal-but-opposite contributions, creating a local cogging torque maximum. This is a known problematic angle for BLDC motors.

- **Velocity LPF (PID_LPF_VELOCITY):** Tried multiple values:
  - `0.00` — terrible: erratic movement, no torque, motor barely functional
  - `0.01` (original) — oscillation at m90
  - `0.015` — oscillation at m90, T5 also failed intermittently
  - `0.02` — oscillation at m90
  - `0.03` — oscillation at m90
  - `0.05` — clunky movement, motor felt sluggish, oscillation didn't improve

- **Conclusion:** The velocity LPF is not the root cause. The oscillation at m90 is caused by something else — possibly cogging torque interaction with the PID, commutation error at that electrical angle, or an issue with the encoder/filter at that position.

**T5 also fails intermittently** — at the `0.015` setting we captured a T5 fail with 9 reversals even though the motor reached the target position perfectly (err:-0.0°). The oscillation happens *after* settling, during the 500ms check window.

---

## Current Configuration

```cpp
// Position PID
PID_P_POSITION   = 20.0    // Proportional
PID_I_POSITION   = 0.0     // Integral (disabled)
PID_D_POSITION   = 1.0     // Derivative (damping)

// Velocity PID
PID_P_VELOCITY   = 0.2     // Standard for gimbal motors
PID_I_VELOCITY   = 10.0    // Full integral (ramping prevents windup)
PID_D_VELOCITY   = 0.0     // Usually 0 for gimbal motors
PID_LPF_VELOCITY = 0.03    // 30ms filter (was 0.01, tuning ongoing)

// Setpoint Ramp
SLEW_RATE_DEG_S  = 120.0   // Cruise speed (°/s)
// Decel gain = 20.0        // Matches P_pos for smooth arrival

// Limits
MAX_VELOCITY_DEG = 180.0   // ~30 RPM
VOLTAGE_LIMIT    = 6.0V    // Gimbal motor safe limit
```

---

## Files Modified This Session

| File | Changes |
|------|---------|
| `config.h` | PID_LPF_VELOCITY 0.01→0.03 |
| `motor_control.h` | Added ramp_position_deg, last_ramp_time_us, SLEW_RATE_DEG_S |
| `motor_control.cpp` | Added setpoint ramping with proportional deceleration in update(), init in constructor/enable() |
| `tests.cpp` | T4: velocity_openloop mode, no loopFOC, 1kHz loop. T5: delay(1), oscillation detection. PID tuner: two-phase adaptive. countOscillations() helper |

---

## Key Learnings

1. **Fix the input, not the PID.** Step inputs cause oscillation. Crushing gains to compensate destroys torque. Setpoint ramping is the standard industrial solution — ramp the target, keep full gains.

2. **Proportional deceleration prevents overshoot.** A constant-speed ramp that stops abruptly leaves the motor with velocity at the target → overshoot. Scaling ramp speed proportional to remaining distance (trapezoidal profile) means the motor arrives with near-zero velocity.

3. **Loop rate must match between tuner and test.** PID gains tuned at 1kHz are unstable at 100Hz. Always run at the same rate.

4. **Velocity LPF is not a silver bullet for position-dependent oscillation.** Tested 0.00 to 0.05 — none fixed the m90 oscillation. The root cause is likely cogging torque or commutation alignment at specific electrical angles, not velocity noise.

5. **Open-loop needs: high loop rate + no loopFOC + velocity_openloop.** Three independent bugs that all had to be fixed for T4 reliability.

---

## Open Bugs

### Bug 1: Position-Dependent Oscillation at m90
- Motor reaches 90° but shows sustained ±1-2° oscillation
- m0 and m180 are stable
- Not caused by velocity LPF (tested 0.00-0.05)
- Likely related to cogging torque at 270° electrical or commutation alignment
- **Next steps:** Capture encoder stream data at m90 and analyze oscillation frequency. Try D_vel > 0 for active damping. Consider testing at other problematic angles (m45, m135, m270). Could also try reducing voltage_limit slightly at steady-state.

### Bug 2: T5 Intermittent Failure
- T5 sometimes fails oscillation check (>3 reversals in 500ms) even when motor reaches target accurately
- Observed at LPF=0.015 with 9 reversals and 0.0° error — motor settled perfectly but oscillated briefly during the check window
- May be related to Bug 1 (position-dependent oscillation at the post-move angle)
- **Next steps:** Log the position T5 lands at when it fails — is it always near a problematic electrical angle? Consider whether the oscillation threshold (3 reversals / 20°/s) is too strict for settling transients vs true instability.

---

## Commits This Session

```
3fe2adb Increase velocity LPF to fix position-dependent oscillation at m90
8fd490e Add proportional deceleration to setpoint ramp
e30e9c6 Add setpoint ramping (slew rate limiter) at 120°/s to MotorController
06e9b60 Fix T5 loop rate mismatch: use delay(1) to match PID tuner's 1kHz
0a0abf2 Add two-phase PID auto-tuner and oscillation detection to T5
9dd40d2 Fix T4: use velocity_openloop mode, remove loopFOC during open-loop, run at 1kHz
```

---

## Next Session Checklist

- [ ] Investigate m90 oscillation with encoder stream data (capture and analyze frequency)
- [ ] Try D_vel > 0 for active velocity damping at steady-state
- [ ] Test oscillation at multiple angles (m45, m90, m135, m180, m270) to map problematic electrical angles
- [ ] Consider adjusting T5 oscillation threshold or checking if T5 failures correlate with specific angles
- [ ] Explore voltage reduction at steady-state (lower voltage = less cogging excitation)
- [ ] If cogging is confirmed: research anti-cogging compensation in SimpleFOC
