# Motor Control — Development History & Lessons Learned

**Purpose:** Compressed history of what was tried, what failed, and why. Read this before proposing a "new" approach to verify it hasn't already been tried.

---

## Timeline

### Phase 1: Hardware Bring-Up (Nov 2025)

**Motor damaged by mounting screws.** M2.5 screws ~1mm too long penetrated into windings → open circuit → replaced motor. All-zero resistance between all phase pairs was the signature. Lesson: always verify screw depth before motor installation.

**Encoder confirmed working.** MT6701 I2C at 400kHz, address 0x06. Position reads correctly 0–16383 counts. Field status register 0x00 = good field.

**All driver channels confirmed.** `phase_test` (6-position test at 0°, 60°, 120°, 180°, 240°, 300° electrical) isolated which channel was dead vs. which motor wire. Phases are 120° apart, NOT 90° — an early error in the test labels that was later corrected.

**SimpleFOC auto-calibration failed.** `motor.initFOC()` returned failure with "Failed to notice movement" or "Error: Not found!" With I2C encoder, the alignment timing (500 steps × 2ms = 1 second, sensor must respond within 2ms per step) was too sensitive. This was confirmed by GitHub issue #172 — same symptom with MT6701 I2C. The early "fix" was manual calibration.

### Phase 2: Sensor Integration Debugging (Jan 8–9, 2026)

**Root problem sequence (all superseded, captured here for reference):**

`shaft_angle` reset to 0° during `motor.move()` — this looked like a sensor issue but was a different problem altogether. Attempted fixes that didn't work:
- `motor.LPF_angle.Tf = 0.0f` — red herring; SimpleFOC already initializes this, doesn't cause reset-to-zero behavior
- Software home offset layer — added 3 coordinate systems (encoder absolute, SimpleFOC absolute, user relative), made things worse
- Overriding `update()` in sensor class — non-standard, broke integration
- Overriding `getAngle()` to suppress full_rotations — caused perfect sign inversion (shaft_angle = −encoder reading)

**Correct sensor pattern established:**
- Only override `getSensorAngle()`, nothing else
- SimpleFOC base class handles rotation tracking, velocity calculation, and wraparound
- This matches SmartKnob and all official SimpleFOC drivers

**Sign inversion from CCW direction (discovered but misidentified as a problem):**
`Direction::CCW = -1` in SimpleFOC's formula: `shaft_angle = sensor_direction × getAngle() − offset`. When CCW, this produces negative shaft_angle — this is expected behavior, not a bug. SimpleFOC handles negative angles correctly internally. The real problem was elsewhere.

### Phase 3: The Root Cause (Jan 14–15, 2026)

**The smoking gun — frozen shaft_angle:**
During position control, `motor.shaft_angle` stayed frozen at a fixed value for the entire 3-second trajectory. Motor drew 0A despite SimpleFOC commanding 6V.

Root cause chain:
1. FOC electrical angle = `shaft_angle × pole_pairs + zero_electric_angle`
2. If shaft_angle is frozen → electrical angle is frozen → static (non-rotating) field → zero torque
3. Frozen shaft_angle → why?

**Root cause: FORCE_SENSOR_DIRECTION_CW = true.** When we forced CW direction but the actual sensor direction was CCW, the calibrated zero_electric_angle was wrong. The motor's magnetic field pushed the rotor backward instead of forward, producing zero net torque. `motor.shaft_angle` couldn't update because the motor couldn't move.

Systematic 8-test diagnostic suite confirmed:
- Control mode: correct ✓
- Sensor calls: 100 calls per 100 loopFOC() iterations ✓
- shaft_angle update: updates when motor rotates ✓
- Position control: PASS after fix ✓

**Fix:** `FORCE_SENSOR_DIRECTION_CW = false`. Let SimpleFOC auto-detect CCW direction.

**Additional fix needed:** Calibration formula error. The zero_electric_angle calculation was wrong (unit mismatch — dividing electrical angle by pole_pairs instead of multiplying mechanical angle by pole_pairs). Correct formula: `zero_electric_angle = (mechanical_position × pole_pairs) − applied_electrical_angle`. At the time, a 180° (later 225°) empirical offset was also added to correct phase winding orientation — this is now handled by SimpleFOC's auto-calibration.

### Phase 4: Functional Motor (Jan 16–20, 2026)

**Motor moves reliably.** After direction fix: motor moves in correct direction, reaches commanded positions, velocity calculation works.

**Fixes required:**
- `getVelocity()` override removed — it was broken (previous_degrees = cached_degrees always → velocity = 0 always). SimpleFOC base class calculates velocity correctly.
- `full_rotations` counter corrupted by calibration movements → `resetRotationTracking()` called after calibration
- Position normalization added: `fmod(shaft_angle_rad × 180/π, 360)` to get 0–360° physical position
- Notification debouncing: print AT_TARGET only once per move, not on every oscillation

**Remaining issue:** ~0.4° steady-state error and oscillation when settling.

### Phase 5: Architecture Cleanup (Feb 4–5, 2026)

**Dual-read bug identified and fixed.** Custom `continuous_position_rad` tracking layer called `getSensorAngle()` a second time per loop iteration, creating two divergent tracking systems:
- SimpleFOC used Read #1 (from `loopFOC()`) for shaft_angle
- Our code used Read #2 for target calculation
- PID error computed between different readings → systematic bias → oscillation

Fix: removed entire custom tracking layer. Trust SimpleFOC's `full_rotations`. `update()` went from 85 lines to 25 lines.

**Open-loop test (T4) fixed (three separate bugs):**
1. `delay(10)` (100Hz) → `delay(1)` (1kHz): at 100Hz, field step = 4° electrical per step, too large for motor to track
2. `loopFOC()` during open-loop overwrites shaft_angle from sensor → field stops advancing → stall. Remove `loopFOC()` during T4.
3. `angle_openloop` mode: stops advancing field at target angle (~260ms). Use `velocity_openloop` which keeps rotating continuously.

**Velocity integral windup identified.** With I_vel=20, integral accumulates aggressively when motor can't achieve commanded velocity. Motor finally moves → wound-up integral → overshoot → reversal → repeats. Reducing I_vel reduced oscillation but didn't eliminate it.

**Architectural insight:** SimpleFOC's angle mode uses cascaded PID (position → velocity → voltage). Without current sensing, the velocity loop is open-loop w.r.t. torque — inherently fragile under varying load. Many gimbal projects bypass the velocity loop entirely (SmartKnob pattern).

### Phase 6: Setpoint Ramping (Feb 10, 2026)

**Root cause of T5 oscillation: step inputs, not PID gains.**
Step input creates massive error signal → cascaded PID overshoots → violent oscillation. Auto-tuner "fixed" this by crushing I_vel → no holding torque. This was treating the symptom, not the cause.

**Fix: setpoint ramping (slew rate limiter).** Ramp the PID target at 120°/s instead of jumping it instantly. The PID only ever sees 3–6° error → stays in stable/linear region → full gains preserved → full torque.

**Proportional deceleration prevents overshoot.** `ramp_speed = min(SLEW_RATE, |error| × 20)`. The gain 20 matches P_pos — motor arrives at target with near-zero velocity by design.

**T5 loop rate must match tuner rate.** Gains tuned at 1kHz become unstable at 100Hz (10× larger effective gains). Always run at consistent rate.

**Oscillation detection added to T5.** Count velocity sign reversals >±20°/s in 500ms window. >3 reversals = fail.

**Position-dependent oscillation at m90 persists.** Motor reaches 90° but shows sustained ±1–2° oscillation. Velocity LPF tested from 0.00 to 0.05 — none fixed it. The oscillation pattern changes with PID params, which means it's a control interaction, not pure hardware cogging.

### Phase 7: Multi-Point Calibration Rollback (Feb 18, 2026)

**Tried:** 24-point custom calibration (circular mean of ZEA samples). Produced persistent 12° tracking error → reduced torque → oscillation everywhere. Baseline dropped from 8/24 stable positions to 2/24.

**Why it failed:** The ZEA calculation using circular mean was correct in theory, but the 225° offset compensation was empirically tuned and didn't generalize across motor positions.

**Also tried and failed:**
- Reduced filter alpha (0.4 → 0.275): more phase lag → more instability. Never reduce filter alpha without compensating by reducing P_pos or increasing D_pos.
- Position deadband (0.25°): discontinuous control → limit cycles

**Rolled back to commit `9de17c9`.** SimpleFOC's built-in auto-calibration, while less sophisticated, produces correct alignment. D_pos=2.0 was theoretically promising but couldn't be cleanly evaluated due to calibration interference — still untested.

**Hardware update:** 1.5mm spacers installed between encoder IC and magnet. Magnet was physically rubbing encoder — potential friction/noise source. Not yet tested with clean firmware baseline.

---

## Key Insights (condensed)

1. **The FORCE_SENSOR_DIRECTION_CW flag was the root cause of ~3 weeks of debugging.** One config flag, wrong value → zero torque, frozen shaft_angle, 0A current despite commanded voltage.

2. **SimpleFOC's auto-calibration works when left alone.** Set direction=UNKNOWN and zero_electric_angle=-12345, call initFOC(). Don't pre-set values or SimpleFOC skips its own calibration.

3. **Calibration accuracy is paramount.** A 12° ZEA error kills performance more than any PID tuning can compensate for. Multi-point manual calibration failed; built-in calibration works.

4. **Can't tune away architectural problems.** Dual-read bug, cascaded velocity instability, deadband limit cycles — none of these respond to PID tuning. Fix the architecture first.

5. **Cascaded PID without current sensing is inherently fragile.** Without torque feedback, the velocity loop guesses voltage from velocity error. SmartKnob uses torque mode + manual PD on near-identical hardware and is more stable.

6. **Cogging saddle points cause position-dependent instability.** The 2804 motor has 12 cogging wells per revolution (every 30°). At 90° mechanical = 270° electrical, the rotor is at a saddle between two wells — maximum restoring torque gradient. This makes the control loop marginally stable at that position. At 0° and 180°, the motor happens to land near wells, which are naturally stable.

7. **Fix the input, not the PID.** Step inputs cause oscillation. Setpoint ramping is the standard industrial solution.

8. **Dual sensor reads = two coordinate systems = oscillation.** One source of truth, always.

9. **Velocity LPF changes affect stability non-monotonically.** Increasing Tf adds lag, which DECREASES phase margin and can INCREASE oscillation, not decrease it. Tested Tf=0.00–0.05 for m90 oscillation — none helped. Increasing further (0.05–0.10) is still untested.

10. **We abandoned cascade mode too early.** We tried cascade angle mode with I_vel=5.0, saw oscillation, and switched to custom torque mode. The community documents I_vel=5.0 as a known oscillation driver for gimbal motors. I_vel=0.1 is the correct starting point — we never tried it.

11. **motion_downsample is critical for noisy I2C encoders.** Without it, the position loop runs at 1kHz, but velocity estimates from differentiating I2C readings at 1kHz are extremely noisy. `motion_downsample=20` decouples the position loop (50Hz) from the torque loop (1kHz), massively reducing velocity noise in the outer loop.

12. **Sensor INL is the dominant remaining error source.** After all PID tuning, the residual ±0.3–1.0° position-dependent errors match the MT6701's ±1.0° typical INL spec exactly. These are repeatable, angle-dependent, and cannot be reduced by tuning — only by sensor calibration (`CalibratedSensor`).

---

### Phase 8: Test Framework Fixes (March 25–26, 2026)

By this point motor control was functional but had accumulated three bugs in the test infrastructure that prevented the full T1–T5 diagnostic from completing.

**Bug 1: T4 I2C lockup (hardware-level hang)**

`T4` used `velocity_openloop` mode to verify motor movement. This mode commands all three phase PWM channels simultaneously, creating electromagnetic interference on the shared I2C lines. The MT6701 encoder communicates via I2C, and `Wire.requestFrom()` has no timeout — it blocks indefinitely waiting for I2C bytes that never arrive due to PWM noise on SDA/SCL.

The symptom: the entire firmware hung during T4, serial output stopped, motor ran until power cycle. `T5` never ran.

Fix: replaced `velocity_openloop` with a 5-second closed-loop `+30°` move. Closed-loop FOC uses sinusoidal commutation (not simultaneous 3-phase PWM), which produces far less radiated EMI. T4 now completes cleanly and verifies driver + power delivery via the actual control path.

**Bug 2: `motor_test.py` DONE_MARKER fired on test header**

`motor_test.py` has per-command done markers — strings that signal the script to stop reading serial output. The marker for `"test"` was `"========================================"` (a line of equals signs). The problem: the test output starts with a box-drawing header that contains exactly this string. The script stopped reading after the very first line, before T1 even ran.

Fix: changed to `["RESULT: ALL TESTS PASSED", "RESULT: PARTIAL", "RESULT: FAILED"]` — strings that only appear in the final summary.

Also increased `"test"` and `"diag"` timeouts to 300s (T5 sweeps 24 positions, each with up to 8s timeout = up to 192s total).

**Bug 3: T5 first position timeout**

T4 leaves the motor at ~30° past its starting position. T5 immediately tries to move to 0° with a 1500ms timeout. For a 136° move at 60°/s slew rate, reaching 0° takes ~2.3 seconds minimum. The motor was timing out mid-move, oscillation measurements were taken while still moving, and all 24 subsequent positions were evaluated from wrong starting conditions.

Fix: added a pre-positioning block before the T5 loop (moves to 0° with 10s timeout), and increased per-position timeout from 1500ms to 8000ms.

**Result:** T1–T5 now all complete and produce meaningful results. RESULT output appears as expected.

---

### Phase 9: Cascade Mode Re-Investigation and Final Tuning (March 26, 2026)

With a working test framework, the 19/24 T5 failure rate in torque mode could be properly characterized.

**Directional overshoot confirmed as root cause:**

```
m 40 → m 45:  TIMEOUT — motor stuck at 47.5° after 15s
m 50 → m 45:  PASS   — settled at 45.09° in ~2s
```

T5 always approaches positions sequentially (0°→15°→30°→45°...), always from below. With `TORQUE_KD=0`, no velocity damping exists — the motor builds momentum during the move and overshoots past the target. The velocity integral resets on crossing, then slowly winds up, but the equilibrium at 47.5° has just enough cogging force to trap it there. Approaching from above avoids overshoot because the motor is decelerating into the target.

**Research finding:** This is a documented SimpleFOC pattern. The cascade angle mode's velocity loop provides natural active damping — overshoot velocity becomes velocity error, which the inner PID opposes. The reason we had abandoned cascade mode was `I_vel=5.0`, which the community explicitly warns against for gimbal motors with noisy velocity estimates.

**I_vel tuning progression in cascade mode:**

| Config | T5 Result | Notes |
|--------|-----------|-------|
| I_vel=0.0, DS=20 | 22/24 stable | Steady-state errors up to -4.95° at some positions |
| I_vel=0.5, DS=20 | 24/24 first run; 60° CHAOTIC second run | On stability boundary |
| I_vel=0.2, DS=10 | 22/24 stable | 150° and 330° consistently OSCILLATING (1.3°, 1.2°) |
| I_vel=0.2, DS=20 | 21/24 stable | 150° and 195° OSCILLATING; 330° settling |
| **I_vel=0.1, DS=20** | **24/24 STABLE** | **Final working config** |

The stability boundary lies between I_vel=0.1 and I_vel=0.2. At 0.1, the velocity integral provides enough authority to overcome friction and settle precisely, without winding up into limit cycles at positions where LPF velocity noise is highest.

**Final validated configuration (as of 2026-03-26):**

```
USE_TORQUE_MODE = false
PID_P_POSITION  = 20.0
PID_P_VELOCITY  = 0.3
PID_I_VELOCITY  = 0.1
LPF_velocity.Tf = 0.10
motion_downsample = 20
```

Result: 24/24 T5 positions stable, all errors within ±1°, worst case 0.97° at 45°. RESULT: ALL TESTS PASSED consistently across multiple runs.
