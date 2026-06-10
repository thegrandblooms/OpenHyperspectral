# Motor Control — Current State & Next Steps

**Last updated:** 2026-06-10
**Status:** ✅ DECISION MADE (2026-06-10): **Option B — stepper motor.** New firmware scaffolded at `firmware/ESP32_Stepper_Firmware/` (NEMA17 + TMC2209 STEP/DIR+UART + belt reduction + MT6835 21-bit encoder on the output shaft; compiles clean, flash with `tools\flash_stepper.bat`). Rationale: the precision requirement is ≤0.1° (target 0.02°) at the output shaft; the gimbal path has two independent hard ceilings — the MT6701's ±1° INL caps absolute accuracy regardless of control, and the 84-detent cogging torque dead zone produced the stall-snap-bounce limit cycle across all five control architectures. Option A (dynamic cogging LUT) targets sweep smoothness only and even at its ODrive-grade best (~90% reduction) lands ~3–15× short of spec. See `firmware/ESP32_Stepper_Firmware/README.md` for the new architecture (move-then-trim, unidirectional approach, passive holding). The gimbal firmware below is retained for reference.

---

**Pre-decision status (2026-04-01):** 🔴 SWEEP BLOCKED — stall-snap-bounce oscillation at cogging detents persists across all control approaches tried. Position holding (T5) remains 24/24 stable. Root cause is well-characterized (see section below).

---

## Hardware

```
ESP32-S3 (Waveshare Touch LCD)
├── GPIO 47/48 → I2C SDA/SCL → MT6701 encoder (addr 0x06, 400kHz)
├── GPIO 11/12/13 → PWM → SimpleFOC Mini (DRV8313 driver)
└── GPIO 15 → Enable pin → DRV8313 (GPIO 14 = fault, GPIO 9 = reset)

MT6701 Encoder: 14-bit absolute, 16384 counts/rev, 0.022°/count
  - 1.5mm spacers installed between encoder IC and magnet
    (magnet was physically rubbing encoder IC — potential friction/noise source)
  - Power-on initialization time ~10ms; init code adds 100ms delay ✓

Mitoot 2804 Gimbal Motor: 7 pole pairs, 6.3Ω measured phase resistance, 100KV
  - 12 stator slots × 14 poles → cogging period = 360/LCM(14,12) = 360/84 = 4.3° mechanical
  - Electrical period = 360/7 = 51.4° mechanical (25.7° is half-electrical, not cogging)
  - DO NOT use mounting screws longer than hole depth (damaged one motor)

SimpleFOC Mini (DRV8313): passive driver, just amplifies PWM.
```

**All hardware confirmed functional.** ESP32, all 3 driver channels, encoder I2C.

---

## Key Files

| File | Purpose |
|------|---------|
| `firmware/ESP32_MCU_Firmware/motor_control.cpp` | Motor control logic (torque mode block ~line 1200) |
| `firmware/ESP32_MCU_Firmware/motor_control.h` | Class declaration, SLEW_RATE, MOVE_TIMEOUT |
| `firmware/ESP32_MCU_Firmware/tests.cpp` | T1–T5 diagnostic suite |
| `firmware/ESP32_MCU_Firmware/config.h` | All tunable parameters |
| `tools/motor_test.py` | Serial automation, presets, streaming capture |

---

## Current Configuration (cascade angle mode, validated 2026-03-26)

```cpp
// Architecture toggle
USE_TORQUE_MODE = false  // SimpleFOC cascade: P_angle → PID_velocity → voltage → FOC

// Cascade mode gains (config.h)
PID_P_POSITION   = 20.0   // angle outer loop P gain — ceiling at LPF=0.05: P=25 → 1 oscillating, P=30 → 2 chaotic
PID_I_POSITION   = 0.0    // no position integral needed
PID_D_POSITION   = 0.2    // light damping

PID_P_VELOCITY   = 0.3    // velocity inner loop P — 0.5 caused widespread oscillation
PID_I_VELOCITY   = 0.1    // CRITICAL: 5.0 oscillates, 0.5 borderline, 0.2 → 2 positions oscillate, 0.1 → 24/24 stable
PID_D_VELOCITY   = 0.0    // always 0 for gimbal motors
PID_LPF_VELOCITY = 0.05   // 50ms filter (3.2Hz bandwidth) — 0.02 marginal even at P_vel=0.2; 0.05 is stable 24/24
PID_RAMP_POSITION = 100.0 // output ramp for position loop
PID_RAMP_VELOCITY = 1000.0 // output ramp for velocity loop

// Outer loop rate (motor_control.cpp)
motor.motion_downsample = 10  // position loop runs at 100Hz; inner torque loop at ~1kHz (halved from 20)

// Setpoint ramp (motor_control.h)
SLEW_RATE_DEG_S  = 60.0f    // cruise speed (deg/s)
MOVE_TIMEOUT_MS  = 15000    // firmware move timeout (ms)
SETTLING_TIME_MS = 200      // ms of being in-tolerance before "at target"
SETTLING_ERROR_DEG = 1.0f
SETTLING_VEL_DEG_S = 5.0f

// Position tolerances (config.h)
POSITION_TOLERANCE_DEG = 0.5    // isAtTarget() position threshold
VELOCITY_THRESHOLD_DEG = 5.0    // isAtTarget() velocity threshold

// Calibration
USE_SIMPLEFOC_AUTO_CALIBRATION = true
LOOP_FREQUENCY_HZ = 1000        // I2C-limited ~1kHz (no delay in control loop)
```

---

## What Works

- **T5 24/24 positions stable** — worst case 0.64° at 60°; most positions ±0.3–0.5°
- **T1–T5 diagnostic tests**: all pass (T4 rewritten to closed-loop +30° move)
- **5/5 position sweep** (±30° from start): all positions ±0.3°, settling in 1–3s
- **NVS persistent calibration**: 5-pass averaged zero_electric_angle, stored to flash. Subsequent boots load in ~1s vs ~10s. Eliminates run-to-run ±4.5° zero variability (17°–22° range → fixed 17.0° ± 0.3°). Commands: `recalibrate` (clear NVS + fresh 5-pass) / `calibrate` (load NVS if valid, else calibrate + save).
- Setpoint ramping (60°/s) with proportional deceleration
- Encoder streaming: `$ENC,<ms>,<pos>,<vel>,<target>,<voltage_q>,<elec_angle_deg>` (6 fields) auto-starts on enable
- Anti-cogging feedforward infrastructure: sinusoidal injection onto `voltage.q` after `motor.move()`. Runtime tunable via `set cog_amp <V>` and `set cog_phase <rad>`. Disabled by default (AMP=0).
- `cogmap` command: 24-position sweep (0°–345°, 15° steps), collects steady-state voltage.q and electrical_angle at each position

---

## How We Got Here: The Cascade Mode Fix

**Problem in torque mode (2026-03-26):** Motor overshot targets when approaching from below (e.g. 40°→45° TIMEOUT at 47.5°; 50°→45° PASS). With `TORQUE_KD=0`, no velocity damping. Motor built momentum, carried past target, couldn't return against cogging + integral lag.

**Root cause:** Torque mode is missing the active damping that SimpleFOC's cascade velocity loop provides. The velocity loop sees overshoot velocity as error and opposes it. We had previously tried cascade mode but used `I_vel=5.0` — a known oscillation driver for gimbal motors (confirmed by community threads). We never tried smaller I_vel values.

**Fix path:**
1. Switched `USE_TORQUE_MODE=false` (cascade angle mode)
2. Added `motion_downsample=20` (outer position loop at 50Hz, reduces noise coupling)
3. Tuned `I_vel`: 0.0 → large steady-state errors, 0.5 → borderline oscillation, 0.2 → 2 positions oscillate, **0.1 → 24/24 stable**

**Research basis:** Standard SimpleFOC practice for gimbal motors is cascade angle mode with very small I_vel (0–0.2) and motion_downsample. The community specifically warns against large I_vel with noisy encoders — the velocity estimate from differentiating I2C readings at 1kHz is high-noise, causing integral windup in the velocity loop.

---

## Next Steps: Improving Accuracy to ±0.25°

Current worst-case error is ~±1° (at 45°); most positions are ±0.3–0.5°. Target is ±0.25°. Based on research (see section below), three levers are available in order of expected impact:

### Step 1 — Lower motion_downsample + LPF_velocity.Tf, raise P_angle [COMPLETE — 2026-03-26]

**Result: worst error improved from 0.97° → 0.82°. All 24 positions stable.**

Tuning progression (DS=10 throughout):
| Config | Result |
|--------|--------|
| DS=20, LPF=0.10, P_angle=20 | 24/24 stable, worst 0.97° at 45° (baseline) |
| DS=10, LPF=0.05, P_angle=20 | 24/24 stable, worst 0.82° at 165° ✅ **best stable** |
| DS=10, LPF=0.02, P_angle=20, P_vel=0.3 | 23/24, 1 oscillating (30°) |
| DS=10, LPF=0.02, P_angle=20, P_vel=0.2 | 23/24, 1 oscillating (45°) — LPF=0.02 is marginal |
| DS=10, LPF=0.05, P_angle=30, P_vel=0.3 | 21/24 stable, 2 chaotic, 1 oscillating |
| DS=10, LPF=0.05, P_angle=25, P_vel=0.3 | 23/24 stable, 1 oscillating (210°) |

**Key findings:**
- `motion_downsample=10` + `LPF=0.05` was the largest gain — most positions improved 0.2–0.4°
- `LPF=0.02` is marginal with this motor/encoder combination; stays oscillating even at `P_vel=0.2`
- `P_angle` ceiling at `LPF=0.05` is ~20-22; P=25 marginal, P=30 chaotic
- Raising `P_angle` beyond 20 requires getting `LPF=0.02` stable (which needs CalibratedSensor to reduce velocity noise)
- The remaining ±0.5–0.8° errors are sensor INL, not a PID gap — Step 2 is the fix

### Step 2 — CalibratedSensor [FAILED — 2026-03-26]

**Result: worse than baseline at all tested voltages. Confirmed dead end.**

Implemented `CalibratedSensor` from Arduino-FOC-drivers v1.0.9 (local copy in sketch dir). Tested at two calibration voltages:
- 1.5V: 22-23/24 stable, worst ~2.7° — already worse than baseline
- 3.0V: 17/24 stable, worst 13° — much worse

**Root cause (discovered): The calibration scan computes a different `zero_electric_angle` (0.36 rad) than the standard initFOC alignment (0.30 rad).** The delta is 0.06 rad = 3.4° electrical = 3.4/7 = **0.49° mechanical systematic offset** added to every position. The LUT corrections (±0.3–0.4°) are the same magnitude as this introduced offset — so CalibratedSensor both introduces a ~0.5° bias AND adds LUT noise on top.

Higher calibration voltage makes this worse: at 3.0V the motor is pushed harder through cogging detents during the scan, causing cogging-snap position errors in the LUT measurements themselves. The LUT is trying to correct sensor INL but measures cogging errors during the scan — the two are indistinguishable at this scan resolution (n_lut=200 / 7 PP = ~29 samples per electrical cycle).

**CalibratedSensor is fundamentally incompatible with this motor:** the open-loop scan cannot cleanly separate sensor INL from cogging detents because the cogging period (4.3°) is similar in magnitude to the sensor INL (±1°). The corrected zero_electric_angle and LUT corrections together introduce ~0.5° systematic error.

### Step 3 — Sinusoidal anti-cogging feedforward [IMPLEMENTED, TUNING PENDING]

Infrastructure implemented. Cogmap data collected. Single-harmonic fit analysis completed.

**Cogmap data (2026-03-26, steady-state voltage.q at each T5 position):**
```
Notable: 135° → +0.280V (large backward force), 270° → -0.280V (large forward force)
Oscillating positions: 210° → +0.059V, 345° → +0.042V, 165° → -0.085V (all SMALL forces)
```

**Single-harmonic fit result (n=4, 84 cogging cycles/rev):**
- Best fit: n=4, amp=0.071V, phase=0.262rad (15°)
- RMS error: 0.105V — poor fit (4/24 positions have wrong-sign predictions)
- Root cause: 15° mechanical steps = 105° electrical steps → Nyquist limit ~1.7 cycles/electrical revolution. Cannot extract n=12 (dominant cogging harmonic) from 15° data.

**Key insight: oscillating positions have SMALL cogging forces.** The limit cycles at 165°/210°/345° are approach-dynamics driven (stochastic), not large cogging forces at those positions. Feedforward is unlikely to fix the oscillation problem. It may improve accuracy at 135°/270° where forces are large.

**Result: FAILED — feedforward confirmed dead end.**

Tested n=4, amp=0.1V, phase=0.26rad. Result: **1/24 stable, 18 chaotic, 28.8° worst**. The feedforward violently destabilized every position. Recovery (amp=0): 23/24 stable — system returned to baseline immediately. Root cause: at standstill, `motor.electrical_angle` is fixed (motor not rotating), so the "cogging cancellation" sinusoid injects a near-DC voltage perturbation that fights the PID rather than rotating with the shaft. The PID-feedforward interaction creates resonances amplified by the velocity integral. The approach is fundamentally incompatible with cascade angle mode at standstill.

**Feedforward is a dead end for this motor/control configuration.** Do not revisit.

### Integration tasks (parallel)
- **Verify under scan conditions** — confirm positions hold during camera integration time
- **Tune SLEW_RATE** — 60°/s conservative; may increase once scan workflow is validated

---

## Accuracy Research Findings (2026-03-26)

Research sources: docs.simplefoc.com, community.simplefoc.com, SimpleFOC GitHub, Ben Katz blog.

### Why current errors are ±0.3–1.0° (not a PID problem)

The MT6701 datasheet specifies **±1.0° typical INL** (Integral Non-Linearity) — a repeatable, position-dependent error from magnet eccentricity and Hall-effect interpolation. It is not noise; it is the same systematic offset every time the shaft is at that angle. The T5 error pattern (45° always worst at -0.97°; 90° always +0.35°; 300° always near-zero) is the signature of sensor INL, not a tuning deficiency. PID gains cannot correct for errors in the position measurement itself.

### CalibratedSensor (SimpleFOC drivers library)

No built-in anti-cogging in the main `Arduino-FOC` library (memory constraints on AVR were cited; ESP32-S3 is not affected). The `Arduino-FOC-drivers` library has `CalibratedSensor` implementing the [Ben Katz autocalibration method](https://build-its-inprogress.blogspot.com/2017/03/encoder-autocalibration-for-brushless.html):

- Forward + reverse open-loop scan, averaged to cancel friction
- FIR filter over one electrical cycle to separate eccentricity from cogging
- Runtime correction LUT applied inside `getAngle()`
- Documented community result: 10× reduction in velocity ripple standard deviation

Library: `Arduino-FOC-drivers` v1.0.9+ (July 2025). Key bug fixed in this version: `n_ticks` was `5*NPP` (causing LUT stairstepping); correct formula is `n_ticks = n_lut / NPP`.

### P_angle and LPF_velocity are coupled

`LPF_velocity.Tf=0.10` gives a velocity feedback bandwidth of `1/(2π × 0.10) ≈ 1.6 Hz`. Any position disturbance above ~0.5 Hz gets phase-lagged velocity feedback, making the inner loop nearly blind to it. This limits how high P_angle can go before overshoot: community reports P_angle≈7 as ceiling with Tf=0.10. The fix is lower Tf first, then raise P_angle.

Recommended end state: `Tf=0.02` (8 Hz bandwidth), `P_angle=30–40`.

### motion_downsample and accuracy

`motion_downsample=20` means the position/velocity PID fires every 20ms. Between corrections, the motor can drift freely. Lowering to DS=5–10 (100–200Hz position loop) gives substantially more authority to correct cogging-induced position hunting. After changing DS, `PID_velocity.I` typically needs proportional adjustment.

### I2C vs accuracy

No documented accuracy difference between I2C and SPI for static position accuracy — both deliver the same 14-bit ADC value. I2C is sufficient at 1kHz. The accuracy limitation is the sensor's INL, not the interface.

### Anti-cogging feedforward (no current sensing needed)

A sinusoidal feedforward at the cogging frequency (4 × pole_pairs cogging detents per revolution = 28 for the 2804) can cancel the fundamental cogging harmonic. Inject `cog_amplitude × sin(4 × electrical_angle + cog_phase)` onto `voltage.q`. Tune amplitude (0.05–0.1V) and phase empirically at the worst position. Addresses cogging after sensor INL is corrected by CalibratedSensor.

---

## Critical Decisions — Must Not Be Undone

### 1. Never force CW sensor direction
`FORCE_SENSOR_DIRECTION_CW = false`. Forcing CW when actual direction is CCW caused the "frozen shaft_angle / 0A current" failure. `Direction::CCW = -1` in SimpleFOC; wrong direction → static electrical field → zero torque.

### 2. One getSensorAngle() call per loop
`getSensorAngle()` is called exactly once, inside `motor.loopFOC()`. A second call in custom tracking code creates two divergent position tracking systems → systematic oscillation.

### 3. Only override getSensorAngle() in the sensor class
Do NOT override `update()` or `getAngle()`. Both were tried and both failed. Standard SimpleFOC pattern: only implement `getSensorAngle()`.

### 4. Setpoint ramping is essential for step inputs
Without ramping, step inputs cause violent oscillation. The ramp lives in `update()` with proportional deceleration.

### 5. Remove delay(10) from test loops
Running at natural ~1kHz (not forced 100Hz) is required for stable control. The delay(10) in tests.cpp's sweep loop was the root cause of all early oscillation issues.

### 6. T4 open-loop test replaced with closed-loop move
Original T4 used velocity_openloop which caused I2C lockup (3-phase PWM electromagnetic interference on I2C lines, Wire.requestFrom() has no timeout). T4 now does a closed-loop +30° move.

---

## Dead Ends (Do Not Revisit)

| What | Why It Failed |
|------|--------------|
| Forcing CW direction | Root cause of frozen shaft_angle / 0A current |
| Overriding `getAngle()` to ignore `full_rotations` | Caused perfect sign inversion (shaft_angle = −encoder) |
| Overriding `update()` in sensor class | Non-standard; broke sensor integration |
| Custom `continuous_position_rad` tracking layer | Double-read caused divergent tracking → oscillation |
| Position deadband (0.25°) | Limit cycles at formerly stable positions |
| Kd > 0 at 1ms dt with I2C encoder | Position-delta velocity is too noisy; any Kd that damps 150–400°/s snaps exceeds stability boundary |
| delay(10) in test loop | Forces 100Hz; cogging snaps look instantaneous; Kd amplifies them → ±400°/s oscillation |
| I_vel = 5.0 in cascade angle mode | Confirmed oscillation driver — was the only value we tried, abandoned cascade mode too early |
| I_vel = 0.5 in cascade angle mode | On stability boundary — first run passes, subsequent runs oscillate |
| I_vel = 0.2 in cascade mode | 2 positions consistently oscillate (150°, 195°); marginal |
| I_vel = 0.0 in cascade mode | Stable but large steady-state errors up to -4.95° — insufficient for ±0.5° requirement |
| I_vel = 0.15 in cascade mode | 21/24, 3 oscillating (15°, 150°, 240°) — over stability boundary |
| Anti-cogging feedforward n=4, amp=0.1V | 1/24 stable, 18 chaotic — catastrophic resonance. At standstill, electrical_angle is fixed so injection is DC vs. PID. Incompatible with cascade angle mode. |
| Sinusoidal feedforward during sweep (amp=0.13V) | No improvement in oscillation or backward step rate during 1°/s sweep. Root cause: 24-point cogmap → 15° spacing → Nyquist too low to capture 84-detent/rev pattern. Single harmonic explains only ~50% of variance. Need 360-point LUT for effective cancellation. |
| 1°/s sweep in cascade angle mode | Stick-slip: motor sits at cogging detents for ~4s then snaps 4.3°, ±2.5° amplitude, ~12Hz oscillation. Worst 2s window = 0.000 deg/s (stall). Fundamentally unusable for pushbroom scan. Root cause: PID limit cycle, not tuning. |
| Torque mode (KP=4, KI=20, KD=0) | 19/24 T5 stable; directional overshoot at 5 positions — KD=0 has no velocity damping |
| T4 velocity_openloop | I2C lockup via PWM EMI; replaced with closed-loop +30° |
| LPF_velocity=0.02 with P_vel=0.3 | 1 oscillating position (30°); on stability boundary |
| LPF_velocity=0.02 with P_vel=0.2 | Still 1 oscillating (moves to 45°); LPF=0.02 marginal regardless of P_vel |
| P_angle=30 with LPF=0.05 | 2 chaotic, 1 oscillating — P_angle ceiling at LPF=0.05 is ~20 |
| P_angle=25 with LPF=0.05 | 23/24 — 210° oscillating; still marginal |
| SLEW_RATE_DEG_S = 30°/s | Too slow to push through cogging, motor stalls at detents |
| CalibratedSensor @1.5V | 22-23/24 stable, worst ~2.7° — worse than baseline. Calibration scan computes different zero_electric_angle (0.36 vs 0.30 rad), introducing ~0.49° mechanical systematic offset |
| CalibratedSensor @3.0V | 17/24 stable, worst 13° — much worse. Higher voltage forces motor through cogging detents during scan, making LUT errors larger. Root cause: can't separate sensor INL from cogging at this scan resolution |
| I_CAP = 1.0V | Overshoots → ±8° sustained limit cycle |
| I_CAP = 0.6V | Too weak to escape cogging at 0.76° error |

---

## Sweep Validation Results (2026-03-27)

**Goal:** validate motor behavior under continuous sweep conditions before building Python scan module.

**Tool:** `tools/sweep_validation.py` — comprehensive sweep validator using `esp_ms` timestamps (not OS time, which has Windows scheduling jitter making instantaneous speed unreliable).

### 1°/s sweep — FAIL (stick-slip oscillation)

```
Actual speed:     1.031 deg/s (3.4% error — average looks correct)
Backward steps:   13.44% of samples show negative velocity
Max backward:     2.2° in a single sample
Worst 2s window:  0.000 deg/s — STALL detected
```

**What actually happens:** Motor sits at a cogging detent for ~4 seconds, then snaps 4.3° forward (one full cogging pitch), overshoots, oscillates at ~12Hz with ±2.5° amplitude. The average speed looks correct over the full sweep, but instantaneous motion is completely non-monotonic. At 30fps, the camera would capture ~117 frames at the same detent position, then 0 frames in the 4.3° gap.

**Why the average looks fine but motion is not:** The motor eventually traverses each cogging detent, so total travel / total time ≈ 1°/s. But the *distribution* of positions is completely wrong — the camera sees positions 4.3° apart, not 0.08°/pixel.

### 5°/s sweep — marginal

```
Actual speed:     4.849 deg/s (3.0% error)
Backward steps:   27.87% of samples show negative velocity
Max backward:     2.03° in a single sample
Worst 2s window:  1.71 deg/s (34% of commanded — no sustained stall)
```

Motor traverses continuously, no stalls. But 27% backward steps means oscillation amplitude (~2°) is large relative to 0.08°/pixel. Short camera shutter (<0.5ms) would freeze the instantaneous position. Usable as a workaround but not the intended scan protocol.

### Root cause: PID limit cycle

```
Cogging barrier height:    ~0.45V (from cogmap — typical Vq to hold most positions)
PID output at 1°/s error:  P_angle=20 × 1° = 20 → vel_setpoint = 20 deg/s
                           P_vel=0.3 × 20 = 6V → instantly saturated, irrelevant
                           At small error (<<4°): P_angle=20 × 0.1° = 2 → Vq ≈ 0.006V << 0.45V

What happens:
1. Motor sits at cogging minimum (stable equilibrium)
2. Target setpoint advances at 1°/s → position error accumulates
3. At ~4° accumulated error: P_angle × P_vel finally outputs ~0.45V
4. Motor breaks free, snaps to next detent (or past it)
5. Overshoot triggers velocity PID in reverse → oscillation at ~12Hz
6. Cycle repeats at next detent
```

The velocity PID inner loop is the structural problem: at 1°/s, the outer position loop outputs a velocity setpoint that the inner loop can only satisfy by building up a large position error first. This is a limit cycle, not a tuning deficiency.

---

## Cogmap Data (2026-03-27, Full 24-Point Sweep)

Command: `calibrate` → `cogmap`. Measures steady-state voltage.q the PID must generate to hold each position. Higher |Vq| = stronger cogging force at that position.

```
Target   Actual   Err(°)    Vq(V)   Elec(°)
  0.0°    0.00°   0.000    0.0000   343.0   (held at known zero)
 15.0°   14.77°  -0.234    0.0647    86.4
 30.0°   30.32°   0.322    0.0584   195.3
 45.0°   42.80°  -2.197    0.0314   282.6   ← large position error (cogging minima shifted)
 60.0°   61.00°   0.996   -0.1320    50.0
 75.0°   75.32°   0.322   -0.0592   150.3
 90.0°   90.44°   0.439   -0.1579   256.1
105.0°  104.50°  -0.498    0.1576   354.5
120.0°  120.41°   0.410    0.0516   105.9
135.0°  134.56°  -0.439    0.2714   204.9
150.0°  150.64°   0.645   -0.0499   317.5
165.0°  164.97°  -0.029   -0.2028    57.8
180.0°  179.56°  -0.439    0.0267   159.9
195.0°  195.03°   0.029    0.0021   268.2
210.0°  209.95°  -0.054    0.1210    10.5
225.0°  224.38°  -0.615   -0.0625   113.7
240.0°  239.65°  -0.345    0.0335   221.8
255.0°  254.97°  -0.029   -0.0319   327.8
270.0°  270.44°   0.439   -0.2006    76.1   ← second largest force
285.0°  284.24°  -0.762    0.1462   172.7
300.0°  300.41°   0.410    0.0207   285.9
315.0°  314.21°  -0.791    0.3359    22.5   ← LARGEST force
330.0°  330.82°   0.820   -0.1099   138.7
345.0°  346.40°   1.397   -0.4526   250.8   ← second largest negative
```

**Stats:**
- Max Vq: +0.3359V at 315°
- Min Vq: -0.4526V at 345°
- Peak-to-peak: 0.7885V
- Mean: -0.0012V (near-zero — no systematic bias)
- Recommended ff_amp (half peak-to-peak): 0.394V

**Pattern:** Multi-harmonic, NOT well-described by a single sinusoid. The 15° sample spacing (N=24 across 360°) gives Nyquist at 12 cycles/rev, but the fundamental cogging is at 84 cycles/rev. Single-harmonic fitting at N=4 (best we can extract) explains only ~50% of variance. This is why the earlier sinusoidal feedforward test (0.13V, N=4) showed no improvement: the correction signal was unrelated to the actual cogging pattern.

**Workaround feedforward test during sweep (2026-03-27):** Tested cog_amp=0.13V during active sweep. No improvement in oscillation amplitude or backward step rate. Confirmed that sparse cogmap + single harmonic is insufficient for sweep anti-cogging.

---

## Research Findings: Control Approaches for Gimbal Motor Sweep (2026-03-27)

### 2804 Motor + SimpleFOC Community Experience

The Mitoot 2804 100KV is a standard 12N14P brushless gimbal motor. Stick-slip at slow speeds is **universally reported** for this class of motor in SimpleFOC community threads. The cascade angle mode limit cycle is the standard failure mode. Key community findings:

- Cogging at slow speeds is physics, not a bug. The 84-detent/rev pattern (4.3° spacing) is inherent to 12N14P geometry.
- Cascade angle mode works well for *position holding* but generates limit cycles at low sweep speeds because the velocity PID inner loop can only respond after a cogging-sized position error accumulates.
- Anti-cogging via sinusoidal feedforward works only with dense sampling (≥360 points), not 24-point cogmap. Community consensus is LUT-based feedforward.
- SmartKnob (popular SimpleFOC-based project) explicitly lists anti-cogging as "planned but not implemented."

### SmartKnob Architecture (angle_nocascade + spring torque)

SmartKnob uses `MotionControlType::torque` with application-level spring force computation — NOT cascaded angle/velocity PID. This is effectively `angle_nocascade` behavior: position error → direct torque, no velocity PID inner loop.

**Why this avoids the limit cycle:** Without a velocity PID, there is no integral windup on velocity error. The system is a direct P controller on position → torque. It can output a small torque proportional to a small position error, rather than requiring the error to accumulate until the velocity setpoint is large enough to overcome the inner loop's dead zone.

SmartKnob still has cogging (same motors, same physics) — but because their use case is haptic feedback at known detents, they don't need smooth slow sweeps. For our scan use case, `angle_nocascade` would still need LUT feedforward for smooth motion.

### SimpleFOC Control Mode Options

| Mode | How it works | Status in this project |
|------|-------------|----------------------|
| `angle` (cascade) | P_angle → vel_setpoint → PID_vel → Vq → FOC | **Current** — stable T5 but limit cycle at slow sweep |
| `angle_nocascade` | P_angle → Vq directly (no velocity PID) | **Not tried** — eliminates limit cycle source; highest priority next step |
| `velocity` | PID_vel → Vq | Not useful for position holding |
| `torque` (voltage) | Direct Vq command | Tried 2026-03-26 — 19/24 T5 stable (no velocity damping → directional overshoot) |
| `torque` + spring | Application computes Vq = K × (target - actual) | Equivalent to angle_nocascade; SmartKnob approach |
| Dense LUT feedforward | 360-point Vq table injected as FF | Not tried — community consensus for anti-cogging cancellation |

### angle_nocascade Mode

SimpleFOC has `MotionControlType::angle_openloop` which ignores the velocity PID. A simpler path is to implement `angle_nocascade` in our existing cascade firmware by zeroing out the velocity PID (P_vel=0, I_vel=0) and routing P_angle output directly to Vq. This is a one-line firmware change (set `motor.PID_velocity.P = 0; motor.PID_velocity.I = 0; motor.voltage_limit = <enough>`) or by setting the velocity setpoint as the Vq directly.

**Expected behavior:** At 1°/s, a 0.1° position error generates `P_angle × 0.1 = 2.0` → if we scale so 1.0 → 0.5V, that's a 0.1V output — enough to move slowly against cogging at most positions. No velocity integral → no windup → no limit cycle oscillation.

**Risk:** Without velocity damping, overshoot at the end of slewed moves. May need to tune P_angle down or add light Kd.

### Dense LUT Feedforward (Community Consensus for Anti-Cogging)

Store 360 Vq values (one per degree) in ESP32 RAM (360 × 4 bytes = 1.4 KB — easily fits). Populate by running cogmap at 1° resolution (360 positions, ~24 minutes) or by measuring Vq at each 1° step. Inject as feedforward: `Vq_out = PID_output + LUT[round(pos_deg)]`.

This directly cancels the mechanical cogging force at each position. Combined with `angle_nocascade`, should produce smooth slow sweeps. LUT only needs to be measured once per motor (cogging is repeatable).

---

## Next Steps — Motor Sweep (Priority Order)

### 1. Try `angle_nocascade` mode [HIGH PRIORITY — 1–2 hr]

Implement by either:
- (a) Zero velocity PID: set `P_vel=0, I_vel=0` in config.h; position error routes through P_angle only → direct Vq output. Test T5 first (confirm still holds positions), then test sweep at 1°/s.
- (b) Add `ANGLE_NOCASCADE` config flag: when true, skip inner PID, set `motor.voltage.q = P_angle × (target - actual)` directly.

**Success criterion:** Sweep at 1°/s with worst 2s window ≥ 0.5 deg/s (>50% of commanded, no stalls).

If T5 position holding degrades significantly (>1.5° worst), may need light Kd term or higher P_angle.

### 2. Dense LUT feedforward [MEDIUM — 3–4 hr including measurement time]

If angle_nocascade alone isn't smooth enough at 1°/s:
- Run cogmap at 1° steps (360 positions × ~4s each ≈ 24 min) → array of 360 Vq values
- Add `COG_LUT[360]` to firmware, populate via serial command or hardcode from measurement
- Inject `COG_LUT[pos_deg % 360]` as feedforward in the voltage.q path

### 3. Accept 5°/s + short shutter [LOW EFFORT — workaround]

At 5°/s the motor sweeps continuously (worst 2s = 1.71 deg/s, no stalls). Camera shutter <0.5ms would freeze any instantaneous position to within `0.5ms × 5°/s = 0.0025°` — well below 0.04° half-pixel target. This bypasses the cogging problem entirely at the cost of slower frame extraction SNR (shorter integration time).

**Limitation:** 5°/s over 60° = 12 second sweep at 30fps = 360 frames. At 0.08°/pixel and 60° range, need 750 frames with positions spaced ≤ 0.04°. At 5°/s / 30fps = 0.167°/frame — oversampling is 0.167/0.08 = 2× (adequate for frame selection, but close). At 1°/s this is 12× oversampling.

### 4. voltage_d injection [RESEARCH — unproven for this application]

Some community posts suggest injecting a small `voltage_d` component (orthogonal to torque) to "lubricate" the cogging detents via reluctance torque. Requires current sensing to be safe; we have no current sensors. Skip unless 1–3 all fail.

---

## Test Results History

### 2026-03-26 — Baseline after CalibratedSensor reverted (USE_CALIBRATED_SENSOR=false)

```
Config: USE_TORQUE_MODE=false, P_vel=0.3, I_vel=0.1, LPF_vel=0.05, motion_downsample=10, P_angle=20

T5 (360° sweep, 15° steps, 24 positions):
  23 stable + 1 settling (0°) = 24/24 OK — 0 oscillating, 0 chaotic
  Worst error: 0.64° at 60°

  0° → settling, err=-0.29°  |  180° → stable, err=-0.44°
 15° → stable, err=+0.12°   |  195° → stable, err=-0.23°
 30° → stable, err=-0.47°   |  210° → stable, err=-0.47°
 45° → stable, err=-0.53°   |  225° → stable, err=-0.62°
 60° → stable, err=+0.64°   |  240° → stable, err=+0.47°
 75° → stable, err=+0.15°   |  255° → stable, err=+0.50°
 90° → stable, err=+0.44°   |  270° → stable, err=+0.44°
105° → stable, err=-0.41°   |  285° → stable, err=-0.50°
120° → stable, err=-0.21°   |  300° → stable, err=+0.41°
135° → stable, err=-0.44°   |  315° → stable, err=-0.44°
150° → stable, err=+0.47°   |  330° → stable, err=+0.47°
                              |  345° → stable, err=+0.32°
```

### 2026-03-26 — Step 1 tuning complete (DS=10, LPF=0.05, P_angle=20)

```
Config: USE_TORQUE_MODE=false, P_vel=0.3, I_vel=0.1, LPF_vel=0.05, motion_downsample=10, P_angle=20

T5 (360° sweep, 15° steps, 24 positions):
  24/24 STABLE — 0 settling, 0 oscillating, 0 chaotic
  Worst error: 0.82° at 165° (improved from 0.97° at 45°)
  Most positions within ±0.5°

  0° → stable, err=+0.44°   |  180° → stable, err=-0.44°
 15° → stable, err=+0.03°   |  195° → stable, err=-0.15°
 30° → stable, err=-0.21°   |  210° → stable, err=+0.32°
 45° → stable, err=-0.53°   |  225° → stable, err=-0.44°
 60° → stable, err=-0.06°   |  240° → stable, err=+0.47°
 75° → stable, err=+0.50°   |  255° → stable, err=+0.06°
 90° → stable, err=+0.44°   |  270° → stable, err=+0.44°
105° → stable, err=-0.23°   |  285° → stable, err=-0.50°
120° → stable, err=+0.50°   |  300° → stable, err=-0.47°
135° → stable, err=-0.44°   |  315° → stable, err=-0.44°
150° → stable, err=+0.47°   |  330° → stable, err=+0.47°
165° → stable, err=-0.82°   |  345° → stable, err=-0.56°
```

### 2026-03-26 — Original working configuration (cascade angle mode)

```
Config: USE_TORQUE_MODE=false, P_vel=0.3, I_vel=0.1, LPF_vel=0.10, motion_downsample=20

calibrate:     OK — Dir:CW, zero_electric_angle 0.38 rad (~21.7°)
T1 Hardware:   PASS (Enc:54.7°, field:good)
T2 Calibration: PASS
T3 Sensor:     PASS (100/100 sensor calls)
T4 Movement:   PASS — moved +31.2° (target 30°, settled in ~1.5s)
T5 (360° sweep, 15° steps, 24 positions):
  24/24 STABLE — 0 settling, 0 oscillating, 0 chaotic
  Worst error: 0.97° at 45°
  All positions within ±1°

sweep (±30°, 5 positions): 5/5 PASS — all within ±0.3° (from earlier torque-mode baseline)
```

**T5 position-by-position:**
```
  0° → stable, err=-0.44°   |  180° → stable, err=-0.44°
 15° → stable, err=-0.50°   |  195° → stable, err=-0.41°
 30° → stable, err=-0.47°   |  210° → stable, err=-0.47°
 45° → stable, err=-0.97°   |  225° → stable, err=-0.88°
 60° → stable, err=+0.47°   |  240° → stable, err=+0.47°
 75° → stable, err=+0.50°   |  255° → stable, err=+0.41°
 90° → stable, err=+0.35°   |  270° → stable, err=+0.44°
105° → stable, err=-0.41°   |  285° → stable, err=-0.50°
120° → stable, err=+0.50°   |  300° → stable, err=-0.03°
135° → stable, err=-0.44°   |  315° → stable, err=-0.44°
150° → stable, err=+0.47°   |  330° → stable, err=+0.47°
165° → stable, err=-0.56°   |  345° → stable, err=-0.56°
```

### 2026-03-26 — Cascade mode tuning progression

```
I_vel=0.0:  22/24 stable but large steady-state errors (up to -4.95° at 345°)
I_vel=0.5:  First run 24/24, second run: 60° CHAOTIC (noise=316), 150° OSCILLATING — on stability boundary
I_vel=0.2 + DS=10:  22/24 stable, 150° OSCILLATING (1.3°), 330° OSCILLATING (1.2°)
I_vel=0.2 + DS=20:  21/24 stable, 150° OSCILLATING (1.3°), 195° OSCILLATING (1.0°), 330° settling
I_vel=0.1 + DS=20:  24/24 STABLE — final working config
```

### 2026-03-26 — Torque mode results (prior to cascade switch)

```
Config: USE_TORQUE_MODE=true, KP=4.0, KI=20.0, KD=0.0, I_CAP=0.8V

T5 (360° sweep, 15° steps, 24 positions):
  19/24 stable, 5 failing: 45°, 165°, 225°, 315°, 345°
  Root cause: directional overshoot — KD=0 → no velocity damping → overshoots
  from below, can't return against cogging. Approaches from above always settle.

sweep (±30°, 5 positions): 5/5 PASS — all within ±0.3°
```

### 2026-03-25 (before torque mode)

```
sweep: 1/5 PARTIAL (positions ±15° and ±30° all TIMEOUT)
Motor oscillates 102–327°/s at most positions, never settles within ±2° in 5s
Root cause: delay(10) forcing 100Hz + I_vel=5.0 in cascade mode
```

---

## Automated Development Workflow

```
1. Edit firmware (config.h or motor_control.cpp)
2. Close any serial monitors (Arduino IDE, SpectrumBoi)
3. cd A:\Code_and_Data\Spectrometry\OpenHyperspectral
4. tools\flash.bat                                     ← compile + upload (~15s)
5. py -3.9 tools\motor_test.py --preset baseline       ← run tests
6. Read tools\last_run.txt for results
7. Repeat
```

### Test presets
```
--preset baseline      calibrate + full test (T1–T5 diagnostic) [updated 2026-03-26]
--preset oscillation   calibrate + enable + move to 0/90/180/270 + status
--preset sweep         calibrate + enable + 5-position accuracy test (±30°)
--preset quick         calibrate + enable + +30° sanity check
--preset torque_check  calibrate + enable + moves to 90/270/45 + status
--stream N             capture N seconds of $ENC, streaming data
```

### Filtering streaming noise
```
grep -v "^\$ENC," tools\last_run.txt    ← clean view without streaming lines
grep -E "T[1-5]|RESULT:|Summary:" tools\last_run.txt   ← just T5 table + verdict
```

### Known issues with automation
- **COM3 must be free** — close Arduino IDE serial monitor and SpectrumBoi before flash or test
- **Calibration silent for ~15s** — SimpleFOC initFOC() alignment takes 10–20s. Normal, not a hang.
- **Debug heartbeat** — send `debug 0` first to suppress periodic status lines that pollute output.

---

## Architecture Diagram (current: cascade angle mode)

```
  ┌──── Cascade Angle Control (~1kHz FOC, 100Hz position loop) ─────────────────┐
  │                                                                               │
  │  target_deg                                                                   │
  │  ──[ramp 60°/s]──►[P_angle=20]──► vel_setpoint ──►[PID_vel P=0.3, I=0.1]──►Vq──►[+cog_ff]──►[FOC]──► Motor
  │                      (100Hz via                        ↑                         │
  │                    downsample=10)               LPF_velocity                     │
  │                                                  Tf=0.05                         │
  │                                              (50ms filter, 3.2Hz BW)            │
  │                                                                                  │
  │  MT6701 encoder (14-bit I2C, 0.022°/count) ◄─────────────────────────────────┘
  └───────────────────────────────────────────────────────────────────────────────┘

Velocity loop provides natural active damping — overshoot is seen as velocity error
and opposed. This is the structural fix over torque mode (which had KD=0, no damping).
```

---

## Serial Commands (Quick Reference)

```
c / calibrate   Load NVS calibration (fast ~1s), or run 5-pass thorough cal + save if no NVS
recalibrate     Clear NVS + run fresh 5-pass thorough calibration + save
e / enable      Enable motor (requires calibration)
d / disable     Disable motor
m <deg>         Move to absolute position 0–360°
stop            Emergency stop
s / status      Encoder + FOC state snapshot
i / info        System info (chip, pins, config)
test / diag     Full T1–T5 diagnostic suite
sweep           5-position accuracy test (±30° from current)
motor_test      Quick +30° move sanity check (note: has delay(10) in inner loop)
phase_test      Driver phase verification
encoder_test    Interactive encoder reading
align           Motor holding strength test
tune on/off     Repeating ±30/60/90° PID tune cycle
stream on/off   Enable/disable $ENC, CSV streaming (6 fields: ms,pos,vel,target,vq,elec_deg)
stream rate N   Set stream rate in Hz (1–500)
debug <0/1>     Toggle periodic status output
scan            I2C bus scan
cogmap          24-position sweep (15° steps), outputs $COG lines — quick cogging survey
cogmap_dense    180-position sweep (2° steps, ~5 min), measures + loads dense LUT feedforward
cogreset        Disable dense LUT feedforward (LUT is lost on power-cycle)
sweep <deg/s>   Start continuous sweep at commanded speed (moving-target position control)
sweep_stop      Stop sweep, hold current position
sync            Clock sync handshake, returns $SYNC,<esp_ms>
stream_on [N]   Enable $ENC, streaming at N Hz (default 100)
stream_off      Disable streaming
set cog_amp V   Set anti-cogging feedforward amplitude (0–1.5V; 0=disabled)
set cog_phase R Set feedforward phase offset (radians)
set pos_tol D   Set position tolerance (degrees)
set vel_thr V   Set velocity threshold (deg/s)
```

---

## Sweep Problem — Full Characterization (2026-04-01)

### The Goal (Clarified)

The motor drives a pushbroom hyperspectral scanner. Video frames are matched to encoder positions in post-processing — backward motion doesn't corrupt data (backward frames are filtered out). The goal is simply a **well-behaved motor controller**: smooth, consistent motion with minimal oscillation. Oscillation is bad because it:
- Causes mechanical vibration that may affect optics
- Reduces effective sweep speed (time wasted bouncing at detents)
- Indicates poor control quality generally

### The Physics: Why This Motor Class Is Hard at Slow Speeds

The Mitoot 2804 is a **12-slot, 14-pole iron-core BLDC**. Iron-core stators have strong magnetic detents (cogging) where rotor pole edges align with stator teeth. Motor has **84 cogging detents/revolution** (LCM(12,14) = 84), spacing **4.286°** between detents.

At 1°/s, the motor spends **4.3 seconds** traversing each cogging detent. The cogging force at each detent must be continuously overcome. Peak cogging force (from cogmap data) is ~0.33–0.45V equivalent voltage.

Camera gimbals use the same motor class but operate at high speeds (stabilization) where cogging is negligible. At sub-1 RPM, cogging is the dominant load.

### The Stall-Snap-Bounce Cycle

Every control approach tried produces this pattern at strong cogging detents:

```
1. STALL: Motor sits at a cogging minimum. Position error grows as ramp advances.
           Control torque builds (via PID integral or velocity PI integral).

2. SNAP:  Cogging force is overcome. Motor breaks free and accelerates rapidly
           (snap velocity 200–800°/s measured). The stored energy drives a violent
           impulse through the detent.

3. BOUNCE: Motor overshoots the next stable position. Momentum carries it into the
            next cogging well. Depending on control mode:
            - Position tracking: lag goes negative → natural braking (vq < 0)
            - Velocity PI: integral drains (vel >> target) → vq → 0 → no braking
            Motor bounces backward 0.3–2.5°. At 100Hz stream, this shows as a
            backward step in the encoder log.

4. REPEAT: Motor settles at next detent. Cycle repeats.
```

The **worst_2s metric** in sweep_validation measures net encoder travel over any 2-second window. When the stall-snap-bounce oscillates the motor forward 4.3° then back 4.3°, net travel ≈ 0 → worst_2s = 0 → FAIL (even though the motor is moving — it's oscillating, not stalling).

### Quantified Results (sweep_validation.py, 2026-04-01)

All five approaches tested on same hardware (existing cogging LUT from NVS):

| Approach | Speed accuracy | worst_2s (1°/s) | Backward steps | Assessment |
|---|---|---|---|---|
| Cascade angle mode (SimpleFOC) | ✓ PASS | ✗ 0.000 (oscillation) | ✗ 13–28% | Limit cycle, not usable |
| Custom TORQUE_KP=4 + LAG=15° | ✓ PASS | ✗ 0.000 (oscillation) | ✗ 2.01% | Best so far; oscillation remains |
| Velocity PI (shaft_velocity) | ✓ PASS | ✗ 0.000 (oscillation) | ✗ 7–14% | Bug: shaft_velocity frozen; worse than position tracking |
| Velocity PI (50ms encoder delta) | ✗ FAIL 385% | ✗ 0.000 (oscillation) | ✗ 8–12% | Fixed bug; still stall-snap; vq≥0 prevents braking |
| All approaches at 5°/s | ✓ PASS | ✓ 2.0–13.3 (no stall) | ✗ 8–14% | Stall fixed at 5°/s; backward bounce persists |

**Key insight:** The oscillation occurs at the same positions (30°, 90°, 300°) across all approaches. These are the positions with the strongest cogging detents. The control architecture cannot fix the underlying physics — only cogging compensation can.

**Key insight 2:** Position tracking (TORQUE_KP) is strictly better than velocity PI because when the motor overshoots the ramp, lag goes negative → vq goes negative → automatic braking. The velocity PI (vq ≥ 0 always) has no braking mechanism, making backward steps much worse.

### What the Research Says (2026-04-01)

Thorough research of SimpleFOC community, ODrive docs, academic literature, and VESC forums:

**On cogging at low speed:**
- Stick-slip at sub-1 RPM is universally reported for 12N14P iron-core motors. Not a tuning problem.
- SimpleFOC has **no built-in anti-cogging feature**.
- Community consensus: position-dependent voltage feedforward LUT is the correct approach. Sinusoidal feedforward (1 harmonic) is insufficient — the 84-detent pattern requires ≥180 samples (Nyquist). A 360-point LUT gives 4.3 samples per cogging cycle, barely adequate.
- ODrive's anti-cogging achieves ~88-90% torque ripple reduction. After compensation, the remaining ~10% disturbance is small enough to overcome without violent snap-through.

**On velocity control at sub-1 RPM:**
- Moving-target position control (ramp setpoint) is the correct architecture — not velocity setpoint control. This is what our firmware's sweep mode implements. Literature agrees.
- `motor.shaft_velocity` in SimpleFOC is only updated by `motor.move()`. Bypassing `motor.move()` in sweep mode means shaft_velocity is frozen — this was the bug causing the velocity PI to fail.

**Critical calibration insight:**
- Static hold cogmap (our `cogmap_dense`) **overestimates** cogging force because it includes static friction (higher than dynamic friction during motion).
- Better approach: calibrate the LUT during a slow sweep at moderate speed (5°/s, above stall threshold), measure position error as a function of angle, FFT to extract cogging harmonics. This gives the true dynamic cogging torque.

**On hardware:**
- Iron-core BLDC motors are fundamentally poor for sub-1 RPM precision. The cogging is physics, not engineering.
- **Slotless BLDC motors** (e.g., T-Motor, Gartt) have no iron core → zero cogging. Used in professional gimbals requiring slow precise motion.
- **Stepper motors + TMC2209** are the standard solution for slow precise rotary stages (3D printers, telescope mounts, spectrometer gratings). No cogging in the BLDC sense. At 1°/s with 16x microstepping (NEMA17), stepping rate is 3.3 steps/sec — no resonance risk.

---

## Hardware Alternatives Assessment (2026-04-01)

The current gimbal motor approach has a fundamental mismatch: gimbal motors are optimized for fast stabilization, not slow precision sweeps.

### Option A: Better Cogging LUT (Current Hardware, ~$0 extra cost)

**What's needed:**
1. Calibrate LUT dynamically (during 5°/s sweep) rather than statically
2. Increase LUT resolution to 0.5° (720 entries, 5760 bytes) for better interpolation
3. Reduce SWEEP_VEL_I_CAP to ~0.3V after LUT active (less snap energy)
4. Return to position tracking (not velocity PI) for natural braking

**Expected result:** 80-90% cogging reduction → residual force ~0.05V → position tracking's proportional term can overcome without building snap energy → minimal oscillation.

**Risk:** Even after good calibration, some oscillation may remain at the strongest detents. The 10% residual cogging is ~0.05V; the proportional term at 1° error generates ~0.07V — barely enough.

**Effort:** Medium. Firmware changes + ~10 minute calibration procedure.

### Option B: NEMA 17 Stepper + TMC2209 (~$13–18 total, <$50 budget)

| Spec | Value |
|---|---|
| Steps/rev (16x micro) | 3200 → 0.11°/step |
| Speed at 1°/s | 3.3 steps/sec (trivially slow, no resonance) |
| Backward motion possible? | No (open-loop) |
| Cogging | No BLDC cogging. Detent torque exists but is handled by stepping mechanism |
| Firmware complexity | Very low. No FOC, no LUT, no PID tuning |
| Cost | NEMA17 ~$5–8, TMC2209 ~$5–10 |
| Position tracking | Open-loop (add AS5600 encoder for ~$3 for closed-loop confirmation) |

**Standard for:** telescope mounts, microscope stages, spectrometer gratings, 3D printers — exactly the class of application we're building.

**Limitation:** Physical size/weight may be different. Requires mechanical redesign to mount. Motor mounting interface may not be compatible.

### Option C: Slotless BLDC Motor (~$20–40 motor, same driver)

Slotless motors (no iron core) have near-zero cogging. Used in professional gimbals requiring precise slow motion (e.g., T-Motor GB series). Would use same SimpleFOC driver and ESP32. No cogging LUT needed.

**Cost:** Motor ~$20–40. Same DRV8313 driver and MT6701 encoder work. Low mechanical redesign.

**Limitation:** Lower torque density than slotted motors. May not hold scanner head weight without power. Need to verify stall torque spec for our load.

---

## Updated Next Steps (Priority Order)

### Decision Point: Hardware vs Software Fix

Given the research, the cleanest path forward depends on acceptable oscillation level:

**If any oscillation is unacceptable:**
→ Option B (stepper) or Option C (slotless BLDC). The physics of iron-core BLDC at sub-1 RPM fundamentally produce some cogging oscillation.

**If small residual oscillation (< 0.5°) is acceptable:**
→ Option A (better LUT calibration). 88-90% cogging reduction should bring backward excursions below 0.3° and eliminate worst_2s stall failure.

### If pursuing Option A (Software):

1. **Revert to position tracking** (undo velocity PI changes) — position tracking provides natural braking, TORQUE_KP=4/LAG=15 gave best results (2.01% backward steps)

2. **Dynamic LUT calibration**: Run slow sweep at 5°/s (above stall threshold), record position error vs. angle over multiple revolutions, build correction table from measured error (not from static hold voltage)

3. **Reduce I_CAP** from 2.0V to 0.3–0.5V after LUT active — with 88% cogging cancelled, the remaining detent force is small, so small I_CAP is sufficient and produces gentle snap-through instead of violent

4. **Run sweep_validation** — expect worst_2s to improve significantly (detent force below proportional term → no stall), backward steps to improve (smaller snap → smaller bounce)

### If pursuing Option B (Stepper):

1. Spec NEMA17 with sufficient stall torque for scanner head load
2. Wire TMC2209 to ESP32 (STEP/DIR signals, standard interface)
3. Implement simple step-generation firmware (or use existing stepper library)
4. Optionally add AS5600 encoder for position confirmation
5. Retire SimpleFOC/FOC infrastructure (or keep for future experiments)

---

## Sweep Validation Test Procedure

**Tool:** `tools/sweep_validation.py`

**Tests:**
1. 1°/s from 0° — 60s (actual scan speed)
2. 1°/s from 270° — 30s (strong detent start)
3. 5°/s from 0° — 20s
4. 5°/s from 270° — 20s
5. 1°/s full revolution — 90s

**Pass criteria:**
- Speed error < 15%
- worst_2s ≥ commanded_speed × 0.20 (no sustained stall/oscillation in any 2s window)
- backward_pct < 2.0% AND max_back_deg < 2.0°

**Current best result (TORQUE_KP=4, LAG=15, position tracking):**
- Speed: ✓ PASS
- worst_2s: ✗ 0.000 deg/s (oscillation at 30°, 90°, 300°)
- Backward: ✗ 2.01% (barely over threshold, max 2.28°)
