# Development Log 11: How to Test the Motor Controller

## Overview

The OpenHyperspectral motor controller uses a BLDC gimbal motor with SimpleFOC for field-oriented control and an MT6701 magnetic encoder for position feedback. Testing validates the entire signal chain from hardware through closed-loop control.

### Signal Chain
```
MT6701 Encoder (I2C) → SimpleFOC Sensor → loopFOC() → Driver PWM → Motor Phases → Mechanical Movement
     ↑                                                                                    ↓
     └────────────────────────── Position Feedback ───────────────────────────────────────┘
```

### Two Sources of Position Data
1. **Encoder (Hardware Truth)**: `getAbsolutePositionDeg()` - Raw MT6701 reading via I2C
2. **SimpleFOC (Control State)**: `getCurrentPositionDeg()` - Internal `shaft_angle` used by FOC algorithm

These should match. Divergence indicates sensor integration problems.

---

## Quick Reference: Serial Commands

```
Control:
  e/enable       Enable motor (requires calibration first)
  d/disable      Disable motor
  c/calibrate    Run SimpleFOC calibration
  m <deg>        Move to absolute position (0-360)
  stop           Emergency stop

Info:
  s/status       Encoder + FOC state snapshot
  i/info         System info (chip, pins, config)
  scan           I2C bus scan
  debug <0/1>    Periodic status output off/on

Tests:
  test/diag      Full diagnostic (recommended starting point)
  motor_test     Quick +30° move
  sweep          5-position accuracy test
  phase_test     Driver phase verification
  align          Motor holding strength test
  encoder_test   Interactive encoder reading
```

---

## The Main Diagnostic Sequence (`test` or `diag`)

Run this first when debugging any motor issue. It validates the system layer by layer.

### T1: Hardware Check
**What it tests:** I2C communication, encoder response, magnetic field strength

**How it works:**
- Sends I2C probe to encoder address (0x06)
- Reads magnetic field status register
- Reads current encoder position

**Results interpretation:**
| Result | Meaning |
|--------|---------|
| `OK (Enc:X° Field:good)` | Hardware working, field strength optimal |
| `WARN (Field:STRONG)` | Magnet too close - may work but noisy |
| `WARN (Field:WEAK)` | Magnet too far - may work but noisy |
| `FAIL (I2C error N)` | No communication - check wiring, power |
| `FAIL (Field:0xNN)` | Unexpected status - encoder may be damaged |

**If T1 fails:**
- Check I2C wiring (SDA=GPIO11, SCL=GPIO12 by default)
- Verify encoder has 3.3V power
- Run `scan` to see all I2C devices
- Verify magnet is centered over encoder IC

---

### T2: Calibration
**What it tests:** Sensor-to-motor electrical alignment (zero_electric_angle, sensor_direction)

**How it works:**
1. Applies voltage at 4 electrical angles (0°, 90°, 180°, 270°)
2. Records encoder position at each
3. Calculates sensor direction (CW/CCW) from movement
4. Calculates zero_electric_angle offset
5. Verifies by commanding forward movement
6. Calls SimpleFOC `initFOC()` to finalize

**Results interpretation:**
| Result | Meaning |
|--------|---------|
| `OK (already calibrated, Dir:CW Zero:X°)` | Previously calibrated this session |
| `Dir:CW Zero:X° Enc:Y° FOC:Y° Err:Z°` | Fresh calibration succeeded |
| `FAIL` | Motor didn't move during calibration |

**Critical values:**
- **Dir (CW/CCW)**: Sensor rotation direction relative to motor. Wrong value = motor runs backward
- **Zero**: Electrical offset in degrees. Wrong value = motor stalls or runs inefficiently
- **Err**: Should be <5°. Large error = FOC and encoder disagree

**If T2 fails:**
- Check motor has power (12V supply)
- Check motor phase wires are connected (all 3)
- Check driver isn't in fault state
- Try `phase_test` to verify driver outputs

---

### T3: Sensor Integration
**What it tests:** SimpleFOC's `loopFOC()` is actually reading the encoder

**How it works:**
- Resets encoder call counter
- Runs 100 FOC loop iterations
- Counts how many times `getSensorAngle()` was called

**Results interpretation:**
| Result | Meaning |
|--------|---------|
| `OK (100+ calls/100 loops)` | Sensor properly linked to motor |
| `FAIL (0 calls)` | Sensor not linked - `motor.linkSensor()` failed |
| `FAIL (<100 calls)` | Partial integration - timing issue |

**If T3 fails:**
- This is a firmware bug, not hardware
- Check `motor.linkSensor(&encoder)` was called in `begin()`
- Check encoder initialization succeeded

---

### T4: Open-Loop Test
**What it tests:** Driver, wiring, and power delivery (bypasses closed-loop control)

**How it works:**
- Switches to `angle_openloop` mode (no feedback)
- Commands +30° movement
- Measures actual encoder movement

**Results interpretation:**
| Result | Meaning |
|--------|---------|
| `OK (moved ~30°)` | Hardware is working |
| `OK (moved ~-30°)` | Hardware works but direction inverted |
| `FAIL (moved <10°)` | Motor didn't respond |

**Why open-loop matters:**
Open-loop bypasses all PID control. If this fails, the problem is hardware:
- Driver not outputting PWM
- Motor phases disconnected
- Power supply insufficient
- Driver in fault state

**If T4 fails:**
- Run `phase_test` to check each driver output
- Check motor phase connections
- Verify 12V power supply
- Check driver fault pin

---

### T5: Position Control
**What it tests:** Full closed-loop position control (PID + FOC + feedback)

**How it works:**
- Commands +30° relative move
- Runs control loop for up to 3 seconds
- Measures final position and error

**Results interpretation:**
| Result | Meaning |
|--------|---------|
| `OK (moved +30° err:<2°)` | Closed-loop control working |
| `PARTIAL (moved +30° err:>5°)` | Working but PID needs tuning |
| `FAIL (WRONG DIRECTION)` | Sensor direction is inverted |
| `FAIL (stalled)` | Motor not responding to commands |

**If T5 shows WRONG DIRECTION:**
This is the most common calibration issue. The motor moved but in the opposite direction commanded.
- Toggle `FORCE_SENSOR_DIRECTION_CW` in config.h
- Re-run calibration

**If T5 shows stalled:**
- T4 passed, so hardware is OK
- Problem is in closed-loop control
- Check PID gains (may be too low)
- Check zero_electric_angle (may be wrong by 90° or 180°)

---

## Individual Test Details

### `motor_test` - Quick Sanity Check
Fast 3-step test for iterative development:
1. Enable motor
2. Verify encoder tracking (Enc vs FOC difference)
3. Move +30° and check error

Use after making code changes to quickly verify nothing broke.

### `sweep` - Position Accuracy Test
Tests 5 positions: 0°, -15°, -30°, +15°, +30° from start position.

Validates:
- Positioning repeatability
- No systematic offset
- PID is properly tuned

If some positions pass and others fail, check for:
- Mechanical binding at certain angles
- Encoder noise at specific positions
- Cable interference

### `phase_test` - Driver Verification
Applies voltage at 6 electrical angles (0°, 60°, 90°, 120°, 180°, 240°).

Each angle energizes a different combination of phases:
- 0°: Phase A dominant
- 120°: Phase B dominant
- 240°: Phase C dominant
- Intermediate angles: combinations

If motor moves to some positions but not others, a phase is dead:
- Check that specific motor wire
- Check driver output for that phase

### `align` - Holding Strength Test
Applies voltage at 4 angles and prompts user to try rotating motor by hand.

Motor should:
- Hold firmly at each position
- Resist rotation with spring-like force
- Return to position when released

If motor is weak:
- Magnet may be demagnetizing
- Driver voltage limit too low
- Motor winding issue

### `encoder_test` - Interactive Reading
Continuously prints encoder values while user rotates motor by hand.

Watch for:
- Smooth progression (no jumps)
- Full 0-360° range covered
- Enc and FOC values tracking together

Common issues:
- Jumps at certain positions = encoder noise or bad magnet alignment
- FOC not tracking = sensor integration issue
- Limited range = encoder axis offset from motor axis

---

## Interpreting Periodic Status Output

When `debug 1` is enabled, status prints every 10 seconds:
```
[123s] Enc:45.2° | FOC:45.2° 0.0°/s | IDLE En:Y Cal:Y
```

| Field | Source | Meaning |
|-------|--------|---------|
| `123s` | System | Uptime in seconds |
| `Enc:45.2°` | MT6701 | Hardware encoder position (truth) |
| `FOC:45.2°` | SimpleFOC | Control algorithm's position belief |
| `0.0°/s` | SimpleFOC | Velocity estimate |
| `IDLE/MOVE/ERR/CAL` | Firmware | Motor state machine |
| `En:Y/N` | Firmware | Motor enabled flag |
| `Cal:Y/N` | Firmware | Calibration complete flag |

**Key diagnostic: Enc vs FOC**
- Should match within ~1°
- Growing divergence = encoder drift or FOC accumulation error
- Sudden jump = encoder glitch or wiring issue

---

## Common Failure Patterns

### Motor vibrates but doesn't move
- **Cause**: Wrong zero_electric_angle (off by ~90°)
- **Fix**: Re-run calibration, or toggle FORCE_SENSOR_DIRECTION_CW

### Motor runs backward
- **Cause**: Sensor direction inverted
- **Fix**: Toggle FORCE_SENSOR_DIRECTION_CW in config.h

### Motor overshoots target
- **Cause**: PID gains too aggressive
- **Fix**: Reduce P_angle.P, increase LPF_velocity.Tf

### Motor slow to reach target
- **Cause**: PID gains too conservative
- **Fix**: Increase P_angle.P, reduce damping

### Encoder reads but motor doesn't move
- **Cause**: Driver fault or no power
- **Fix**: Check 12V supply, run phase_test

### Everything passes but position drifts over time
- **Cause**: Encoder noise causing FOC accumulation error
- **Fix**: Check magnet alignment, increase filter alpha

---

## Testing Workflow

### Initial Bring-up
1. `scan` - Verify I2C devices present
2. `test` - Run full diagnostic
3. If all pass, motor is ready

### After Code Changes
1. `motor_test` - Quick sanity check
2. If issues, run full `test`

### PID Tuning
1. `c` - Ensure calibrated
2. `e` - Enable motor
3. `m 90` - Move to position
4. `m 180` - Move to another position
5. Watch for overshoot/undershoot
6. Adjust PID gains in config.h
7. `sweep` - Validate accuracy

### Debugging Hardware Issues
1. `scan` - I2C devices present?
2. `encoder_test` - Encoder working?
3. `phase_test` - All phases working?
4. `align` - Motor can hold position?
5. `test` - Full diagnostic

---

## Architecture Notes

### Why Two Position Sources?

**Encoder (getAbsolutePositionDeg)**
- Direct I2C read from MT6701
- Always 0-360° (absolute)
- No accumulated error
- Slower (I2C latency)

**SimpleFOC (getCurrentPositionDeg)**
- Internal shaft_angle variable
- Can accumulate beyond 360° (tracks rotations)
- Used by FOC algorithm for control
- Fast (cached value)

The firmware uses encoder as "truth" for position reporting but SimpleFOC's internal state for control. This is intentional - SimpleFOC needs continuous angles for smooth PID control across the 0/360° boundary.

### Why Manual Calibration?

SimpleFOC's automatic calibration expects fast sensor response for movement detection. The MT6701 I2C interface is too slow (~400kHz I2C) for reliable auto-detection. Manual calibration applies known voltages and measures response, which is more reliable for slow sensors.
