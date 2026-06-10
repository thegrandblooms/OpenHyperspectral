# Motor Control — Testing Guide

**Purpose:** How to test the system, interpret results, and debug failures.

---

## Flashing Firmware

All firmware is compiled and uploaded with the `flash.bat` script, which wraps `arduino-cli.exe` (included in `tools/`). Arduino IDE is NOT required for flashing — only for initial library install.

```bat
:: From the repo root:
tools\flash.bat

:: With explicit port (default is COM3):
tools\flash.bat COM4
```

**What it does:** Compiles the sketch at `firmware/ESP32_MCU_Firmware/` against the three library paths (SimpleFOC, SerialTransfer, MT6701), then uploads to the ESP32-S3 via USB-Serial/JTAG at 921600 baud. Typical compile+flash time is ~15 seconds after the first cached build.

**Prerequisites:**
- `tools/arduino-cli.exe` — already in repo (no install needed)
- `A:\Code_and_Data\Arduino\sketchbook\libraries\Simple_FOC` — SimpleFOC library
- `A:\Code_and_Data\Arduino\sketchbook\libraries\SerialTransfer` — SerialTransfer library
- `firmware/libraries/MT6701` — MT6701 driver (in repo)

**Before flashing:**
- Close Arduino IDE serial monitor if open (COM port must be free)
- Close SpectrumBoi or any other app using COM3
- The ESP32-S3 does NOT need to be in bootloader mode — it uses USB-Serial/JTAG which auto-resets

**If upload fails with "port busy":** Something else has COM3 open. Check Arduino IDE, PuTTY, SpectrumBoi.

**If upload fails with "No device found":** Try pressing and holding BOOT on the board, then click Upload, then release BOOT.

**Direct arduino-cli command** (same as what flash.bat runs, useful for debugging):
```bash
tools/arduino-cli.exe compile --upload --port COM3 \
  --fqbn "esp32:esp32:esp32s3:UploadSpeed=921600,USBMode=hwcdc,CDCOnBoot=cdc,..." \
  --library "A:\Code_and_Data\Arduino\sketchbook\libraries\Simple_FOC" \
  --library "A:\Code_and_Data\Arduino\sketchbook\libraries\SerialTransfer" \
  --library "A:\Code_and_Data\Spectrometry\OpenHyperspectral\firmware\libraries\MT6701" \
  "A:\Code_and_Data\Spectrometry\OpenHyperspectral\firmware\ESP32_MCU_Firmware"
```
(See `flash.bat` for the full FQBN string with all board options.)

---

## Iterative Development Cycle

The standard loop for any firmware change:

```
1. Edit config.h or motor_control.cpp (or tests.cpp)
2. Run: tools\flash.bat                          (~15s)
3. Run: py -3.9 tools\motor_test.py --preset baseline
4. Read: tools\last_run.txt
5. Repeat
```

**Why last_run.txt:** The motor_test.py script captures all serial output and saves it to `tools/last_run.txt` after every run. This means test results can be read without copy-pasting from the serial monitor. Claude can also read last_run.txt directly to analyze results.

**Useful grep patterns for last_run.txt:**
```bash
# Just T1–T5 verdicts and final result:
grep -E "T[1-5]|RESULT:|Summary:" tools/last_run.txt

# Just the T5 position table (no streaming noise):
grep -E "^\s+[0-9]+\.[0-9]" tools/last_run.txt

# Strip $ENC streaming lines for clean reading:
grep -v "^\$ENC," tools/last_run.txt | grep -v "^\$STREAM"
```

**Parameter change workflow:**

Most tuning is config.h only — no logic changes:

1. Open `firmware/ESP32_MCU_Firmware/config.h`
2. Change one parameter (e.g. `PID_I_VELOCITY`)
3. `tools\flash.bat`
4. `py -3.9 tools\motor_test.py --preset baseline`
5. Check T5 summary: how many stable/oscillating/chaotic?
6. Update the comment in config.h with what you observed (e.g. `// I=0.2 → 2 oscillating`)
7. Repeat

**When to use which preset:**

| Preset | When to use |
|--------|-------------|
| `--preset baseline` | After any firmware change — full T1–T5 suite |
| `--preset sweep` | Quick accuracy check after PID changes (5 positions, ~30s) |
| `--preset quick` | Sanity check after structural code changes (just moves +30°) |
| `--preset oscillation` | Characterizing stability at 4 cardinal positions |
| `--stream N` | Capture streaming encoder data for signal analysis |

**Streaming data for analysis:**
```bash
# Capture 10 seconds of position data while motor holds a position:
py -3.9 tools\motor_test.py --preset oscillation --stream 10
```
The `$ENC,<timestamp_ms>,<pos_deg>,<vel_deg_s>,<target_deg>` lines can be piped to Python/Excel for FFT, settling time analysis, etc.

---

## Signal Chain

```
MT6701 Encoder (I2C) → SimpleFOC sensor → loopFOC() → Driver PWM → Motor → Movement
        ↑                                                                    ↓
        └──────────────────── Position Feedback ─────────────────────────────┘
```

Two position sources (should match within ~1°):
- **Hardware truth:** `encoder.getDegrees()` — raw MT6701 I2C read, always 0–360°
- **Control state:** `getPosition()` — `motor.shaft_angle fmod 360°`, used by FOC

---

## Serial Commands

```
Control:
  c / calibrate     Run SimpleFOC calibration (required before enable)
  e / enable        Enable motor
  d / disable       Disable motor
  m <deg>           Move to absolute position (0–360°)
  stop              Emergency stop

Diagnostics:
  test / diag       Full T1–T5 suite (start here for any issue)
  motor_test        Quick +30° move sanity check
  sweep             5-position accuracy test (±30° from current position)
  phase_test        6-position driver phase verification
  encoder_test      Interactive encoder reading while rotating by hand
  align             Holding strength test
  tune on/off       Repeating ±30/60/90° PID tune cycle

Info:
  s / status        Encoder + FOC state snapshot
  i / info          System info (chip, pins, config)
  debug <0/1>       Toggle periodic heartbeat output
  stream on/off     Toggle $ENC, CSV position streaming
  stream rate N     Set stream rate in Hz (1–500)
  scan              I2C bus scan
```

---

## The T1–T5 Diagnostic Suite

Run `test` or `diag`. Tests validate layer by layer. If T(n) fails, fix it before running T(n+1).

### T1: Hardware Check
Tests I2C communication, field strength, encoder response.

| Result | Meaning | Fix |
|--------|---------|-----|
| `OK (Enc:X° Field:good)` | Hardware working | — |
| `WARN (Field:STRONG)` | Magnet too close | Adjust magnet distance |
| `WARN (Field:WEAK)` | Magnet too far / spacers | Adjust spacers |
| `FAIL (I2C error N)` | No communication | Check wiring, 3.3V power, I2C address (0x06) |

If T1 fails: run `scan` to see all I2C devices. Check SDA=GPIO47, SCL=GPIO48.

### T2: Calibration
Tests sensor-to-motor electrical alignment (zero_electric_angle, sensor_direction).

**How it works:** Applies voltage at 4 electrical positions, records encoder response, computes direction + ZEA. Then resets motor to UNKNOWN/NOT_SET and calls `initFOC()` — SimpleFOC's internal alignment routine runs 500 steps × 2ms = ~1 second, reading the sensor every 2ms. I2C at 400kHz delivers ~1000 reads/sec, which is marginally sufficient. Occasional I2C retries or bus contention can produce stale samples → noisy ZEA values. (GitHub #172)

| Result | Meaning |
|--------|---------|
| `Dir:CCW Zero:X° Err:<5°` | Good calibration |
| `Err:>5°` | Warning — FOC and encoder disagree, may still work |
| `FAIL` | Motor didn't move — check power, phase wires, driver fault |

**Critical:** sensor direction should be `CCW` for this motor/encoder combination. If `CW` is detected, something changed (phase wires swapped, magnet flipped, etc.).

### T3: Sensor Integration
Verifies SimpleFOC's `loopFOC()` is actually reading the encoder.

| Result | Meaning | Fix |
|--------|---------|-----|
| `OK (100 calls / 100 loops)` | Sensor properly linked | — |
| `FAIL (0 calls)` | Sensor not linked | Check `motor.linkSensor(&encoder)` |
| `FAIL (<100 calls)` | Partial integration | Firmware bug |

### T4: Open-Loop Test
Tests driver, wiring, power delivery — bypasses all closed-loop control.

**Note:** This test uses `velocity_openloop` mode at 1kHz loop rate with NO `loopFOC()` calls. All three of these conditions are required for T4 to pass reliably. See HISTORY.md.

| Result | Meaning | Fix |
|--------|---------|-----|
| `OK (moved ~50°)` | Hardware working | — |
| `OK (moved ~-50°)` | Hardware works, direction inverted | Investigate direction config |
| `FAIL (moved <10°)` | Motor didn't respond | Run `phase_test`, check power |

### T5: Position Control + Field Uniformity
Full 360° sweep at 24 positions (15° steps). Tests closed-loop control AND maps encoder/field quality around the full rotation.

Output columns: `Pos° | Field | RawNoise | Osc° | Err° | Status`

Each position is classified by oscillation range over a 300ms hold:

| Status | Osc Range | Meaning |
|--------|-----------|---------|
| `stable` | <0.3° | Good — no oscillation |
| `settling` | <1.0° | Acceptable — minor transient |
| `OSCILLATING` | <3.0° | Limit cycle present |
| `CHAOTIC` | ≥3.0° | Severe — needs diagnosis |

**Pass criteria:** 0 chaotic positions AND ≥75% stable+settling.

`RawNoise` = spread (max−min) of 50 rapid I2C reads in encoder counts. High spread at one arc → magnet misalignment or tilt.

`Field` = magnetic field status: `OK` (good), `HI` (too strong), `LO` (too weak).

If widespread oscillation across most positions: indicates global control loop instability — see Issue 2 in STATE.md. If oscillation concentrated in one arc: magnet tilt or encoder noise.

---

## Common Failure Patterns & Fixes

| Symptom | Most Likely Cause | Fix |
|---------|------------------|-----|
| Motor draws 0A despite 6V commanded | Wrong sensor direction (CW forced) | Verify `FORCE_SENSOR_DIRECTION_CW = false` |
| shaft_angle frozen throughout move | Same as above | Same fix |
| Motor runs backward | Sensor direction wrong | Let auto-cal run cleanly |
| Motor oscillates violently on step command | Step input without ramping | Ensure setpoint ramping is in `update()` |
| Motor hunts / can't settle | Integral windup (I_vel too high) | Reduce I_vel; check ramp is working |
| Position error ~0.4° consistent | P-only steady-state droop | Characterize: systematic → offset table; random → small I_pos |
| Oscillation at most/all positions | Global cascaded velocity-loop instability | Increase LPF_velocity (0.01→0.05); try D_pos=2.0; try torque mode |
| Oscillation concentrated in one arc | Magnet tilt / encoder noise | Check T5 RawNoise column; adjust magnet alignment |
| T4 fails (~1 in 15 runs) | Loop rate / mode / loopFOC interaction | Verify T4 uses: velocity_openloop, 1kHz, no loopFOC |
| `[SYNC]` errors, systematic oscillation | Double getSensorAngle() call | Verify only one read per loop (inside loopFOC) |
| Encoder reads jump/glitch | I2C clock stretching (ESP32-S3 errata) | Try 100kHz Wire.setClock; check pull-ups (4.7kΩ) |
| Calibration error > 30° | Multi-point calibration ZEA math wrong | Use SimpleFOC auto-cal (current config is correct) |
| `[MOVE_TIMEOUT]` printed | Motor couldn't reach target in 3s | Check power, phase_test, PID params |

---

## Testing Workflows

### Initial Bring-Up
1. `scan` — verify encoder at 0x06
2. `test` — full T1–T5
3. If T1–T5 pass: `c`, `e`, `m 90`, `m 0`, `sweep`

### After Code Changes
1. `motor_test` — quick sanity check
2. If issues: full `test`

### PID Tuning
1. `c` → `e`
2. `stream on` (or use debug output)
3. `m 90` — most problematic position
4. Observe: does it reach target? Overshoot? Oscillate?
5. Change one parameter at a time in config.h, reflash, repeat
6. `sweep` to validate all positions

### Oscillation Diagnosis (using $ENC, stream)
1. `stream on` (auto-starts on enable)
2. Capture serial output during oscillation at problem position
3. Parse CSV lines starting with `$ENC,`:
   `$ENC,<timestamp_ms>,<position_deg>,<velocity_deg_s>,<target_deg>`
4. FFT of position vs. time:
   - 1–5 Hz → PID gain issue (reduce P or add D)
   - 5–20 Hz → filter lag / phase margin (reduce LPF Tf or filter alpha)
   - >20 Hz → electrical/commutation issue

### Characterizing Steady-State Error
1. `c` → `e`
2. `sweep` or manual `m` commands to 8+ evenly-spaced positions
3. Record `[AT_TARGET]` lines: commanded vs. actual
4. Check: consistent direction (always high/low) → apply correction table; random → add small I_pos

---

## Interpreting Status Output

```
[HB] 123s | Enc:45.6° FOC:45.5° Δ:0.1° Raw:1234 | IDLE En:Y

[STATUS] MT6701: Raw=1234 Pos=45.60° | FOC: Pos=45.58° Vel=0.12°/s
         State=IDLE En=Y Cal=Y | Diff(Enc-FOC)=0.02°

[AT_TARGET] Current: 49.57°, Target: 50.00°, Error: 0.43°, Vel: 0.55°/s
```

Key diagnostic: **Enc vs. FOC difference** should be <1°. Growing divergence = encoder drift or FOC accumulation error. Sudden jump = encoder glitch or I2C issue.

`$ENC,<ms>,<pos>,<vel>,<target>` — streaming data, parse with Python.
