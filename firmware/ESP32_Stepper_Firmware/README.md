# ESP32 Stepper Firmware (motor control rewrite)

Replaces `ESP32_MCU_Firmware` (SimpleFOC + Mitoot 2804 gimbal motor) with a
stepper-based rotary stage:

```
ESP32-S3 ──STEP/DIR/EN──► TMC2209 ──► NEMA17 ──belt reduction──► output shaft
   ▲                        ▲                                        │
   │ UART (current,         └ StealthChop, 1/16 µsteps,              │
   │  µsteps, hold current)    MicroPlyer 1/256 interpolation        │
   └────────────── SPI ◄── MT6835 21-bit absolute encoder ◄──────────┘
```

## Why the rewrite

Documented in `Dev_Log/motor/STATE.md` / `HISTORY.md`. Short version:

- The 12N14P gimbal motor has 84 cogging detents/rev with a ~0.45V break-free
  barrier. Every control architecture (cascade PID, torque PD, velocity PI,
  LUT feedforward, sinusoidal feedforward) hit the same stall-snap-bounce
  limit cycle at scan speeds, and positions could oscillate indefinitely.
- The MT6701's ±1.0° INL capped absolute accuracy regardless of control.
- Requirement is ≤0.1° (target 0.02°) at the output shaft.

The stepper architecture solves both structurally:

- **Passive holding.** A stepper at rest needs no feedback — the hunting
  failure mode cannot occur.
- **Resolution.** 200 steps × 16 µsteps × 3:1 belt = 9600 steps/output-rev
  = 0.0375°/µstep. Microstep *accuracy* (±0.05–0.1° at the motor) divides by
  the belt ratio → ±0.02–0.03° at the output before trim.
- **Truth at the output.** The MT6835 (0.00017°/count) sits on the output
  shaft, after the belt — so lash and compliance are measured, not guessed,
  and a move-then-trim loop removes them.

## Control strategy: move-then-trim (no continuous servo)

1. Move open-loop with a trapezoid profile (20kHz timer ISR, DDA step generator).
2. If the net move direction opposes the configured approach direction,
   overshoot by `BACKLASH_OVERSHOOT_DEG` and return, so the final leg always
   loads the belt the same way (`UNIDIRECTIONAL_APPROACH`).
3. Settle `TRIM_SETTLE_MS`, read the output encoder, and correct the residual
   with a small move. Repeat up to `TRIM_MAX_ITERATIONS` or until
   `TRIM_TOLERANCE_DEG` (default 0.02°).
4. Hold passively. No control action at rest → no oscillation, ever.

Sweeps (`sweep <deg/s>`) run a constant step rate; at 1°/s through a 3:1 belt
that's ~27 steps/s — far below any resonance band, and StealthChop +
MicroPlyer interpolation keep it smooth and silent.

## Scan modes (the two capture strategies under evaluation)

### 1. Stepped capture — `scan_step <start> <end> <inc> [dwell_ms=500]`

Move → settle → trim to 0.02° → emit `$FRAME,<idx>,<ms>,<target_deg>,<enc_deg>,<err_deg>`
→ dwell (host captures during this window) → next position. The final
approach is forced **into the scan direction** at every frame, so belt lash
is loaded identically for every capture. Encoder-verified position at every
frame; throughput ~1–2 frames/s including settle+trim.

```
$SCAN_START,<ms>,step,<start>,<end>,<inc>,<dwell_ms>,<n_frames>
$FRAME,0,<ms>,...        ← trigger capture on each of these
...
$SCAN_END,<ms>,frames=<n>
```

### 2. Smooth video scan — `scan_smooth <start> <end> <speed_deg_s>`

Pre-positions to the start point **approaching in the scan direction** (lash
taken up before recording begins), auto-enables `$ENC` streaming, emits
`$SCAN_START,<ms>,smooth,<start>,<end>,<speed>`, runs a constant step rate
through the range, emits `$SCAN_END,<ms>,<enc_deg>` and holds. The host
records video concurrently and frame-matches against the encoder log
afterwards using the `sync` clock handshake (same workflow as the gimbal-era
`sweep_validation.py`).

Smoothness note: one 1/16 microstep is 0.0375° at the output, but with
StealthChop + MicroPlyer the TMC2209 interpolates each step into 1/256
increments spread over the step interval, so constant-rate motion is
quasi-continuous. Matching precision comes from the encoder log, not the
step quantum.

**Speed selection for a 30fps camera:** `speed = bin_deg × 30 / frames_per_bin`.
For 0.02° bins with 1–4 frames each → 0.15–0.6 °/s. For 0.1° bins → 0.75–3 °/s.

Both modes interpret the range as the shortest path (< 180°). `scan_stop`
aborts either mode (emits `$SCAN_ABORT,<ms>,frame=<idx>`); `stop` and
`disable` also abort. Mechanical evaluation before camera integration:
run each mode with `stream rate 200` and compare the `$ENC` logs —
trim_err field shows lash/compliance, velocity shows sweep uniformity.

## Hardware bring-up checklist

1. **Pins are placeholders.** Verify every pin in `config.h` against the
   actual wiring / free GPIOs on the Waveshare ESP32-S3-Touch-LCD-2.
2. **Set `GEAR_RATIO`** in `config.h` to the real pulley ratio.
3. **Set `TMC_RMS_CURRENT_MA`** ≤ the motor's rated current.
4. **TMC2209 UART** is single-wire: ESP32 TX → 1kΩ → PDN_UART, ESP32 RX →
   PDN_UART direct. If you skip UART wiring, set `USE_TMC_UART false` and
   configure microsteps via MS1/MS2 strapping + current via the Vref pot.
5. **MT6835**: verify the SPI frame layout and CRC against your datasheet rev
   (driver assumes mode 3, 4-bit cmd + 12-bit addr + 8-bit data, burst 0xA,
   CRC-8 poly 0x1D). `encoder_test` is the bring-up command. Run the MT6835's
   built-in self-calibration once after mechanical assembly to shrink its INL.
6. First boot: `c` (calibrate) → `e` → `m 90` → `status`, then `recalibrate`
   to measure backlash, then `test` for the full T1–T5 suite.

## Library dependencies

- **TMCStepper** (Arduino Library Manager) — only when `USE_TMC_UART true`.
- **SerialTransfer** — same as the gimbal firmware (binary protocol).
- No SimpleFOC, no FOC, no PID library.

## Serial protocol compatibility

Command set, `$ENC`/`$SWEEP_START`/`$SWEEP_END`/`$SYNC`/`$SET` markers, and
the `RESULT:` test verdicts match the gimbal firmware, so `tools/motor_test.py`
and `tools/sweep_validation.py` work unchanged. The `$ENC` line keeps 6 fields;
the last two are now `cmd_pos_deg` (step counter) and `trim_err_deg`
(encoder − commanded) instead of `voltage_q`/`elec_angle`.

Removed (no longer meaningful): `cogmap`, `cogmap_dense`, `cogreset`,
`pidtune`, `tune`, `phase_test`, `align`, `scan` (I2C), `mode`, `set pid_*`,
`set cog_*`. New: `set trim 0|1`, `set trim_tol <deg>`, `recalibrate`
(backlash measurement).

## Files

| File | Origin |
|---|---|
| `config.h` | New — stepper hardware + mechanics + trim parameters |
| `motor_control.h/.cpp` | New — step ISR, trapezoid profile, move-then-trim |
| `MT6835.h/.cpp` | New — standalone 21-bit SPI encoder driver |
| `tests.h/.cpp` | New — T1–T5 rewritten for stepper (same RESULT markers) |
| `ESP32_Stepper_Firmware.ino` | Adapted from `ESP32_MCU_Firmware.ino` |
| `communication.*`, `commands.h`, `debug_globals.h` | Copied verbatim |
