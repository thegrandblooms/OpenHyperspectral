# Dev Log 14: Multi-Point Calibration Rollback

**Date:** 2026-02-18
**Branch:** `claude/fix-motor-oscillations-lsefN`
**Status:** Rolled back to `9de17c9` (pre-calibration state). Spacers installed.

---

## Session Summary

Over three commits (`4189584`, `f8bf048`, `e473e7f`), we attempted to fix position-dependent oscillation by:
1. Replacing SimpleFOC's auto-calibration with a custom 24-point multi-point calibration
2. Adding a PID auto-tuner (adaptive I_vel reduction) to the test sequence
3. Reducing the EMA filter alpha and adding a position deadband
4. Then reverting the filter/deadband changes and adding calibration warmups

None of these improved T5 stability. Performance degraded from 8/24 stable (baseline) to 2/24 stable. Motor torque also appeared significantly reduced. We rolled back all three commits.

---

## What Was Tried and Why It Failed

### Multi-Point Calibration (commit `4189584`)

**Concept:** Sample the encoder at 24 electrical positions (15° apart), compute the zero electrical angle (ZEA) via circular mean (atan2 of sin/cos sums), then feed ZEA + direction to SimpleFOC's `initFOC()`.

**Problem:** The calibration consistently produced a ~12° tracking error (`Enc:194.2° FOC:182.1° TrkErr:12.0°`). This means the motor's electrical field was misaligned from the rotor's actual position by 12°, which:
- Reduces torque (field not orthogonal to rotor)
- Creates position-dependent errors (the misalignment manifests differently at each position)
- Explains why nearly every position oscillated — the motor couldn't hold position cleanly with reduced torque

The 12° error persisted even after adding filter warmups (commit `e473e7f`), which reduced it from ~30° but couldn't eliminate it. The root cause is likely in the ZEA math — the circular mean may be correct for the electrical angle samples but the 225° offset compensation was empirically tuned and may not generalize.

### Reduced Filter Alpha (commit `f8bf048`)

**Change:** FILTER_ALPHA 0.4 → 0.275 (heavier smoothing)

**Why it failed:** Heavier filtering = more phase lag. At 1kHz loop rate:
- alpha=0.4: ~2.0ms effective lag
- alpha=0.275: ~3.1ms effective lag (55% more)

With P_pos=20, the system was already marginally stable. The extra 1.1ms of lag pushed more positions past the stability boundary. Stable positions dropped from 8/24 to 4/24.

**Lesson:** Never reduce filter alpha without compensating (e.g., reducing P_pos or increasing D_pos).

### Position Deadband (commit `f8bf048`)

**Change:** 0.25° deadband — PID output zeroed when |error| < 0.25°

**Why it failed:** Discontinuous control law creates limit cycles:
1. Motor approaches target → error enters deadband → PID drops to zero
2. Motor coasts past on momentum (no braking)
3. Exits deadband on other side → full PID force
4. Drives back → enters deadband → repeat

This converted stable positions into oscillating ones. Run 2 (with deadband) had uniform ~2° oscillation at 14/24 positions — classic limit-cycle behavior.

**Lesson:** Never use deadbands in continuous control loops. If you want to reduce jitter at rest, use an integrator or feedforward term, not a discontinuous threshold.

### D_pos Increase (commit `e473e7f`)

**Change:** D_pos 1.0 → 2.0

**Rationale:** Derivative term provides phase lead to counteract filter lag, improving phase margin.

**Outcome:** Couldn't be evaluated in isolation because the multi-point calibration's 12° tracking error dominated. Worth re-testing after rollback.

---

## Key Learnings

1. **Calibration accuracy is paramount.** A 12° ZEA error kills performance more than any PID tuning can compensate for. SimpleFOC's built-in `initFOC()` auto-calibration, while less sophisticated, produces correct alignment.

2. **RawNoise in T5 measures motor vibration, not encoder noise.** The 50-sample measurement runs without `mc.update()`, so if the motor is oscillating, the readings reflect physical movement. This is why noise patterns change dramatically between runs with different PID params.

3. **Oscillation pattern shifts with control parameters = control issue, not hardware.** If oscillation were from cogging (51.4° period for 7 pole pairs) or eccentricity, the pattern would be fixed across parameter changes. It isn't.

4. **Hardware update: 1.5mm spacers installed** between encoder IC and magnet. The magnet was physically rubbing against the encoder — spacers should reduce any friction-related effects.

---

## Current State After Rollback

- Firmware is back to `9de17c9` state
- T2 uses SimpleFOC's built-in `calibrate()` (auto-calibration)
- No PID auto-tuner in test sequence
- No multi-point calibration function
- D_pos = 1.0, FILTER_ALPHA = 0.4
- Spacers newly installed (not yet tested with this firmware)

---

## Recommended Next Steps

1. **Flash and test** with the rolled-back firmware + spacers. This will give us a clean baseline with the hardware improvement.

2. **If baseline is good (>8/24 stable):** Try D_pos=2.0 as the only change. This was theoretically sound but couldn't be evaluated due to calibration interference.

3. **If multi-point calibration is revisited:** Debug the ZEA calculation — compare the computed ZEA against SimpleFOC's auto-calibration ZEA and understand the discrepancy. The 225° offset hack is suspicious.

4. **Consider velocity feedforward** for the scanning use case rather than relying solely on PID position control.
