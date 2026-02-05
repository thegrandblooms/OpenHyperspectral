# TODO: Motor Control Architecture Cleanup

**Date:** 2026-02-05
**Status:** PLANNED
**Priority:** HIGH - blocking ability to reason about and fix oscillation bug
**Prerequisite:** Fix the dual-read oscillation bug first, THEN rename/reorganize

---

## Problem: Naming Confusion Makes Debugging Nearly Impossible

The motor control code has **three parallel tracking systems** for "position," and their
names give no indication of which system they belong to. When reading code or debug output,
you can't tell whether a value came from the hardware encoder, SimpleFOC's internal state,
or our custom bridge layer.

---

## The Three Layers (Current Names → Proposed Names)

### Layer 1: Hardware Encoder (MT6701Sensor)
**What it is:** Direct I2C reads from the MT6701 magnetic encoder. Ground truth.
**Range:** Always 0-2pi (wraps at boundary)

| Current Name | Called Where | What It Returns | Proposed Name |
|---|---|---|---|
| `encoder.getSensorAngle()` | update() line 1071, loopFOC() internal | 0-2pi radians, Cartesian-filtered | `encoder.readFilteredRadians()` |
| `encoder.getDegrees()` | update() line 1150, status output | 0-360 degrees (cached from last read) | `encoder.readDegrees()` |
| `encoder.getRawCount()` | diagnostics | 0-16383 raw counts | OK as-is |
| `encoder_rad` (local var) | update() line 1071 | 0-2pi from hardware | `hw_encoder_rad` |

### Layer 2: SimpleFOC Library Internals
**What it is:** SimpleFOC's internal state machine. Maintained by loopFOC()/move().
**Range:** Continuous (can be negative or >2pi via full_rotations)

| Current Name | Called Where | What It Returns | Proposed Name |
|---|---|---|---|
| `motor.shaft_angle` | everywhere | Radians - BUT gets overwritten by 3 different sources! | See note below |
| `motor.shaftAngle()` | inside motor.move() | sensor_direction * getAngle() - offset | N/A (internal to SimpleFOC) |
| `Sensor::angle_prev` | inside sensor->update() | Last getSensorAngle() result (0-2pi) | N/A (internal to SimpleFOC) |
| `Sensor::full_rotations` | inside sensor->update() | Rotation counter for continuous tracking | N/A (internal to SimpleFOC) |
| `motor.shaft_velocity` | isAtTarget(), timeouts | Radians/sec from angle differences | OK as-is |

**Critical problem with `motor.shaft_angle`:** This single variable is written by THREE sources:
1. `motor.loopFOC()` sets it from sensor via `shaftAngle()` (line 1065)
2. Our code overwrites it with `continuous_position_rad` (line 1113)
3. `motor.move()` overwrites it AGAIN via `shaftAngle()` (line 1134, internally)

This means `motor.shaft_angle` has a DIFFERENT value depending on WHEN you read it
during a single update() call. This is the core source of confusion.

### Layer 3: Custom Bridge Layer (continuous_position_rad)
**What it is:** Our custom wraparound-safe position tracking. Built to work around
SimpleFOC's full_rotations corruption issue.
**Range:** Continuous (can be negative or >2pi)

| Current Name | Called Where | What It Returns | Proposed Name |
|---|---|---|---|
| `continuous_position_rad` | update() lines 1075-1132 | Unwrapped position from encoder deltas | `unwrapped_position_rad` |
| `prev_encoder_rad` | update() lines 1087-1110 | Previous encoder read for delta calc | `prev_hw_encoder_rad` |
| `continuous_wrapped` (local) | update() line 1075 | continuous_position_rad mod 2pi | `unwrapped_mod2pi` |
| `normalized_target_rad` (local) | update() line 1132 | Target in continuous space | `target_continuous_rad` |

---

## The Dual-Read Problem (Architecture Bug)

The naming confusion directly obscures the core bug: `getSensorAngle()` is called
**twice per update() cycle**, once by SimpleFOC and once by our code. Each call does
a fresh I2C read AND advances the Cartesian filter, creating **two divergent
position streams**.

```
update() execution order:
─────────────────────────────────────────────────────────────────
1. motor.loopFOC()              → I2C READ #1 → SimpleFOC tracks as value "A"
                                   shaft_angle = f(A)
2. encoder.getSensorAngle()     → I2C READ #2 → We track as value "B"
                                   encoder_rad = B (different from A!)
3. motor.shaft_angle = continuous_position_rad  (based on B series)
4. motor.move(target)           → internally: shaft_angle = shaftAngle()
                                   (overwrites line 3 with A series!)
                                   PID error = target(B) - position(A) = BIASED
─────────────────────────────────────────────────────────────────
```

This is invisible unless you know the naming: `encoder.getSensorAngle()` on line 1071
looks like it's just "reading the encoder" but it's actually the SECOND read that
creates a parallel tracking universe.

---

## Proposed Reorganization Steps

### Step 1: Fix the Dual-Read Bug FIRST
Before renaming anything, eliminate the architectural problem. Two approaches:

**Approach A: Remove second read, use SimpleFOC's tracking**
- Delete lines 1071-1113 (entire custom tracking block)
- Use `motor.shaft_angle` (from loopFOC) directly for target calculation
- Requires trusting SimpleFOC's `full_rotations` (should work now that
  `resetRotationTracking()` exists)
- Test: does oscillation stop?

**Approach B: Remove first read, make custom tracking the sole source**
- Override `update()` in MT6701Sensor to prevent SimpleFOC from reading separately
- Make our custom tracking feed SimpleFOC instead of competing with it
- More complex but gives us full control

### Step 2: Rename for Clarity
After the architecture is fixed (only ONE tracking system), rename:

1. **Functions:**
   - `getSensorAngle()` → `readFilteredRadians()` (clarifies: does I2C + filter)
   - `getDegrees()` → `getCachedDegrees()` (clarifies: returns cached, not fresh read)
   - `getPosition()` → `getPositionDeg()` (clarifies units)
   - `getEncoderDegrees()` → `readEncoderDeg()` (clarifies: fresh I2C read)

2. **Variables:**
   - If custom layer survives: `continuous_position_rad` → `unwrapped_position_rad`
   - `prev_encoder_rad` → `prev_hw_encoder_rad`
   - `encoder_rad` locals → `hw_encoder_rad`
   - `target_position_deg` → OK (already clear)

3. **Comments/Sections:**
   - Add `// === HARDWARE ENCODER READ ===` section markers
   - Add `// === SIMPLEFOC CONTROL ===` section markers
   - Remove stale comments referencing deleted tracking systems

### Step 3: Update Diagrams
- Update `Dev_Log/t4_t5_signal_flow.md` Mermaid diagrams with new names
- Diagrams should become readable once names indicate which layer they belong to

### Step 4: Update Dev Logs
- Add dev log entry documenting the before/after architecture
- Reference this TODO as completed

---

## Why This Matters

The current naming makes it almost impossible to:
1. **Debug oscillation** - you can't tell which "position" is wrong
2. **Understand signal flow** - Mermaid diagrams are confusing because names overlap
3. **Reason about correctness** - `shaft_angle` means different things at different lines
4. **Onboard new contributors** - five different "get position" functions with no naming convention

---

## Files to Modify

| File | Changes |
|------|---------|
| `motor_control.cpp` | Rename functions/variables, restructure update() |
| `motor_control.h` | Update declarations, class interface, comments |
| `tests.cpp` | Update any references to renamed functions |
| `config.h` | No changes expected |
| `Dev_Log/t4_t5_signal_flow.md` | Update Mermaid diagrams with new names |

---

## Estimated Scope

- Step 1 (fix dual-read): ~30-50 lines changed in motor_control.cpp
- Step 2 (rename): ~100 lines across motor_control.cpp/h, tests.cpp
- Step 3 (diagrams): regenerate 3 Mermaid diagrams
- Step 4 (dev log): new entry

**Order matters:** Fix the bug first, then rename. Renaming while the dual-read
bug exists just makes it a well-named bug.

---

**End of TODO**
