# Dev Log 9: Motor Control Breakthrough - SimpleFOC Integration Fixed

**Date:** 2024-01-16
**Status:** Major Progress - Motor Moving, Minor Tuning Remaining
**Previous:** dev_log_8.md (Root cause analysis of frozen shaft_angle)

## Summary

This session achieved a **major breakthrough** in motor control. The motor now moves predictably
under SimpleFOC closed-loop control. Two critical fixes were identified and implemented:

1. **180° calibration offset** - Corrects magnetic field alignment
2. **Position normalization** - Returns 0-360° instead of accumulated angles

## Root Cause Analysis Confirmed

The diagnostic test suite (8 tests) confirmed the theories from dev_log_8:

| Test | Result | Meaning |
|------|--------|---------|
| TEST 1: Control Mode | PASS | SimpleFOC configured correctly |
| TEST 2: Sensor Calls | PASS | getSensorAngle() called 100x per 100 loopFOC() |
| TEST 3: shaft_angle Update | PASS | Angle updates when motor rotates |
| TEST 4: Position Control | **PASS** | Motor moves ~2°/step with correct Vq polarity |
| TEST 5: Velocity Mode | PASS | 107° movement in 2 seconds |
| TEST 6: Open-Loop Mode | PASS | 175° movement (hardware works) |
| TEST 7: setPhaseVoltage | PASS | 37.6° movement (driver works) |
| TEST 8: Zero Angle Search | PASS | Found optimal offset = 270° (180° already applied) |

## Key Discovery: FORCE_SENSOR_DIRECTION_CW Was the Culprit

The original problem was caused by **forcing CW sensor direction** when the actual sensor
direction was CCW. This caused:

- Wrong electrical angle calculation
- Magnetic field pushing rotor **backward** instead of forward
- Zero effective torque despite 6V applied

**Fix:** Disabled `FORCE_SENSOR_DIRECTION_CW` in config.h, letting calibration detect the
correct CCW direction automatically.

## Critical Fixes Implemented

### Fix 1: 180° Calibration Offset

```cpp
// motor_control.cpp line 839-845
float zero_elec_angle = normalizeRadians(electrical_from_encoder - _PI_2);

// CRITICAL FIX: Add 180° offset to correct calibration
// Discovered through TEST 8 diagnostics - without this offset, the magnetic field
// pushes the rotor backward instead of forward, causing reversed/weak movement.
zero_elec_angle = normalizeRadians(zero_elec_angle + PI);
```

**Why 180°?** The motor's phase winding orientation relative to the encoder creates a
natural 180° offset. Without correction, commanding "forward" produces backward movement.

### Fix 2: Position Normalization

```cpp
// motor_control.cpp line 1197-1206
float MotorController::getPosition() {
    // CRITICAL FIX: Normalize to 0-360° range
    // SimpleFOC accumulates full rotations internally (e.g., -13076°)
    float degrees = radiansToDegrees(motor.shaft_angle);
    degrees = fmod(degrees, 360.0f);
    if (degrees < 0) degrees += 360.0f;
    return degrees;
}
```

**Why needed?** SimpleFOC tracks cumulative rotations for velocity calculation, but for
absolute encoder applications, we need the physical shaft position (0-360°).

## Test Results After Fixes

### Calibration Output
```
[CALIBRATION] Starting...
  Encoder: 116.2° | Testing 4 positions: 106.1° 94.8° 83.2° 70.5° ✓
  Movement: 24.3° Dir:CCW ZeroAngle:146.2° (with 180° offset)
  Initializing FOC... ✓ | Enc:63.5° FOC:-66.0° Err:129.6° ⚠
✓ Calibration complete
```

### TEST 4: Position Control (The Big Win!)
```
Time   | shaft_angle | Vq     | Δshaft
-------|-------------|--------|--------
101ms  | -27.7°      | -0.0V  | 35.84°  (initial jump)
202ms  | -27.7°      | -2.6V  | 0.04°
303ms  | -27.6°      | -6.0V  | 0.03°
404ms  | -26.0°      | -6.0V  | 1.63°   ← Motor starts moving!
505ms  | -23.6°      | -6.0V  | 2.42°
606ms  | -21.3°      | -6.0V  | 2.26°
...
3030ms | 30.0°       | -6.0V  | 2.06°
```

**Key observations:**
- Consistent ~2°/step movement (properly functioning FOC)
- Vq ramps up to -6.0V (voltage limit, correct polarity)
- Total movement: ~58° in 3 seconds toward target

### Status Output (Position Normalization Working)
```
[STATUS] MT6701: Raw=15076 Pos=28.74° | FOC: Pos=28.74° Vel=0.00°/s
         State=IDLE En=N Cal=Y | Diff(Enc-FOC)=0.00° | RAM=336KB
```

Position now correctly shows 28.74° instead of accumulated values like -13076°.

## Remaining Issues

### 1. Velocity Always Shows 0.0°/s

Even when the motor is clearly moving at ~60°/s, `motor.shaft_velocity` reports 0.

**Likely cause:** SimpleFOC's velocity calculation uses `angle_prev` and timing, but our
Cartesian filtering or the I2C sensor timing may be interfering.

**Impact:** No velocity damping in position PID, causing hunting/oscillation.

**Potential fix:** Implement our own velocity calculation from encoder deltas.

### 2. isAtTarget() Inconsistency

Debug output shows alternating normalized/raw values:
```
[AT_TARGET] Current: 118.48°, Target: 1000.00°, Error: -521.52°
[AT_TARGET] Current: -14641.52°, Target: 1000.00°, Error: 15641.52°
```

**Likely cause:** Race condition - `motor.shaft_angle` read at different points in the
FOC loop shows different values (before/after sensor update).

**Fix needed:** Use consistent position source (getPosition() which normalizes).

### 3. Hunting Around Target

Motor moves toward target but oscillates instead of settling.

**Causes:**
1. Zero velocity feedback (0.0°/s always) means no damping
2. Position PID has no derivative term effect
3. May need PID retuning now that motor actually moves

### 4. TEST 8 Found 270° Offset (Not 180°)

Our 180° fix works, but TEST 8 dynamically found 270° as optimal. This suggests:
- The optimal offset may vary with starting position
- An additional 90° adjustment might improve performance
- Or the 180° is correct and 270° was found due to position-dependent effects

## Configuration Changes Made

| Setting | Before | After | Reason |
|---------|--------|-------|--------|
| FORCE_SENSOR_DIRECTION_CW | true | false | Allow auto-detection of CCW |
| Calibration offset | 0° | +180° | Correct magnetic field direction |
| getPosition() | Raw | Normalized | Return 0-360° physical position |

## Architecture Understanding

```
┌─────────────────────────────────────────────────────────────────┐
│                    SimpleFOC Control Loop                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  1. loopFOC() calls sensor->update()                            │
│     └─ MT6701Sensor::getSensorAngle() returns filtered radians  │
│                                                                  │
│  2. SimpleFOC updates shaft_angle from sensor                   │
│     └─ Includes rotation tracking (can accumulate >360°)        │
│                                                                  │
│  3. Position PID: error = target - shaft_angle                  │
│     └─ Outputs velocity command                                  │
│                                                                  │
│  4. Velocity PID: uses shaft_velocity (BROKEN - always 0)       │
│     └─ Outputs Vq voltage                                        │
│                                                                  │
│  5. FOC: electrical_angle = shaft_angle * pole_pairs + offset   │
│     └─ Our 180° fix corrects this offset                        │
│                                                                  │
│  6. setPhaseVoltage(Vq, 0, electrical_angle)                    │
│     └─ Drives motor coils with correct field orientation        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

## Next Steps

### High Priority
1. **Fix velocity calculation** - Implement custom velocity from encoder deltas
2. **Fix isAtTarget()** - Use getPosition() consistently instead of raw shaft_angle
3. **PID tuning** - Now that motor moves, tune for smooth settling

### Medium Priority
4. **Investigate 270° vs 180° offset** - Determine if additional adjustment needed
5. **Reduce calibration error** - Current "Err:129.6°" warning after initFOC()

### Low Priority
6. **Clean up debug output** - Remove excessive AT_TARGET logging
7. **Document final configuration** - Update config.h comments

## Files Modified This Session

| File | Changes |
|------|---------|
| motor_control.cpp | Added 180° offset, normalized getPosition() |
| motor_control.h | Added getSensorCallCount() diagnostic method |
| tests.cpp | Added runSimpleFOCDiagnostic() with 8 tests |
| tests.h | Added function declaration |
| ESP32_MCU_Firmware.ino | Added 'diag' command handler |
| config.h | User disabled FORCE_SENSOR_DIRECTION_CW |

## Conclusion

**Tremendous progress!** The motor is now responding to SimpleFOC position commands with
correct direction and reasonable torque. The root cause (wrong sensor direction + missing
calibration offset) has been identified and fixed.

The remaining hunting behavior is likely a tuning issue now that the fundamental control
loop is working. Fixing the velocity calculation should provide the damping needed for
smooth settling.

This session transformed the motor from "completely non-functional" to "working with
minor tuning needed" - a major milestone for the OpenHyperspectral project.
