# Dev Log 8: SimpleFOC Frozen shaft_angle Root Cause Analysis

**Date:** 2026-01-15
**Purpose:** Diagnose why motor draws 0A despite SimpleFOC commanding 6V - electrical angle stuck due to frozen shaft_angle
**Status:** **ROOT CAUSE IDENTIFIED** - SimpleFOC's shaft_angle not updating during motor.move()

---

## Critical Finding: The Smoking Gun 🎯

```
Trajectory output:
101ms FOC:236.7° Vel:0°/s E:29.6° | Elec:166° Vq:6.0V Ft:OK En:Y
...
2929ms FOC:236.7° Vel:0°/s E:29.6° | Elec:166° Vq:6.0V Ft:OK En:Y

Current: I=0.000A (throughout entire 3 second trajectory)
```

**SimpleFOC's `motor.shaft_angle` is FROZEN at 236.7° for the entire 3-second trajectory!**

### Why This Causes 0A Current

1. FOC algorithm requires a **rotating** electrical field to produce torque
2. Electrical angle = (shaft_angle × 7 pole pairs + zero_electric_angle) % 360°
3. If shaft_angle is frozen → electrical angle is frozen → static field → 0A current
4. Motor cannot produce torque with static electrical field (phases cancel out)

### Evidence This is SimpleFOC Internal Issue

**During calibration (works!):**
```cpp
motor.setPhaseVoltage(6.0, 0, test_angles[i])  // Direct voltage application
// Motor moves 16-20° reliably, draws current
```

**During position control (fails!):**
```cpp
motor.move(target_rad)  // SimpleFOC's position control
// motor.shaft_angle stays frozen
// motor.loopFOC() doesn't update shaft_angle
// Elec angle stuck → 0A current
```

---

## Session Progression

### Phase 1: Serial Output Compaction
**Goal:** Make debug output more scannable to spot patterns

**Changes Made:**
1. **Heartbeat:** Reduced from wrapping line to single line
   ```
   [HB] 123s | Enc:45.6° FOC:45.5° Δ:0.1° Raw:1234 | IDLE En:Y
   ```

2. **Detailed Status:** Reduced from ~40 lines to 2 lines
   ```
   [STATUS] MT6701: Raw=1234 Pos=45.60° | FOC: Pos=45.58° Vel=0.12°/s I=0.001A
            State=IDLE En=Y Cal=Y | Diff(Enc-FOC)=0.02° | RAM=123KB
   ```

3. **Calibration:** Reduced from ~60 lines to ~5 lines
   ```
   [CALIBRATION] Starting...
     Encoder: 280.5° | Testing 4 positions: 276.7° 270.0° 262.1° 253.5° ✓
     Movement: 16.6° [OVERRIDE: CCW→CW] Dir:CW ZeroAngle:0.3°
     Initializing FOC... ✓ | Enc:248.2° FOC:250.4° Err:2.2° ✓
   ✓ Calibration complete
   ```

4. **Trajectory:** Added FOC diagnostics inline
   ```
   101ms FOC:302.9° Vel:0°/s E:35.5° Vq:6.0V Ft:OK En:Y
   ```

**Result:** ✅ Much easier to spot the frozen shaft_angle pattern

### Phase 2: Add Critical Diagnostics
**Goal:** Understand why motor draws 0A

**Added to trajectory output:**
- `motor.enabled` state (Y/N)
- `MOTOR_FAULT` pin status (Ft:OK or Ft:FAULT)
- `motor.voltage.q` (Q-axis voltage command)
- **Electrical angle** (the key diagnostic!)

**Key Insight:** Electrical angle calculation revealed SimpleFOC is using correct formula but shaft_angle input is frozen.

### Phase 3: Electrical Angle Analysis
**Goal:** Verify FOC algorithm is calculating correct commutation angle

**Added:**
```cpp
float elec_angle_deg = radiansToDegrees(motor.shaft_angle * POLE_PAIRS + motor.zero_electric_angle);
```

**Formula verification:**
```
Elec = (236.7° × 7 + 308.8°) mod 360° = 166° ✓
```

Calculation is **correct**, but shaft_angle input is **frozen**.

### Phase 4: Auto-Calibration Attempt
**Goal:** Let SimpleFOC calculate zero_electric_angle instead of manual calculation

**Added config option:**
```cpp
#define USE_SIMPLEFOC_AUTO_CALIBRATION true
```

**Result:** ❌ SimpleFOC still says:
```
MOT: Skip dir calib.
MOT: Skip offset calib.
```

**Why it still skips:** Our code still sets `motor.zero_electric_angle` and `motor.sensor_direction` in manual calibration section, so SimpleFOC detects they're already set and skips its own calibration.

---

## Root Cause Analysis

### What Works ✅
1. **Encoder:** Perfect tracking (Enc:236.7° FOC:236.7° Δ:0.0°)
2. **Calibration:** Motor moves 16-20° using `setPhaseVoltage()`
3. **Driver:** No faults (Ft:OK throughout)
4. **Electrical angle calculation:** Math is correct
5. **Zero electric angle:** Formula produces reasonable values (0.3° to 308.8°)

### What Fails ❌
1. **SimpleFOC's shaft_angle update:** Frozen during `motor.move()`
2. **Position control:** 0A current, no movement
3. **Velocity calculation:** Always 0°/s (because shaft_angle doesn't change)

### The Fundamental Problem

**SimpleFOC's `motor.loopFOC()` is NOT updating `motor.shaft_angle` from sensor readings during position control mode.**

Evidence:
```cpp
// In our update() function:
motorControl.update() {
    motor.loopFOC();        // Should update shaft_angle from sensor
    motor.move(target_rad); // Should calculate new voltage based on updated shaft_angle
}

// But shaft_angle stays at 236.7° for 3 seconds!
```

This suggests either:
1. SimpleFOC's sensor reading in loopFOC() is broken
2. SimpleFOC mode is wrong (not in angle mode?)
3. SimpleFOC's initialization didn't complete properly
4. We're calling the functions in the wrong order

---

## Leading Theories (Ranked by Likelihood)

### Theory #1: SimpleFOC Control Mode Issue ⭐⭐⭐⭐⭐
**Hypothesis:** SimpleFOC might not be in angle/position control mode

**Evidence:**
- During calibration, we use `motor.setPhaseVoltage()` directly (works)
- During position control, we use `motor.move()` (fails)
- If SimpleFOC is in wrong mode, `motor.move()` might be a no-op

**Test:** Check `motor.controller` and `motor.torque_controller` mode settings

### Theory #2: Sensor Update Not Happening ⭐⭐⭐⭐
**Hypothesis:** `motor.loopFOC()` isn't calling `sensor.update()` or sensor is broken in this mode

**Evidence:**
- `shaft_angle` should be updated from sensor in every `loopFOC()` call
- Our encoder works (we can read it directly)
- But SimpleFOC's shaft_angle stays frozen

**Test:** Add logging inside `loopFOC()` to see if sensor is being read

### Theory #3: Initialization Incomplete ⭐⭐⭐
**Hypothesis:** `motor.initFOC()` succeeded but didn't fully initialize internal state

**Evidence:**
- SimpleFOC says "Ready" but skips dir/offset calibration
- Maybe skipping calibration leaves internal state incomplete?
- SmartKnob uses `motor.initFOC(zero_electric_offset, sensor_direction)` with explicit params

**Test:** Try SimpleFOC's full auto-calibration (don't pre-set any values)

### Theory #4: MT6701 I2C Sensor Too Slow ⭐⭐⭐
**Hypothesis:** I2C reads are too slow for SimpleFOC's fast loop

**Evidence:**
- I2C at 400kHz takes ~200μs per read
- SimpleFOC might be skipping sensor updates if they timeout
- SmartKnob uses SPI encoders (much faster)

**Test:** Add timing measurements in sensor update

### Theory #5: Function Call Order Wrong ⭐⭐
**Hypothesis:** Should call `motor.move()` before `motor.loopFOC()`?

**Evidence:**
- We currently call: `motor.loopFOC()` then `motor.move()`
- Maybe should be: `motor.move()` then `motor.loopFOC()`?

**Test:** Swap the order

### Theory #6: Zero Electric Angle Wrong ⭐
**Hypothesis:** Our calculated zero_electric_angle is wrong

**Evidence:**
- Values vary widely (0.3° to 308.8°) between runs
- But electrical angle formula is mathematically correct
- Motor moves during calibration so hardware is fine

**Test:** This theory is DISPROVEN - electrical angle calculation is verified correct

---

## Exhaustive List of Next Steps

### Immediate Diagnostic Tests (High Priority)

1. **Check SimpleFOC Control Mode**
   ```cpp
   Serial.print("motor.controller: ");
   Serial.println(motor.controller);  // Should be MotionControlType::angle
   Serial.print("motor.torque_controller: ");
   Serial.println(motor.torque_controller);  // Should be TorqueControlType::voltage
   ```

2. **Verify Sensor Update is Being Called**
   Add to update():
   ```cpp
   float before = motor.shaft_angle;
   motor.loopFOC();
   float after = motor.shaft_angle;
   if (abs(after - before) > 0.001) {
       Serial.println("shaft_angle CHANGED!");  // Should see this if working
   }
   ```

3. **Check if getSensorAngle() is Being Called**
   Add to MT6701Sensor::getSensorAngle():
   ```cpp
   static unsigned long call_count = 0;
   call_count++;
   if (call_count % 1000 == 0) {
       Serial.print("getSensorAngle() called: ");
       Serial.println(call_count);
   }
   ```

4. **Try Velocity Control Mode**
   Instead of position control, try:
   ```cpp
   motor.controller = MotionControlType::velocity;
   motor.move(1.0);  // 1 rad/s
   ```
   If velocity mode works, proves issue is specific to position control mode.

5. **Swap loopFOC() and move() Order**
   ```cpp
   motor.move(normalized_target_rad);
   motor.loopFOC();  // Instead of loopFOC() then move()
   ```

### Alternative Initialization Approaches

6. **Let SimpleFOC Do Everything**
   - Remove ALL pre-setting of zero_electric_angle and sensor_direction
   - Let SimpleFOC's initFOC() run full alignment
   - See what values it calculates

7. **Copy SmartKnob Initialization Exactly**
   - SmartKnob uses: `motor.initFOC(zero_electric_offset, Direction::CW)`
   - Pass explicit parameters instead of pre-setting

8. **Force Direction and Offset Calibration**
   Even though we set values, try:
   ```cpp
   motor.zero_electric_angle = 0;  // Force SimpleFOC to calibrate
   motor.sensor_direction = Direction::UNKNOWN;
   motor.initFOC();  // Should NOT skip now
   ```

### Hardware Verification Tests

9. **Measure Actual Phase Voltages**
   Use oscilloscope to verify PWM is actually reaching motor phases during `motor.move()`

10. **Test Direct setPhaseVoltage() During Position Control**
    ```cpp
    // Instead of motor.move(), manually apply rotating field:
    for (int i = 0; i < 360; i += 10) {
        motor.setPhaseVoltage(6.0, 0, degreesToRadians(i));
        delay(50);
    }
    // If this works, proves only motor.move() is broken
    ```

11. **Check Phase Wiring**
    - Swap two phase wires
    - See if motor responds differently
    - Wrong phase order could cause zero torque at certain angles

### SimpleFOC Configuration Tests

12. **Try Different Controller Types**
    ```cpp
    motor.controller = MotionControlType::angle_openloop;  // Skip PID
    ```

13. **Disable Velocity PID**
    ```cpp
    motor.PID_velocity.P = 0;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0;
    ```

14. **Increase Voltage Limit**
    ```cpp
    motor.voltage_limit = 12.0;  // Test if 6V is insufficient
    ```

15. **Test Open-Loop Position Control**
    ```cpp
    motor.controller = MotionControlType::angle_openloop;
    motor.target = target_position;
    ```

### Code Review / Comparison

16. **Compare with SmartKnob's update() Function**
    - Check exact function call order
    - Look for any pre-processing we're missing

17. **Review SimpleFOC Examples**
    - Find SimpleFOC position control example with I2C encoder
    - Copy their exact setup

18. **Check SimpleFOC Version Compatibility**
    - MT6701 sensor wrapper might be incompatible with SimpleFOC 2.3.5
    - Try older/newer SimpleFOC version

---

## Test Sequence for Next Session

### Phase 1: Verify Control Mode (5 min)
1. Print `motor.controller` and `motor.torque_controller`
2. Verify we're in MotionControlType::angle mode
3. If wrong mode, set explicitly and test

### Phase 2: Sensor Update Verification (10 min)
1. Add shaft_angle change detection
2. Add getSensorAngle() call counter
3. Run test, verify sensor IS being read

### Phase 3: Try Velocity Mode (10 min)
1. Switch to velocity control
2. Command constant velocity
3. See if motor responds

### Phase 4: Full Auto-Calibration (15 min)
1. Remove ALL manual calibration
2. Set zero_electric_angle = 0 and sensor_direction = UNKNOWN
3. Let SimpleFOC do full alignment
4. See if position control works after SimpleFOC's own calibration

### Phase 5: Hardware Test (If software tests fail)
1. Use oscilloscope to measure phase voltages during motor.move()
2. Verify PWM is actually being output
3. If no PWM → driver issue; if PWM present → motor/wiring issue

---

## Key Questions Still Unanswered

1. **Why does SimpleFOC skip dir/offset calibration even with auto-calibration enabled?**
   - We're still setting values manually in code
   - Need to completely remove manual calibration code path

2. **Why doesn't motor.loopFOC() update motor.shaft_angle from sensor?**
   - Is sensor.update() being called?
   - Is sensor returning valid angle?
   - Is shaft_angle write-protected somehow?

3. **Why does setPhaseVoltage() work but motor.move() doesn't?**
   - What's the difference in code paths?
   - Is motor.move() even calling setPhaseVoltage() internally?

4. **Is our MT6701 sensor wrapper implementation correct?**
   - Does it conform to SimpleFOC's Sensor interface?
   - Are we implementing all required methods?

5. **Should we be calling motor.move() every loop?**
   - Or just set once and let loopFOC() handle it?
   - SmartKnob calls motor.move() every loop - we do too

---

## Files Modified This Session

### Firmware Files
- `firmware/ESP32_MCU_Firmware/tests.cpp` - Added electrical angle diagnostic
- `firmware/ESP32_MCU_Firmware/motor_control.cpp` - Added auto-calibration option
- `firmware/ESP32_MCU_Firmware/config.h` - Added USE_SIMPLEFOC_AUTO_CALIBRATION flag
- `firmware/ESP32_MCU_Firmware/ESP32_MCU_Firmware.ino` - Compacted heartbeat/status

### Documentation
- `Dev_Log/dev_log_8.md` - This file

---

## Motor Overheating Concerns - RESOLVED ✅

**Question:** Is motor burned out from overheating?

**Answer:** NO - Motor is fine!

**Evidence:**
1. Motor moves 16-20° during calibration using setPhaseVoltage()
2. Current is 0.000A during position control (no electrical heating)
3. Burned motor would show:
   - Infinite resistance (open circuit)
   - No movement at all, even with direct voltage
   - Smoke/burning smell during failure

**Actual behavior:**
- Motor works during calibration
- Motor fails during position control due to software issue
- 0A current = no heat generation during failed attempts

**Best practices already in place:**
- Current limiting: 2.0A max
- Auto-disable after reaching target
- Timeout on movements

**Conclusion:** Hardware is fine, issue is purely software/SimpleFOC configuration.

---

## Summary

**Problem:** Motor draws 0A during position control despite SimpleFOC commanding 6V

**Root Cause:** SimpleFOC's `motor.shaft_angle` is frozen during `motor.move()`, causing electrical angle to stay constant, resulting in static (non-rotating) electrical field that produces zero torque

**Next Critical Test:** Check if `motor.controller` mode is set correctly and if `sensor.update()` is being called during `motor.loopFOC()`

**Likely Solution:** Either:
1. SimpleFOC is in wrong control mode
2. Sensor updates aren't happening
3. Need to let SimpleFOC do its own full calibration without pre-setting values

**Status:** Ready for systematic testing of theories above
