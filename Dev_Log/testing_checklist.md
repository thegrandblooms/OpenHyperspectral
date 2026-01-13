# Motor Control Testing Checklist

**Date:** 2026-01-10
**Status:** 🔄 **READY FOR HARDWARE TESTING** - Velocity jump fix implemented, awaiting validation

---

## 🎯 Recent Fix: Velocity Jump at 0°/360° Boundary

**Problem:** Motor showed 10M+ °/s velocity spikes when crossing boundaries
**Solution:** Removed `getAngle()` override to use SimpleFOC's continuous angle tracking
**Confidence:** VERY HIGH (98%) - Root cause mathematically proven

---

## Test 1: Verify Velocity Jump Fix (CRITICAL)

### Objective
Confirm that velocity stays smooth when motor crosses 0°/360° boundary (no more millions of degrees/second spikes).

### Test Procedure

1. **Compile and Upload Firmware:**
   ```bash
   cd firmware/ESP32_MCU_Firmware
   pio run -t upload
   pio device monitor
   ```

2. **Run Motor Test:**
   ```
   test motor
   ```

3. **What to Look For:**

   **Expected Behavior (FIXED):**
   - ✅ Velocity stays < 100 rad/s (< 5,730 °/s) during boundary crossing
   - ✅ Motor reaches target position smoothly
   - ✅ No hunting or rapid oscillation
   - ✅ Position error decreases steadily
   - ✅ Calibration completes successfully (20-30° movement)

   **Previous Broken Behavior:**
   - ❌ Velocity jumps to 1M+ °/s or 10M+ °/s
   - ❌ Motor hunts/vibrates rapidly
   - ❌ Position error increases instead of decreases
   - ❌ Motor gets "stuck" after crossing boundary

4. **Key Metrics to Monitor:**

   ```
   Target: [target_deg]°
   Current: [current_deg]°
   Velocity: [velocity_deg_s]°/s  ← Should be < 5,730 °/s (not millions!)
   shaft_angle: [shaft_angle_rad] rad  ← Can be negative or >2π now
   full_rotations: [count]  ← Will change when crossing boundary (this is OK!)
   ```

5. **Specific Test Cases:**

   **Test A: Cross 0° from high angle**
   ```
   Motor at: 350° → Command: 10°
   Expected: Smooth movement through 0°, velocity < 100 rad/s
   ```

   **Test B: Cross 360° from low angle**
   ```
   Motor at: 10° → Command: 350°
   Expected: Smooth movement through 360°, velocity < 100 rad/s
   ```

   **Test C: Multiple boundary crossings**
   ```
   Motor at: 0° → Command: 350° → Command: 10° → Command: 355°
   Expected: Smooth movements every time
   ```

### Success Criteria

- [ ] Velocity stays < 100 rad/s (< 5,730 °/s) during boundary crossings
- [ ] Motor reaches target within POSITION_TOLERANCE_DEG (5°)
- [ ] No hunting or oscillation
- [ ] Calibration completes successfully

### If Test Fails

**If velocity still spikes:**
1. Check that firmware was compiled and uploaded correctly
2. Verify `MT6701Sensor::getAngle()` is NOT defined in motor_control.cpp
3. Check SimpleFOC library version (should be 2.3.0+)
4. Review serial output for any errors during initialization

**If motor hunts/oscillates:**
1. Velocity fix may be working but PID tuning needed (see Test 3)
2. Check that `LPF_velocity.Tf = 0.01` (low-pass filter)
3. Verify electrical commutation is correct (no jerky movements)

---

## Test 2: Verify Calibration Still Works

### Objective
Ensure manual calibration works correctly with the velocity fix.

### Test Procedure

1. **Run Calibration:**
   ```
   test calibrate
   ```

2. **What to Look For:**

   **Expected Behavior:**
   - ✅ Motor moves 20-30° during calibration
   - ✅ Sensor direction detected as CCW (for current setup)
   - ✅ `zero_electric_angle` calculated correctly
   - ✅ No wild jerking or backwards movement
   - ✅ Test movements after calibration work smoothly

   **Broken Behavior:**
   - ❌ Motor only moves 1-2° (calibration failed)
   - ❌ Motor moves backwards when commanded forward
   - ❌ Wild jerking with 10M+ °/s velocities
   - ❌ Sensor direction detected as CW but physical is CCW

3. **Key Metrics:**

   ```
   [CALIBRATE] Mechanical angle change: [X]° (should be 20-30°)
   [CALIBRATE] Detected direction: CCW (for current setup)
   [CALIBRATE] zero_electric_angle: [value] rad
   ```

### Success Criteria

- [ ] Motor moves 20-30° during calibration
- [ ] Direction detected correctly (CCW for current setup)
- [ ] `zero_electric_angle` calculated without errors
- [ ] Motor moves smoothly in phase tests

---

## Test 3: Position Control Performance

### Objective
Verify motor can reach target positions accurately without hunting.

### Test Procedure

1. **Run Position Tests:**
   ```
   test motor
   ```

2. **Monitor Multiple Positions:**

   Test various positions across the full 360° range:
   - 0° (start position)
   - 90° (quarter turn)
   - 180° (half turn)
   - 270° (three-quarter turn)
   - 45°, 135°, 225°, 315° (intermediate positions)

3. **What to Look For:**

   **Expected Behavior:**
   - ✅ Motor settles at target within 5° (POSITION_TOLERANCE_DEG)
   - ✅ Settling time < 2 seconds for 90° movements
   - ✅ No overshoot > 10°
   - ✅ No hunting or oscillation around target
   - ✅ Velocity decreases smoothly as motor approaches target

   **Performance Issues:**
   - ⚠️ Large overshoot (> 10°) → PID tuning needed (reduce P_position)
   - ⚠️ Hunting around target → PID tuning needed (reduce D_position or increase LPF)
   - ⚠️ Slow settling (> 3s) → PID tuning needed (increase P_position)
   - ⚠️ Never reaches target → Check velocity fix worked correctly

### Success Criteria

- [ ] Motor reaches all test positions accurately
- [ ] No hunting or oscillation
- [ ] Settling time < 2 seconds for 90° movements
- [ ] Position error < 5° at steady state

---

## Test 4: Current PID Parameter Verification

### Current Configuration (config.h)

```cpp
// Position PID
#define PID_P_POSITION   20.0   // Proportional gain
#define PID_I_POSITION   0.0    // Integral gain (disabled)
#define PID_D_POSITION   0.0    // Derivative gain (disabled)

// Velocity PID
#define PID_P_VELOCITY   0.2    // Proportional gain
#define PID_I_VELOCITY   20.0   // Integral gain
#define PID_D_VELOCITY   0.0    // Derivative gain (disabled)
#define PID_LPF_VELOCITY 0.01   // Low-pass filter (10ms)

// Velocity/Acceleration Limits
#define MAX_VELOCITY_DEG      200.0   // deg/s (3.49 rad/s)
#define MAX_ACCELERATION_DEG  500.0   // deg/s²
```

### Expected Performance

**Position PID (P=20.0):**
- High proportional gain → Fast response
- I=0, D=0 → Simple P-only control
- Good for: Quick movements without overshoot
- Risk: May oscillate if too high

**Velocity PID (P=0.2, I=20.0):**
- Low P, high I → Smooth movements, strong holding torque
- Standard for gimbal motors
- Low-pass filter (0.01) → Reduces velocity noise

**Velocity Limits:**
- 200 °/s max velocity → Moderate speed (33 RPM)
- 500 °/s² acceleration → Smooth acceleration

### Tuning Guidelines (If Needed)

**If motor oscillates/hunts:**
1. Reduce `PID_P_POSITION` from 20.0 to 10.0
2. Add small D term: `PID_D_POSITION = 0.5`
3. Increase LPF: `PID_LPF_VELOCITY = 0.02`

**If motor is too slow:**
1. Increase `MAX_VELOCITY_DEG` to 300.0
2. Increase `PID_P_POSITION` to 30.0
3. Check that velocity fix is working (no spikes)

**If motor has large overshoot:**
1. Reduce `PID_P_POSITION` from 20.0 to 15.0
2. Reduce `MAX_ACCELERATION_DEG` to 300.0
3. Add velocity ramp smoothing

---

## Test 5: Boundary Crossing Stress Test

### Objective
Repeatedly cross 0°/360° boundary to verify stability.

### Test Procedure

1. **Manual Rapid Boundary Crossings:**
   - Command: 350° → wait for settle
   - Command: 10° → wait for settle
   - Repeat 10 times

2. **What to Monitor:**
   - Velocity during crossing (should stay < 100 rad/s)
   - `full_rotations` changes (will decrement/increment, this is OK)
   - `shaft_angle` can be negative or >2π (this is expected now)
   - Position error calculation (should handle wraparound correctly)

3. **Look for Accumulating Errors:**
   - After 10 crossings, command back to 0°
   - Motor should return to starting position accurately
   - No cumulative drift or offset

### Success Criteria

- [ ] Velocity stays smooth through all 10 crossings
- [ ] No accumulating position errors
- [ ] Motor returns to 0° accurately after test
- [ ] No random jumps or hunting

---

## Test 6: Multi-Turn Tracking Verification

### Objective
Verify that continuous angle tracking allows multiple revolutions (even though MT6701 is single-turn).

### Test Procedure

1. **Command Multiple Full Rotations:**
   ```
   Command: 0° → 360° → 720° → 1080° (3 revolutions)
   ```

2. **What to Look For:**

   **Expected Behavior (with continuous tracking):**
   - ✅ Motor completes multiple revolutions smoothly
   - ✅ `shaft_angle` increases beyond 2π (e.g., 4π, 6π)
   - ✅ `full_rotations` increments for each complete turn
   - ✅ Velocity stays smooth throughout

   **Note:** While MT6701 is a single-turn encoder (always returns 0-2π), SimpleFOC's `full_rotations` tracking allows commanding multi-turn movements. The motor will physically rotate multiple times, with the software tracking cumulative position.

3. **Return to Start:**
   ```
   Command: 0°
   ```
   Motor should return to original position (may take shortest path).

### Success Criteria

- [ ] Motor can complete multiple revolutions
- [ ] `full_rotations` tracks correctly
- [ ] Velocity stays smooth
- [ ] Motor can return to starting position

---

## Test 7: Edge Cases and Error Handling

### Test Procedure

1. **Test wrap-around position targets:**
   - Command: -10° (should work, wraps to 350°)
   - Command: 370° (should work, wraps to 10°)
   - Command: 720° (2 full rotations)

2. **Test rapid command changes:**
   - Command: 180°
   - Immediately command: 0° (before reaching 180°)
   - Motor should smoothly change direction

3. **Test boundary conditions:**
   - Command: 0.0°
   - Command: 359.9°
   - Command: 0.1°

### Success Criteria

- [ ] All positions reachable (including negative and >360°)
- [ ] Motor handles rapid command changes smoothly
- [ ] No crashes or undefined behavior

---

## Post-Testing Analysis

### If All Tests Pass ✅

**Celebrate!** The velocity jump fix is working correctly. Next steps:

1. ✅ Mark Task #4 complete in todo.md
2. 📝 Update Dev_Log/dev_log_6.md with test results
3. 🔧 Optional: Fine-tune PID parameters if needed
4. ✅ Move on to remaining hardware tests (Tasks #9, 10, 12)

### If Tests Fail ❌

**Debugging Steps:**

1. **Verify firmware upload:**
   - Check git commit hash matches latest: `dfd74b6`
   - Confirm motor_control.cpp line 170+ has comment about getAngle() NOT overridden
   - Check binary upload logs for errors

2. **Check SimpleFOC library version:**
   ```bash
   pio pkg list
   ```
   Should show `simplefoc/SimpleFOC @ ^2.3.0`

3. **Enable debug logging:**
   - Ensure `DEBUG_MOTOR = true` in config.h
   - Check serial output for SimpleFOC internal state

4. **Document failure mode:**
   - Capture serial output showing the issue
   - Note which specific test failed
   - Check if problem is consistent or intermittent

---

## Hardware Test Status Tracking

| Test # | Test Name | Status | Date | Notes |
|--------|-----------|--------|------|-------|
| 1 | Velocity Jump Fix | ⏳ Pending | - | Critical test |
| 2 | Calibration | ⏳ Pending | - | Should work same as before |
| 3 | Position Control | ⏳ Pending | - | Main functional test |
| 4 | PID Parameters | ⏳ Pending | - | Verify or tune |
| 5 | Boundary Stress Test | ⏳ Pending | - | Stability check |
| 6 | Multi-Turn Tracking | ⏳ Pending | - | Verify continuous tracking |
| 7 | Edge Cases | ⏳ Pending | - | Error handling |

**Legend:**
- ⏳ Pending - Not yet tested
- ✅ Passed - Test successful
- ⚠️ Partial - Test passed with minor issues
- ❌ Failed - Test failed, needs debugging

---

## Expected Serial Output (Reference)

### Successful Calibration
```
[MOTOR] Manual calibration starting...
[CALIBRATE] Initial sensor reading: 123.45°
[CALIBRATE] Electrical 0° → Mechanical: 234.56°
[CALIBRATE] Electrical 90° → Mechanical: 256.78°
[CALIBRATE] Mechanical angle change: 22.22°
[CALIBRATE] Detected direction: CCW
[CALIBRATE] zero_electric_angle: 4.321 rad
[CALIBRATE] ✓ Calibration successful
```

### Successful Motor Test
```
[TEST] Motor Test - Position Control
Target: 90.00°
Current: 2.45°, Velocity: 145.32°/s  ← Initial movement
Current: 45.67°, Velocity: 189.23°/s  ← Approaching
Current: 78.34°, Velocity: 98.45°/s   ← Slowing down
Current: 88.92°, Velocity: 23.12°/s   ← Near target
Current: 90.12°, Velocity: 5.34°/s    ← At target
✓ Target reached! Error: 0.12°
```

### Fixed Velocity (No More Jumps!)
```
# Crossing 0° boundary:
Current: 355.87°, Velocity: 45.23°/s   ← Before boundary
Current: 358.12°, Velocity: 52.34°/s   ← Approaching 0°
Current: 1.45°, Velocity: 48.67°/s     ← After boundary ✅ SMOOTH!
Current: 5.78°, Velocity: 43.21°/s     ← Continuing smoothly

# Previous broken behavior (reference):
Current: 355.87°, Velocity: 45.23°/s
Current: 285.91°, Velocity: 10,445,362.22°/s  ← MILLIONS! (BROKEN)
```

---

**End of Testing Checklist**
