# OpenHyperspectral Motor Control - Todo List

**Last Updated:** 2026-01-10
**Status:** Planning phase after dev logs 1-5 review

---

## 🔴 Critical Priority

### 1. Fix zero_electric_angle Formula in Dev Log 1
**File:** `Dev_Log/dev_log_1.md` (line 479)

**Current (WRONG):**
```cpp
motor.zero_electric_angle = aligned_position - (_3PI_2 / POLE_PAIRS);
```

**Should Be:**
```cpp
motor.zero_electric_angle = (aligned_position * POLE_PAIRS) - _3PI_2;
// Or with normalization:
motor.zero_electric_angle = fmod((aligned_position * POLE_PAIRS) - _3PI_2 + _2PI, _2PI);
```

**Why:** Unit mismatch - subtracting mechanical angle from mechanical angle but storing as electrical angle. Critical for calibration to work.

**Status:** ⬜ Not Started

---

### 2. Investigate Motor Oscillation/Hunting Root Cause
**Issue:** Motor oscillates/hunts instead of settling at target position

**Investigation Steps:**
- [ ] Check current PID parameters (P, I, D values)
- [ ] Review shaft_angle tracking during movement
- [ ] Verify encoder readings during oscillation
- [ ] Check if oscillation frequency correlates with PID update rate
- [ ] Test with reduced P gain to see if oscillation decreases
- [ ] Verify voltage limits and current limits

**Possible Causes:**
- PID parameters too aggressive (P gain too high)
- Derivative term causing instability
- I2C latency affecting control loop timing
- Incorrect zero_electric_angle from formula error
- CCW direction handling in control loop

**Status:** ⬜ Not Started

---

### 3. Check Actual Firmware Code for Invalid SimpleFOC Enums
**File:** `firmware/ESP32_MCU_Firmware/motor_control.cpp`

**Look For:**
- `TorqueControlType::angle` (DOES NOT EXIST - should be voltage/dc_current/foc_current)
- Any other non-existent enum values

**Impact:** Won't compile or undefined behavior

**Status:** ⬜ Not Started

---

## 🟡 High Priority - Documentation Corrections

### 4. Correct Phase Angle Descriptions in Dev Log 2
**File:** `Dev_Log/dev_log_2.md` (lines 44-79)

**Wrong Descriptions:**
- Phase B positive at 90° → should be 120°
- Phase B→C transition at 120° → should be 180°

**Correct Pattern:**
- Phase A: 0°
- Phase B: 120° (2π/3)
- Phase C: 240° (4π/3)
- Transitions at 60°, 180°, 300°

**Status:** ⬜ Not Started

---

### 5. Correct I2C Timing Claim in Dev Log 1
**File:** `Dev_Log/dev_log_1.md` (lines 5-6)

**Wrong:** "~10-20ms per read"
**Correct:** "~1ms per read at standard speeds (100-400kHz)"

**Reality:** 10-20ms would indicate serious I2C problem. Actual timing is <1ms but still slower than SPI.

**Status:** ⬜ Not Started

---

### 6. Clarify LPF_angle Diagnosis in Dev Log 3
**File:** `Dev_Log/dev_log_3.md` (lines 105-156)

**Issue:** Presented as "discovery" and "fix" but didn't actually solve the problem

**Clarification Needed:**
- SimpleFOC initializes LPF_angle.Tf by default
- Setting to 0.0f disables filtering (not a fix)
- Uninitialized filter would cause noise, not "reset to 0°" behavior
- This was a red herring, not the root cause

**Status:** ⬜ Not Started

---

### 7. Document CCW Sign Inversion as Expected SimpleFOC Behavior
**Location:** Add note to relevant dev logs (3, 4, 5)

**Key Points:**
- `Direction::CCW = -1` in SimpleFOC source
- Formula: `shaft_angle = sensor_direction * angle - offset`
- Negative shaft_angle with CCW is working as designed
- SimpleFOC handles negative angles correctly
- Not a bug to fix, but expected behavior

**Status:** ⬜ Not Started

---

## 🟢 Medium Priority - Testing & Verification

### 8. Test Manual Calibration with Corrected Formula
**File:** `firmware/ESP32_MCU_Firmware/motor_control.cpp`

**Steps:**
1. Apply corrected zero_electric_angle formula
2. Run manual calibration sequence
3. Verify motor moves smoothly
4. Check if oscillation improves

**Status:** ⬜ Not Started
**Depends On:** Task #1 (fix formula)

---

### 9. Verify PID Tuning Parameters for Position Control
**File:** `firmware/ESP32_MCU_Firmware/config.h` or `motor_control.cpp`

**Current Parameters (from logs):**
```cpp
motor.P_angle.P = 20.0;  // Position P gain
motor.P_angle.I = 0.0;   // Position I gain
motor.P_angle.D = 0.0;   // Position D gain
```

**Investigation:**
- [ ] Is P=20.0 too aggressive for this motor?
- [ ] Should we enable D term for damping?
- [ ] Check velocity limit settings
- [ ] Verify output_ramp value

**Status:** ⬜ Not Started

---

### 10. Handle CCW Sensor Direction Correctly in Software
**File:** `firmware/ESP32_MCU_Firmware/motor_control.cpp`

**Verify:**
- [ ] Code handles negative shaft_angle values correctly
- [ ] Target position calculations account for CCW direction
- [ ] Position error calculations work with negative angles
- [ ] Wrap-around at 0°/360° boundary handled correctly

**Note:** Physical orientation is fine, fix in software only

**Status:** ⬜ Not Started

---

### 11. Add Instrumentation to Measure Actual I2C Read Timing
**File:** `firmware/ESP32_MCU_Firmware/MT6701Sensor.cpp`

**Implementation:**
```cpp
unsigned long start = micros();
cached_raw_count = encoder.readRawAngle();
unsigned long duration = micros() - start;
// Log timing data
```

**Goal:** Confirm actual I2C read latency (should be <1ms)

**Status:** ⬜ Not Started

---

### 12. Remove or Fix getAngle() Override if Present
**File:** `firmware/ESP32_MCU_Firmware/MT6701Sensor.cpp`

**Check For:**
```cpp
float MT6701Sensor::getAngle() override {
    return angle_prev;  // Override that caused sign inversion
}
```

**Action:**
- [ ] Check if override exists in current code
- [ ] Remove if present (caused sign inversion in Dev Log 4)
- [ ] Verify only getSensorAngle() is overridden (correct pattern)

**Status:** ⬜ Not Started

---

## 📊 Status Summary

**Total Tasks:** 12

- 🔴 Critical: 3 tasks
- 🟡 High Priority (Docs): 4 tasks
- 🟢 Medium Priority (Testing): 5 tasks

**Not Started:** 12
**In Progress:** 0
**Completed:** 0

---

## Priority Order for Execution

1. **Fix zero_electric_angle formula** (Task #1) - Critical math error
2. **Check firmware for invalid enums** (Task #3) - May prevent compilation
3. **Investigate oscillation root cause** (Task #2) - Main functional issue
4. **Verify PID parameters** (Task #9) - Likely related to oscillation
5. **Test corrected calibration** (Task #8) - Validate formula fix
6. Remaining documentation corrections (Tasks #4-7)
7. Software verification tasks (Tasks #10-12)

---

## Notes

- Physical orientation is confirmed fine - all fixes should be software-only
- Motor oscillation/hunting is the main functional problem, not necessarily moving backwards
- CCW sign inversion is expected SimpleFOC behavior, not a bug
- Many previous "fixes" were confident but wrong - verify with evidence before implementing

---

**End of Todo List**
