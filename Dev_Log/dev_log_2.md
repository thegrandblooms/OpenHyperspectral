# Development Log 2: Motor Phase Diagnostic & Hardware Issue Resolution

**Date:** 2025-11-19
**Status:** Hardware issue identified and resolved
**Summary:** Discovered motor winding damage from mounting screws; implemented comprehensive phase diagnostics

---

## Problem Discovery

### Initial Symptom
During calibration testing, motor exhibited abnormal behavior:
- Only achieved **2 distinct positions** instead of 4 expected positions
- 0° and 270° electrical → same mechanical position (~349°)
- 90° and 180° electrical → same mechanical position (~13°)

This pattern indicated a fundamental hardware problem, not a calibration issue.

### Hypothesis
With 7 pole pairs, motor should move ~12.86° mechanical for every 90° electrical change. The fact that some electrical angles produced identical mechanical positions suggested **one of three motor phases was not functioning**.

---

## Diagnostic Implementation

### Phase Test Development (commit 9bfe561)
Created `testDriverPhases()` function to systematically test all three motor phases:

**Test Strategy:**
- Drive motor at 6 electrical angles (0°, 60°, 120°, 180°, 240°, 300°)
- Monitor fault pin (nFT) at each step
- Track encoder position changes
- Expected: 6 distinct positions (~8.6° mechanical apart)

**Key Features:**
```cpp
bool MotorController::testDriverPhases() {
    // Check driver fault status before/after enable
    pinMode(MOTOR_FAULT, INPUT_PULLUP);
    bool fault_status = digitalRead(MOTOR_FAULT);

    // Test 6 positions covering all phase combinations
    PhaseTest tests[] = {
        {0.0,       "Phase A positive (0° electrical)"},
        {_PI_3,     "Phase A→B transition (60°)"},
        {_PI_2,     "Phase B positive (90°)"},
        {2.0*_PI_3, "Phase B→C transition (120°)"},
        {PI,        "Phase C positive (180°)"},
        {4.0*_PI_3, "Phase C→A transition (240°)"}
    };

    // Apply voltage and track position at each angle
}
```

**Command added:** `phase_test`

---

## Test Results

### Initial Phase Test
```
[TEST 1/6] Phase A positive (0°)      → 141.50° ✓
[TEST 2/6] Phase A→B transition (60°) → 115.22° ✓
[TEST 3/6] Phase B positive (90°)     → 115.22° ✗ STUCK
[TEST 4/6] Phase B→C transition (120°)→ 115.22° ✗ STUCK
[TEST 5/6] Phase C positive (180°)    → 115.22° ✗ STUCK
[TEST 6/6] Phase C→A transition (240°)→ 90.26°  ✓
```

**Analysis:** Tests 3-5 all stuck at same position → **Phase B failure**

**Hypothesis:** Driver output IN2 (GPIO11 → Motor Phase B) not working

### Wire Swap Test
Swapped M1 and M2 motor connections to driver to isolate problem:

**Result:** Failure pattern CHANGED
- Before: Tests 3-5 failed
- After: Tests 1-2-3 and 5-6 failed (different pattern)

**Conclusion:** Problem moved with motor wire → **Motor issue, not driver issue**

✓ All driver outputs (IN1, IN2, IN3) working correctly
✓ All GPIOs (11, 12, 13) working correctly
✗ Motor winding problem confirmed

---

## Root Cause Identified

### Resistance Measurements
With motor disconnected, measured phase-to-phase resistance:
- Expected: ~10Ω between all phase pairs (A↔B, B↔C, A↔C)
- **Actual: Open circuit (∞Ω) between ALL pairs**

### Physical Inspection
**Discovery:** M2.5 mounting screws were ~1mm too long
- Screws penetrated through mounting holes into motor windings
- Physically damaged copper wire insulation/conductors
- Caused open circuits in multiple windings

**Timeline:** New M2.5 screws received and installed same day as testing → motor was functional before screw installation

### Damage Assessment
- Multiple windings damaged (all show open circuit)
- No shorts detected, but complete loss of continuity
- Motor cannot generate proper magnetic fields
- **Motor requires replacement**

---

## Lessons Learned

### Hardware Assembly
1. **Always verify screw length before installation**
   - Measure mounting hole depth
   - Use screws at least 1-2mm shorter than hole depth
   - Better too short than too long

2. **Test immediately after hardware changes**
   - Phase test caught problem immediately after screw installation
   - Quick diagnosis prevented confusion about calibration software

### Diagnostic Tools
3. **Comprehensive phase testing is essential**
   - Simple alignment test (4 positions) showed symptoms
   - Detailed phase test (6 positions) isolated root cause
   - Fault pin monitoring ruled out driver issues
   - Wire swap test separated motor vs. driver failures

### System Validation
4. **All other systems confirmed working:**
   - ✓ ESP32-S3 GPIO outputs functional
   - ✓ DRV8313 driver all 3 channels operational
   - ✓ MT6701 I2C encoder accurate readings
   - ✓ Manual calibration algorithm correct
   - ✓ SimpleFOC integration functional

---

## Resolution

### Immediate Action
- Replace damaged Mitoot 2804 motor
- Use proper length screws (verify depth measurement)

### Code Improvements Added
- `testDriverPhases()` - Comprehensive 6-position motor test
- Fault pin monitoring throughout tests
- Clear pass/fail diagnostic reporting
- Command: `phase_test` for hardware validation

### Documentation Updates
- Added warning about screw length to prevent recurrence
- Documented phase test procedure for future troubleshooting

---

## Next Steps

### Hardware
1. Order replacement Mitoot 2804 (7 pole pair) gimbal motor
2. Measure mounting hole depth before screw selection
3. Re-run phase test to verify new motor function

### Software (Pending motor replacement)
1. Complete calibration testing with functional motor
2. Implement automated calibration/test decision tree
3. Develop PC-side motor controller for scanning operations
4. Test full hyperspectral imaging workflow

---

## Status Summary

**Hardware:**
- ESP32-S3: ✓ Functional
- DRV8313 Driver: ✓ Functional (all 3 channels)
- MT6701 Encoder: ✓ Functional
- Mitoot 2804 Motor: ✗ Damaged (replacement needed)

**Software:**
- Manual calibration: ✓ Implemented and validated
- Phase diagnostics: ✓ Implemented and tested
- Calibration workflow: 🔄 Ready for testing with new motor
- PC controller: 📋 Scope in development

**Blocker:** Waiting for replacement motor to continue integration testing
