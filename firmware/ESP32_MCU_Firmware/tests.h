/**
 * Test Functions for OpenHyperspectral Motor Controller
 *
 * This file contains all test, diagnostic, and utility functions for
 * interactive testing via the serial monitor.
 */

#ifndef TESTS_H
#define TESTS_H

#include <Arduino.h>
#include "motor_control.h"

//=============================================================================
// I2C DIAGNOSTICS
//=============================================================================
void scanI2C();

//=============================================================================
// INFORMATION & STATUS DISPLAY
//=============================================================================
void printHelp();
void printSystemInfo();
void printStatus(MotorController& motorControl);

//=============================================================================
// MAIN DIAGNOSTIC (combined calibration + tests)
//=============================================================================
// runSystemDiagnostic: Full system validation sequence
//   T1: Hardware - I2C bus, encoder response, magnetic field strength
//   T2: Calibration - SimpleFOC initFOC (sensor-motor electrical alignment)
//   T3: Sensor Integration - Verify loopFOC() reads encoder correctly
//   T4: Open-Loop - Driver/wiring/power check (bypasses closed-loop control)
//   Field Sweep: 180° encoder uniformity check (raw noise + oscillation per angle)
//   T5: Position Control - Closed-loop +30° move with error measurement
void runSystemDiagnostic(MotorController& motorControl);

//=============================================================================
// INDIVIDUAL TESTS (for specific debugging)
//=============================================================================
// runEncoderTest: Interactive - rotate motor by hand, watch encoder values
//   Shows: raw count, encoder degrees, SimpleFOC shaft_angle
//   Use for: Verifying encoder tracks correctly, checking for noise/dropout
void runEncoderTest(MotorController& motorControl);

// runPhaseTest: Tests all 3 driver phases at 6 electrical angles
//   Applies voltage at 0°, 60°, 90°, 120°, 180°, 240° electrical
//   Use for: Diagnosing driver faults, phase wiring issues, dead phases
void runPhaseTest(MotorController& motorControl);

// runAlignmentTest: Tests motor magnetic holding at 4 angles
//   Applies voltage at 0°, 90°, 180°, 270° - user rotates by hand
//   Use for: Checking motor can hold position, detecting weak magnets
void runAlignmentTest(MotorController& motorControl);

// runMotorTest: Quick +30° movement sanity check
//   Enable → verify tracking → move +30° → check error
//   Use for: Fast verification after changes
void runMotorTest(MotorController& motorControl);

// runPositionSweepTest: 5-position accuracy test within ±30°
//   Tests: 0°, -15°, -30°, +15°, +30° offsets from start
//   Use for: PID tuning validation, checking repeatability
void runPositionSweepTest(MotorController& motorControl);

// logMotorState: Diagnostic helper - logs current motor state
void logMotorState(MotorController& motorControl, const char* context);

#endif // TESTS_H
