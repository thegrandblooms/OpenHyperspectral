/**
 * Test functions for the OpenHyperspectral stepper motor controller.
 * Serial-interactive diagnostics, mirroring the T1-T5 structure of the
 * gimbal firmware so tools/motor_test.py result parsing keeps working.
 */

#ifndef TESTS_H
#define TESTS_H

#include <Arduino.h>
#include "motor_control.h"

//=============================================================================
// INFORMATION & STATUS DISPLAY
//=============================================================================
void printHelp();
void printSystemInfo();
void printStatus(MotorController& motorControl);

//=============================================================================
// MAIN DIAGNOSTIC
//=============================================================================
// runSystemDiagnostic: full validation sequence
//   T1: Hardware    — MT6835 SPI/CRC/field, TMC2209 UART
//   T2: Calibration — encoder verify + stored backlash
//   T3: Encoder     — read stability / noise floor (motor holding)
//   T4: Movement    — closed-loop +30 deg move, encoder-verified
//   T5: Accuracy    — 360 deg sweep, 24 positions, trim-verified accuracy
// Prints "RESULT: ALL TESTS PASSED" / "RESULT: PARTIAL" / "RESULT: FAILED"
// (same markers motor_test.py keys on).
void runSystemDiagnostic(MotorController& motorControl);

//=============================================================================
// INDIVIDUAL TESTS
//=============================================================================
// Interactive: rotate the output shaft by hand, watch encoder values
void runEncoderTest(MotorController& motorControl);

// Quick +30 deg move sanity check
void runMotorTest(MotorController& motorControl);

// 5-position accuracy test within +/-30 deg of the current position
void runPositionSweepTest(MotorController& motorControl);

// Helper: run update() until the move settles (or timeout); returns final error (deg)
float waitForMove(MotorController& motorControl, float target_deg, unsigned long timeout_ms);

#endif // TESTS_H
