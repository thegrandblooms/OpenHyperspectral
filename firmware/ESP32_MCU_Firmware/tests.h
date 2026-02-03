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
void runSystemDiagnostic(MotorController& motorControl);  // T1-T5: Hardware, Cal, Sensor, Open-loop, Position

//=============================================================================
// INDIVIDUAL TESTS (for specific debugging)
//=============================================================================
void runEncoderTest(MotorController& motorControl);      // Interactive encoder reading
void runPhaseTest(MotorController& motorControl);        // Test 6 electrical phase angles
void runAlignmentTest(MotorController& motorControl);    // Test motor holding at 4 angles
void runMotorTest(MotorController& motorControl);        // Quick +30° movement test
void runPositionSweepTest(MotorController& motorControl); // 5-position accuracy test

// Diagnostic helper
void logMotorState(MotorController& motorControl, const char* context);

#endif // TESTS_H
