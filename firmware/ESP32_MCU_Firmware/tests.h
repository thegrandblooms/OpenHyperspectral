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
// MOTOR TESTS
//=============================================================================
void runEncoderTest(MotorController& motorControl);
void runPhaseTest(MotorController& motorControl);     // Test each driver phase individually
void runAlignmentTest(MotorController& motorControl);  // Diagnostic test BEFORE calibration
void runMotorTest(MotorController& motorControl);
void runFullTest(MotorController& motorControl);

#endif // TESTS_H
