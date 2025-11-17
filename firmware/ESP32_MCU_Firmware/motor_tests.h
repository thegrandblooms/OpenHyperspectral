#ifndef MOTOR_TESTS_H
#define MOTOR_TESTS_H

#include <Arduino.h>
#include "motor_control.h"
#include "config.h"

//=============================================================================
// MOTOR TEST FUNCTIONS
//=============================================================================
// This file contains all motor testing and diagnostic functions.
// These are invoked from the serial monitor using interactive commands.
//
// Available tests:
// - runEncoderTest() - Test encoder readings in real-time
// - runAlignmentTest() - Test motor alignment at known electrical angles (DIAGNOSTIC)
// - runMotorTest() - Run basic motor movement sequence
// - runFullTest() - Full system test (calibration + PID tuning + movement)
//=============================================================================

// Forward declarations
extern MotorController motorControl;
extern bool g_debug_serial;
extern bool g_debug_motor;

//=============================================================================
// ENCODER TEST
//=============================================================================
/**
 * Test encoder readings by displaying real-time position data.
 *
 * This test reads the MT6701 absolute magnetic encoder directly and displays:
 * - Raw encoder count (0-16383)
 * - Encoder position in degrees (ABSOLUTE ENCODER - direct I2C read)
 * - SimpleFOC shaft angle for comparison
 *
 * Press any key to stop the test.
 */
void runEncoderTest() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    Encoder Test                                ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("\nThis test reads the encoder directly and displays position data.");
    Serial.println("Manually rotate the motor shaft to see encoder response.");
    Serial.println("Press any key to stop the test.\n");
    Serial.println("Format: [TIME] Raw: XXXXX | Encoder: XXX.XX° | SimpleFOC: XXX.XX°");
    Serial.println("─────────────────────────────────────────────────────────────────");

    unsigned long start_time = millis();
    unsigned long last_sample = 0;
    const unsigned long sample_interval = 100;  // 10 Hz

    // Clear serial buffer
    while (Serial.available() > 0) {
        Serial.read();
    }

    while (true) {
        // Check for key press to exit
        if (Serial.available() > 0) {
            Serial.read();
            Serial.println("\n\nEncoder test stopped by user.");
            break;
        }

        // Sample at 10 Hz
        unsigned long current_time = millis();
        if (current_time - last_sample >= sample_interval) {
            last_sample = current_time;

            // CRITICAL: Force fresh encoder read from MT6701 via I2C
            // This reads the ABSOLUTE ENCODER position directly
            motorControl.updateEncoder();

            uint16_t raw_count = motorControl.getRawEncoderCount();
            float encoder_angle = motorControl.getEncoderDegrees();         // ABSOLUTE ENCODER
            float simplefoc_angle = motorControl.getCurrentPositionDeg();   // SimpleFOC internal state
            float time_sec = (current_time - start_time) / 1000.0;

            Serial.print("[");
            Serial.print(time_sec, 2);
            Serial.print("s] Raw: ");
            Serial.print(raw_count);
            Serial.print(" | Encoder: ");
            Serial.print(encoder_angle, 2);
            Serial.print("° | SimpleFOC: ");
            Serial.print(simplefoc_angle, 2);
            Serial.println("°");
        }

        delay(1);
    }

    Serial.println();
}

//=============================================================================
// MOTOR ALIGNMENT DIAGNOSTIC TEST
//=============================================================================
/**
 * Test motor alignment by applying voltages at known electrical angles.
 *
 * This diagnostic test verifies that:
 * 1. Motor responds to setPhaseVoltage() commands
 * 2. Motor settles into stable positions (not oscillating)
 * 3. Motor holds positions firmly (strong holding torque)
 * 4. Encoder readings are consistent
 *
 * This should be run BEFORE manual calibration to verify hardware is working.
 *
 * Expected behavior:
 * - Motor moves quickly when voltage changes
 * - Motor settles into stable position within 500ms
 * - Motor holds position firmly (try to rotate by hand - should resist)
 * - No continuous oscillation
 *
 * If motor oscillates continuously, check:
 * - Driver fault state
 * - Power supply voltage
 * - Common ground connection
 * - Pole pairs setting (should be 7 for Mitoot 2804)
 */
void runAlignmentTest() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║              Motor Alignment Diagnostic Test                   ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("\nThis test applies voltage at known electrical angles.");
    Serial.println("Motor should move quickly then HOLD each position FIRMLY.");
    Serial.println("\nExpected behavior:");
    Serial.println("  • Motor moves and settles within 500ms");
    Serial.println("  • Strong holding torque (resists manual rotation)");
    Serial.println("  • NO continuous oscillation");
    Serial.println("\nIf motor oscillates, check hardware connections and settings.");
    Serial.println("─────────────────────────────────────────────────────────────────\n");

    // Run the test function from MotorController
    bool success = motorControl.testMotorAlignment();

    if (success) {
        Serial.println("✓ Alignment test complete!");
        Serial.println("\nIf motor held positions firmly, hardware is working correctly.");
        Serial.println("You can now run calibration (type 'c' or 'calibrate').");
    } else {
        Serial.println("✗ Alignment test failed!");
        Serial.println("\nCheck encoder connection and try again.");
    }

    Serial.println();
}

//=============================================================================
// MOTOR MOVEMENT TEST
//=============================================================================
/**
 * Run a basic motor movement test sequence.
 *
 * This test:
 * 1. Auto-enables the motor
 * 2. Sets home position
 * 3. Moves to 90°, 180°, and back to 0°
 * 4. Prints status after each movement
 *
 * Uses ABSOLUTE ENCODER for position verification.
 */
void runMotorTest() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    Motor Test Sequence                         ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");

    // Auto-enable motor for this test
    if (!motorControl.isEnabled()) {
        Serial.print("\nEnabling motor... ");
        motorControl.enable();
        if (motorControl.isEnabled()) {
            Serial.println("Motor enabled");
        } else {
            Serial.println("FAILED - motor not calibrated!");
            Serial.println("Please run calibration first (type 'c' or 'calibrate')");
            return;
        }
    } else {
        Serial.println("\nMotor already enabled");
    }

    delay(500);

    Serial.print("Setting home position... ");
    motorControl.setHome();

    // Use ABSOLUTE ENCODER to verify home position
    motorControl.updateEncoder();
    float encoder_pos = motorControl.getEncoderDegrees();  // ABSOLUTE ENCODER
    Serial.print("Home set at encoder angle: ");
    Serial.print(encoder_pos, 2);
    Serial.println("°");
    delay(500);

    // Test movement #1: 90°
    Serial.print("Moving to 90°... ");
    motorControl.moveToPosition(90.0);
    Serial.print("Target: ");
    Serial.print(motorControl.getTargetPositionDeg(), 2);
    Serial.println("°");
    delay(3000);

    // Verify with ABSOLUTE ENCODER
    motorControl.updateEncoder();
    encoder_pos = motorControl.getEncoderDegrees();  // ABSOLUTE ENCODER
    Serial.print("Encoder position: ");
    Serial.print(encoder_pos, 2);
    Serial.println("°");

    // Test movement #2: 180°
    Serial.print("Moving to 180°... ");
    motorControl.moveToPosition(180.0);
    Serial.print("Target: ");
    Serial.print(motorControl.getTargetPositionDeg(), 2);
    Serial.println("°");
    delay(3000);

    // Verify with ABSOLUTE ENCODER
    motorControl.updateEncoder();
    encoder_pos = motorControl.getEncoderDegrees();  // ABSOLUTE ENCODER
    Serial.print("Encoder position: ");
    Serial.print(encoder_pos, 2);
    Serial.println("°");

    // Test movement #3: Return to home
    Serial.print("Moving back to home (0°)... ");
    motorControl.moveToPosition(0.0);
    Serial.print("Target: ");
    Serial.print(motorControl.getTargetPositionDeg(), 2);
    Serial.println("°");
    delay(3000);

    // Verify with ABSOLUTE ENCODER
    motorControl.updateEncoder();
    encoder_pos = motorControl.getEncoderDegrees();  // ABSOLUTE ENCODER
    Serial.print("Encoder position: ");
    Serial.print(encoder_pos, 2);
    Serial.println("°");

    Serial.println("\n✓ Motor test sequence complete!\n");
}

//=============================================================================
// FULL SYSTEM TEST
//=============================================================================
/**
 * Run complete system test: calibration + PID tuning + movement test.
 *
 * This comprehensive test:
 * 1. Runs motor calibration (if not already calibrated)
 * 2. Runs PID auto-tuning to find optimal parameters
 * 3. Runs motor movement test to verify operation
 *
 * All position verification uses ABSOLUTE ENCODER.
 */
void runFullTest() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    Full System Test                            ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");

    // Step 1: Calibration
    Serial.println("\n=== Step 1: Motor Calibration ===");
    if (motorControl.isCalibrated()) {
        Serial.println("Motor already calibrated - skipping calibration");
    } else {
        Serial.println("Running motor calibration...");
        if (motorControl.calibrate()) {
            Serial.println("✓ Calibration successful!");
        } else {
            Serial.println("✗ Calibration failed!");
            Serial.println("Test sequence aborted.");
            return;
        }
    }

    delay(1000);

    // Step 2: PID Auto-Tuning
    Serial.println("\n=== Step 2: PID Auto-Tuning ===");
    Serial.println("This will test motor response and find optimal PID values.");
    Serial.println("Uses ABSOLUTE ENCODER for position tracking (not SimpleFOC shaft_angle).");
    Serial.println("This may take 2-5 minutes depending on motor response...");
    Serial.println();

    if (motorControl.autoTunePID(true)) {
        Serial.println("✓ PID auto-tuning successful!");
        Serial.println("Optimal PID parameters have been applied.");
    } else {
        Serial.println("✗ PID auto-tuning failed!");
        Serial.println("Continuing with current PID values...");
    }

    delay(1000);

    // Step 3: Motor test
    Serial.println("\n=== Step 3: Motor Movement Test ===");
    runMotorTest();

    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    Full Test Complete!                        ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝\n");
}

#endif // MOTOR_TESTS_H
