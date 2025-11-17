/**
 * Test Functions for OpenHyperspectral Motor Controller
 *
 * This file contains all test, diagnostic, and utility functions for
 * interactive testing via the serial monitor.
 */

#include "tests.h"
#include "config.h"
#include "commands.h"
#include <Wire.h>

//=============================================================================
// I2C SCANNER UTILITY
//=============================================================================

void scanI2C() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                      I2C Scanner                               ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.print("Scanning I2C bus (SDA=GPIO");
    Serial.print(ENCODER_SDA);
    Serial.print(", SCL=GPIO");
    Serial.print(ENCODER_SCL);
    Serial.println(")...\n");

    byte count = 0;
    for (byte i = 1; i < 127; i++) {
        Wire.beginTransmission(i);
        byte error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("Found I2C device at 0x");
            if (i < 16) Serial.print("0");
            Serial.print(i, HEX);

            // Identify known devices
            if (i == 0x06) {
                Serial.print(" (MT6701 encoder - DETECTED!)");
            } else if (i == 0x15) {
                Serial.print(" (CST816D touch controller)");
            } else if (i == 0x6B) {
                Serial.print(" (QMI8658 IMU)");
            }
            Serial.println();
            count++;
        }
    }

    if (count == 0) {
        Serial.println("⚠ WARNING: No I2C devices found!");
        Serial.println("Check wiring:");
        Serial.print("  - 3V3 → MT6701 VDD\n");
        Serial.print("  - GND → MT6701 GND\n");
        Serial.print("  - GPIO47 → MT6701 SDA\n");
        Serial.print("  - GPIO48 → MT6701 SCL\n");
    } else {
        Serial.print("\nTotal devices found: ");
        Serial.println(count);
    }
    Serial.println();
}

//=============================================================================
// INFORMATION & STATUS DISPLAY
//=============================================================================

void printHelp() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║          OpenHyperspectral Motor Controller - Help            ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("\nInteractive Test Commands (type and press Enter):");
    Serial.println("  h, help        - Show this help menu");
    Serial.println("  s, status      - Print current motor status");
    Serial.println("  i, info        - Show system information");
    Serial.println("  scan           - Scan I2C bus for encoder (MT6701)");
    Serial.println("");
    Serial.println("Motor Control:");
    Serial.println("  e, enable      - Enable motor");
    Serial.println("  d, disable     - Disable motor");
    Serial.println("  c, calibrate   - Run motor calibration");
    Serial.println("  p, pidtune     - Run PID auto-tuning (after calibration)");
    Serial.println("  home           - Set current position as home");
    Serial.println("  stop           - Stop motor movement");
    Serial.println("  m <angle>      - Move to angle (e.g., 'm 90' for 90 degrees)");
    Serial.println("  v <velocity>   - Set velocity (e.g., 'v 100.0' deg/s)");
    Serial.println("  a <accel>      - Set acceleration (e.g., 'a 50.0' deg/s²)");
    Serial.println("  mode <0-2>     - Set control mode (0=position, 1=velocity, 2=torque)");
    Serial.println("");
    Serial.println("Testing:");
    Serial.println("  phase_test     - Test each driver phase output (HARDWARE DIAGNOSTIC)");
    Serial.println("  test           - Run full test (calibration + PID tuning + motor test)");
    Serial.println("  motor_test     - Run motor movement test (auto-enables motor)");
    Serial.println("  encoder_test   - Test encoder readings (press any key to stop)");
    Serial.println("");
    Serial.println("Debug:");
    Serial.println("  debug <0/1>    - Toggle debug verbosity (0=quiet, 1=verbose)");
    Serial.println("");
    Serial.println("\nBinary Protocol Commands:");
    Serial.println("  The controller also accepts binary commands via SerialTransfer");
    Serial.println("  Use the Python/PC software for full protocol communication");
    Serial.println("\n");
}

void printSystemInfo() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    System Information                          ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.print("Firmware Version: 1.0.0\n");
    Serial.print("Board: ESP32-S3-Touch-LCD-2 (Waveshare)\n");
    Serial.print("Chip Model: ");
    Serial.println(ESP.getChipModel());
    Serial.print("CPU Frequency: ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(" MHz");
    Serial.print("Flash Size: ");
    Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
    Serial.println(" MB");
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(" KB");
    Serial.print("Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");

    Serial.println("\n--- Motor Configuration ---");
    Serial.print("Pole Pairs: ");
    Serial.println(POLE_PAIRS);
    Serial.print("Encoder PPR: ");
    Serial.println(ENCODER_PPR);
    Serial.print("Power Supply: ");
    Serial.print(VOLTAGE_PSU);
    Serial.println(" V");
    Serial.print("Current Limit: ");
    Serial.print(CURRENT_LIMIT);
    Serial.println(" A");
    Serial.print("Max Velocity: ");
    Serial.print(MAX_VELOCITY);
    Serial.println(" rad/s");

    Serial.println("\n--- Pin Configuration ---");
    Serial.print("Motor Driver (SimpleFOC Mini):\n");
    Serial.print("  EN=GPIO");
    Serial.print(MOTOR_ENABLE);
    Serial.print(", IN1=GPIO");
    Serial.print(MOTOR_PWM_A);
    Serial.print(", IN2=GPIO");
    Serial.print(MOTOR_PWM_B);
    Serial.print(", IN3=GPIO");
    Serial.println(MOTOR_PWM_C);
    Serial.print("  nFT=GPIO");
    Serial.print(MOTOR_FAULT);
    Serial.print(", nRT=GPIO");
    Serial.println(MOTOR_RESET);
    Serial.print("Encoder (MT6701 I2C):\n");
    Serial.print("  SDA=GPIO");
    Serial.print(ENCODER_SDA);
    Serial.print(", SCL=GPIO");
    Serial.print(ENCODER_SCL);
    Serial.print(", Addr=0x");
    Serial.println(ENCODER_I2C_ADDR, HEX);
    Serial.println();
}

void printStatus(MotorController& motorControl) {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    MOTOR STATUS                                ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");

    // Force fresh encoder read
    motorControl.updateEncoder();

    // MT6701 ENCODER (Source of truth for position)
    Serial.println("\n[MT6701 ENCODER] (Absolute position - source of truth)");
    Serial.print("  Raw Count:  ");
    Serial.print(motorControl.getRawEncoderCount());
    Serial.println(" (0-16383)");
    Serial.print("  Position:   ");
    Serial.print(motorControl.getEncoderDegrees(), 2);
    Serial.println("°");

    // SimpleFOC STATE (Motor controller internal - may not track correctly)
    Serial.println("\n[SimpleFOC STATE] (Controller internal - for reference only)");
    Serial.print("  Shaft Angle: ");
    Serial.print(motorControl.getCurrentPositionDeg(), 2);
    Serial.println("° (may be 0° if not tracking)");
    Serial.print("  Velocity:    ");
    Serial.print(motorControl.getCurrentVelocityDegPerSec(), 2);
    Serial.println("°/s");
    Serial.print("  Current:     ");
    Serial.print(motorControl.getCurrent(), 3);
    Serial.println(" A");
    Serial.print("  Voltage:     ");
    Serial.print(motorControl.getVoltage(), 2);
    Serial.println(" V");

    // SYSTEM STATE
    Serial.println("\n[SYSTEM STATE]");
    Serial.print("  State:       ");
    switch (motorControl.getState()) {
        case STATE_IDLE: Serial.println("IDLE"); break;
        case STATE_MOVING: Serial.println("MOVING"); break;
        case STATE_ERROR: Serial.println("ERROR"); break;
        case STATE_CALIBRATING: Serial.println("CALIBRATING"); break;
        default: Serial.println("UNKNOWN");
    }
    Serial.print("  Control Mode: ");
    switch (motorControl.getControlMode()) {
        case MODE_POSITION: Serial.println("POSITION"); break;
        case MODE_VELOCITY: Serial.println("VELOCITY"); break;
        case MODE_TORQUE: Serial.println("TORQUE"); break;
        default: Serial.println("UNKNOWN");
    }
    Serial.print("  Enabled:     ");
    Serial.println(motorControl.isEnabled() ? "YES" : "NO");
    Serial.print("  Calibrated:  ");
    Serial.println(motorControl.isCalibrated() ? "YES" : "NO");
    Serial.print("  At Target:   ");
    Serial.println(motorControl.isAtTarget() ? "YES" : "NO");
    Serial.println();
}

//=============================================================================
// MOTOR TESTS
//=============================================================================

void runEncoderTest(MotorController& motorControl) {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    Encoder Test                                ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("\nThis test reads the MT6701 encoder directly and displays position.");
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

            // Force fresh encoder read from I2C (MT6701 absolute encoder)
            motorControl.updateEncoder();

            uint16_t raw_count = motorControl.getRawEncoderCount();
            float encoder_angle = motorControl.getEncoderDegrees();
            float simplefoc_angle = motorControl.getCurrentPositionDeg();
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

void runPhaseTest(MotorController& motorControl) {
    // Run the phase test from MotorController
    motorControl.testDriverPhases();
}

void runAlignmentTest(MotorController& motorControl) {
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

void runMotorTest(MotorController& motorControl) {
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

    // Get encoder position (source of truth)
    motorControl.updateEncoder();
    Serial.print("Home set at angle: ");
    Serial.print(motorControl.getEncoderDegrees(), 2);
    Serial.println("°");
    delay(500);

    Serial.print("Moving to 90°... ");
    motorControl.moveToPosition(90.0);
    Serial.print("Target: ");
    Serial.print(motorControl.getTargetPositionDeg(), 2);
    Serial.println("°");
    delay(3000);
    printStatus(motorControl);

    Serial.print("Moving to 180°... ");
    motorControl.moveToPosition(180.0);
    Serial.print("Target: ");
    Serial.print(motorControl.getTargetPositionDeg(), 2);
    Serial.println("°");
    delay(3000);
    printStatus(motorControl);

    Serial.print("Moving back to home (0°)... ");
    motorControl.moveToPosition(0.0);
    Serial.print("Target: ");
    Serial.print(motorControl.getTargetPositionDeg(), 2);
    Serial.println("°");
    delay(3000);

    Serial.println("\nMotor test sequence complete!\n");
    printStatus(motorControl);
}

void runFullTest(MotorController& motorControl) {
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
    runMotorTest(motorControl);

    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    Full Test Complete!                        ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝\n");
}
