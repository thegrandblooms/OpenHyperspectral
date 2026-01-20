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
// DIAGNOSTIC HELPERS
//=============================================================================

/**
 * Log critical motor control state for debugging (COMPACT FORMAT)
 * Call this to understand why motor isn't moving or tracking correctly
 */
void logMotorState(MotorController& motorControl, const char* context) {
    // DO NOT call updateEncoder() during movement - it races with loopFOC()!
    // SimpleFOC's loopFOC() already updates the sensor automatically

    // Get all critical values
    float encoder_abs_deg = motorControl.getEncoderDegrees();
    BLDCMotor& motor = motorControl.getMotor();
    float shaft_angle_abs_deg = radiansToDegrees(motor.shaft_angle);
    float target_deg = motorControl.getTargetPositionDeg();
    float position_error_rad = motor.shaft_angle_sp - motor.shaft_angle;
    float tracking_error = abs(encoder_abs_deg - shaft_angle_abs_deg);

    // Line 1: Context + Position data
    Serial.print("[DIAG] ");
    Serial.print(context);
    Serial.print(" | Enc:");
    Serial.print(encoder_abs_deg, 1);
    Serial.print("° FOC:");
    Serial.print(shaft_angle_abs_deg, 1);
    Serial.print("° Tgt:");
    Serial.print(target_deg, 1);
    Serial.print("° | En:");
    Serial.print(motorControl.isEnabled() ? "Y" : "N");
    Serial.print(" Cal:");
    Serial.print(motorControl.isCalibrated() ? "Y" : "N");
    Serial.print(" | TrkErr:");
    Serial.print(tracking_error, 2);
    Serial.println("°");

    // Line 2: SimpleFOC PID state
    Serial.print("       PIDerr:");
    Serial.print(radiansToDegrees(position_error_rad), 1);
    Serial.print("° VelCmd:");
    Serial.print(radiansToDegrees(motor.shaft_velocity_sp), 1);
    Serial.print("°/s VelAct:");
    Serial.print(radiansToDegrees(motor.shaft_velocity), 1);
    Serial.print("°/s | Vq:");
    Serial.print(motor.voltage.q, 2);
    Serial.print("V Vd:");
    Serial.print(motor.voltage.d, 2);
    Serial.println("V");
}

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
    Serial.println("  home           - Log current position (absolute encoders don't need homing)");
    Serial.println("  stop           - Stop motor movement");
    Serial.println("  m <angle>      - Move to absolute angle (e.g., 'm 90' for 90 degrees absolute)");
    Serial.println("  v <velocity>   - Set velocity (e.g., 'v 100.0' deg/s)");
    Serial.println("  a <accel>      - Set acceleration (e.g., 'a 50.0' deg/s²)");
    Serial.println("  mode <0-2>     - Set control mode (0=position, 1=velocity, 2=torque)");
    Serial.println("");
    Serial.println("Testing:");
    Serial.println("  phase_test     - Test each driver phase output (HARDWARE DIAGNOSTIC)");
    Serial.println("  test           - Run full test (calibration + motor test)");
    Serial.println("  motor_test     - Diagnostic test with detailed logging (USE THIS FIRST)");
    Serial.println("  position_sweep - Test 5 precise positions (±30° safe range)");
    Serial.println("  encoder_test   - Test encoder readings (press any key to stop)");
    Serial.println("  diag           - *** SimpleFOC root cause analysis (frozen shaft_angle) ***");
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

    // Step 1: Enable motor
    if (!motorControl.isEnabled()) {
        Serial.print("[1/3] ");
        motorControl.enable();
        delay(100);
        if (!motorControl.isEnabled()) {
            Serial.println("✗ Motor won't enable (not calibrated?)");
            return;
        }
    } else {
        Serial.println("[1/3] Motor already enabled");
    }

    // Step 2: Verify SimpleFOC tracking
    motorControl.updateEncoder();
    BLDCMotor& motor = motorControl.getMotor();
    float shaft_angle_abs_deg = radiansToDegrees(motor.shaft_angle);
    float encoder_abs_deg = motorControl.getEncoderDegrees();
    float tracking_error_abs = abs(encoder_abs_deg - shaft_angle_abs_deg);

    if (tracking_error_abs > 5.0) {
        Serial.print("[2/3] ✗ Tracking error ");
        Serial.print(tracking_error_abs, 1);
        Serial.println("° - SimpleFOC not reading sensor!");
        return;
    }
    Serial.print("[2/3] ✓ Tracking (Err:");
    Serial.print(tracking_error_abs, 1);
    Serial.println("°)");

    // Step 3: Test movement (+30° from current position)
    float start_position = motorControl.getEncoderDegrees();
    float test_target = start_position + 30.0;
    if (test_target >= 360.0) test_target -= 360.0;

    Serial.print("[3/3] Move +30°: ");
    Serial.print(start_position, 1);
    Serial.print("° → ");
    Serial.print(test_target, 1);
    Serial.println("°");

    motorControl.moveToPosition(test_target);

    // Run control loop with compact trajectory logging
    unsigned long start_time = millis();
    const unsigned long timeout_ms = 3000;
    int loop_count = 0;

    Serial.println("--- Trajectory ---");
    while (millis() - start_time < timeout_ms) {
        motorControl.update();
        delay(10);
        loop_count++;

        // Ultra-compact trajectory log every 10 loops (100ms) - with FOC diagnostics
        if (loop_count % 10 == 0) {
            unsigned long elapsed = millis() - start_time;
            float pos_error = motor.shaft_angle_sp - motor.shaft_angle;

            // Read fault pin
            pinMode(MOTOR_FAULT, INPUT_PULLUP);
            bool fault_ok = digitalRead(MOTOR_FAULT);

            // Calculate electrical angle (what SimpleFOC uses for commutation)
            float elec_angle_deg = radiansToDegrees(motor.shaft_angle * POLE_PAIRS + motor.zero_electric_angle);
            // Normalize to 0-360
            while (elec_angle_deg < 0) elec_angle_deg += 360.0;
            while (elec_angle_deg >= 360.0) elec_angle_deg -= 360.0;

            Serial.print(elapsed);
            Serial.print("ms FOC:");
            Serial.print(radiansToDegrees(motor.shaft_angle), 1);
            Serial.print("° Vel:");
            Serial.print(radiansToDegrees(motor.shaft_velocity), 0);
            Serial.print("°/s E:");
            Serial.print(radiansToDegrees(pos_error), 1);
            Serial.print("° | Elec:");
            Serial.print(elec_angle_deg, 0);
            Serial.print("° Vq:");
            Serial.print(motor.voltage.q, 1);
            Serial.print("V ");
            Serial.print(fault_ok ? "Ft:OK" : "Ft:FAULT");
            Serial.print(motor.enabled ? " En:Y" : " En:N");
            Serial.println();
        }

        // Detailed [DIAG] logging removed - trajectory already shows progress

        if (motorControl.isAtTarget()) {
            Serial.println("✓ Reached target");
            break;
        }
    }

    // Check results
    motorControl.updateEncoder();
    float final_position = motorControl.getEncoderDegrees();
    float movement = final_position - start_position;
    if (movement < -180.0) movement += 360.0;
    if (movement > 180.0) movement -= 360.0;
    movement = abs(movement);

    float error = final_position - test_target;
    if (error < -180.0) error += 360.0;
    if (error > 180.0) error -= 360.0;
    error = abs(error);

    Serial.print("\nResults: Start:");
    Serial.print(start_position, 1);
    Serial.print("° Final:");
    Serial.print(final_position, 1);
    Serial.print("° | Moved:");
    Serial.print(movement, 1);
    Serial.print("° Err:");
    Serial.print(error, 1);
    Serial.println("°");

    // FAILURE DETECTION
    if (movement < 5.0) {
        Serial.print("✗ FAILED: Motor barely moved (");
        Serial.print(movement, 1);
        Serial.println("°) - Check power/driver/PID gains");
        logMotorState(motorControl, "Final (no movement)");
        return;
    }

    if (abs(movement - 30.0) > 10.0) {
        Serial.print("⚠ WARNING: Moved ");
        Serial.print(movement, 1);
        Serial.println("° (expected ~30°)");
    }

    if (error > 3.0) {
        Serial.print("✗ FAILED: Large error ");
        Serial.print(error, 1);
        Serial.println("° (motor moves but control failing)");
        logMotorState(motorControl, "Final (poor tracking)");
        return;
    }

    Serial.println("✓✓✓ TEST PASSED - Motor working!\n");
}

void runPositionSweepTest(MotorController& motorControl) {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║            5-Position Motor Control Sweep Test                 ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("\nThis test verifies precise motor position control by moving");
    Serial.println("through 5 positions within ±30° (safe for cables).");
    Serial.println("Compares commanded position vs encoder reading (source of truth).\n");

    // Auto-enable motor for this test
    if (!motorControl.isEnabled()) {
        Serial.print("Enabling motor... ");
        motorControl.enable();
        if (motorControl.isEnabled()) {
            Serial.println("✓ Motor enabled");
        } else {
            Serial.println("✗ FAILED - motor not calibrated!");
            Serial.println("Please run calibration first (type 'c' or 'calibrate')");
            return;
        }
    } else {
        Serial.println("Motor already enabled");
    }

    delay(500);

    // Get starting position and calculate test positions (absolute coordinates)
    motorControl.updateEncoder();
    float start_angle = motorControl.getEncoderDegrees();
    Serial.print("\nStarting position: ");
    Serial.print(start_angle, 2);
    Serial.println("° (absolute)");
    Serial.println("Will test 5 positions within ±30° range (safe for cables)");
    delay(1000);

    // Define 5 test positions as offsets from start, then convert to absolute
    const float position_offsets[] = {0.0, -15.0, -30.0, 15.0, 30.0};
    const int num_positions = 5;
    const float position_tolerance = 2.0;  // ±2° tolerance for success

    // Convert to absolute positions with wraparound
    float test_positions[num_positions];
    for (int i = 0; i < num_positions; i++) {
        test_positions[i] = start_angle + position_offsets[i];
        if (test_positions[i] < 0.0) test_positions[i] += 360.0;
        if (test_positions[i] >= 360.0) test_positions[i] -= 360.0;
    }

    Serial.println("\n───────────────────────────────────────────────────────────────");
    Serial.println("Starting position sweep...");
    Serial.println("───────────────────────────────────────────────────────────────\n");

    int positions_passed = 0;
    int positions_failed = 0;

    for (int i = 0; i < num_positions; i++) {
        float target_deg = test_positions[i];
        float offset = position_offsets[i];

        Serial.print("[");
        Serial.print(i + 1);
        Serial.print("/");
        Serial.print(num_positions);
        Serial.print("] Moving to ");
        Serial.print(target_deg, 1);
        Serial.print("° (");
        if (offset >= 0) Serial.print("+");
        Serial.print(offset, 1);
        Serial.println("° from start)...");

        // Command movement
        motorControl.moveToPosition(target_deg);

        // Wait for movement to complete (with timeout)
        unsigned long start_time = millis();
        const unsigned long timeout_ms = 5000;  // 5 second timeout
        bool settled = false;

        while (millis() - start_time < timeout_ms) {
            motorControl.update();  // ✅ Run FOC control loop!
            delay(10);  // 100 Hz update rate

            // Check if settled at target
            if (motorControl.isAtTarget()) {
                settled = true;
                break;
            }
        }

        // Read actual position from encoder (source of truth!)
        motorControl.updateEncoder();
        float actual_deg = motorControl.getEncoderDegrees();
        float error_deg = actual_deg - target_deg;

        // Handle angle wrapping for error calculation
        if (error_deg > 180.0) error_deg -= 360.0;
        if (error_deg < -180.0) error_deg += 360.0;
        float abs_error_deg = abs(error_deg);

        // Calculate time taken
        unsigned long time_taken_ms = millis() - start_time;

        // Print results with clear pass/fail indication
        Serial.print("  Commanded: ");
        Serial.print(target_deg, 2);
        Serial.println("°");
        Serial.print("  Encoder:   ");
        Serial.print(actual_deg, 2);
        Serial.println("° (source of truth)");
        Serial.print("  Error:     ");
        Serial.print(error_deg >= 0 ? "+" : "");
        Serial.print(error_deg, 2);
        Serial.print("° (");
        if (abs_error_deg < position_tolerance) {
            Serial.print("✓ within ±");
            Serial.print(position_tolerance, 1);
            Serial.println("°)");
        } else {
            Serial.print("✗ exceeds ±");
            Serial.print(position_tolerance, 1);
            Serial.println("°)");
        }
        Serial.print("  Time:      ");
        Serial.print(time_taken_ms);
        Serial.println(" ms");

        // Check result
        if (!settled) {
            Serial.println("  Result:    ✗ TIMEOUT - Motor did not settle");
            positions_failed++;
        } else if (abs_error_deg < position_tolerance) {
            Serial.println("  Result:    ✓ PASS");
            positions_passed++;
        } else {
            Serial.println("  Result:    ✗ FAIL - Position error too large");
            positions_failed++;
        }

        Serial.println();
        delay(1000);  // Pause between movements
    }

    // Print summary
    Serial.println("───────────────────────────────────────────────────────────────");
    Serial.println("                        TEST SUMMARY");
    Serial.println("───────────────────────────────────────────────────────────────");
    Serial.print("Positions tested:  ");
    Serial.println(num_positions);
    Serial.print("✓ Passed:          ");
    Serial.print(positions_passed);
    Serial.print(" (");
    Serial.print((positions_passed * 100) / num_positions);
    Serial.println("%)");
    Serial.print("✗ Failed:          ");
    Serial.print(positions_failed);
    Serial.print(" (");
    Serial.print((positions_failed * 100) / num_positions);
    Serial.println("%)");
    Serial.print("Position tolerance: ±");
    Serial.print(position_tolerance, 1);
    Serial.println("°");
    Serial.println();

    // Overall assessment
    if (positions_passed == num_positions) {
        Serial.println("╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║  ✓✓✓ ALL TESTS PASSED - Motor control & encoder synchronized! ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.println("\nMotor is ready for operation!");
    } else if (positions_passed == 0) {
        Serial.println("╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║  ✗✗✗ ALL TESTS FAILED - Motor not responding at all!          ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.println("\nCRITICAL: Motor didn't move to any position!");
        Serial.println("Run 'motor_test' for detailed diagnostics.");
    } else {
        Serial.println("╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║  ✗ PARTIAL FAILURE - Some positions missed                    ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.println("\nMotor moves but has accuracy issues.");
        Serial.println("Recommended actions:");
        Serial.println("  1. Adjust PID values in config.h");
        Serial.println("  2. Check for mechanical binding");
        Serial.println("  3. Verify encoder mounting is secure");
    }
    Serial.println();
}

//=============================================================================
// SIMPLEFOC DIAGNOSTIC TEST - Root cause analysis for frozen shaft_angle
//=============================================================================

void runSimpleFOCDiagnostic(MotorController& motorControl) {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║       SimpleFOC Diagnostic Test - Root Cause Analysis          ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("\nThis test diagnoses why motor.shaft_angle is frozen during move()");
    Serial.println("Testing each theory from dev_log_8 systematically.\n");

    BLDCMotor& motor = motorControl.getMotor();
    MT6701Sensor& encoder = motorControl.getEncoder();

    // Ensure motor is calibrated
    if (!motorControl.isCalibrated()) {
        Serial.println("✗ Motor not calibrated! Run 'c' first.");
        return;
    }

    Serial.println("═══════════════════════════════════════════════════════════════════");
    Serial.println("TEST 1: SimpleFOC Control Mode Verification");
    Serial.println("═══════════════════════════════════════════════════════════════════");

    Serial.print("  motor.controller = ");
    switch (motor.controller) {
        case MotionControlType::torque: Serial.println("torque"); break;
        case MotionControlType::velocity: Serial.println("velocity"); break;
        case MotionControlType::angle: Serial.println("angle ✓"); break;
        case MotionControlType::velocity_openloop: Serial.println("velocity_openloop"); break;
        case MotionControlType::angle_openloop: Serial.println("angle_openloop"); break;
        default: Serial.println("UNKNOWN ✗"); break;
    }

    Serial.print("  motor.torque_controller = ");
    switch (motor.torque_controller) {
        case TorqueControlType::voltage: Serial.println("voltage ✓"); break;
        case TorqueControlType::dc_current: Serial.println("dc_current"); break;
        case TorqueControlType::foc_current: Serial.println("foc_current"); break;
        default: Serial.println("UNKNOWN ✗"); break;
    }

    Serial.print("  motor.enabled = ");
    Serial.println(motor.enabled ? "YES" : "NO");

    Serial.print("  motor.sensor_direction = ");
    Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");

    Serial.print("  motor.zero_electric_angle = ");
    Serial.print(radiansToDegrees(motor.zero_electric_angle), 1);
    Serial.println("°");

    Serial.print("  motor.voltage_limit = ");
    Serial.print(motor.voltage_limit, 1);
    Serial.println("V");

    Serial.print("  motor.sensor linked = ");
    Serial.println(motor.sensor ? "YES ✓" : "NO ✗");

    delay(500);

    Serial.println("\n═══════════════════════════════════════════════════════════════════");
    Serial.println("TEST 2: Sensor Call Verification (is getSensorAngle() being called?)");
    Serial.println("═══════════════════════════════════════════════════════════════════");

    // Enable motor
    motorControl.enable();
    delay(100);

    // Reset call counter
    encoder.resetCallCount();
    unsigned long calls_before = encoder.getCallCount();

    Serial.print("  Calls before loopFOC(): ");
    Serial.println(calls_before);

    // Run loopFOC() 100 times
    Serial.print("  Running motor.loopFOC() 100 times... ");
    for (int i = 0; i < 100; i++) {
        motor.loopFOC();
        delayMicroseconds(100);
    }

    unsigned long calls_after = encoder.getCallCount();
    Serial.print("done\n  Calls after loopFOC(): ");
    Serial.println(calls_after);

    unsigned long calls_made = calls_after - calls_before;
    Serial.print("  getSensorAngle() called: ");
    Serial.print(calls_made);
    Serial.println(" times");

    if (calls_made >= 100) {
        Serial.println("  ✓ PASS: SimpleFOC IS calling getSensorAngle()");
    } else if (calls_made > 0) {
        Serial.println("  ⚠ PARTIAL: SimpleFOC calling sensor, but less than expected");
    } else {
        Serial.println("  ✗ FAIL: SimpleFOC NOT calling getSensorAngle()!");
        Serial.println("    → This is the root cause! Sensor not being read.");
    }

    delay(500);

    Serial.println("\n═══════════════════════════════════════════════════════════════════");
    Serial.println("TEST 3: shaft_angle Update Verification");
    Serial.println("═══════════════════════════════════════════════════════════════════");

    // Read shaft_angle before and after loopFOC while physically moving motor
    Serial.println("  NOTE: Manually rotate motor shaft during this test!");
    Serial.println("  Reading shaft_angle before/after loopFOC()...");
    delay(2000);  // Give user time to prepare

    float shaft_before = motor.shaft_angle;
    Serial.print("  shaft_angle before: ");
    Serial.print(radiansToDegrees(shaft_before), 2);
    Serial.println("°");

    // Run loopFOC multiple times
    for (int i = 0; i < 50; i++) {
        motor.loopFOC();
        delay(20);
    }

    float shaft_after = motor.shaft_angle;
    Serial.print("  shaft_angle after:  ");
    Serial.print(radiansToDegrees(shaft_after), 2);
    Serial.println("°");

    float shaft_change = abs(radiansToDegrees(shaft_after - shaft_before));
    Serial.print("  Change: ");
    Serial.print(shaft_change, 2);
    Serial.println("°");

    if (shaft_change > 1.0) {
        Serial.println("  ✓ PASS: shaft_angle IS updating from sensor");
    } else {
        Serial.println("  ⚠ shaft_angle barely changed (did you rotate the motor?)");
    }

    delay(500);

    Serial.println("\n═══════════════════════════════════════════════════════════════════");
    Serial.println("TEST 4: Position Control with Detailed Logging");
    Serial.println("═══════════════════════════════════════════════════════════════════");

    // Get current position and target +30°
    float start_pos = radiansToDegrees(motor.shaft_angle);
    float target_pos = start_pos + 30.0;
    if (target_pos >= 360.0) target_pos -= 360.0;

    Serial.print("  Current position: ");
    Serial.print(start_pos, 1);
    Serial.println("°");
    Serial.print("  Target position:  ");
    Serial.print(target_pos, 1);
    Serial.println("°");

    motorControl.moveToPosition(target_pos);

    Serial.println("\n  Time  | shaft_angle | velocity  | Vq    | sensor_calls | Δshaft");
    Serial.println("  ------|-------------|-----------|-------|--------------|-------");

    float prev_shaft = motor.shaft_angle;
    encoder.resetCallCount();
    unsigned long start_time = millis();

    for (int i = 0; i < 30; i++) {  // 3 seconds
        motorControl.update();
        delay(100);

        float curr_shaft = motor.shaft_angle;
        float shaft_delta = radiansToDegrees(curr_shaft - prev_shaft);
        unsigned long elapsed = millis() - start_time;

        Serial.print("  ");
        Serial.print(elapsed);
        Serial.print("ms | ");
        Serial.print(radiansToDegrees(curr_shaft), 1);
        Serial.print("°     | ");
        Serial.print(radiansToDegrees(motor.shaft_velocity), 1);
        Serial.print("°/s  | ");
        Serial.print(motor.voltage.q, 1);
        Serial.print("V  | ");
        Serial.print(encoder.getCallCount());
        Serial.print("          | ");
        Serial.print(shaft_delta, 2);
        Serial.println("°");

        prev_shaft = curr_shaft;
    }

    Serial.print("\n  Total getSensorAngle() calls: ");
    Serial.println(encoder.getCallCount());

    delay(500);

    Serial.println("\n═══════════════════════════════════════════════════════════════════");
    Serial.println("TEST 5: Velocity Mode Test");
    Serial.println("═══════════════════════════════════════════════════════════════════");

    // Switch to velocity mode
    Serial.println("  Switching to velocity control mode...");
    motor.controller = MotionControlType::velocity;

    float target_velocity = degreesToRadians(60.0);  // 60°/s = 10 RPM
    Serial.print("  Target velocity: 60°/s (");
    Serial.print(target_velocity, 2);
    Serial.println(" rad/s)");

    start_time = millis();
    encoder.resetCallCount();
    float start_shaft = motor.shaft_angle;

    Serial.println("\n  Time  | shaft_angle | velocity  | Vq    | movement");
    Serial.println("  ------|-------------|-----------|-------|----------");

    for (int i = 0; i < 20; i++) {  // 2 seconds
        motor.loopFOC();
        motor.move(target_velocity);
        delay(100);

        float movement = radiansToDegrees(motor.shaft_angle - start_shaft);
        unsigned long elapsed = millis() - start_time;

        Serial.print("  ");
        Serial.print(elapsed);
        Serial.print("ms | ");
        Serial.print(radiansToDegrees(motor.shaft_angle), 1);
        Serial.print("°     | ");
        Serial.print(radiansToDegrees(motor.shaft_velocity), 1);
        Serial.print("°/s  | ");
        Serial.print(motor.voltage.q, 1);
        Serial.print("V  | ");
        Serial.print(movement, 1);
        Serial.println("°");
    }

    float total_movement = abs(radiansToDegrees(motor.shaft_angle - start_shaft));
    Serial.print("\n  Total movement in velocity mode: ");
    Serial.print(total_movement, 1);
    Serial.println("°");

    if (total_movement > 30.0) {
        Serial.println("  ✓ PASS: Motor moves in velocity mode!");
        Serial.println("    → Issue is specific to position (angle) mode");
    } else {
        Serial.println("  ✗ FAIL: Motor doesn't move in velocity mode either");
        Serial.println("    → Issue is in loopFOC() or sensor");
    }

    // Restore angle mode
    motor.controller = MotionControlType::angle;

    delay(500);

    Serial.println("\n═══════════════════════════════════════════════════════════════════");
    Serial.println("TEST 6: Open-Loop Mode Test (bypasses sensor)");
    Serial.println("═══════════════════════════════════════════════════════════════════");

    Serial.println("  Switching to open-loop angle mode (no sensor feedback)...");
    motor.controller = MotionControlType::angle_openloop;

    start_shaft = motor.shaft_angle;
    float openloop_target = start_shaft + degreesToRadians(30.0);

    Serial.print("  Commanding +30° open-loop rotation...\n");

    start_time = millis();
    for (int i = 0; i < 20; i++) {
        motor.loopFOC();
        motor.move(openloop_target);
        delay(100);
    }

    // Read encoder directly
    encoder.update();
    float encoder_movement = abs(encoder.getDegrees() - radiansToDegrees(start_shaft));
    if (encoder_movement > 180.0) encoder_movement = 360.0 - encoder_movement;

    Serial.print("  Encoder shows movement: ");
    Serial.print(encoder_movement, 1);
    Serial.println("°");

    if (encoder_movement > 10.0) {
        Serial.println("  ✓ PASS: Motor moves in open-loop mode!");
        Serial.println("    → Hardware is working, issue is closed-loop sensor feedback");
    } else {
        Serial.println("  ✗ FAIL: Motor doesn't move in open-loop either");
        Serial.println("    → Check hardware (driver, wiring, power)");
    }

    // Restore angle mode
    motor.controller = MotionControlType::angle;

    delay(500);

    Serial.println("\n═══════════════════════════════════════════════════════════════════");
    Serial.println("TEST 7: Direct setPhaseVoltage() Rotation (like calibration)");
    Serial.println("═══════════════════════════════════════════════════════════════════");

    Serial.println("  This replicates what works during calibration...");

    encoder.update();
    float start_enc = encoder.getDegrees();

    Serial.print("  Starting encoder position: ");
    Serial.print(start_enc, 1);
    Serial.println("°");

    Serial.println("  Rotating electrical angle 0° → 90° → 180° → 270° → 360°...\n");

    float test_angles[] = {0, _PI_2, PI, _3PI_2, _2PI};
    for (int i = 0; i < 5; i++) {
        motor.setPhaseVoltage(6.0, 0, test_angles[i]);
        delay(500);

        encoder.update();
        float pos = encoder.getDegrees();
        float movement = pos - start_enc;
        if (movement < -180.0) movement += 360.0;
        if (movement > 180.0) movement -= 360.0;

        Serial.print("  Elec angle ");
        Serial.print(radiansToDegrees(test_angles[i]), 0);
        Serial.print("°: Encoder=");
        Serial.print(pos, 1);
        Serial.print("° (moved ");
        Serial.print(movement, 1);
        Serial.println("°)");
    }

    // Stop voltage
    motor.setPhaseVoltage(0, 0, 0);

    encoder.update();
    float final_enc = encoder.getDegrees();
    total_movement = abs(final_enc - start_enc);
    if (total_movement > 180.0) total_movement = 360.0 - total_movement;

    Serial.print("\n  Total movement with setPhaseVoltage(): ");
    Serial.print(total_movement, 1);
    Serial.println("°");

    if (total_movement > 5.0) {
        Serial.println("  ✓ PASS: setPhaseVoltage() moves motor!");
        Serial.println("    → Hardware works, SimpleFOC FOC loop has issue");
    } else {
        Serial.println("  ✗ FAIL: setPhaseVoltage() doesn't move motor");
        Serial.println("    → Check driver enable, power supply");
    }

    Serial.println("\n═══════════════════════════════════════════════════════════════════");
    Serial.println("                         SUMMARY");
    Serial.println("═══════════════════════════════════════════════════════════════════");

    Serial.println("\nDiagnostic complete. Based on results above:");
    Serial.println("  • If TEST 2 failed → SimpleFOC not reading sensor");
    Serial.println("  • If TEST 4 shows Δshaft=0 → shaft_angle not updating from sensor");
    Serial.println("  • If TEST 5 passes but TEST 4 fails → Position PID issue");
    Serial.println("  • If TEST 6 passes → Closed-loop feedback issue");
    Serial.println("  • If TEST 7 passes → FOC loop not applying correct voltages");
    Serial.println("\nReview output above to identify root cause.\n");

    // TEST 8: Try different zero_electric_angle offsets
    Serial.println("═══════════════════════════════════════════════════════════════════");
    Serial.println("TEST 8: Zero Electric Angle Offset Search");
    Serial.println("═══════════════════════════════════════════════════════════════════");
    Serial.println("  Testing different zero_electric_angle offsets to find correct alignment...\n");

    motorControl.enable();
    delay(100);

    float original_zero = motor.zero_electric_angle;
    float best_offset = 0;
    float best_movement = 0;

    // Test offsets: 0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°
    float offsets[] = {0, PI/4, PI/2, 3*PI/4, PI, 5*PI/4, 3*PI/2, 7*PI/4};
    const char* offset_names[] = {"0°", "45°", "90°", "135°", "180°", "225°", "270°", "315°"};

    for (int i = 0; i < 8; i++) {
        // Set new zero_electric_angle
        motor.zero_electric_angle = normalizeRadians(original_zero + offsets[i]);

        // Read starting position
        encoder.update();
        float start_enc = encoder.getDegrees();

        // Try to move +20° using velocity mode (simpler than position)
        motor.controller = MotionControlType::velocity;
        float target_vel = degreesToRadians(60.0);  // 60°/s

        // Run for 500ms
        for (int j = 0; j < 50; j++) {
            motor.loopFOC();
            motor.move(target_vel);
            delay(10);
        }

        // Read final position
        encoder.update();
        float end_enc = encoder.getDegrees();
        float movement = end_enc - start_enc;
        if (movement < -180.0) movement += 360.0;
        if (movement > 180.0) movement -= 360.0;

        Serial.print("  Offset ");
        Serial.print(offset_names[i]);
        Serial.print(" (zero=");
        Serial.print(radiansToDegrees(motor.zero_electric_angle), 1);
        Serial.print("°): moved ");
        Serial.print(movement, 1);
        Serial.print("°");

        if (movement > best_movement) {
            best_movement = movement;
            best_offset = offsets[i];
            Serial.print(" ← BEST so far!");
        }
        Serial.println();

        // Stop motor briefly
        motor.move(0);
        delay(200);
    }

    // Restore angle mode
    motor.controller = MotionControlType::angle;

    Serial.println();
    if (best_movement > 10.0) {
        Serial.print("  ✓ FOUND WORKING OFFSET: ");
        Serial.print(radiansToDegrees(best_offset), 0);
        Serial.println("°");
        Serial.print("    Best movement: ");
        Serial.print(best_movement, 1);
        Serial.println("°");
        Serial.print("    Recommended zero_electric_angle: ");
        Serial.print(radiansToDegrees(normalizeRadians(original_zero + best_offset)), 1);
        Serial.println("°");
        Serial.println("\n  → Update calibration or add this offset to zero_electric_angle");

        // Apply the best offset for user to test
        motor.zero_electric_angle = normalizeRadians(original_zero + best_offset);
        Serial.print("    Applied offset - run 'motor_test' to verify!\n");
    } else {
        Serial.println("  ✗ No offset produced significant movement.");
        Serial.println("    → Issue may be sensor_direction (try toggling FORCE_SENSOR_DIRECTION_CW)");
        Serial.println("    → Or try letting SimpleFOC do full auto-calibration");
        motor.zero_electric_angle = original_zero;  // Restore original
    }

    Serial.println();

    // Clean up: restore motor to known good state
    motor.controller = MotionControlType::angle;
    motor.zero_electric_angle = original_zero;  // Always restore original calibration
    motorControl.disable();

    Serial.println("  Motor state restored. Run 'c' to recalibrate if needed.\n");
}

void runFullTest(MotorController& motorControl) {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    Full System Test                            ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");

    // Step 1: Calibration
    Serial.println("\n=== Step 1: Calibration ===");
    if (motorControl.isCalibrated()) {
        Serial.println("Already calibrated - skipping");
    } else {
        if (!motorControl.calibrate()) {
            Serial.println("✗ Calibration failed - aborting test");
            return;
        }
    }

    delay(1000);

    // Step 2: PID Tuning
    // PID tuning is now done via Python script (motor_control/pid_tuner.py)
    // This allows for better visualization and more sophisticated tuning algorithms
    Serial.println("\n=== Step 2: PID Tuning ===");
    Serial.println("PID tuning available via Python: python motor_control/pid_tuner.py <port>");
    Serial.println("Modes: --mode step | autotune | sweep | manual");

    delay(1000);

    // Step 3: Motor test
    Serial.println("\n=== Step 3: Movement Test ===");
    runMotorTest(motorControl);

    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    Full Test Complete!                        ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝\n");
}
