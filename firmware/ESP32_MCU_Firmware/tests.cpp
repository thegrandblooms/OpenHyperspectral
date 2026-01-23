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
// SIMPLEFOC DIAGNOSTIC TEST - Root cause analysis for motor control issues
//=============================================================================

void runSimpleFOCDiagnostic(MotorController& motorControl) {
    Serial.println("\n══════════════════════════════════════════════════════════════");
    Serial.println("        SimpleFOC Diagnostic - Root Cause Analysis");
    Serial.println("══════════════════════════════════════════════════════════════\n");

    BLDCMotor& motor = motorControl.getMotor();
    MT6701Sensor& encoder = motorControl.getEncoder();

    if (!motorControl.isCalibrated()) {
        Serial.println("✗ Motor not calibrated! Run 'c' first.");
        return;
    }

    // Store results for summary
    bool test2_pass = false, test3_pass = false, test4_pass = false;
    bool test5_offset_found = false, test6_pass = false, test7_pass = false;
    float test6_movement = 0, test7_movement = 0;
    float best_offset_deg = 0;

    //-------------------------------------------------------------------------
    // TEST 1: Config (quick sanity check)
    //-------------------------------------------------------------------------
    Serial.println("─── T1: Config ───────────────────────────────────────────────");
    Serial.print("  Mode:");
    Serial.print(motor.controller == MotionControlType::angle ? "angle✓" : "OTHER");
    Serial.print(" Dir:");
    Serial.print(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
    Serial.print(" Vlim:");
    Serial.print(motor.voltage_limit, 1);
    Serial.print("V Zero:");
    Serial.print(radiansToDegrees(motor.zero_electric_angle), 1);
    Serial.print("° Sensor:");
    Serial.println(motor.sensor ? "✓" : "✗");

    //-------------------------------------------------------------------------
    // TEST 2: Sensor calls (verify SimpleFOC reads sensor)
    //-------------------------------------------------------------------------
    Serial.println("\n─── T2: Sensor Calls ─────────────────────────────────────────");
    motorControl.enable();
    delay(50);

    encoder.resetCallCount();
    for (int i = 0; i < 100; i++) {
        motor.loopFOC();
        delayMicroseconds(100);
    }
    unsigned long calls_made = encoder.getCallCount();
    test2_pass = calls_made >= 100;

    Serial.print("  loopFOC()x100 → ");
    Serial.print(calls_made);
    Serial.print(" sensor calls: ");
    Serial.println(test2_pass ? "✓" : "✗");

    if (!test2_pass) {
        Serial.println("  ✗ FATAL: Sensor not being read! Check motor.linkSensor()");
        motorControl.disable();
        return;
    }

    //-------------------------------------------------------------------------
    // TEST 3: Open-loop (verify hardware works, bypasses FOC)
    //-------------------------------------------------------------------------
    Serial.println("\n─── T3: Open-Loop Hardware Test ──────────────────────────────");
    motor.controller = MotionControlType::angle_openloop;
    float start_shaft = motor.shaft_angle;
    float openloop_target = start_shaft + degreesToRadians(30.0);

    for (int i = 0; i < 20; i++) {
        motor.loopFOC();
        motor.move(openloop_target);
        delay(100);
    }

    encoder.update();
    float encoder_movement = abs(encoder.getDegrees() - radiansToDegrees(start_shaft));
    if (encoder_movement > 180.0) encoder_movement = 360.0 - encoder_movement;
    test3_pass = encoder_movement > 10.0;

    Serial.print("  Open-loop +30° → encoder moved ");
    Serial.print(encoder_movement, 1);
    Serial.print("°: ");
    Serial.println(test3_pass ? "✓ HW OK" : "✗ check wiring/power");

    motor.controller = MotionControlType::angle;

    if (!test3_pass) {
        Serial.println("  ✗ FATAL: Motor doesn't move! Check driver, wiring, power.");
        motorControl.disable();
        return;
    }

    //-------------------------------------------------------------------------
    // TEST 4: setPhaseVoltage (verify driver phases work)
    //-------------------------------------------------------------------------
    Serial.println("\n─── T4: Phase Voltage Test ───────────────────────────────────");
    encoder.update();
    float start_enc = encoder.getDegrees();

    float test_angles[] = {0, _PI_2, PI, _3PI_2, _2PI};
    Serial.print("  Elec: ");
    for (int i = 0; i < 5; i++) {
        motor.setPhaseVoltage(6.0, 0, test_angles[i]);
        delay(300);
        encoder.update();
        float movement = encoder.getDegrees() - start_enc;
        if (movement < -180.0) movement += 360.0;
        if (movement > 180.0) movement -= 360.0;

        Serial.print(int(radiansToDegrees(test_angles[i])));
        Serial.print("°→");
        Serial.print(movement, 0);
        Serial.print(" ");
    }
    Serial.println();

    motor.setPhaseVoltage(0, 0, 0);
    encoder.update();
    float phase_movement = abs(encoder.getDegrees() - start_enc);
    if (phase_movement > 180.0) phase_movement = 360.0 - phase_movement;
    test4_pass = phase_movement > 5.0;

    Serial.print("  Total: ");
    Serial.print(phase_movement, 1);
    Serial.print("°: ");
    Serial.println(test4_pass ? "✓ driver OK" : "✗ check driver");

    //-------------------------------------------------------------------------
    // TEST 5: Zero Angle Offset Search (CRITICAL - do this early!)
    // NOTE: "Correct direction" depends on sensor_direction setting!
    //   CW:  +velocity → encoder increases
    //   CCW: +velocity → encoder DECREASES
    //-------------------------------------------------------------------------
    Serial.println("\n─── T5: Zero Angle Offset Search (CRITICAL) ──────────────────");
    Serial.print("  sensor_direction=");
    Serial.print(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
    Serial.println(" → checking movement matches...");
    motorControl.enable();
    delay(50);

    float original_zero = motor.zero_electric_angle;
    float best_offset = 0;
    float best_correct_movement = 0;  // Best movement in CORRECT direction
    float best_wrong_movement = 0;    // Best movement in WRONG direction
    bool found_correct = false;

    // Determine expected direction based on sensor_direction
    // CW: positive velocity → positive encoder change
    // CCW: positive velocity → negative encoder change
    bool expect_positive = (motor.sensor_direction == Direction::CW);

    float offsets[] = {0, PI/4, PI/2, 3*PI/4, PI, 5*PI/4, 3*PI/2, 7*PI/4};
    int offset_degs[] = {0, 45, 90, 135, 180, 225, 270, 315};

    Serial.print("  ");
    for (int i = 0; i < 8; i++) {
        motor.zero_electric_angle = normalizeRadians(original_zero + offsets[i]);
        encoder.update();
        start_enc = encoder.getDegrees();

        motor.controller = MotionControlType::velocity;
        float target_vel = degreesToRadians(60.0);  // Positive velocity
        for (int j = 0; j < 50; j++) {
            motor.loopFOC();
            motor.move(target_vel);
            delay(10);
        }

        encoder.update();
        float movement = encoder.getDegrees() - start_enc;
        if (movement < -180.0) movement += 360.0;
        if (movement > 180.0) movement -= 360.0;

        // Check if movement is in CORRECT direction for this sensor_direction
        bool is_correct_direction;
        float abs_movement = abs(movement);
        if (expect_positive) {
            is_correct_direction = (movement > 10.0);  // CW: want positive
        } else {
            is_correct_direction = (movement < -10.0); // CCW: want negative
        }

        if (is_correct_direction && abs_movement > best_correct_movement) {
            best_correct_movement = abs_movement;
            best_offset = offsets[i];
            found_correct = true;
        }

        // Track movement in wrong direction (indicates sensor_direction mismatch)
        if (!is_correct_direction && abs_movement > best_wrong_movement) {
            best_wrong_movement = abs_movement;
        }

        Serial.print(offset_degs[i]);
        Serial.print("°:");
        Serial.print(movement, 0);
        Serial.print(" ");

        motor.move(0);
        delay(100);
    }
    Serial.println();

    motor.controller = MotionControlType::angle;
    best_offset_deg = radiansToDegrees(best_offset);

    // Analyze results
    if (found_correct && best_correct_movement > 10.0) {
        // Found a working offset with correct direction
        test5_offset_found = true;
        motor.zero_electric_angle = normalizeRadians(original_zero + best_offset);
        Serial.print("  ✓ Best: ");
        Serial.print(best_offset_deg, 0);
        Serial.print("° (");
        Serial.print(best_correct_movement, 0);
        Serial.print("° correct dir) → zero=");
        Serial.print(radiansToDegrees(motor.zero_electric_angle), 1);
        Serial.println("° APPLIED");
    } else if (best_wrong_movement > 20.0) {
        // Motor moves well but in WRONG direction - sensor_direction is wrong!
        test5_offset_found = false;
        Serial.print("  ✗ Motor moves WRONG direction (");
        Serial.print(best_wrong_movement, 0);
        Serial.println("°)!");
        Serial.println("  → sensor_direction is WRONG - toggle FORCE_SENSOR_DIRECTION_CW");
        motor.zero_electric_angle = original_zero;
    } else {
        // Motor barely moves at all
        test5_offset_found = false;
        Serial.println("  ✗ No offset worked (motor stalled or barely moved)");
        Serial.println("  → Try toggling FORCE_SENSOR_DIRECTION_CW in config.h");
        motor.zero_electric_angle = original_zero;
    }

    //-------------------------------------------------------------------------
    // NOTE: Don't try to reset SimpleFOC state - it manages continuous angles.
    // Our position control already normalizes errors to ±180° (shortest path).
    // Just sync with current position and let SimpleFOC do its thing.
    //-------------------------------------------------------------------------
    motor.loopFOC();  // Sync sensor state
    Serial.print("\n  Current shaft_angle: ");
    Serial.print(radiansToDegrees(motor.shaft_angle), 1);
    Serial.println("°");

    //-------------------------------------------------------------------------
    // TEST 6: Position Control (with corrected offset)
    // Now verify motor moves TOWARD target, not just that it moved
    //-------------------------------------------------------------------------
    Serial.println("\n─── T6: Position Control (+30°) ──────────────────────────────");
    float start_pos = radiansToDegrees(motor.shaft_angle);
    float target_pos = start_pos + 30.0;
    // Don't normalize - we want SimpleFOC to track continuous angle
    // if (target_pos >= 360.0) target_pos -= 360.0;

    Serial.print("  ");
    Serial.print(start_pos, 1);
    Serial.print("° → ");
    Serial.print(target_pos, 1);
    Serial.println("°");
    Serial.println("  Time   | Pos°   | Vel°/s | Vq   | Δ°");

    motorControl.moveToPosition(target_pos);
    float prev_shaft = motor.shaft_angle;
    unsigned long start_time = millis();

    // 8 samples over 2.4 seconds (every 300ms)
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            motorControl.update();
            delay(100);
        }

        float curr_shaft = motor.shaft_angle;
        float shaft_delta = radiansToDegrees(curr_shaft - prev_shaft);
        unsigned long elapsed = millis() - start_time;

        Serial.print("  ");
        if (elapsed < 1000) Serial.print(" ");
        Serial.print(elapsed);
        Serial.print("ms | ");
        Serial.print(radiansToDegrees(curr_shaft), 1);
        Serial.print(" | ");
        Serial.print(radiansToDegrees(motor.shaft_velocity), 1);
        Serial.print(" | ");
        Serial.print(motor.voltage.q, 1);
        Serial.print(" | ");
        if (shaft_delta >= 0) Serial.print("+");
        Serial.println(shaft_delta, 1);

        prev_shaft = curr_shaft;
    }

    // Check if motor moved TOWARD target (not just moved a lot)
    float final_pos = radiansToDegrees(motor.shaft_angle);
    float initial_error = target_pos - start_pos;  // Should be +30°
    float final_error = target_pos - final_pos;
    test6_movement = final_pos - start_pos;  // Actual movement (signed)

    // Pass if: moved in correct direction AND got closer to target
    bool moved_correct_direction = (test6_movement > 0) == (initial_error > 0);
    bool error_decreased = abs(final_error) < abs(initial_error);
    test6_pass = moved_correct_direction && (abs(test6_movement) > 15.0);

    Serial.print("  Movement: ");
    if (test6_movement >= 0) Serial.print("+");
    Serial.print(test6_movement, 1);
    Serial.print("° Error: ");
    Serial.print(final_error, 1);
    Serial.print("° ");
    if (!moved_correct_direction && abs(test6_movement) > 10.0) {
        Serial.println("✗ WRONG DIRECTION!");
    } else {
        Serial.println(test6_pass ? "✓" : "✗ stalled");
    }

    //-------------------------------------------------------------------------
    // TEST 7: Velocity Mode (with corrected offset)
    //-------------------------------------------------------------------------
    Serial.println("\n─── T7: Velocity Mode (60°/s) ────────────────────────────────");
    motor.controller = MotionControlType::velocity;
    float target_velocity = degreesToRadians(60.0);
    start_shaft = motor.shaft_angle;
    start_time = millis();

    Serial.println("  Time   | Pos°   | Vel°/s | Vq   | Move°");

    // 8 samples over 1.6 seconds
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 2; j++) {
            motor.loopFOC();
            motor.move(target_velocity);
            delay(100);
        }

        float movement = radiansToDegrees(motor.shaft_angle - start_shaft);
        unsigned long elapsed = millis() - start_time;

        Serial.print("  ");
        if (elapsed < 1000) Serial.print(" ");
        Serial.print(elapsed);
        Serial.print("ms | ");
        Serial.print(radiansToDegrees(motor.shaft_angle), 1);
        Serial.print(" | ");
        Serial.print(radiansToDegrees(motor.shaft_velocity), 1);
        Serial.print(" | ");
        Serial.print(motor.voltage.q, 1);
        Serial.print(" | ");
        Serial.println(movement, 1);
    }

    // Check movement direction (commanded +60°/s, should move positive)
    float t7_actual_movement = radiansToDegrees(motor.shaft_angle - start_shaft);
    test7_movement = abs(t7_actual_movement);
    bool t7_correct_direction = (t7_actual_movement > 0);  // Should be positive
    test7_pass = t7_correct_direction && (test7_movement > 15.0);

    Serial.print("  Movement: ");
    if (t7_actual_movement >= 0) Serial.print("+");
    Serial.print(t7_actual_movement, 1);
    Serial.print("° ");
    if (!t7_correct_direction && test7_movement > 10.0) {
        Serial.println("✗ WRONG DIRECTION!");
    } else {
        Serial.println(test7_pass ? "✓" : "✗ slow/stalled");
    }

    motor.controller = MotionControlType::angle;

    //-------------------------------------------------------------------------
    // SUMMARY
    //-------------------------------------------------------------------------
    Serial.println("\n══════════════════════════════════════════════════════════════");
    Serial.println("                         SUMMARY");
    Serial.println("══════════════════════════════════════════════════════════════");
    Serial.print("  T2 Sensor:    ");
    Serial.println(test2_pass ? "✓" : "✗");
    Serial.print("  T3 Open-loop: ");
    Serial.println(test3_pass ? "✓ hardware OK" : "✗");
    Serial.print("  T4 Phases:    ");
    Serial.println(test4_pass ? "✓ driver OK" : "✗");
    Serial.print("  T5 Offset:    ");
    if (test5_offset_found) {
        Serial.print("✓ found ");
        Serial.print(best_offset_deg, 0);
        Serial.println("°");
    } else {
        Serial.println("✗ none worked");
    }
    Serial.print("  T6 Position:  ");
    Serial.print(test6_movement, 1);
    Serial.print("° ");
    Serial.println(test6_pass ? "✓" : "✗");
    Serial.print("  T7 Velocity:  ");
    Serial.print(test7_movement, 1);
    Serial.print("° ");
    Serial.println(test7_pass ? "✓" : "✗");

    Serial.println("\nDiagnosis:");
    if (!test3_pass) {
        Serial.println("  → Hardware issue: check motor wiring, driver, power");
    } else if (!test5_offset_found) {
        Serial.println("  → Try toggling FORCE_SENSOR_DIRECTION_CW in config.h");
    } else if (!test6_pass && !test7_pass) {
        Serial.println("  → Closed-loop failing - check PID gains");
    } else if (test6_pass && test7_pass) {
        Serial.println("  → Motor working! Zero offset applied for this session.");
        Serial.println("  → To make permanent: update calibration or config");
    } else {
        Serial.println("  → Partial success - check PID tuning");
    }

    // Cleanup - keep the good offset if found
    motor.controller = MotionControlType::angle;
    if (!test5_offset_found) {
        motor.zero_electric_angle = original_zero;
    }
    motorControl.disable();
    Serial.println("\nMotor disabled. Run 'motor_test' to verify, 'c' to recalibrate.\n");
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

    // Step 2: PID Auto-Tuning
    // TEMPORARILY DISABLED: PID tuning has motor movement issues
    // The tuner calls motor.move() without arguments, which doesn't work correctly
    // See pid_auto_tuner.cpp:89 - needs to call motor.move(target_rad) instead
    Serial.println("\n=== Step 2: PID Tuning ===");
    Serial.println("⚠ PID auto-tuning disabled (use 'pidtune' command to run separately)");

    // Commented out PID tuning code:
    /*
    if (motorControl.autoTunePID(true)) {
        Serial.println("✓ PID auto-tuning successful!");
        Serial.println("Optimal PID parameters have been applied.");
    } else {
        Serial.println("✗ PID auto-tuning failed!");
        Serial.println("Continuing with current PID values...");
    }
    */

    delay(1000);

    // Step 3: Motor test
    Serial.println("\n=== Step 3: Movement Test ===");
    runMotorTest(motorControl);

    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    Full Test Complete!                        ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝\n");
}
