/**
 * Integration Test - Full System Test
 *
 * This test verifies the complete motor control system with encoder feedback.
 * Combines motor control, encoder reading, and closed-loop position control.
 *
 * Test Objectives:
 * - Verify motor + encoder integration
 * - Test closed-loop position control
 * - Validate FOC algorithm performance
 * - Measure positioning accuracy
 * - Test velocity control
 * - Verify calibration procedure
 *
 * Usage:
 * 1. Upload this sketch to the ESP32
 * 2. Open Serial Monitor at 115200 baud
 * 3. Follow calibration instructions
 * 4. Run automated test sequence
 *
 * WARNING: Motor will spin! Ensure safe setup.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include "../../ESP32_MCU_Firmware/pid_auto_tuner.h"

// Hardware configuration (from motor_firmware config.h)
#define MOTOR_ENABLE     15
#define MOTOR_PWM_A      13
#define MOTOR_PWM_B      11
#define MOTOR_PWM_C      12
#define ENCODER_SDA      47
#define ENCODER_SCL      48
#define ENCODER_I2C_ADDR 0x06

// Motor parameters
#define POLE_PAIRS       7
#define ENCODER_PPR      16384
#define VOLTAGE_PSU      12.0
#define CURRENT_LIMIT    2.0
#define MAX_VELOCITY     20.0

// Test configuration
#define HEARTBEAT_INTERVAL 10000  // 10 seconds
#define POSITION_TOLERANCE 0.1    // rad (~5.7 degrees)

// Create motor objects
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE);
MagneticSensorI2C encoder = MagneticSensorI2C(ENCODER_I2C_ADDR, 14, 0x03, 8);

// Test state
unsigned long last_heartbeat = 0;
bool calibrated = false;
String command_buffer = "";

// Oscillation detection
float last_positions[10];  // Ring buffer for position history
int position_index = 0;
unsigned long last_oscillation_check = 0;
#define OSCILLATION_CHECK_INTERVAL 100  // ms
#define OSCILLATION_THRESHOLD 5  // Number of direction changes in history
#define OSCILLATION_MIN_CHANGE 0.05  // rad - minimum change to count as movement

// Test statistics
struct TestResult {
    float target_position;
    float actual_position;
    float error;
    unsigned long settle_time_ms;
    bool success;
};

TestResult test_results[10];
int test_count = 0;

bool checkForOscillation() {
    // Update position history
    float current_pos = encoder.getAngle();
    last_positions[position_index] = current_pos;
    position_index = (position_index + 1) % 10;

    // Need at least a full buffer to detect oscillation
    static bool buffer_filled = false;
    static int fill_count = 0;
    if (!buffer_filled) {
        fill_count++;
        if (fill_count >= 10) {
            buffer_filled = true;
        }
        return false;
    }

    // Count direction changes
    int direction_changes = 0;
    for (int i = 0; i < 9; i++) {
        int curr_idx = (position_index + i) % 10;
        int next_idx = (position_index + i + 1) % 10;
        int next_next_idx = (position_index + i + 2) % 10;

        float change1 = last_positions[next_idx] - last_positions[curr_idx];
        float change2 = last_positions[next_next_idx] - last_positions[next_idx];

        // Check if significant movement and direction changed
        if (abs(change1) > OSCILLATION_MIN_CHANGE && abs(change2) > OSCILLATION_MIN_CHANGE) {
            if ((change1 > 0 && change2 < 0) || (change1 < 0 && change2 > 0)) {
                direction_changes++;
            }
        }
    }

    // If too many direction changes, we're oscillating
    if (direction_changes >= OSCILLATION_THRESHOLD) {
        return true;
    }

    return false;
}

void printHelp() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║               INTEGRATION TEST - COMMANDS                      ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("RECOMMENDED WORKFLOW:");
    Serial.println("  1. c, calibrate  - Run motor/encoder calibration (REQUIRED FIRST)");
    Serial.println("  2. p, pidtune    - Auto-tune PID (HIGHLY RECOMMENDED)");
    Serial.println("  3. t, test       - Run automated position test sequence");
    Serial.println();
    Serial.println("OTHER COMMANDS:");
    Serial.println("  h, help      - Show this help menu");
    Serial.println("  e, enable    - Enable motor");
    Serial.println("  d, disable   - Disable motor (emergency stop)");
    Serial.println("  s, status    - Show current motor/encoder status");
    Serial.println("  r, results   - Show test results from last test run");
    Serial.println("  m <angle>    - Move to angle in radians (e.g., 'm 3.14')");
    Serial.println();
    Serial.println("SAFETY:");
    Serial.println("  - Oscillation detection is active and will auto-disable motor");
    Serial.println("  - Press 'd' any time for emergency stop");
    Serial.println("  - Always run PID tuning after calibration to prevent oscillation");
    Serial.println();
}

void printStatus() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                     SYSTEM STATUS                              ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");

    // Encoder status
    float angle = encoder.getAngle();
    float velocity = encoder.getVelocity();
    Serial.print("Encoder Position: ");
    Serial.print(angle, 4);
    Serial.print(" rad (");
    Serial.print(angle * 180.0 / PI, 2);
    Serial.println(" deg)");
    Serial.print("Encoder Velocity: ");
    Serial.print(velocity, 2);
    Serial.println(" rad/s");

    // Motor status
    Serial.print("Motor Target: ");
    Serial.print(motor.target, 4);
    Serial.println(" rad");
    Serial.print("Motor Voltage: ");
    Serial.print(motor.voltage.q, 2);
    Serial.println(" V");
    Serial.print("Motor Enabled: ");
    Serial.println(motor.enabled ? "YES" : "NO");
    Serial.print("Calibrated: ");
    Serial.println(calibrated ? "YES" : "NO");

    // Position error
    float error = motor.target - angle;
    Serial.print("Position Error: ");
    Serial.print(error, 4);
    Serial.print(" rad (");
    Serial.print(error * 180.0 / PI, 2);
    Serial.println(" deg)");

    Serial.println();
}

void printResults() {
    if (test_count == 0) {
        Serial.println("No test results yet. Run 't' to start test sequence.");
        return;
    }

    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                      TEST RESULTS                              ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Test | Target (rad) | Actual (rad) | Error (rad) | Time (ms) | Result");
    Serial.println("──────────────────────────────────────────────────────────────────────");

    int passed = 0;
    float total_error = 0;
    float max_error = 0;

    for (int i = 0; i < test_count; i++) {
        Serial.print("  ");
        Serial.print(i + 1);
        Serial.print("  |    ");
        Serial.print(test_results[i].target_position, 2);
        Serial.print("     |    ");
        Serial.print(test_results[i].actual_position, 2);
        Serial.print("     |   ");
        Serial.print(test_results[i].error, 4);
        Serial.print("   |   ");
        Serial.print(test_results[i].settle_time_ms);
        Serial.print("   | ");
        Serial.println(test_results[i].success ? "PASS" : "FAIL");

        if (test_results[i].success) passed++;
        total_error += abs(test_results[i].error);
        if (abs(test_results[i].error) > max_error) {
            max_error = abs(test_results[i].error);
        }
    }

    Serial.println();
    Serial.print("Tests Passed: ");
    Serial.print(passed);
    Serial.print(" / ");
    Serial.println(test_count);

    float avg_error = total_error / test_count;
    Serial.print("Average Error: ");
    Serial.print(avg_error, 4);
    Serial.print(" rad (");
    Serial.print(avg_error * 180.0 / PI, 2);
    Serial.println(" deg)");

    Serial.print("Maximum Error: ");
    Serial.print(max_error, 4);
    Serial.print(" rad (");
    Serial.print(max_error * 180.0 / PI, 2);
    Serial.println(" deg)");

    Serial.println();
}

bool waitForPosition(float target, unsigned long timeout_ms, float tolerance = POSITION_TOLERANCE) {
    unsigned long start = millis();

    while (millis() - start < timeout_ms) {
        motor.loopFOC();
        motor.move();

        float current_pos = encoder.getAngle();
        float error = abs(target - current_pos);

        if (error < tolerance) {
            return true;  // Position reached
        }

        delay(10);
    }

    return false;  // Timeout
}

void runCalibration() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                  MOTOR CALIBRATION                             ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Starting motor calibration...");
    Serial.println("This will:");
    Serial.println("  1. Align encoder direction with motor rotation");
    Serial.println("  2. Initialize FOC algorithm");
    Serial.println("  3. Set electrical zero position");
    Serial.println();
    Serial.println("⚠ Motor will spin during calibration!");
    delay(2000);

    Serial.println("[1/3] Initializing FOC...");
    if (motor.initFOC()) {
        Serial.println("[OK]  FOC initialized successfully");
        calibrated = true;
    } else {
        Serial.println("[FAIL] FOC initialization failed!");
        Serial.println("      Check encoder and motor connections");
        return;
    }

    Serial.println();
    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║               CALIBRATION SUCCESSFUL!                          ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("IMPORTANT NEXT STEP:");
    Serial.println("  Run 'p' or 'pidtune' to auto-tune PID parameters!");
    Serial.println();
    Serial.println("Why PID tuning is critical:");
    Serial.println("  - Prevents motor oscillation and wild movements");
    Serial.println("  - Ensures accurate position tracking");
    Serial.println("  - Finds optimal gains for YOUR specific motor setup");
    Serial.println();
    Serial.println("Current PID: P=2.0 (very conservative, may be slow)");
    Serial.println("Expected after tuning: P=6-12 (fast and accurate)");
    Serial.println();
}

void runPIDTuning() {
    if (!calibrated) {
        Serial.println("ERROR: Motor must be calibrated first! Run 'c' to calibrate.");
        return;
    }

    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                  PID AUTO-TUNING                               ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Starting PID auto-tuning...");
    Serial.println("This will:");
    Serial.println("  1. Test motor response at multiple positions");
    Serial.println("  2. Start with conservative PID values to prevent overshoot");
    Serial.println("  3. Gradually increase gains for optimal performance");
    Serial.println("  4. Apply tuned parameters automatically");
    Serial.println();
    Serial.println("⚠ Motor will move to multiple positions during tuning!");
    Serial.println("  This may take several minutes.");
    Serial.println();
    delay(2000);

    // Ensure motor is enabled
    if (!motor.enabled) {
        Serial.println("Enabling motor...");
        motor.enable();
        delay(500);
    }

    // Create tuner and run tuning
    PIDAutoTuner tuner(motor, encoder);
    bool success = tuner.runTuning(true);  // verbose = true

    if (success) {
        // Apply optimal PID parameters
        float p, i, d, ramp;
        tuner.getOptimalPID(p, i, d, ramp);

        motor.P_angle.P = p;
        motor.P_angle.I = i;
        motor.P_angle.D = d;
        motor.P_angle.output_ramp = ramp;

        Serial.println();
        Serial.println("╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║             PID TUNING SUCCESSFUL!                             ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.println();
        Serial.println("Optimal PID parameters have been applied.");
        Serial.println();
        Serial.println("To make these values permanent, update config.h:");
        Serial.print("  #define PID_P_POSITION ");
        Serial.println(p, 2);
        Serial.print("  #define PID_I_POSITION ");
        Serial.println(i, 2);
        Serial.print("  #define PID_D_POSITION ");
        Serial.println(d, 3);
        Serial.println();
    } else {
        Serial.println();
        Serial.println("╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║             PID TUNING FAILED!                                 ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.println();
        Serial.println("PID tuning was unsuccessful. Check:");
        Serial.println("  - Motor is properly calibrated");
        Serial.println("  - Motor can move freely");
        Serial.println("  - Power supply is adequate");
        Serial.println("  - Encoder is reading correctly");
        Serial.println();
    }
}

void runTestSequence() {
    if (!calibrated) {
        Serial.println("ERROR: Motor must be calibrated first! Run 'c' to calibrate.");
        return;
    }

    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║             AUTOMATED TEST SEQUENCE                            ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.print("Current PID: P=");
    Serial.print(motor.P_angle.P, 2);
    Serial.print(", I=");
    Serial.print(motor.P_angle.I, 2);
    Serial.print(", D=");
    Serial.println(motor.P_angle.D, 3);
    Serial.println();

    if (motor.P_angle.P < 3.0) {
        Serial.println("WARNING: PID values are very conservative (P < 3.0)");
        Serial.println("         This may result in slow movements and poor tracking.");
        Serial.println("         Consider running 'p' to auto-tune PID first.");
        Serial.println();
        Serial.println("Continue anyway? Type 'y' and press enter, or 'n' to cancel:");

        // Wait for user confirmation
        String response = "";
        unsigned long start = millis();
        while (millis() - start < 10000) {  // 10 second timeout
            if (Serial.available() > 0) {
                char c = Serial.read();
                if (c == '\n' || c == '\r') {
                    response.trim();
                    response.toLowerCase();
                    if (response == "y" || response == "yes") {
                        break;
                    } else {
                        Serial.println("Test cancelled. Run 'p' to tune PID first.");
                        return;
                    }
                } else {
                    response += c;
                }
            }
            delay(10);
        }
    }

    // Ensure motor is enabled
    if (!motor.enabled) {
        Serial.println("Enabling motor...");
        motor.enable();
        delay(500);
    }

    Serial.println();

    test_count = 0;
    float test_positions[] = {0.0, 1.57, 3.14, 4.71, 6.28, 3.14, 0.0};
    int num_tests = sizeof(test_positions) / sizeof(test_positions[0]);

    for (int i = 0; i < num_tests; i++) {
        Serial.print("Test ");
        Serial.print(i + 1);
        Serial.print("/");
        Serial.print(num_tests);
        Serial.print(": Moving to ");
        Serial.print(test_positions[i], 2);
        Serial.println(" rad...");

        motor.target = test_positions[i];
        unsigned long start = millis();
        bool success = waitForPosition(test_positions[i], 5000);
        unsigned long settle_time = millis() - start;

        float actual = encoder.getAngle();
        float error = test_positions[i] - actual;

        test_results[test_count].target_position = test_positions[i];
        test_results[test_count].actual_position = actual;
        test_results[test_count].error = error;
        test_results[test_count].settle_time_ms = settle_time;
        test_results[test_count].success = success;
        test_count++;

        Serial.print("  Result: ");
        Serial.print(success ? "PASS" : "FAIL");
        Serial.print(" | Error: ");
        Serial.print(error * 180.0 / PI, 2);
        Serial.print(" deg | Time: ");
        Serial.print(settle_time);
        Serial.println(" ms");

        delay(500);
    }

    Serial.println();
    Serial.println("Test sequence complete!");
    printResults();
}

void processCommand() {
    command_buffer.trim();
    command_buffer.toLowerCase();

    if (command_buffer.length() == 0) return;

    if (command_buffer == "h" || command_buffer == "help") {
        printHelp();
    }
    else if (command_buffer == "c" || command_buffer == "calibrate") {
        runCalibration();
    }
    else if (command_buffer == "p" || command_buffer == "pidtune") {
        runPIDTuning();
    }
    else if (command_buffer == "e" || command_buffer == "enable") {
        motor.enable();
        Serial.println("Motor enabled!");
    }
    else if (command_buffer == "d" || command_buffer == "disable") {
        motor.disable();
        Serial.println("Motor disabled!");
    }
    else if (command_buffer == "t" || command_buffer == "test") {
        runTestSequence();
    }
    else if (command_buffer == "s" || command_buffer == "status") {
        printStatus();
    }
    else if (command_buffer == "r" || command_buffer == "results") {
        printResults();
    }
    else if (command_buffer.startsWith("m ")) {
        float angle = command_buffer.substring(2).toFloat();
        Serial.print("Moving to ");
        Serial.print(angle, 2);
        Serial.println(" rad...");
        motor.target = angle;
    }
    else {
        Serial.print("Unknown command: '");
        Serial.print(command_buffer);
        Serial.println("'. Type 'h' for help.");
    }

    command_buffer = "";
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║           INTEGRATION TEST - MOTOR + ENCODER                   ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();

    // Initialize I2C
    Serial.println("[INIT] Initializing I2C...");
    Wire.begin(ENCODER_SDA, ENCODER_SCL);
    Wire.setClock(400000);
    encoder.init();
    Serial.println("[OK]   Encoder initialized");

    // Initialize driver
    Serial.println("[INIT] Initializing motor driver...");
    driver.voltage_power_supply = VOLTAGE_PSU;
    driver.voltage_limit = VOLTAGE_PSU / 2;
    driver.init();
    motor.linkDriver(&driver);
    Serial.println("[OK]   Driver initialized");

    // Link encoder to motor
    Serial.println("[INIT] Linking encoder to motor...");
    motor.linkSensor(&encoder);

    // Configure motor with CONSERVATIVE PID values
    // These prevent oscillation but may be slow - run PID tuning for optimal values
    motor.controller = MotionControlType::angle;
    motor.voltage_limit = VOLTAGE_PSU / 2;
    motor.velocity_limit = MAX_VELOCITY;
    motor.PID_velocity.P = 0.5;
    motor.PID_velocity.I = 10.0;
    motor.P_angle.P = 2.0;   // Conservative - prevents oscillation
    motor.P_angle.I = 0.0;
    motor.P_angle.D = 0.0;
    motor.current_limit = CURRENT_LIMIT;

    // Initialize motor
    motor.init();
    Serial.println("[OK]   Motor initialized");

    Serial.println();
    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                      SYSTEM READY!                             ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("RECOMMENDED WORKFLOW:");
    Serial.println("  1. Type 'c' to calibrate motor/encoder");
    Serial.println("  2. Type 'p' to auto-tune PID (prevents oscillation!)");
    Serial.println("  3. Type 't' to run automated position tests");
    Serial.println("  4. Type 'h' for all commands");
    Serial.println();
    Serial.println("NOTE: Using conservative PID values (P=2.0) until tuning is run.");
    Serial.println("      This prevents oscillation but may be slow. Run 'p' for optimal PID.");
    Serial.println();

    last_heartbeat = millis();

    // Initialize position history for oscillation detection
    for (int i = 0; i < 10; i++) {
        last_positions[i] = 0.0;
    }
}

void loop() {
    // Run motor control loop
    motor.loopFOC();
    motor.move();

    // Oscillation detection
    unsigned long current_time = millis();
    if (motor.enabled && current_time - last_oscillation_check >= OSCILLATION_CHECK_INTERVAL) {
        last_oscillation_check = current_time;

        if (checkForOscillation()) {
            Serial.println();
            Serial.println("╔════════════════════════════════════════════════════════════════╗");
            Serial.println("║                   ⚠ OSCILLATION DETECTED! ⚠                   ║");
            Serial.println("╚════════════════════════════════════════════════════════════════╝");
            Serial.println("Motor is oscillating wildly - DISABLING for safety!");
            Serial.println();
            Serial.println("This usually means PID gains are too aggressive.");
            Serial.println("Solutions:");
            Serial.println("  1. Run 'p' to auto-tune PID parameters");
            Serial.println("  2. Manually reduce P gain (currently using conservative defaults)");
            Serial.println("  3. Check for mechanical obstructions");
            Serial.println();

            motor.disable();
            calibrated = false;  // Require recalibration after oscillation
        }
    }

    // Check for serial commands
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            processCommand();
        } else {
            command_buffer += c;
        }
    }

    // Heartbeat every 10 seconds
    unsigned long current_time = millis();
    if (current_time - last_heartbeat >= HEARTBEAT_INTERVAL) {
        last_heartbeat = current_time;

        Serial.print("[HEARTBEAT] Uptime: ");
        Serial.print(current_time / 1000);
        Serial.print("s | Pos: ");
        Serial.print(encoder.getAngle(), 2);
        Serial.print(" rad | Enabled: ");
        Serial.print(motor.enabled ? "Y" : "N");
        Serial.print(" | Cal: ");
        Serial.println(calibrated ? "Y" : "N");
    }

    delay(1);
}
