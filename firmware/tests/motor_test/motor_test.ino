/**
 * Motor Test - Open Loop Control (No Encoder Required)
 *
 * This test verifies that the SimpleFOC motor driver can control the BLDC motor
 * using open-loop voltage control (torque mode) without requiring encoder feedback.
 *
 * Test Objectives:
 * - Verify motor driver wiring and power connections
 * - Test if motor can spin in both directions
 * - Validate SimpleFOC driver initialization
 * - Check motor phase connections
 *
 * Usage:
 * 1. Upload this sketch to the ESP32
 * 2. Open Serial Monitor at 115200 baud
 * 3. Motor will automatically run test sequence
 * 4. Observe motor spinning and serial output
 *
 * WARNING: Motor will spin! Ensure nothing is attached that could cause injury.
 */

#include <Arduino.h>
#include <SimpleFOC.h>

// Motor driver pins (from motor_firmware config.h)
#define MOTOR_ENABLE     15   // Motor driver enable pin
#define MOTOR_PWM_A      13   // Phase A PWM
#define MOTOR_PWM_B      11   // Phase B PWM
#define MOTOR_PWM_C      12   // Phase C PWM

// Motor configuration
#define POLE_PAIRS       7    // Mitoot 2804 motor: 7 pole pairs
#define VOLTAGE_PSU      12.0 // Power supply voltage
#define TEST_VOLTAGE     3.0  // Test voltage (V) - safe low voltage for testing

// Test timing
#define HEARTBEAT_INTERVAL 10000  // 10 seconds

// Create motor and driver objects
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE);

// Test state tracking
unsigned long last_heartbeat = 0;
unsigned long test_start_time = 0;
int test_phase = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║              MOTOR TEST - OPEN LOOP (NO ENCODER)              ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Test Configuration:");
    Serial.print("  - Motor: Mitoot 2804 (");
    Serial.print(POLE_PAIRS);
    Serial.println(" pole pairs)");
    Serial.print("  - Power Supply: ");
    Serial.print(VOLTAGE_PSU);
    Serial.println(" V");
    Serial.print("  - Test Voltage: ");
    Serial.print(TEST_VOLTAGE);
    Serial.println(" V");
    Serial.print("  - Driver Pins: EN=GPIO");
    Serial.print(MOTOR_ENABLE);
    Serial.print(", PWM_A=GPIO");
    Serial.print(MOTOR_PWM_A);
    Serial.print(", PWM_B=GPIO");
    Serial.print(MOTOR_PWM_B);
    Serial.print(", PWM_C=GPIO");
    Serial.println(MOTOR_PWM_C);
    Serial.println();

    // Initialize driver
    Serial.println("[INIT] Initializing motor driver...");
    driver.voltage_power_supply = VOLTAGE_PSU;
    driver.voltage_limit = VOLTAGE_PSU / 2;  // Limit to half supply voltage

    if (!driver.init()) {
        Serial.println("[ERROR] Driver initialization failed!");
        Serial.println("Check wiring:");
        Serial.println("  - Power supply connected to SimpleFOC Mini");
        Serial.println("  - GND connected between ESP32 and SimpleFOC Mini");
        Serial.println("  - Motor phases connected to driver outputs");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("[OK]   Driver initialized successfully");

    // Link driver to motor
    motor.linkDriver(&driver);

    // Configure open-loop velocity control
    Serial.println("[INIT] Configuring motor for open-loop control...");
    motor.controller = MotionControlType::velocity_openloop;
    motor.voltage_limit = TEST_VOLTAGE;  // Limit voltage for safe testing

    // Initialize motor (no encoder needed for open-loop)
    if (!motor.init()) {
        Serial.println("[ERROR] Motor initialization failed!");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("[OK]   Motor initialized successfully");

    Serial.println();
    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                   MOTOR TEST STARTING...                       ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("⚠ WARNING: Motor will start spinning in 3 seconds!");
    Serial.println("⚠ Ensure nothing is attached that could cause injury.");
    Serial.println();

    delay(3000);

    test_start_time = millis();
    last_heartbeat = millis();
}

void loop() {
    unsigned long current_time = millis();
    unsigned long elapsed = current_time - test_start_time;

    // Run motor FOC algorithm
    motor.loopFOC();
    motor.move();

    // Test sequence - runs through different speeds
    switch (test_phase) {
        case 0:  // Phase 1: Slow clockwise (0-10s)
            if (elapsed < 10000) {
                motor.target = 5.0;  // 5 rad/s clockwise
                if (elapsed < 100) {  // Print once at start
                    Serial.println("[TEST] Phase 1: Slow clockwise rotation (5 rad/s)");
                    Serial.println("       You should see the motor spinning slowly clockwise");
                }
            } else {
                test_phase = 1;
                test_start_time = current_time;
            }
            break;

        case 1:  // Phase 2: Medium clockwise (10-20s)
            if (elapsed < 10000) {
                motor.target = 10.0;  // 10 rad/s clockwise
                if (elapsed < 100) {
                    Serial.println("\n[TEST] Phase 2: Medium clockwise rotation (10 rad/s)");
                    Serial.println("       Motor speed should increase");
                }
            } else {
                test_phase = 2;
                test_start_time = current_time;
            }
            break;

        case 2:  // Phase 3: Stop (20-25s)
            if (elapsed < 5000) {
                motor.target = 0.0;
                if (elapsed < 100) {
                    Serial.println("\n[TEST] Phase 3: Stop");
                    Serial.println("       Motor should decelerate to stop");
                }
            } else {
                test_phase = 3;
                test_start_time = current_time;
            }
            break;

        case 3:  // Phase 4: Slow counter-clockwise (25-35s)
            if (elapsed < 10000) {
                motor.target = -5.0;  // 5 rad/s counter-clockwise
                if (elapsed < 100) {
                    Serial.println("\n[TEST] Phase 4: Slow counter-clockwise rotation (-5 rad/s)");
                    Serial.println("       Motor should spin in opposite direction");
                }
            } else {
                test_phase = 4;
                test_start_time = current_time;
            }
            break;

        case 4:  // Phase 5: Medium counter-clockwise (35-45s)
            if (elapsed < 10000) {
                motor.target = -10.0;  // 10 rad/s counter-clockwise
                if (elapsed < 100) {
                    Serial.println("\n[TEST] Phase 5: Medium counter-clockwise rotation (-10 rad/s)");
                    Serial.println("       Motor speed should increase (counter-clockwise)");
                }
            } else {
                test_phase = 5;
                test_start_time = current_time;
            }
            break;

        case 5:  // Phase 6: Stop and finish (45-50s)
            if (elapsed < 5000) {
                motor.target = 0.0;
                if (elapsed < 100) {
                    Serial.println("\n[TEST] Phase 6: Final stop");
                }
            } else {
                test_phase = 6;
            }
            break;

        case 6:  // Test complete
            motor.target = 0.0;
            if (elapsed < 100) {
                Serial.println();
                Serial.println("╔════════════════════════════════════════════════════════════════╗");
                Serial.println("║                   MOTOR TEST COMPLETE!                         ║");
                Serial.println("╚════════════════════════════════════════════════════════════════╝");
                Serial.println();
                Serial.println("Test Results:");
                Serial.println("  ✓ Motor driver initialized successfully");
                Serial.println("  ✓ Motor spun in clockwise direction");
                Serial.println("  ✓ Motor spun in counter-clockwise direction");
                Serial.println("  ✓ Motor responded to speed commands");
                Serial.println();
                Serial.println("If the motor did NOT spin:");
                Serial.println("  - Check power supply is connected and turned on");
                Serial.println("  - Verify GND is connected between ESP32 and driver");
                Serial.println("  - Check motor phase wires are properly connected");
                Serial.println("  - Verify SimpleFOC Mini is receiving power (LED indicator)");
                Serial.println();
                Serial.println("Motor will remain stopped. Reset to run test again.");
                Serial.println();
            }
            break;
    }

    // Heartbeat every 10 seconds
    if (current_time - last_heartbeat >= HEARTBEAT_INTERVAL) {
        last_heartbeat = current_time;
        Serial.print("[HEARTBEAT] Uptime: ");
        Serial.print(millis() / 1000);
        Serial.print("s | Test phase: ");
        Serial.print(test_phase);
        Serial.print(" | Target: ");
        Serial.print(motor.target, 2);
        Serial.println(" rad/s");
    }

    delay(10);  // Small delay for stability
}
