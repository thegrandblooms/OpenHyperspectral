/**
 * Driver Fault Detection Test
 *
 * This test verifies the SimpleFOC Mini motor driver can be properly enabled/
 * disabled and checks for fault conditions. Tests the enable pin and optional
 * fault detection pin functionality.
 *
 * Test Objectives:
 * - Verify driver enable/disable functionality
 * - Test fault pin monitoring (nFT)
 * - Validate driver initialization
 * - Check power supply connectivity
 * - Test driver protection features
 *
 * Usage:
 * 1. Upload this sketch to the ESP32
 * 2. Open Serial Monitor at 115200 baud
 * 3. Test will cycle through enable/disable sequences
 * 4. Send commands to manually control driver
 */

#include <Arduino.h>
#include <SimpleFOC.h>

// Motor driver pins (from motor_firmware config.h)
#define MOTOR_ENABLE     15   // Motor driver enable pin
#define MOTOR_PWM_A      13   // Phase A PWM
#define MOTOR_PWM_B      11   // Phase B PWM
#define MOTOR_PWM_C      12   // Phase C PWM
#define MOTOR_FAULT      14   // Fault detection (nFT, active LOW)
#define MOTOR_RESET      9    // Driver reset (nRT, active LOW)

// Motor configuration
#define POLE_PAIRS       7    // Mitoot 2804 motor: 7 pole pairs
#define VOLTAGE_PSU      12.0 // Power supply voltage

// Test configuration
#define HEARTBEAT_INTERVAL 10000  // 10 seconds

// Create driver object
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE);

// Test state
unsigned long last_heartbeat = 0;
unsigned long test_start_time = 0;
int test_phase = 0;
String command_buffer = "";
bool fault_detected = false;
unsigned long fault_count = 0;

void printHelp() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                DRIVER FAULT TEST - COMMANDS                    ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("Commands:");
    Serial.println("  h, help     - Show this help menu");
    Serial.println("  e, enable   - Enable motor driver");
    Serial.println("  d, disable  - Disable motor driver");
    Serial.println("  r, reset    - Reset motor driver (pulse nRT pin)");
    Serial.println("  s, status   - Show driver status");
    Serial.println("  f, fault    - Check fault pin");
    Serial.println("  a, auto     - Run automatic test sequence");
    Serial.println();
}

void printStatus() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                     DRIVER STATUS                              ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");

    // Read enable pin
    bool enable_state = digitalRead(MOTOR_ENABLE);
    Serial.print("Enable Pin (GPIO");
    Serial.print(MOTOR_ENABLE);
    Serial.print("): ");
    Serial.println(enable_state ? "HIGH (enabled)" : "LOW (disabled)");

    // Read fault pin if configured
    pinMode(MOTOR_FAULT, INPUT_PULLUP);
    bool fault_state = digitalRead(MOTOR_FAULT);
    Serial.print("Fault Pin (GPIO");
    Serial.print(MOTOR_FAULT);
    Serial.print("): ");
    Serial.print(fault_state ? "HIGH (OK)" : "LOW (FAULT!)");
    if (!fault_state) {
        Serial.println(" ⚠ FAULT DETECTED!");
    } else {
        Serial.println();
    }

    // Driver configuration
    Serial.print("Power Supply: ");
    Serial.print(VOLTAGE_PSU);
    Serial.println(" V");
    Serial.print("Voltage Limit: ");
    Serial.print(driver.voltage_limit);
    Serial.println(" V");

    Serial.print("Total Faults Detected: ");
    Serial.println(fault_count);

    Serial.println();
}

void checkFault() {
    pinMode(MOTOR_FAULT, INPUT_PULLUP);
    bool fault_state = digitalRead(MOTOR_FAULT);

    if (!fault_state && !fault_detected) {
        // New fault detected
        fault_detected = true;
        fault_count++;

        Serial.println();
        Serial.println("╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║                    ⚠ FAULT DETECTED! ⚠                        ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.println();
        Serial.println("Possible causes:");
        Serial.println("  • Overcurrent condition");
        Serial.println("  • Short circuit in motor phases");
        Serial.println("  • Over-temperature");
        Serial.println("  • Supply voltage too low/high");
        Serial.println("  • Driver malfunction");
        Serial.println();
        Serial.println("Recommended actions:");
        Serial.println("  1. Disable driver (send 'd')");
        Serial.println("  2. Check motor phase connections");
        Serial.println("  3. Verify power supply voltage");
        Serial.println("  4. Reset driver (send 'r')");
        Serial.println("  5. Check for mechanical obstructions");
        Serial.println();
    } else if (fault_state && fault_detected) {
        // Fault cleared
        fault_detected = false;
        Serial.println("[INFO] Fault condition cleared");
    }
}

void enableDriver() {
    Serial.println("[ACTION] Enabling motor driver...");
    driver.enable();
    delay(100);  // Allow driver to stabilize
    Serial.println("[OK]    Driver enabled");
    printStatus();
}

void disableDriver() {
    Serial.println("[ACTION] Disabling motor driver...");
    driver.disable();
    delay(100);
    Serial.println("[OK]    Driver disabled");
    printStatus();
}

void resetDriver() {
    Serial.println("[ACTION] Resetting motor driver...");
    Serial.println("         Pulsing nRT pin LOW for 10ms...");

    pinMode(MOTOR_RESET, OUTPUT);
    digitalWrite(MOTOR_RESET, LOW);  // Active LOW reset
    delay(10);
    digitalWrite(MOTOR_RESET, HIGH);
    delay(100);  // Allow driver to recover

    Serial.println("[OK]    Driver reset complete");
    printStatus();
}

void runAutoTest() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║              AUTOMATIC DRIVER TEST SEQUENCE                    ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();

    // Test 1: Disable driver
    Serial.println("Test 1: Disable driver");
    disableDriver();
    delay(2000);

    // Test 2: Enable driver
    Serial.println("\nTest 2: Enable driver");
    enableDriver();
    delay(2000);

    // Test 3: Disable again
    Serial.println("\nTest 3: Disable driver again");
    disableDriver();
    delay(2000);

    // Test 4: Reset driver
    Serial.println("\nTest 4: Reset driver");
    resetDriver();
    delay(2000);

    // Test 5: Enable after reset
    Serial.println("\nTest 5: Enable driver after reset");
    enableDriver();
    delay(2000);

    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                   AUTO TEST COMPLETE!                          ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    printStatus();
}

void processCommand() {
    command_buffer.trim();
    command_buffer.toLowerCase();

    if (command_buffer.length() == 0) return;

    if (command_buffer == "h" || command_buffer == "help") {
        printHelp();
    }
    else if (command_buffer == "e" || command_buffer == "enable") {
        enableDriver();
    }
    else if (command_buffer == "d" || command_buffer == "disable") {
        disableDriver();
    }
    else if (command_buffer == "r" || command_buffer == "reset") {
        resetDriver();
    }
    else if (command_buffer == "s" || command_buffer == "status") {
        printStatus();
    }
    else if (command_buffer == "f" || command_buffer == "fault") {
        checkFault();
        printStatus();
    }
    else if (command_buffer == "a" || command_buffer == "auto") {
        runAutoTest();
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
    Serial.println("║              DRIVER FAULT DETECTION TEST                       ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Test Configuration:");
    Serial.print("  - Driver: SimpleFOC Mini v1 (DRV8313)\n");
    Serial.print("  - Enable Pin: GPIO");
    Serial.println(MOTOR_ENABLE);
    Serial.print("  - Fault Pin: GPIO");
    Serial.print(MOTOR_FAULT);
    Serial.println(" (nFT, active LOW)");
    Serial.print("  - Reset Pin: GPIO");
    Serial.print(MOTOR_RESET);
    Serial.println(" (nRT, active LOW)");
    Serial.print("  - Power Supply: ");
    Serial.print(VOLTAGE_PSU);
    Serial.println(" V");
    Serial.println();

    // Configure fault and reset pins
    pinMode(MOTOR_FAULT, INPUT_PULLUP);
    pinMode(MOTOR_RESET, OUTPUT);
    digitalWrite(MOTOR_RESET, HIGH);  // Keep reset HIGH (inactive)

    // Initialize driver
    Serial.println("[INIT] Initializing motor driver...");
    driver.voltage_power_supply = VOLTAGE_PSU;
    driver.voltage_limit = VOLTAGE_PSU / 2;

    if (!driver.init()) {
        Serial.println("[ERROR] Driver initialization failed!");
        Serial.println();
        Serial.println("Possible issues:");
        Serial.println("  - Power supply not connected");
        Serial.println("  - Incorrect wiring");
        Serial.println("  - Driver malfunction");
        Serial.println();
        while (1) {
            delay(1000);
        }
    }
    Serial.println("[OK]   Driver initialized successfully");
    Serial.println();

    // Check initial status
    printStatus();

    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                      TEST READY!                               ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Type 'h' for help, 'a' to run automatic test");
    Serial.println();

    last_heartbeat = millis();
    test_start_time = millis();
}

void loop() {
    unsigned long current_time = millis();

    // Check for serial commands
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            processCommand();
        } else {
            command_buffer += c;
        }
    }

    // Continuously monitor fault pin
    checkFault();

    // Heartbeat every 10 seconds
    if (current_time - last_heartbeat >= HEARTBEAT_INTERVAL) {
        last_heartbeat = current_time;

        Serial.print("[HEARTBEAT] Uptime: ");
        Serial.print(current_time / 1000);
        Serial.print("s | Enable: ");
        Serial.print(digitalRead(MOTOR_ENABLE) ? "ON" : "OFF");
        Serial.print(" | Fault: ");
        Serial.print(fault_detected ? "ACTIVE" : "OK");
        Serial.print(" | Total Faults: ");
        Serial.println(fault_count);
    }

    delay(100);  // Check fault pin at 10 Hz
}
