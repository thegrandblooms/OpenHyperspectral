/**
 * Encoder Test - Continuous Data Streaming
 *
 * This test continuously reads and streams encoder data from the MT6701 14-bit
 * magnetic encoder over I2C. Useful for verifying encoder connectivity, accuracy,
 * and response to manual rotation.
 *
 * Test Objectives:
 * - Verify MT6701 encoder I2C communication
 * - Monitor encoder angle in real-time
 * - Calculate angular velocity from position changes
 * - Monitor magnetic field strength
 * - Test zero position calibration
 * - Detect and report communication errors
 *
 * Usage:
 * 1. Upload this sketch to the ESP32
 * 2. Open Serial Monitor at 115200 baud
 * 3. Manually rotate the motor shaft
 * 4. Observe position and velocity readings
 * 5. Send 'h' for help, 'p' to stop streaming
 *
 * Output Format:
 * [TIME] Pos: X.XXXX rad | Vel: X.XX rad/s | Raw: XXXXX | Field: OK
 */

#include <Arduino.h>
#include <Wire.h>
#include <MT6701.h>  // Use MT6701 library from firmware/libraries/MT6701

// Encoder pins (from motor_firmware config.h)
#define ENCODER_SDA      47   // MT6701 I2C SDA
#define ENCODER_SCL      48   // MT6701 I2C SCL
#define ENCODER_I2C_ADDR 0x06 // MT6701 I2C address
#define ENCODER_PPR      16384 // 2^14 = 16384 positions per revolution

// Test configuration
#define HEARTBEAT_INTERVAL 10000  // 10 seconds
#define SAMPLE_RATE_HZ     100    // 100 Hz sampling rate
#define SAMPLE_PERIOD_MS   (1000 / SAMPLE_RATE_HZ)

// Create encoder object with new MT6701 library
MT6701 encoder(ENCODER_I2C_ADDR);

// Test state
unsigned long last_heartbeat = 0;
unsigned long last_sample = 0;
unsigned long sample_count = 0;
float last_angle = 0;
float last_time = 0;
bool streaming = true;
String command_buffer = "";

// Statistics
float min_angle = 0;
float max_angle = 0;
float max_velocity = 0;
unsigned long error_count = 0;
unsigned long field_warnings = 0;

// Revolution tracking
int revolutions = 0;
float last_angle_for_rev = 0;

void printHelp() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    ENCODER TEST - COMMANDS                     ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("Commands:");
    Serial.println("  h, help    - Show this help menu");
    Serial.println("  s, start   - Start streaming encoder data");
    Serial.println("  p, stop    - Pause streaming");
    Serial.println("  r, reset   - Reset statistics");
    Serial.println("  i, info    - Show encoder information");
    Serial.println("  c, stats   - Show statistics");
    Serial.println("  z, zero    - Set current position as zero");
    Serial.println("  f, field   - Check magnetic field strength");
    Serial.println("  d, dir     - Toggle rotation direction");
    Serial.println();
}

void printInfo() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                   ENCODER INFORMATION                          ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.print("Encoder Model: MT6701 14-bit Magnetic Encoder\n");
    Serial.print("Resolution: ");
    Serial.print(ENCODER_PPR);
    Serial.println(" positions/revolution");
    Serial.print("I2C Address: 0x");
    Serial.println(ENCODER_I2C_ADDR, HEX);
    Serial.print("I2C Pins: SDA=GPIO");
    Serial.print(ENCODER_SDA);
    Serial.print(", SCL=GPIO");
    Serial.println(ENCODER_SCL);
    Serial.print("Angle Resolution: ");
    Serial.print(360.0 / ENCODER_PPR, 4);
    Serial.println(" degrees/count");
    Serial.print("Angle Resolution: ");
    Serial.print((2.0 * PI) / ENCODER_PPR, 6);
    Serial.println(" radians/count");
    Serial.print("Sample Rate: ");
    Serial.print(SAMPLE_RATE_HZ);
    Serial.println(" Hz");

    // Read configuration
    Serial.println("\n--- Current Configuration ---");
    Serial.print("Zero Position: ");
    Serial.print(encoder.readZeroPosition(), 4);
    Serial.println(" rad");
    Serial.print("Direction: ");
    Serial.println(encoder.isClockwise() ? "Clockwise" : "Counter-Clockwise");

    Serial.println();
}

void printStats() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                      STATISTICS                                ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.print("Total Samples: ");
    Serial.println(sample_count);
    Serial.print("Angle Range: ");
    Serial.print(min_angle, 4);
    Serial.print(" to ");
    Serial.print(max_angle, 4);
    Serial.println(" rad");
    Serial.print("Max Velocity: ");
    Serial.print(max_velocity, 2);
    Serial.println(" rad/s");
    Serial.print("Communication Errors: ");
    Serial.println(error_count);
    Serial.print("Field Warnings: ");
    Serial.println(field_warnings);
    Serial.print("Revolutions: ");
    Serial.println(revolutions);
    Serial.print("Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");
    Serial.println();
}

void checkFieldStrength() {
    uint8_t status = encoder.readFieldStatus();

    if (status == MT6701_FIELD_GOOD) {
        Serial.println("✓ Magnetic field strength is GOOD");
    } else {
        if (status & MT6701_FIELD_TOO_STRONG) {
            Serial.println("⚠ WARNING: Magnetic field TOO STRONG");
            Serial.println("  → Move magnet further from sensor");
        }
        if (status & MT6701_FIELD_TOO_WEAK) {
            Serial.println("⚠ WARNING: Magnetic field TOO WEAK");
            Serial.println("  → Move magnet closer to sensor");
            Serial.println("  → Check magnet is properly magnetized");
        }
    }
}

void resetStats() {
    sample_count = 0;
    min_angle = encoder.readAngleRadians();
    max_angle = min_angle;
    max_velocity = 0;
    error_count = 0;
    field_warnings = 0;
    revolutions = 0;
    Serial.println("Statistics reset!");
}

void setZero() {
    Serial.print("Setting current position as zero... ");
    if (encoder.setZeroToCurrent()) {
        Serial.println("Success!");
        Serial.print("New zero position: ");
        Serial.print(encoder.readZeroPosition(), 4);
        Serial.println(" rad");
    } else {
        Serial.println("Failed!");
    }
}

void toggleDirection() {
    bool current_dir = encoder.isClockwise();
    bool new_dir = !current_dir;

    Serial.print("Changing direction from ");
    Serial.print(current_dir ? "Clockwise" : "Counter-Clockwise");
    Serial.print(" to ");
    Serial.print(new_dir ? "Clockwise" : "Counter-Clockwise");
    Serial.print("... ");

    if (encoder.setDirection(new_dir)) {
        Serial.println("Success!");
    } else {
        Serial.println("Failed!");
    }
}

void processCommand() {
    command_buffer.trim();
    command_buffer.toLowerCase();

    if (command_buffer.length() == 0) return;

    if (command_buffer == "h" || command_buffer == "help") {
        printHelp();
    }
    else if (command_buffer == "s" || command_buffer == "start") {
        streaming = true;
        Serial.println("Streaming started!");
    }
    else if (command_buffer == "p" || command_buffer == "stop") {
        streaming = false;
        Serial.println("Streaming paused. Send 's' to resume.");
    }
    else if (command_buffer == "r" || command_buffer == "reset") {
        resetStats();
    }
    else if (command_buffer == "i" || command_buffer == "info") {
        printInfo();
    }
    else if (command_buffer == "c" || command_buffer == "stats") {
        printStats();
    }
    else if (command_buffer == "z" || command_buffer == "zero") {
        setZero();
    }
    else if (command_buffer == "f" || command_buffer == "field") {
        checkFieldStrength();
    }
    else if (command_buffer == "d" || command_buffer == "dir") {
        toggleDirection();
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
    Serial.println("║           ENCODER TEST - CONTINUOUS DATA STREAMING            ║");
    Serial.println("║              Using Native MT6701 Library                       ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Test Configuration:");
    Serial.print("  - Encoder: MT6701 14-bit (");
    Serial.print(ENCODER_PPR);
    Serial.println(" PPR)");
    Serial.print("  - I2C Address: 0x");
    Serial.println(ENCODER_I2C_ADDR, HEX);
    Serial.print("  - I2C Pins: SDA=GPIO");
    Serial.print(ENCODER_SDA);
    Serial.print(", SCL=GPIO");
    Serial.println(ENCODER_SCL);
    Serial.print("  - Sample Rate: ");
    Serial.print(SAMPLE_RATE_HZ);
    Serial.println(" Hz");
    Serial.println();

    // Initialize I2C
    Serial.println("[INIT] Initializing I2C bus...");
    Wire.begin(ENCODER_SDA, ENCODER_SCL);
    Wire.setClock(400000);  // 400kHz fast mode
    Serial.println("[OK]   I2C initialized");

    // Initialize encoder
    Serial.print("[INIT] Initializing MT6701 encoder... ");
    if (encoder.begin(&Wire)) {
        Serial.println("Success!");
    } else {
        Serial.println("FAILED!");
        Serial.println();
        Serial.println("[ERROR] MT6701 encoder not found!");
        Serial.println();
        Serial.println("Troubleshooting:");
        Serial.println("  1. Check wiring:");
        Serial.println("     - 3.3V → MT6701 VDD");
        Serial.println("     - GND  → MT6701 GND");
        Serial.println("     - GPIO47 → MT6701 SDA");
        Serial.println("     - GPIO48 → MT6701 SCL");
        Serial.println("  2. Verify encoder has power (check voltage at VDD pin)");
        Serial.println("  3. Check for loose connections");
        Serial.println("  4. Verify I2C address is correct (should be 0x06)");
        Serial.println();
        Serial.println("System will continue anyway for debugging...");
        delay(2000);
    }

    // Check magnetic field
    Serial.print("[CHECK] Magnetic field status... ");
    checkFieldStrength();

    // Initial reading
    float initial_angle = encoder.readAngleRadians();
    Serial.print("[INFO] Initial angle: ");
    Serial.print(initial_angle, 4);
    Serial.print(" rad (");
    Serial.print(initial_angle * 180.0 / PI, 2);
    Serial.println(" degrees)");

    Serial.println();
    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                   STREAMING ENCODER DATA...                    ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Rotate the motor shaft manually to see encoder response!");
    Serial.println("Type 'h' for help, 'p' to pause, 'z' to set zero, 'f' for field check");
    Serial.println();
    Serial.println("Format: [TIME] Pos: X.XXXX rad | Vel: X.XX rad/s | Raw: XXXXX | Field: STATUS");
    Serial.println("─────────────────────────────────────────────────────────────────");

    last_angle = initial_angle;
    last_time = millis() / 1000.0;
    min_angle = initial_angle;
    max_angle = initial_angle;
    last_angle_for_rev = initial_angle;
    last_heartbeat = millis();
    last_sample = millis();
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

    // Sample encoder at specified rate
    if (current_time - last_sample >= SAMPLE_PERIOD_MS && streaming) {
        last_sample = current_time;

        // Read encoder
        uint16_t raw = encoder.readRawAngle();
        float angle = encoder.readAngleRadians();
        float time_now = current_time / 1000.0;
        float dt = time_now - last_time;

        // Calculate velocity (handle wraparound)
        float angle_diff = angle - last_angle;
        if (angle_diff > PI) {
            angle_diff -= 2 * PI;
            revolutions--;  // Crossed zero backwards
        } else if (angle_diff < -PI) {
            angle_diff += 2 * PI;
            revolutions++;  // Crossed zero forwards
        }
        float velocity = angle_diff / dt;

        // Check field status
        uint8_t field_status = encoder.readFieldStatus();
        const char* field_str;
        if (field_status == MT6701_FIELD_GOOD) {
            field_str = "OK";
        } else if (field_status & MT6701_FIELD_TOO_STRONG) {
            field_str = "TOO STRONG";
            field_warnings++;
        } else if (field_status & MT6701_FIELD_TOO_WEAK) {
            field_str = "TOO WEAK";
            field_warnings++;
        } else {
            field_str = "ERROR";
            error_count++;
        }

        // Update statistics
        sample_count++;
        if (angle < min_angle) min_angle = angle;
        if (angle > max_angle) max_angle = angle;
        if (abs(velocity) > max_velocity) max_velocity = abs(velocity);

        // Print data
        Serial.print("[");
        Serial.print(time_now, 2);
        Serial.print("s] Pos: ");
        Serial.print(angle, 4);
        Serial.print(" rad | Vel: ");
        if (velocity >= 0) Serial.print(" ");
        Serial.print(velocity, 2);
        Serial.print(" rad/s | Raw: ");
        if (raw < 10000) Serial.print(" ");
        if (raw < 1000) Serial.print(" ");
        if (raw < 100) Serial.print(" ");
        if (raw < 10) Serial.print(" ");
        Serial.print(raw);
        Serial.print(" | Rev: ");
        if (abs(revolutions) < 10) Serial.print(" ");
        Serial.print(revolutions);
        Serial.print(" | Field: ");
        Serial.println(field_str);

        // Update for next iteration
        last_angle = angle;
        last_time = time_now;
    }

    // Heartbeat every 10 seconds
    if (current_time - last_heartbeat >= HEARTBEAT_INTERVAL) {
        last_heartbeat = current_time;
        Serial.println();
        Serial.print("[HEARTBEAT] Uptime: ");
        Serial.print(current_time / 1000);
        Serial.print("s | Samples: ");
        Serial.print(sample_count);
        Serial.print(" | Errors: ");
        Serial.print(error_count);
        Serial.print(" | Field Warnings: ");
        Serial.print(field_warnings);
        Serial.print(" | Streaming: ");
        Serial.println(streaming ? "ON" : "OFF");
        Serial.println();
    }

    delay(1);  // Small delay for stability
}
