/**
 * I2C Scanner Test
 *
 * This test scans the I2C bus for connected devices and identifies known
 * devices like the MT6701 encoder. Useful for debugging I2C connectivity
 * issues and verifying hardware connections.
 *
 * Test Objectives:
 * - Scan I2C bus for all devices (addresses 0x01 to 0x7F)
 * - Identify known devices (MT6701, CST816D, QMI8658)
 * - Verify I2C bus functionality
 * - Test different I2C clock speeds
 *
 * Usage:
 * 1. Upload this sketch to the ESP32
 * 2. Open Serial Monitor at 115200 baud
 * 3. Scanner will run automatically on startup
 * 4. Send 's' to scan again, 'f' for fast scan, 'h' for help
 */

#include <Arduino.h>
#include <Wire.h>

// I2C configuration (from motor_firmware config.h)
#define ENCODER_SDA      47   // MT6701 I2C SDA
#define ENCODER_SCL      48   // MT6701 I2C SCL

// Test configuration
#define HEARTBEAT_INTERVAL 10000  // 10 seconds
#define I2C_STANDARD_SPEED 100000  // 100 kHz
#define I2C_FAST_SPEED     400000  // 400 kHz

// Known device database
struct I2CDevice {
    byte address;
    const char* name;
    const char* description;
};

const I2CDevice knownDevices[] = {
    {0x06, "MT6701", "14-bit Magnetic Encoder"},
    {0x15, "CST816D", "Touch Controller (on ESP32-S3-Touch-LCD-2)"},
    {0x6B, "QMI8658", "6-axis IMU (on ESP32-S3-Touch-LCD-2)"},
    // Add more known devices here
};

const int numKnownDevices = sizeof(knownDevices) / sizeof(knownDevices[0]);

// Test state
unsigned long last_heartbeat = 0;
String command_buffer = "";
uint32_t current_speed = I2C_FAST_SPEED;

void printHelp() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                    I2C SCANNER - COMMANDS                      ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("Commands:");
    Serial.println("  h, help    - Show this help menu");
    Serial.println("  s, scan    - Scan I2C bus");
    Serial.println("  f, fast    - Set fast mode (400 kHz)");
    Serial.println("  n, normal  - Set standard mode (100 kHz)");
    Serial.println("  i, info    - Show I2C configuration");
    Serial.println("  d, detect  - Detect specific device address");
    Serial.println();
}

void printI2CInfo() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                   I2C CONFIGURATION                            ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.print("I2C Pins: SDA=GPIO");
    Serial.print(ENCODER_SDA);
    Serial.print(", SCL=GPIO");
    Serial.println(ENCODER_SCL);
    Serial.print("Clock Speed: ");
    Serial.print(current_speed / 1000);
    Serial.print(" kHz (");
    Serial.print(current_speed == I2C_FAST_SPEED ? "Fast" : "Standard");
    Serial.println(" mode)");
    Serial.println();
    Serial.println("Known Devices:");
    for (int i = 0; i < numKnownDevices; i++) {
        Serial.print("  0x");
        if (knownDevices[i].address < 16) Serial.print("0");
        Serial.print(knownDevices[i].address, HEX);
        Serial.print(" - ");
        Serial.print(knownDevices[i].name);
        Serial.print(" (");
        Serial.print(knownDevices[i].description);
        Serial.println(")");
    }
    Serial.println();
}

const char* getDeviceName(byte address) {
    for (int i = 0; i < numKnownDevices; i++) {
        if (knownDevices[i].address == address) {
            return knownDevices[i].name;
        }
    }
    return nullptr;
}

const char* getDeviceDescription(byte address) {
    for (int i = 0; i < numKnownDevices; i++) {
        if (knownDevices[i].address == address) {
            return knownDevices[i].description;
        }
    }
    return nullptr;
}

void scanI2CBus() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                      I2C BUS SCANNER                           ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.print("Scanning I2C bus (SDA=GPIO");
    Serial.print(ENCODER_SDA);
    Serial.print(", SCL=GPIO");
    Serial.print(ENCODER_SCL);
    Serial.print(", Speed=");
    Serial.print(current_speed / 1000);
    Serial.println(" kHz)...\n");

    byte count = 0;
    byte errors = 0;
    unsigned long start_time = millis();

    Serial.println("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
    Serial.println("     ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─  ─");

    for (byte row = 0; row < 8; row++) {
        Serial.print(row, HEX);
        Serial.print("0: ");

        for (byte col = 0; col < 16; col++) {
            byte address = (row << 4) | col;

            // Skip reserved addresses
            if (address < 0x03 || address > 0x77) {
                Serial.print("   ");
                continue;
            }

            Wire.beginTransmission(address);
            byte error = Wire.endTransmission();

            if (error == 0) {
                // Device found
                if (address < 16) Serial.print("0");
                Serial.print(address, HEX);
                Serial.print(" ");
                count++;
            } else if (error == 4) {
                // Unknown error
                Serial.print("!! ");
                errors++;
            } else {
                // No device
                Serial.print("-- ");
            }
        }
        Serial.println();
    }

    unsigned long scan_time = millis() - start_time;

    Serial.println();
    Serial.println("─────────────────────────────────────────────────────────────────");

    if (count == 0 && errors == 0) {
        Serial.println("⚠ WARNING: No I2C devices found!");
        Serial.println();
        Serial.println("Troubleshooting steps:");
        Serial.println("  1. Check wiring:");
        Serial.println("     - 3.3V → Device VDD");
        Serial.println("     - GND  → Device GND");
        Serial.println("     - GPIO47 → Device SDA");
        Serial.println("     - GPIO48 → Device SCL");
        Serial.println("  2. Verify device has power");
        Serial.println("  3. Check for loose connections");
        Serial.println("  4. Try different I2C speed (send 'n' for standard mode)");
        Serial.println("  5. Verify pull-up resistors are present (2.2kΩ - 10kΩ)");
    } else {
        Serial.print("Found ");
        Serial.print(count);
        Serial.print(" device(s)");
        if (errors > 0) {
            Serial.print(" (");
            Serial.print(errors);
            Serial.print(" error(s))");
        }
        Serial.println();
        Serial.println();

        // List found devices with details
        Serial.println("Detected Devices:");
        for (byte address = 0x03; address <= 0x77; address++) {
            Wire.beginTransmission(address);
            byte error = Wire.endTransmission();

            if (error == 0) {
                Serial.print("  • 0x");
                if (address < 16) Serial.print("0");
                Serial.print(address, HEX);

                const char* name = getDeviceName(address);
                const char* desc = getDeviceDescription(address);

                if (name != nullptr) {
                    Serial.print(" - ");
                    Serial.print(name);
                    if (desc != nullptr) {
                        Serial.print(" (");
                        Serial.print(desc);
                        Serial.print(")");
                    }

                    // Special check for MT6701
                    if (address == 0x06) {
                        Serial.print(" ✓ ENCODER DETECTED!");
                    }
                } else {
                    Serial.print(" - Unknown device");
                }
                Serial.println();
            }
        }
    }

    Serial.println();
    Serial.print("Scan completed in ");
    Serial.print(scan_time);
    Serial.println(" ms");
    Serial.println();
}

void detectDevice() {
    Serial.println();
    Serial.print("Enter device address (hex, e.g., '06' for 0x06): ");

    // Wait for input with timeout
    unsigned long timeout = millis() + 10000;
    String input = "";

    while (millis() < timeout) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                break;
            }
            input += c;
        }
        delay(10);
    }

    if (input.length() == 0) {
        Serial.println("\nTimeout - no input received");
        return;
    }

    Serial.println(input);

    // Parse hex address
    byte address = strtol(input.c_str(), NULL, 16);

    if (address < 0x03 || address > 0x77) {
        Serial.println("Invalid address! Must be between 0x03 and 0x77");
        return;
    }

    Serial.print("Checking for device at 0x");
    if (address < 16) Serial.print("0");
    Serial.print(address, HEX);
    Serial.println("...");

    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
        Serial.println("✓ Device found!");

        const char* name = getDeviceName(address);
        if (name != nullptr) {
            Serial.print("  Identified as: ");
            Serial.println(name);
        }
    } else {
        Serial.println("✗ No device found at this address");
    }
    Serial.println();
}

void processCommand() {
    command_buffer.trim();
    command_buffer.toLowerCase();

    if (command_buffer.length() == 0) return;

    if (command_buffer == "h" || command_buffer == "help") {
        printHelp();
    }
    else if (command_buffer == "s" || command_buffer == "scan") {
        scanI2CBus();
    }
    else if (command_buffer == "f" || command_buffer == "fast") {
        current_speed = I2C_FAST_SPEED;
        Wire.setClock(current_speed);
        Serial.println("I2C speed set to Fast mode (400 kHz)");
    }
    else if (command_buffer == "n" || command_buffer == "normal") {
        current_speed = I2C_STANDARD_SPEED;
        Wire.setClock(current_speed);
        Serial.println("I2C speed set to Standard mode (100 kHz)");
    }
    else if (command_buffer == "i" || command_buffer == "info") {
        printI2CInfo();
    }
    else if (command_buffer == "d" || command_buffer == "detect") {
        detectDevice();
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
    Serial.println("║                     I2C BUS SCANNER TEST                       ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();

    // Initialize I2C
    Serial.println("[INIT] Initializing I2C bus...");
    Wire.begin(ENCODER_SDA, ENCODER_SCL);
    Wire.setClock(current_speed);
    Serial.println("[OK]   I2C initialized");
    Serial.println();

    printI2CInfo();

    Serial.println("Starting initial scan...");
    delay(500);

    // Run initial scan
    scanI2CBus();

    Serial.println("Type 'h' for help, 's' to scan again");
    Serial.println();

    last_heartbeat = millis();
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

    // Heartbeat every 10 seconds
    if (current_time - last_heartbeat >= HEARTBEAT_INTERVAL) {
        last_heartbeat = current_time;
        Serial.print("[HEARTBEAT] Uptime: ");
        Serial.print(current_time / 1000);
        Serial.println("s | Type 's' to scan I2C bus");
    }

    delay(100);  // Reduce CPU usage
}
