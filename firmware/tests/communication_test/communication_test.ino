/**
 * Communication Protocol Test
 *
 * This test verifies the binary serial communication protocol used between
 * the ESP32 and PC. Tests the SerialTransfer library and command/response
 * handling without requiring motor hardware.
 *
 * Test Objectives:
 * - Verify SerialTransfer library functionality
 * - Test PING/PONG communication
 * - Validate command parsing
 * - Measure communication latency
 * - Test error handling
 *
 * Usage:
 * 1. Upload this sketch to the ESP32
 * 2. Run the Python test script or use Serial Monitor
 * 3. Send commands and verify responses
 *
 * Python Test Example:
 *   python motor_control/controller.py /dev/ttyUSB0
 */

#include <Arduino.h>
#include "SerialTransfer.h"

// Serial configuration
#define SERIAL_BAUD      115200
#define HEARTBEAT_INTERVAL 10000  // 10 seconds

// Command IDs (from commands.h)
#define CMD_PING         0x0A
#define RSP_PING         0x86
#define RSP_OK           0x81
#define RSP_ERROR        0x82

// Error codes
#define ERR_INVALID_COMMAND    0x01
#define ERR_INVALID_PARAMETER  0x02
#define ERR_COMMAND_FAILED     0x03

// Structures for commands
struct PingCommand {
    uint32_t echo_value;
};

struct PingResponse {
    uint32_t echo_value;
};

// Create SerialTransfer object
SerialTransfer myTransfer;

// Test statistics
unsigned long packets_received = 0;
unsigned long packets_sent = 0;
unsigned long errors = 0;
unsigned long last_heartbeat = 0;
unsigned long total_latency_us = 0;
unsigned long latency_samples = 0;

void sendPingResponse(uint32_t echo_value) {
    PingResponse response;
    response.echo_value = echo_value;

    uint16_t send_size = myTransfer.txObj(response, 0);
    myTransfer.sendData(send_size);
    packets_sent++;
}

void sendOkResponse(uint8_t command_id) {
    uint8_t response[2];
    response[0] = RSP_OK;
    response[1] = command_id;

    uint16_t send_size = myTransfer.txObj(response, 0);
    myTransfer.sendData(send_size);
    packets_sent++;
}

void sendErrorResponse(uint8_t command_id, uint8_t error_code) {
    uint8_t response[3];
    response[0] = RSP_ERROR;
    response[1] = command_id;
    response[2] = error_code;

    uint16_t send_size = myTransfer.txObj(response, 0);
    myTransfer.sendData(send_size);
    packets_sent++;
    errors++;
}

void processCommand() {
    // Get command ID (first byte)
    uint8_t cmd_id;
    myTransfer.rxObj(cmd_id, 0);

    unsigned long start_time = micros();

    switch (cmd_id) {
        case CMD_PING: {
            PingCommand cmd;
            if (myTransfer.rxObj(cmd, 1) == sizeof(PingCommand)) {
                unsigned long latency = micros() - start_time;
                total_latency_us += latency;
                latency_samples++;

                sendPingResponse(cmd.echo_value);

                Serial.print("[PING] Echo: 0x");
                Serial.print(cmd.echo_value, HEX);
                Serial.print(" | Latency: ");
                Serial.print(latency);
                Serial.println(" μs");
            } else {
                sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
                Serial.println("[ERROR] Invalid PING packet");
            }
            break;
        }

        default:
            sendErrorResponse(cmd_id, ERR_INVALID_COMMAND);
            Serial.print("[ERROR] Unknown command: 0x");
            Serial.println(cmd_id, HEX);
            break;
    }
}

void printStatistics() {
    Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                  COMMUNICATION STATISTICS                      ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.print("Packets Received: ");
    Serial.println(packets_received);
    Serial.print("Packets Sent: ");
    Serial.println(packets_sent);
    Serial.print("Errors: ");
    Serial.println(errors);

    if (latency_samples > 0) {
        unsigned long avg_latency = total_latency_us / latency_samples;
        Serial.print("Average Latency: ");
        Serial.print(avg_latency);
        Serial.println(" μs");
    } else {
        Serial.println("Average Latency: N/A (no samples)");
    }

    float success_rate = packets_received > 0 ?
        (float)(packets_received - errors) / packets_received * 100.0 : 100.0;
    Serial.print("Success Rate: ");
    Serial.print(success_rate, 2);
    Serial.println("%");

    Serial.print("Uptime: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");
    Serial.println();
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000) {
        delay(10);
    }

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║           COMMUNICATION PROTOCOL TEST                          ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Test Configuration:");
    Serial.print("  - Serial Baud Rate: ");
    Serial.println(SERIAL_BAUD);
    Serial.println("  - Protocol: Binary (SerialTransfer)");
    Serial.println("  - Supported Commands: PING");
    Serial.println();

    // Initialize SerialTransfer
    Serial.println("[INIT] Initializing SerialTransfer...");
    myTransfer.begin(Serial);
    Serial.println("[OK]   SerialTransfer initialized");
    Serial.println();

    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                      TEST READY!                               ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Waiting for commands from PC...");
    Serial.println();
    Serial.println("Python test command:");
    Serial.println("  python motor_control/controller.py /dev/ttyUSB0");
    Serial.println();
    Serial.println("Or send binary PING commands:");
    Serial.println("  Command ID: 0x0A");
    Serial.println("  Payload: 4-byte echo value");
    Serial.println();

    last_heartbeat = millis();
}

void loop() {
    // Check for incoming packets
    if (myTransfer.available()) {
        packets_received++;
        processCommand();
    }

    // Check for SerialTransfer errors
    if (myTransfer.status < 0) {
        errors++;
        Serial.print("[ERROR] SerialTransfer error: ");
        Serial.println(myTransfer.status);
    }

    // Heartbeat every 10 seconds
    unsigned long current_time = millis();
    if (current_time - last_heartbeat >= HEARTBEAT_INTERVAL) {
        last_heartbeat = current_time;

        Serial.print("[HEARTBEAT] Uptime: ");
        Serial.print(current_time / 1000);
        Serial.print("s | RX: ");
        Serial.print(packets_received);
        Serial.print(" | TX: ");
        Serial.print(packets_sent);
        Serial.print(" | Errors: ");
        Serial.println(errors);
    }

    // Check for serial input (for statistics command)
    if (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 's' || c == 'S') {
            printStatistics();
        }
    }
}
