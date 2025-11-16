/**
 * OpenHyperspectral Motor Controller Firmware
 *
 * This firmware runs on an ESP32-S3-Touch-LCD-2 (Waveshare) and provides:
 * - BLDC motor control using SimpleFOC
 * - USB serial communication for PC synchronization
 * - Position-based triggering for camera synchronization
 * - Display interface for status and manual control
 *
 * Based on:
 * - HyperMicro serial communication protocol
 * - ESP32 motor control architecture with SimpleFOC
 *
 * KEY ARCHITECTURE NOTES:
 * - Uses MT6701 absolute magnetic encoder as TRUTH SOURCE for position
 * - SimpleFOC handles motor control (FOC algorithm) but we use encoder for position checks
 * - All position verification uses getAbsolutePositionDeg() not getCurrentPositionDeg()
 */

#include <Arduino.h>
#include "config.h"
#include "commands.h"
#include "communication.h"
#include "motor_control.h"
#include "debug_globals.h"
#include "motor_tests.h"

//=============================================================================
// GLOBAL DEBUG FLAGS (Runtime toggleable)
//=============================================================================
// These can be changed at runtime using the "debug <flag> <0/1>" command
bool g_debug_serial = DEBUG_SERIAL_DEFAULT;
bool g_debug_motor = DEBUG_MOTOR_DEFAULT;
bool g_debug_comm = DEBUG_COMM_DEFAULT;
bool g_debug_heartbeat = DEBUG_HEARTBEAT_DEFAULT;

//=============================================================================
// GLOBAL OBJECTS
//=============================================================================

CommunicationManager comm;
MotorController motorControl;

// State tracking
uint16_t current_sequence_id = 0;
unsigned long last_update_time = 0;
unsigned long last_status_print = 0;
unsigned long last_heartbeat = 0;

// Serial command buffer for interactive testing
String serialCommandBuffer = "";

//=============================================================================
// COMMAND PROCESSING
//=============================================================================

void processCommand() {
    uint8_t cmd_id = comm.getCommandId();

    switch (cmd_id) {
        case CMD_MOVE_TO: {
            MoveToCommand cmd;
            if (comm.getCommandData(&cmd)) {
                current_sequence_id = cmd.sequence_id;
                motorControl.moveToPosition(cmd.position);
                comm.sendOkResponse(cmd_id);

                if (g_debug_serial) {
                    Serial.print("MOVE_TO: position=");
                    Serial.print(cmd.position);
                    Serial.print(", seq=");
                    Serial.println(cmd.sequence_id);
                }
            } else {
                comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            }
            break;
        }

        case CMD_SET_SPEED: {
            SetSpeedCommand cmd;
            if (comm.getCommandData(&cmd)) {
                motorControl.setVelocity(cmd.velocity);
                comm.sendOkResponse(cmd_id);

                if (g_debug_serial) {
                    Serial.print("SET_SPEED: ");
                    Serial.println(cmd.velocity);
                }
            } else {
                comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            }
            break;
        }

        case CMD_SET_ACCEL: {
            SetAccelCommand cmd;
            if (comm.getCommandData(&cmd)) {
                motorControl.setAcceleration(cmd.acceleration);
                comm.sendOkResponse(cmd_id);

                if (g_debug_serial) {
                    Serial.print("SET_ACCEL: ");
                    Serial.println(cmd.acceleration);
                }
            } else {
                comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            }
            break;
        }

        case CMD_STOP: {
            motorControl.stop();
            comm.sendOkResponse(cmd_id);

            if (g_debug_serial) {
                Serial.println("STOP");
            }
            break;
        }

        case CMD_HOME: {
            motorControl.setHome();
            comm.sendOkResponse(cmd_id);

            if (g_debug_serial) {
                Serial.println("HOME");
            }
            break;
        }

        case CMD_ENABLE: {
            motorControl.enable();
            comm.sendOkResponse(cmd_id);

            if (g_debug_serial) {
                Serial.println("ENABLE");
            }
            break;
        }

        case CMD_DISABLE: {
            motorControl.disable();
            comm.sendOkResponse(cmd_id);

            if (g_debug_serial) {
                Serial.println("DISABLE");
            }
            break;
        }

        case CMD_GET_STATUS: {
            // CRITICAL: Use ABSOLUTE ENCODER for position reporting
            motorControl.updateEncoder();  // Force fresh encoder read
            float position = motorControl.getAbsolutePositionDeg();  // ABSOLUTE ENCODER (TRUTH)
            float velocity = motorControl.getCurrentVelocityDegPerSec();
            float current = motorControl.getCurrent();
            float voltage = motorControl.getVoltage();
            uint8_t state = motorControl.getState();
            uint8_t control_mode = motorControl.getControlMode();
            bool enabled = motorControl.isEnabled();
            bool calibrated = motorControl.isCalibrated();

            comm.sendStatus(state, control_mode, position, velocity,
                          current, voltage, enabled, calibrated);

            if (g_debug_serial) {
                Serial.print("STATUS - Encoder pos: ");
                Serial.print(position, 2);
                Serial.println("°");
            }
            break;
        }

        case CMD_PING: {
            PingCommand cmd;
            if (comm.getCommandData(&cmd)) {
                comm.sendPingResponse(cmd.echo_value);

                if (g_debug_serial) {
                    Serial.print("PING: 0x");
                    Serial.println(cmd.echo_value, HEX);
                }
            } else {
                comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            }
            break;
        }

        case CMD_SET_MODE: {
            uint8_t mode;
            if (comm.getCommandData(&mode)) {
                if (mode <= MODE_TORQUE) {
                    motorControl.setControlMode(mode);
                    comm.sendOkResponse(cmd_id);

                    if (g_debug_serial) {
                        Serial.print("SET_MODE: ");
                        Serial.println(mode);
                    }
                } else {
                    comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
                }
            } else {
                comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            }
            break;
        }

        case CMD_SET_CURRENT_LIMIT: {
            SetCurrentLimitCommand cmd;
            if (comm.getCommandData(&cmd)) {
                motorControl.setCurrentLimit(cmd.current_limit);
                comm.sendOkResponse(cmd_id);

                if (g_debug_serial) {
                    Serial.print("SET_CURRENT_LIMIT: ");
                    Serial.println(cmd.current_limit);
                }
            } else {
                comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            }
            break;
        }

        case CMD_CALIBRATE: {
            if (g_debug_serial) {
                Serial.println("CALIBRATE");
            }

            // Run calibration
            bool success = motorControl.calibrate();

            if (success) {
                comm.sendOkResponse(cmd_id);
            } else {
                comm.sendErrorResponse(cmd_id, ERR_COMMAND_FAILED);
            }
            break;
        }

        case CMD_SET_PID: {
            SetPIDCommand cmd;
            if (comm.getCommandData(&cmd)) {
                switch (cmd.controller_type) {
                    case 0:  // Position
                        motorControl.setPositionPID(cmd.p, cmd.i, cmd.d, cmd.ramp);
                        break;
                    case 1:  // Velocity
                        motorControl.setVelocityPID(cmd.p, cmd.i, cmd.d, cmd.ramp);
                        break;
                    case 2:  // Current
                        motorControl.setCurrentPID(cmd.p, cmd.i, cmd.d, cmd.ramp);
                        break;
                    default:
                        comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
                        return;
                }

                comm.sendOkResponse(cmd_id);

                if (g_debug_serial) {
                    Serial.print("SET_PID: type=");
                    Serial.println(cmd.controller_type);
                }
            } else {
                comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            }
            break;
        }

        default:
            comm.sendErrorResponse(cmd_id, ERR_INVALID_COMMAND);

            if (g_debug_serial) {
                Serial.print("Unknown command: 0x");
                Serial.println(cmd_id, HEX);
            }
            break;
    }
}

//=============================================================================
// POSITION NOTIFICATION
//=============================================================================

void checkPositionReached() {
    static bool last_at_target = false;
    bool at_target = motorControl.isAtTarget();  // Uses ABSOLUTE ENCODER internally

    // Detect transition to target reached
    if (at_target && !last_at_target) {
        // Report ABSOLUTE ENCODER position (truth)
        motorControl.updateEncoder();
        float position = motorControl.getAbsolutePositionDeg();  // ABSOLUTE ENCODER
        comm.sendPositionReached(current_sequence_id, position);

        if (g_debug_serial) {
            Serial.print("Position reached notification sent: seq=");
            Serial.print(current_sequence_id);
            Serial.print(", encoder pos=");
            Serial.print(position, 2);
            Serial.println("°");
        }
    }

    last_at_target = at_target;
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
// INTERACTIVE SERIAL COMMANDS (for testing)
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
    Serial.println("  m <angle>      - Move to angle (e.g., 'm 3.14' for π radians)");
    Serial.println("  v <velocity>   - Set velocity (e.g., 'v 10.0' rad/s)");
    Serial.println("  a <accel>      - Set acceleration (e.g., 'a 5.0' rad/s²)");
    Serial.println("  mode <0-2>     - Set control mode (0=position, 1=velocity, 2=torque)");
    Serial.println("");
    Serial.println("Testing:");
    Serial.println("  test           - Run full test (calibration + PID tuning + motor test)");
    Serial.println("  motor_test     - Run motor movement test (auto-enables motor)");
    Serial.println("  encoder_test   - Test encoder readings (press any key to stop)");
    Serial.println("");
    Serial.println("Debug Control:");
    Serial.println("  debug              - Show current debug settings");
    Serial.println("  debug all <0/1>    - Toggle all debug output (0=off, 1=on)");
    Serial.println("  debug serial <0/1> - Toggle serial debug");
    Serial.println("  debug motor <0/1>  - Toggle motor debug");
    Serial.println("  debug comm <0/1>   - Toggle communication debug");
    Serial.println("  debug heart <0/1>  - Toggle heartbeat messages");
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

void printStatus() {
    Serial.println("\n--- Motor Status ---");

    // CRITICAL: Force fresh encoder read for accurate position
    motorControl.updateEncoder();

    // Show BOTH absolute encoder and SimpleFOC positions for comparison
    float abs_pos = motorControl.getAbsolutePositionDeg();     // ABSOLUTE ENCODER (MT6701 - TRUTH)
    float foc_pos = motorControl.getCurrentPositionDeg();      // SimpleFOC internal state

    Serial.print("Position (Absolute Encoder): ");
    Serial.print(abs_pos, 2);
    Serial.println("° ← TRUTH from MT6701");

    Serial.print("Position (SimpleFOC):        ");
    Serial.print(foc_pos, 2);
    Serial.print("° (diff: ");
    Serial.print(abs_pos - foc_pos, 2);
    Serial.println("°)");

    Serial.print("Velocity: ");
    Serial.print(motorControl.getCurrentVelocityDegPerSec(), 2);
    Serial.println("°/s");
    Serial.print("Current: ");
    Serial.print(motorControl.getCurrent(), 3);
    Serial.println(" A");
    Serial.print("Voltage: ");
    Serial.print(motorControl.getVoltage(), 2);
    Serial.println(" V");
    Serial.print("State: ");
    switch (motorControl.getState()) {
        case STATE_IDLE: Serial.println("IDLE"); break;
        case STATE_MOVING: Serial.println("MOVING"); break;
        case STATE_ERROR: Serial.println("ERROR"); break;
        case STATE_CALIBRATING: Serial.println("CALIBRATING"); break;
        default: Serial.println("UNKNOWN");
    }
    Serial.print("Control Mode: ");
    switch (motorControl.getControlMode()) {
        case MODE_POSITION: Serial.println("POSITION"); break;
        case MODE_VELOCITY: Serial.println("VELOCITY"); break;
        case MODE_TORQUE: Serial.println("TORQUE"); break;
        default: Serial.println("UNKNOWN");
    }
    Serial.print("Motor Enabled: ");
    Serial.println(motorControl.isEnabled() ? "YES" : "NO");
    Serial.print("Calibrated: ");
    Serial.println(motorControl.isCalibrated() ? "YES" : "NO");
    Serial.print("At Target: ");
    Serial.println(motorControl.isAtTarget() ? "YES" : "NO");

    // Raw encoder data
    Serial.print("Raw Encoder: ");
    Serial.print(motorControl.getRawEncoderCount());
    Serial.println(" (0-16383)");
    Serial.println();
}

// Test functions moved to motor_tests.h
// - runEncoderTest()
// - runMotorTest()
// - runFullTest()

void processSerialCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();

    if (cmd.length() == 0) return;

    Serial.print("\n> ");
    Serial.println(cmd);

    // Parse command and arguments
    int spaceIndex = cmd.indexOf(' ');
    String command = (spaceIndex > 0) ? cmd.substring(0, spaceIndex) : cmd;
    String args = (spaceIndex > 0) ? cmd.substring(spaceIndex + 1) : "";

    // Process commands
    if (command == "h" || command == "help") {
        printHelp();
    }
    else if (command == "s" || command == "status") {
        printStatus();
    }
    else if (command == "i" || command == "info") {
        printSystemInfo();
    }
    else if (command == "scan") {
        scanI2C();
    }
    else if (command == "e" || command == "enable") {
        Serial.println("Enabling motor...");
        motorControl.enable();
        Serial.println("Motor enabled!");
    }
    else if (command == "d" || command == "disable") {
        Serial.println("Disabling motor...");
        motorControl.disable();
        Serial.println("Motor disabled!");
    }
    else if (command == "c" || command == "calibrate") {
        Serial.println("Running motor calibration...");
        if (motorControl.calibrate()) {
            Serial.println("Calibration successful!");
        } else {
            Serial.println("Calibration failed!");
        }
    }
    else if (command == "p" || command == "pidtune") {
        if (!motorControl.isCalibrated()) {
            Serial.println("Error: Motor must be calibrated first!");
            Serial.println("Run 'calibrate' or 'c' first.");
        } else {
            Serial.println("Running PID auto-tuning...");
            Serial.println("This may take 2-5 minutes.");
            Serial.println();
            if (motorControl.autoTunePID(true)) {
                Serial.println("PID auto-tuning successful!");
            } else {
                Serial.println("PID auto-tuning failed!");
            }
        }
    }
    else if (command == "home") {
        Serial.println("Setting current position as home...");
        motorControl.setHome();
        Serial.println("Home position set!");
    }
    else if (command == "stop") {
        Serial.println("Stopping motor...");
        motorControl.stop();
        Serial.println("Motor stopped!");
    }
    else if (command == "m" || command == "move") {
        if (args.length() > 0) {
            float angle = args.toFloat();
            Serial.print("Moving to position: ");
            Serial.print(angle);
            Serial.println(" rad");
            motorControl.moveToPosition(angle);
        } else {
            Serial.println("Error: Please specify angle (e.g., 'm 3.14')");
        }
    }
    else if (command == "v" || command == "velocity") {
        if (args.length() > 0) {
            float vel = args.toFloat();
            Serial.print("Setting velocity to: ");
            Serial.print(vel);
            Serial.println(" rad/s");
            motorControl.setVelocity(vel);
        } else {
            Serial.println("Error: Please specify velocity (e.g., 'v 10.0')");
        }
    }
    else if (command == "a" || command == "accel") {
        if (args.length() > 0) {
            float accel = args.toFloat();
            Serial.print("Setting acceleration to: ");
            Serial.print(accel);
            Serial.println(" rad/s²");
            motorControl.setAcceleration(accel);
        } else {
            Serial.println("Error: Please specify acceleration (e.g., 'a 5.0')");
        }
    }
    else if (command == "mode") {
        if (args.length() > 0) {
            int mode = args.toInt();
            if (mode >= 0 && mode <= 2) {
                Serial.print("Setting control mode to: ");
                Serial.println(mode);
                motorControl.setControlMode(mode);
            } else {
                Serial.println("Error: Mode must be 0 (position), 1 (velocity), or 2 (torque)");
            }
        } else {
            Serial.println("Error: Please specify mode (e.g., 'mode 0')");
        }
    }
    else if (command == "test") {
        runFullTest();
    }
    else if (command == "motor_test") {
        runMotorTest();
    }
    else if (command == "encoder_test") {
        runEncoderTest();
    }
    else if (command == "debug") {
        // Parse debug subcommand
        if (args.length() == 0) {
            // Show current debug settings
            Serial.println("\n--- Debug Settings (Runtime Toggleable) ---");
            Serial.print("Serial:    ");
            Serial.println(g_debug_serial ? "ON" : "OFF");
            Serial.print("Motor:     ");
            Serial.println(g_debug_motor ? "ON" : "OFF");
            Serial.print("Comm:      ");
            Serial.println(g_debug_comm ? "ON" : "OFF");
            Serial.print("Heartbeat: ");
            Serial.println(g_debug_heartbeat ? "ON" : "OFF");
            Serial.println("\nUsage: debug <flag> <0/1>");
            Serial.println("  Flags: all, serial, motor, comm, heart");
        } else {
            // Parse "debug <flag> <value>"
            int spaceIndex = args.indexOf(' ');
            String flag = (spaceIndex > 0) ? args.substring(0, spaceIndex) : args;
            String value = (spaceIndex > 0) ? args.substring(spaceIndex + 1) : "";

            if (value.length() > 0) {
                bool enabled = (value == "1" || value == "on" || value == "true");

                if (flag == "all") {
                    g_debug_serial = enabled;
                    g_debug_motor = enabled;
                    g_debug_comm = enabled;
                    g_debug_heartbeat = enabled;
                    Serial.print("All debug flags set to: ");
                    Serial.println(enabled ? "ON" : "OFF");
                } else if (flag == "serial") {
                    g_debug_serial = enabled;
                    Serial.print("Serial debug: ");
                    Serial.println(enabled ? "ON" : "OFF");
                } else if (flag == "motor") {
                    g_debug_motor = enabled;
                    Serial.print("Motor debug: ");
                    Serial.println(enabled ? "ON" : "OFF");
                } else if (flag == "comm") {
                    g_debug_comm = enabled;
                    Serial.print("Comm debug: ");
                    Serial.println(enabled ? "ON" : "OFF");
                } else if (flag == "heart" || flag == "heartbeat") {
                    g_debug_heartbeat = enabled;
                    Serial.print("Heartbeat: ");
                    Serial.println(enabled ? "ON" : "OFF");
                } else {
                    Serial.println("Unknown debug flag. Use: all, serial, motor, comm, heart");
                }
            } else {
                Serial.println("Error: Please specify value (0/1 or on/off)");
                Serial.println("Example: debug serial 0");
            }
        }
    }
    else {
        Serial.print("Unknown command: '");
        Serial.print(cmd);
        Serial.println("'");
        Serial.println("Type 'help' for available commands");
    }
}

void checkSerialInput() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (serialCommandBuffer.length() > 0) {
                processSerialCommand(serialCommandBuffer);
                serialCommandBuffer = "";
            }
        } else {
            serialCommandBuffer += c;
        }
    }
}

//=============================================================================
// SETUP
//=============================================================================

void setup() {
    // Initialize debug serial if enabled
    if (g_debug_serial) {
        Serial.begin(SERIAL_BAUD);
        while (!Serial && millis() < 3000) {
            delay(10);
        }
        Serial.println("\n\n");
        Serial.println("╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║        OpenHyperspectral Motor Controller v1.0.0              ║");
        Serial.println("║        ESP32-S3-Touch-LCD-2 (Waveshare)                       ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.println();

        Serial.print("[INIT] CPU: ");
        Serial.print(ESP.getChipModel());
        Serial.print(" @ ");
        Serial.print(ESP.getCpuFreqMHz());
        Serial.println(" MHz");

        Serial.print("[INIT] Free RAM: ");
        Serial.print(ESP.getFreeHeap() / 1024);
        Serial.println(" KB");

        Serial.print("[INIT] Serial baud rate: ");
        Serial.println(SERIAL_BAUD);
    }

    // Initialize communication
    Serial.println("[INIT] Initializing communication...");
    comm.begin(SERIAL_BAUD);
    Serial.println("[OK]   Communication initialized");

    // Initialize motor controller
    Serial.println("[INIT] Initializing motor controller...");
    Serial.print("[INFO] Motor config: ");
    Serial.print(POLE_PAIRS);
    Serial.print(" pole pairs, ");
    Serial.print(ENCODER_PPR);
    Serial.println(" PPR encoder");

    motorControl.begin();
    Serial.println("[OK]   Motor controller initialized");

    // Optional: Run automatic calibration on startup
    // Uncomment if you want automatic calibration
    // Serial.println("[INIT] Running motor calibration...");
    // if (motorControl.calibrate()) {
    //     Serial.println("[OK]   Calibration successful");
    // } else {
    //     Serial.println("[WARN] Calibration failed - please calibrate manually");
    // }

    Serial.println();
    Serial.println("╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║                      SYSTEM READY!                             ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("✓ Serial monitor is working!");
    Serial.println("✓ Type 'help' for available commands");
    Serial.println("✓ System is ready to accept binary protocol commands");
    Serial.println();
    Serial.println("Quick test commands:");
    Serial.println("  - Type 'info' to see detailed system information");
    Serial.println("  - Type 'status' to see current motor status");
    Serial.println("  - Type 'test' to run a motor movement test");
    Serial.println();

    last_update_time = micros();
    last_heartbeat = millis();
}

//=============================================================================
// MAIN LOOP
//=============================================================================

void loop() {
    // Check for interactive serial commands (typed by user)
    if (g_debug_serial) {
        checkSerialInput();
    }

    // Check for incoming binary protocol commands
    if (comm.available()) {
        processCommand();
    }

    // Update motor control (FOC algorithm)
    motorControl.update();

    // Check for position reached notifications
    if (motorControl.getControlMode() == MODE_POSITION) {
        checkPositionReached();
    }

    // Periodic heartbeat message
    if (g_debug_heartbeat && (millis() - last_heartbeat > HEARTBEAT_INTERVAL_MS)) {
        last_heartbeat = millis();

        // CRITICAL: Force fresh encoder read from MT6701 via I2C
        // This reads the ABSOLUTE ENCODER - the truth source for position
        motorControl.updateEncoder();

        // Read all values (grouped by source)
        uint16_t raw_enc = motorControl.getRawEncoderCount();           // ABSOLUTE ENCODER raw
        float enc_deg = motorControl.getAbsolutePositionDeg();          // ABSOLUTE ENCODER (TRUTH)
        float foc_deg = motorControl.getCurrentPositionDeg();           // SimpleFOC internal state

        Serial.print("[HEARTBEAT] ");
        Serial.print(millis() / 1000);
        Serial.print("s | MT6701 Encoder: Raw=");
        Serial.print(raw_enc);
        Serial.print(" Pos=");
        Serial.print(enc_deg, 1);
        Serial.print("° | SimpleFOC: Pos=");
        Serial.print(foc_deg, 1);
        Serial.print("° (diff=");
        Serial.print(enc_deg - foc_deg, 1);
        Serial.print("°) | State: ");
        switch (motorControl.getState()) {
            case STATE_IDLE: Serial.print("IDLE"); break;
            case STATE_MOVING: Serial.print("MOVING"); break;
            case STATE_ERROR: Serial.print("ERROR"); break;
            case STATE_CALIBRATING: Serial.print("CALIBRATING"); break;
            default: Serial.print("UNKNOWN");
        }
        Serial.print(" | Enabled: ");
        Serial.println(motorControl.isEnabled() ? "Y" : "N");
    }

    // Periodic detailed status printing for debugging
    if (g_debug_serial && (millis() - last_status_print > 10000)) {
        last_status_print = millis();

        // CRITICAL: Force fresh encoder read from ABSOLUTE ENCODER
        motorControl.updateEncoder();

        Serial.println("\n╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║                    DETAILED STATUS                             ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");

        // MT6701 MAGNETIC ENCODER (Direct I2C reads - TRUTH SOURCE)
        Serial.println("\n[MT6701 ABSOLUTE ENCODER] (Direct I2C - TRUTH SOURCE)");
        Serial.print("  Raw Count: ");
        Serial.print(motorControl.getRawEncoderCount());
        Serial.println(" (0-16383)");
        Serial.print("  Position:  ");
        Serial.print(motorControl.getAbsolutePositionDeg(), 2);
        Serial.println("° ← USE THIS FOR POSITION CHECKS");

        // SIMPLEFOC STATE (Motor control library internal state - may lag/drift)
        Serial.println("\n[SimpleFOC STATE] (Motor controller internal - may lag)");
        Serial.print("  Shaft Angle:    ");
        Serial.print(motorControl.getCurrentPositionDeg(), 2);
        Serial.print("° (diff from encoder: ");
        Serial.print(motorControl.getAbsolutePositionDeg() - motorControl.getCurrentPositionDeg(), 2);
        Serial.println("°)");
        Serial.print("  Shaft Velocity: ");
        Serial.print(motorControl.getCurrentVelocityDegPerSec(), 2);
        Serial.println("°/s");
        Serial.print("  Q-axis Current: ");
        Serial.print(motorControl.getCurrent(), 3);
        Serial.println(" A");

        // SYSTEM STATE
        Serial.println("\n[SYSTEM STATE]");
        Serial.print("  State:      ");
        switch (motorControl.getState()) {
            case STATE_IDLE: Serial.println("IDLE"); break;
            case STATE_MOVING: Serial.println("MOVING"); break;
            case STATE_ERROR: Serial.println("ERROR"); break;
            case STATE_CALIBRATING: Serial.println("CALIBRATING"); break;
            default: Serial.println("UNKNOWN");
        }
        Serial.print("  Enabled:    ");
        Serial.println(motorControl.isEnabled() ? "YES" : "NO");
        Serial.print("  Calibrated: ");
        Serial.println(motorControl.isCalibrated() ? "YES" : "NO");
        Serial.print("  Free Heap:  ");
        Serial.print(ESP.getFreeHeap() / 1024);
        Serial.println(" KB");
        Serial.println();
    }

    // Maintain loop timing
    unsigned long current_time = micros();
    unsigned long elapsed = current_time - last_update_time;

    if (elapsed < LOOP_PERIOD_US) {
        delayMicroseconds(LOOP_PERIOD_US - elapsed);
    }

    last_update_time = micros();
}
