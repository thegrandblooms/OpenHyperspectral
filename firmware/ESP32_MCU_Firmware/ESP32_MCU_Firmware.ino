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
#include "tests.h"  // Test and diagnostic functions

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

// Debug control (runtime toggleable)
bool debug_heartbeat_enabled = DEBUG_HEARTBEAT;
bool debug_detailed_status_enabled = DEBUG_SERIAL;

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

                if (DEBUG_SERIAL) {
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

                if (DEBUG_SERIAL) {
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

                if (DEBUG_SERIAL) {
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

            if (DEBUG_SERIAL) {
                Serial.println("STOP");
            }
            break;
        }

        case CMD_HOME: {
            motorControl.setHome();
            comm.sendOkResponse(cmd_id);

            if (DEBUG_SERIAL) {
                Serial.println("HOME");
            }
            break;
        }

        case CMD_ENABLE: {
            motorControl.enable();
            comm.sendOkResponse(cmd_id);

            if (DEBUG_SERIAL) {
                Serial.println("ENABLE");
            }
            break;
        }

        case CMD_DISABLE: {
            motorControl.disable();
            comm.sendOkResponse(cmd_id);

            if (DEBUG_SERIAL) {
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

            if (DEBUG_SERIAL) {
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

                if (DEBUG_SERIAL) {
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

                    if (DEBUG_SERIAL) {
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

                if (DEBUG_SERIAL) {
                    Serial.print("SET_CURRENT_LIMIT: ");
                    Serial.println(cmd.current_limit);
                }
            } else {
                comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            }
            break;
        }

        case CMD_CALIBRATE: {
            if (DEBUG_SERIAL) {
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

                if (DEBUG_SERIAL) {
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

            if (DEBUG_SERIAL) {
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
    static unsigned long at_target_start_time = 0;
    static unsigned long last_notification_time = 0;
    static bool notification_sent_for_this_move = false;

    // Hysteresis and cooldown constants
    const unsigned long SETTLE_TIME_MS = 100;      // Must be at target for 100ms
    const unsigned long NOTIFICATION_COOLDOWN_MS = 500;  // Min time between notifications

    bool at_target = motorControl.isAtTarget();  // Uses ABSOLUTE ENCODER internally
    unsigned long now = millis();

    // Track when we first entered at_target state
    if (at_target && !last_at_target) {
        at_target_start_time = now;
    }

    // Reset notification flag when we leave target zone (preparing for next move)
    if (!at_target) {
        notification_sent_for_this_move = false;
    }

    // Check if motor has settled at target
    bool settled = at_target && (now - at_target_start_time >= SETTLE_TIME_MS);
    bool cooldown_elapsed = (now - last_notification_time >= NOTIFICATION_COOLDOWN_MS);

    // Send ONE notification when motor first settles at target
    if (settled && cooldown_elapsed && !notification_sent_for_this_move) {
        // Report ABSOLUTE ENCODER position (truth)
        motorControl.updateEncoder();
        float position = motorControl.getAbsolutePositionDeg();  // ABSOLUTE ENCODER
        comm.sendPositionReached(current_sequence_id, position);
        last_notification_time = now;
        notification_sent_for_this_move = true;

        if (DEBUG_SERIAL) {
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
// SERIAL COMMAND PROCESSING (Test functions moved to tests.cpp)
//=============================================================================


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

    // Process commands (test functions in tests.cpp)
    if (command == "h" || command == "help") {
        printHelp();
    }
    else if (command == "s" || command == "status") {
        printStatus(motorControl);
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
            Serial.println("° (absolute position 0-360)");
            motorControl.moveToPosition(angle);
        } else {
            Serial.println("Error: Please specify angle in degrees (e.g., 'm 90' or 'm 180.5')");
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
        runFullTest(motorControl);
    }
    else if (command == "motor_test") {
        runMotorTest(motorControl);
    }
    else if (command == "position_sweep" || command == "sweep") {
        runPositionSweepTest(motorControl);
    }
    else if (command == "encoder_test") {
        runEncoderTest(motorControl);
    }
    else if (command == "phase_test" || command == "phasetest") {
        runPhaseTest(motorControl);
    }
    else if (command == "align" || command == "alignment_test") {
        runAlignmentTest(motorControl);
    }
    else if (command == "diag" || command == "diagnostic" || command == "foc_diag") {
        runSimpleFOCDiagnostic(motorControl);
    }
    else if (command == "debug") {
        if (args.length() > 0) {
            int level = args.toInt();
            if (level == 0) {
                // Quiet mode - disable heartbeat and detailed status
                debug_heartbeat_enabled = false;
                debug_detailed_status_enabled = false;
                Serial.println("Debug mode: QUIET (heartbeat and detailed status disabled)");
            } else if (level == 1) {
                // Verbose mode - enable heartbeat and detailed status
                debug_heartbeat_enabled = true;
                debug_detailed_status_enabled = true;
                Serial.println("Debug mode: VERBOSE (heartbeat and detailed status enabled)");
            } else if (level == 2) {
                // Heartbeat only
                debug_heartbeat_enabled = true;
                debug_detailed_status_enabled = false;
                Serial.println("Debug mode: HEARTBEAT ONLY");
            } else {
                Serial.println("Error: Debug level must be 0 (quiet), 1 (verbose), or 2 (heartbeat only)");
            }
        } else {
            Serial.println("\n=== Debug Status ===");
            Serial.print("Heartbeat (every 1s):         ");
            Serial.println(debug_heartbeat_enabled ? "ENABLED" : "DISABLED");
            Serial.print("Detailed Status (every 10s):  ");
            Serial.println(debug_detailed_status_enabled ? "ENABLED" : "DISABLED");
            Serial.println("\nUsage: debug <level>");
            Serial.println("  0 = Quiet (all debug output off)");
            Serial.println("  1 = Verbose (all debug output on)");
            Serial.println("  2 = Heartbeat only");
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
    if (DEBUG_SERIAL) {
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
    if (DEBUG_SERIAL) {
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

    // Periodic heartbeat message (toggle with 'debug' command)
    if (debug_heartbeat_enabled && (millis() - last_heartbeat > HEARTBEAT_INTERVAL_MS)) {
        last_heartbeat = millis();

        // CRITICAL: Force fresh encoder read from MT6701 via I2C
        // This reads the ABSOLUTE ENCODER - the truth source for position
        motorControl.updateEncoder();

        // Read all values (grouped by source)
        uint16_t raw_enc = motorControl.getRawEncoderCount();           // ABSOLUTE ENCODER raw
        float enc_deg = motorControl.getAbsolutePositionDeg();          // ABSOLUTE ENCODER (TRUTH)
        float foc_deg = motorControl.getCurrentPositionDeg();           // SimpleFOC internal state

        // Compact heartbeat format: all data on one line
        const char* state_str = "?";
        switch (motorControl.getState()) {
            case STATE_IDLE: state_str = "IDLE"; break;
            case STATE_MOVING: state_str = "MOVE"; break;
            case STATE_ERROR: state_str = "ERR"; break;
            case STATE_CALIBRATING: state_str = "CAL"; break;
        }
        Serial.print("[HB] ");
        Serial.print(millis() / 1000);
        Serial.print("s | Enc:");
        Serial.print(enc_deg, 1);
        Serial.print("° FOC:");
        Serial.print(foc_deg, 1);
        Serial.print("° Δ:");
        Serial.print(enc_deg - foc_deg, 1);
        Serial.print("° Raw:");
        Serial.print(raw_enc);
        Serial.print(" | ");
        Serial.print(state_str);
        Serial.print(" En:");
        Serial.println(motorControl.isEnabled() ? "Y" : "N");
    }

    // Periodic detailed status printing (toggle with 'debug' command)
    if (debug_detailed_status_enabled && (millis() - last_status_print > 10000)) {
        last_status_print = millis();

        // CRITICAL: Force fresh encoder read from ABSOLUTE ENCODER
        motorControl.updateEncoder();

        // Compact detailed status: 2 lines with all data
        const char* state_str = "?";
        switch (motorControl.getState()) {
            case STATE_IDLE: state_str = "IDLE"; break;
            case STATE_MOVING: state_str = "MOVING"; break;
            case STATE_ERROR: state_str = "ERROR"; break;
            case STATE_CALIBRATING: state_str = "CALIBRATING"; break;
        }
        Serial.print("[STATUS] MT6701: Raw=");
        Serial.print(motorControl.getRawEncoderCount());
        Serial.print(" Pos=");
        Serial.print(motorControl.getAbsolutePositionDeg(), 2);
        Serial.print("° | FOC: Pos=");
        Serial.print(motorControl.getCurrentPositionDeg(), 2);
        Serial.print("° Vel=");
        Serial.print(motorControl.getCurrentVelocityDegPerSec(), 2);
        Serial.print("°/s I=");
        Serial.print(motorControl.getCurrent(), 3);
        Serial.println("A");
        Serial.print("         State=");
        Serial.print(state_str);
        Serial.print(" En=");
        Serial.print(motorControl.isEnabled() ? "Y" : "N");
        Serial.print(" Cal=");
        Serial.print(motorControl.isCalibrated() ? "Y" : "N");
        Serial.print(" | Diff(Enc-FOC)=");
        Serial.print(motorControl.getAbsolutePositionDeg() - motorControl.getCurrentPositionDeg(), 2);
        Serial.print("° | RAM=");
        Serial.print(ESP.getFreeHeap() / 1024);
        Serial.println("KB");
    }

    // Maintain loop timing
    unsigned long current_time = micros();
    unsigned long elapsed = current_time - last_update_time;

    if (elapsed < LOOP_PERIOD_US) {
        delayMicroseconds(LOOP_PERIOD_US - elapsed);
    }

    last_update_time = micros();
}
