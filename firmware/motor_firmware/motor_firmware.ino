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
 */

#include <Arduino.h>
#include "config.h"
#include "commands.h"
#include "communication.h"
#include "motor_control.h"

//=============================================================================
// GLOBAL OBJECTS
//=============================================================================

CommunicationManager comm;
MotorController motorControl;

// Global pointer for ISR access
MotorController* g_motor_controller = nullptr;

// State tracking
uint16_t current_sequence_id = 0;
unsigned long last_update_time = 0;
unsigned long last_status_print = 0;

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
            float position = motorControl.getCurrentPosition();
            float velocity = motorControl.getCurrentVelocity();
            float current = motorControl.getCurrent();
            float voltage = motorControl.getVoltage();
            uint8_t state = motorControl.getState();
            uint8_t control_mode = motorControl.getControlMode();
            bool enabled = motorControl.isEnabled();
            bool calibrated = motorControl.isCalibrated();

            comm.sendStatus(state, control_mode, position, velocity,
                          current, voltage, enabled, calibrated);

            if (DEBUG_SERIAL) {
                Serial.println("STATUS");
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
    bool at_target = motorControl.isAtTarget();

    // Detect transition to target reached
    if (at_target && !last_at_target) {
        float position = motorControl.getCurrentPosition();
        comm.sendPositionReached(current_sequence_id, position);

        if (DEBUG_SERIAL) {
            Serial.print("Position reached notification sent: seq=");
            Serial.print(current_sequence_id);
            Serial.print(", pos=");
            Serial.println(position);
        }
    }

    last_at_target = at_target;
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
        Serial.println("\n\n=== OpenHyperspectral Motor Controller ===");
        Serial.println("Firmware Version: 1.0.0");
        Serial.println("Board: ESP32-S3-Touch-LCD-2 (Waveshare)");
    }

    // Initialize communication
    comm.begin(SERIAL_BAUD);
    Serial.println("Communication initialized");

    // Set global pointer for ISR access
    g_motor_controller = &motorControl;

    // Initialize motor controller
    motorControl.begin();
    Serial.println("Motor controller initialized");

    // Optional: Run automatic calibration on startup
    // Uncomment if you want automatic calibration
    // Serial.println("Running motor calibration...");
    // if (motorControl.calibrate()) {
    //     Serial.println("Calibration successful");
    // } else {
    //     Serial.println("Calibration failed - please calibrate manually");
    // }

    Serial.println("System ready!");
    Serial.println("Waiting for commands...\n");

    last_update_time = micros();
}

//=============================================================================
// MAIN LOOP
//=============================================================================

void loop() {
    // Check for incoming commands
    if (comm.available()) {
        processCommand();
    }

    // Update motor control (FOC algorithm)
    motorControl.update();

    // Check for position reached notifications
    if (motorControl.getControlMode() == MODE_POSITION) {
        checkPositionReached();
    }

    // Periodic status printing for debugging
    if (DEBUG_SERIAL && (millis() - last_status_print > 1000)) {
        last_status_print = millis();

        Serial.print("Status: ");
        Serial.print("pos=");
        Serial.print(motorControl.getCurrentPosition(), 3);
        Serial.print(", vel=");
        Serial.print(motorControl.getCurrentVelocity(), 3);
        Serial.print(", state=");
        Serial.print(motorControl.getState());
        Serial.print(", enabled=");
        Serial.print(motorControl.isEnabled() ? "Y" : "N");
        Serial.print(", calibrated=");
        Serial.println(motorControl.isCalibrated() ? "Y" : "N");
    }

    // Maintain loop timing
    unsigned long current_time = micros();
    unsigned long elapsed = current_time - last_update_time;

    if (elapsed < LOOP_PERIOD_US) {
        delayMicroseconds(LOOP_PERIOD_US - elapsed);
    }

    last_update_time = micros();
}
