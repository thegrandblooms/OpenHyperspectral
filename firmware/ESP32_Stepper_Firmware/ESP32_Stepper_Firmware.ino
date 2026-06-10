/**
 * OpenHyperspectral Stepper Motor Controller Firmware
 *
 * ESP32-S3-Touch-LCD-2 (Waveshare) driving a NEMA17 stepper through a
 * TMC2209 (STEP/DIR + UART) and a belt reduction, with an MT6835 21-bit
 * absolute encoder on the OUTPUT shaft for closed-loop position trim.
 *
 * This replaces the SimpleFOC gimbal-motor firmware (ESP32_MCU_Firmware).
 * Rationale: the iron-core gimbal motor's cogging produced an unfixable
 * stall-snap-bounce limit cycle at scan speeds, and the MT6701's +/-1 deg
 * INL capped absolute accuracy far above the 0.1/0.02 deg requirement.
 * Full history: Dev_Log/motor/STATE.md and HISTORY.md.
 *
 * SERIAL PROTOCOL: command set and tagged-CSV streaming ($ENC, $SWEEP_*,
 * $SYNC, $SET) are kept compatible with the gimbal firmware so
 * tools/motor_test.py and tools/sweep_validation.py continue to work.
 */

#include <Arduino.h>
#include "config.h"
#include "commands.h"
#include "communication.h"
#include "motor_control.h"
#include "tests.h"

//=============================================================================
// GLOBAL OBJECTS
//=============================================================================

CommunicationManager comm;
MotorController motorControl;

uint16_t current_sequence_id = 0;
unsigned long last_status_print = 0;
bool debug_status_enabled = DEBUG_SERIAL;
String serialCommandBuffer = "";

//=============================================================================
// BINARY PROTOCOL COMMANDS (SerialTransfer — same protocol as gimbal firmware)
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
            } else {
                comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            }
            break;
        }
        case CMD_STOP:    motorControl.stop();    comm.sendOkResponse(cmd_id); break;
        case CMD_HOME:    motorControl.setHome(); comm.sendOkResponse(cmd_id); break;
        case CMD_ENABLE:  motorControl.enable();  comm.sendOkResponse(cmd_id); break;
        case CMD_DISABLE: motorControl.disable(); comm.sendOkResponse(cmd_id); break;
        case CMD_GET_STATUS: {
            motorControl.updateEncoder();
            comm.sendStatus(motorControl.getState(), motorControl.getControlMode(),
                            motorControl.getAbsolutePositionDeg(),
                            motorControl.getCurrentVelocityDegPerSec(),
                            motorControl.getCurrent(), motorControl.getVoltage(),
                            motorControl.isEnabled(), motorControl.isCalibrated());
            break;
        }
        case CMD_PING: {
            PingCommand cmd;
            if (comm.getCommandData(&cmd)) comm.sendPingResponse(cmd.echo_value);
            else comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            break;
        }
        case CMD_SET_CURRENT_LIMIT: {
            SetCurrentLimitCommand cmd;
            if (comm.getCommandData(&cmd)) {
                motorControl.setCurrentLimit(cmd.current_limit);
                comm.sendOkResponse(cmd_id);
            } else {
                comm.sendErrorResponse(cmd_id, ERR_INVALID_PARAMETER);
            }
            break;
        }
        case CMD_CALIBRATE: {
            if (motorControl.calibrate()) comm.sendOkResponse(cmd_id);
            else comm.sendErrorResponse(cmd_id, ERR_COMMAND_FAILED);
            break;
        }
        case CMD_SET_MODE:
        case CMD_SET_PID:
            // Stepper firmware: position-only, no PID. Acknowledge for compat.
            comm.sendOkResponse(cmd_id);
            break;
        default:
            comm.sendErrorResponse(cmd_id, ERR_INVALID_COMMAND);
            break;
    }
}

//=============================================================================
// POSITION REACHED NOTIFICATION (binary protocol)
//=============================================================================

void checkPositionReached() {
    static bool last_at_target = false;
    static unsigned long at_target_start_time = 0;
    static unsigned long last_notification_time = 0;
    static bool notification_sent_for_this_move = false;
    static float last_target_position = -999.0f;

    const unsigned long SETTLE_TIME_MS = 150;
    const unsigned long NOTIFICATION_COOLDOWN_MS = 1000;

    bool at_target = motorControl.isAtTarget();
    float current_target = motorControl.getTargetPositionDeg();
    unsigned long now = millis();

    if (fabsf(current_target - last_target_position) > 0.001f) {
        notification_sent_for_this_move = false;
        last_target_position = current_target;
        at_target_start_time = 0;
    }
    if (at_target && !last_at_target) at_target_start_time = now;

    bool settled = at_target && at_target_start_time > 0 &&
                   (now - at_target_start_time >= SETTLE_TIME_MS);

    if (settled && !notification_sent_for_this_move &&
        (now - last_notification_time >= NOTIFICATION_COOLDOWN_MS)) {
        motorControl.updateEncoder();
        comm.sendPositionReached(current_sequence_id, motorControl.getAbsolutePositionDeg());
        last_notification_time = now;
        notification_sent_for_this_move = true;
        if (DEBUG_SERIAL) {
            Serial.printf("AT_TARGET: %.4f deg (seq=%u)\n",
                          motorControl.getAbsolutePositionDeg(), current_sequence_id);
        }
    }
    last_at_target = at_target;
}

//=============================================================================
// INTERACTIVE SERIAL COMMANDS
//=============================================================================

// Tokenize up to maxn space-separated floats from args; returns count parsed
static int parseFloats(const String& s, float* out, int maxn) {
    int n = 0, start = 0;
    while (n < maxn && start < (int)s.length()) {
        while (start < (int)s.length() && s[start] == ' ') start++;
        if (start >= (int)s.length()) break;
        int sp = s.indexOf(' ', start);
        String tok = (sp < 0) ? s.substring(start) : s.substring(start, sp);
        out[n++] = tok.toFloat();
        if (sp < 0) break;
        start = sp + 1;
    }
    return n;
}

void processSerialCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    if (cmd.length() == 0) return;

    motorControl.notifyCommandActivity();
    Serial.print("\n> ");
    Serial.println(cmd);

    int spaceIndex = cmd.indexOf(' ');
    String command = (spaceIndex > 0) ? cmd.substring(0, spaceIndex) : cmd;
    String args = (spaceIndex > 0) ? cmd.substring(spaceIndex + 1) : "";

    if (command == "h" || command == "help") {
        printHelp();
    }
    else if (command == "s" || command == "status") {
        printStatus(motorControl);
    }
    else if (command == "i" || command == "info") {
        printSystemInfo();
    }
    else if (command == "e" || command == "enable") {
        motorControl.enable();
        motorControl.setAutoEnabled(false);
        Serial.println("Motor enabled (holding)");
    }
    else if (command == "d" || command == "disable") {
        motorControl.disable();
        Serial.println("Motor disabled (shaft free)");
    }
    else if (command == "c" || command == "calibrate") {
        Serial.print("Calibrating... ");
        Serial.println(motorControl.calibrate() ? "OK" : "FAILED");
    }
    else if (command == "recalibrate") {
        Serial.println("Measuring backlash (moves the shaft a few degrees)...");
        Serial.println(motorControl.recalibrate() ? "OK" : "FAILED");
    }
    else if (command == "stop") {
        motorControl.stop();
        Serial.println("Stopped (holding)");
    }
    else if (command == "m" || command == "move" ||
             (command.startsWith("m") && command.length() > 1 &&
              (isDigit(command[1]) || command[1] == '-' || command[1] == '.'))) {
        // Supports "m 90", "m90", "move 180.5"
        String angleStr = args;
        if (command.length() > 1 && command.startsWith("m") &&
            command != "move" && command != "motor_test") {
            angleStr = command.substring(1) + (args.length() > 0 ? " " + args : "");
        }
        if (angleStr.length() > 0) {
            float angle = angleStr.toFloat();
            if (!motorControl.isEnabled() && motorControl.isCalibrated()) {
                Serial.println("[AUTO] Enabling motor for move command...");
                motorControl.enable();
                motorControl.setAutoEnabled(true);
            }
            Serial.printf("Moving to %.3f deg (absolute, output shaft)\n", angle);
            motorControl.moveToPosition(angle);
        } else {
            Serial.println("Error: specify angle (e.g. 'm 90', 'm90')");
        }
    }
    else if (command == "sync") {
        Serial.print("$SYNC,");
        Serial.println(millis());
    }
    else if (command == "sweep" && args.length() > 0) {
        if (!motorControl.isCalibrated()) {
            Serial.println("[SWEEP] Not calibrated — run 'c' first");
        } else {
            if (!motorControl.isEnabled()) {
                motorControl.enable();
                motorControl.setAutoEnabled(true);
            }
            motorControl.startSweep(args.toFloat());
        }
    }
    else if (command == "sweep_stop") {
        if (motorControl.isSweepActive() || motorControl.isScanActive())
            motorControl.stopSweep();
        else Serial.println("[SWEEP] No sweep active");
    }
    else if (command == "scan_step") {
        // Stepped capture scan: scan_step <start> <end> <inc> [dwell_ms=500]
        // Visits each position with full settle+trim, emits $FRAME markers.
        float v[4];
        int n = parseFloats(args, v, 4);
        if (n < 3) {
            Serial.println("Usage: scan_step <start_deg> <end_deg> <inc_deg> [dwell_ms]");
        } else if (!motorControl.isCalibrated()) {
            Serial.println("[SCAN] Not calibrated — run 'c' first");
        } else {
            if (!motorControl.isEnabled()) {
                motorControl.enable();
                motorControl.setAutoEnabled(true);
            }
            motorControl.startSteppedScan(v[0], v[1], v[2],
                                          n >= 4 ? (uint32_t)v[3] : 500);
        }
    }
    else if (command == "scan_smooth") {
        // Smooth video scan: scan_smooth <start> <end> <speed_deg_s>
        // Pre-positions (lash taken up in scan direction), streams $ENC,
        // runs constant rate start->end. Frame-match host-side via 'sync'.
        float v[3];
        int n = parseFloats(args, v, 3);
        if (n < 3) {
            Serial.println("Usage: scan_smooth <start_deg> <end_deg> <speed_deg_s>");
        } else if (!motorControl.isCalibrated()) {
            Serial.println("[SCAN] Not calibrated — run 'c' first");
        } else {
            if (!motorControl.isEnabled()) {
                motorControl.enable();
                motorControl.setAutoEnabled(true);
            }
            motorControl.startSmoothScan(v[0], v[1], v[2]);
        }
    }
    else if (command == "scan_stop") {
        if (motorControl.isScanActive()) motorControl.abortScan();
        else Serial.println("[SCAN] No scan active");
    }
    else if (command == "test" || command == "diag" || command == "diagnostic") {
        runSystemDiagnostic(motorControl);
    }
    else if (command == "motor_test") {
        runMotorTest(motorControl);
    }
    else if (command == "position_sweep") {
        runPositionSweepTest(motorControl);
    }
    else if (command == "encoder_test") {
        runEncoderTest(motorControl);
    }
    else if (command == "stream") {
        if (args == "on") {
            motorControl.setStreamEnabled(true);
            Serial.println("Encoder streaming ON");
        } else if (args == "off") {
            motorControl.setStreamEnabled(false);
            Serial.println("Encoder streaming OFF");
        } else if (args.startsWith("rate")) {
            int spaceIdx = args.indexOf(' ');
            if (spaceIdx > 0) {
                motorControl.setStreamRate(args.substring(spaceIdx + 1).toInt());
            }
            Serial.printf("Stream rate: %d Hz\n", motorControl.getStreamRate());
        } else {
            Serial.printf("Streaming: %s @ %d Hz\n",
                          motorControl.isStreamEnabled() ? "ON" : "OFF",
                          motorControl.getStreamRate());
            Serial.println("Format: $ENC,ms,enc_pos,enc_vel,target,cmd_pos,trim_err");
        }
    }
    else if (command == "stream_on") {
        if (args.length() > 0) motorControl.setStreamRate(args.toInt());
        motorControl.setStreamEnabled(true);
    }
    else if (command == "stream_off") {
        motorControl.setStreamEnabled(false);
    }
    else if (command == "debug") {
        if (args.length() > 0) debug_status_enabled = (args.toInt() != 0);
        Serial.printf("Status output: %s\n", debug_status_enabled ? "ON" : "OFF");
    }
    else if (command == "set") {
        int space_idx = args.indexOf(' ');
        if (space_idx < 0) {
            Serial.println("Usage: set <key> <value>");
            Serial.println("Keys: vel_max, accel, cur_lim, pos_tol, vel_thresh, trim, trim_tol");
        } else {
            String key = args.substring(0, space_idx);
            float val = args.substring(space_idx + 1).toFloat();
            bool ok = true;

            if      (key == "vel_max")    motorControl.setVelocity(val);
            else if (key == "accel")      motorControl.setAcceleration(val);
            else if (key == "cur_lim")    motorControl.setCurrentLimit(val);
            else if (key == "pos_tol")    motorControl.setPositionTolerance(val);
            else if (key == "vel_thresh") motorControl.setVelocityThreshold(val);
            else if (key == "trim")       motorControl.setTrimEnabled(val != 0.0f);
            else if (key == "trim_tol")   motorControl.setTrimTolerance(val);
            else {
                Serial.printf("Unknown setting key: '%s'\n", key.c_str());
                ok = false;
            }
            if (ok) {
                Serial.printf("$SET,%s,%.4f,ok\n", key.c_str(), val);
            }
        }
    }
    else {
        Serial.printf("Unknown command: '%s' — type 'help'\n", cmd.c_str());
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
// SETUP / LOOP
//=============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000) delay(10);

    Serial.println("\n\n");
    Serial.println("==================================================================");
    Serial.println("  OpenHyperspectral STEPPER Controller v2.0.0");
    Serial.println("  NEMA17 + TMC2209 + belt reduction + MT6835 output encoder");
    Serial.println("==================================================================");
    Serial.printf("[INIT] CPU: %s @ %d MHz, free RAM %d KB\n",
                  ESP.getChipModel(), ESP.getCpuFreqMHz(), ESP.getFreeHeap() / 1024);

    comm.begin(SERIAL_BAUD);
    Serial.println("[OK]   Communication initialized");

    motorControl.begin();
    Serial.println("[OK]   Motor controller initialized");
    Serial.printf("[INFO] %.0f steps/output-rev (%.5f deg/ustep), encoder %.6f deg/count\n",
                  STEPS_PER_OUTPUT_REV, 1.0f / STEPS_PER_OUTPUT_DEG,
                  360.0f / (float)ENCODER_COUNTS_PER_REV);

    Serial.println("\nSYSTEM READY — type 'help' for commands\n");
}

void loop() {
    checkSerialInput();

    if (comm.available()) {
        processCommand();
    }

    motorControl.update();
    checkPositionReached();

    if (debug_status_enabled && (millis() - last_status_print > HEARTBEAT_INTERVAL_MS)) {
        last_status_print = millis();
        motorControl.updateEncoder();
        Serial.printf("[%lus] Enc:%.3f deg | Cmd:%.3f deg | %.2f deg/s | %s En:%c Cal:%c\n",
                      millis() / 1000,
                      motorControl.getAbsolutePositionDeg(),
                      motorControl.getCommandedPositionDeg(),
                      motorControl.getCurrentVelocityDegPerSec(),
                      motorControl.getState() == STATE_MOVING ? "MOVE" : "IDLE",
                      motorControl.isEnabled() ? 'Y' : 'N',
                      motorControl.isCalibrated() ? 'Y' : 'N');
    }

    // Light loop pacing — step timing lives in the hardware-timer ISR, so the
    // main loop only needs to be fast enough for profile updates (~1kHz).
    delayMicroseconds(200);
}
