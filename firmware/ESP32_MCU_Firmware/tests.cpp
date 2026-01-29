/**
 * Test Functions for OpenHyperspectral Motor Controller
 * Consolidated test suite - all tests in one place
 */

#include "tests.h"
#include "config.h"
#include "commands.h"
#include <Wire.h>

//=============================================================================
// DIAGNOSTIC HELPERS
//=============================================================================

void logMotorState(MotorController& mc, const char* ctx) {
    BLDCMotor& motor = mc.getMotor();
    float enc = mc.getEncoderDegrees();
    float foc = radiansToDegrees(motor.shaft_angle);
    float tgt = mc.getTargetPositionDeg();
    float err = motor.shaft_angle_sp - motor.shaft_angle;

    Serial.printf("[DIAG] %s | Enc:%.1f° FOC:%.1f° Tgt:%.1f° | En:%c Cal:%c | TrkErr:%.2f°\n",
        ctx, enc, foc, tgt, mc.isEnabled() ? 'Y' : 'N', mc.isCalibrated() ? 'Y' : 'N', abs(enc - foc));
    Serial.printf("       PIDerr:%.1f° VelCmd:%.1f°/s VelAct:%.1f°/s | Vq:%.2fV Vd:%.2fV\n",
        radiansToDegrees(err), radiansToDegrees(motor.shaft_velocity_sp),
        radiansToDegrees(motor.shaft_velocity), motor.voltage.q, motor.voltage.d);
}

//=============================================================================
// I2C SCANNER
//=============================================================================

void scanI2C() {
    Serial.println("\n=== I2C Scanner ===");
    Serial.printf("Scanning I2C (SDA=%d, SCL=%d)...\n", ENCODER_SDA, ENCODER_SCL);

    byte count = 0;
    for (byte i = 1; i < 127; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X", i);
            if (i == 0x06) Serial.print(" (MT6701)");
            else if (i == 0x15) Serial.print(" (CST816D)");
            else if (i == 0x6B) Serial.print(" (QMI8658)");
            Serial.println();
            count++;
        }
    }
    Serial.printf("Found %d device(s)\n\n", count);
}

//=============================================================================
// INFO & STATUS
//=============================================================================

void printHelp() {
    Serial.println("\n=== OpenHyperspectral Motor Controller ===");
    Serial.println("Commands:");
    Serial.println("  h/help, s/status, i/info, scan");
    Serial.println("  e/enable, d/disable, c/calibrate, home, stop");
    Serial.println("  m <deg>, v <deg/s>, a <deg/s²>, mode <0-2>");
    Serial.println("Encoder/Filter (precision tuning):");
    Serial.println("  encoder/enc - Show encoder status");
    Serial.println("  filter <0-1> - Set filter alpha (0=smooth, 1=fast)");
    Serial.println("  direct <0|1> - Toggle direct encoder mode");
    Serial.println("Tests:");
    Serial.println("  phase_test, test, motor_test, position_sweep");
    Serial.println("  encoder_test, diag, alignment_test");
    Serial.println("  debug <0/1>\n");
}

void printSystemInfo() {
    Serial.println("\n=== System Info ===");
    Serial.printf("Chip: %s @ %dMHz | Flash: %dMB | Heap: %dKB | Up: %ds\n",
        ESP.getChipModel(), ESP.getCpuFreqMHz(),
        ESP.getFlashChipSize()/1024/1024, ESP.getFreeHeap()/1024, millis()/1000);
    Serial.printf("Motor: %d poles, %d PPR, %.1fV PSU, %.2fA limit, %.0f°/s max\n",
        POLE_PAIRS, ENCODER_PPR, VOLTAGE_PSU, CURRENT_LIMIT, MAX_VELOCITY);
    Serial.printf("Pins: EN=%d IN1=%d IN2=%d IN3=%d nFT=%d | SDA=%d SCL=%d\n\n",
        MOTOR_ENABLE, MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_FAULT,
        ENCODER_SDA, ENCODER_SCL);
}

void printStatus(MotorController& mc) {
    mc.updateEncoder();
    Serial.println("\n=== Motor Status ===");
    Serial.printf("Encoder: %d raw, %.2f° (source of truth)\n",
        mc.getRawEncoderCount(), mc.getEncoderDegrees());
    Serial.printf("SimpleFOC: %.2f° %.2f°/s %.3fA %.2fV\n",
        mc.getCurrentPositionDeg(), mc.getCurrentVelocityDegPerSec(),
        mc.getCurrent(), mc.getVoltage());

    const char* states[] = {"IDLE", "MOVING", "ERROR", "CALIBRATING"};
    const char* modes[] = {"POSITION", "VELOCITY", "TORQUE"};
    Serial.printf("State: %s | Mode: %s | En:%c Cal:%c AtTgt:%c\n\n",
        states[mc.getState()], modes[mc.getControlMode()],
        mc.isEnabled() ? 'Y' : 'N', mc.isCalibrated() ? 'Y' : 'N',
        mc.isAtTarget() ? 'Y' : 'N');
}

//=============================================================================
// ENCODER TEST
//=============================================================================

void runEncoderTest(MotorController& mc) {
    Serial.println("\n=== Encoder Test ===");
    Serial.println("Rotate motor manually. Press any key to stop.\n");

    unsigned long start = millis(), last = 0;
    while (Serial.available()) Serial.read();

    while (!Serial.available()) {
        if (millis() - last >= 100) {
            last = millis();
            mc.updateEncoder();
            Serial.printf("[%.2fs] Raw:%d Enc:%.2f° FOC:%.2f°\n",
                (last - start) / 1000.0, mc.getRawEncoderCount(),
                mc.getEncoderDegrees(), mc.getCurrentPositionDeg());
        }
        delay(1);
    }
    Serial.read();
    Serial.println("Stopped.\n");
}

//=============================================================================
// PHASE TEST (moved from motor_control.cpp)
//=============================================================================

void runPhaseTest(MotorController& mc) {
    Serial.println("\n=== Driver Phase Test ===");
    Serial.println("Testing 6 phase angles to verify driver outputs.\n");

    BLDCMotor& motor = mc.getMotor();
    MT6701Sensor& encoder = mc.getEncoder();

    pinMode(MOTOR_FAULT, INPUT_PULLUP);
    bool fault = digitalRead(MOTOR_FAULT);
    Serial.printf("Fault pin before: %s\n", fault ? "OK" : "FAULT!");

    motor.enable();
    delay(100);
    fault = digitalRead(MOTOR_FAULT);
    Serial.printf("Fault pin after enable: %s\n", fault ? "OK" : "FAULT!");

    encoder.update();
    float start_pos = encoder.getDegrees();
    Serial.printf("Start position: %.2f°\n\n", start_pos);

    float angles[] = {0, _PI_3, _PI_2, 2*_PI_3, PI, 4*_PI_3};
    const char* names[] = {"0°", "60°", "90°", "120°", "180°", "240°"};

    for (int i = 0; i < 6; i++) {
        motor.setPhaseVoltage(6.0, 0, angles[i]);
        delay(1000);
        encoder.update();
        float pos = encoder.getDegrees();
        float move = abs(pos - start_pos);
        if (move > 180) move = 360 - move;
        fault = digitalRead(MOTOR_FAULT);
        Serial.printf("[%d/6] %s: %.1f° (moved %.1f°) %s\n",
            i+1, names[i], pos, move, fault ? "OK" : "FAULT!");
        delay(200);
    }

    motor.disable();
    Serial.println("\nPhase test complete. All 6 positions = all phases OK.\n");
}

//=============================================================================
// ALIGNMENT TEST (moved from motor_control.cpp)
//=============================================================================

void runAlignmentTest(MotorController& mc) {
    Serial.println("\n=== Motor Alignment Test ===");
    Serial.println("Testing 4 electrical angles. Motor should hold firmly.\n");

    BLDCMotor& motor = mc.getMotor();
    MT6701Sensor& encoder = mc.getEncoder();

    encoder.update();
    float init = encoder.getDegrees();
    if (isnan(init)) {
        Serial.println("ERROR: Cannot read encoder!");
        return;
    }
    Serial.printf("Encoder OK: %.2f°\n", init);

    motor.enable();
    pinMode(MOTOR_FAULT, INPUT_PULLUP);
    bool fault = digitalRead(MOTOR_FAULT);
    Serial.printf("Fault: %s\n\n", fault ? "OK" : "FAULT!");

    float angles[] = {0, _PI_2, PI, _3PI_2};
    const char* names[] = {"0°", "90°", "180°", "270°"};

    for (int i = 0; i < 4; i++) {
        motor.setPhaseVoltage(6.0, 0, angles[i]);
        delay(1500);
        encoder.update();
        fault = digitalRead(MOTOR_FAULT);
        Serial.printf("[%s] Pos: %.2f° Fault:%s (try rotating by hand)\n",
            names[i], encoder.getDegrees(), fault ? "OK" : "FAULT!");
        delay(300);
    }

    motor.disable();
    Serial.println("\nAlignment test complete. Motor should spin freely now.\n");
}

//=============================================================================
// MOTOR TEST
//=============================================================================

void runMotorTest(MotorController& mc) {
    Serial.println("\n=== Motor Test ===");

    // Enable
    if (!mc.isEnabled()) {
        mc.enable();
        delay(100);
        if (!mc.isEnabled()) {
            Serial.println("[1/3] FAIL: Motor won't enable");
            return;
        }
    }
    Serial.println("[1/3] Motor enabled");

    // Verify tracking
    mc.updateEncoder();
    BLDCMotor& motor = mc.getMotor();
    float enc = mc.getEncoderDegrees();
    float foc = radiansToDegrees(motor.shaft_angle);
    float err = abs(enc - foc);
    if (err > 5.0) {
        Serial.printf("[2/3] FAIL: Tracking error %.1f°\n", err);
        return;
    }
    Serial.printf("[2/3] Tracking OK (err:%.1f°)\n", err);

    // Move test
    float start = mc.getEncoderDegrees();
    float target = fmod(start + 30.0, 360.0);
    Serial.printf("[3/3] Move +30°: %.1f° → %.1f°\n", start, target);
    mc.moveToPosition(target);

    unsigned long t0 = millis();
    while (millis() - t0 < 3000) {
        mc.update();
        delay(10);
        if ((millis() - t0) % 200 == 0) {
            float pos_err = motor.shaft_angle_sp - motor.shaft_angle;
            Serial.printf("  %ldms FOC:%.1f° Vel:%.0f°/s Err:%.1f° Vq:%.1f\n",
                millis() - t0, radiansToDegrees(motor.shaft_angle),
                radiansToDegrees(motor.shaft_velocity), radiansToDegrees(pos_err), motor.voltage.q);
        }
        if (mc.isAtTarget()) {
            Serial.println("  Reached target");
            break;
        }
    }

    mc.updateEncoder();
    float final_pos = mc.getEncoderDegrees();
    float moved = final_pos - start;
    if (moved < -180) moved += 360;
    if (moved > 180) moved -= 360;
    float final_err = abs(final_pos - target);
    if (final_err > 180) final_err = 360 - final_err;

    Serial.printf("\nResult: Moved %.1f° Error %.1f°\n", abs(moved), final_err);
    if (abs(moved) < 5) Serial.println("FAIL: Motor barely moved");
    else if (final_err > 3) Serial.println("FAIL: Large position error");
    else Serial.println("PASS: Motor working!\n");
}

//=============================================================================
// POSITION SWEEP TEST
//=============================================================================

void runPositionSweepTest(MotorController& mc) {
    Serial.println("\n=== Position Sweep Test ===");
    Serial.println("Testing 5 positions within ±30° (cable-safe range)\n");

    if (!mc.isEnabled()) {
        mc.enable();
        if (!mc.isEnabled()) {
            Serial.println("FAIL: Motor not calibrated");
            return;
        }
    }
    delay(500);

    mc.updateEncoder();
    float start = mc.getEncoderDegrees();
    Serial.printf("Start: %.2f°\n\n", start);

    float offsets[] = {0, -15, -30, 15, 30};
    int passed = 0, failed = 0;
    const float tol = 2.0;

    for (int i = 0; i < 5; i++) {
        float target = fmod(start + offsets[i] + 360, 360.0);
        Serial.printf("[%d/5] Target %.1f° (%+.0f°)...", i+1, target, offsets[i]);

        mc.moveToPosition(target);
        unsigned long t0 = millis();
        bool settled = false;

        while (millis() - t0 < 5000) {
            mc.update();
            delay(10);
            if (mc.isAtTarget()) { settled = true; break; }
        }

        mc.updateEncoder();
        float actual = mc.getEncoderDegrees();
        float err = actual - target;
        if (err > 180) err -= 360;
        if (err < -180) err += 360;

        if (!settled) {
            Serial.printf(" TIMEOUT at %.2f°\n", actual);
            failed++;
        } else if (abs(err) < tol) {
            Serial.printf(" PASS %.2f° (err:%+.2f°)\n", actual, err);
            passed++;
        } else {
            Serial.printf(" FAIL %.2f° (err:%+.2f°)\n", actual, err);
            failed++;
        }
        delay(500);
    }

    Serial.printf("\nResult: %d/5 passed (±%.1f° tolerance)\n", passed, tol);
    if (passed == 5) Serial.println("SUCCESS: Motor control working!\n");
    else if (passed == 0) Serial.println("FAILURE: Motor not responding!\n");
    else Serial.println("PARTIAL: Check PID tuning or mechanics.\n");
}

//=============================================================================
// SIMPLEFOC DIAGNOSTIC - Root cause analysis
//=============================================================================

void runSimpleFOCDiagnostic(MotorController& mc) {
    Serial.println("\n=== SimpleFOC Diagnostic ===\n");

    BLDCMotor& motor = mc.getMotor();
    MT6701Sensor& encoder = mc.getEncoder();

    if (!mc.isCalibrated()) {
        Serial.println("ERROR: Not calibrated. Run 'c' first.");
        return;
    }

    bool t2_pass = false, t3_pass = false, t4_pass = false;
    bool t5_found = false, t6_pass = false, t7_pass = false;
    float t6_move = 0, t7_move = 0, best_offset_deg = 0;

    // T1: Config
    Serial.println("--- T1: Config ---");
    Serial.printf("Mode:%s Dir:%s Vlim:%.1fV Zero:%.1f° Sensor:%c\n\n",
        motor.controller == MotionControlType::angle ? "angle" : "other",
        motor.sensor_direction == Direction::CW ? "CW" : "CCW",
        motor.voltage_limit, radiansToDegrees(motor.zero_electric_angle),
        motor.sensor ? 'Y' : 'N');

    // T2: Sensor calls
    Serial.println("--- T2: Sensor Calls ---");
    mc.enable();
    delay(50);
    encoder.resetCallCount();
    for (int i = 0; i < 100; i++) { motor.loopFOC(); delayMicroseconds(100); }
    unsigned long calls = encoder.getCallCount();
    t2_pass = calls >= 100;
    Serial.printf("loopFOC()x100 → %lu calls: %s\n\n", calls, t2_pass ? "OK" : "FAIL");
    if (!t2_pass) { mc.disable(); return; }

    // T3: Open-loop (use encoder as ground truth, not shaft_angle)
    Serial.println("--- T3: Open-Loop Test ---");
    encoder.update();
    float start_enc_deg = encoder.getDegrees();  // Use encoder, not shaft_angle
    motor.controller = MotionControlType::angle_openloop;
    float ol_target = motor.shaft_angle + degreesToRadians(30.0);
    for (int i = 0; i < 20; i++) { motor.loopFOC(); motor.move(ol_target); delay(100); }
    encoder.update();
    float enc_move = encoder.getDegrees() - start_enc_deg;
    if (enc_move < -180) enc_move += 360;
    if (enc_move > 180) enc_move -= 360;
    t3_pass = abs(enc_move) > 10;
    motor.controller = MotionControlType::angle;
    Serial.printf("Moved %.1f°: %s\n\n", enc_move, t3_pass ? "HW OK" : "FAIL - check wiring");
    if (!t3_pass) { mc.disable(); return; }

    // T4: Phase voltage
    Serial.println("--- T4: Phase Voltage ---");
    encoder.update();
    float start_enc = encoder.getDegrees();
    float test_angles[] = {0, _PI_2, PI, _3PI_2, _2PI};
    Serial.print("Elec: ");
    for (int i = 0; i < 5; i++) {
        motor.setPhaseVoltage(6.0, 0, test_angles[i]);
        delay(300);
        encoder.update();
        float mv = encoder.getDegrees() - start_enc;
        if (mv < -180) mv += 360; if (mv > 180) mv -= 360;
        Serial.printf("%d°→%.0f ", int(radiansToDegrees(test_angles[i])), mv);
    }
    Serial.println();
    motor.setPhaseVoltage(0, 0, 0);
    encoder.update();
    float phase_move = abs(encoder.getDegrees() - start_enc);
    if (phase_move > 180) phase_move = 360 - phase_move;
    t4_pass = phase_move > 5;
    Serial.printf("Total: %.1f°: %s\n\n", phase_move, t4_pass ? "OK" : "FAIL");

    // T5: Zero Angle Offset Search
    Serial.println("--- T5: Zero Offset Search ---");
    Serial.printf("sensor_direction=%s\n", motor.sensor_direction == Direction::CW ? "CW" : "CCW");
    mc.enable();
    delay(50);

    float orig_zero = motor.zero_electric_angle;
    float best_offset = 0, best_correct = 0, best_wrong = 0;
    bool found_correct = false;
    bool expect_positive = (motor.sensor_direction == Direction::CW);

    float offsets[] = {0, PI/4, PI/2, 3*PI/4, PI, 5*PI/4, 3*PI/2, 7*PI/4};
    Serial.print("Testing: ");
    for (int i = 0; i < 8; i++) {
        motor.zero_electric_angle = normalizeRadians(orig_zero + offsets[i]);
        encoder.update();
        start_enc = encoder.getDegrees();

        motor.controller = MotionControlType::velocity;
        for (int j = 0; j < 50; j++) {
            motor.loopFOC();
            motor.move(degreesToRadians(60.0));
            delay(10);
        }

        encoder.update();
        float mv = encoder.getDegrees() - start_enc;
        if (mv < -180) mv += 360; if (mv > 180) mv -= 360;

        bool correct_dir = expect_positive ? (mv > 10) : (mv < -10);
        if (correct_dir && abs(mv) > best_correct) {
            best_correct = abs(mv);
            best_offset = offsets[i];
            found_correct = true;
        }
        if (!correct_dir && abs(mv) > best_wrong) best_wrong = abs(mv);

        Serial.printf("%d°:%.0f ", int(radiansToDegrees(offsets[i])), mv);
        motor.move(0);
        delay(100);
    }
    Serial.println();

    motor.controller = MotionControlType::angle;
    best_offset_deg = radiansToDegrees(best_offset);

    if (found_correct && best_correct > 10) {
        t5_found = true;
        motor.zero_electric_angle = normalizeRadians(orig_zero + best_offset);
        Serial.printf("Best: %.0f° (%.0f° correct) → zero=%.1f° APPLIED\n",
            best_offset_deg, best_correct, radiansToDegrees(motor.zero_electric_angle));
    } else if (best_wrong > 20) {
        Serial.printf("WRONG direction (%.0f°)! Toggle FORCE_SENSOR_DIRECTION_CW\n", best_wrong);
        motor.zero_electric_angle = orig_zero;
    } else {
        Serial.println("No offset worked - try toggling FORCE_SENSOR_DIRECTION_CW");
        motor.zero_electric_angle = orig_zero;
    }

    // Sync state (don't reset - SimpleFOC manages continuous angles)
    motor.loopFOC();
    Serial.printf("\nCurrent shaft_angle: %.1f°\n", radiansToDegrees(motor.shaft_angle));

    // T6: Position Control
    Serial.println("\n--- T6: Position Control (+30°) ---");
    float start_pos = radiansToDegrees(motor.shaft_angle);
    float target_pos = start_pos + 30.0;
    Serial.printf("%.1f° → %.1f°\n", start_pos, target_pos);

    mc.moveToPosition(target_pos);
    float prev = motor.shaft_angle;
    unsigned long t0 = millis();

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) { mc.update(); delay(100); }
        float delta = radiansToDegrees(motor.shaft_angle - prev);
        Serial.printf("  %ldms | %.1f° | %.1f°/s | %.1fV | %+.1f°\n",
            millis() - t0, radiansToDegrees(motor.shaft_angle),
            radiansToDegrees(motor.shaft_velocity), motor.voltage.q, delta);
        prev = motor.shaft_angle;
    }

    float final_pos = radiansToDegrees(motor.shaft_angle);
    t6_move = final_pos - start_pos;
    float t6_err = target_pos - final_pos;
    bool correct_dir = (t6_move > 0) == ((target_pos - start_pos) > 0);
    t6_pass = correct_dir && abs(t6_move) > 15;
    Serial.printf("Move: %+.1f° Err: %.1f° %s\n", t6_move, t6_err,
        (!correct_dir && abs(t6_move) > 10) ? "WRONG DIR!" : (t6_pass ? "OK" : "stalled"));

    // T7: Velocity Mode
    Serial.println("\n--- T7: Velocity Mode (60°/s) ---");
    motor.controller = MotionControlType::velocity;
    float start_shaft = motor.shaft_angle;
    t0 = millis();

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 2; j++) {
            motor.loopFOC();
            motor.move(degreesToRadians(60.0));
            delay(100);
        }
        float mv = radiansToDegrees(motor.shaft_angle - start_shaft);
        Serial.printf("  %ldms | %.1f° | %.1f°/s | %.1fV | %.1f°\n",
            millis() - t0, radiansToDegrees(motor.shaft_angle),
            radiansToDegrees(motor.shaft_velocity), motor.voltage.q, mv);
    }

    float t7_actual = radiansToDegrees(motor.shaft_angle - start_shaft);
    t7_move = abs(t7_actual);
    t7_pass = (t7_actual > 0) && (t7_move > 15);
    motor.controller = MotionControlType::angle;
    Serial.printf("Move: %+.1f° %s\n", t7_actual,
        (t7_actual < 0 && t7_move > 10) ? "WRONG DIR!" : (t7_pass ? "OK" : "slow/stalled"));

    // Summary
    Serial.println("\n=== SUMMARY ===");
    Serial.printf("T2 Sensor:    %s\n", t2_pass ? "OK" : "FAIL");
    Serial.printf("T3 Open-loop: %s\n", t3_pass ? "HW OK" : "FAIL");
    Serial.printf("T4 Phases:    %s\n", t4_pass ? "OK" : "FAIL");
    Serial.printf("T5 Offset:    %s", t5_found ? "found " : "FAIL");
    if (t5_found) Serial.printf("%.0f°", best_offset_deg);
    Serial.println();
    Serial.printf("T6 Position:  %.1f° %s\n", t6_move, t6_pass ? "OK" : "FAIL");
    Serial.printf("T7 Velocity:  %.1f° %s\n", t7_move, t7_pass ? "OK" : "FAIL");

    Serial.println("\nDiagnosis:");
    if (!t3_pass) Serial.println("  → Hardware issue: check wiring/driver/power");
    else if (!t5_found) Serial.println("  → Toggle FORCE_SENSOR_DIRECTION_CW in config.h");
    else if (!t6_pass && !t7_pass) Serial.println("  → Closed-loop failing - check PID gains");
    else if (t6_pass && t7_pass) Serial.println("  → Motor working! Offset applied for session.");
    else Serial.println("  → Partial - check PID tuning");

    if (!t5_found) motor.zero_electric_angle = orig_zero;
    mc.disable();
    Serial.println("\nMotor disabled. Run 'motor_test' to verify.\n");
}

//=============================================================================
// FULL TEST
//=============================================================================

void runFullTest(MotorController& mc) {
    Serial.println("\n=== Full System Test ===\n");

    // Calibration
    Serial.println("Step 1: Calibration");
    if (mc.isCalibrated()) {
        Serial.println("  Already calibrated");
    } else if (!mc.calibrate()) {
        Serial.println("  FAIL: Calibration failed");
        return;
    }
    delay(1000);

    // PID tuning disabled
    Serial.println("\nStep 2: PID Tuning");
    Serial.println("  Skipped (use manual tuning)");
    delay(500);

    // Motor test
    Serial.println("\nStep 3: Movement Test");
    runMotorTest(mc);

    Serial.println("=== Full Test Complete ===\n");
}
