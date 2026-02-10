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
    Serial.println("Control:");
    Serial.println("  e/enable       Enable motor (requires calibration)");
    Serial.println("  d/disable      Disable motor");
    Serial.println("  c/calibrate    Run SimpleFOC calibration");
    Serial.println("  m <deg>/m<deg> Move to position (auto-enables if calibrated)");
    Serial.println("  stop           Emergency stop");
    Serial.println("Streaming:");
    Serial.println("  stream on/off  Encoder data streaming ($ENC, lines)");
    Serial.println("  stream rate N  Set stream rate in Hz (1-500)");
    Serial.println("Info:");
    Serial.println("  s/status       Encoder + FOC state");
    Serial.println("  i/info         System info (chip, pins, config)");
    Serial.println("  scan           I2C bus scan");
    Serial.println("  debug <0/1>    Status output off/on");
    Serial.println("Tests:");
    Serial.println("  test/diag      Full diagnostic (HW+cal+sensor+move)");
    Serial.println("  motor_test     Quick +30 move");
    Serial.println("  sweep          5-position accuracy test");
    Serial.println("  phase_test     Driver phase verification");
    Serial.println("  align          Motor holding strength test");
    Serial.println("  encoder_test   Interactive encoder reading\n");
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
// SYSTEM DIAGNOSTIC - Combined calibration + diagnostics + motor test
//=============================================================================

void runSystemDiagnostic(MotorController& mc) {
    Serial.println("\n========================================");
    Serial.println("       SYSTEM DIAGNOSTIC");
    Serial.println("========================================\n");

    BLDCMotor& motor = mc.getMotor();
    MT6701Sensor& encoder = mc.getEncoder();

    bool t1_pass = false, t2_pass = false, t3_pass = false;
    bool t4_pass = false, t5_pass = false;
    float t4_move = 0;
    int t5_stable = 0, t5_settling = 0, t5_oscillating = 0, t5_chaotic = 0;
    float t5_worst_osc = 0;
    float t5_worst_pos = 0;

    //=========================================================================
    // T1: Hardware Check (I2C, encoder, magnetic field)
    //=========================================================================
    Serial.print("[T1] Hardware Check... ");

    // Check I2C communication
    Wire.beginTransmission(ENCODER_I2C_ADDR);
    byte i2c_error = Wire.endTransmission();
    if (i2c_error != 0) {
        Serial.printf("FAIL (I2C error %d)\n", i2c_error);
        Serial.println("\n>> Check I2C wiring (SDA/SCL) and encoder power\n");
        return;
    }

    // Check magnetic field
    uint8_t field = encoder.getFieldStatus();
    encoder.update();
    float enc_deg = encoder.getDegrees();

    if (field == 0x00 && !isnan(enc_deg)) {
        t1_pass = true;
        Serial.printf("OK (Enc:%.1f° Field:good)\n", enc_deg);
    } else if (field == 0x01) {
        Serial.printf("WARN (Enc:%.1f° Field:STRONG)\n", enc_deg);
        t1_pass = true;  // Can continue but warn
    } else if (field == 0x02) {
        Serial.printf("WARN (Enc:%.1f° Field:WEAK)\n", enc_deg);
        t1_pass = true;  // Can continue but warn
    } else {
        Serial.printf("FAIL (Enc:%.1f° Field:0x%02X)\n", enc_deg, field);
        return;
    }

    //=========================================================================
    // T2: Calibration (SimpleFOC initFOC)
    //=========================================================================
    Serial.print("[T2] Calibration... ");

    if (mc.isCalibrated()) {
        Serial.printf("OK (already calibrated, Dir:%s Zero:%.1f°)\n",
            motor.sensor_direction == Direction::CW ? "CW" : "CCW",
            radiansToDegrees(motor.zero_electric_angle));
        t2_pass = true;
    } else {
        // Run calibration (this prints its own compact status)
        bool cal_ok = mc.calibrate();
        if (cal_ok) {
            t2_pass = true;
            // Calibration already printed status, just confirm
        } else {
            Serial.println("FAIL");
            Serial.println("\n>> Check motor wiring, power supply, and encoder alignment\n");
            return;
        }
    }

    //=========================================================================
    // T3: Sensor Integration (verify loopFOC reads encoder)
    //=========================================================================
    Serial.print("[T3] Sensor Integration... ");

    mc.enable();
    delay(50);
    encoder.resetCallCount();

    // Run 100 FOC loops and count sensor calls
    for (int i = 0; i < 100; i++) {
        motor.loopFOC();
        delayMicroseconds(100);
    }

    unsigned long calls = encoder.getCallCount();
    if (calls >= 100) {
        t3_pass = true;
        Serial.printf("OK (%lu calls/100 loops)\n", calls);
    } else {
        Serial.printf("FAIL (%lu calls - sensor not linked)\n", calls);
        mc.disable();
        return;
    }

    //=========================================================================
    // T4: Open-Loop Test (verify driver/wiring/power)
    //=========================================================================
    Serial.println("[T4] Open-Loop Test... ");

    encoder.update();
    float start_enc = encoder.getDegrees();

    // Save velocity limit
    float saved_vel_limit = motor.velocity_limit;

    // Use velocity_openloop (not angle_openloop) so the field rotates continuously
    // at a steady rate for the full 2s test. angle_openloop stops advancing once the
    // internal shaft_angle reaches the target (~260ms for 30°), leaving the motor
    // with only a static field for the remaining 1.7s — not enough time to move.
    motor.controller = MotionControlType::velocity_openloop;
    float ol_velocity = 1.0;  // rad/s (~57°/s) - moderate for gimbal motor

    // Log pre-test state
    Serial.printf("  Setup: Enc=%.1f° shaft=%.1f° OL_vel=%.1frad/s Vdrive=%.1fV\n",
        start_enc, radiansToDegrees(motor.shaft_angle),
        ol_velocity, motor.voltage_limit);

    // CRITICAL: Do NOT call loopFOC() during open-loop movement!
    // loopFOC() overwrites shaft_angle with the sensor reading each iteration,
    // preventing the open-loop controller from freely advancing the field angle.
    unsigned long t4_start_ms = millis();
    unsigned long last_sample_ms = t4_start_ms;
    int t4_loops = 0;

    while (millis() - t4_start_ms < 2000) {  // 2 second test window
        motor.move(ol_velocity);
        t4_loops++;

        // Log progress every 500ms
        unsigned long now = millis();
        if (now - last_sample_ms >= 500) {
            encoder.update();
            float progress = encoder.getDegrees() - start_enc;
            if (progress < -180) progress += 360;
            if (progress > 180) progress -= 360;
            Serial.printf("  @%lums: moved %.1f° (%d loops)\n",
                now - t4_start_ms, progress, t4_loops);
            last_sample_ms = now;
        }

        delay(1);
    }

    encoder.update();
    t4_move = encoder.getDegrees() - start_enc;
    if (t4_move < -180) t4_move += 360;
    if (t4_move > 180) t4_move -= 360;

    // Restore velocity limit and control mode
    motor.velocity_limit = saved_vel_limit;
    motor.controller = MotionControlType::angle;

    if (abs(t4_move) > 10) {
        t4_pass = true;
        Serial.printf("OK (moved %.1f° in %d loops)\n", t4_move, t4_loops);
    } else {
        Serial.printf("FAIL (moved %.1f° in %d loops)\n", t4_move, t4_loops);
        mc.disable();
        return;
    }

    //=========================================================================
    // Settle after T4 open-loop
    //=========================================================================
    // T4's velocity_openloop leaves the motor spinning with shaft_angle desynced.
    mc.disable();
    delay(500);  // Coast to a stop
    mc.enable();  // Re-syncs shaft_angle from sensor
    for (int i = 0; i < 100; i++) {
        mc.update();
        delay(1);
    }

    //=========================================================================
    // T5: Position Control + Field Uniformity (360° sweep, 15° steps)
    //=========================================================================
    // Combined test: validates closed-loop position control at 24 positions
    // AND maps encoder/field quality around the full rotation.
    // Magnet offset or tilt shows as oscillation concentrated in one arc.
    Serial.println("[T5] Position Control + Field Uniformity (360° sweep)");
    Serial.println("  Pos°    Field  RawNoise  Osc°   Err°   Status");

    for (int step = 0; step < 24; step++) {
        float t5_target = fmod(step * 15.0f, 360.0f);
        mc.moveToPosition(t5_target);

        // Wait for arrival (up to 1.5s)
        unsigned long move_t = millis();
        while (millis() - move_t < 1500) {
            mc.update();
            delay(1);
            if (mc.isAtTarget()) break;
        }

        // Brief settle
        for (int i = 0; i < 100; i++) {
            mc.update();
            delay(1);
        }

        // --- Measure 1: Raw encoder noise (50 rapid I2C reads) ---
        uint16_t raw_min = 16383, raw_max = 0;
        for (int i = 0; i < 50; i++) {
            uint16_t raw = encoder.readRawAngleDirect();
            if (raw < raw_min) raw_min = raw;
            if (raw > raw_max) raw_max = raw;
            delayMicroseconds(500);
        }
        uint16_t raw_spread = raw_max - raw_min;
        if (raw_spread > 8192) raw_spread = 16384 - raw_spread;

        // --- Measure 2: Position oscillation (300ms hold) ---
        float pos_min = 9999.0f, pos_max = -9999.0f;
        unsigned long osc_t = millis();
        while (millis() - osc_t < 300) {
            mc.update();
            float pos = mc.getPosition();
            if (pos < pos_min) pos_min = pos;
            if (pos > pos_max) pos_max = pos;
            delay(1);
        }
        float osc_range = pos_max - pos_min;
        if (osc_range > 180.0f) osc_range = 360.0f - osc_range;

        // --- Measure 3: Position error ---
        encoder.update();
        float actual = encoder.getDegrees();
        float pos_err = actual - t5_target;
        if (pos_err > 180.0f) pos_err -= 360.0f;
        if (pos_err < -180.0f) pos_err += 360.0f;

        // --- Measure 4: Field status ---
        uint8_t field = encoder.getFieldStatus();
        const char* field_str = (field == 0x00) ? "OK" :
                                (field == 0x01) ? "HI" :
                                (field == 0x02) ? "LO" : "??";

        // Classify stability
        const char* status;
        if (osc_range < 0.3f) {
            status = "stable";
            t5_stable++;
        } else if (osc_range < 1.0f) {
            status = "settling";
            t5_settling++;
        } else if (osc_range < 3.0f) {
            status = "OSCILLATING";
            t5_oscillating++;
        } else {
            status = "CHAOTIC";
            t5_chaotic++;
        }

        if (osc_range > t5_worst_osc) {
            t5_worst_osc = osc_range;
            t5_worst_pos = t5_target;
        }

        Serial.printf("  %5.1f   %s     %3d       %4.2f   %+5.2f  %s\n",
            t5_target, field_str, raw_spread, osc_range, pos_err, status);
    }

    // T5 pass criteria: majority stable, no chaotic positions
    int t5_total = t5_stable + t5_settling + t5_oscillating + t5_chaotic;
    t5_pass = (t5_chaotic == 0) && (t5_stable + t5_settling >= t5_total * 3 / 4);

    Serial.printf("  Summary: %d stable, %d settling, %d oscillating, %d chaotic\n",
        t5_stable, t5_settling, t5_oscillating, t5_chaotic);
    if (t5_worst_osc > 0.3f) {
        Serial.printf("  Worst: %.1f° at position %.0f°\n", t5_worst_osc, t5_worst_pos);
    }
    Serial.printf("[T5] %s\n", t5_pass ? "PASS" : "FAIL");

    mc.disable();

    //=========================================================================
    // Summary
    //=========================================================================
    Serial.println("\n----------------------------------------");
    Serial.println("SUMMARY");
    Serial.println("----------------------------------------");
    Serial.printf("  T1 Hardware:    %s\n", t1_pass ? "PASS" : "FAIL");
    Serial.printf("  T2 Calibration: %s\n", t2_pass ? "PASS" : "FAIL");
    Serial.printf("  T3 Sensor:      %s\n", t3_pass ? "PASS" : "FAIL");
    Serial.printf("  T4 Open-Loop:   %s (%.1f°)\n", t4_pass ? "PASS" : "FAIL", t4_move);
    Serial.printf("  T5 Pos+Field:   %s (%d/%d stable, worst %.1f° at %.0f°)\n",
        t5_pass ? "PASS" : "FAIL", t5_stable, t5_total,
        t5_worst_osc, t5_worst_pos);
    Serial.println("----------------------------------------");

    int passed = t1_pass + t2_pass + t3_pass + t4_pass + t5_pass;
    if (passed == 5) {
        Serial.println("RESULT: ALL TESTS PASSED - Motor ready!");
    } else if (passed >= 4) {
        Serial.println("RESULT: PARTIAL - Check failed test above");
    } else {
        Serial.println("RESULT: FAILED - See diagnostics above");
    }
    Serial.println("========================================\n");
}
