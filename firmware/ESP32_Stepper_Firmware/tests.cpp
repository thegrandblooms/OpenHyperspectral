#include "tests.h"
#include "config.h"

//=============================================================================
// HELP / INFO / STATUS
//=============================================================================

void printHelp() {
    Serial.println("\n=== OpenHyperspectral Stepper Controller — Commands ===");
    Serial.println("  c / calibrate    Verify encoder + driver, load backlash from NVS");
    Serial.println("  recalibrate      Measure belt backlash with encoder, save to NVS");
    Serial.println("  e / enable       Enable driver (holds position)");
    Serial.println("  d / disable      Disable driver (free shaft)");
    Serial.println("  m <deg>          Move to absolute output position 0-360");
    Serial.println("  stop             Halt and hold current position");
    Serial.println("  s / status       Position/state snapshot");
    Serial.println("  i / info         System info");
    Serial.println("  test / diag      Full T1-T5 diagnostic suite");
    Serial.println("  motor_test       Quick +30 deg sanity check");
    Serial.println("  position_sweep   5-position accuracy test (+/-30 deg)");
    Serial.println("  encoder_test     Interactive encoder readout (rotate by hand)");
    Serial.println("  scan_step <start> <end> <inc> [dwell_ms]");
    Serial.println("                   Stepped capture scan: settle+trim at each position,");
    Serial.println("                   emit $FRAME,<idx>,<ms>,<target>,<enc>,<err>, dwell");
    Serial.println("  scan_smooth <start> <end> <deg/s>");
    Serial.println("                   Smooth video scan: pre-position (lash in scan dir),");
    Serial.println("                   stream $ENC, constant rate through range");
    Serial.println("  scan_stop        Abort active scan (either mode)");
    Serial.println("  sweep <deg/s>    Unbounded constant-rate sweep (manual testing)");
    Serial.println("  sweep_stop       Stop sweep (or abort scan), hold");
    Serial.println("  sync             Clock sync handshake ($SYNC,<ms>)");
    Serial.println("  stream on|off    $ENC CSV streaming; 'stream rate <hz>'");
    Serial.println("  set <key> <val>  vel_max, accel, cur_lim, pos_tol, vel_thresh,");
    Serial.println("                   trim (0/1), trim_tol");
    Serial.println("  debug 0|1        Toggle periodic status output");
}

void printSystemInfo() {
    Serial.println("\n=== System Info ===");
    Serial.printf("Chip: %s @ %d MHz, free heap %d KB\n",
                  ESP.getChipModel(), ESP.getCpuFreqMHz(), ESP.getFreeHeap() / 1024);
    Serial.println("Architecture: NEMA17 + TMC2209 (STEP/DIR) + belt reduction");
    Serial.printf("  STEP=%d DIR=%d EN=%d  UART %s (TX=%d RX=%d)\n",
                  STEPPER_STEP_PIN, STEPPER_DIR_PIN, STEPPER_EN_PIN,
                  USE_TMC_UART ? "on" : "off", TMC_UART_TX_PIN, TMC_UART_RX_PIN);
    Serial.printf("  Encoder: MT6835 21-bit SPI, CS=%d SCK=%d MISO=%d MOSI=%d\n",
                  ENCODER_CS_PIN, ENCODER_SCK_PIN, ENCODER_MISO_PIN, ENCODER_MOSI_PIN);
    Serial.printf("  Mechanics: %d steps/rev x %d usteps x %.3f ratio = %.0f steps/output-rev\n",
                  MOTOR_FULL_STEPS_PER_REV, MICROSTEPS, GEAR_RATIO, STEPS_PER_OUTPUT_REV);
    Serial.printf("  Resolution: %.5f deg/ustep at output; encoder %.6f deg/count\n",
                  1.0f / STEPS_PER_OUTPUT_DEG, 360.0f / (float)ENCODER_COUNTS_PER_REV);
}

void printStatus(MotorController& m) {
    m.updateEncoder();
    Serial.println("\n=== Status ===");
    Serial.printf("Encoder:   %.4f deg (raw %lu)  vel %.3f deg/s  field:%s  CRC errs:%lu/%lu\n",
                  m.getPosition(), (unsigned long)m.getRawEncoderCount(),
                  m.getCurrentVelocityDegPerSec(),
                  m.getEncoder().isFieldGood() ? "good" : "WEAK",
                  m.getEncoder().getCrcErrorCount(), m.getEncoder().getReadCount());
    Serial.printf("Commanded: %.4f deg (step counter)   target: %.4f deg\n",
                  m.getCommandedPositionDeg(), m.getTargetPositionDeg());
    Serial.printf("State: %s  Enabled:%c  Calibrated:%c  AtTarget:%c  Sweep:%c  Scan:%c\n",
                  m.getState() == STATE_MOVING ? "MOVING" : "IDLE",
                  m.isEnabled() ? 'Y' : 'N', m.isCalibrated() ? 'Y' : 'N',
                  m.isAtTarget() ? 'Y' : 'N', m.isSweepActive() ? 'Y' : 'N',
                  m.isScanActive() ? 'Y' : 'N');
    Serial.printf("Trim: %s (tol %.3f deg)   Backlash: %.4f deg   Current: %.2f A rms\n",
                  m.isTrimEnabled() ? "on" : "off", m.getTrimTolerance(),
                  m.getMeasuredBacklashDeg(), m.getCurrent());
}

//=============================================================================
// MOVE HELPER
//=============================================================================

float waitForMove(MotorController& m, float target_deg, unsigned long timeout_ms) {
    m.moveToPosition(target_deg);
    unsigned long t0 = millis();
    while (millis() - t0 < timeout_ms) {
        m.update();
        if (m.isAtTarget()) break;
        delay(1);
    }
    // Extra settle + fresh read for the final number
    for (int i = 0; i < 100; i++) { m.update(); delay(1); }
    m.updateEncoder();
    return angularDeltaDeg(m.getPosition(), target_deg);   // actual - target
}

//=============================================================================
// FULL DIAGNOSTIC (T1-T5)
//=============================================================================

void runSystemDiagnostic(MotorController& m) {
    int passed = 0, failed = 0;
    Serial.println("\n=== SYSTEM DIAGNOSTIC (stepper) ===");

    // ---- T1: Hardware ----
    Serial.println("\n[T1] Hardware (encoder SPI + driver)");
    bool enc_ok = m.getEncoder().update();
    bool field_ok = m.getEncoder().isFieldGood();
    Serial.printf("  Encoder read: %s, field: %s, pos %.3f deg\n",
                  enc_ok ? "OK" : "FAIL (CRC)", field_ok ? "good" : "WEAK",
                  m.getEncoder().getDegrees());
    if (enc_ok && field_ok) { Serial.println("  T1 PASS"); passed++; }
    else { Serial.println("  T1 FAIL"); failed++; }

    // ---- T2: Calibration ----
    Serial.println("\n[T2] Calibration");
    if (m.calibrate()) { Serial.println("  T2 PASS"); passed++; }
    else { Serial.println("  T2 FAIL"); failed++; }

    // ---- T3: Encoder stability ----
    Serial.println("\n[T3] Encoder stability (100 reads at standstill)");
    float min_d = 1e9, max_d = -1e9;
    int crc_fails = 0;
    for (int i = 0; i < 100; i++) {
        if (m.getEncoder().update()) {
            float d = m.getEncoder().getDegrees();
            if (d < min_d) min_d = d;
            if (d > max_d) max_d = d;
        } else crc_fails++;
        delay(2);
    }
    float noise = max_d - min_d;
    if (noise > 180.0f) noise = 360.0f - noise;   // wrap straddle
    Serial.printf("  Noise p-p: %.5f deg, CRC failures: %d/100\n", noise, crc_fails);
    if (crc_fails <= 2 && noise < 0.05f) { Serial.println("  T3 PASS"); passed++; }
    else { Serial.println("  T3 FAIL"); failed++; }

    // ---- T4: Movement ----
    Serial.println("\n[T4] Movement (+30 deg closed-loop, encoder-verified)");
    if (!m.isEnabled()) m.enable();
    float start = m.getEncoderDegrees();
    float target = normalizeDegrees(start + 30.0f);
    float err = waitForMove(m, target, 15000);
    Serial.printf("  Target %.3f, actual %.3f, error %.4f deg\n",
                  target, m.getPosition(), err);
    if (fabsf(err) <= m.getPositionTolerance()) { Serial.println("  T4 PASS"); passed++; }
    else { Serial.println("  T4 FAIL"); failed++; }

    // ---- T5: 24-position accuracy sweep ----
    Serial.println("\n[T5] Accuracy sweep (24 positions, 15 deg steps)");
    float worst = 0; int t5_fail = 0;
    for (int i = 0; i < 24; i++) {
        float pos = i * 15.0f;
        float e = waitForMove(m, pos, 15000);
        bool ok = fabsf(e) <= m.getPositionTolerance();
        if (!ok) t5_fail++;
        if (fabsf(e) > fabsf(worst)) worst = e;
        Serial.printf("  %5.1f deg -> %s, err=%+.4f deg\n", pos, ok ? "stable" : "FAIL", e);
    }
    Serial.printf("  Worst error: %+.4f deg, failures: %d/24\n", worst, t5_fail);
    if (t5_fail == 0) { Serial.println("  T5 PASS"); passed++; }
    else { Serial.println("  T5 FAIL"); failed++; }

    // ---- Summary (markers parsed by motor_test.py) ----
    Serial.printf("\nSummary: %d passed, %d failed\n", passed, failed);
    if (failed == 0)            Serial.println("RESULT: ALL TESTS PASSED");
    else if (passed > 0)        Serial.println("RESULT: PARTIAL");
    else                        Serial.println("RESULT: FAILED");
}

//=============================================================================
// INDIVIDUAL TESTS
//=============================================================================

void runEncoderTest(MotorController& m) {
    Serial.println("\n=== Encoder Test (10s) — rotate the output shaft by hand ===");
    unsigned long t0 = millis();
    while (millis() - t0 < 10000) {
        m.updateEncoder();
        Serial.printf("raw=%8lu  deg=%9.4f  vel=%8.3f deg/s  field=%s\n",
                      (unsigned long)m.getRawEncoderCount(), m.getPosition(),
                      m.getCurrentVelocityDegPerSec(),
                      m.getEncoder().isFieldGood() ? "ok" : "WEAK");
        delay(200);
    }
    Serial.printf("Done. CRC errors: %lu / %lu reads\n",
                  m.getEncoder().getCrcErrorCount(), m.getEncoder().getReadCount());
}

void runMotorTest(MotorController& m) {
    Serial.println("\n=== Quick Motor Test: +30 deg ===");
    if (!m.isCalibrated() && !m.calibrate()) { Serial.println("Calibration failed"); return; }
    if (!m.isEnabled()) m.enable();
    float target = normalizeDegrees(m.getEncoderDegrees() + 30.0f);
    float err = waitForMove(m, target, 15000);
    Serial.printf("Target %.3f deg, actual %.3f deg, error %+.4f deg — %s\n",
                  target, m.getPosition(), err,
                  fabsf(err) <= m.getPositionTolerance() ? "PASS" : "FAIL");
}

void runPositionSweepTest(MotorController& m) {
    Serial.println("\n=== 5-Position Sweep (+/-30 deg) ===");
    if (!m.isCalibrated() && !m.calibrate()) { Serial.println("Calibration failed"); return; }
    if (!m.isEnabled()) m.enable();
    float origin = m.getEncoderDegrees();
    float offsets[] = {0.0f, -15.0f, -30.0f, 15.0f, 30.0f};
    int pass = 0;
    for (int i = 0; i < 5; i++) {
        float target = normalizeDegrees(origin + offsets[i]);
        float err = waitForMove(m, target, 15000);
        bool ok = fabsf(err) <= m.getPositionTolerance();
        if (ok) pass++;
        Serial.printf("  %+6.1f deg -> err %+.4f deg %s\n", offsets[i], err, ok ? "PASS" : "FAIL");
    }
    Serial.printf("Result: %d/5 PASS\n", pass);
    // Return to origin
    waitForMove(m, normalizeDegrees(origin), 15000);
}
