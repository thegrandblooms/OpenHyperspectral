#include "motor_control.h"
#include <Preferences.h>
#include "driver/gpio.h"
#include "soc/gpio_struct.h"   // GPIO register struct for fast ISR pin writes

#if USE_TMC_UART
#include <TMCStepper.h>   // install "TMCStepper" via Arduino Library Manager
static TMC2209Stepper s_tmc(&Serial1, TMC_R_SENSE, TMC_DRIVER_ADDR);
#endif

static Preferences s_prefs;
static float s_current_rms_a = TMC_RMS_CURRENT_MA / 1000.0f;

//=============================================================================
// STEP GENERATOR — 20kHz hardware-timer ISR with a DDA accumulator
//=============================================================================
// Every tick: accum += rate; when accum rolls over the tick frequency, emit
// one STEP pulse. This gives any step rate from 0 to STEP_TICK_HZ/2 with at
// most one tick (50us) of jitter — invisible after the TMC2209's MicroPlyer
// interpolation and the belt reduction.
//
// ISR-shared state is file-scope (single motor). 64-bit step counter reads
// from task context must take the critical section (not atomic on Xtensa).

static hw_timer_t* s_step_timer = nullptr;
static portMUX_TYPE s_isr_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t s_rate_sps = 0;       // unsigned step rate (steps/s)
static volatile int8_t   s_dir = 1;            // requested direction (+1/-1)
static volatile int8_t   s_dir_applied = 0;    // direction currently on the DIR pin
static volatile int64_t  s_step_count = 0;     // signed microstep position
static volatile uint32_t s_accum = 0;

static inline void IRAM_ATTR fastGpioWrite(uint8_t pin, bool level) {
    if (pin < 32) {
        if (level) GPIO.out_w1ts = (1UL << pin);
        else       GPIO.out_w1tc = (1UL << pin);
    } else {
        if (level) GPIO.out1_w1ts.val = (1UL << (pin - 32));
        else       GPIO.out1_w1tc.val = (1UL << (pin - 32));
    }
}

static void IRAM_ATTR onStepTick() {
    portENTER_CRITICAL_ISR(&s_isr_mux);
    uint32_t rate = s_rate_sps;
    int8_t dir = s_dir;

    if (rate == 0) {
        portEXIT_CRITICAL_ISR(&s_isr_mux);
        return;
    }

    if (dir != s_dir_applied) {
        // Change DIR and let it settle for one full tick before stepping
        // (TMC2209 needs only ~20ns setup; 50us is overkill but free)
        fastGpioWrite(STEPPER_DIR_PIN, dir > 0);
        s_dir_applied = dir;
        s_accum = 0;
        portEXIT_CRITICAL_ISR(&s_isr_mux);
        return;
    }

    s_accum += rate;
    if (s_accum >= (uint32_t)STEP_TICK_HZ) {
        s_accum -= (uint32_t)STEP_TICK_HZ;
        fastGpioWrite(STEPPER_STEP_PIN, true);
        // STEP high width: TMC2209 minimum is ~100ns. ~40 cycles at 240MHz.
        for (int i = 0; i < 40; i++) { __asm__ __volatile__("nop"); }
        fastGpioWrite(STEPPER_STEP_PIN, false);
        s_step_count += dir;
    }
    portEXIT_CRITICAL_ISR(&s_isr_mux);
}

static int64_t readStepCount() {
    portENTER_CRITICAL(&s_isr_mux);
    int64_t c = s_step_count;
    portEXIT_CRITICAL(&s_isr_mux);
    return c;
}

// Continuous output-shaft degrees implied by the step counter, anchored to
// the encoder reading taken at begin().
static float s_cmd_zero_deg = 0.0f;

//=============================================================================
// CONSTRUCTION / SETUP
//=============================================================================

MotorController::MotorController()
    : encoder(ENCODER_CS_PIN),
      move_state(MS_IDLE),
      motor_enabled(false), motor_calibrated(false), auto_enabled(false),
      trim_enabled(TRIM_ENABLED_DEFAULT), trim_tolerance_deg(TRIM_TOLERANCE_DEG),
      user_target_deg(0), profile_target_cont(0), final_target_cont(0),
      overshoot_pending(false), trim_iterations(0), settle_start_ms(0),
      at_target_printed(false), trim_gave_up(false),
      profile_vel_deg_s(0), cruise_vel_deg_s(MAX_VELOCITY_DEG),
      accel_deg_s2(MAX_ACCELERATION_DEG), last_profile_us(0),
      sweep_active(false), sweep_speed_deg_s(0),
      scan_mode(SCAN_NONE), scan_phase(SP_IDLE),
      scan_start_deg(0), scan_end_deg(0), scan_inc_deg(0), scan_dwell_ms(500),
      scan_frame_idx(0), scan_frame_count(0), scan_dwell_start_ms(0),
      scan_speed_signed(0), scan_end_cont(0), scan_dir(1),
      scan_stream_was_enabled(false),
      enc_deg(0), enc_continuous_deg(0), enc_vel_deg_s(0),
      vel_prev_deg(0), vel_prev_us(0), last_command_time(0),
      backlash_deg(0),
      stream_enabled(STREAM_DEFAULT_ENABLED), stream_rate_hz(STREAM_RATE_HZ),
      stream_interval_us(1000000UL / STREAM_RATE_HZ), last_stream_time_us(0),
      pos_tolerance_deg(POSITION_TOLERANCE_DEG),
      vel_threshold_deg_s(VELOCITY_THRESHOLD_DEG),
      approach_sign_current(APPROACH_DIR_POSITIVE ? 1 : -1) {}

void MotorController::begin() {
    pinMode(STEPPER_STEP_PIN, OUTPUT);
    pinMode(STEPPER_DIR_PIN, OUTPUT);
    pinMode(STEPPER_EN_PIN, OUTPUT);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    digitalWrite(STEPPER_DIR_PIN, HIGH);
    digitalWrite(STEPPER_EN_PIN, HIGH);   // active LOW: start disabled

    encoder.begin();
    encoder.update();
    enc_deg = encoder.getDegrees();
    enc_continuous_deg = enc_deg;
    vel_prev_deg = enc_continuous_deg;
    vel_prev_us = micros();

    // Anchor step-counter space to the encoder's boot position
    s_cmd_zero_deg = enc_deg;

#if USE_TMC_UART
    if (tmcInit()) {
        Serial.println("[OK]   TMC2209 UART configured");
    } else {
        Serial.println("[WARN] TMC2209 UART not responding — check wiring/address.");
        Serial.println("       Falling back to STEP/DIR only (set microsteps via MS1/MS2,");
        Serial.println("       current via Vref pot).");
    }
#endif

    // Step generator timer
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    s_step_timer = timerBegin(1000000);                       // 1MHz timebase
    timerAttachInterrupt(s_step_timer, &onStepTick);
    timerAlarm(s_step_timer, 1000000UL / STEP_TICK_HZ, true, 0);
#else
    s_step_timer = timerBegin(0, 80, true);                   // 80MHz/80 = 1MHz
    timerAttachInterrupt(s_step_timer, &onStepTick, true);
    timerAlarmWrite(s_step_timer, 1000000UL / STEP_TICK_HZ, true);
    timerAlarmEnable(s_step_timer);
#endif

    loadBacklashFromNVS();
}

#if USE_TMC_UART
bool MotorController::tmcInit() {
    Serial1.begin(TMC_UART_BAUD, SERIAL_8N1, TMC_UART_RX_PIN, TMC_UART_TX_PIN);
    delay(10);
    s_tmc.begin();
    if (s_tmc.test_connection() != 0) return false;

    s_tmc.pdn_disable(true);         // UART mode (PDN pin no longer powers down)
    s_tmc.I_scale_analog(false);     // current from UART register, not Vref pot
    s_tmc.mstep_reg_select(true);    // microsteps from register, not MS1/MS2 pins
    s_tmc.toff(5);                   // chopper on
    s_tmc.rms_current(TMC_RMS_CURRENT_MA, 0.3f);  // 30% standstill hold current
    s_tmc.microsteps(MICROSTEPS);
    s_tmc.intpol(true);              // MicroPlyer: interpolate to 256 usteps
    s_tmc.en_spreadCycle(false);     // StealthChop: silent, smooth at low speed
    s_tmc.pwm_autoscale(true);
    return true;
}
#else
bool MotorController::tmcInit() { return true; }
#endif

//=============================================================================
// CALIBRATION
//=============================================================================
// Absolute encoder on the output shaft -> no homing needed. "Calibration"
// here means: verify both devices respond, and load the stored backlash.

bool MotorController::calibrate() {
    if (!encoder.update()) {
        Serial.println("[CAL] Encoder read failed (CRC) — check SPI wiring");
        return false;
    }
    if (!encoder.isFieldGood()) {
        Serial.println("[CAL] Encoder magnetic field weak — check magnet gap");
        return false;
    }
#if USE_TMC_UART
    if (s_tmc.test_connection() != 0) {
        Serial.println("[CAL] WARNING: TMC2209 UART not responding (STEP/DIR still works)");
    }
#endif
    loadBacklashFromNVS();
    motor_calibrated = true;
    Serial.printf("[CAL] OK — encoder at %.3f deg, backlash %.3f deg%s\n",
                  encoder.getDegrees(), backlash_deg,
                  backlash_deg == 0.0f ? " (not yet measured — run 'recalibrate')" : "");
    return true;
}

// Measure belt/pulley backlash with the output encoder:
// take up lash in +, then reverse and see how much commanded motion the
// output ignores. Repeat in the other direction and average.
bool MotorController::recalibrate() {
    if (!motor_calibrated && !calibrate()) return false;
    if (!motor_enabled) enable();

    bool trim_was = trim_enabled;
    trim_enabled = false;   // raw open-loop moves for this measurement

    const float LEG_DEG = 2.0f;
    float lash_sum = 0.0f;
    int lash_n = 0;

    auto settleAndRead = [&]() -> float {
        unsigned long t0 = millis();
        while (move_state == MS_MOVING || move_state == MS_SETTLING || move_state == MS_TRIMMING) {
            update();
            delay(1);
            if (millis() - t0 > 10000) break;
        }
        delay(TRIM_SETTLE_MS);
        updateEncoder();
        return enc_continuous_deg;
    };

    // Take up lash in + direction first
    beginProfileTo(commandedContinuousDeg() + LEG_DEG);
    move_state = MS_MOVING;
    float p0 = settleAndRead();

    // Reverse: output should move LEG_DEG minus the lash
    beginProfileTo(commandedContinuousDeg() - LEG_DEG);
    move_state = MS_MOVING;
    float p1 = settleAndRead();
    float moved = p0 - p1;
    if (moved > 0.1f && moved <= LEG_DEG) { lash_sum += (LEG_DEG - moved); lash_n++; }

    // Forward again for the second estimate
    beginProfileTo(commandedContinuousDeg() + LEG_DEG);
    move_state = MS_MOVING;
    float p2 = settleAndRead();
    moved = p2 - p1;
    if (moved > 0.1f && moved <= LEG_DEG) { lash_sum += (LEG_DEG - moved); lash_n++; }

    trim_enabled = trim_was;

    if (lash_n == 0) {
        Serial.println("[CAL] Backlash measurement failed — encoder didn't track moves.");
        Serial.println("      Check GEAR_RATIO, MICROSTEPS, and encoder mounting.");
        return false;
    }
    backlash_deg = lash_sum / lash_n;
    saveBacklashToNVS();
    Serial.printf("[CAL] Measured backlash: %.4f deg (saved to NVS)\n", backlash_deg);
    return true;
}

void MotorController::loadBacklashFromNVS() {
    s_prefs.begin("stepper", true);
    backlash_deg = s_prefs.getFloat("backlash", 0.0f);
    s_prefs.end();
}

void MotorController::saveBacklashToNVS() {
    s_prefs.begin("stepper", false);
    s_prefs.putFloat("backlash", backlash_deg);
    s_prefs.end();
}

//=============================================================================
// ENABLE / DISABLE / STOP
//=============================================================================

void MotorController::enable() {
    digitalWrite(STEPPER_EN_PIN, LOW);   // active low
    motor_enabled = true;
    if (move_state == MS_IDLE) move_state = MS_HOLDING;
    // Sync command space to wherever the shaft actually is (it may have been
    // moved by hand while disabled)
    updateEncoder();
    portENTER_CRITICAL(&s_isr_mux);
    s_step_count = 0;
    s_accum = 0;
    portEXIT_CRITICAL(&s_isr_mux);
    s_cmd_zero_deg = enc_continuous_deg;
    user_target_deg = normalizeDegrees(enc_deg);
}

void MotorController::disable() {
    abortScan();
    haltSteps();
    sweep_active = false;
    digitalWrite(STEPPER_EN_PIN, HIGH);
    motor_enabled = false;
    move_state = MS_IDLE;
}

void MotorController::stop() {
    // Halt immediately and hold here. At our step rates the instantaneous
    // stop is well within the motor's pull-in torque — no decel ramp needed.
    abortScan();
    haltSteps();
    sweep_active = false;
    updateEncoder();
    user_target_deg = normalizeDegrees(enc_deg);
    move_state = motor_enabled ? MS_HOLDING : MS_IDLE;
}

void MotorController::setHome() {
    updateEncoder();
    Serial.printf("[HOME] Absolute encoder — current position %.3f deg (no homing needed)\n", enc_deg);
}

//=============================================================================
// MOVE PLANNING
//=============================================================================

float MotorController::commandedContinuousDeg() {
    return s_cmd_zero_deg + (float)readStepCount() / STEPS_PER_OUTPUT_DEG;
}

float MotorController::getCommandedPositionDeg() {
    return normalizeDegrees(commandedContinuousDeg());
}

void MotorController::moveToPosition(float absolute_deg) {
    if (!motor_enabled) {
        Serial.println("[MOVE] Motor not enabled");
        return;
    }
    if (isScanActive()) {
        Serial.println("[MOVE] Scan active — 'scan_stop' first");
        return;
    }
    planMove(normalizeDegrees(absolute_deg));
}

void MotorController::planMove(float target_deg_0_360, int8_t approach_override) {
    updateEncoder();
    user_target_deg = target_deg_0_360;
    trim_iterations = 0;
    at_target_printed = false;
    trim_gave_up = false;
    approach_sign_current = (approach_override != 0)
        ? approach_override
        : (APPROACH_DIR_POSITIVE ? 1 : -1);

    // Shortest path from where the OUTPUT actually is, executed in cmd space
    float delta = angularDeltaDeg(target_deg_0_360, enc_deg);
    float cmd_now = commandedContinuousDeg();
    final_target_cont = cmd_now + delta;

    bool needs_overshoot = false;
#if UNIDIRECTIONAL_APPROACH
    if (delta * approach_sign_current < 0) needs_overshoot = true;
#endif

    if (needs_overshoot) {
        // Go past the target against the approach direction, then come back
        // so the final leg always loads the belt the same way.
        overshoot_pending = true;
        beginProfileTo(final_target_cont - approach_sign_current * BACKLASH_OVERSHOOT_DEG);
    } else {
        overshoot_pending = false;
        beginProfileTo(final_target_cont);
    }
    move_state = MS_MOVING;
}

void MotorController::beginProfileTo(float target_cont) {
    profile_target_cont = target_cont;
    last_profile_us = micros();
    // profile_vel_deg_s carries over if already moving (smooth transition)
}

//=============================================================================
// UPDATE — call every loop() pass
//=============================================================================

void MotorController::update() {
    // Encoder + velocity estimate at ~500Hz (SPI burst read is ~60us)
    unsigned long now_us = micros();
    if (now_us - vel_prev_us >= 2000) {
        updateEncoder();
        float dt = (now_us - vel_prev_us) * 1e-6f;
        float raw_vel = (enc_continuous_deg - vel_prev_deg) / dt;
        // ~50ms LPF: plenty of bandwidth for status/settling checks
        const float alpha = 0.04f;
        enc_vel_deg_s += alpha * (raw_vel - enc_vel_deg_s);
        vel_prev_deg = enc_continuous_deg;
        vel_prev_us = now_us;
    }

    // Scan sequencer rides on top of the move/sweep machinery below
    runScan();

    if (sweep_active) {
        // Constant-rate sweep: slew the step rate toward the commanded speed
        float dt = (now_us - last_profile_us) * 1e-6f;
        last_profile_us = now_us;
        float dv = accel_deg_s2 * dt;
        if (profile_vel_deg_s < sweep_speed_deg_s)
            profile_vel_deg_s = min(profile_vel_deg_s + dv, sweep_speed_deg_s);
        else
            profile_vel_deg_s = max(profile_vel_deg_s - dv, sweep_speed_deg_s);
        setStepRate(profile_vel_deg_s);
    } else {
        switch (move_state) {
            case MS_MOVING:
                runProfile();
                break;
            case MS_SETTLING:
                if (millis() - settle_start_ms >= TRIM_SETTLE_MS) {
                    move_state = MS_TRIMMING;
                }
                break;
            case MS_TRIMMING:
                evaluateTrim();
                break;
            case MS_HOLDING:
            case MS_IDLE:
            default:
                break;
        }
    }

    emitStreamLine();
}

void MotorController::runProfile() {
    unsigned long now_us = micros();
    float dt = (now_us - last_profile_us) * 1e-6f;
    if (dt <= 0) return;
    last_profile_us = now_us;

    float cmd = commandedContinuousDeg();
    float remaining = profile_target_cont - cmd;
    float half_step = 0.5f / STEPS_PER_OUTPUT_DEG;

    if (fabsf(remaining) <= half_step) {
        haltSteps();
        profile_vel_deg_s = 0;
        settle_start_ms = millis();
        move_state = MS_SETTLING;
        return;
    }

    float dir = (remaining > 0) ? 1.0f : -1.0f;
    float min_rate_deg = MIN_STEP_RATE_HZ / STEPS_PER_OUTPUT_DEG;

    // Velocity that lets us stop at the target with accel_deg_s2
    float v_stop = sqrtf(2.0f * accel_deg_s2 * fabsf(remaining));
    float v_des = dir * max(min_rate_deg, min(cruise_vel_deg_s, v_stop));

    // Accel-limited slew toward desired velocity
    float dv = accel_deg_s2 * dt;
    if (profile_vel_deg_s < v_des) profile_vel_deg_s = min(profile_vel_deg_s + dv, v_des);
    else                           profile_vel_deg_s = max(profile_vel_deg_s - dv, v_des);

    setStepRate(profile_vel_deg_s);
}

void MotorController::evaluateTrim() {
    updateEncoder();

    if (overshoot_pending) {
        // Overshoot leg done — run the final approach leg
        overshoot_pending = false;
        beginProfileTo(final_target_cont);
        move_state = MS_MOVING;
        return;
    }

    float err = angularDeltaDeg(user_target_deg, enc_deg);   // target - actual

    if (!trim_enabled || fabsf(err) <= trim_tolerance_deg) {
        move_state = MS_HOLDING;
        return;
    }

    if (trim_iterations >= TRIM_MAX_ITERATIONS) {
        if (!trim_gave_up) {
            trim_gave_up = true;
            Serial.printf("[TRIM] Gave up after %d iterations — residual %.4f deg "
                          "(tol %.4f). Check mechanics.\n",
                          TRIM_MAX_ITERATIONS, err, trim_tolerance_deg);
        }
        move_state = MS_HOLDING;
        return;
    }

    trim_iterations++;
    float corr = constrain(err, -TRIM_MAX_CORRECTION_DEG, TRIM_MAX_CORRECTION_DEG);
    float cmd_now = commandedContinuousDeg();
    final_target_cont = cmd_now + corr;

#if UNIDIRECTIONAL_APPROACH
    if (corr * approach_sign_current < 0) {
        // Correction goes against the approach direction: overshoot + return
        overshoot_pending = true;
        beginProfileTo(final_target_cont - approach_sign_current * BACKLASH_OVERSHOOT_DEG);
        move_state = MS_MOVING;
        return;
    }
#endif
    beginProfileTo(final_target_cont);
    move_state = MS_MOVING;
}

//=============================================================================
// SCAN MODES
//=============================================================================
// Two capture strategies, selectable per scan, so they can be compared on
// the same hardware with the same encoder logging:
//
//   STEPPED  move -> settle -> trim to TRIM_TOLERANCE_DEG -> $FRAME marker
//            -> dwell (host captures) -> next. Encoder-verified position at
//            every frame; slow (~1-2 frames/s including settle+trim).
//
//   SMOOTH   constant step rate through the range while $ENC streams; host
//            records video and frame-matches against the encoder log using
//            the 'sync' clock handshake. With StealthChop + MicroPlyer the
//            TMC2209 interpolates each 1/16 microstep into 1/256 increments
//            spread over the step interval, so constant-rate motion is
//            quasi-continuous — matching precision comes from the encoder
//            log, not the step quantum.
//            Speed guide for a 30fps camera: deg/s = bin_deg * 30 / frames
//            per bin -> 0.15-0.6 deg/s for 0.02 deg bins with 1-4 frames.
//
// Both modes take up belt lash IN the scan direction before the first
// capture (forced approach direction), so lash never appears mid-scan.

bool MotorController::startSteppedScan(float start_deg, float end_deg,
                                       float inc_deg, uint32_t dwell_ms) {
    if (!motor_enabled) { Serial.println("[SCAN] Motor not enabled"); return false; }
    if (isScanActive()) { Serial.println("[SCAN] Scan already active — scan_stop first"); return false; }
    if (inc_deg <= 0.0f) { Serial.println("[SCAN] Increment must be > 0"); return false; }

    float span = angularDeltaDeg(end_deg, start_deg);   // shortest path, <180 deg
    if (fabsf(span) < inc_deg) { Serial.println("[SCAN] Range smaller than increment"); return false; }

    scan_mode = SCAN_STEPPED;
    scan_phase = SP_PREPOSITION;
    scan_start_deg = normalizeDegrees(start_deg);
    scan_end_deg = normalizeDegrees(end_deg);
    scan_dir = (span >= 0) ? 1 : -1;
    scan_inc_deg = scan_dir * inc_deg;
    scan_dwell_ms = dwell_ms;
    scan_frame_idx = 0;
    scan_frame_count = (int)(fabsf(span) / inc_deg) + 1;

    Serial.printf("$SCAN_START,%lu,step,%.4f,%.4f,%.4f,%lu,%d\n",
                  millis(), scan_start_deg, scan_end_deg, scan_inc_deg,
                  (unsigned long)scan_dwell_ms, scan_frame_count);
    planMove(scan_start_deg, scan_dir);   // approach in scan direction
    return true;
}

bool MotorController::startSmoothScan(float start_deg, float end_deg,
                                      float speed_deg_s) {
    if (!motor_enabled) { Serial.println("[SCAN] Motor not enabled"); return false; }
    if (isScanActive()) { Serial.println("[SCAN] Scan already active — scan_stop first"); return false; }
    speed_deg_s = fabsf(speed_deg_s);
    if (speed_deg_s <= 0.0f || speed_deg_s > SWEEP_MAX_DEG_S) {
        Serial.printf("[SCAN] Speed must be 0 < v <= %.1f deg/s\n", (float)SWEEP_MAX_DEG_S);
        return false;
    }

    float span = angularDeltaDeg(end_deg, start_deg);   // shortest path, <180 deg
    if (fabsf(span) < 0.1f) { Serial.println("[SCAN] Range too small"); return false; }

    scan_mode = SCAN_SMOOTH;
    scan_phase = SP_PREPOSITION;
    scan_start_deg = normalizeDegrees(start_deg);
    scan_end_deg = normalizeDegrees(end_deg);
    scan_dir = (span >= 0) ? 1 : -1;
    scan_speed_signed = scan_dir * speed_deg_s;

    Serial.printf("[SCAN] Smooth: %.3f -> %.3f deg at %.3f deg/s (~%.0f s, %.1f steps/s)\n",
                  scan_start_deg, scan_end_deg, scan_speed_signed,
                  fabsf(span) / speed_deg_s, speed_deg_s * STEPS_PER_OUTPUT_DEG);
    planMove(scan_start_deg, scan_dir);   // take up lash in the travel direction
    return true;
}

void MotorController::abortScan() {
    if (!isScanActive()) return;
    if (scan_mode == SCAN_SMOOTH && scan_phase == SP_RUN) {
        sweep_active = false;
        haltSteps();
        profile_vel_deg_s = 0;
        setStreamEnabled(scan_stream_was_enabled);
    }
    Serial.printf("$SCAN_ABORT,%lu,frame=%d\n", millis(), scan_frame_idx);
    scan_mode = SCAN_NONE;
    scan_phase = SP_IDLE;
    updateEncoder();
    user_target_deg = normalizeDegrees(enc_deg);
    move_state = motor_enabled ? MS_HOLDING : MS_IDLE;
}

void MotorController::runScan() {
    if (!isScanActive()) return;

    switch (scan_phase) {
        case SP_PREPOSITION:
            if (move_state != MS_HOLDING) return;   // still moving to start
            if (scan_mode == SCAN_STEPPED) {
                // Start position reached — it is frame 0
                scan_dwell_start_ms = millis();
                updateEncoder();
                Serial.printf("$FRAME,%d,%lu,%.4f,%.4f,%+.4f\n",
                              scan_frame_idx, millis(), user_target_deg, enc_deg,
                              angularDeltaDeg(enc_deg, user_target_deg));
                scan_phase = SP_DWELL;
            } else {
                // SMOOTH: lash is loaded — start recording and rolling
                scan_stream_was_enabled = stream_enabled;
                setStreamEnabled(true);
                updateEncoder();
                scan_end_cont = commandedContinuousDeg()
                              + angularDeltaDeg(scan_end_deg, enc_deg);
                Serial.printf("$SCAN_START,%lu,smooth,%.4f,%.4f,%.4f\n",
                              millis(), scan_start_deg, scan_end_deg, scan_speed_signed);
                sweep_speed_deg_s = scan_speed_signed;
                sweep_active = true;
                last_profile_us = micros();
                scan_phase = SP_RUN;
            }
            return;

        case SP_MOVE:   // stepped: waiting for settle+trim at a frame position
            if (move_state != MS_HOLDING) return;
            scan_dwell_start_ms = millis();
            updateEncoder();
            Serial.printf("$FRAME,%d,%lu,%.4f,%.4f,%+.4f\n",
                          scan_frame_idx, millis(), user_target_deg, enc_deg,
                          angularDeltaDeg(enc_deg, user_target_deg));
            scan_phase = SP_DWELL;
            return;

        case SP_DWELL:  // stepped: host capture window
            if (millis() - scan_dwell_start_ms < scan_dwell_ms) return;
            scan_frame_idx++;
            if (scan_frame_idx >= scan_frame_count) {
                Serial.printf("$SCAN_END,%lu,frames=%d\n", millis(), scan_frame_count);
                scan_mode = SCAN_NONE;
                scan_phase = SP_IDLE;
                return;
            }
            planMove(normalizeDegrees(scan_start_deg + scan_frame_idx * scan_inc_deg),
                     scan_dir);
            scan_phase = SP_MOVE;
            return;

        case SP_RUN: {  // smooth: constant rate until the end of the range
            float cmd = commandedContinuousDeg();
            bool done = (scan_dir > 0) ? (cmd >= scan_end_cont)
                                       : (cmd <= scan_end_cont);
            if (!done) return;
            sweep_active = false;
            haltSteps();
            profile_vel_deg_s = 0;
            updateEncoder();
            Serial.printf("$SCAN_END,%lu,%.4f\n", millis(), enc_deg);
            setStreamEnabled(scan_stream_was_enabled);
            user_target_deg = normalizeDegrees(enc_deg);
            move_state = MS_HOLDING;
            scan_mode = SCAN_NONE;
            scan_phase = SP_IDLE;
            return;
        }

        case SP_IDLE:
        default:
            return;
    }
}

//=============================================================================
// SWEEP (continuous-rate motion, unbounded — kept for manual testing)
//=============================================================================

void MotorController::startSweep(float speed_deg_s) {
    if (!motor_enabled) {
        Serial.println("[SWEEP] Motor not enabled");
        return;
    }
    if (isScanActive()) {
        Serial.println("[SWEEP] Scan active — 'scan_stop' first");
        return;
    }
    speed_deg_s = constrain(speed_deg_s, -SWEEP_MAX_DEG_S, SWEEP_MAX_DEG_S);
    sweep_speed_deg_s = speed_deg_s;
    sweep_active = true;
    last_profile_us = micros();
    Serial.printf("$SWEEP_START,%lu\n", millis());
    Serial.printf("[SWEEP] %.3f deg/s (%.1f steps/s)\n",
                  speed_deg_s, fabsf(speed_deg_s) * STEPS_PER_OUTPUT_DEG);
}

void MotorController::stopSweep() {
    if (isScanActive()) {   // 'sweep_stop' during a smooth scan = abort the scan
        abortScan();
        return;
    }
    sweep_active = false;
    haltSteps();
    profile_vel_deg_s = 0;
    updateEncoder();
    user_target_deg = normalizeDegrees(enc_deg);
    move_state = MS_HOLDING;
    Serial.printf("$SWEEP_END,%lu\n", millis());
}

//=============================================================================
// STEP RATE INTERFACE
//=============================================================================

void MotorController::setStepRate(float deg_s) {
    float sps = deg_s * STEPS_PER_OUTPUT_DEG;
    uint32_t rate = (uint32_t)(fabsf(sps) + 0.5f);
    int8_t dir = (sps >= 0) ? 1 : -1;
    portENTER_CRITICAL(&s_isr_mux);
    s_rate_sps = rate;
    s_dir = dir;
    portEXIT_CRITICAL(&s_isr_mux);
}

void MotorController::haltSteps() {
    portENTER_CRITICAL(&s_isr_mux);
    s_rate_sps = 0;
    s_accum = 0;
    portEXIT_CRITICAL(&s_isr_mux);
}

//=============================================================================
// CONFIGURATION
//=============================================================================

void MotorController::setVelocity(float velocity_deg_s) {
    cruise_vel_deg_s = constrain(fabsf(velocity_deg_s), 0.1f, (float)MAX_VELOCITY_DEG);
}

void MotorController::setAcceleration(float accel) {
    accel_deg_s2 = constrain(fabsf(accel), 1.0f, (float)MAX_ACCELERATION_DEG * 4.0f);
}

void MotorController::setCurrentLimit(float amps_rms) {
    s_current_rms_a = constrain(amps_rms, 0.1f, 2.0f);
#if USE_TMC_UART
    s_tmc.rms_current((uint16_t)(s_current_rms_a * 1000.0f), 0.3f);
    Serial.printf("[TMC] RMS current set to %.0f mA\n", s_current_rms_a * 1000.0f);
#else
    Serial.println("[TMC] UART disabled — current is set by the Vref pot");
#endif
}

void MotorController::setControlMode(uint8_t mode) {
    if (mode != MODE_POSITION) {
        Serial.println("[MODE] Stepper firmware is position-control only");
    }
}

void MotorController::setPositionTolerance(float deg) {
    pos_tolerance_deg = max(0.001f, deg);
}

void MotorController::setVelocityThreshold(float deg_s) {
    vel_threshold_deg_s = max(0.01f, deg_s);
}

//=============================================================================
// STATUS
//=============================================================================

void MotorController::updateEncoder() {
    if (!encoder.update()) return;   // CRC fail: keep previous reading
    float new_deg = encoder.getDegrees();
    enc_continuous_deg += angularDeltaDeg(new_deg, enc_deg);  // unwrap
    enc_deg = new_deg;
}

float MotorController::getPosition()              { return enc_deg; }
uint32_t MotorController::getRawEncoderCount()    { return encoder.getRawCount(); }
float MotorController::getEncoderDegrees()        { updateEncoder(); return enc_deg; }
float MotorController::getCurrentVelocityDegPerSec() { return enc_vel_deg_s; }
float MotorController::getTargetPositionDeg()     { return user_target_deg; }
float MotorController::getCurrent()               { return s_current_rms_a; }
float MotorController::getVoltage()               { return 0.0f; }

bool MotorController::isAtTarget() {
    if (move_state != MS_HOLDING) return false;
    float err = angularDeltaDeg(user_target_deg, enc_deg);
    return fabsf(err) <= pos_tolerance_deg && fabsf(enc_vel_deg_s) <= vel_threshold_deg_s;
}

uint8_t MotorController::getState() {
    if (!motor_enabled) return STATE_IDLE;
    if (isScanActive() || sweep_active || move_state == MS_MOVING ||
        move_state == MS_SETTLING || move_state == MS_TRIMMING) return STATE_MOVING;
    return STATE_IDLE;
}

//=============================================================================
// STREAMING ($ENC tagged CSV — same transport as gimbal firmware)
//=============================================================================

void MotorController::setStreamEnabled(bool enabled) {
    stream_enabled = enabled;
    last_stream_time_us = micros();
}

void MotorController::setStreamRate(uint16_t hz) {
    stream_rate_hz = constrain(hz, (uint16_t)STREAM_MIN_RATE_HZ, (uint16_t)STREAM_MAX_RATE_HZ);
    stream_interval_us = 1000000UL / stream_rate_hz;
}

void MotorController::emitStreamLine() {
    if (!stream_enabled) return;
    unsigned long now_us = micros();
    if (now_us - last_stream_time_us < stream_interval_us) return;
    last_stream_time_us = now_us;

    float cmd_pos = getCommandedPositionDeg();
    float trim_err = angularDeltaDeg(enc_deg, cmd_pos);   // encoder - commanded

    // $ENC,<ms>,<enc_pos>,<enc_vel>,<target>,<cmd_pos>,<trim_err>
    Serial.print("$ENC,");
    Serial.print(millis());          Serial.print(',');
    Serial.print(enc_deg, 4);        Serial.print(',');
    Serial.print(enc_vel_deg_s, 3);  Serial.print(',');
    Serial.print(user_target_deg, 4); Serial.print(',');
    Serial.print(cmd_pos, 4);        Serial.print(',');
    Serial.println(trim_err, 4);
}

void MotorController::emitScanStart() {
    Serial.printf("$SCAN_START,%lu\n", millis());
}

void MotorController::emitScanEnd() {
    Serial.printf("$SCAN_END,%lu\n", millis());
}
