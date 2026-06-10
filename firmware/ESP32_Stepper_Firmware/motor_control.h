#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "MT6835.h"
#include "config.h"
#include "commands.h"

//=============================================================================
// STEPPER MOTOR CONTROLLER (TMC2209 + belt reduction + MT6835 output encoder)
//=============================================================================
//
// ARCHITECTURE:
//
//   moveToPosition(deg)                       MT6835 (21-bit, OUTPUT shaft)
//        |                                          ^
//        v                                          | SPI (truth source)
//   [planner: unidirectional approach,              |
//    backlash overshoot]                            |
//        |                                          |
//        v                                          |
//   [trapezoid profile, update() @ ~1kHz]           |
//        |                                          |
//        v                                          |
//   [step generator: 20kHz timer ISR, DDA]          |
//        |                                          |
//        v          STEP/DIR                        |
//   TMC2209 ----------------------> NEMA17 --belt--> output shaft
//
//   After every move: SETTLE -> read encoder -> TRIM residual error with a
//   small correction move (up to TRIM_MAX_ITERATIONS) -> HOLD passively.
//
// WHY NO CONTINUOUS FEEDBACK LOOP:
//   A stepper at standstill is passively stable — holding requires no control
//   action, so the hunting/limit-cycle failure mode of the FOC gimbal design
//   is structurally absent. The encoder closes the loop BETWEEN moves
//   (move-then-trim), which compensates belt lash, belt compliance, and
//   microstep nonlinearity without ever dithering at rest.
//
// UNITS: user-facing positions are OUTPUT-shaft degrees (0-360), measured by
// the MT6835. Internally the planner works in continuous (unwrapped) degrees.
//=============================================================================

inline float normalizeDegrees(float degrees) {
    degrees = fmodf(degrees, 360.0f);
    if (degrees < 0) degrees += 360.0f;
    return degrees;
}

// Shortest signed angular difference a-b, result in (-180, 180]
inline float angularDeltaDeg(float a, float b) {
    float d = fmodf(a - b, 360.0f);
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

class MotorController {
public:
    MotorController();

    // Setup
    void begin();
    bool calibrate();      // Verify encoder + driver comms, load backlash from NVS
    bool recalibrate();    // Measure backlash with the encoder, save to NVS

    // Control
    void enable();
    void disable();
    void stop();                              // Halt profile, hold current position
    void moveToPosition(float absolute_deg);  // Absolute output-shaft position 0-360
    void startSweep(float speed_deg_s);       // Continuous-rate sweep, unbounded
    void stopSweep();
    bool isSweepActive() const { return sweep_active; }
    void update();                            // Call every loop() pass

    //=========================================================================
    // SCAN MODES (the two capture strategies under evaluation)
    //=========================================================================
    // STEPPED: move -> settle -> trim -> emit $FRAME marker -> dwell (host
    //   captures during dwell) -> next position. Final approach is forced
    //   into the scan direction at every frame so belt lash is loaded
    //   identically for every capture.
    // SMOOTH: pre-position to the start point approaching IN the scan
    //   direction (lash taken up before recording), auto-enable $ENC
    //   streaming, emit $SCAN_START, run a constant step rate to the end,
    //   emit $SCAN_END. Host records video + encoder log and frame-matches
    //   afterwards (clock sync via the 'sync' command).
    // Both limited to ranges < 180 deg (shortest-path interpretation).
    bool startSteppedScan(float start_deg, float end_deg, float inc_deg,
                          uint32_t dwell_ms);
    bool startSmoothScan(float start_deg, float end_deg, float speed_deg_s);
    void abortScan();                         // Safe no-op when no scan active
    bool isScanActive() const { return scan_mode != SCAN_NONE; }

    void setHome();   // Informational — absolute encoder needs no homing

    // Configuration (protocol compatibility with gimbal firmware)
    void setVelocity(float velocity_deg_s);
    void setAcceleration(float accel_deg_s2);
    void setCurrentLimit(float amps_rms);     // Forwarded to TMC2209 over UART
    void setControlMode(uint8_t mode);        // Accepted but stepper is position-only

    // Closed-loop trim configuration
    void setTrimEnabled(bool en)  { trim_enabled = en; }
    bool isTrimEnabled() const    { return trim_enabled; }
    void setTrimTolerance(float deg) { trim_tolerance_deg = max(0.001f, deg); }
    float getTrimTolerance() const   { return trim_tolerance_deg; }

    // Runtime thresholds
    void setPositionTolerance(float deg);
    void setVelocityThreshold(float deg_s);
    float getPositionTolerance() const { return pos_tolerance_deg; }
    float getVelocityThreshold() const { return vel_threshold_deg_s; }

    // Status getters (positions are OUTPUT encoder truth)
    float getPosition();                  // Encoder absolute position 0-360
    float getAbsolutePositionDeg()  { return getPosition(); }
    float getCurrentPositionDeg()   { return getPosition(); }
    float getCommandedPositionDeg();      // Open-loop step-counter position 0-360
    float getCurrentVelocityDegPerSec();  // Encoder-derived
    float getTargetPositionDeg();
    float getCurrent();                   // Configured RMS current (no sensing)
    float getVoltage();                   // Always 0 (kept for protocol compat)
    bool isEnabled()    { return motor_enabled; }
    bool isCalibrated() { return motor_calibrated; }
    bool isAtTarget();
    uint8_t getState();
    uint8_t getControlMode() { return MODE_POSITION; }

    // Direct encoder access
    void updateEncoder();
    uint32_t getRawEncoderCount();
    float getEncoderDegrees();
    MT6835& getEncoder() { return encoder; }

    // Backlash
    float getMeasuredBacklashDeg() const { return backlash_deg; }

    // Encoder streaming (tagged CSV, same protocol as gimbal firmware)
    void setStreamEnabled(bool enabled);
    bool isStreamEnabled() { return stream_enabled; }
    void setStreamRate(uint16_t hz);
    uint16_t getStreamRate() { return stream_rate_hz; }
    void emitStreamLine();
    void emitScanStart();
    void emitScanEnd();

    // Idle handling (kept for .ino compatibility; stepper default is HOLD —
    // TMC2209 standstill current reduction makes holding cheap and silent)
    void notifyCommandActivity() { last_command_time = millis(); }
    void checkIdleDisable() {}    // Intentionally a no-op: never drop holding torque
    void setAutoEnabled(bool en) { auto_enabled = en; }
    bool isAutoEnabled() { return auto_enabled; }

private:
    MT6835 encoder;

    // --- State machine ---
    enum MoveState : uint8_t {
        MS_IDLE,        // Disabled or never moved
        MS_MOVING,      // Trapezoid profile running toward profile_target
        MS_SETTLING,    // Motion done, waiting TRIM_SETTLE_MS before encoder read
        MS_TRIMMING,    // Evaluating/applying a correction move
        MS_HOLDING      // At target, passive hold
    };
    MoveState move_state;

    bool motor_enabled;
    bool motor_calibrated;
    bool auto_enabled;
    bool trim_enabled;
    float trim_tolerance_deg;

    // --- Targets (continuous/unwrapped output degrees) ---
    float user_target_deg;        // What the user asked for (0-360, encoder space)
    float profile_target_cont;    // Current profile endpoint, continuous cmd space
    float final_target_cont;      // Final endpoint after overshoot leg (if any)
    bool  overshoot_pending;      // True while executing the overshoot leg
    int   trim_iterations;
    unsigned long settle_start_ms;
    bool  at_target_printed;
    bool  trim_gave_up;

    // --- Motion profile ---
    float profile_vel_deg_s;      // Current signed profile velocity
    float cruise_vel_deg_s;       // Configurable cruise speed
    float accel_deg_s2;
    unsigned long last_profile_us;

    // --- Sweep ---
    bool  sweep_active;
    float sweep_speed_deg_s;

    // --- Scan orchestration (layered above the move/sweep machinery) ---
    enum ScanMode : uint8_t  { SCAN_NONE = 0, SCAN_STEPPED, SCAN_SMOOTH };
    enum ScanPhase : uint8_t { SP_IDLE = 0, SP_PREPOSITION, SP_MOVE, SP_DWELL, SP_RUN };
    ScanMode  scan_mode;
    ScanPhase scan_phase;
    float scan_start_deg, scan_end_deg;   // user inputs (0-360)
    float scan_inc_deg;                   // stepped: signed per-frame increment
    uint32_t scan_dwell_ms;               // stepped: hold time per frame
    int   scan_frame_idx, scan_frame_count;
    unsigned long scan_dwell_start_ms;
    float scan_speed_signed;              // smooth: signed sweep rate
    float scan_end_cont;                  // smooth: stop threshold, continuous cmd space
    int8_t scan_dir;                      // +1 / -1 (direction of travel)
    bool  scan_stream_was_enabled;        // restore streaming state after smooth scan

    // --- Encoder tracking ---
    float enc_deg;                // Last encoder reading 0-360
    float enc_continuous_deg;     // Unwrapped
    float enc_vel_deg_s;          // LPF'd encoder velocity
    float vel_prev_deg;
    unsigned long vel_prev_us;
    unsigned long last_command_time;

    // --- Backlash (measured by recalibrate(), stored in NVS) ---
    float backlash_deg;

    // --- Streaming ---
    bool stream_enabled;
    uint16_t stream_rate_hz;
    unsigned long stream_interval_us;
    unsigned long last_stream_time_us;

    // Runtime thresholds
    float pos_tolerance_deg;
    float vel_threshold_deg_s;

    // Approach direction for the current move's final leg and its trim
    // corrections: set by planMove from the override or the config default.
    int8_t approach_sign_current;

    // --- Helpers ---
    float commandedContinuousDeg();           // Step counter -> continuous output deg
    // approach_override: 0 = config default, +1/-1 = force final-leg direction
    void  planMove(float target_deg_0_360, int8_t approach_override = 0);
    void  beginProfileTo(float target_cont);
    void  runProfile();                       // Trapezoid update, feeds step ISR rate
    void  evaluateTrim();                     // Encoder check + correction planning
    void  runScan();                          // Scan-mode sequencer (no-op if inactive)
    void  setStepRate(float deg_s);           // Signed, forwards to ISR
    void  haltSteps();
    bool  tmcInit();                          // TMC2209 UART config (if enabled)
    void  loadBacklashFromNVS();
    void  saveBacklashToNVS();
};

#endif // MOTOR_CONTROL_H
