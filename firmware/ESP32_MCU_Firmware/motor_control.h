#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "MT6701.h"  // MT6701 encoder driver (local file in sketch root)
#include "config.h"
#include "commands.h"

#if USE_CALIBRATED_SENSOR
#include "CalibratedSensor.h"  // local copy from Arduino-FOC-drivers v1.0.9
#endif

//=============================================================================
// MOTOR CONTROL MODULE (SimpleFOC Integration with MT6701 Encoder)
//=============================================================================
//
// ARCHITECTURE OVERVIEW:
// ----------------------
// Raw Encoder (0-16383 counts, 14-bit I2C)
//     ↓
// Cartesian Filtering (SmartKnob pattern - eliminates wraparound discontinuities)
//     ↓
// MT6701Sensor (exposes: raw counts, degrees, filtered radians)
//     ↓
// MotorController (works in DEGREES internally)
//     ↓
// SimpleFOC Boundary (converts degrees → radians)
//     ↓
// SimpleFOC Library (expects radians, handles position control)
//
// WHY DEGREES?
// - More intuitive for humans (0-360° vs 0-6.28 rad)
// - Easier to debug and tune
// - Clearer separation from SimpleFOC (which uses radians)
//
// CARTESIAN FILTERING (SmartKnob Innovation):
// - Converts angle to (x,y) coordinates before filtering
// - Filters x and y separately (eliminates 0°/360° discontinuity)
// - Converts back to angle via atan2
// - Prevents hunting/vibration at angle boundaries
//
// SIMPLEFOC INTEGRATION:
// - motor.loopFOC() - Runs current control (FOC algorithm)
// - motor.move(target_rad) - Runs position/velocity control
// - We set targets, SimpleFOC handles getting there (no "fighting")
// - SimpleFOC manages cascaded PIDs: Position → Velocity → Current
//=============================================================================

//=============================================================================
// UNIT CONVERSION HELPERS
//=============================================================================

// Convert raw encoder counts (0-16383) to degrees (0-360)
inline float rawToDegrees(uint16_t raw_count) {
    return (raw_count / (float)ENCODER_PPR) * 360.0f;
}

// Convert degrees to radians (for SimpleFOC)
inline float degreesToRadians(float degrees) {
    return degrees * (PI / 180.0f);
}

// Convert radians to degrees
inline float radiansToDegrees(float radians) {
    return radians * (180.0f / PI);
}

// Normalize angle to 0-360 degrees
inline float normalizeDegrees(float degrees) {
    degrees = fmod(degrees, 360.0f);
    if (degrees < 0) degrees += 360.0f;
    return degrees;
}

// Normalize angle to 0-2π radians
inline float normalizeRadians(float radians) {
    radians = fmod(radians, 2.0f * PI);
    if (radians < 0) radians += 2.0f * PI;
    return radians;
}

//=============================================================================
// ENCODER SENSOR CLASS
//=============================================================================

/**
 * SimpleFOC-compatible wrapper for MT6701 encoder library
 *
 * This class implements the SmartKnob-proven sensor integration pattern:
 * - Reads raw encoder counts (0-16383) via I2C
 * - Applies Cartesian filtering to eliminate wraparound discontinuities
 * - Returns filtered angle (0-2π radians) to SimpleFOC
 * - Provides raw counts and degrees for diagnostics
 *
 * CARTESIAN FILTERING:
 * Instead of filtering angles directly (which fails at 0°/360° boundary),
 * we convert to (x,y) coordinates, filter those, then convert back.
 * This prevents hunting/vibration when crossing angle boundaries.
 *
 * CLEAR SEPARATION: This is the encoder, SimpleFOC is the motor controller.
 * We read position here, SimpleFOC uses it for control.
 */
class MT6701Sensor : public Sensor {
public:
    MT6701Sensor(uint8_t address = 0x06);

    //=========================================================================
    // SENSOR INTERFACE (Required by SimpleFOC - works in RADIANS)
    //=========================================================================
    void init() override;
    float getSensorAngle() override;       // Returns RADIANS with Cartesian filtering (SimpleFOC expects this)
    // NOTE: update() and getAngle() NOT overridden - base class handles rotation tracking
    // This follows the standard SimpleFOC pattern used by SmartKnob

    int needsSearch() override;            // Return 0 normally, 1 during calibration
    // NOTE: getVelocity() NOT overridden - base class calculates velocity correctly
    // Our previous override was broken (previous_degrees == cached_degrees always)

    // Calibration mode control
    void setCalibrationMode(bool enabled) { force_needs_search = enabled; }

    // Reset rotation tracking to absolute encoder mode
    // Call this after calibration movements to clear corrupted full_rotations counter
    void resetRotationTracking();

    //=========================================================================
    // DIRECT ENCODER ACCESS (Our preferred interface - minimal abstraction)
    //=========================================================================
    uint16_t getRawCount();                // Raw encoder value (0-16383, cached from last update)
    uint16_t readRawAngleDirect();         // Fresh I2C read bypassing filter (for diagnostics)
    float getDegrees();                    // Degrees (0-360)
    float getDegreesPerSecond();           // Angular velocity in deg/s

    //=========================================================================
    // HARDWARE STATUS
    //=========================================================================
    bool isFieldGood();
    uint8_t getFieldStatus();

    //=========================================================================
    // DIAGNOSTICS
    //=========================================================================
    unsigned long getCallCount() { return call_count; }
    void resetCallCount() { call_count = 0; }

private:
    MT6701 encoder;                        // Hardware driver

    // Cached values (updated by getSensorAngle())
    uint16_t cached_raw_count;             // Raw count (0-16383)
    float cached_degrees;                  // Degrees (0-360)
    float cached_radians;                  // Radians (0-2π) for SimpleFOC

    // Cartesian filtering (SmartKnob pattern - eliminates wraparound discontinuities)
    float filtered_x;                      // Filtered X coordinate (cos component)
    float filtered_y;                      // Filtered Y coordinate (sin component)
    static constexpr float FILTER_ALPHA = 0.4f;  // Low-pass filter coefficient (0=no filtering, 1=no smoothing)

    // Previous values for velocity calculation
    float previous_degrees;
    unsigned long last_update_time;

    // Calibration mode flag
    bool force_needs_search;               // Force needsSearch()=1 during calibration

    // Diagnostic call counter
    unsigned long call_count;              // Number of times getSensorAngle() has been called
};

//=============================================================================
// MOTOR CONTROLLER CLASS
//=============================================================================

/**
 * Motor Controller - High-level motor control using SimpleFOC
 *
 * INTERNAL UNITS: DEGREES (easier to understand and debug)
 * SIMPLEFOC BOUNDARY: Converts to/from radians when interfacing with SimpleFOC
 *
 * CONTROL FLOW:
 * 1. User sets target in degrees (moveToPosition, setVelocity, etc.)
 * 2. We convert to radians and pass to SimpleFOC via motor.move()
 * 3. SimpleFOC manages cascaded PIDs (Position → Velocity → Current)
 * 4. We read back motor.shaft_angle (radians) and convert to degrees
 *
 * NO "FIGHTING": SimpleFOC doesn't fight us. We set targets, it follows.
 */
class MotorController {
public:
    MotorController();

    // Setup
    void begin();
    bool calibrate();          // Load saved calibration from NVS, or run thorough cal + save
    bool recalibrate();        // Clear NVS and run thorough calibration from scratch
    void clearCalibrationNVS(); // Erase stored calibration (forces fresh cal next boot)

    // Control
    void enable();
    void disable();
    void stop();  // Emergency stop
    void moveToPosition(float absolute_deg);  // Move to absolute position (0-360°)
    void startSweep(float speed_deg_s);  // Moving-target position sweep at given deg/s
    void stopSweep();                    // Stop sweep, hold current position
    bool isSweepActive() const { return sweep_active; }
    void update();  // Call in loop - runs motor.loopFOC() + motor.move()

    // Home positioning
    void setHome();  // Log current position (absolute encoders don't need homing)

    // Configuration (legacy compatibility)
    void setVelocity(float velocity_deg_s);
    void setAcceleration(float accel_deg_s2);
    void setCurrentLimit(float new_current_limit_a);
    void setControlMode(uint8_t mode);

    // PID tuning
    void setPositionPID(float p, float i, float d, float ramp_deg_s);
    void setVelocityPID(float p, float i, float d, float ramp_deg_s);
    void setCurrentPID(float p, float i, float d, float ramp);
    bool autoTunePID(bool verbose = false);

    // Runtime-tunable position tracking thresholds (defaults from config.h)
    void setPositionTolerance(float deg);
    void setVelocityThreshold(float deg_s);
    float getPositionTolerance() const { return pos_tolerance_deg; }
    float getVelocityThreshold() const { return vel_threshold_deg_s; }

    // Status getters
    float getPosition();  // Current position (absolute 0-360°)
    float getAbsolutePositionDeg();  // Alias for getPosition()
    float getCurrentPositionDeg();  // Alias for getPosition()
    float getCurrentVelocityDegPerSec();
    float getTargetPositionDeg();
    float getTargetVelocityDegPerSec();
    float getCurrent();
    float getVoltage();
    bool isEnabled();
    bool isCalibrated();
    bool isAtTarget();
    uint8_t getState();
    uint8_t getControlMode();

    // Direct encoder access (for diagnostics)
    void updateEncoder();  // Force encoder read
    uint16_t getRawEncoderCount();
    float getEncoderDegrees();  // Direct encoder read

    // Diagnostic: Get sensor call count (for debugging SimpleFOC integration)
    unsigned long getSensorCallCount() { return encoder.getCallCount(); }

    // Encoder streaming (tagged CSV over serial)
    void setStreamEnabled(bool enabled);
    bool isStreamEnabled() { return stream_enabled; }
    void setStreamRate(uint16_t hz);
    uint16_t getStreamRate() { return stream_rate_hz; }
    void emitStreamLine();       // Call from loop - emits $ENC, line at configured rate
    void emitScanStart();        // Emit $SCAN_START marker
    void emitScanEnd();          // Emit $SCAN_END marker

    // Auto-enable/disable
    void notifyCommandActivity();    // Call on any serial command to reset idle timer
    void checkIdleDisable();         // Call in loop - disables motor after idle timeout
    void setAutoEnabled(bool enabled) { auto_enabled = enabled; }
    bool isAutoEnabled() { return auto_enabled; }

    // Anti-cogging feedforward (runtime-tunable)
    void  setCogFFAmp(float amp)     { cog_ff_amp   = amp;   }
    void  setCogFFPhase(float phase) { cog_ff_phase = phase; }
    float getCogFFAmp()   const { return cog_ff_amp;   }
    float getCogFFPhase() const { return cog_ff_phase; }

    // Dense cogging LUT (populated by 'cogmap_dense' firmware command)
    void setCogLutEntry(int idx, float vq);   // Store one 1°-step measurement
    void finalizeCogLut();                    // Mark LUT ready, save to NVS, print summary
    void clearCogLut();                       // Disable, zero, and erase LUT from NVS
    bool isCogLutLoaded() const { return cog_lut_loaded; }
    void loadCogLutFromNVS();                 // Try to restore LUT from NVS on boot

    // Direct motor access for tests
    BLDCMotor& getMotor() { return motor; }
    MT6701Sensor& getEncoder() { return encoder; }

private:
    // SimpleFOC objects
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    MT6701Sensor encoder;
#if USE_CALIBRATED_SENSOR
    CalibratedSensor calibratedSensor;  // wraps encoder; must be declared after encoder
#endif

    // State
    bool motor_enabled;
    bool motor_calibrated;
    float target_position_deg;  // Absolute position (0-360°)

    // Move timeout and settling tracking
    unsigned long move_start_time;      // When current move started (millis)
    unsigned long settling_start_time;  // When we entered "close enough" state
    bool move_timeout_printed;          // Flag to only print timeout once per move
    bool at_target_printed;             // Flag to only print AT_TARGET once per move
    float last_target_for_timeout;      // Track target to detect new moves
    static constexpr unsigned long MOVE_TIMEOUT_MS = 15000;  // 15 second hard timeout — position 2 cogging-snap + return takes ~10-11s total
    static constexpr unsigned long SETTLING_TIME_MS = 200;   // 200ms settling window
    static constexpr float SETTLING_ERROR_DEG = 1.0f;        // Consider "close" if < 1°
    static constexpr float SETTLING_VEL_DEG_S = 5.0f;        // And velocity < 5°/s

    // Calibration helpers
    bool runCalibration();
    bool runManualCalibration();
    bool runThoroughCalibration(int n_runs = 5);  // Run N alignments, average zero_electric_angle
    bool loadCalibrationFromNVS();   // true if valid saved cal loaded
    void saveCalibrationToNVS();
#if USE_CALIBRATED_SENSOR
    bool runCalibratedSensorCalibration();
#endif

    // Setpoint ramping (motion profiling)
    // Instead of jumping the PID target instantly, we ramp it at a limited rate.
    // This prevents overshoot/oscillation while preserving full PID gains (= torque).
    float ramp_position_deg;               // Current ramped setpoint (advances toward target)
    unsigned long last_ramp_time_us;       // Timestamp for dt computation
    static constexpr float SLEW_RATE_DEG_S = 60.0f;   // Max setpoint rate (°/s) — halved from 120: lower approach velocity reduces overshoot when Kd=0

    // Torque mode integral accumulator (reset on new target to avoid windup after large moves)
    float torque_integral;                 // Accumulated integral voltage (V) — capped at ±TORQUE_I_CAP

    // Torque mode velocity estimation: position-difference over fixed 10ms window.
    // motor.shaft_velocity is instantaneous (noisy at 1kHz); this is smoother.
    float vel_pos_prev_deg;                // Position sample taken vel_window_us ago
    unsigned long vel_window_start_us;     // Timestamp when vel_pos_prev_deg was captured
    float torque_vel_deg_s;               // Current velocity estimate (deg/s), updated every 10ms

    // Actual velocity (deg/s) computed from position deltas between update() calls.
    // Replaces motor.shaft_velocity (which is near-zero at 100Hz due to LPF double-application).
    float lpf_vel_deg_s;  // set to actual_vel_deg_s each update() — used for logging/status

    // Settling timer for isAtTarget(): motor must stay within pos_tolerance_deg for
    // SETTLING_TIME_MS before AT_TARGET fires. Prevents firing while passing through zone.
    unsigned long at_target_settle_ms;  // 0 = not in zone; nonzero = time when zone was entered

    // Auto-idle disable tracking
    unsigned long last_command_time;        // Last time a serial command was received (millis)
    unsigned long last_movement_time;       // Last time encoder detected movement (millis)
    float last_idle_check_deg;             // Position at last idle check
    bool auto_enabled;                      // True if motor was auto-enabled (vs manually enabled)
    static constexpr unsigned long IDLE_COMMAND_TIMEOUT_MS = 10000;  // 10s no commands
    static constexpr unsigned long IDLE_MOVEMENT_TIMEOUT_MS = 5000;  // 5s no encoder movement
    static constexpr float IDLE_MOVEMENT_THRESHOLD_DEG = 0.5f;      // Movement detection threshold

    // Runtime position tracking thresholds (overrideable via 'set' command)
    float pos_tolerance_deg;
    float vel_threshold_deg_s;

    // Anti-cogging feedforward parameters (runtime-tunable via 'set cog_amp/cog_phase')
    float cog_ff_amp;    // Sinusoidal feedforward amplitude (V); 0 = disabled
    float cog_ff_phase;  // Phase offset (rad) — tune to align feedforward with cogging detents

    // Dense cogging LUT feedforward (360 entries at 1° steps = full revolution).
    // Populated by 'cogmap_dense' command. When loaded, replaces sinusoidal feedforward.
    // Each entry: steady-state Vq measured at that angle = cogging force the PID must fight.
    // Injecting negates the cogging force, letting the PID track with small errors only.
    // 1° resolution: 4.3 samples per 4.3° cogging cycle — adequate to reconstruct waveform.
    // RAM cost: 360 × 4 bytes = 1440 bytes (negligible on ESP32-S3).
    static const int COG_LUT_SIZE = 360;   // 360° / 1° per step
    float cog_lut[COG_LUT_SIZE];
    bool cog_lut_loaded;

    // Saved velocity PID integral gain — set to 0 during sweep to prevent limit cycle,
    // restored when sweep stops so position holding still has integral action.
    float saved_i_vel;

    // Scan sweep state: moving-target position control for continuous hyperspectral scan.
    // Instead of fighting cogging with fixed voltage, we advance the position setpoint at
    // the desired speed and let the existing position PID track it. The PID generates
    // up to 6V transiently to push through cogging detents — no stalling, no special tuning.
    bool sweep_active;           // True when sweep is running
    float sweep_speed_deg_s;     // Setpoint advance rate (deg/s, signed)
    float sweep_prev_enc_deg;    // Encoder position at last 50ms velocity measurement
    unsigned long sweep_vel_t_ms;// Timestamp of last velocity measurement (ms)
    float sweep_vel_lpf;         // Low-pass filtered 50ms-sampled velocity (deg/s) for PI controller
    float sweep_vel_integral;    // PI velocity controller integral accumulator (V)

    // Encoder streaming state
    bool stream_enabled;
    uint16_t stream_rate_hz;
    unsigned long stream_interval_us;   // Microseconds between stream lines
    unsigned long last_stream_time_us;  // Last time a stream line was emitted
};

#endif // MOTOR_CONTROL_H
