#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "MT6701.h"  // MT6701 encoder driver (local file in sketch root)
#include "config.h"
#include "commands.h"

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
    static constexpr float FILTER_ALPHA = 0.275f; // Low-pass filter coefficient (0=no filtering, 1=no smoothing)

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
    bool calibrate();  // Runs motor.initFOC()
    bool runMultiPointCalibration(bool verbose = true);  // 24-point precision calibration

    // Control
    void enable();
    void disable();
    void stop();  // Emergency stop
    void moveToPosition(float absolute_deg);  // Move to absolute position (0-360°)
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

    // Direct motor access for tests
    BLDCMotor& getMotor() { return motor; }
    MT6701Sensor& getEncoder() { return encoder; }

private:
    // SimpleFOC objects
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    MT6701Sensor encoder;

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
    static constexpr unsigned long MOVE_TIMEOUT_MS = 3000;   // 3 second hard timeout
    static constexpr unsigned long SETTLING_TIME_MS = 200;   // 200ms settling window
    static constexpr float SETTLING_ERROR_DEG = 1.0f;        // Consider "close" if < 1°
    static constexpr float SETTLING_VEL_DEG_S = 5.0f;        // And velocity < 5°/s

    // Calibration helpers
    bool runCalibration();
    bool runManualCalibration();

    // Setpoint ramping (motion profiling)
    // Instead of jumping the PID target instantly, we ramp it at a limited rate.
    // This prevents overshoot/oscillation while preserving full PID gains (= torque).
    float ramp_position_deg;               // Current ramped setpoint (advances toward target)
    unsigned long last_ramp_time_us;       // Timestamp for dt computation
    static constexpr float SLEW_RATE_DEG_S = 120.0f;  // Max setpoint rate (°/s)
    static constexpr float DEADBAND_RAD = 0.00436f;   // ~0.25° position deadband

    // Auto-idle disable tracking
    unsigned long last_command_time;        // Last time a serial command was received (millis)
    unsigned long last_movement_time;       // Last time encoder detected movement (millis)
    float last_idle_check_deg;             // Position at last idle check
    bool auto_enabled;                      // True if motor was auto-enabled (vs manually enabled)
    static constexpr unsigned long IDLE_COMMAND_TIMEOUT_MS = 10000;  // 10s no commands
    static constexpr unsigned long IDLE_MOVEMENT_TIMEOUT_MS = 5000;  // 5s no encoder movement
    static constexpr float IDLE_MOVEMENT_THRESHOLD_DEG = 0.5f;      // Movement detection threshold

    // Encoder streaming state
    bool stream_enabled;
    uint16_t stream_rate_hz;
    unsigned long stream_interval_us;   // Microseconds between stream lines
    unsigned long last_stream_time_us;  // Last time a stream line was emitted
};

#endif // MOTOR_CONTROL_H
