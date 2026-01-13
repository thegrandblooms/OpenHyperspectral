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
// Raw Encoder (0-16383 counts, 14-bit)
//     ↓
// MT6701Sensor (exposes: raw counts, degrees, radians)
//     ↓
// MotorController (works in DEGREES internally)
//     ↓
// SimpleFOC Boundary (converts degrees → radians)
//     ↓
// SimpleFOC Library (expects radians)
//
// WHY DEGREES?
// - More intuitive for humans (0-360° vs 0-6.28 rad)
// - Easier to debug and tune
// - Clearer separation from SimpleFOC (which uses radians)
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
 * This class provides MINIMAL ABSTRACTION over the raw encoder:
 * - Exposes raw encoder counts (0-16383)
 * - Provides degrees (0-360) for internal use
 * - Provides radians (0-2π) for SimpleFOC
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
    float getSensorAngle() override;       // Returns RADIANS (SimpleFOC expects this)
    // NOTE: update() NOT overridden - base class handles it (standard SimpleFOC pattern)

    // Override getAngle() to provide continuous angle tracking
    // Returns: (float)full_rotations * 2π + angle_prev
    // This prevents velocity jumps at 0°/360° boundary (see motor_control.cpp for details)
    float getAngle() override;             // Returns continuous angle (can be negative or >2π)

    int needsSearch() override;            // Return 0 normally, 1 during calibration
    float getVelocity() override;          // Returns rad/s (with boundary crossing detection)

    // Calibration mode control
    void setCalibrationMode(bool enabled) { force_needs_search = enabled; }

    //=========================================================================
    // DIRECT ENCODER ACCESS (Our preferred interface - minimal abstraction)
    //=========================================================================
    uint16_t getRawCount();                // Raw encoder value (0-16383)
    float getDegrees();                    // Degrees (0-360)
    float getDegreesPerSecond();           // Angular velocity in deg/s

    //=========================================================================
    // HARDWARE STATUS
    //=========================================================================
    bool isFieldGood();
    uint8_t getFieldStatus();

private:
    MT6701 encoder;                        // Hardware driver

    // Cached values (updated by getSensorAngle())
    uint16_t cached_raw_count;             // Raw count (0-16383)
    float cached_degrees;                  // Degrees (0-360)
    float cached_radians;                  // Radians (0-2π) for SimpleFOC

    // Previous values for velocity calculation
    float previous_degrees;
    unsigned long last_update_time;

    // Calibration mode flag
    bool force_needs_search;               // Force needsSearch()=1 during calibration
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

    // Testing functions
    bool testDriverPhases();
    bool testMotorAlignment();

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

    // Calibration helpers
    bool runCalibration();
    bool runManualCalibration();
};

#endif // MOTOR_CONTROL_H
