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
    void update() override;                // Update cached values
    int needsSearch() override;            // Return 0 normally, 1 during calibration
    float getVelocity() override;          // Returns rad/s
    int32_t getFullRotations() { return 0; }  // Single-turn encoder

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

    // Cached values (updated by update())
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

    //=========================================================================
    // INITIALIZATION
    //=========================================================================
    void begin();
    bool calibrate();

    //=========================================================================
    // MANUAL CALIBRATION (For MT6701 I2C sensors where auto-calibration fails)
    //=========================================================================
    bool testDriverPhases();               // Test each driver phase individually
    bool testMotorAlignment();             // Diagnostic test - verify motor holds positions
    bool runManualCalibration();           // Manual calibration to find zero_electric_angle

    //=========================================================================
    // CONTROL COMMANDS
    //=========================================================================
    void enable();
    void disable();
    void stop();
    void setHome();

    //=========================================================================
    // MOTION COMMANDS (All in DEGREES)
    //=========================================================================
    void moveToPosition(float position_deg);        // Move to absolute position (degrees)
    void setVelocity(float velocity_deg_s);         // Set velocity (degrees/second)
    void setAcceleration(float accel_deg_s2);       // Set acceleration (degrees/second²)
    void setCurrentLimit(float current_limit_a);    // Set current limit (amps)

    //=========================================================================
    // CONTROL MODE
    //=========================================================================
    void setControlMode(uint8_t mode);
    uint8_t getControlMode();

    //=========================================================================
    // PID TUNING
    // NOTE: PID gains are unitless, ramp is in units/second
    //=========================================================================
    void setPositionPID(float p, float i, float d, float ramp_deg_s);
    void setVelocityPID(float p, float i, float d, float ramp_deg_s);
    void setCurrentPID(float p, float i, float d, float ramp);

    //=========================================================================
    // PID AUTO-TUNING
    //=========================================================================
    bool autoTunePID(bool verbose = true);

    //=========================================================================
    // STATE QUERIES (All in DEGREES unless otherwise noted)
    //=========================================================================
    // ABSOLUTE ENCODER POSITION (Direct hardware read - TRUTH SOURCE)
    float getAbsolutePositionDeg();             // Absolute encoder position (degrees) - REAL position from MT6701

    // SIMPLEFOC CONTROLLER STATE (Internal motor controller state)
    float getCurrentPositionDeg();              // SimpleFOC shaft_angle (degrees) - may lag or drift
    float getCurrentVelocityDegPerSec();        // SimpleFOC velocity (deg/s)

    // OTHER STATE
    float getTargetPositionDeg();               // Target position (degrees)
    float getTargetVelocityDegPerSec();         // Target velocity (deg/s)
    float getCurrent();                         // Q-axis current (amps)
    float getVoltage();                         // Q-axis voltage (volts)
    bool isEnabled();
    bool isCalibrated();
    bool isAtTarget();                          // Uses ABSOLUTE ENCODER position
    uint8_t getState();

    //=========================================================================
    // DIRECT ENCODER ACCESS (Bypass SimpleFOC - for debugging)
    //=========================================================================
    void updateEncoder();                       // Force fresh encoder read from I2C
    uint16_t getRawEncoderCount();              // Raw encoder (0-16383)
    float getEncoderDegrees();                  // Direct encoder read (degrees)

    //=========================================================================
    // SIMPLEFOC ACCESS (For PID tuner compatibility)
    //=========================================================================
    BLDCMotor& getMotor() { return motor; }
    MT6701Sensor& getEncoder() { return encoder; }

    //=========================================================================
    // UPDATE LOOP (Call frequently - runs FOC and motion control)
    //=========================================================================
    void update();

private:
    //=========================================================================
    // SIMPLEFOC OBJECTS (Work in RADIANS)
    //=========================================================================
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    MT6701Sensor encoder;

    //=========================================================================
    // STATE VARIABLES (Our internal state - in DEGREES)
    //=========================================================================
    uint8_t system_state;
    uint8_t control_mode;
    bool motor_enabled;
    bool motor_calibrated;

    // Targets (in DEGREES)
    float target_position_deg;
    float target_velocity_deg_s;

    // Limits (in DEGREES where applicable)
    float max_velocity_deg_s;
    float max_acceleration_deg_s2;
    float current_limit_a;

    // Position tracking
    bool target_reached;
    float position_tolerance_deg;

    //=========================================================================
    // PRIVATE METHODS
    //=========================================================================
    bool runCalibration();
};

#endif // MOTOR_CONTROL_H
