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

/**
 * SimpleFOC-compatible wrapper for MT6701 encoder library
 * This class adapts the MT6701 library to work with SimpleFOC's Sensor interface
 */
class MT6701Sensor : public Sensor {
public:
    MT6701Sensor(uint8_t address = 0x06);

    // Sensor interface implementation (required by SimpleFOC)
    void init() override;
    float getSensorAngle() override;
    void update() override;           // Update sensor state (called by SimpleFOC before reading)
    int needsSearch() override;       // For absolute encoders, return 0 (no search needed)
    float getVelocity() override;     // Get angular velocity (SimpleFOC calculates this internally)
    int32_t getFullRotations() { return 0; }  // For absolute single-turn encoder

    // Additional MT6701 features
    bool isFieldGood();
    uint8_t getFieldStatus();

private:
    MT6701 encoder;
    float current_angle;      // Cached angle value
    float previous_angle;     // Previous angle for velocity calculation
    unsigned long last_update_time;  // Timestamp of last update
};

class MotorController {
public:
    MotorController();

    // Initialization
    void begin();
    bool calibrate();

    // Control commands
    void enable();
    void disable();
    void stop();
    void setHome();

    // Motion commands
    void moveToPosition(float position_rad);
    void setVelocity(float velocity_rad_s);
    void setAcceleration(float accel_rad_s2);
    void setCurrentLimit(float current_limit_a);

    // Control mode
    void setControlMode(uint8_t mode);
    uint8_t getControlMode();

    // PID tuning
    void setPositionPID(float p, float i, float d, float ramp);
    void setVelocityPID(float p, float i, float d, float ramp);
    void setCurrentPID(float p, float i, float d, float ramp);

    // State queries
    float getCurrentPosition();
    float getCurrentVelocity();
    float getTargetPosition();
    float getTargetVelocity();
    float getCurrent();
    float getVoltage();
    bool isEnabled();
    bool isCalibrated();
    bool isAtTarget();
    uint8_t getState();

    // Debug helpers
    float getDirectEncoderAngle();  // Read encoder directly (bypass SimpleFOC)

    // Update loop (call frequently)
    void update();

private:
    // SimpleFOC objects
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    MT6701Sensor encoder;

    // State variables
    uint8_t system_state;
    uint8_t control_mode;
    bool motor_enabled;
    bool motor_calibrated;
    float target_position;
    float target_velocity;
    float max_velocity;
    float max_acceleration;
    float current_limit;

    // Position tracking for notifications
    bool target_reached;
    float position_tolerance;

    // Calibration
    bool runCalibration();
};

#endif // MOTOR_CONTROL_H
