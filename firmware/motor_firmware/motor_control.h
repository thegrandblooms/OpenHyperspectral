#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"
#include "commands.h"

//=============================================================================
// SYSTEM STATE DEFINITIONS
//=============================================================================
enum SystemState {
    STATE_IDLE = 0,
    STATE_CALIBRATING = 1,
    STATE_MOVING = 2,
    STATE_ERROR = 3
};

enum ControlMode {
    MODE_TORQUE = 0,
    MODE_VELOCITY = 1,
    MODE_POSITION = 2
};

//=============================================================================
// ENCODER INTERRUPT SERVICE ROUTINES
//=============================================================================
// Forward declarations for encoder ISR callbacks
void doA();
void doB();

//=============================================================================
// MOTOR CONTROL MODULE (SimpleFOC Integration)
//=============================================================================

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

    // Update loop (call frequently)
    void update();

private:
    // SimpleFOC objects
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    Encoder encoder;

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
