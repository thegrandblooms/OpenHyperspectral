#include "motor_control.h"
#include "commands.h"  // Explicit include for state/mode definitions
#include <Wire.h>      // I2C library for MT6701 encoder

MotorController::MotorController()
    : motor(POLE_PAIRS),
      driver(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE),
      encoder(ENCODER_I2C_ADDR, 14, 0x0E),  // MT6701: I2C address, 14-bit, angle register 0x0E
      system_state(STATE_IDLE),
      control_mode(DEFAULT_CONTROL_MODE),
      motor_enabled(false),
      motor_calibrated(false),
      target_position(0.0),
      target_velocity(0.0),
      max_velocity(MAX_VELOCITY),
      max_acceleration(DEFAULT_ACCELERATION),
      current_limit(CURRENT_LIMIT),
      target_reached(false),
      position_tolerance(POSITION_TOLERANCE) {
}

void MotorController::begin() {
    if (DEBUG_MOTOR) {
        Serial.println("Initializing motor controller...");
    }

    // Initialize I2C for MT6701 encoder
    Wire.setPins(ENCODER_SDA, ENCODER_SCL);
    Wire.begin();

    if (DEBUG_MOTOR) {
        Serial.print("I2C initialized on SDA=");
        Serial.print(ENCODER_SDA);
        Serial.print(", SCL=");
        Serial.println(ENCODER_SCL);
    }

    // Initialize I2C encoder
    encoder.init(&Wire);

    // Link encoder to motor
    motor.linkSensor(&encoder);

    // Driver configuration
    driver.voltage_power_supply = VOLTAGE_PSU;
    driver.init();

    // Link driver to motor
    motor.linkDriver(&driver);

    // Set motor limits
    motor.voltage_limit = VOLTAGE_PSU * 0.8;  // 80% of supply voltage
    motor.current_limit = current_limit;
    motor.velocity_limit = max_velocity;

    // Set FOC modulation (space vector PWM is more efficient)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // Set motion control type based on default mode
    switch (control_mode) {
        case MODE_POSITION:
            motor.controller = MotionControlType::angle;
            break;
        case MODE_VELOCITY:
            motor.controller = MotionControlType::velocity;
            break;
        case MODE_TORQUE:
            motor.controller = MotionControlType::torque;
            break;
    }

    // Configure PID controllers
    motor.PID_velocity.P = PID_P_VELOCITY;
    motor.PID_velocity.I = PID_I_VELOCITY;
    motor.PID_velocity.D = PID_D_VELOCITY;
    motor.PID_velocity.output_ramp = PID_RAMP_VELOCITY;
    motor.PID_velocity.limit = max_velocity;

    motor.P_angle.P = PID_P_POSITION;
    motor.P_angle.I = PID_I_POSITION;
    motor.P_angle.D = PID_D_POSITION;
    motor.P_angle.output_ramp = PID_RAMP_POSITION;
    motor.P_angle.limit = max_velocity;

    // Current control PID (for FOC)
    motor.PID_current_q.P = PID_P_CURRENT;
    motor.PID_current_q.I = PID_I_CURRENT;
    motor.PID_current_q.D = PID_D_CURRENT;
    motor.PID_current_q.output_ramp = PID_RAMP_CURRENT;
    motor.PID_current_q.limit = current_limit;

    motor.PID_current_d.P = PID_P_CURRENT;
    motor.PID_current_d.I = PID_I_CURRENT;
    motor.PID_current_d.D = PID_D_CURRENT;
    motor.PID_current_d.output_ramp = PID_RAMP_CURRENT;
    motor.PID_current_d.limit = current_limit;

    // Initialize motor
    motor.init();

    if (DEBUG_MOTOR) {
        Serial.println("Motor controller initialized");
    }

    // Motor starts disabled for safety
    motor.disable();
}

bool MotorController::calibrate() {
    if (DEBUG_MOTOR) {
        Serial.println("Starting motor calibration...");
    }

    system_state = STATE_CALIBRATING;

    // Run SimpleFOC calibration
    bool success = runCalibration();

    if (success) {
        motor_calibrated = true;
        system_state = STATE_IDLE;

        if (DEBUG_MOTOR) {
            Serial.println("Calibration successful");
        }
    } else {
        motor_calibrated = false;
        system_state = STATE_ERROR;

        if (DEBUG_MOTOR) {
            Serial.println("Calibration failed");
        }
    }

    return success;
}

bool MotorController::runCalibration() {
    // Align motor and encoder
    motor.initFOC();

    // Verify encoder is working
    float test_angle = encoder.getAngle();
    delay(100);
    float test_angle2 = encoder.getAngle();

    // Check if encoder is providing different readings (indicating it's working)
    if (abs(test_angle - test_angle2) < 0.001) {
        // Try moving motor to check encoder response
        motor.enable();
        motor.move(1.0);  // Small test movement
        delay(500);
        float test_angle3 = encoder.getAngle();
        motor.disable();

        if (abs(test_angle - test_angle3) < 0.001) {
            if (DEBUG_MOTOR) {
                Serial.println("Encoder not responding");
            }
            return false;
        }
    }

    return true;
}

void MotorController::enable() {
    if (!motor_calibrated) {
        if (DEBUG_MOTOR) {
            Serial.println("Cannot enable - motor not calibrated");
        }
        return;
    }

    motor.enable();
    motor_enabled = true;

    if (DEBUG_MOTOR) {
        Serial.println("Motor enabled");
    }
}

void MotorController::disable() {
    motor.disable();
    motor_enabled = false;

    if (DEBUG_MOTOR) {
        Serial.println("Motor disabled");
    }
}

void MotorController::stop() {
    // Emergency stop - disable motor immediately
    motor.disable();
    motor_enabled = false;
    system_state = STATE_IDLE;

    if (DEBUG_MOTOR) {
        Serial.println("Motor stopped (emergency)");
    }
}

void MotorController::setHome() {
    // Set current position as home (zero)
    float current_angle = motor.shaft_angle;
    motor.sensor_offset = current_angle;

    if (DEBUG_MOTOR) {
        Serial.print("Home set at angle: ");
        Serial.println(current_angle);
    }
}

void MotorController::moveToPosition(float position_rad) {
    if (!motor_enabled) {
        if (DEBUG_MOTOR) {
            Serial.println("Cannot move - motor not enabled");
        }
        return;
    }

    if (control_mode != MODE_POSITION) {
        if (DEBUG_MOTOR) {
            Serial.println("Cannot move - not in position control mode");
        }
        return;
    }

    target_position = position_rad;
    target_reached = false;
    system_state = STATE_MOVING;

    if (DEBUG_MOTOR) {
        Serial.print("Moving to position: ");
        Serial.println(position_rad);
    }
}

void MotorController::setVelocity(float velocity_rad_s) {
    target_velocity = constrain(velocity_rad_s, -max_velocity, max_velocity);

    if (DEBUG_MOTOR) {
        Serial.print("Velocity set to: ");
        Serial.println(target_velocity);
    }
}

void MotorController::setAcceleration(float accel_rad_s2) {
    max_acceleration = constrain(accel_rad_s2, 0, MAX_ACCELERATION);

    // Update PID ramp limits
    motor.PID_velocity.output_ramp = max_acceleration;
    motor.P_angle.output_ramp = max_acceleration;

    if (DEBUG_MOTOR) {
        Serial.print("Acceleration set to: ");
        Serial.println(max_acceleration);
    }
}

void MotorController::setCurrentLimit(float current_limit_a) {
    current_limit = constrain(current_limit_a, 0, CURRENT_LIMIT);
    motor.current_limit = current_limit;

    if (DEBUG_MOTOR) {
        Serial.print("Current limit set to: ");
        Serial.println(current_limit);
    }
}

void MotorController::setControlMode(uint8_t mode) {
    // Disable motor before changing mode
    bool was_enabled = motor_enabled;
    if (motor_enabled) {
        disable();
    }

    control_mode = mode;

    // Update motor controller type
    switch (mode) {
        case MODE_POSITION:
            motor.controller = MotionControlType::angle;
            break;
        case MODE_VELOCITY:
            motor.controller = MotionControlType::velocity;
            break;
        case MODE_TORQUE:
            motor.controller = MotionControlType::torque;
            break;
    }

    if (DEBUG_MOTOR) {
        Serial.print("Control mode changed to: ");
        Serial.println(mode);
    }

    // Re-enable motor if it was enabled before
    if (was_enabled) {
        enable();
    }
}

uint8_t MotorController::getControlMode() {
    return control_mode;
}

void MotorController::setPositionPID(float p, float i, float d, float ramp) {
    motor.P_angle.P = p;
    motor.P_angle.I = i;
    motor.P_angle.D = d;
    motor.P_angle.output_ramp = ramp;

    if (DEBUG_MOTOR) {
        Serial.println("Position PID updated");
    }
}

void MotorController::setVelocityPID(float p, float i, float d, float ramp) {
    motor.PID_velocity.P = p;
    motor.PID_velocity.I = i;
    motor.PID_velocity.D = d;
    motor.PID_velocity.output_ramp = ramp;

    if (DEBUG_MOTOR) {
        Serial.println("Velocity PID updated");
    }
}

void MotorController::setCurrentPID(float p, float i, float d, float ramp) {
    motor.PID_current_q.P = p;
    motor.PID_current_q.I = i;
    motor.PID_current_q.D = d;
    motor.PID_current_q.output_ramp = ramp;

    motor.PID_current_d.P = p;
    motor.PID_current_d.I = i;
    motor.PID_current_d.D = d;
    motor.PID_current_d.output_ramp = ramp;

    if (DEBUG_MOTOR) {
        Serial.println("Current PID updated");
    }
}

float MotorController::getCurrentPosition() {
    return motor.shaft_angle;
}

float MotorController::getCurrentVelocity() {
    return motor.shaft_velocity;
}

float MotorController::getTargetPosition() {
    return target_position;
}

float MotorController::getTargetVelocity() {
    return target_velocity;
}

float MotorController::getCurrent() {
    return motor.current.q;  // Q-axis current (proportional to torque)
}

float MotorController::getVoltage() {
    return motor.voltage.q;  // Q-axis voltage
}

bool MotorController::isEnabled() {
    return motor_enabled;
}

bool MotorController::isCalibrated() {
    return motor_calibrated;
}

bool MotorController::isAtTarget() {
    if (control_mode != MODE_POSITION) {
        return false;
    }

    float position_error = abs(getCurrentPosition() - target_position);
    float velocity = abs(getCurrentVelocity());

    // Consider target reached if position error is small and velocity is near zero
    return (position_error < position_tolerance) && (velocity < VELOCITY_THRESHOLD);
}

uint8_t MotorController::getState() {
    return system_state;
}

void MotorController::update() {
    if (!motor_calibrated) {
        return;
    }

    // Update FOC algorithm
    motor.loopFOC();

    // Update motion control
    switch (control_mode) {
        case MODE_POSITION:
            motor.move(target_position);

            // Check if target reached
            if (system_state == STATE_MOVING && isAtTarget()) {
                if (!target_reached) {
                    target_reached = true;
                    system_state = STATE_IDLE;

                    if (DEBUG_MOTOR) {
                        Serial.println("Target position reached");
                    }
                }
            }
            break;

        case MODE_VELOCITY:
            motor.move(target_velocity);
            break;

        case MODE_TORQUE:
            // For torque mode, target is in current/torque units
            motor.move(target_velocity);  // Reuse target_velocity for torque
            break;
    }
}
