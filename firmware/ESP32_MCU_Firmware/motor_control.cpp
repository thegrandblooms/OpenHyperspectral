#include "motor_control.h"
#include "commands.h"  // Explicit include for state/mode definitions
#include <Wire.h>      // I2C library for MT6701 encoder

//=============================================================================
// MT6701 Sensor Wrapper Implementation
//=============================================================================

MT6701Sensor::MT6701Sensor(uint8_t address)
    : encoder(address) {
}

void MT6701Sensor::init() {
    // Initialize I2C for MT6701 encoder (using working approach from encoder_test)
    Wire.begin(ENCODER_SDA, ENCODER_SCL);
    Wire.setClock(400000);  // 400kHz fast mode

    if (DEBUG_MOTOR) {
        Serial.print("[MT6701] I2C initialized on SDA=GPIO");
        Serial.print(ENCODER_SDA);
        Serial.print(", SCL=GPIO");
        Serial.println(ENCODER_SCL);
    }

    // Initialize MT6701 encoder library from firmware/libraries/MT6701
    if (!encoder.begin(&Wire)) {
        if (DEBUG_MOTOR) {
            Serial.println("[MT6701] ERROR: Encoder initialization failed!");
        }
        return;
    }

    if (DEBUG_MOTOR) {
        Serial.println("[MT6701] Encoder initialized successfully");

        // Check field strength
        uint8_t field_status = encoder.readFieldStatus();
        if (field_status == 0x00) {
            Serial.println("[MT6701] Magnetic field: GOOD");
        } else if (field_status == 0x01) {
            Serial.println("[MT6701] WARNING: Magnetic field TOO STRONG");
        } else if (field_status == 0x02) {
            Serial.println("[MT6701] WARNING: Magnetic field TOO WEAK");
        }
    }
}

float MT6701Sensor::getSensorAngle() {
    // Read angle in radians from MT6701 library
    return encoder.readAngleRadians();
}

bool MT6701Sensor::isFieldGood() {
    return encoder.isFieldGood();
}

uint8_t MT6701Sensor::getFieldStatus() {
    return encoder.readFieldStatus();
}

//=============================================================================
// Motor Controller Implementation
//=============================================================================

MotorController::MotorController()
    : motor(POLE_PAIRS),
      driver(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE),
      encoder(ENCODER_I2C_ADDR),
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

    // Initialize MT6701 encoder using library from firmware/libraries/MT6701
    encoder.init();

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
    if (DEBUG_MOTOR) {
        Serial.print("[FOC] Calling motor.init()... ");
    }
    motor.init();
    if (DEBUG_MOTOR) {
        Serial.println("Done");
    }

    if (DEBUG_MOTOR) {
        Serial.println("[MOTOR] Motor controller initialized");

        // Test direct encoder read vs SimpleFOC shaft_angle
        float direct_encoder_read = encoder.getSensorAngle();
        float simplefoc_angle = motor.shaft_angle;
        Serial.print("[DEBUG] Direct encoder read: ");
        Serial.print(direct_encoder_read, 4);
        Serial.println(" rad");
        Serial.print("[DEBUG] SimpleFOC shaft_angle: ");
        Serial.print(simplefoc_angle, 4);
        Serial.println(" rad");
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
    // Check magnetic field strength first (warning only, not fatal)
    if (!encoder.isFieldGood()) {
        if (DEBUG_MOTOR) {
            Serial.print("[MT6701] Field status: 0x");
            Serial.println(encoder.getFieldStatus(), HEX);
            Serial.println("[MT6701] WARNING: Magnetic field not optimal");
        }
    }

    // Verify encoder is readable before calibration
    float test_angle = encoder.getSensorAngle();
    if (isnan(test_angle) || isinf(test_angle)) {
        if (DEBUG_MOTOR) {
            Serial.println("[MT6701] ERROR: Cannot read encoder angle");
        }
        return false;
    }

    if (DEBUG_MOTOR) {
        Serial.print("[MT6701] Encoder reading OK: ");
        Serial.print(test_angle, 4);
        Serial.println(" rad");
    }

    // Run SimpleFOC calibration (aligns motor and encoder)
    if (DEBUG_MOTOR) {
        Serial.println("[FOC] Running motor alignment (initFOC)...");
        Serial.print("[DEBUG] Pre-initFOC encoder read: ");
        Serial.print(encoder.getSensorAngle(), 4);
        Serial.println(" rad");
        Serial.print("[DEBUG] Pre-initFOC shaft_angle: ");
        Serial.print(motor.shaft_angle, 4);
        Serial.println(" rad");
    }

    // Note: initFOC() returns 1 on success, 0 on failure
    int foc_result = motor.initFOC();

    if (DEBUG_MOTOR) {
        Serial.print("[FOC] initFOC() returned: ");
        Serial.println(foc_result);
        Serial.print("[DEBUG] Post-initFOC encoder read: ");
        Serial.print(encoder.getSensorAngle(), 4);
        Serial.println(" rad");
        Serial.print("[DEBUG] Post-initFOC shaft_angle: ");
        Serial.print(motor.shaft_angle, 4);
        Serial.println(" rad");

        if (foc_result == 1) {
            Serial.println("[FOC] Motor alignment complete - SUCCESS");
        } else {
            Serial.println("[FOC] Motor alignment FAILED!");
        }
    }

    // Check if initFOC actually succeeded
    if (foc_result != 1) {
        if (DEBUG_MOTOR) {
            Serial.println("[ERROR] initFOC failed - calibration unsuccessful");
        }
        return false;
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

float MotorController::getDirectEncoderAngle() {
    // Read encoder directly, bypassing SimpleFOC
    return encoder.getSensorAngle();
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
