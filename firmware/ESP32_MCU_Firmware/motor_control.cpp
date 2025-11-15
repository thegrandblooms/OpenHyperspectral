#include "motor_control.h"
#include "commands.h"  // Explicit include for state/mode definitions
#include "pid_auto_tuner.h"  // PID auto-tuning functionality
#include <Wire.h>      // I2C library for MT6701 encoder

//=============================================================================
// MT6701 Sensor Wrapper Implementation
//=============================================================================

MT6701Sensor::MT6701Sensor(uint8_t address)
    : encoder(address),
      current_angle(0.0),
      previous_angle(0.0),
      last_update_time(0) {
}

void MT6701Sensor::init() {
    // Initialize I2C for MT6701 encoder (using working approach from encoder_test)
    if (DEBUG_MOTOR) {
        Serial.println("[MT6701] Initializing I2C...");
        Serial.print("  SDA=GPIO");
        Serial.print(ENCODER_SDA);
        Serial.print(", SCL=GPIO");
        Serial.println(ENCODER_SCL);
    }

    Wire.begin(ENCODER_SDA, ENCODER_SCL);
    Wire.setClock(400000);  // 400kHz fast mode
    delay(100);  // Give I2C time to stabilize

    if (DEBUG_MOTOR) {
        Serial.println("[MT6701] Scanning I2C bus for encoder...");
        Wire.beginTransmission(ENCODER_I2C_ADDR);
        byte error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("[MT6701] Found device at 0x");
            if (ENCODER_I2C_ADDR < 16) Serial.print("0");
            Serial.println(ENCODER_I2C_ADDR, HEX);
        } else {
            Serial.print("[MT6701] ERROR: No device at 0x");
            if (ENCODER_I2C_ADDR < 16) Serial.print("0");
            Serial.print(ENCODER_I2C_ADDR, HEX);
            Serial.print(" (error code: ");
            Serial.print(error);
            Serial.println(")");
        }
    }

    // Initialize MT6701 encoder library from firmware/libraries/MT6701
    if (DEBUG_MOTOR) {
        Serial.println("[MT6701] Calling encoder.begin()...");
    }

    bool init_success = encoder.begin(&Wire);

    if (DEBUG_MOTOR) {
        Serial.print("[MT6701] encoder.begin() returned: ");
        Serial.println(init_success ? "SUCCESS" : "FAILURE");
    }

    if (!init_success) {
        if (DEBUG_MOTOR) {
            Serial.println("[MT6701] ERROR: Encoder initialization failed!");
        }
        return;
    }

    if (DEBUG_MOTOR) {
        Serial.println("[MT6701] Encoder initialized successfully");

        // Check field strength with raw register read
        uint8_t field_status = encoder.readFieldStatus();
        Serial.print("[MT6701] Field status register: 0x");
        if (field_status < 16) Serial.print("0");
        Serial.println(field_status, HEX);

        if (field_status == 0x00) {
            Serial.println("[MT6701] Magnetic field: GOOD");
        } else if (field_status == 0x01) {
            Serial.println("[MT6701] WARNING: Magnetic field TOO STRONG");
        } else if (field_status == 0x02) {
            Serial.println("[MT6701] WARNING: Magnetic field TOO WEAK");
        } else {
            Serial.println("[MT6701] WARNING: Unexpected field status (possible I2C failure)");
        }

        // Read raw angle register
        uint16_t raw_angle = encoder.readRawAngle();
        Serial.print("[MT6701] Raw angle register: ");
        Serial.print(raw_angle);
        Serial.print(" (0x");
        if (raw_angle < 0x1000) Serial.print("0");
        if (raw_angle < 0x100) Serial.print("0");
        if (raw_angle < 0x10) Serial.print("0");
        Serial.print(raw_angle, HEX);
        Serial.println(")");

        // Verify sensor is readable
        float test_read = encoder.readAngleRadians();
        Serial.print("[MT6701] Computed angle: ");
        Serial.print(test_read, 4);
        Serial.print(" rad (");
        Serial.print(test_read * 180.0 / PI, 2);
        Serial.println(" deg)");
    }

    // Initialize angle cache
    current_angle = encoder.readAngleRadians();
    previous_angle = current_angle;
    last_update_time = micros();

    if (DEBUG_MOTOR) {
        Serial.print("[MT6701] Initial angle cached: ");
        Serial.print(current_angle, 4);
        Serial.println(" rad");
    }
}

float MT6701Sensor::getSensorAngle() {
    // Return cached angle value (updated by update() method)
    // SimpleFOC calls update() first, then getSensorAngle()

    #ifdef DEBUG_SENSOR_READS
    static unsigned long last_print = 0;
    if (millis() - last_print > 100) {  // Limit to 10Hz to avoid spam
        Serial.print("[SENSOR] getSensorAngle() = ");
        Serial.print(current_angle, 4);
        Serial.println(" rad (cached)");
        last_print = millis();
    }
    #endif

    return current_angle;
}

void MT6701Sensor::update() {
    // SimpleFOC calls this before reading sensor angle via getSensorAngle()
    // We read the encoder here and cache the value

    // Save previous angle for velocity calculation
    previous_angle = current_angle;

    // Read fresh angle from MT6701
    current_angle = encoder.readAngleRadians();

    // Update timestamp
    last_update_time = micros();

    #ifdef DEBUG_MOTOR_VERBOSE
    if (DEBUG_MOTOR) {
        Serial.print("[MT6701] update() - angle: ");
        Serial.print(current_angle, 4);
        Serial.print(" rad, prev: ");
        Serial.print(previous_angle, 4);
        Serial.println(" rad");
    }
    #endif
}

int MT6701Sensor::needsSearch() {
    // MT6701 is an absolute encoder - it doesn't need electrical angle search
    // Return 0 to indicate no search needed

    if (DEBUG_MOTOR) {
        Serial.println("[MT6701] needsSearch() called - returning 0 (no search needed for absolute encoder)");
    }

    return 0;
}

bool MT6701Sensor::isFieldGood() {
    return encoder.isFieldGood();
}

float MT6701Sensor::getVelocity() {
    // Calculate velocity from cached angle changes
    // SimpleFOC may call this, or calculate velocity itself

    // Simple velocity calculation (could be improved with filtering)
    float angle_diff = current_angle - previous_angle;

    // Handle wraparound (crossing 0/2π boundary)
    if (angle_diff > PI) {
        angle_diff -= 2.0 * PI;
    } else if (angle_diff < -PI) {
        angle_diff += 2.0 * PI;
    }

    // Calculate time difference (convert microseconds to seconds)
    unsigned long current_time = micros();
    float dt = (current_time - last_update_time) / 1000000.0;

    // Avoid division by zero
    if (dt < 0.000001) {
        return 0.0;
    }

    return angle_diff / dt;
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
    if (DEBUG_MOTOR) {
        Serial.println("[MOTOR] Initializing MT6701 encoder...");
    }
    encoder.init();

    // Link encoder to motor
    if (DEBUG_MOTOR) {
        Serial.println("[MOTOR] Linking encoder to motor...");
    }
    motor.linkSensor(&encoder);

    if (DEBUG_MOTOR) {
        Serial.println("[MOTOR] Encoder linked to motor");
    }

    // Driver configuration
    driver.voltage_power_supply = VOLTAGE_PSU;
    driver.pwm_frequency = 20000;  // 20kHz PWM frequency

    if (DEBUG_MOTOR) {
        Serial.print("[MOTOR] Driver voltage_power_supply: ");
        Serial.print(driver.voltage_power_supply);
        Serial.println(" V");
        Serial.print("[MOTOR] Initializing driver... ");
    }
    driver.init();
    if (DEBUG_MOTOR) {
        Serial.println("Done");
    }

    // Link driver to motor
    if (DEBUG_MOTOR) {
        Serial.println("[MOTOR] Linking driver to motor...");
    }
    motor.linkDriver(&driver);
    if (DEBUG_MOTOR) {
        Serial.println("[MOTOR] Driver linked to motor");
    }

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

    // Enable SimpleFOC monitoring for debugging
    if (DEBUG_MOTOR) {
        Serial.println("[MOTOR] Enabling SimpleFOC monitoring...");
        motor.useMonitoring(Serial);
        motor.monitor_downsample = 0;  // Show all messages
    }

    // Initialize motor
    if (DEBUG_MOTOR) {
        Serial.print("[FOC] Calling motor.init()... ");
    }
    motor.init();
    if (DEBUG_MOTOR) {
        Serial.println("Done");
        Serial.println("[MOTOR] Checking motor initialization status:");
        Serial.print("  - Sensor linked: ");
        Serial.println(motor.sensor ? "YES" : "NO");
        Serial.print("  - Driver linked: ");
        Serial.println(motor.driver ? "YES" : "NO");
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

    // Motor must be enabled before FOC init for some operations
    if (DEBUG_MOTOR) {
        Serial.println("[FOC] Enabling motor for calibration...");
    }
    motor.enable();

    // Run SimpleFOC calibration (aligns motor and encoder)
    if (DEBUG_MOTOR) {
        Serial.println("[FOC] Running motor alignment (initFOC)...");
        Serial.print("[DEBUG] Pre-initFOC encoder read: ");
        float pre_angle = encoder.getSensorAngle();
        Serial.print(pre_angle, 4);
        Serial.println(" rad");
        Serial.print("[DEBUG] Pre-initFOC shaft_angle: ");
        Serial.print(motor.shaft_angle, 4);
        Serial.println(" rad");
        Serial.print("[DEBUG] Sensor needs search: ");
        Serial.println(encoder.needsSearch());

        // Test: manually read sensor multiple times to check for consistency
        Serial.println("[DEBUG] Testing sensor consistency (5 reads):");
        for (int i = 0; i < 5; i++) {
            float test_angle = encoder.getSensorAngle();
            Serial.print("  Read ");
            Serial.print(i+1);
            Serial.print(": ");
            Serial.print(test_angle, 4);
            Serial.print(" rad (diff: ");
            Serial.print((test_angle - pre_angle) * 180.0 / PI, 2);
            Serial.println(" deg)");
            delay(50);
        }
    }

    // Note: initFOC() returns 1 on success, 0 on failure
    // For absolute encoders, SimpleFOC's automatic detection sometimes fails
    // Try manual alignment: provide zero offset and let initFOC determine direction
    if (DEBUG_MOTOR) {
        Serial.println("[FOC] Manual approach: Align sensor to electrical zero...");
    }

    // Get current sensor angle
    float sensor_angle = encoder.getSensorAngle();

    // Calculate electrical angle (mechanical angle * pole_pairs)
    float electrical_angle = _normalizeAngle(sensor_angle * POLE_PAIRS);

    // Set zero electrical offset to current position
    motor.zero_electric_angle = electrical_angle;
    motor.sensor_direction = Direction::CW;  // Try CW first (can be reversed if motor spins wrong way)

    if (DEBUG_MOTOR) {
        Serial.print("[FOC] Sensor angle: ");
        Serial.print(sensor_angle, 4);
        Serial.println(" rad");
        Serial.print("[FOC] Electrical angle: ");
        Serial.print(electrical_angle, 4);
        Serial.println(" rad");
        Serial.print("[FOC] Zero electric offset set to: ");
        Serial.print(motor.zero_electric_angle, 4);
        Serial.println(" rad");
        Serial.println("[FOC] Sensor direction set to: CW");
        Serial.println("[FOC] Calling initFOC() - should skip direction detection...");
    }

    // Now call initFOC() - it should just verify and not do direction detection
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

    // Disable motor after initFOC (will be re-enabled when user calls enable())
    motor.disable();

    // Check if initFOC actually succeeded
    if (foc_result != 1) {
        if (DEBUG_MOTOR) {
            Serial.println("[ERROR] initFOC failed - calibration unsuccessful");
            Serial.println("[ERROR] Possible causes:");
            Serial.println("  - Power supply not connected to motor driver");
            Serial.println("  - Driver enable pin not working");
            Serial.println("  - Sensor reading but motor can't align");
            Serial.println("  - Voltage or current limits too restrictive");
        }
        return false;
    }

    if (DEBUG_MOTOR) {
        Serial.println("[SUCCESS] Motor calibration completed successfully!");
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

bool MotorController::autoTunePID(bool verbose) {
    // Ensure motor is calibrated and enabled
    if (!motor_calibrated) {
        if (verbose) {
            Serial.println("[TUNE] ERROR: Motor must be calibrated before PID tuning");
        }
        return false;
    }

    if (!motor_enabled) {
        if (verbose) {
            Serial.println("[TUNE] Enabling motor for tuning...");
        }
        enable();
        delay(500);
    }

    // Save current control mode and switch to position control
    uint8_t saved_mode = control_mode;
    if (control_mode != MODE_POSITION) {
        if (verbose) {
            Serial.println("[TUNE] Switching to position control mode...");
        }
        setControlMode(MODE_POSITION);
    }

    // Create tuner and run tuning
    PIDAutoTuner tuner(motor, encoder);
    bool success = tuner.runTuning(verbose);

    if (success) {
        // Apply optimal PID parameters
        float p, i, d, ramp;
        tuner.getOptimalPID(p, i, d, ramp);

        setPositionPID(p, i, d, ramp);

        if (verbose) {
            Serial.println("[TUNE] Optimal PID parameters applied!");
            Serial.println("[TUNE] To make permanent, update config.h:");
            Serial.print("  #define PID_P_POSITION ");
            Serial.println(p, 2);
            Serial.print("  #define PID_I_POSITION ");
            Serial.println(i, 2);
            Serial.print("  #define PID_D_POSITION ");
            Serial.println(d, 3);
        }
    } else {
        if (verbose) {
            Serial.println("[TUNE] ERROR: PID tuning failed!");
        }
    }

    // Restore original control mode
    if (saved_mode != control_mode) {
        setControlMode(saved_mode);
    }

    return success;
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
