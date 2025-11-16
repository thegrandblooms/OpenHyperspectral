#include "motor_control.h"
#include "commands.h"  // Explicit include for state/mode definitions
#include "pid_auto_tuner.h"  // PID auto-tuning functionality
#include <Wire.h>      // I2C library for MT6701 encoder

//=============================================================================
// MT6701 Sensor Wrapper Implementation
//=============================================================================

MT6701Sensor::MT6701Sensor(uint8_t address)
    : encoder(address),
      cached_raw_count(0),
      cached_degrees(0.0f),
      cached_radians(0.0f),
      previous_degrees(0.0f),
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

    // Initialize cached values
    cached_raw_count = encoder.readRawAngle();
    cached_degrees = rawToDegrees(cached_raw_count);
    cached_radians = degreesToRadians(cached_degrees);
    previous_degrees = cached_degrees;
    last_update_time = micros();

    if (DEBUG_MOTOR) {
        Serial.println("[MT6701] Initial values cached:");
        Serial.print("  Raw count: ");
        Serial.println(cached_raw_count);
        Serial.print("  Degrees: ");
        Serial.print(cached_degrees, 2);
        Serial.println("°");
        Serial.print("  Radians: ");
        Serial.print(cached_radians, 4);
        Serial.println(" rad");
    }
}

float MT6701Sensor::getSensorAngle() {
    // SIMPLEFOC BOUNDARY: Return angle in RADIANS (SimpleFOC expects this)
    // SimpleFOC calls update() first, then getSensorAngle()

    #ifdef DEBUG_SENSOR_READS
    static unsigned long last_print = 0;
    if (millis() - last_print > 100) {  // Limit to 10Hz to avoid spam
        Serial.print("[SENSOR] getSensorAngle() = ");
        Serial.print(cached_radians, 4);
        Serial.print(" rad (");
        Serial.print(cached_degrees, 2);
        Serial.println("°)");
        last_print = millis();
    }
    #endif

    return cached_radians;
}

void MT6701Sensor::update() {
    // SimpleFOC calls this before reading sensor angle via getSensorAngle()
    // We read the encoder here and cache ALL values (raw, degrees, radians)

    // Save previous for velocity calculation
    previous_degrees = cached_degrees;

    // Read raw encoder count (MINIMAL ABSTRACTION - closest to hardware)
    cached_raw_count = encoder.readRawAngle();

    // Convert to degrees (our preferred unit)
    cached_degrees = rawToDegrees(cached_raw_count);

    // Convert to radians for SimpleFOC
    cached_radians = degreesToRadians(cached_degrees);

    // Update timestamp
    last_update_time = micros();

    #ifdef DEBUG_MOTOR_VERBOSE
    if (DEBUG_MOTOR) {
        Serial.print("[MT6701] update() - Raw: ");
        Serial.print(cached_raw_count);
        Serial.print(", Deg: ");
        Serial.print(cached_degrees, 2);
        Serial.print("°, Rad: ");
        Serial.print(cached_radians, 4);
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
    // SIMPLEFOC BOUNDARY: Return velocity in RADIANS/SECOND
    // Calculate from degrees, then convert to radians

    float velocity_deg_s = getDegreesPerSecond();
    return degreesToRadians(velocity_deg_s);
}

//=============================================================================
// DIRECT ENCODER ACCESS (Our preferred interface)
//=============================================================================

uint16_t MT6701Sensor::getRawCount() {
    return cached_raw_count;
}

float MT6701Sensor::getDegrees() {
    return cached_degrees;
}

float MT6701Sensor::getDegreesPerSecond() {
    // Calculate velocity from angle changes in DEGREES
    float angle_diff_deg = cached_degrees - previous_degrees;

    // Handle wraparound (crossing 0/360° boundary)
    if (angle_diff_deg > 180.0f) {
        angle_diff_deg -= 360.0f;
    } else if (angle_diff_deg < -180.0f) {
        angle_diff_deg += 360.0f;
    }

    // Calculate time difference (convert microseconds to seconds)
    unsigned long current_time = micros();
    float dt = (current_time - last_update_time) / 1000000.0f;

    // Avoid division by zero
    if (dt < 0.000001f) {
        return 0.0f;
    }

    return angle_diff_deg / dt;
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
      target_position_deg(0.0f),
      target_velocity_deg_s(0.0f),
      max_velocity_deg_s(MAX_VELOCITY_DEG),
      max_acceleration_deg_s2(DEFAULT_ACCELERATION_DEG),
      current_limit_a(CURRENT_LIMIT),
      target_reached(false),
      position_tolerance_deg(POSITION_TOLERANCE_DEG) {
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
    driver.pwm_frequency = 25000;  // 25kHz PWM frequency (was 20kHz) - better for ESP32

    if (DEBUG_MOTOR) {
        Serial.print("[MOTOR] Driver voltage_power_supply: ");
        Serial.print(driver.voltage_power_supply);
        Serial.println(" V");
        Serial.print("[MOTOR] PWM frequency: ");
        Serial.print(driver.pwm_frequency);
        Serial.println(" Hz");
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

    // Set motor limits (CRITICAL FOR GIMBAL MOTORS)
    // SIMPLEFOC BOUNDARY: convert degrees to radians
    // Voltage limit: Research shows 6V is optimal for gimbal motors (prevents overshoot/cogging)
    motor.voltage_limit = VOLTAGE_LIMIT_GIMBAL;  // 6V for smooth gimbal operation (was 9.6V)
    motor.current_limit = current_limit_a;
    motor.velocity_limit = degreesToRadians(max_velocity_deg_s);

    if (DEBUG_MOTOR) {
        Serial.print("[MOTOR] Voltage limit set to: ");
        Serial.print(motor.voltage_limit);
        Serial.println(" V (optimized for gimbal)");
        Serial.print("[MOTOR] Current limit: ");
        Serial.print(motor.current_limit);
        Serial.println(" A");
        Serial.print("[MOTOR] Velocity limit: ");
        Serial.print(max_velocity_deg_s);
        Serial.println("°/s");
    }

    // Set FOC modulation (space vector PWM is more efficient)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // CRITICAL: Use voltage mode for gimbal motors (not torque mode)
    // Torque mode can cause cogging at low speeds
    motor.torque_controller = TorqueControlType::voltage;

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
    // Velocity PID (SIMPLEFOC: uses radians internally)
    motor.PID_velocity.P = PID_P_VELOCITY;
    motor.PID_velocity.I = PID_I_VELOCITY;
    motor.PID_velocity.D = PID_D_VELOCITY;
    motor.PID_velocity.output_ramp = PID_RAMP_VELOCITY;  // rad/s
    motor.LPF_velocity.Tf = PID_LPF_VELOCITY;  // Low-pass filter for gimbal motor stability
    motor.PID_velocity.limit = degreesToRadians(max_velocity_deg_s);

    // Position PID (SIMPLEFOC: uses radians internally)
    motor.P_angle.P = PID_P_POSITION;
    motor.P_angle.I = PID_I_POSITION;
    motor.P_angle.D = PID_D_POSITION;
    motor.P_angle.output_ramp = PID_RAMP_POSITION;  // rad/s
    motor.P_angle.limit = degreesToRadians(max_velocity_deg_s);

    // Current control PID (for FOC)
    // Current control PID (FOC - uses amperes, no unit conversion needed)
    motor.PID_current_q.P = PID_P_CURRENT;
    motor.PID_current_q.I = PID_I_CURRENT;
    motor.PID_current_q.D = PID_D_CURRENT;
    motor.PID_current_q.output_ramp = PID_RAMP_CURRENT;
    motor.PID_current_q.limit = current_limit_a;

    motor.PID_current_d.P = PID_P_CURRENT;
    motor.PID_current_d.I = PID_I_CURRENT;
    motor.PID_current_d.D = PID_D_CURRENT;
    motor.PID_current_d.output_ramp = PID_RAMP_CURRENT;
    motor.PID_current_d.limit = current_limit_a;

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

    // CRITICAL: Temporarily increase voltage for calibration
    // Gimbal motors need higher voltage to overcome static friction during alignment
    // Normal operation uses 6V, but calibration needs ~8-10V
    float normal_voltage_limit = motor.voltage_limit;
    motor.voltage_limit = 10.0f;  // Temporarily increase for calibration movement

    if (DEBUG_MOTOR) {
        Serial.print("[FOC] Temporarily increasing voltage limit to ");
        Serial.print(motor.voltage_limit);
        Serial.println("V for calibration");
    }

    // Run SimpleFOC calibration (aligns motor and encoder)
    if (DEBUG_MOTOR) {
        Serial.println("[FOC] Running motor alignment (initFOC)...");
        Serial.print("[DEBUG] Pre-initFOC encoder read: ");
        float pre_angle = encoder.getSensorAngle();
        Serial.print(pre_angle, 4);
        Serial.print(" rad (");
        Serial.print(radiansToDegrees(pre_angle), 2);
        Serial.print("°) Raw: ");
        Serial.println(encoder.getRawCount());
        Serial.print("[DEBUG] Pre-initFOC shaft_angle: ");
        Serial.print(motor.shaft_angle, 4);
        Serial.println(" rad");
        Serial.print("[DEBUG] Sensor needs search: ");
        Serial.println(encoder.needsSearch());

        // Test: manually read sensor multiple times to check for consistency
        Serial.println("[DEBUG] Testing sensor consistency (5 reads):");
        for (int i = 0; i < 5; i++) {
            encoder.update();  // Explicitly update before each read
            float test_angle = encoder.getSensorAngle();
            uint16_t test_raw = encoder.getRawCount();
            Serial.print("  Read ");
            Serial.print(i+1);
            Serial.print(": ");
            Serial.print(test_angle, 4);
            Serial.print(" rad (");
            Serial.print(radiansToDegrees(test_angle), 2);
            Serial.print("°) Raw: ");
            Serial.print(test_raw);
            Serial.print(" (diff: ");
            Serial.print((test_angle - pre_angle) * 180.0 / PI, 2);
            Serial.println("°)");
            delay(50);
        }
    }

    // Note: initFOC() returns 1 on success, 0 on failure
    // For absolute encoders like MT6701, SimpleFOC can auto-detect alignment
    if (DEBUG_MOTOR) {
        Serial.println("[FOC] Calling initFOC() with automatic sensor alignment...");
        encoder.update();  // Fresh read before initFOC
        float pre_angle = encoder.getSensorAngle();
        uint16_t pre_raw = encoder.getRawCount();
        Serial.print("[FOC] Pre-initFOC sensor angle: ");
        Serial.print(pre_angle, 4);
        Serial.print(" rad (");
        Serial.print(radiansToDegrees(pre_angle), 2);
        Serial.print("°) Raw: ");
        Serial.println(pre_raw);
    }

    // Let SimpleFOC automatically align sensor to electrical zero
    // For absolute encoders, this should:
    // 1. Read current sensor position
    // 2. Apply motor movement to find electrical zero
    // 3. Calculate zero_electric_angle offset
    // 4. Determine sensor_direction (CW/CCW)
    int foc_result = motor.initFOC();

    if (DEBUG_MOTOR) {
        Serial.print("[FOC] initFOC() returned: ");
        Serial.println(foc_result);

        // Update encoder and show post-calibration readings
        encoder.update();
        float post_angle = encoder.getSensorAngle();
        uint16_t post_raw = encoder.getRawCount();

        Serial.print("[DEBUG] Post-initFOC encoder read: ");
        Serial.print(post_angle, 4);
        Serial.print(" rad (");
        Serial.print(radiansToDegrees(post_angle), 2);
        Serial.print("°) Raw: ");
        Serial.println(post_raw);
        Serial.print("[DEBUG] Post-initFOC shaft_angle: ");
        Serial.print(motor.shaft_angle, 4);
        Serial.print(" rad (");
        Serial.print(radiansToDegrees(motor.shaft_angle), 2);
        Serial.println("°)");

        if (foc_result == 1) {
            Serial.println("[FOC] Motor alignment complete - SUCCESS");
            Serial.print("[FOC] Zero electric angle: ");
            Serial.print(motor.zero_electric_angle, 4);
            Serial.println(" rad");
            Serial.print("[FOC] Sensor direction: ");
            Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
        } else {
            Serial.println("[FOC] Motor alignment FAILED!");
        }
    }

    // Restore normal voltage limit for gimbal operation
    motor.voltage_limit = normal_voltage_limit;

    if (DEBUG_MOTOR) {
        Serial.print("[FOC] Restored voltage limit to ");
        Serial.print(motor.voltage_limit);
        Serial.println("V for normal operation");
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
    // SIMPLEFOC BOUNDARY: Read radians, display in degrees
    float current_angle_rad = motor.shaft_angle;
    motor.sensor_offset = current_angle_rad;

    if (DEBUG_MOTOR) {
        Serial.print("Home set at angle: ");
        Serial.print(radiansToDegrees(current_angle_rad), 2);
        Serial.print("° (");
        Serial.print(current_angle_rad, 4);
        Serial.println(" rad)");
    }
}

void MotorController::moveToPosition(float position_deg) {
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

    target_position_deg = position_deg;
    target_reached = false;
    system_state = STATE_MOVING;

    if (DEBUG_MOTOR) {
        Serial.print("Moving to position: ");
        Serial.print(position_deg, 2);
        Serial.println("°");
    }
}

void MotorController::setVelocity(float velocity_deg_s) {
    target_velocity_deg_s = constrain(velocity_deg_s, -max_velocity_deg_s, max_velocity_deg_s);

    if (DEBUG_MOTOR) {
        Serial.print("Velocity set to: ");
        Serial.print(target_velocity_deg_s, 2);
        Serial.println("°/s");
    }
}

void MotorController::setAcceleration(float accel_deg_s2) {
    max_acceleration_deg_s2 = constrain(accel_deg_s2, 0, MAX_ACCELERATION_DEG);

    // SIMPLEFOC BOUNDARY: Update PID ramp limits (convert to rad/s)
    float accel_rad_s2 = degreesToRadians(max_acceleration_deg_s2);
    motor.PID_velocity.output_ramp = accel_rad_s2;
    motor.P_angle.output_ramp = accel_rad_s2;

    if (DEBUG_MOTOR) {
        Serial.print("Acceleration set to: ");
        Serial.print(max_acceleration_deg_s2, 2);
        Serial.println("°/s²");
    }
}

void MotorController::setCurrentLimit(float new_current_limit_a) {
    current_limit_a = constrain(new_current_limit_a, 0, CURRENT_LIMIT);
    motor.current_limit = current_limit_a;

    if (DEBUG_MOTOR) {
        Serial.print("Current limit set to: ");
        Serial.print(current_limit_a, 2);
        Serial.println(" A");
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

void MotorController::setPositionPID(float p, float i, float d, float ramp_deg_s) {
    motor.P_angle.P = p;
    motor.P_angle.I = i;
    motor.P_angle.D = d;
    // SIMPLEFOC BOUNDARY: Convert ramp from deg/s to rad/s
    motor.P_angle.output_ramp = degreesToRadians(ramp_deg_s);

    if (DEBUG_MOTOR) {
        Serial.print("Position PID updated: P=");
        Serial.print(p, 2);
        Serial.print(", I=");
        Serial.print(i, 2);
        Serial.print(", D=");
        Serial.print(d, 3);
        Serial.print(", Ramp=");
        Serial.print(ramp_deg_s, 1);
        Serial.println("°/s");
    }
}

void MotorController::setVelocityPID(float p, float i, float d, float ramp_deg_s) {
    motor.PID_velocity.P = p;
    motor.PID_velocity.I = i;
    motor.PID_velocity.D = d;
    // SIMPLEFOC BOUNDARY: Convert ramp from deg/s to rad/s
    motor.PID_velocity.output_ramp = degreesToRadians(ramp_deg_s);

    if (DEBUG_MOTOR) {
        Serial.print("Velocity PID updated: P=");
        Serial.print(p, 2);
        Serial.print(", I=");
        Serial.print(i, 2);
        Serial.print(", D=");
        Serial.print(d, 3);
        Serial.print(", Ramp=");
        Serial.print(ramp_deg_s, 1);
        Serial.println("°/s");
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
        // Apply optimal PID parameters (tuner uses degrees)
        float p, i, d, ramp_deg_s;
        tuner.getOptimalPID(p, i, d, ramp_deg_s);

        setPositionPID(p, i, d, ramp_deg_s);

        if (verbose) {
            Serial.println("[TUNE] Optimal PID parameters applied!");
            Serial.println("[TUNE] To make permanent, update config.h:");
            Serial.print("  #define PID_P_POSITION ");
            Serial.println(p, 2);
            Serial.print("  #define PID_I_POSITION ");
            Serial.println(i, 2);
            Serial.print("  #define PID_D_POSITION ");
            Serial.println(d, 3);
            Serial.print("  #define PID_RAMP_POSITION_DEG ");
            Serial.println(ramp_deg_s, 1);
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

float MotorController::getCurrentPositionDeg() {
    // SIMPLEFOC BOUNDARY: Read radians, return degrees
    return radiansToDegrees(motor.shaft_angle);
}

float MotorController::getCurrentVelocityDegPerSec() {
    // SIMPLEFOC BOUNDARY: Read rad/s, return deg/s
    return radiansToDegrees(motor.shaft_velocity);
}

float MotorController::getTargetPositionDeg() {
    return target_position_deg;
}

float MotorController::getTargetVelocityDegPerSec() {
    return target_velocity_deg_s;
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

    float position_error_deg = abs(getCurrentPositionDeg() - target_position_deg);
    float velocity_deg_s = abs(getCurrentVelocityDegPerSec());

    // Consider target reached if position error is small and velocity is near zero
    return (position_error_deg < position_tolerance_deg) && (velocity_deg_s < VELOCITY_THRESHOLD_DEG);
}

uint8_t MotorController::getState() {
    return system_state;
}

uint16_t MotorController::getRawEncoderCount() {
    // Read raw encoder count (MINIMAL ABSTRACTION)
    return encoder.getRawCount();
}

float MotorController::getEncoderDegrees() {
    // Read encoder directly in degrees (bypassing SimpleFOC)
    return encoder.getDegrees();
}

void MotorController::update() {
    if (!motor_calibrated) {
        return;
    }

    // CRITICAL: Explicitly update encoder before FOC
    // SimpleFOC *should* call sensor->update() automatically in loopFOC(),
    // but we're seeing frozen encoder readings, so call it explicitly here
    encoder.update();

    // SIMPLEFOC: Run FOC algorithm (current control)
    motor.loopFOC();

    // SIMPLEFOC: Run motion control (position/velocity control)
    // BOUNDARY: Convert degrees to radians for SimpleFOC
    switch (control_mode) {
        case MODE_POSITION:
            // Convert target from degrees to radians
            motor.move(degreesToRadians(target_position_deg));

            // Check if target reached
            if (system_state == STATE_MOVING && isAtTarget()) {
                if (!target_reached) {
                    target_reached = true;
                    system_state = STATE_IDLE;

                    if (DEBUG_MOTOR) {
                        Serial.print("Target position reached: ");
                        Serial.print(target_position_deg, 2);
                        Serial.println("°");
                    }
                }
            }
            break;

        case MODE_VELOCITY:
            // Convert velocity from degrees/s to radians/s
            motor.move(degreesToRadians(target_velocity_deg_s));
            break;

        case MODE_TORQUE:
            // For torque mode, target is in current/torque units (no conversion)
            motor.move(target_velocity_deg_s);  // Reuse variable for torque
            break;
    }
}
