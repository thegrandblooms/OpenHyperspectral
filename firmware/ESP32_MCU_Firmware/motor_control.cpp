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
      last_update_time(0),
      force_needs_search(false) {
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
    // NOTE: SimpleFOC SHOULD call update() first, but let's verify

    // DEBUG: Throttled logging (disabled - too verbose even with throttling)
    // SimpleFOC calls this frequently during loopFOC(), use heartbeat instead

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

    // DEBUG: Throttled update logging (every 1 second max)
    if (DEBUG_MOTOR) {
        static unsigned long last_debug_print = 0;
        if (millis() - last_debug_print > 1000) {
            Serial.print("[SENSOR] update() - Raw: ");
            Serial.print(cached_raw_count);
            Serial.print(", Deg: ");
            Serial.print(cached_degrees, 2);
            Serial.print("°, Rad: ");
            Serial.print(cached_radians, 4);
            Serial.println(" rad");
            last_debug_print = millis();
        }
    }
}

int MT6701Sensor::needsSearch() {
    // MT6701 is an absolute encoder - normally doesn't need index search
    // BUT: SimpleFOC's alignment detection works better with needsSearch()=1
    // So during calibration, we temporarily return 1 to trigger index search logic

    if (force_needs_search) {
        if (DEBUG_MOTOR) {
            static bool logged = false;
            if (!logged) {
                Serial.println("[MT6701] needsSearch() = 1 (calibration mode - forcing index search)");
                logged = true;
            }
        }
        return 1;  // Force index search during calibration
    }

    if (DEBUG_MOTOR) {
        static unsigned long last_log = 0;
        if (millis() - last_log > 5000) {
            Serial.println("[MT6701] needsSearch() = 0 (absolute encoder)");
            last_log = millis();
        }
    }

    return 0;  // Normal operation: absolute encoder, no search needed
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

    // Set alignment voltage for sensor calibration
    // Gimbal motors need higher voltage to overcome friction during alignment
    // Default is 3V, but gimbal motors with ~10Ω resistance need 5-6V for reliable movement detection
    motor.voltage_sensor_align = 6.0f;

    if (DEBUG_MOTOR) {
        Serial.print("[MOTOR] Sensor alignment voltage: ");
        Serial.print(motor.voltage_sensor_align);
        Serial.println(" V");
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

bool MotorController::testMotorAlignment() {
    if (DEBUG_MOTOR) {
        Serial.println("=== Motor Alignment Diagnostic Test ===");
        Serial.println("This test will apply voltage at known electrical angles");
        Serial.println("Motor should move quickly then HOLD each position firmly");
        Serial.println("");
    }

    // Check encoder is working
    encoder.update();
    float initial_angle = encoder.getDegrees();
    if (isnan(initial_angle) || isinf(initial_angle)) {
        if (DEBUG_MOTOR) {
            Serial.println("[ERROR] Cannot read encoder!");
        }
        return false;
    }

    if (DEBUG_MOTOR) {
        Serial.print("[OK] Encoder reading: ");
        Serial.print(initial_angle, 2);
        Serial.println("°");
        Serial.println("");
    }

    // Enable motor
    motor.enable();

    // Test angles: 0°, 90°, 180°, 270° electrical
    float test_angles[] = {0, _PI_2, PI, _3PI_2};
    const char* angle_names[] = {"0°", "90°", "180°", "270°"};

    for (int i = 0; i < 4; i++) {
        if (DEBUG_MOTOR) {
            Serial.print("[TEST] Applying voltage at ");
            Serial.print(angle_names[i]);
            Serial.println(" electrical...");
        }

        // Apply voltage at known electrical angle
        motor.setPhaseVoltage(6.0, 0, test_angles[i]);

        // Wait for motor to settle
        delay(1500);

        // Read encoder position
        encoder.update();
        float mech_angle = encoder.getDegrees();

        if (DEBUG_MOTOR) {
            Serial.print("  → Motor settled at ");
            Serial.print(mech_angle, 2);
            Serial.println("° mechanical");
            Serial.println("  → Motor should be holding position FIRMLY");
            Serial.println("  → Try to rotate motor by hand - should resist");
            Serial.println("");
        }

        // Pause for user observation
        delay(500);
    }

    // Disable motor
    motor.disable();

    if (DEBUG_MOTOR) {
        Serial.println("[TEST] Motor disabled - should spin freely now");
        Serial.println("");
        Serial.println("=== Diagnostic Test Complete ===");
        Serial.println("If motor oscillated or felt weak:");
        Serial.println("  - Check driver fault pin");
        Serial.println("  - Check power supply voltage");
        Serial.println("  - Verify common ground connection");
        Serial.println("  - Check pole pairs setting (should be 7)");
        Serial.println("");
    }

    return true;
}

bool MotorController::runManualCalibration() {
    if (DEBUG_MOTOR) {
        Serial.println("=== Manual FOC Calibration ===");
        Serial.println("This will calculate zero_electric_angle and sensor_direction");
        Serial.println("");
    }

    // Check magnetic field strength
    if (!encoder.isFieldGood()) {
        if (DEBUG_MOTOR) {
            Serial.print("[WARNING] Magnetic field not optimal: 0x");
            Serial.println(encoder.getFieldStatus(), HEX);
        }
    }

    // Verify encoder is readable
    encoder.update();
    float test_angle = encoder.getSensorAngle();
    if (isnan(test_angle) || isinf(test_angle)) {
        if (DEBUG_MOTOR) {
            Serial.println("[ERROR] Cannot read encoder!");
        }
        return false;
    }

    // Enable motor
    if (DEBUG_MOTOR) {
        Serial.println("[CALIBRATION] Enabling motor...");
    }
    motor.enable();

    // Increase voltage temporarily for better alignment
    float normal_voltage_limit = motor.voltage_limit;
    motor.voltage_limit = 10.0f;

    // Apply voltage at 270° electrical (_3PI_2) and wait for settling
    if (DEBUG_MOTOR) {
        Serial.println("[CALIBRATION] Aligning motor to 270° electrical...");
    }

    motor.setPhaseVoltage(6.0, 0, _3PI_2);
    delay(700);  // Wait for motor to settle

    // Read encoder position (motor is now at 270° electrical)
    encoder.update();
    float angle_at_3pi2 = encoder.getSensorAngle();  // In radians

    if (DEBUG_MOTOR) {
        Serial.print("[CALIBRATION] At 270° electrical, encoder reads: ");
        Serial.print(angle_at_3pi2, 4);
        Serial.print(" rad (");
        Serial.print(radiansToDegrees(angle_at_3pi2), 2);
        Serial.println("°)");
    }

    // Apply voltage at 0° electrical and wait
    if (DEBUG_MOTOR) {
        Serial.println("[CALIBRATION] Aligning motor to 0° electrical...");
    }

    motor.setPhaseVoltage(6.0, 0, 0);
    delay(700);

    // Read encoder position (motor is now at 0° electrical)
    encoder.update();
    float angle_at_0 = encoder.getSensorAngle();  // In radians

    if (DEBUG_MOTOR) {
        Serial.print("[CALIBRATION] At 0° electrical, encoder reads: ");
        Serial.print(angle_at_0, 4);
        Serial.print(" rad (");
        Serial.print(radiansToDegrees(angle_at_0), 2);
        Serial.println("°)");
    }

    // Calculate sensor direction
    // If sensor increased when rotating from 270° to 0° electrical, it's CW
    float angle_change = angle_at_0 - angle_at_3pi2;

    // Handle wraparound (crossing 0/2π boundary)
    if (angle_change > PI) {
        angle_change -= 2.0 * PI;
    } else if (angle_change < -PI) {
        angle_change += 2.0 * PI;
    }

    Direction sensor_dir = (angle_change > 0) ? Direction::CW : Direction::CCW;

    if (DEBUG_MOTOR) {
        Serial.print("[CALIBRATION] Sensor direction: ");
        Serial.println(sensor_dir == Direction::CW ? "CW" : "CCW");
    }

    // Calculate zero_electric_angle
    // We know motor is at 0° electrical, encoder reads angle_at_0
    // zero_electric_angle is the offset between mechanical and electrical coordinates
    float electrical_from_encoder = angle_at_0 * POLE_PAIRS;
    electrical_from_encoder = normalizeRadians(electrical_from_encoder);

    float zero_elec_angle = normalizeRadians(electrical_from_encoder - 0);

    if (DEBUG_MOTOR) {
        Serial.print("[CALIBRATION] Calculated zero_electric_angle: ");
        Serial.print(zero_elec_angle, 4);
        Serial.print(" rad (");
        Serial.print(radiansToDegrees(zero_elec_angle), 2);
        Serial.println("°)");
        Serial.println("");
    }

    // Set calibration values
    motor.zero_electric_angle = zero_elec_angle;
    motor.sensor_direction = sensor_dir;

    // Restore voltage limit
    motor.voltage_limit = normal_voltage_limit;

    // Now call initFOC() which should skip alignment and succeed
    if (DEBUG_MOTOR) {
        Serial.println("[CALIBRATION] Calling initFOC() with preset values...");
    }

    int foc_result = motor.initFOC();

    // Disable motor after calibration
    motor.disable();

    if (DEBUG_MOTOR) {
        Serial.print("[CALIBRATION] initFOC() returned: ");
        Serial.println(foc_result);

        if (foc_result == 1) {
            Serial.println("");
            Serial.println("=== ✓ Calibration SUCCESS ===");
            Serial.println("Calibration values:");
            Serial.print("  zero_electric_angle = ");
            Serial.print(motor.zero_electric_angle, 4);
            Serial.println(" rad");
            Serial.print("  sensor_direction = ");
            Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
            Serial.println("");
            Serial.println("You can save these to NVS to skip calibration on future boots");
        } else {
            Serial.println("");
            Serial.println("=== ✗ Calibration FAILED ===");
            Serial.println("initFOC() failed even with manual calibration");
        }
    }

    return (foc_result == 1);
}

bool MotorController::runCalibration() {
    // Use manual calibration instead of SimpleFOC's automatic calibration
    // This is necessary because SimpleFOC's auto-calibration doesn't work
    // reliably with MT6701 I2C sensors (too slow for movement detection)

    if (DEBUG_MOTOR) {
        Serial.println("[MOTOR] Starting calibration...");
        Serial.println("[MOTOR] Using manual calibration (MT6701 I2C)");
        Serial.println("");
    }

    // Run manual calibration
    bool success = runManualCalibration();

    if (success) {
        if (DEBUG_MOTOR) {
            Serial.println("[SUCCESS] Motor calibration completed!");
            Serial.println("");
        }
    } else {
        if (DEBUG_MOTOR) {
            Serial.println("[ERROR] Motor calibration failed!");
            Serial.println("");
        }
    }

    return success;
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

float MotorController::getAbsolutePositionDeg() {
    // ABSOLUTE ENCODER (MT6701): Direct hardware read - THIS IS TRUTH
    // This bypasses SimpleFOC and reads the encoder directly via I2C
    // Use this for position checking, not SimpleFOC's shaft_angle
    return encoder.getDegrees();
}

float MotorController::getCurrentPositionDeg() {
    // SIMPLEFOC BOUNDARY: Read radians, return degrees
    // WARNING: This is SimpleFOC's internal state, which may lag or drift!
    // For accurate position, use getAbsolutePositionDeg() instead
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

    // CRITICAL: Use ENCODER position, not SimpleFOC shaft_angle!
    // SimpleFOC's shaft_angle may not update correctly, but the MT6701 absolute
    // encoder is the source of truth for actual motor position.

    // Update encoder to get fresh reading
    encoder.update();
    float current_position_deg = encoder.getDegrees();
    float current_velocity_deg_s = encoder.getDegreesPerSecond();

    float position_error_deg = abs(current_position_deg - target_position_deg);
    float velocity_deg_s = abs(current_velocity_deg_s);

    // Consider target reached if position error is small and velocity is near zero
    bool at_target = (position_error_deg < position_tolerance_deg) && (velocity_deg_s < VELOCITY_THRESHOLD_DEG);

    if (DEBUG_MOTOR && at_target) {
        static unsigned long last_debug = 0;
        if (millis() - last_debug > 1000) {  // Debug once per second
            Serial.print("[AT_TARGET] Encoder: ");
            Serial.print(current_position_deg, 2);
            Serial.print("°, Target: ");
            Serial.print(target_position_deg, 2);
            Serial.print("°, Error: ");
            Serial.print(position_error_deg, 2);
            Serial.print("°, Vel: ");
            Serial.print(velocity_deg_s, 2);
            Serial.println("°/s");
            last_debug = millis();
        }
    }

    return at_target;
}

uint8_t MotorController::getState() {
    return system_state;
}

void MotorController::updateEncoder() {
    // Force fresh encoder read from I2C (updates cached values)
    encoder.update();
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

    // SIMPLEFOC: Run FOC algorithm (current control)
    // NOTE: SimpleFOC automatically calls sensor->update() inside loopFOC()
    // Do NOT call encoder.update() manually here - it breaks the control loop!
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
