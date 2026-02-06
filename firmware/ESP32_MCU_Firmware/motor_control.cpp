#include "motor_control.h"
#include "commands.h"  // Explicit include for state/mode definitions
#include <Wire.h>      // I2C library for MT6701 encoder

//=============================================================================
// MT6701 Sensor Wrapper Implementation
//=============================================================================

MT6701Sensor::MT6701Sensor(uint8_t address)
    : encoder(address),
      cached_raw_count(0),
      cached_degrees(0.0f),
      cached_radians(0.0f),
      filtered_x(1.0f),                    // Initialize to angle=0 (cos(0)=1)
      filtered_y(0.0f),                    // Initialize to angle=0 (sin(0)=0)
      previous_degrees(0.0f),
      last_update_time(0),
      force_needs_search(false),
      call_count(0) {
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

    // Initialize cached values and Cartesian filter
    cached_raw_count = encoder.readRawAngle();
    cached_degrees = rawToDegrees(cached_raw_count);
    cached_radians = degreesToRadians(cached_degrees);
    previous_degrees = cached_degrees;
    last_update_time = micros();

    // Initialize Cartesian filter to current position
    filtered_x = cos(cached_radians);
    filtered_y = sin(cached_radians);

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
        Serial.print("  Cartesian filter initialized: (x=");
        Serial.print(filtered_x, 4);
        Serial.print(", y=");
        Serial.print(filtered_y, 4);
        Serial.println(")");
    }
}

float MT6701Sensor::getSensorAngle() {
    // Increment call counter for diagnostics
    call_count++;

    // SMARTKNOB PATTERN: Cartesian filtering eliminates angle wraparound discontinuities
    //
    // Traditional approach:
    //   Raw: 359.95° → 0.02° → 359.98° → 0.01° (discontinuous!)
    //   Filter: ???  (can't average 359.95° and 0.02° meaningfully)
    //
    // Cartesian approach:
    //   Raw angle → (x, y) coordinates → filter x and y separately → atan2 back to angle
    //   At 0/360° boundary: x≈1, y≈0 (smooth transition, no discontinuity!)
    //
    // This prevents velocity jumps and hunting at angle boundaries.
    // See: https://github.com/scottbez1/smartknob/blob/master/firmware/src/mt6701_sensor.cpp

    // Read raw encoder count (closest to hardware)
    cached_raw_count = encoder.readRawAngle();

    // Convert to radians (raw reading, not yet filtered)
    float raw_radians = degreesToRadians(rawToDegrees(cached_raw_count));

    // Convert angle to Cartesian coordinates
    float new_x = cos(raw_radians);
    float new_y = sin(raw_radians);

    // Apply exponential moving average filter to x and y components
    // alpha = 0.4: 40% new value, 60% previous value (smooths jitter while staying responsive)
    filtered_x = new_x * FILTER_ALPHA + filtered_x * (1.0f - FILTER_ALPHA);
    filtered_y = new_y * FILTER_ALPHA + filtered_y * (1.0f - FILTER_ALPHA);

    // Convert filtered Cartesian coordinates back to angle
    // atan2 handles all quadrants correctly and returns -π to π
    float filtered_radians = atan2(filtered_y, filtered_x);

    // Normalize to 0-2π range (SimpleFOC expects positive angles)
    if (filtered_radians < 0) {
        filtered_radians += 2.0f * PI;
    }

    // Cache values for diagnostics
    cached_degrees = radiansToDegrees(filtered_radians);
    cached_radians = filtered_radians;

    // Update previous values for our velocity calculation
    previous_degrees = cached_degrees;

    // Update timestamp
    last_update_time = micros();

    // DEBUG: Verbose sensor logging disabled - too noisy during tests
    // Enable this manually when debugging sensor filter issues
    // if (DEBUG_MOTOR) {
    //     static unsigned long last_debug_print = 0;
    //     if (millis() - last_debug_print > 1000) {
    //         Serial.print("[SENSOR] getSensorAngle() - Raw: ");
    //         Serial.print(cached_raw_count);
    //         Serial.print(", Raw°: ");
    //         Serial.print(radiansToDegrees(raw_radians), 2);
    //         Serial.print("°, Filtered°: ");
    //         Serial.print(cached_degrees, 2);
    //         Serial.print("° (x=");
    //         Serial.print(filtered_x, 3);
    //         Serial.print(", y=");
    //         Serial.print(filtered_y, 3);
    //         Serial.println(")");
    //         last_debug_print = millis();
    //     }
    // }

    // Return filtered angle in radians (SimpleFOC expects this)
    // Base class Sensor::update() will handle:
    // - Rotation tracking (angle_prev, full_rotations)
    // - Wraparound detection
    // - Velocity calculation
    return cached_radians;
}

// NOTE: getAngle() and update() are NOT overridden
// We use the standard SimpleFOC Sensor base class implementation which:
// - Calls our getSensorAngle() to get the current filtered angle
// - Tracks rotations automatically (full_rotations counter)
// - Calculates velocity from angle changes
// This matches the SmartKnob pattern and all official SimpleFOC drivers

void MT6701Sensor::resetRotationTracking() {
    // Reset sensor state to absolute encoder mode (no rotation tracking)
    // This fixes full_rotations corruption from calibration movements
    full_rotations = 0;
    angle_prev = getSensorAngle();
    // Note: timestamp management handled by base class on next update()

    // Rotation tracking reset message removed - shown in calibration summary
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
    return 0;  // Normal operation: absolute encoder, no search needed
}

bool MT6701Sensor::isFieldGood() {
    return encoder.isFieldGood();
}

// NOTE: getVelocity() NOT overridden - using SimpleFOC base class implementation
// Our previous override was broken: previous_degrees was always set equal to
// cached_degrees in getSensorAngle(), so velocity was always 0.
// The base class correctly tracks angle_prev vs vel_angle_prev separately.

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
      motor_enabled(false),
      motor_calibrated(false),
      target_position_deg(0.0f),
      move_start_time(0),
      settling_start_time(0),
      move_timeout_printed(true),  // Start as true so we don't print before first move
      at_target_printed(true),     // Start as true so we don't print before first move
      last_target_for_timeout(-999.0f),
      stream_enabled(STREAM_DEFAULT_ENABLED),
      stream_rate_hz(STREAM_RATE_HZ),
      stream_interval_us(1000000UL / STREAM_RATE_HZ),
      last_stream_time_us(0) {
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

    // Set motor limits
    // SIMPLEFOC BOUNDARY: convert degrees to radians
    // Voltage limit: 6V usually optimal for gimbal motors (prevents overshoot/cogging)
    motor.voltage_limit = VOLTAGE_LIMIT_GIMBAL;
    motor.current_limit = CURRENT_LIMIT;
    motor.velocity_limit = degreesToRadians(MAX_VELOCITY_DEG);

    if (DEBUG_MOTOR) {
        Serial.print("[MOTOR] Voltage limit set to: ");
        Serial.print(motor.voltage_limit);
        Serial.println(" V (optimized for gimbal)");
        Serial.print("[MOTOR] Current limit: ");
        Serial.print(motor.current_limit);
        Serial.println(" A");
        Serial.print("[MOTOR] Velocity limit: ");
        Serial.print(MAX_VELOCITY_DEG);
        Serial.println("°/s");
    }

    // Set FOC modulation (space vector PWM is more efficient)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // CRITICAL: Use voltage mode for gimbal motors (not torque mode)
    // Torque mode can cause cogging at low speeds
    motor.torque_controller = TorqueControlType::voltage;

    // Set motion control type to position control (angle mode)
    motor.controller = MotionControlType::angle;

    // Configure PID controllers
    // Velocity PID (SIMPLEFOC: uses radians internally)
    motor.PID_velocity.P = PID_P_VELOCITY;
    motor.PID_velocity.I = PID_I_VELOCITY;
    motor.PID_velocity.D = PID_D_VELOCITY;
    motor.PID_velocity.output_ramp = PID_RAMP_VELOCITY;  // rad/s
    motor.LPF_velocity.Tf = PID_LPF_VELOCITY;  // Low-pass filter for gimbal motor stability
    motor.PID_velocity.limit = degreesToRadians(MAX_VELOCITY_DEG);

    // Position PID (SIMPLEFOC: uses radians internally)
    motor.P_angle.P = PID_P_POSITION;
    motor.P_angle.I = PID_I_POSITION;
    motor.P_angle.D = PID_D_POSITION;
    motor.P_angle.output_ramp = PID_RAMP_POSITION;  // rad/s
    motor.P_angle.limit = degreesToRadians(MAX_VELOCITY_DEG);

    // CRITICAL FIX: Initialize angle low-pass filter (prevents shaft_angle reset bug)
    motor.LPF_angle.Tf = 0.0f;  // No filtering for absolute encoders

    // Current control PID (for FOC)
    // Current control PID (FOC - uses amperes, no unit conversion needed)
    motor.PID_current_q.P = PID_P_CURRENT;
    motor.PID_current_q.I = PID_I_CURRENT;
    motor.PID_current_q.D = PID_D_CURRENT;
    motor.PID_current_q.output_ramp = PID_RAMP_CURRENT;
    motor.PID_current_q.limit = CURRENT_LIMIT;

    motor.PID_current_d.P = PID_P_CURRENT;
    motor.PID_current_d.I = PID_I_CURRENT;
    motor.PID_current_d.D = PID_D_CURRENT;
    motor.PID_current_d.output_ramp = PID_RAMP_CURRENT;
    motor.PID_current_d.limit = CURRENT_LIMIT;

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
    // Run SimpleFOC calibration
    bool success = runCalibration();

    if (success) {
        motor_calibrated = true;
    } else {
        motor_calibrated = false;
    }

    return success;
}

bool MotorController::runManualCalibration() {
    // Verify encoder is readable
    encoder.update();
    float test_angle = encoder.getSensorAngle();
    if (isnan(test_angle) || isinf(test_angle)) {
        return false;
    }

    if (DEBUG_MOTOR) {
        Serial.print("  Pos: ");
        Serial.print(radiansToDegrees(test_angle), 1);
        Serial.print("° ");
    }
    motor.enable();

    // Increase voltage temporarily for better alignment
    float normal_voltage_limit = motor.voltage_limit;
    motor.voltage_limit = 10.0f;

    // Test at 4 electrical positions
    float test_angles[] = {0, _PI_2, PI, _3PI_2};
    float positions[4];

    for (int i = 0; i < 4; i++) {
        motor.setPhaseVoltage(6.0, 0, test_angles[i]);
        delay(700);
        encoder.update();
        positions[i] = encoder.getSensorAngle();
    }

    // Use 90° and 270° for calibration (they have best separation)

    float angle_at_pi2 = positions[1];    // 90° electrical
    float angle_at_3pi2 = positions[3];   // 270° electrical

    // Verify sufficient movement between calibration points
    float angle_diff = abs(angle_at_3pi2 - angle_at_pi2);
    // Handle wraparound
    if (angle_diff > PI) {
        angle_diff = 2.0 * PI - angle_diff;
    }

    if (angle_diff < 0.1) {  // Less than ~5.7 degrees
        motor.disable();
        motor.voltage_limit = normal_voltage_limit;
        return false;
    }

#if !USE_SIMPLEFOC_AUTO_CALIBRATION
    // MANUAL CALIBRATION: Calculate zero_electric_angle ourselves
    // Calculate sensor direction
    // If sensor increased when rotating from 90° to 270° electrical, it's CW
    float angle_change = angle_at_3pi2 - angle_at_pi2;

    // Handle wraparound (crossing 0/2π boundary)
    if (angle_change > PI) {
        angle_change -= 2.0 * PI;
    } else if (angle_change < -PI) {
        angle_change += 2.0 * PI;
    }

    Direction sensor_dir = (angle_change > 0) ? Direction::CW : Direction::CCW;

    // Override sensor direction to CW if configured (for consistent positive angles)
#if FORCE_SENSOR_DIRECTION_CW
    sensor_dir = Direction::CW;
#endif

    // Calculate zero_electric_angle
    // We know motor is at 90° electrical, encoder reads angle_at_pi2
    // zero_electric_angle is the offset between mechanical and electrical coordinates
    // NOTE: This calculation now uses the potentially-overridden sensor_dir
    float electrical_from_encoder = angle_at_pi2 * POLE_PAIRS * (sensor_dir == Direction::CW ? 1 : -1);
    electrical_from_encoder = normalizeRadians(electrical_from_encoder);

    float zero_elec_angle = normalizeRadians(electrical_from_encoder - _PI_2);

    // CRITICAL FIX: Add 225° offset to correct calibration
    // Discovered through TEST 8 diagnostics:
    // - 180° offset gave wrong direction movement
    // - 225° offset gave 110.6° forward movement (best result)
    // This compensates for phase wiring order and sensor polarity.
    // 225° = PI + PI/4 = 5*PI/4
    zero_elec_angle = normalizeRadians(zero_elec_angle + PI + (PI / 4.0f));

    // Set initial calibration values
    motor.zero_electric_angle = zero_elec_angle;
    motor.sensor_direction = sensor_dir;

    // Get current encoder position for direction verification
    encoder.update();
    float start_angle = encoder.getSensorAngle();

    // Apply voltage to create forward movement using calculated zero_electric_angle
    // electrical_angle = mechanical_angle * pole_pairs + zero_electric_angle
    // We want to apply field slightly ahead of current position to pull forward
    float current_electrical = start_angle * POLE_PAIRS * (sensor_dir == Direction::CW ? 1 : -1) + zero_elec_angle;
    float target_electrical = current_electrical + _PI_2;  // 90° ahead in electrical

    // Apply voltage for 300ms
    for (int i = 0; i < 6; i++) {
        motor.setPhaseVoltage(6.0f, 0, target_electrical);
        delay(50);
    }

    // Check movement
    encoder.update();
    float end_angle = encoder.getSensorAngle();
    float actual_movement = end_angle - start_angle;

    // Normalize movement to [-π, π]
    while (actual_movement > PI) actual_movement -= TWO_PI;
    while (actual_movement < -PI) actual_movement += TWO_PI;

    float movement_deg = radiansToDegrees(actual_movement);

    // If motor moved in WRONG direction (negative), add 180° to fix
    if (movement_deg < -3.0f) {  // Moved backward more than 3°
        zero_elec_angle = normalizeRadians(zero_elec_angle + PI);
        motor.zero_electric_angle = zero_elec_angle;
    } else if (movement_deg > 3.0f) {
        // Direction OK
    } else {
        // Movement too small - try with 180° offset and see if that's better
        float alt_zero = normalizeRadians(zero_elec_angle + PI);
        float alt_electrical = start_angle * POLE_PAIRS * (sensor_dir == Direction::CW ? 1 : -1) + alt_zero;
        float alt_target = alt_electrical + _PI_2;

        encoder.update();
        float alt_start = encoder.getSensorAngle();

        for (int i = 0; i < 6; i++) {
            motor.setPhaseVoltage(6.0f, 0, alt_target);
            delay(50);
        }

        encoder.update();
        float alt_end = encoder.getSensorAngle();
        float alt_movement = alt_end - alt_start;
        while (alt_movement > PI) alt_movement -= TWO_PI;
        while (alt_movement < -PI) alt_movement += TWO_PI;

        if (radiansToDegrees(alt_movement) > movement_deg + 5.0f) {
            // Alternative offset works better
            zero_elec_angle = alt_zero;
            motor.zero_electric_angle = zero_elec_angle;
        }
    }

    // Disable motor phases before continuing
    motor.setPhaseVoltage(0, 0, 0);

    // CRITICAL: Reset rotation tracking AFTER direction test
    // The direction test moved the motor, corrupting full_rotations
    encoder.resetRotationTracking();
#else
    // AUTO CALIBRATION: Let SimpleFOC calculate zero_electric_angle during initFOC()
    // CRITICAL: Reset calibration state to force fresh calibration
    // Without this, SimpleFOC skips calibration ("Skip dir calib", "Skip offset calib")
    // SimpleFOC uses Direction::UNKNOWN and NOT_SET (-12345.0f) as "uncalibrated" markers
    motor.sensor_direction = Direction::UNKNOWN;
    motor.zero_electric_angle = -12345.0f;  // NOT_SET value in SimpleFOC (NOT 1000.0f!)

    if (DEBUG_MOTOR) {
        Serial.println("");
        Serial.print("  Using SimpleFOC auto-calibration...");
    }
#endif

    // Restore voltage limit
    motor.voltage_limit = normal_voltage_limit;

    // Reset rotation tracking before initFOC()
    encoder.resetRotationTracking();

#if !USE_SIMPLEFOC_AUTO_CALIBRATION
    // Suppress SimpleFOC's verbose messages during initFOC
    Print* saved_monitor = motor.monitor_port;
    motor.monitor_port = nullptr;
#endif

    int foc_result = motor.initFOC();

#if !USE_SIMPLEFOC_AUTO_CALIBRATION
    motor.monitor_port = saved_monitor;
#endif

    if (foc_result == 1) {
        // Run stabilization cycles
        for (int i = 0; i < 20; i++) {
            motor.loopFOC();
            delay(1);
        }

        // Verify shaft_angle matches encoder
        float encoder_deg = radiansToDegrees(encoder.getSensorAngle());
        float shaft_deg = radiansToDegrees(motor.shaft_angle);
        float error = abs(shaft_deg - encoder_deg);
        if (error > 180.0f) error = 360.0f - error;

        if (DEBUG_MOTOR) {
            Serial.printf("Dir:%s Zero:%.1f° Enc:%.1f° FOC:%.1f° Err:%.1f°\n",
                motor.sensor_direction == Direction::CW ? "CW" : "CCW",
                radiansToDegrees(motor.zero_electric_angle),
                encoder_deg, shaft_deg, error);
        }
    }

    motor.disable();
    return (foc_result == 1);
}

bool MotorController::runCalibration() {
    // Use manual calibration instead of SimpleFOC's automatic calibration
    // This is necessary because SimpleFOC's auto-calibration doesn't work
    // reliably with MT6701 I2C sensors (too slow for movement detection)

    // runManualCalibration() now includes diagnostic test + calibration
    return runManualCalibration();
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

    // Run one sensor update so SimpleFOC's shaft_angle is current
    motor.loopFOC();

    // Set target to current position to prevent immediate movement
    target_position_deg = getPosition();

    // Auto-start encoder streaming so test sequences get logged
    if (!stream_enabled) {
        setStreamEnabled(true);
    }

    if (DEBUG_MOTOR) {
        Serial.print("✓ Motor enabled at ");
        Serial.print(target_position_deg, 1);
        Serial.println("°");
    }
}

void MotorController::disable() {
    motor.disable();
    motor_enabled = false;

    // Auto-stop encoder streaming
    if (stream_enabled) {
        setStreamEnabled(false);
    }

    // Reset timeout tracking
    move_start_time = 0;
    move_timeout_printed = true;

    if (DEBUG_MOTOR) {
        Serial.println("Motor disabled");
    }
}

void MotorController::stop() {
    // Emergency stop - disable motor immediately
    motor.disable();
    motor_enabled = false;

    // Reset timeout tracking
    move_start_time = 0;
    move_timeout_printed = true;

    if (DEBUG_MOTOR) {
        Serial.println("Motor stopped (emergency)");
    }
}

void MotorController::setHome() {
    // Absolute encoders don't need homing - they retain position after power-off
    // This function just logs the current position as a reference point
    // All position commands work in absolute coordinates (0-360°)

    if (DEBUG_MOTOR) {
        Serial.print("Current absolute position: ");
        Serial.print(radiansToDegrees(motor.shaft_angle), 2);
        Serial.println("°");
        Serial.println("  Note: Absolute encoders work in fixed coordinates (0-360°)");
        Serial.println("  All move commands use absolute positions");
    }
}

void MotorController::moveToPosition(float position_deg) {
    if (!motor_enabled) {
        if (DEBUG_MOTOR) {
            Serial.println("Cannot move - motor not enabled");
        }
        return;
    }

    // Normalize target position to 0-360° range
    // This handles inputs like 720° (becomes 0°) or -90° (becomes 270°)
    position_deg = fmod(position_deg, 360.0f);
    if (position_deg < 0) position_deg += 360.0f;

    // Detect new move command (target changed)
    if (abs(position_deg - last_target_for_timeout) > 0.1f) {
        // New move - reset timeout tracking and AT_TARGET flag
        move_start_time = millis();
        move_timeout_printed = false;
        at_target_printed = false;  // Reset so we print once when target reached
        last_target_for_timeout = position_deg;
    }

    // Store target position (absolute coordinates 0-360°)
    target_position_deg = position_deg;

    // Debug output removed - test harness already shows move commands
}

void MotorController::setVelocity(float velocity_deg_s) {
    // Legacy function - simplified interface uses position control only
    // Update velocity limit directly in SimpleFOC
    float velocity_rad_s = degreesToRadians(constrain(velocity_deg_s, 0, MAX_VELOCITY_DEG));
    motor.velocity_limit = velocity_rad_s;

    if (DEBUG_MOTOR) {
        Serial.print("Velocity limit set to: ");
        Serial.print(velocity_deg_s, 2);
        Serial.println("°/s");
    }
}

void MotorController::setAcceleration(float accel_deg_s2) {
    // Update PID ramp limits directly (this affects acceleration)
    float accel_rad_s2 = degreesToRadians(constrain(accel_deg_s2, 0, MAX_ACCELERATION_DEG));
    motor.PID_velocity.output_ramp = accel_rad_s2;
    motor.P_angle.output_ramp = accel_rad_s2;

    if (DEBUG_MOTOR) {
        Serial.print("Acceleration set to: ");
        Serial.print(accel_deg_s2, 2);
        Serial.println("°/s²");
    }
}

void MotorController::setCurrentLimit(float new_current_limit_a) {
    // Update current limit directly in SimpleFOC
    float limited_current = constrain(new_current_limit_a, 0, CURRENT_LIMIT);
    motor.current_limit = limited_current;

    if (DEBUG_MOTOR) {
        Serial.print("Current limit set to: ");
        Serial.print(limited_current, 2);
        Serial.println(" A");
    }
}

void MotorController::setControlMode(uint8_t mode) {
    // Simplified interface: position control only
    // This function is kept for compatibility but does nothing
    if (DEBUG_MOTOR) {
        Serial.print("Control mode: ");
        Serial.print(mode);
        Serial.println(" (note: simplified interface uses position control only)");
    }
}

uint8_t MotorController::getControlMode() {
    // Simplified interface: always in position control mode
    return MODE_POSITION;
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
    // PID auto-tuner removed - use manual PID tuning via setPositionPID()
    if (verbose) {
        Serial.println("[TUNE] PID auto-tuner removed. Use manual tuning.");
    }
    return false;
}

float MotorController::getPosition() {
    // Return current position from SimpleFOC (absolute 0-360°)
    // SimpleFOC's shaft_angle is kept synchronized by loopFOC()
    // CRITICAL FIX: Normalize to 0-360° range
    // SimpleFOC accumulates full rotations internally (e.g., -13076°)
    // but for absolute encoder usage, we want the physical position (0-360°)
    float degrees = radiansToDegrees(motor.shaft_angle);
    degrees = fmod(degrees, 360.0f);
    if (degrees < 0) degrees += 360.0f;
    return degrees;
}

float MotorController::getAbsolutePositionDeg() {
    // Alias for getPosition() - kept for compatibility
    return getPosition();
}

float MotorController::getCurrentPositionDeg() {
    // Alias for getPosition() - kept for compatibility
    return getPosition();
}

float MotorController::getCurrentVelocityDegPerSec() {
    // SIMPLEFOC BOUNDARY: Read rad/s, return deg/s
    return radiansToDegrees(motor.shaft_velocity);
}

float MotorController::getTargetPositionDeg() {
    return target_position_deg;
}

float MotorController::getTargetVelocityDegPerSec() {
    // Simplified interface: return current velocity limit
    return radiansToDegrees(motor.velocity_limit);
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
    // FIX: Use motor.shaft_angle (already updated by loopFOC)
    // Do NOT call encoder.update() - it creates a race condition!
    // SimpleFOC's loopFOC() already updated the sensor and shaft_angle

    float current_position_rad = motor.shaft_angle;
    float current_velocity_rad_s = motor.shaft_velocity;

    // Normalize current position to 0-360° range (handles negative angles from CCW)
    float current_position_deg = radiansToDegrees(current_position_rad);
    while (current_position_deg < 0) current_position_deg += 360.0f;
    while (current_position_deg >= 360.0f) current_position_deg -= 360.0f;

    float velocity_deg_s = abs(radiansToDegrees(current_velocity_rad_s));

    // Calculate position error with wraparound handling
    // Both angles now guaranteed to be in 0-360° range
    float position_error_deg = abs(current_position_deg - target_position_deg);

    // Handle wraparound (e.g., target=5°, current=355° → error=10°, not 350°)
    if (position_error_deg > 180.0f) {
        position_error_deg = 360.0f - position_error_deg;
    }

    // Target reached if position error is small and velocity is low
    // Use constants from config.h
    bool at_target = (position_error_deg < POSITION_TOLERANCE_DEG) &&
                     (velocity_deg_s < VELOCITY_THRESHOLD_DEG);

    // Print AT_TARGET only ONCE per move command (not every second)
    if (DEBUG_MOTOR && at_target && !at_target_printed) {
        at_target_printed = true;  // Only print once until next move command
        Serial.print("[AT_TARGET] Current: ");
        Serial.print(current_position_deg, 2);
        Serial.print("°, Target: ");
        Serial.print(target_position_deg, 2);
        Serial.print("°, Error: ");
        Serial.print(position_error_deg, 2);
        Serial.print("°, Vel: ");
        Serial.print(velocity_deg_s, 2);
        Serial.println("°/s");
    }

    return at_target;
}

uint8_t MotorController::getState() {
    // Simplified state tracking based on calibration and enable status
    if (!motor_calibrated) {
        return STATE_ERROR;  // Not calibrated = error state
    }
    if (!motor_enabled) {
        return STATE_IDLE;  // Calibrated but not enabled = idle
    }
    // Motor is enabled and calibrated - check if at target
    if (isAtTarget()) {
        return STATE_IDLE;  // At target = idle
    }
    return STATE_MOVING;  // Moving to target
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

    // SimpleFOC: Read sensor, update shaft_angle, compute electrical angle
    // loopFOC() calls sensor->update() which calls our getSensorAngle()
    // SimpleFOC tracks continuous angle via full_rotations counter
    motor.loopFOC();

    // Target calculation using SimpleFOC's shaft_angle directly
    // shaft_angle is continuous (can be <0 or >2π) via full_rotations tracking,
    // so we compute shortest-path error and express target in the same space
    float target_rad = degreesToRadians(target_position_deg);

    float current_wrapped = fmod(motor.shaft_angle, TWO_PI);
    if (current_wrapped < 0) current_wrapped += TWO_PI;

    float error_rad = target_rad - current_wrapped;

    // Normalize error to [-π, +π] (shortest path)
    while (error_rad > PI) error_rad -= TWO_PI;
    while (error_rad < -PI) error_rad += TWO_PI;

    // Express target in continuous space so PID doesn't see jumps at boundary
    float normalized_target_rad = motor.shaft_angle + error_rad;

    motor.move(normalized_target_rad);

    // Emit encoder stream data (rate-limited, no-op when streaming is off).
    // Called here so test loops that call update() also get stream data,
    // not just the main Arduino loop().
    emitStreamLine();

    // NOTE: AT_TARGET debug removed from here - it was using raw shaft_angle
    // which caused confusing duplicate logs with non-normalized angles.
    // The isAtTarget() function already has proper normalized debug output.

    // Move timeout detection with settling awareness
    if (motor_enabled && move_start_time > 0 && !move_timeout_printed) {
        bool at_target = isAtTarget();

        if (at_target) {
            // Move completed - reset timeout tracking
            move_timeout_printed = true;
            settling_start_time = 0;
        } else {
            // Check if we're "close enough" (settling)
            float enc_deg = encoder.getDegrees();
            float vel_deg_s = abs(radiansToDegrees(motor.shaft_velocity));
            float pos_error = abs(enc_deg - target_position_deg);
            if (pos_error > 180.0f) pos_error = 360.0f - pos_error;

            bool is_settling = (pos_error < SETTLING_ERROR_DEG) && (vel_deg_s < SETTLING_VEL_DEG_S);

            if (is_settling) {
                // Start or continue settling timer
                if (settling_start_time == 0) {
                    settling_start_time = millis();
                } else if (millis() - settling_start_time > SETTLING_TIME_MS) {
                    // Settled! Consider this a success, not a timeout
                    move_timeout_printed = true;  // Don't print timeout
                    settling_start_time = 0;
                }
            } else {
                // Not close enough - reset settling timer
                settling_start_time = 0;

                // Check for hard timeout (truly stuck)
                if (millis() - move_start_time > MOVE_TIMEOUT_MS) {
                    move_timeout_printed = true;

                    float foc_deg = radiansToDegrees(motor.shaft_angle);
                    foc_deg = fmod(foc_deg, 360.0f);
                    if (foc_deg < 0) foc_deg += 360.0f;

                    Serial.print("[MOVE_TIMEOUT] Target:");
                    Serial.print(target_position_deg, 1);
                    Serial.print("° Enc:");
                    Serial.print(enc_deg, 1);
                    Serial.print("° Err:");
                    Serial.print(pos_error, 1);
                    Serial.print("° Vel:");
                    Serial.print(vel_deg_s, 1);
                    Serial.print("°/s Vq:");
                    Serial.print(motor.voltage.q, 2);
                    Serial.println("V");

                    Serial.print("  Dir:");
                    Serial.print(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
                    Serial.print(" Zero:");
                    Serial.print(radiansToDegrees(motor.zero_electric_angle), 1);
                    Serial.print("° Δ(Enc-FOC):");
                    Serial.print(enc_deg - foc_deg, 1);
                    Serial.println("°");
                }
            }
        }
    }
}

//=============================================================================
// ENCODER STREAMING
//=============================================================================
// Tagged CSV lines: $ENC,timestamp_ms,position_deg,velocity_deg_s,target_deg
// Python filters on '$ENC,' prefix; debug text passes through untagged.
// This keeps the serial monitor usable for debugging while streaming data.

void MotorController::setStreamEnabled(bool enabled) {
    stream_enabled = enabled;
    last_stream_time_us = micros();  // Reset timer to avoid burst on enable
    if (enabled) {
        Serial.print("$STREAM_ON,");
        Serial.println(millis());
    } else {
        Serial.print("$STREAM_OFF,");
        Serial.println(millis());
    }
}

void MotorController::setStreamRate(uint16_t hz) {
    hz = constrain(hz, STREAM_MIN_RATE_HZ, STREAM_MAX_RATE_HZ);
    stream_rate_hz = hz;
    stream_interval_us = 1000000UL / hz;
}

void MotorController::emitStreamLine() {
    if (!stream_enabled) return;

    unsigned long now_us = micros();
    if (now_us - last_stream_time_us < stream_interval_us) return;
    last_stream_time_us = now_us;

    // Use already-cached values from this loop iteration (no extra I2C read)
    float pos_deg = getPosition();
    float vel_deg_s = getCurrentVelocityDegPerSec();
    float target_deg = target_position_deg;

    // Tagged CSV: $ENC,timestamp_ms,position_deg,velocity_deg_s,target_deg
    // Using Serial.print chain for minimal memory allocation on ESP32
    Serial.print("$ENC,");
    Serial.print(millis());
    Serial.print(',');
    Serial.print(pos_deg, 2);
    Serial.print(',');
    Serial.print(vel_deg_s, 2);
    Serial.print(',');
    Serial.println(target_deg, 2);
}

void MotorController::emitScanStart() {
    Serial.print("$SCAN_START,");
    Serial.println(millis());
}

void MotorController::emitScanEnd() {
    Serial.print("$SCAN_END,");
    Serial.println(millis());
}
