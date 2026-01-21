#include "motor_control.h"
#include "commands.h"  // Explicit include for state/mode definitions
#include "nvs_storage.h"  // NVS storage for calibration/PID persistence
#include "pid_tuner.h"    // Firmware-based PID auto-tuner
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

    // Debug message suppressed - too verbose during calibration

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
      target_position_deg(0.0f) {
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

bool MotorController::testDriverPhases() {
    if (DEBUG_MOTOR) {
        Serial.println("");
        Serial.println("╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║              Driver Phase Output Test                          ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.println("");
        Serial.println("This test directly drives each motor phase to verify all three");
        Serial.println("driver outputs (IN1, IN2, IN3) are working correctly.");
        Serial.println("");
        Serial.println("Expected: Motor should move and hold firmly at each test");
        Serial.println("If motor doesn't move during a test, that phase has a problem.");
        Serial.println("─────────────────────────────────────────────────────────────────");
        Serial.println("");
    }

    // Check driver fault status
    pinMode(MOTOR_FAULT, INPUT_PULLUP);  // nFT is active LOW
    bool fault_status = digitalRead(MOTOR_FAULT);
    if (DEBUG_MOTOR) {
        Serial.print("[DRIVER] Fault pin (nFT) before enable: ");
        Serial.println(fault_status ? "HIGH (OK)" : "LOW (FAULT!)");
        if (!fault_status) {
            Serial.println("[WARNING] Driver showing fault before motor enabled!");
        }
        Serial.println("");
    }

    // Enable driver
    motor.enable();
    delay(100);

    // Check fault after enable
    fault_status = digitalRead(MOTOR_FAULT);
    if (DEBUG_MOTOR) {
        Serial.print("[DRIVER] Fault pin after enable: ");
        Serial.println(fault_status ? "HIGH (OK)" : "LOW (FAULT!)");
        Serial.println("");
    }

    // Read initial encoder position
    encoder.update();
    float start_pos = encoder.getDegrees();
    if (DEBUG_MOTOR) {
        Serial.print("[ENCODER] Starting position: ");
        Serial.print(start_pos, 2);
        Serial.println("°");
        Serial.println("");
    }

    // Test 6 different phase combinations to verify all outputs work
    // For a 3-phase motor, voltage creates a field vector
    // We test positions 60° apart in electrical angle
    struct PhaseTest {
        float electrical_angle;
        const char* description;
    };

    PhaseTest tests[] = {
        {0.0,          "Phase A positive (0° electrical)"},
        {_PI_3,        "Phase A→B transition (60° electrical)"},
        {_PI_2,        "Phase B positive (90° electrical)"},
        {2.0*_PI_3,    "Phase B→C transition (120° electrical)"},
        {PI,           "Phase C positive (180° electrical)"},
        {4.0*_PI_3,    "Phase C→A transition (240° electrical)"}
    };

    for (int i = 0; i < 6; i++) {
        if (DEBUG_MOTOR) {
            Serial.print("[TEST ");
            Serial.print(i + 1);
            Serial.print("/6] ");
            Serial.println(tests[i].description);
        }

        // Apply voltage at this electrical angle
        motor.setPhaseVoltage(6.0, 0, tests[i].electrical_angle);
        delay(1000);

        // Read position
        encoder.update();
        float pos = encoder.getDegrees();
        float movement = abs(pos - start_pos);
        if (movement > 180.0) movement = 360.0 - movement;  // Handle wraparound

        // Check fault
        fault_status = digitalRead(MOTOR_FAULT);

        if (DEBUG_MOTOR) {
            Serial.print("  → Position: ");
            Serial.print(pos, 2);
            Serial.print("° (moved ");
            Serial.print(movement, 2);
            Serial.print("° from start)");
            Serial.println();
            Serial.print("  → Fault: ");
            Serial.println(fault_status ? "OK" : "FAULT!");
            Serial.println("");
        }

        delay(500);
    }

    // Disable motor
    motor.disable();

    if (DEBUG_MOTOR) {
        Serial.println("╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║                   Phase Test Complete                          ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.println("");
        Serial.println("Analysis:");
        Serial.println("  • If motor moved to 6 different positions → All phases OK");
        Serial.println("  • If only 2-3 positions → One or more phases not working");
        Serial.println("  • If no movement → Driver not powered or enable not working");
        Serial.println("  • If fault pin showed LOW → Check power supply and current");
        Serial.println("");
    }

    return true;
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

    // Check driver fault status
    pinMode(MOTOR_FAULT, INPUT_PULLUP);  // nFT is active LOW
    bool fault_status = digitalRead(MOTOR_FAULT);
    if (DEBUG_MOTOR) {
        Serial.print("[DRIVER] Fault pin (nFT): ");
        Serial.println(fault_status ? "HIGH (OK)" : "LOW (FAULT!)");
        if (!fault_status) {
            Serial.println("[ERROR] Driver is in fault state!");
            Serial.println("[ERROR] Check power supply, current limit, or thermal shutdown");
        }
        Serial.println("");
    }

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

        // Check fault pin after applying voltage
        fault_status = digitalRead(MOTOR_FAULT);

        if (DEBUG_MOTOR) {
            Serial.print("  → Motor settled at ");
            Serial.print(mech_angle, 2);
            Serial.println("° mechanical");
            Serial.print("  → Fault pin: ");
            Serial.println(fault_status ? "OK" : "FAULT!");
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
        Serial.println("\n[CALIBRATION] Starting...");
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

    if (DEBUG_MOTOR) {
        Serial.print("  Encoder: ");
        Serial.print(radiansToDegrees(test_angle), 1);
        Serial.print("° | Testing 4 positions: ");
    }
    motor.enable();

    // Increase voltage temporarily for better alignment
    float normal_voltage_limit = motor.voltage_limit;
    motor.voltage_limit = 10.0f;

    // Diagnostic test at 4 positions
    float test_angles[] = {0, _PI_2, PI, _3PI_2};
    float positions[4];

    for (int i = 0; i < 4; i++) {
        motor.setPhaseVoltage(6.0, 0, test_angles[i]);
        delay(700);

        encoder.update();
        positions[i] = encoder.getSensorAngle();

        if (DEBUG_MOTOR) {
            Serial.print(radiansToDegrees(positions[i]), 1);
            Serial.print("° ");
        }
    }

    if (DEBUG_MOTOR) {
        Serial.println("✓");
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
        if (DEBUG_MOTOR) {
            Serial.print("✗ Motor didn't move (");
            Serial.print(radiansToDegrees(angle_diff), 1);
            Serial.println("°) - check power/driver/wiring");
        }
        motor.disable();
        motor.voltage_limit = normal_voltage_limit;
        return false;
    }

    if (DEBUG_MOTOR) {
        Serial.print("  Movement: ");
        Serial.print(radiansToDegrees(angle_diff), 1);
        Serial.print("°");
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

    // CRITICAL FIX: Override sensor direction to CW if configured
    // MUST happen BEFORE zero_electric_angle calculation!
    // When sensor_direction = CCW, SimpleFOC produces negative shaft_angle values
    // which breaks position error calculations and velocity estimation
    // Forcing CW ensures all angles are positive (0-2π range)
#if FORCE_SENSOR_DIRECTION_CW
    if (DEBUG_MOTOR && sensor_dir == Direction::CCW) {
        Serial.print(" [OVERRIDE: CCW→CW]");
    }
    sensor_dir = Direction::CW;
#endif

    if (DEBUG_MOTOR) {
        Serial.print(" Dir:");
        Serial.print(sensor_dir == Direction::CW ? "CW" : "CCW");
    }

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

    // DIRECTION VERIFICATION: Use setPhaseVoltage() to test direction
    // (Don't use FOC loop - it can spin wildly with wrong calibration!)
    if (DEBUG_MOTOR) {
        Serial.print(" ZeroAngle:");
        Serial.print(radiansToDegrees(zero_elec_angle), 1);
        Serial.print("°");
    }

    // Get current encoder position
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

    if (DEBUG_MOTOR) {
        Serial.print(" DirTest:");
        Serial.print(movement_deg, 1);
        Serial.print("°");
    }

    // If motor moved in WRONG direction (negative), add 180° to fix
    if (movement_deg < -3.0f) {  // Moved backward more than 3°
        zero_elec_angle = normalizeRadians(zero_elec_angle + PI);
        motor.zero_electric_angle = zero_elec_angle;

        if (DEBUG_MOTOR) {
            Serial.print(" [REVERSED! New:");
            Serial.print(radiansToDegrees(zero_elec_angle), 1);
            Serial.print("°]");
        }
    } else if (movement_deg > 3.0f) {
        if (DEBUG_MOTOR) {
            Serial.print(" [OK]");
        }
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
            if (DEBUG_MOTOR) {
                Serial.print(" [ALT BETTER:");
                Serial.print(radiansToDegrees(zero_elec_angle), 1);
                Serial.print("°]");
            }
        } else {
            if (DEBUG_MOTOR) {
                Serial.print(" [WEAK]");
            }
        }
    }

    // Disable motor phases before continuing
    motor.setPhaseVoltage(0, 0, 0);

    if (DEBUG_MOTOR) {
        Serial.println("");
    }

    // CRITICAL: Reset rotation tracking AFTER direction test
    // The direction test moved the motor, corrupting full_rotations
    encoder.resetRotationTracking();
#else
    // AUTO CALIBRATION: Let SimpleFOC calculate zero_electric_angle during initFOC()
    // Don't set zero_electric_angle or sensor_direction - SimpleFOC will do it
    if (DEBUG_MOTOR) {
        Serial.println("");
        Serial.print("  Using SimpleFOC auto-calibration...");
    }
#endif

    // Restore voltage limit
    motor.voltage_limit = normal_voltage_limit;

    // CRITICAL FIX: Reset rotation tracking before initFOC()
    // During calibration, setPhaseVoltage() caused instant electrical jumps (300° → 0°)
    // which fooled SimpleFOC's wraparound detection, corrupting full_rotations counter.
    // Reset to absolute encoder mode (full_rotations = 0) before initFOC().
    encoder.resetRotationTracking();

    // Now call initFOC()
    if (DEBUG_MOTOR) {
        Serial.print("  Initializing FOC...");
    }

#if !USE_SIMPLEFOC_AUTO_CALIBRATION
    // Manual calibration: Suppress SimpleFOC's "MOT:" messages (we already calculated values)
    Print* saved_monitor = motor.monitor_port;
    motor.monitor_port = nullptr;
#else
    // Auto calibration: Show SimpleFOC's messages (to see what it calculates)
    if (DEBUG_MOTOR) {
        Serial.println("");  // New line for SimpleFOC messages
    }
#endif

    int foc_result = motor.initFOC();

#if !USE_SIMPLEFOC_AUTO_CALIBRATION
    // Restore monitoring
    motor.monitor_port = saved_monitor;
#endif

    // SMARTKNOB PATTERN: Trust SimpleFOC's initialization completely
    // Do NOT manually set shaft_angle - this creates mismatches with sensor state
    // (full_rotations, angle_prev) that cause velocity calculation errors.
    //
    // Instead, run stabilization cycles to let SimpleFOC synchronize naturally.
    // This matches the proven SmartKnob implementation pattern.
    if (foc_result == 1) {
        if (DEBUG_MOTOR) {
#if USE_SIMPLEFOC_AUTO_CALIBRATION
            // Show what SimpleFOC calculated
            Serial.print(" ✓ | Dir:");
            Serial.print(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
            Serial.print(" ZeroAngle:");
            Serial.print(radiansToDegrees(motor.zero_electric_angle), 1);
            Serial.print("°");
#else
            Serial.print(" ✓");
#endif
        }

        // Run stabilization cycles to let sensor state settle
        // This ensures full_rotations and angle_prev are synchronized with shaft_angle
        for (int i = 0; i < 20; i++) {
            motor.loopFOC();
            delay(1);
        }

        // VALIDATION: Verify shaft_angle matches encoder
        float encoder_angle = encoder.getSensorAngle();
        float encoder_deg = radiansToDegrees(encoder_angle);
        float shaft_deg = radiansToDegrees(motor.shaft_angle);
        float error = abs(shaft_deg - encoder_deg);

        // Handle wraparound (e.g., 355° vs 5° should be 10° error, not 350°)
        if (error > 180.0f) {
            error = 360.0f - error;
        }

        if (DEBUG_MOTOR) {
            Serial.print(" | Enc:");
            Serial.print(encoder_deg, 1);
            Serial.print("° FOC:");
            Serial.print(shaft_deg, 1);
            Serial.print("° Err:");
            Serial.print(error, 1);
            Serial.print("°");
            if (error > 5.0f) {
                Serial.print(" ⚠");
            } else {
                Serial.print(" ✓");
            }
        }
    }

    // Disable motor after calibration
    motor.disable();

    if (DEBUG_MOTOR) {
        if (foc_result == 1) {
            Serial.println("\n✓ Calibration complete");
        } else {
            Serial.println("\n✗ initFOC() failed - check hardware");
        }
    }

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

    if (DEBUG_MOTOR) {
        Serial.print("✓ Motor enabled at ");
        Serial.print(radiansToDegrees(motor.shaft_angle), 1);
        Serial.println("°");
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
    // Ensure motor is calibrated
    if (!motor_calibrated) {
        if (verbose) {
            Serial.println("[TUNE] ERROR: Motor must be calibrated before PID tuning");
            Serial.println("       Run 'c' or 'calibrate' first.");
        }
        return false;
    }

    // Ensure motor is enabled
    if (!motor_enabled) {
        if (verbose) {
            Serial.println("[TUNE] Enabling motor for tuning...");
        }
        enable();
        delay(500);
    }

    // Run firmware-based PID tuning
    PIDAutoTuner tuner(motor, encoder);
    bool success = tuner.runTuning(verbose);

    if (success) {
        // Get optimal parameters
        float p, i, d, ramp;
        tuner.getOptimalPID(p, i, d, ramp);

        // Apply to motor
        setPositionPID(p, i, d, ramp);

        // Save to NVS for persistence
        if (verbose) {
            Serial.println("\nSaving tuned parameters to NVS...");
        }
        if (saveToNVS()) {
            if (verbose) {
                Serial.println("Parameters saved! Will be loaded on next boot.");
            }
        } else {
            if (verbose) {
                Serial.println("Warning: Failed to save to NVS");
            }
        }
    }

    return success;
}

//=============================================================================
// NVS STORAGE METHODS
//=============================================================================

bool MotorController::saveToNVS() {
    CalibrationData data;

    // Calibration data
    data.zero_electric_angle = motor.zero_electric_angle;
    data.sensor_direction = (motor.sensor_direction == Direction::CW) ? 1 : -1;

    // Position PID
    data.pos_p = motor.P_angle.P;
    data.pos_i = motor.P_angle.I;
    data.pos_d = motor.P_angle.D;
    data.pos_ramp = radiansToDegrees(motor.P_angle.output_ramp);

    // Velocity PID
    data.vel_p = motor.PID_velocity.P;
    data.vel_i = motor.PID_velocity.I;
    data.vel_d = motor.PID_velocity.D;
    data.vel_ramp = motor.PID_velocity.output_ramp;

    return nvsStorage.save(data);
}

bool MotorController::loadFromNVS() {
    CalibrationData data;

    if (!nvsStorage.load(data)) {
        return false;
    }

    // Apply calibration data
    motor.zero_electric_angle = data.zero_electric_angle;
    motor.sensor_direction = (data.sensor_direction == 1) ? Direction::CW : Direction::CCW;

    // Apply position PID
    setPositionPID(data.pos_p, data.pos_i, data.pos_d, data.pos_ramp);

    // Apply velocity PID
    setVelocityPID(data.vel_p, data.vel_i, data.vel_d, data.vel_ramp);

    if (DEBUG_MOTOR) {
        Serial.println("[NVS] Loaded calibration and PID from storage");
        Serial.print("  ZeroAngle: ");
        Serial.print(radiansToDegrees(data.zero_electric_angle), 1);
        Serial.print("deg, Dir: ");
        Serial.println(data.sensor_direction == 1 ? "CW" : "CCW");
        Serial.print("  PosPID: P=");
        Serial.print(data.pos_p, 2);
        Serial.print(" I=");
        Serial.print(data.pos_i, 3);
        Serial.print(" D=");
        Serial.println(data.pos_d, 4);
    }

    return true;
}

bool MotorController::hasNVSData() {
    return nvsStorage.hasValidData();
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

    if (DEBUG_MOTOR && at_target) {
        static unsigned long last_debug = 0;
        if (millis() - last_debug > 1000) {  // Debug once per second
            Serial.print("[AT_TARGET] Current: ");
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

    // SIMPLEFOC: Run FOC algorithm (current control)
    // NOTE: SimpleFOC automatically calls sensor->update() inside loopFOC()
    // Do NOT call encoder.update() manually here - it breaks the control loop!
    motor.loopFOC();

    // NOTE: No angle normalization needed here!
    // FORCE_SENSOR_DIRECTION_CW=true in config.h forces calibration to use CW direction
    // This ensures shaft_angle is always positive (0-2π range) even if physical sensor is CCW
    // Normalizing here would create discontinuities and break velocity estimation

    // DIAGNOSTIC: Verbose shaft_angle logging disabled - too noisy during tests
    // Enable this manually when debugging shaft_angle calculation issues
    // if (DEBUG_MOTOR) {
    //     static unsigned long last_debug = 0;
    //     if (millis() - last_debug > 500) {  // Log every 500ms
    //         Serial.print("[DIAG_shaft_angle] shaft_angle: ");
    //         Serial.print(radiansToDegrees(motor.shaft_angle), 2);
    //         Serial.print("° | sensor_direction: ");
    //         Serial.print(motor.sensor_direction == Direction::CW ? "CW(+1)" : "CCW(-1)");
    //         Serial.print(" | sensor_offset: ");
    //         Serial.print(radiansToDegrees(motor.sensor_offset), 2);
    //         Serial.println("°");
    //         last_debug = millis();
    //     }
    // }

    // SIMPLEFOC: Run motion control (position control)
    // CRITICAL: SimpleFOC's position PID doesn't normalize angle errors for absolute encoders!
    // We must normalize the target to be within ±π of current position (shortest path)
    // Otherwise SimpleFOC can calculate the wrong direction for wraparound cases
    float current_rad = motor.shaft_angle;
    float target_rad = degreesToRadians(target_position_deg);

    // Normalize error to [-π, +π] (shortest path)
    float error_rad = target_rad - current_rad;
    while (error_rad > PI) error_rad -= TWO_PI;
    while (error_rad < -PI) error_rad += TWO_PI;

    // Calculate normalized target that's closest to current position
    float normalized_target_rad = current_rad + error_rad;

    motor.move(normalized_target_rad);

    // NOTE: AT_TARGET debug removed from here - it was using raw shaft_angle
    // which caused confusing duplicate logs with non-normalized angles.
    // The isAtTarget() function already has proper normalized debug output.
}
