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
    // CORRECT SIMPLEFOC PATTERN: Read sensor and return angle in radians
    // The base class Sensor::update() will call this method
    // No need to override update() - let base class handle tracking

    // Read raw encoder count (MINIMAL ABSTRACTION - closest to hardware)
    cached_raw_count = encoder.readRawAngle();

    // Convert to degrees (our preferred unit for diagnostics)
    cached_degrees = rawToDegrees(cached_raw_count);

    // Convert to radians for SimpleFOC
    cached_radians = degreesToRadians(cached_degrees);

    // Update timestamp
    last_update_time = micros();

    // DEBUG: Throttled update logging (every 1 second max)
    if (DEBUG_MOTOR) {
        static unsigned long last_debug_print = 0;
        if (millis() - last_debug_print > 1000) {
            Serial.print("[SENSOR] getSensorAngle() - Raw: ");
            Serial.print(cached_raw_count);
            Serial.print(", Deg: ");
            Serial.print(cached_degrees, 2);
            Serial.print("°, Rad: ");
            Serial.print(cached_radians, 4);
            Serial.println(" rad");
            last_debug_print = millis();
        }
    }

    // Return angle in radians (SimpleFOC expects this)
    // Base class Sensor::update() will handle:
    // - Rotation tracking (angle_prev, full_rotations)
    // - Wraparound detection
    // - Timestamp management
    return cached_radians;
}

float MT6701Sensor::getAngle() {
    // DIAGNOSTIC: Log what we're returning and what SimpleFOC will do with it
    float angle_to_return = angle_prev;

    if (DEBUG_MOTOR) {
        static unsigned long last_debug = 0;
        if (millis() - last_debug > 500) {  // Log every 500ms
            Serial.print("[DIAG_getAngle] returning: ");
            Serial.print(radiansToDegrees(angle_to_return), 2);
            Serial.print("° | angle_prev: ");
            Serial.print(radiansToDegrees(angle_prev), 2);
            Serial.print("° | full_rotations: ");
            Serial.println(full_rotations);
            last_debug = millis();
        }
    }

    return angle_to_return;  // Return 0-2π only, ignore full_rotations
}

// DO NOT override update() - follow standard SimpleFOC pattern
// Let base class Sensor::update() call getSensorAngle() and handle tracking
// This matches all official SimpleFOC drivers and third-party implementations

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
    if (DEBUG_MOTOR) {
        Serial.println("Starting motor calibration...");
    }

    // Run SimpleFOC calibration
    bool success = runCalibration();

    if (success) {
        motor_calibrated = true;

        if (DEBUG_MOTOR) {
            Serial.println("Calibration successful");
        }
    } else {
        motor_calibrated = false;

        if (DEBUG_MOTOR) {
            Serial.println("Calibration failed");
        }
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
        Serial.println("");
        Serial.println("╔════════════════════════════════════════════════════════════════╗");
        Serial.println("║                    Motor Calibration                           ║");
        Serial.println("╚════════════════════════════════════════════════════════════════╝");
        Serial.println("");
        Serial.println("This will test motor movement and calculate calibration values.");
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

    if (DEBUG_MOTOR) {
        Serial.print("[OK] Encoder reading: ");
        Serial.print(radiansToDegrees(test_angle), 2);
        Serial.println("°");
        Serial.println("");
    }

    // Enable motor
    if (DEBUG_MOTOR) {
        Serial.println("Step 1: Diagnostic Test");
        Serial.println("─────────────────────────────────────────────────────────────────");
        Serial.println("Testing motor at 4 electrical angles...");
        Serial.println("");
    }
    motor.enable();

    // Increase voltage temporarily for better alignment
    float normal_voltage_limit = motor.voltage_limit;
    motor.voltage_limit = 10.0f;

    // Diagnostic test at 4 positions
    float test_angles[] = {0, _PI_2, PI, _3PI_2};
    const char* angle_names[] = {"0°", "90°", "180°", "270°"};
    float positions[4];

    for (int i = 0; i < 4; i++) {
        motor.setPhaseVoltage(6.0, 0, test_angles[i]);
        delay(700);

        encoder.update();
        positions[i] = encoder.getSensorAngle();

        if (DEBUG_MOTOR) {
            Serial.print("[TEST] ");
            Serial.print(angle_names[i]);
            Serial.print(" electrical → ");
            Serial.print(radiansToDegrees(positions[i]), 2);
            Serial.println("° mechanical");
        }
    }

    if (DEBUG_MOTOR) {
        Serial.println("");
        Serial.println("✓ Motor responding to all positions");
        Serial.println("");
    }

    // Use 90° and 270° for calibration (they have best separation)
    if (DEBUG_MOTOR) {
        Serial.println("Step 2: Calculate Calibration Values");
        Serial.println("─────────────────────────────────────────────────────────────────");
        Serial.println("Using 90° and 270° electrical for calibration...");
        Serial.println("");
    }

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
            Serial.println("[ERROR] Insufficient movement between calibration points!");
            Serial.print("[ERROR] Only ");
            Serial.print(radiansToDegrees(angle_diff), 2);
            Serial.println("° difference");
            Serial.println("[ERROR] This usually means:");
            Serial.println("  - Motor driver not powered");
            Serial.println("  - Driver in fault state");
            Serial.println("  - Enable pin not working");
            Serial.println("  - Phase wires disconnected");
        }
        motor.disable();
        motor.voltage_limit = normal_voltage_limit;
        return false;
    }

    if (DEBUG_MOTOR) {
        Serial.print("[OK] Motor moved ");
        Serial.print(radiansToDegrees(angle_diff), 2);
        Serial.println("° between calibration points");
        Serial.println("");
    }

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

    if (DEBUG_MOTOR) {
        Serial.print("[OK] Sensor direction: ");
        Serial.println(sensor_dir == Direction::CW ? "CW" : "CCW");
        Serial.println("");
    }

    // Calculate zero_electric_angle
    // We know motor is at 90° electrical, encoder reads angle_at_pi2
    // zero_electric_angle is the offset between mechanical and electrical coordinates
    float electrical_from_encoder = angle_at_pi2 * POLE_PAIRS;
    electrical_from_encoder = normalizeRadians(electrical_from_encoder);

    float zero_elec_angle = normalizeRadians(electrical_from_encoder - _PI_2);

    if (DEBUG_MOTOR) {
        Serial.print("[OK] zero_electric_angle = ");
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
        Serial.println("Step 3: Initialize FOC");
        Serial.println("─────────────────────────────────────────────────────────────────");
    }

    int foc_result = motor.initFOC();

    // CRITICAL FIX: When using absolute encoders with pre-set zero_electric_angle,
    // SimpleFOC skips alignment and doesn't initialize shaft_angle from the sensor.
    // We must manually sync shaft_angle to the current sensor reading.
    // This is a known SimpleFOC issue with absolute encoders.
    // See: https://community.simplefoc.com/t/sensor-getangle-works-correctly-but-motor-shaft-angle-is-always-zero-mot-init-foc-failed-as-a-result/5357
    if (foc_result == 1) {
        encoder.update();  // Update sensor
        motor.shaft_angle = encoder.getSensorAngle();  // Force sync!

        if (DEBUG_MOTOR) {
            Serial.println("[FIX] Initialized shaft_angle from sensor");
            Serial.print("  shaft_angle = ");
            Serial.print(radiansToDegrees(motor.shaft_angle), 2);
            Serial.println("°");
        }
    }

    // Disable motor after calibration
    motor.disable();

    if (DEBUG_MOTOR) {
        if (foc_result == 1) {
            Serial.println("");
            Serial.println("╔════════════════════════════════════════════════════════════════╗");
            Serial.println("║                 ✓ CALIBRATION SUCCESSFUL                       ║");
            Serial.println("╚════════════════════════════════════════════════════════════════╝");
            Serial.println("");
            Serial.println("Calibration values:");
            Serial.print("  • zero_electric_angle = ");
            Serial.print(motor.zero_electric_angle, 4);
            Serial.print(" rad (");
            Serial.print(radiansToDegrees(motor.zero_electric_angle), 2);
            Serial.println("°)");
            Serial.print("  • sensor_direction = ");
            Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
            Serial.println("");
            Serial.println("Next steps:");
            Serial.println("  1. Type 'e' to enable motor");
            Serial.println("  2. Type 'status' to verify shaft_angle tracks encoder");
            Serial.println("  3. Type 'm 90' to test position control");
            Serial.println("");
        } else {
            Serial.println("");
            Serial.println("╔════════════════════════════════════════════════════════════════╗");
            Serial.println("║                  ✗ CALIBRATION FAILED                          ║");
            Serial.println("╚════════════════════════════════════════════════════════════════╝");
            Serial.println("");
            Serial.println("initFOC() failed even with manual calibration");
            Serial.println("Check hardware connections and driver status");
            Serial.println("");
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
        Serial.println("Motor enabled");
        Serial.print("Current position: ");
        Serial.print(radiansToDegrees(motor.shaft_angle), 2);
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

    // Store target position (absolute coordinates 0-360°)
    target_position_deg = position_deg;

    if (DEBUG_MOTOR) {
        Serial.print("Moving to absolute position: ");
        Serial.print(position_deg, 2);
        Serial.println("°");
    }
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

    if (verbose) {
        Serial.println("[TUNE] Starting PID auto-tuning in position control mode...");
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

    return success;
}

float MotorController::getPosition() {
    // Return current position from SimpleFOC (absolute 0-360°)
    // SimpleFOC's shaft_angle is kept synchronized by loopFOC()
    return radiansToDegrees(motor.shaft_angle);
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

    // Convert to degrees for comparison
    float current_position_deg = radiansToDegrees(current_position_rad);
    float velocity_deg_s = abs(radiansToDegrees(current_velocity_rad_s));

    // Calculate position error (both in absolute coordinates)
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

    // CRITICAL FIX: Normalize shaft_angle to 0-2π range
    // When sensor_direction = CCW, SimpleFOC produces negative shaft_angle values
    // This breaks position error calculations (error = target - current becomes huge)
    // and velocity estimation (sees massive jumps across wraparound)
    // Normalizing to 0-2π fixes both issues while preserving motor behavior
    float normalized_angle = motor.shaft_angle;
    while (normalized_angle < 0) {
        normalized_angle += _2PI;
    }
    while (normalized_angle >= _2PI) {
        normalized_angle -= _2PI;
    }
    motor.shaft_angle = normalized_angle;

    // DIAGNOSTIC: Log SimpleFOC's shaft_angle calculation
    if (DEBUG_MOTOR) {
        static unsigned long last_debug = 0;
        if (millis() - last_debug > 500) {  // Log every 500ms
            Serial.print("[DIAG_shaft_angle] shaft_angle: ");
            Serial.print(radiansToDegrees(motor.shaft_angle), 2);
            Serial.print("° | sensor_direction: ");
            Serial.print(motor.sensor_direction == Direction::CW ? "CW(+1)" : "CCW(-1)");
            Serial.print(" | sensor_offset: ");
            Serial.print(radiansToDegrees(motor.sensor_offset), 2);
            Serial.println("°");
            last_debug = millis();
        }
    }

    // SIMPLEFOC: Run motion control (position control)
    // Convert target from degrees to radians for SimpleFOC
    // Both target and shaft_angle are in absolute coordinates (0-360° / 0-2π rad)
    motor.move(degreesToRadians(target_position_deg));

    // Optional: Log when target is reached (for debugging)
    if (DEBUG_MOTOR && isAtTarget()) {
        static unsigned long last_reached_log = 0;
        if (millis() - last_reached_log > 2000) {  // Log every 2 seconds when at target
            Serial.print("[AT_TARGET] Current: ");
            Serial.print(radiansToDegrees(motor.shaft_angle), 2);
            Serial.print("°, Target: ");
            Serial.print(target_position_deg, 2);
            Serial.print("°, Error: ");
            Serial.print(abs(radiansToDegrees(motor.shaft_angle) - target_position_deg), 2);
            Serial.print("°, Vel: ");
            Serial.print(radiansToDegrees(motor.shaft_velocity), 2);
            Serial.println("°/s");
            last_reached_log = millis();
        }
    }
}
