#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"

// =============================================================================
// MT6701 SENSOR (SimpleFOC-compatible wrapper)
// =============================================================================
// SimpleFOC's Sensor base class handles:
//   - Rotation tracking (full_rotations counter)
//   - Velocity calculation
//   - Continuous angle via getAngle()
// We just implement getSensorAngle() to return raw 0-2π position.
// =============================================================================

class MT6701Sensor : public Sensor {
public:
    MT6701Sensor(uint8_t addr = 0x06) : i2c_addr(addr) {}

    void init() override {
        Wire.begin(ENCODER_SDA, ENCODER_SCL);
        Wire.setClock(400000);
    }

    // Return raw mechanical angle 0-2π (SimpleFOC handles rotation tracking)
    float getSensorAngle() override {
        Wire.beginTransmission(i2c_addr);
        Wire.write(0x03);
        Wire.endTransmission(false);
        Wire.requestFrom(i2c_addr, (uint8_t)2);

        if (Wire.available() < 2) return -1;

        uint16_t raw = (Wire.read() << 6) | (Wire.read() >> 2);
        return (float)raw / 16384.0f * _2PI;
    }

    // Direct access for diagnostics
    float getDegrees() {
        float rad = getSensorAngle();
        return rad * 180.0f / PI;
    }

    uint16_t getRawCount() {
        Wire.beginTransmission(i2c_addr);
        Wire.write(0x03);
        Wire.endTransmission(false);
        Wire.requestFrom(i2c_addr, (uint8_t)2);
        if (Wire.available() < 2) return 0;
        return (Wire.read() << 6) | (Wire.read() >> 2);
    }

private:
    uint8_t i2c_addr;
};

// =============================================================================
// MOTOR CONTROLLER
// =============================================================================
// Simple wrapper around SimpleFOC. Key principle:
//   - SimpleFOC manages shaft_angle (continuous, can be >360° or <0°)
//   - We work in degrees 0-360 for user interface
//   - Target calculation uses shortest-path logic
//   - We NEVER override motor.shaft_angle
// =============================================================================

class MotorController {
public:
    MotorController()
        : motor(POLE_PAIRS),
          driver(MOTOR_PWM_A, MOTOR_PWM_B, MOTOR_PWM_C, MOTOR_ENABLE),
          target_deg(0),
          enabled(false),
          calibrated(false) {}

    void begin() {
        Serial.println("[MOTOR] Initializing...");

        // Initialize encoder
        encoder.init();
        motor.linkSensor(&encoder);

        // Initialize driver
        driver.voltage_power_supply = VOLTAGE_PSU;
        driver.init();
        motor.linkDriver(&driver);

        // Motor config
        motor.voltage_limit = VOLTAGE_LIMIT;
        motor.current_limit = CURRENT_LIMIT;
        motor.velocity_limit = MAX_VELOCITY_RAD;
        motor.controller = MotionControlType::angle;
        motor.torque_controller = TorqueControlType::voltage;
        motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

        // Position PID
        motor.P_angle.P = PID_P_POSITION;
        motor.P_angle.I = PID_I_POSITION;
        motor.P_angle.D = PID_D_POSITION;
        motor.P_angle.limit = MAX_VELOCITY_RAD;

        // Velocity PID
        motor.PID_velocity.P = PID_P_VELOCITY;
        motor.PID_velocity.I = PID_I_VELOCITY;
        motor.PID_velocity.D = PID_D_VELOCITY;
        motor.LPF_velocity.Tf = PID_LPF_VELOCITY;

        // Calibration voltage
        motor.voltage_sensor_align = 6.0f;

        motor.init();
        motor.disable();

        Serial.println("[MOTOR] Ready (not calibrated)");
    }

    bool calibrate() {
        Serial.println("[CAL] Starting calibration...");

        // Let SimpleFOC do full calibration
        motor.sensor_direction = Direction::UNKNOWN;
        motor.zero_electric_angle = NOT_SET;

        int result = motor.initFOC();

        if (result == 1) {
            calibrated = true;
            // Reset rotation tracking so shaft_angle starts near encoder position
            encoder.update();
            Serial.printf("[CAL] OK - Dir:%s Zero:%.1f°\n",
                motor.sensor_direction == Direction::CW ? "CW" : "CCW",
                motor.zero_electric_angle * 180.0f / PI);
            return true;
        }

        Serial.println("[CAL] FAILED");
        return false;
    }

    void enable() {
        if (!calibrated) {
            Serial.println("[MOTOR] Calibrate first!");
            return;
        }
        motor.enable();
        enabled = true;
        // Set target to current position
        target_deg = getPositionDeg();
        Serial.printf("[MOTOR] Enabled at %.1f°\n", target_deg);
    }

    void disable() {
        motor.disable();
        enabled = false;
        Serial.println("[MOTOR] Disabled");
    }

    // Move to absolute position (0-360°)
    // Uses shortest-path: calculates target relative to current SimpleFOC position
    void moveTo(float deg) {
        // Normalize input to 0-360
        deg = fmod(deg, 360.0f);
        if (deg < 0) deg += 360.0f;
        target_deg = deg;

        // Get current position from SimpleFOC (continuous, may be >360 or <0)
        float current_rad = motor.shaft_angle;
        float current_deg = current_rad * 180.0f / PI;

        // Current position normalized to 0-360
        float current_wrapped = fmod(current_deg, 360.0f);
        if (current_wrapped < 0) current_wrapped += 360.0f;

        // Calculate error using shortest path
        float error = target_deg - current_wrapped;
        if (error > 180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        // Target in SimpleFOC's continuous space
        float target_rad = current_rad + (error * PI / 180.0f);

        if (DEBUG_MOTOR) {
            Serial.printf("[MOVE] %.1f° → %.1f° (err:%.1f° target_rad:%.2f)\n",
                current_wrapped, target_deg, error, target_rad);
        }

        motor.move(target_rad);
    }

    // Call every loop iteration
    void update() {
        if (!calibrated) return;
        motor.loopFOC();

        if (enabled) {
            // Recalculate target each loop (handles any drift)
            float current_rad = motor.shaft_angle;
            float current_deg = fmod(current_rad * 180.0f / PI, 360.0f);
            if (current_deg < 0) current_deg += 360.0f;

            float error = target_deg - current_deg;
            if (error > 180.0f) error -= 360.0f;
            if (error < -180.0f) error += 360.0f;

            float target_rad = current_rad + (error * PI / 180.0f);
            motor.move(target_rad);
        }
    }

    // Position in degrees (0-360, from encoder)
    float getPositionDeg() {
        return encoder.getDegrees();
    }

    // SimpleFOC's continuous angle (can be >360 or <0)
    float getShaftAngleDeg() {
        return motor.shaft_angle * 180.0f / PI;
    }

    float getVelocityDegS() {
        return motor.shaft_velocity * 180.0f / PI;
    }

    bool isAtTarget() {
        float pos = getPositionDeg();
        float err = abs(pos - target_deg);
        if (err > 180.0f) err = 360.0f - err;
        float vel = abs(getVelocityDegS());
        return (err < POSITION_TOLERANCE_DEG) && (vel < VELOCITY_THRESHOLD_DEG);
    }

    bool isEnabled() { return enabled; }
    bool isCalibrated() { return calibrated; }
    float getTargetDeg() { return target_deg; }

    // Direct access for diagnostics
    BLDCMotor& getMotor() { return motor; }
    MT6701Sensor& getEncoder() { return encoder; }

    // Reset SimpleFOC's rotation tracking (use after diag tests)
    void resetRotationTracking() {
        encoder.update();  // Get fresh reading
        // SimpleFOC's Sensor base class tracks full_rotations internally
        // We can't directly reset it, but we can re-init
        Serial.println("[MOTOR] Rotation tracking note: re-calibrate if needed");
    }

private:
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    MT6701Sensor encoder;

    float target_deg;
    bool enabled;
    bool calibrated;
};

#endif // MOTOR_CONTROL_H
