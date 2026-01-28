#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// HARDWARE PINS (T-Display-S3 + Custom Motor Driver)
// =============================================================================

// Motor driver (DRV8313-based)
#define MOTOR_PWM_A     1    // IN1
#define MOTOR_PWM_B     2    // IN2
#define MOTOR_PWM_C     3    // IN3
#define MOTOR_ENABLE    4    // nSLEEP (active high)
#define MOTOR_FAULT     5    // nFAULT (active low = fault)

// MT6701 encoder (I2C)
#define ENCODER_SDA     43
#define ENCODER_SCL     44
#define ENCODER_I2C_ADDR 0x06

// =============================================================================
// MOTOR PARAMETERS
// =============================================================================

#define POLE_PAIRS      7           // Motor pole pairs
#define VOLTAGE_PSU     12.0f       // Power supply voltage
#define VOLTAGE_LIMIT   6.0f        // Max motor voltage (gimbal safe)
#define CURRENT_LIMIT   2.0f        // Max current (A)

// =============================================================================
// ENCODER
// =============================================================================

#define ENCODER_PPR     16384       // 14-bit encoder (2^14)

// =============================================================================
// PID TUNING (SimpleFOC uses radians internally)
// =============================================================================

// Position PID
#define PID_P_POSITION      15.0f
#define PID_I_POSITION      0.0f    // Start without I term
#define PID_D_POSITION      0.5f

// Velocity PID
#define PID_P_VELOCITY      0.5f
#define PID_I_VELOCITY      5.0f
#define PID_D_VELOCITY      0.0f
#define PID_LPF_VELOCITY    0.02f   // Velocity low-pass filter

// Limits
#define MAX_VELOCITY_RAD    6.28f   // ~360°/s max velocity

// =============================================================================
// CONTROL THRESHOLDS
// =============================================================================

#define POSITION_TOLERANCE_DEG  0.5f    // "At target" threshold
#define VELOCITY_THRESHOLD_DEG  1.0f    // "Stopped" threshold

// =============================================================================
// DEBUG
// =============================================================================

#define DEBUG_MOTOR     true

#endif // CONFIG_H
