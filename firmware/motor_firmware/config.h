#ifndef CONFIG_H
#define CONFIG_H

//=============================================================================
// HARDWARE CONFIGURATION FOR ESP32-S3-Touch-LCD-2 (Waveshare)
//=============================================================================

//=============================================================================
// MOTOR DRIVER PINS (SimpleFOC Mini v1 - DRV8313)
//=============================================================================
// SimpleFOC Mini cluster - Middle Right header positions 7-13
// Wiring: Connect ESP32 to SimpleFOC Mini as follows
#define MOTOR_ENABLE     15   // Motor driver enable pin (GPIO15 → EN)
#define MOTOR_PWM_A      13   // Phase 1 PWM output (GPIO13 → IN1)
#define MOTOR_PWM_B      11   // Phase 2 PWM output (GPIO11 → IN2)
#define MOTOR_PWM_C      12   // Phase 3 PWM output (GPIO12 → IN3)

// Optional motor driver monitoring/control pins
#define MOTOR_FAULT      14   // Fault detection (GPIO14 → nFT, active LOW)
#define MOTOR_RESET      9    // Driver reset (GPIO9 → nRT, active LOW)

//=============================================================================
// ENCODER PINS (MT6701 14-bit Magnetic Encoder via I2C)
//=============================================================================
// MT6701 encoder cluster - Top Right header positions 1-6
// Wiring: 3V3 → VDD, GND → GND, GPIO47 → SDA, GPIO48 → SCL
#define ENCODER_SDA      47   // MT6701 I2C SDA (GPIO47)
#define ENCODER_SCL      48   // MT6701 I2C SCL (GPIO48)
#define ENCODER_I2C_ADDR 0x06 // MT6701 I2C address (angle data at register 0x03-0x04)

//=============================================================================
// MOTOR PARAMETERS
//=============================================================================
#define POLE_PAIRS       7             // Mitoot 2804 motor: 7 pole pairs
#define ENCODER_PPR      16384          // MT6701 14-bit encoder: 2^14 = 16384 positions/revolution
#define VOLTAGE_PSU      12.0           // Power supply voltage (V) - adjust to your supply (8-35V range)
#define CURRENT_LIMIT    2.0            // Maximum current limit (A) - SimpleFOC Mini max 2A continuous

// Motor characteristics (Mitoot 2804 100kv Gimbal Motor)
#define MOTOR_RESISTANCE 10.0           // Phase resistance (Ohms) - gimbal motors typically R>10Ω
#define MOTOR_KV         100            // Motor KV rating (RPM/V)

//=============================================================================
// MOTION CONTROL PARAMETERS
//=============================================================================
// Velocity limits (in rad/s)
#define MAX_VELOCITY     100.0          // Maximum velocity (rad/s) ~955 RPM
#define DEFAULT_VELOCITY 20.0           // Default velocity (rad/s) ~191 RPM
#define MIN_VELOCITY     0.1            // Minimum velocity (rad/s)

// Acceleration limits (in rad/s²)
#define MAX_ACCELERATION 50.0           // Maximum acceleration
#define DEFAULT_ACCELERATION 10.0       // Default acceleration

// Position control PID parameters
#define PID_P_POSITION   20.0           // Proportional gain for position control
#define PID_I_POSITION   0.0            // Integral gain for position control
#define PID_D_POSITION   0.1            // Derivative gain for position control
#define PID_RAMP_POSITION 1000.0        // Output ramp for position control

// Velocity control PID parameters
#define PID_P_VELOCITY   0.5            // Proportional gain for velocity control
#define PID_I_VELOCITY   10.0           // Integral gain for velocity control
#define PID_D_VELOCITY   0.0            // Derivative gain for velocity control
#define PID_RAMP_VELOCITY 1000.0        // Output ramp for velocity control

// Current control PID parameters (for FOC)
#define PID_P_CURRENT    5.0            // Proportional gain for current control
#define PID_I_CURRENT    100.0          // Integral gain for current control
#define PID_D_CURRENT    0.0            // Derivative gain for current control
#define PID_RAMP_CURRENT 1000.0         // Output ramp for current control

//=============================================================================
// SYSTEM STATES
//=============================================================================
#define STATE_IDLE               0      // Motor is idle
#define STATE_MOVING             1      // Motor is moving to target
#define STATE_ERROR              2      // System in error state
#define STATE_CALIBRATING        3      // Motor calibration in progress

//=============================================================================
// CONTROL MODES
//=============================================================================
// Protocol control modes (used in commands)
#define MODE_POSITION            0      // Position control mode
#define MODE_VELOCITY            1      // Velocity control mode
#define MODE_TORQUE              2      // Torque control mode

// SimpleFOC control modes (legacy, kept for reference)
#define TORQUE_MODE      0              // Torque/current control
#define VELOCITY_MODE    1              // Velocity control
#define ANGLE_MODE       2              // Position/angle control

// Default control mode
#define DEFAULT_CONTROL_MODE MODE_POSITION

//=============================================================================
// SERIAL COMMUNICATION
//=============================================================================
#define SERIAL_BAUD      115200         // Serial communication baud rate
#define SERIAL_TIMEOUT   1000           // Serial timeout in milliseconds

//=============================================================================
// POSITION TRACKING
//=============================================================================
#define POSITION_TOLERANCE 0.1          // Position tolerance in radians (~5.7 degrees)
#define VELOCITY_THRESHOLD 0.01         // Velocity threshold to consider motor stopped

//=============================================================================
// SYSTEM TIMING
//=============================================================================
#define LOOP_FREQUENCY_HZ 1000          // Main loop frequency (Hz)
#define LOOP_PERIOD_US    (1000000 / LOOP_FREQUENCY_HZ)

// Task priorities for FreeRTOS
#define MOTOR_TASK_PRIORITY   2         // Motor control task priority
#define SERIAL_TASK_PRIORITY  1         // Serial communication task priority
#define DISPLAY_TASK_PRIORITY 1         // Display update task priority

//=============================================================================
// DISPLAY CONFIGURATION (Built-in ST7789 on Waveshare board)
//=============================================================================
// Display is pre-configured by the board's Display_ST7789.h
// These are just reference values
#define DISPLAY_WIDTH     320
#define DISPLAY_HEIGHT    240
#define DISPLAY_BACKLIGHT 50            // Default backlight level (0-100)

//=============================================================================
// SAFETY FEATURES
//=============================================================================
#define ENABLE_WATCHDOG          true   // Enable watchdog timer
#define WATCHDOG_TIMEOUT_MS      5000   // Watchdog timeout in milliseconds
#define ENABLE_CURRENT_LIMITING  true   // Enable current limiting
#define ENABLE_OVERHEAT_PROTECTION false // Enable temperature monitoring (if sensor available)
#define MAX_TEMPERATURE          80.0   // Maximum allowed temperature (°C)

//=============================================================================
// DEBUGGING
//=============================================================================
#define DEBUG_SERIAL     true           // Enable debug output on serial
#define DEBUG_MOTOR      true           // Enable detailed motor debug output
#define DEBUG_COMM       true           // Enable communication debug output
#define DEBUG_HEARTBEAT  true           // Enable heartbeat messages
#define HEARTBEAT_INTERVAL_MS 5000      // Heartbeat message interval

#endif // CONFIG_H
