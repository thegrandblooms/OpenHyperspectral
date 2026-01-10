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

// GIMBAL MOTOR VOLTAGE LIMIT
// Research shows gimbal motors work best with 5-10V, preferably starting at 6V
// This prevents overshooting and cogging/detent issues
#define VOLTAGE_LIMIT_GIMBAL 6.0        // Voltage limit for gimbal motor (V) - conservative for smooth operation

// Motor characteristics (Mitoot 2804 100kv Gimbal Motor)
#define MOTOR_RESISTANCE 10.0           // Phase resistance (Ohms) - gimbal motors typically R>10Ω
#define MOTOR_KV         100            // Motor KV rating (RPM/V)

// SENSOR DIRECTION OVERRIDE
// SimpleFOC auto-detects sensor direction during calibration (CW or CCW)
// When CCW is detected, shaft_angle becomes negative, breaking position control
// Override to CW to force positive angles and fix position/velocity calculations
// Physical mounting: If encoder reads increase when motor rotates "forward", use CW
//                    If encoder reads decrease when motor rotates "forward", use CCW
// NOTE: Changing this does NOT change motor behavior, only angle interpretation
#define FORCE_SENSOR_DIRECTION_CW true  // Set to true to override CCW and force CW direction

//=============================================================================
// MOTION CONTROL PARAMETERS
//=============================================================================
// NOTE: Internal code uses DEGREES, SimpleFOC uses radians.
// We define both for clarity, but DEGREES are preferred for user-facing values.

// Velocity limits - OPTIMIZED FOR GIMBAL MOTORS
// Gimbal motors are designed for slow, precise movements, not high speed
#define MAX_VELOCITY_DEG     573.0      // Maximum velocity (deg/s) ~95 RPM
#define MAX_VELOCITY         10.0       // Maximum velocity (rad/s) - for SimpleFOC
#define DEFAULT_VELOCITY_DEG 286.5      // Default velocity (deg/s) ~48 RPM
#define DEFAULT_VELOCITY     5.0        // Default velocity (rad/s)
#define MIN_VELOCITY_DEG     5.7        // Minimum velocity (deg/s)
#define MIN_VELOCITY         0.1        // Minimum velocity (rad/s)

// Acceleration limits - OPTIMIZED FOR GIMBAL MOTORS
#define MAX_ACCELERATION_DEG 1146.0     // Maximum acceleration (deg/s²)
#define MAX_ACCELERATION     20.0       // Maximum acceleration (rad/s²) - for SimpleFOC
#define DEFAULT_ACCELERATION_DEG 286.5  // Default acceleration (deg/s²)
#define DEFAULT_ACCELERATION 5.0        // Default acceleration (rad/s²)

// Position control PID parameters - TUNED FOR POSITION CONTROL
// CRITICAL FIX: Previous P=1.0 was based on incorrect gimbal motor guidance
// SimpleFOC documentation and smart knob projects use P=15-25 for position control
// Even gimbal motors need sufficient P gain to overcome static friction!
// Start with SimpleFOC default P=20, tune down if oscillations occur
#define PID_P_POSITION   20.0           // Proportional gain - FIXED from 1.0
#define PID_I_POSITION   0.0            // Integral gain (usually not needed for position)
#define PID_D_POSITION   0.0            // Derivative gain (can cause noise/vibration)
#define PID_RAMP_POSITION_DEG 1000.0    // Output ramp (deg/s)
#define PID_RAMP_POSITION 100.0         // Output ramp (rad/s) - for SimpleFOC

// Velocity control PID parameters - Inner loop (more critical for stability)
// CRITICAL: These values are based on SimpleFOC gimbal motor recommendations
// Research shows gimbal motors typically use: P=0.2, I=20, output_ramp=1000, voltage_limit=6V
#define PID_P_VELOCITY   0.2            // Standard for gimbal motors (was 0.1)
#define PID_I_VELOCITY   20.0           // Standard for gimbal motors (was 10.0)
#define PID_D_VELOCITY   0.0            // Usually 0 for gimbal motors
#define PID_RAMP_VELOCITY 1000.0        // Higher ramp for smoother control (was 100.0)
#define PID_LPF_VELOCITY 0.01           // Low-pass filter time constant (10ms, less aggressive)

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
// CRITICAL FIX: Previous tolerance of 5.7° was WAY too large!
// Motor would think it reached target without actually moving
// SimpleFOC examples and smart knob projects use ~0.1°-0.5°
#define POSITION_TOLERANCE_DEG 0.5      // Position tolerance (degrees) - FIXED from 5.7°
#define POSITION_TOLERANCE 0.009        // Position tolerance (radians) ~0.5° - for SimpleFOC
#define VELOCITY_THRESHOLD_DEG 0.57     // Velocity threshold (deg/s)
#define VELOCITY_THRESHOLD 0.01         // Velocity threshold (rad/s) - for SimpleFOC

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
// NOTE: Debug flags can be toggled at runtime using the "debug" command
// These are just the default startup values
#define DEBUG_SERIAL_DEFAULT     true           // Enable debug output on serial (default)
#define DEBUG_MOTOR_DEFAULT      true           // Enable detailed motor debug output (default)
#define DEBUG_COMM_DEFAULT       true           // Enable communication debug output (default)
#define DEBUG_HEARTBEAT_DEFAULT  true           // Enable heartbeat messages (default)
#define HEARTBEAT_INTERVAL_MS    10000          // Heartbeat message interval (10 seconds)

// Legacy defines for backward compatibility during transition
#define DEBUG_SERIAL     DEBUG_SERIAL_DEFAULT
#define DEBUG_MOTOR      DEBUG_MOTOR_DEFAULT
#define DEBUG_COMM       DEBUG_COMM_DEFAULT
#define DEBUG_HEARTBEAT  DEBUG_HEARTBEAT_DEFAULT

#endif // CONFIG_H
