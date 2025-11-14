#ifndef CONFIG_H
#define CONFIG_H

//=============================================================================
// HARDWARE CONFIGURATION FOR ESP32-S3-Touch-LCD-2 (Waveshare)
//=============================================================================

//=============================================================================
// MOTOR DRIVER PINS (3-Phase BLDC)
//=============================================================================
// Configure these pins based on your motor driver wiring
#define MOTOR_PWM_A      GPIO_NUM_10   // Phase A PWM output
#define MOTOR_PWM_B      GPIO_NUM_11   // Phase B PWM output
#define MOTOR_PWM_C      GPIO_NUM_12   // Phase C PWM output
#define MOTOR_ENABLE     GPIO_NUM_13   // Motor driver enable pin

//=============================================================================
// ENCODER PINS
//=============================================================================
#define ENCODER_A        GPIO_NUM_14   // Encoder channel A
#define ENCODER_B        GPIO_NUM_15   // Encoder channel B
#define ENCODER_I        GPIO_NUM_16   // Encoder index (optional, set to -1 if not used)

//=============================================================================
// MOTOR PARAMETERS
//=============================================================================
#define POLE_PAIRS       7             // Number of pole pairs in the BLDC motor
#define ENCODER_PPR      2048           // Encoder pulses per revolution
#define VOLTAGE_PSU      12.0           // Power supply voltage (V)
#define CURRENT_LIMIT    1.0            // Maximum current limit (A)

// Motor characteristics
#define MOTOR_RESISTANCE 2.5            // Phase resistance (Ohms) - measure with multimeter
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
// CONTROL MODES
//=============================================================================
// SimpleFOC control modes
#define TORQUE_MODE      0              // Torque/current control
#define VELOCITY_MODE    1              // Velocity control
#define ANGLE_MODE       2              // Position/angle control

// Default control mode
#define DEFAULT_CONTROL_MODE ANGLE_MODE

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
#define DEBUG_MOTOR      false          // Enable detailed motor debug output
#define DEBUG_COMM       false          // Enable communication debug output

#endif // CONFIG_H
