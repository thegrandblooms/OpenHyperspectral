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
// Phase resistance measured from SimpleFOC community thread for GBM2804-100T:
//   community.simplefoc.com/t/simplefocmini-gbm2804h-how-much-voltage/7892
// At 6V limit: ~0.95A peak phase current (6V / 6.3Ω ≈ 0.95A)
#define MOTOR_RESISTANCE 6.3            // Phase resistance (Ohms) - measured GBM2804-100T
#define MOTOR_KV         100            // Motor KV rating (RPM/V)

// SENSOR DIRECTION OVERRIDE
// SimpleFOC auto-detects sensor direction during calibration (CW or CCW)
//
// Why this is safe:
// - The zero_electric_angle calculation (motor_control.cpp:872) accounts for direction
// - This only affects SimpleFOC's angle interpretation, not motor physics
//
// Physical relationship can be CCW, but we override to CW for correct angle math.
// SimpleFOC expects shaft_angle in 0-2π range; CCW creates negative angles.
#define FORCE_SENSOR_DIRECTION_CW false  // DISABLED: Let SimpleFOC auto-detect

// Calibration method selection
// false = Use manual calibration (calculates zero_electric_angle ourselves)
// true  = Let SimpleFOC auto-calibrate (runs alignment and calculates offset)
#define USE_SIMPLEFOC_AUTO_CALIBRATION true  // ENABLED: Use SimpleFOC's proven calibration

// CalibratedSensor: encoder nonlinearity correction via LUT (Arduino-FOC-drivers v1.0.9+)
// Wraps MT6701Sensor with a forward+reverse open-loop scan that builds a per-position
// correction table, cancelling the MT6701's ±1.0° typical INL.
// Expected improvement: ±0.5–1.0° → ±0.1–0.2° (addresses sensor INL, not PID error)
// Note: adds ~10s to calibration time at every boot (no flash persistence yet)
#define USE_CALIBRATED_SENSOR false

//=============================================================================
// MOTION CONTROL PARAMETERS
//=============================================================================
// NOTE: Internal code uses DEGREES, SimpleFOC uses radians.
// We define both for clarity, but DEGREES are preferred for user-facing values.

// Velocity limits
// Gimbal motors are designed for slow, precise movements, not high speed
// Previous 573°/s (10 rad/s) was too aggressive - caused open-loop test failures
#define MAX_VELOCITY_DEG     180.0      // Maximum velocity (deg/s) ~30 RPM
#define MAX_VELOCITY         3.14       // Maximum velocity (rad/s) - for SimpleFOC
#define DEFAULT_VELOCITY_DEG 90.0       // Default velocity (deg/s) ~15 RPM
#define DEFAULT_VELOCITY     1.57       // Default velocity (rad/s)
#define MIN_VELOCITY_DEG     5.0        // Minimum velocity (deg/s)
#define MIN_VELOCITY         0.05        // Minimum velocity (rad/s)

// Acceleration limits - OPTIMIZED FOR GIMBAL MOTORS
#define MAX_ACCELERATION_DEG 1146.0     // Maximum acceleration (deg/s²)
#define MAX_ACCELERATION     20.0       // Maximum acceleration (rad/s²) - for SimpleFOC
#define DEFAULT_ACCELERATION_DEG 286.5  // Default acceleration (deg/s²)
#define DEFAULT_ACCELERATION 5.0        // Default acceleration (rad/s²)

//=============================================================================
// CONTROL ARCHITECTURE SELECTOR
//=============================================================================
// Toggle between two FOC control architectures:
//
//   USE_TORQUE_MODE = false  →  Angle mode (cascaded PID)
//     Position P → Velocity PI → Voltage → FOC → Motor
//     Pros: SimpleFOC built-in, easy to configure
//     Cons: Velocity loop integrates noisy I2C-derived velocity → widespread
//           limit-cycle oscillation confirmed (see Dev_Log/MOTOR_CONTROL_STATE.md)
//
//   USE_TORQUE_MODE = true   →  Torque mode + manual PD (SmartKnob pattern)
//     voltage = Kp × pos_error − Kd × velocity → FOC → Motor
//     Pros: No velocity integral → no windup → stable at all positions
//     Cons: Proportional droop (steady-state error = friction/Kp); no built-in
//           velocity limit (handled by setpoint ramp, already implemented)
//
// Start with true. Switch to false to compare.
#define USE_TORQUE_MODE  false

// Torque mode PD gains (used only when USE_TORQUE_MODE = true)
// Units: Kp in V/rad, Kd in V/(rad/s)
// Kd tuned for REAL velocity (position-delta, not LPF-corrupted motor.shaft_velocity).
// At 400°/s actual: Kd=0.05 → 0.35V braking (light); Kd=0.3 → 2.09V (too strong → oscillation).
// I_CAP=1.0V: enough to break friction (motor needs ~0.8-1.2V to overcome cogging) without
// violent overshoot when integral charges during momentary slow patches of oscillation.
#define TORQUE_KP         4.0f   // V/rad — raised from 3.0 to compensate for lower I_CAP; at 3° static error: 4.0×0.052=0.21V + 0.6V integral = 0.81V (enough to break cogging ~0.8V)
#define TORQUE_KD         0.0f   // V/(rad/s) — set to 0: position-delta vel at 11ms amplifies cogging noise into oscillation. Back-EMF provides natural damping. High-vel Kd (>100°/s) tested at 0.15 — still caused ±400°/s oscillation.
// Sweep velocity control: PI controller that tracks target sweep speed directly,
// avoiding the stall-and-snap of position ramp tracking.
// During stall: integral builds until breaking through the detent (adaptive torque).
// During snap-through: velocity >> target → large negative error → integral drains quickly,
// preventing overshoot build-up. Natural damping without velocity-based braking.
#define SWEEP_VEL_KP   0.01f   // V / (deg/s): gentle proportional (0.01 × 1deg/s = 0.01V)
#define SWEEP_VEL_KI   1.5f    // V / (deg): integral rate (1.5V/s at stall → 2V cap reached in 1.33s < 2s threshold)
#define SWEEP_VEL_I_CAP 2.0f   // V: integral anti-windup cap (max cogging force is ~1.5V)
#define SWEEP_VEL_TAU  0.15f   // s: LPF time constant for encoder-delta velocity noise
#define TORQUE_KI         20.0f  // V/(rad·s) — integral to overcome friction deadband (rate = KI×error V/s, loop-rate independent with real dt)
#define TORQUE_I_CAP      0.8f   // V — integral anti-windup cap: 0.6V too low (can't escape cogging at 0.76° error), 1.0V too high (overshoots → limit cycle). 0.8V: at 0.76° error: Kp=0.053V + 0.8V = 0.85V — just enough to break cogging threshold ~0.8V

// Position control PID parameters (used when USE_TORQUE_MODE = false)
// Source: Official SimpleFOC angle control docs + SmartKnob reference project
//   docs.simplefoc.com/angle_loop — P_angle.P=20 is the standard starting value
#define PID_P_POSITION   20.0           // Ceiling at LPF=0.05: P=25 → 1 oscillating, P=30 → 2 chaotic. 20 is stable 24/24.
#define PID_I_POSITION   0.0            // Integral gain (not needed; pure P is sufficient)
#define PID_D_POSITION   0.2            // Derivative gain — light damping to reduce overshoot
#define PID_RAMP_POSITION_DEG 1000.0    // Output ramp (deg/s)
#define PID_RAMP_POSITION 100.0         // Output ramp (rad/s) - for SimpleFOC

// Velocity control PID parameters (used when USE_TORQUE_MODE = false — inner loop)
// Source: Official SimpleFOC gimbal controller example
//   docs.simplefoc.com/gimbal_velocity_example
//   motor.PID_velocity.P = 0.2; motor.LPF_velocity.Tf = 0.01;
// NOTE: I_vel=5.0 confirmed to cause widespread limit-cycle oscillation (canary test 2026-03-25).
//       In torque mode these values are configured but not used.
#define PID_P_VELOCITY   0.3            // Standard for gimbal motors — P=0.5 caused widespread oscillation; LPF=0.02 requires 0.2 but is marginal
#define PID_I_VELOCITY   0.1            // OPTIMAL: I=5.0 oscillates, I=0.5 borderline, I=0.2 → 2 positions oscillate, I=0.15 → 3 positions oscillate, I=0.1 → 24/24 stable
#define PID_D_VELOCITY   0.0            // Always 0 for gimbal motors (amplifies noise)
#define PID_RAMP_VELOCITY 1000.0        // V/s output ramp (official gimbal example value)
#define PID_LPF_VELOCITY 0.05           // 50ms filter — 3.2Hz bandwidth. Ceiling: LPF=0.03 more oscillation (noise floor), LPF=0.02 marginal.

// Current control PID parameters (for FOC)
#define PID_P_CURRENT    5.0            // Proportional gain for current control
#define PID_I_CURRENT    100.0          // Integral gain for current control
#define PID_D_CURRENT    0.0            // Derivative gain for current control
#define PID_RAMP_CURRENT 1000.0         // Output ramp for current control

//=============================================================================
// ANTI-COGGING FEEDFORWARD
//=============================================================================
// Sinusoidal voltage injected onto motor.voltage.q after the PID output to
// cancel the dominant cogging harmonic.
//
// Motor: 12 stator slots × 14 poles (7 PP) → LCM(12,14)=84 cogging detents/rev
//   → 4 cogging cycles per electrical revolution → inject at 4× electrical_angle
//
// Disabled by default (AMP=0). Tune at runtime via:
//   set cog_amp <V>    — amplitude in volts (start 0, step up 0.05V at worst position)
//   set cog_phase <rad>— phase offset (sweep 0→6.28 at worst position, find minimum noise)
//
// Use 'cogmap' command to measure steady-state voltage.q at all 24 test positions.
// That map reveals the cogging pattern and guides AMP/PHASE selection.
#define COG_FF_AMP    0.0f    // Feedforward amplitude (V) — 0 = disabled
#define COG_FF_PHASE  0.0f    // Feedforward phase offset (rad)

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
#define VELOCITY_THRESHOLD_DEG 5.0      // Velocity threshold (deg/s) — raised from 0.57: LPF noise floor at 100Hz loop is ~1-3°/s
#define VELOCITY_THRESHOLD 0.087        // Velocity threshold (rad/s) - for SimpleFOC (~5°/s)

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
// ENCODER STREAMING
//=============================================================================
// Tagged-line encoder streaming for position logging and oscillation diagnosis.
// Lines prefixed with '$ENC,' are data; everything else is debug text.
// Python filters on prefix, serial monitor stays usable for debugging.
//
// Format: $ENC,<timestamp_ms>,<position_deg>,<velocity_deg_s>,<target_deg>
// Markers: $SCAN_START,<timestamp_ms>  and  $SCAN_END,<timestamp_ms>
#define STREAM_DEFAULT_ENABLED   false          // Streaming off by default (toggle with 'stream on')
#define STREAM_RATE_HZ           100            // Default stream rate (Hz) - uses ~35% of 115200 baud
#define STREAM_MIN_RATE_HZ       1              // Minimum configurable rate
#define STREAM_MAX_RATE_HZ       500            // Maximum configurable rate (limited by loop + I2C)

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

// Binary protocol: Set to false to disable binary packet output (the ~��� gibberish)
// When debugging with serial monitor, disable this to see only text output
#define ENABLE_BINARY_PROTOCOL   false          // Disable binary packets for serial debugging

// Legacy defines for backward compatibility during transition
#define DEBUG_SERIAL     DEBUG_SERIAL_DEFAULT
#define DEBUG_MOTOR      DEBUG_MOTOR_DEFAULT
#define DEBUG_COMM       DEBUG_COMM_DEFAULT
#define DEBUG_HEARTBEAT  DEBUG_HEARTBEAT_DEFAULT

#endif // CONFIG_H
