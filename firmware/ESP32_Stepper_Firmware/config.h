#ifndef CONFIG_H
#define CONFIG_H

//=============================================================================
// HARDWARE CONFIGURATION FOR ESP32-S3-Touch-LCD-2 (Waveshare)
// Stepper rewrite: NEMA17 + TMC2209 (STEP/DIR + UART) + belt reduction,
// MT6835 21-bit absolute encoder on the OUTPUT shaft.
//
// Why this architecture (see Dev_Log/motor/STATE.md for the full history):
// - The 2804 gimbal motor's 84-detent cogging put a ~0.45V torque dead zone
//   between the controller and sub-detent precision. Every FOC control
//   architecture hit the same stall-snap-bounce limit cycle.
// - The MT6701's +/-1.0 deg INL capped absolute accuracy regardless of control.
// - A stepper holds position PASSIVELY (no feedback loop -> no hunting), and
//   the belt reduction divides motor-side error by the ratio. The MT6835 on
//   the output shaft measures the truth (including belt lash), and a slow
//   closed-loop trim removes residual error between moves.
//=============================================================================

//=============================================================================
// TMC2209 STEPPER DRIVER PINS
//=============================================================================
// !! PLACEHOLDER PINS — verify against your wiring before first power-up. !!
// Any free GPIO works (ESP32-S3 GPIO matrix); these reuse the header cluster
// freed up by removing the SimpleFOC Mini.
#define STEPPER_STEP_PIN   13   // STEP input (rising edge = one microstep)
#define STEPPER_DIR_PIN    11   // DIR input (sampled on STEP edge; >20ns setup)
#define STEPPER_EN_PIN     12   // EN input (ACTIVE LOW: LOW = driver on)

// TMC2209 UART (single-wire PDN_UART). Optional but recommended: enables
// runtime microstep/current config, StealthChop control, and StallGuard.
// Wiring: ESP32 TX -> 1k resistor -> PDN_UART; ESP32 RX -> PDN_UART directly.
#define USE_TMC_UART       true
#define TMC_UART_TX_PIN    15
#define TMC_UART_RX_PIN    14
#define TMC_UART_BAUD      115200
#define TMC_DRIVER_ADDR    0b00   // MS1/MS2 low = address 0
#define TMC_R_SENSE        0.11f  // Ohms — standard on TMC2209 breakout modules
#define TMC_RMS_CURRENT_MA 800    // Motor RMS current. Set <= motor rating.
                                  // Typical NEMA17: 0.8-1.7A. Start conservative.

//=============================================================================
// MT6835 ENCODER PINS (21-bit absolute, SPI, on OUTPUT shaft)
//=============================================================================
// !! PLACEHOLDER PINS — verify against your wiring. !!
#define ENCODER_CS_PIN     9
#define ENCODER_SCK_PIN    48
#define ENCODER_MISO_PIN   47
#define ENCODER_MOSI_PIN   16
#define ENCODER_SPI_HZ     1000000   // MT6835 supports up to 16MHz; 1MHz is plenty

#define ENCODER_COUNTS_PER_REV 2097152UL  // 2^21 = 0.000172 deg/count

//=============================================================================
// MECHANICS
//=============================================================================
#define MOTOR_FULL_STEPS_PER_REV 200    // 1.8 deg stepper
#define MICROSTEPS               16     // TMC2209 microstep setting (via UART).
                                        // TMC2209 interpolates to 256 internally
                                        // (MicroPlyer) so motion is smooth even at 16.
#define GEAR_RATIO               3.0f   // Belt reduction: motor revs per output rev.
                                        // UPDATE to match your pulley teeth ratio.

// Derived resolution (do not edit):
//   steps/output-rev = 200 * 16 * 3.0 = 9600  ->  0.0375 deg/microstep at output
#define STEPS_PER_OUTPUT_REV  ((float)MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO)
#define STEPS_PER_OUTPUT_DEG  (STEPS_PER_OUTPUT_REV / 360.0f)

//=============================================================================
// MOTION PROFILE (all values in OUTPUT-shaft degrees)
//=============================================================================
#define MAX_VELOCITY_DEG      60.0f    // Cruise speed for point-to-point moves (deg/s)
#define MAX_ACCELERATION_DEG  120.0f   // Trapezoid accel/decel (deg/s^2)
#define MIN_STEP_RATE_HZ      4.0f     // Floor for profile start/stop (avoids 0-rate stall)
#define SWEEP_MAX_DEG_S       20.0f    // Sanity cap on 'sweep <deg/s>' command

// Step generator tick rate. DDA accumulator in a hardware-timer ISR.
// 20kHz tick supports step rates up to ~10kHz with <=1 tick jitter;
// our max is 60 deg/s * 26.67 steps/deg = 1600 steps/s. Lots of margin.
#define STEP_TICK_HZ          20000

//=============================================================================
// CLOSED-LOOP TRIM (encoder-verified positioning)
//=============================================================================
// Moves execute open-loop (steppers don't miss steps at our rates/loads),
// then the output encoder is read and residual error (belt lash, compliance,
// microstep nonlinearity) is removed with small correction moves.
// Between corrections the motor holds passively — no hunting, ever.
#define TRIM_ENABLED_DEFAULT   true
#define TRIM_TOLERANCE_DEG     0.02f   // Stop trimming when |error| below this
#define TRIM_MAX_ITERATIONS    5       // Give up (and report) after this many trims
#define TRIM_SETTLE_MS         150     // Wait after motion stops before reading encoder
#define TRIM_MAX_CORRECTION_DEG 5.0f   // Safety clamp on a single correction move

// Backlash handling: always make the FINAL approach in one consistent
// direction so belt lash is taken up the same way every time.
// If a move's net direction is opposite the approach direction, overshoot
// past the target and come back.
#define UNIDIRECTIONAL_APPROACH  true
#define APPROACH_DIR_POSITIVE    true   // true: final approach always moves +deg
#define BACKLASH_OVERSHOOT_DEG   1.5f   // Overshoot before reversing into target

//=============================================================================
// POSITION TRACKING
//=============================================================================
#define POSITION_TOLERANCE_DEG 0.05f   // isAtTarget() threshold (output encoder)
#define VELOCITY_THRESHOLD_DEG 0.5f    // isAtTarget() velocity threshold (deg/s)

//=============================================================================
// SYSTEM STATES (same values as gimbal firmware — protocol compatible)
//=============================================================================
#define STATE_IDLE               0
#define STATE_MOVING             1
#define STATE_ERROR              2
#define STATE_CALIBRATING        3

//=============================================================================
// CONTROL MODES (protocol compatibility; stepper is always position-based)
//=============================================================================
#define MODE_POSITION            0
#define MODE_VELOCITY            1
#define MODE_TORQUE              2
#define DEFAULT_CONTROL_MODE MODE_POSITION

//=============================================================================
// SERIAL COMMUNICATION
//=============================================================================
#define SERIAL_BAUD      115200
#define SERIAL_TIMEOUT   1000

//=============================================================================
// SYSTEM TIMING
//=============================================================================
#define LOOP_FREQUENCY_HZ 1000
#define LOOP_PERIOD_US    (1000000 / LOOP_FREQUENCY_HZ)

//=============================================================================
// ENCODER STREAMING (same $ENC tagged-CSV protocol as gimbal firmware)
//=============================================================================
// Format (6 fields, field-count compatible with sweep_validation.py):
//   $ENC,<timestamp_ms>,<enc_pos_deg>,<enc_vel_deg_s>,<target_deg>,<cmd_pos_deg>,<trim_err_deg>
// cmd_pos_deg  = open-loop commanded position (step counter)
// trim_err_deg = encoder - commanded (belt lash / compliance signature)
#define STREAM_DEFAULT_ENABLED   false
#define STREAM_RATE_HZ           100
#define STREAM_MIN_RATE_HZ       1
#define STREAM_MAX_RATE_HZ       500

//=============================================================================
// DEBUGGING
//=============================================================================
#define DEBUG_SERIAL_DEFAULT     true
#define DEBUG_MOTOR_DEFAULT      true
#define DEBUG_COMM_DEFAULT       true
#define DEBUG_HEARTBEAT_DEFAULT  true
#define HEARTBEAT_INTERVAL_MS    10000

// Binary protocol off by default (text serial monitor stays readable)
#define ENABLE_BINARY_PROTOCOL   false

// Legacy defines used by communication.cpp / debug_globals.h
#define DEBUG_SERIAL     DEBUG_SERIAL_DEFAULT
#define DEBUG_MOTOR      DEBUG_MOTOR_DEFAULT
#define DEBUG_COMM       DEBUG_COMM_DEFAULT
#define DEBUG_HEARTBEAT  DEBUG_HEARTBEAT_DEFAULT

#endif // CONFIG_H
