#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>

//=============================================================================
// COMMAND IDs (PC → ESP32)
//=============================================================================
// These match the HyperMicro protocol for compatibility
#define CMD_MOVE_TO              0x01   // Move to absolute position
#define CMD_SET_SPEED            0x03   // Set movement speed
#define CMD_SET_ACCEL            0x04   // Set acceleration
#define CMD_STOP                 0x05   // Emergency stop
#define CMD_HOME                 0x06   // Set current position as home (zero)
#define CMD_ENABLE               0x07   // Enable motor
#define CMD_DISABLE              0x08   // Disable motor
#define CMD_GET_STATUS           0x09   // Request status information
#define CMD_PING                 0x0A   // Connection test
#define CMD_SET_MODE             0x0B   // Set control mode
#define CMD_SET_CURRENT_LIMIT    0x0C   // Set FOC current limit (new for SimpleFOC)
#define CMD_CALIBRATE            0x0D   // Run motor calibration (new for SimpleFOC)
#define CMD_SET_PID              0x0E   // Set PID parameters (new for SimpleFOC)

//=============================================================================
// RESPONSE IDs (ESP32 → PC)
//=============================================================================
#define RESP_OK                  0x81   // Command acknowledged successfully
#define RESP_ERROR               0x82   // Command failed
#define RESP_POSITION_REACHED    0x84   // Target position reached
#define RESP_PING                0x86   // Ping response
#define RESP_STATUS              0x87   // Status information

//=============================================================================
// ERROR CODES
//=============================================================================
#define ERR_INVALID_COMMAND      0x01   // Unknown command ID
#define ERR_INVALID_PARAMETER    0x02   // Invalid parameter value
#define ERR_COMMAND_FAILED       0x03   // Command execution failed
#define ERR_MOTOR_FAULT          0x04   // Motor fault detected
#define ERR_OVERCURRENT          0x05   // Overcurrent detected
#define ERR_OVERHEAT             0x06   // Overheat detected
#define ERR_NOT_CALIBRATED       0x07   // Motor not calibrated
#define ERR_ENCODER_FAULT        0x08   // Encoder fault detected

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
#define MODE_POSITION            0      // Position control mode
#define MODE_VELOCITY            1      // Velocity control mode
#define MODE_TORQUE              2      // Torque control mode

//=============================================================================
// COMMAND DATA STRUCTURES
//=============================================================================

/**
 * Command: MOVE_TO
 * Move to absolute position in radians
 *
 * Data format:
 * - float position (4 bytes) - target position in radians
 * - uint16_t sequence_id (2 bytes) - sequence identifier for tracking
 */
struct MoveToCommand {
    float position;
    uint16_t sequence_id;
};

/**
 * Command: SET_SPEED
 * Set motor velocity in rad/s
 *
 * Data format:
 * - float velocity (4 bytes) - target velocity in rad/s
 */
struct SetSpeedCommand {
    float velocity;
};

/**
 * Command: SET_ACCEL
 * Set motor acceleration in rad/s²
 *
 * Data format:
 * - float acceleration (4 bytes) - target acceleration in rad/s²
 */
struct SetAccelCommand {
    float acceleration;
};

/**
 * Command: SET_CURRENT_LIMIT
 * Set maximum current limit for FOC
 *
 * Data format:
 * - float current_limit (4 bytes) - maximum current in amps
 */
struct SetCurrentLimitCommand {
    float current_limit;
};

/**
 * Command: SET_PID
 * Set PID parameters for a control loop
 *
 * Data format:
 * - uint8_t controller_type (1 byte) - 0=position, 1=velocity, 2=current
 * - float p (4 bytes) - proportional gain
 * - float i (4 bytes) - integral gain
 * - float d (4 bytes) - derivative gain
 * - float ramp (4 bytes) - output ramp limit
 */
struct SetPIDCommand {
    uint8_t controller_type;
    float p;
    float i;
    float d;
    float ramp;
};

/**
 * Command: PING
 * Connection test with echo value
 *
 * Data format:
 * - uint32_t echo_value (4 bytes) - value to echo back
 */
struct PingCommand {
    uint32_t echo_value;
};

//=============================================================================
// RESPONSE DATA STRUCTURES
//=============================================================================

/**
 * Response: OK
 * Acknowledge command received
 *
 * Data format:
 * - uint8_t response_id (1 byte) - RESP_OK
 * - uint8_t command_id (1 byte) - echoed command ID
 */
struct OkResponse {
    uint8_t response_id;
    uint8_t command_id;
};

/**
 * Response: ERROR
 * Report command error
 *
 * Data format:
 * - uint8_t response_id (1 byte) - RESP_ERROR
 * - uint8_t command_id (1 byte) - echoed command ID
 * - uint8_t error_code (1 byte) - error code
 */
struct ErrorResponse {
    uint8_t response_id;
    uint8_t command_id;
    uint8_t error_code;
};

/**
 * Response: POSITION_REACHED
 * Notify that target position has been reached
 *
 * Data format:
 * - uint8_t response_id (1 byte) - RESP_POSITION_REACHED
 * - uint16_t sequence_id (2 bytes) - sequence identifier
 * - float position (4 bytes) - actual position in radians
 * - uint32_t timestamp (4 bytes) - timestamp in milliseconds
 */
struct PositionReachedResponse {
    uint8_t response_id;
    uint16_t sequence_id;
    float position;
    uint32_t timestamp;
};

/**
 * Response: PING
 * Echo back ping value
 *
 * Data format:
 * - uint8_t response_id (1 byte) - RESP_PING
 * - uint32_t echo_value (4 bytes) - echoed value
 */
struct PingResponse {
    uint8_t response_id;
    uint32_t echo_value;
};

/**
 * Response: STATUS
 * Report current system status
 *
 * Data format:
 * - uint8_t response_id (1 byte) - RESP_STATUS
 * - uint8_t state (1 byte) - current system state
 * - uint8_t control_mode (1 byte) - current control mode
 * - float position (4 bytes) - current position in radians
 * - float velocity (4 bytes) - current velocity in rad/s
 * - float current (4 bytes) - current draw in amps
 * - float voltage (4 bytes) - supply voltage
 * - uint8_t motor_enabled (1 byte) - motor enable state
 * - uint8_t calibrated (1 byte) - calibration state
 */
struct StatusResponse {
    uint8_t response_id;
    uint8_t state;
    uint8_t control_mode;
    float position;
    float velocity;
    float current;
    float voltage;
    uint8_t motor_enabled;
    uint8_t calibrated;
};

#endif // COMMANDS_H
