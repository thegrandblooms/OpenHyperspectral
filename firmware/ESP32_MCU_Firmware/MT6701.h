/**
 * MT6701.h - Arduino library for MT6701 14-bit magnetic encoder
 *
 * Based on: https://github.com/I-AM-ENGINEER/MT6701-driver
 * Adapted for Arduino/ESP32 with Wire library
 *
 * The MT6701 is a 14-bit magnetic rotary position sensor with I2C interface.
 * Features:
 * - 14-bit resolution (16,384 positions per revolution)
 * - I2C interface (address 0x06)
 * - Configurable zero position
 * - Direction control
 * - Field strength monitoring
 * - Non-volatile EEPROM configuration
 *
 * Author: Adapted for OpenHyperspectral
 * License: MIT
 */

#ifndef MT6701_H
#define MT6701_H

#include <Arduino.h>
#include <Wire.h>

// Default I2C address
#define MT6701_DEFAULT_ADDR 0x06

// Register addresses
#define MT6701_REG_ANGLE_MSB    0x03  // Angle high byte
#define MT6701_REG_ANGLE_LSB    0x04  // Angle low byte
#define MT6701_REG_FIELD_STATUS 0x1B  // Field strength status
#define MT6701_REG_DIRECTION    0x29  // Direction setting
#define MT6701_REG_ZERO_MSB     0x00  // Zero position high byte
#define MT6701_REG_ZERO_LSB     0x01  // Zero position low byte

// Field strength status bits
#define MT6701_FIELD_TOO_STRONG 0x01
#define MT6701_FIELD_TOO_WEAK   0x02
#define MT6701_FIELD_GOOD       0x00

// Direction values
#define MT6701_DIR_CLOCKWISE    0x00
#define MT6701_DIR_COUNTER_CW   0x01

// Resolution
#define MT6701_RESOLUTION       16384  // 2^14 = 16,384 counts per revolution

class MT6701 {
public:
    /**
     * Constructor
     * @param address I2C address (default 0x06)
     */
    MT6701(uint8_t address = MT6701_DEFAULT_ADDR);

    /**
     * Initialize the encoder
     * @param wire Pointer to Wire object (default &Wire)
     * @return true if initialization successful
     */
    bool begin(TwoWire *wire = &Wire);

    /**
     * Read raw angle value (0-16383)
     * @return Raw 14-bit angle value
     */
    uint16_t readRawAngle();

    /**
     * Read angle in radians (0 to 2π)
     * @return Angle in radians
     */
    float readAngleRadians();

    /**
     * Read angle in degrees (0-360)
     * @return Angle in degrees
     */
    float readAngleDegrees();

    /**
     * Read field strength status
     * @return Status byte (0=good, 1=too strong, 2=too weak)
     */
    uint8_t readFieldStatus();

    /**
     * Check if magnetic field is within acceptable range
     * @return true if field is good
     */
    bool isFieldGood();

    /**
     * Set zero position to current angle
     * @return true if successful
     */
    bool setZeroToCurrent();

    /**
     * Set zero position to specific angle
     * @param angle Angle in radians
     * @return true if successful
     */
    bool setZero(float angle);

    /**
     * Set rotation direction
     * @param clockwise true for clockwise, false for counter-clockwise
     * @return true if successful
     */
    bool setDirection(bool clockwise);

    /**
     * Read current zero position setting
     * @return Zero position in radians
     */
    float readZeroPosition();

    /**
     * Get current direction setting
     * @return true if clockwise, false if counter-clockwise
     */
    bool isClockwise();

    /**
     * Test I2C communication
     * @return true if device responds
     */
    bool testConnection();

    /**
     * Get the I2C address
     * @return I2C address
     */
    uint8_t getAddress() { return _address; }

private:
    TwoWire *_wire;
    uint8_t _address;
    bool _initialized;

    /**
     * Read a single byte from register
     * @param reg Register address
     * @param value Pointer to store read value
     * @return true if successful
     */
    bool readRegister(uint8_t reg, uint8_t *value);

    /**
     * Read multiple bytes from register
     * @param reg Starting register address
     * @param buffer Buffer to store data
     * @param length Number of bytes to read
     * @return true if successful
     */
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);

    /**
     * Write a single byte to register
     * @param reg Register address
     * @param value Value to write
     * @return true if successful
     */
    bool writeRegister(uint8_t reg, uint8_t value);

    /**
     * Write multiple bytes to register
     * @param reg Starting register address
     * @param buffer Data to write
     * @param length Number of bytes to write
     * @return true if successful
     */
    bool writeRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
};

#endif // MT6701_H
