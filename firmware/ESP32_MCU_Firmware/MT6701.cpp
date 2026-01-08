/**
 * MT6701.cpp - Arduino library for MT6701 14-bit magnetic encoder
 *
 * Based on: https://github.com/I-AM-ENGINEER/MT6701-driver
 * Adapted for Arduino/ESP32 with Wire library
 */

#include "MT6701.h"

MT6701::MT6701(uint8_t address) {
    _address = address;
    _wire = nullptr;
    _initialized = false;
    _last_valid_angle = 0;  // Initialize cache
}

bool MT6701::begin(TwoWire *wire) {
    _wire = wire;

    // Test connection
    if (!testConnection()) {
        _initialized = false;
        return false;
    }

    _initialized = true;

    // Initialize the failure recovery cache with current position
    // This prevents returning 0 if the first read after begin() fails
    uint8_t buffer[2];
    if (readRegisters(MT6701_REG_ANGLE_MSB, buffer, 2)) {
        uint16_t angle = ((uint16_t)buffer[0] << 6) | (buffer[1] & 0x3F);
        angle &= 0x3FFF;
        _last_valid_angle = angle;
    }
    // If initial read fails, _last_valid_angle stays at 0 (set in constructor)
    // This is acceptable since we verified connection works via testConnection()

    return true;
}

uint16_t MT6701::readRawAngle() {
    if (!_initialized) {
        // Return last valid reading instead of 0
        return _last_valid_angle;
    }

    uint8_t buffer[2];
    if (!readRegisters(MT6701_REG_ANGLE_MSB, buffer, 2)) {
        // I2C communication failed - return last valid reading
        // This prevents shaft_angle from resetting to 0° on transient errors
        return _last_valid_angle;
    }

    // Combine MSB and LSB (14-bit value)
    // MSB contains bits 13-6, LSB contains bits 5-0
    uint16_t angle = ((uint16_t)buffer[0] << 6) | (buffer[1] & 0x3F);

    // Mask to 14 bits
    angle &= 0x3FFF;

    // Cache successful read for I2C failure recovery
    _last_valid_angle = angle;

    return angle;
}

float MT6701::readAngleRadians() {
    uint16_t raw = readRawAngle();
    // Convert raw value (0-16383) to radians (0-2π)
    return (float)raw * (2.0 * PI) / (float)MT6701_RESOLUTION;
}

float MT6701::readAngleDegrees() {
    uint16_t raw = readRawAngle();
    // Convert raw value (0-16383) to degrees (0-360)
    return (float)raw * 360.0 / (float)MT6701_RESOLUTION;
}

uint8_t MT6701::readFieldStatus() {
    if (!_initialized) {
        return 0xFF;
    }

    uint8_t status;
    if (!readRegister(MT6701_REG_FIELD_STATUS, &status)) {
        return 0xFF;
    }

    return status & 0x03;  // Only interested in lower 2 bits
}

bool MT6701::isFieldGood() {
    uint8_t status = readFieldStatus();
    return (status == MT6701_FIELD_GOOD);
}

bool MT6701::setZeroToCurrent() {
    uint16_t current_angle = readRawAngle();

    // Write to zero position registers
    uint8_t buffer[2];
    buffer[0] = (current_angle >> 6) & 0xFF;  // MSB (bits 13-6)
    buffer[1] = current_angle & 0x3F;          // LSB (bits 5-0)

    return writeRegisters(MT6701_REG_ZERO_MSB, buffer, 2);
}

bool MT6701::setZero(float angle) {
    // Convert radians to raw value
    uint16_t raw = (uint16_t)(angle * (float)MT6701_RESOLUTION / (2.0 * PI));
    raw &= 0x3FFF;  // Mask to 14 bits

    // Write to zero position registers
    uint8_t buffer[2];
    buffer[0] = (raw >> 6) & 0xFF;  // MSB (bits 13-6)
    buffer[1] = raw & 0x3F;          // LSB (bits 5-0)

    return writeRegisters(MT6701_REG_ZERO_MSB, buffer, 2);
}

bool MT6701::setDirection(bool clockwise) {
    uint8_t dir = clockwise ? MT6701_DIR_CLOCKWISE : MT6701_DIR_COUNTER_CW;
    return writeRegister(MT6701_REG_DIRECTION, dir);
}

float MT6701::readZeroPosition() {
    if (!_initialized) {
        return 0.0;
    }

    uint8_t buffer[2];
    if (!readRegisters(MT6701_REG_ZERO_MSB, buffer, 2)) {
        return 0.0;
    }

    // Combine MSB and LSB (14-bit value)
    uint16_t zero = ((uint16_t)buffer[0] << 6) | (buffer[1] & 0x3F);
    zero &= 0x3FFF;

    // Convert to radians
    return (float)zero * (2.0 * PI) / (float)MT6701_RESOLUTION;
}

bool MT6701::isClockwise() {
    if (!_initialized) {
        return true;
    }

    uint8_t dir;
    if (!readRegister(MT6701_REG_DIRECTION, &dir)) {
        return true;
    }

    return (dir == MT6701_DIR_CLOCKWISE);
}

bool MT6701::testConnection() {
    if (_wire == nullptr) {
        return false;
    }

    _wire->beginTransmission(_address);
    uint8_t error = _wire->endTransmission();

    return (error == 0);
}

bool MT6701::readRegister(uint8_t reg, uint8_t *value) {
    if (_wire == nullptr || value == nullptr) {
        return false;
    }

    _wire->beginTransmission(_address);
    _wire->write(reg);
    uint8_t error = _wire->endTransmission(false);  // Send repeated start

    if (error != 0) {
        return false;
    }

    uint8_t bytes = _wire->requestFrom(_address, (uint8_t)1);
    if (bytes != 1) {
        return false;
    }

    *value = _wire->read();
    return true;
}

bool MT6701::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    if (_wire == nullptr || buffer == nullptr || length == 0) {
        return false;
    }

    _wire->beginTransmission(_address);
    _wire->write(reg);
    uint8_t error = _wire->endTransmission(false);  // Send repeated start

    if (error != 0) {
        return false;
    }

    uint8_t bytes = _wire->requestFrom(_address, length);
    if (bytes != length) {
        return false;
    }

    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = _wire->read();
    }

    return true;
}

bool MT6701::writeRegister(uint8_t reg, uint8_t value) {
    if (_wire == nullptr) {
        return false;
    }

    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value);
    uint8_t error = _wire->endTransmission();

    return (error == 0);
}

bool MT6701::writeRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
    if (_wire == nullptr || buffer == nullptr || length == 0) {
        return false;
    }

    _wire->beginTransmission(_address);
    _wire->write(reg);
    for (uint8_t i = 0; i < length; i++) {
        _wire->write(buffer[i]);
    }
    uint8_t error = _wire->endTransmission();

    return (error == 0);
}
