#ifndef MT6835_H
#define MT6835_H

#include <Arduino.h>
#include <SPI.h>
#include "config.h"

//=============================================================================
// MT6835 21-BIT ABSOLUTE MAGNETIC ENCODER (SPI)
//=============================================================================
//
// Standalone driver (no SimpleFOC dependency). Mounted on the OUTPUT shaft,
// so it measures the truth after the belt reduction — including lash.
//
// SPI protocol (per MT6835 datasheet, verify against your datasheet rev):
//   - SPI mode 3 (CPOL=1, CPHA=1), MSB first
//   - Frame: 4-bit command + 12-bit register address + 8-bit data
//   - CMD 0b0011 = register read, CMD 0b1010 = burst angle read
//   - Angle registers: 0x003 ANGLE[20:13], 0x004 ANGLE[12:5],
//                      0x005 {ANGLE[4:0], STATUS[2:0]}, 0x006 CRC
//   - STATUS bits: [2] overspeed, [1] field weak, [0] undervoltage
//
// The burst read (0xA) clocks out 0x003..0x006 in one CS window — one
// transaction returns angle + status + CRC. That's what readAngle() uses.
//
// NOTE: The MT6835 also has a built-in self-calibration feature (spin at
// constant speed, pin-triggered) that reduces its INL substantially.
// Run it once after mechanical assembly — see datasheet section on
// self-calibration. Until then expect a few tenths of a degree INL.
//=============================================================================

class MT6835 {
public:
    MT6835(uint8_t cs_pin = ENCODER_CS_PIN);

    void begin();

    // Read the current angle. Returns true on success (CRC ok, no error flags).
    // On failure, the previous good reading is retained.
    bool update();

    // Accessors (values from the last successful update())
    uint32_t getRawCount() const   { return raw_count; }        // 0 .. 2^21-1
    float    getDegrees() const    { return angle_deg; }        // 0 .. 360
    uint8_t  getStatusBits() const { return status_bits; }      // 3 status bits
    bool     isFieldGood() const   { return (status_bits & 0x02) == 0; }
    bool     isOverspeed() const   { return (status_bits & 0x04) != 0; }
    bool     isUndervolt() const   { return (status_bits & 0x01) != 0; }

    // Diagnostics
    unsigned long getReadCount() const  { return read_count; }
    unsigned long getCrcErrorCount() const { return crc_error_count; }

    // Single register read (for bring-up / debugging)
    uint8_t readRegister(uint16_t addr);

private:
    uint8_t  cs_pin;
    uint32_t raw_count;
    float    angle_deg;   // "degrees" collides with the Arduino degrees() macro
    uint8_t  status_bits;
    unsigned long read_count;
    unsigned long crc_error_count;

    static uint8_t crc8(const uint8_t* data, size_t len);  // CRC-8/SAE-J1850 (poly 0x1D)
};

#endif // MT6835_H
