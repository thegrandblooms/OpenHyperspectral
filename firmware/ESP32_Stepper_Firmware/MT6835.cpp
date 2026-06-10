#include "MT6835.h"

// MT6835 SPI commands (upper 4 bits of the 24-bit frame)
static const uint8_t CMD_READ  = 0b0011;
static const uint8_t CMD_BURST = 0b1010;   // burst angle read starting at 0x003

static SPISettings spi_settings(ENCODER_SPI_HZ, MSBFIRST, SPI_MODE3);

MT6835::MT6835(uint8_t cs)
    : cs_pin(cs), raw_count(0), angle_deg(0.0f), status_bits(0),
      read_count(0), crc_error_count(0) {}

void MT6835::begin() {
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
    // Explicit pin mapping (ESP32-S3 GPIO matrix routes SPI to any pin)
    SPI.begin(ENCODER_SCK_PIN, ENCODER_MISO_PIN, ENCODER_MOSI_PIN, cs_pin);
    delay(10);  // power-on settle
    update();   // prime the cached reading
}

uint8_t MT6835::readRegister(uint16_t addr) {
    uint8_t tx[3] = {
        (uint8_t)((CMD_READ << 4) | ((addr >> 8) & 0x0F)),
        (uint8_t)(addr & 0xFF),
        0x00
    };
    uint8_t rx[3] = {0};

    SPI.beginTransaction(spi_settings);
    digitalWrite(cs_pin, LOW);
    for (int i = 0; i < 3; i++) rx[i] = SPI.transfer(tx[i]);
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();

    return rx[2];  // data byte arrives in the third byte of the frame
}

bool MT6835::update() {
    // Burst angle read: command frame, then 0x003,0x004,0x005,0x006 clock out
    // in consecutive bytes within the same CS window.
    uint8_t tx[6] = {
        (uint8_t)((CMD_BURST << 4) | 0x00),  // addr 0x003 high nibble = 0
        0x03,                                 // addr 0x003 low byte
        0x00, 0x00, 0x00, 0x00
    };
    uint8_t rx[6] = {0};

    SPI.beginTransaction(spi_settings);
    digitalWrite(cs_pin, LOW);
    for (int i = 0; i < 6; i++) rx[i] = SPI.transfer(tx[i]);
    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();

    // rx[2]=ANGLE[20:13], rx[3]=ANGLE[12:5], rx[4]={ANGLE[4:0],STATUS[2:0]}, rx[5]=CRC
    uint8_t crc_calc = crc8(&rx[2], 3);
    read_count++;
    if (crc_calc != rx[5]) {
        crc_error_count++;
        return false;  // keep previous good reading
    }

    uint32_t angle = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5) | (rx[4] >> 3);
    status_bits = rx[4] & 0x07;

    raw_count = angle;
    angle_deg = (angle / (float)ENCODER_COUNTS_PER_REV) * 360.0f;
    return true;
}

// CRC-8 with polynomial 0x1D (x^8+x^4+x^3+x^2+1), init 0x00 — per MT6835 datasheet
uint8_t MT6835::crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}
