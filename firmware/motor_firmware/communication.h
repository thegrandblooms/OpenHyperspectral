#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SerialTransfer.h>
#include "config.h"
#include "commands.h"

//=============================================================================
// COMMUNICATION MODULE
//=============================================================================

class CommunicationManager {
public:
    CommunicationManager();

    // Initialize communication
    void begin(uint32_t baud_rate = SERIAL_BAUD);

    // Check for available commands
    bool available();

    // Get the received command ID
    uint8_t getCommandId();

    // Get command data
    template<typename T>
    bool getCommandData(T* data);

    // Send responses
    void sendOkResponse(uint8_t command_id);
    void sendErrorResponse(uint8_t command_id, uint8_t error_code);
    void sendPositionReached(uint16_t sequence_id, float position);
    void sendPingResponse(uint32_t echo_value);
    void sendStatus(uint8_t state, uint8_t control_mode, float position,
                   float velocity, float current, float voltage,
                   bool motor_enabled, bool calibrated);

private:
    SerialTransfer transfer;
    uint8_t command_id;
    bool command_available;

    // Helper to send data
    template<typename T>
    void sendData(const T* data, uint16_t size);
};

// Template implementation
template<typename T>
bool CommunicationManager::getCommandData(T* data) {
    if (!command_available) {
        return false;
    }

    // Extract data from receive buffer (skip command ID byte)
    // Check if we have enough data (command ID + data)
    memcpy(data, &transfer.rxBuff[1], sizeof(T));
    return true;
}

template<typename T>
void CommunicationManager::sendData(const T* data, uint16_t size) {
    // Copy data to transmit buffer
    memcpy(&transfer.txBuff[0], data, size);

    // Send the data
    transfer.sendDatum(size);
}

#endif // COMMUNICATION_H
