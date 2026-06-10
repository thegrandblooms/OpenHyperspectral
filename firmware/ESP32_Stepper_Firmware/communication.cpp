#include "communication.h"

CommunicationManager::CommunicationManager()
    : command_id(0), command_available(false) {
}

void CommunicationManager::begin(uint32_t baud_rate) {
    // Initialize serial port
    Serial.begin(baud_rate);

    // Wait for serial to be ready
    while (!Serial && millis() < 5000) {
        delay(10);
    }

    // Initialize SerialTransfer
    transfer.begin(Serial);

    if (DEBUG_COMM) {
        Serial.println("Communication initialized");
    }
}

bool CommunicationManager::available() {
    // Check for available data
    if (transfer.available()) {
        // Get the command ID (first byte)
        command_id = transfer.packet.rxBuff[0];
        command_available = true;

        if (DEBUG_COMM) {
            Serial.print("Command received: 0x");
            Serial.println(command_id, HEX);
        }

        return true;
    }

    command_available = false;
    return false;
}

uint8_t CommunicationManager::getCommandId() {
    return command_id;
}

void CommunicationManager::sendOkResponse(uint8_t command_id) {
    OkResponse response;
    response.response_id = RESP_OK;
    response.command_id = command_id;

    sendData(&response, sizeof(response));

    if (DEBUG_COMM) {
        Serial.println("Sent OK response");
    }
}

void CommunicationManager::sendErrorResponse(uint8_t command_id, uint8_t error_code) {
    ErrorResponse response;
    response.response_id = RESP_ERROR;
    response.command_id = command_id;
    response.error_code = error_code;

    // Clear buffer first
    for (uint8_t i = 0; i < 8; i++) {
        transfer.packet.txBuff[i] = 0;
    }

    sendData(&response, sizeof(response));

    // Send twice for reliability (as in HyperMicro)
    delay(5);
    sendData(&response, sizeof(response));

    if (DEBUG_COMM) {
        Serial.print("Sent ERROR response: ");
        Serial.println(error_code, HEX);
    }
}

void CommunicationManager::sendPositionReached(uint16_t sequence_id, float position) {
    PositionReachedResponse response;
    response.response_id = RESP_POSITION_REACHED;
    response.sequence_id = sequence_id;
    response.position = position;
    response.timestamp = millis();

    sendData(&response, sizeof(response));

    if (DEBUG_COMM) {
        Serial.print("Position reached: seq=");
        Serial.print(sequence_id);
        Serial.print(", pos=");
        Serial.println(position);
    }
}

void CommunicationManager::sendPingResponse(uint32_t echo_value) {
    PingResponse response;
    response.response_id = RESP_PING;
    response.echo_value = echo_value;

    sendData(&response, sizeof(response));

    if (DEBUG_COMM) {
        Serial.print("Sent PING response: 0x");
        Serial.println(echo_value, HEX);
    }
}

void CommunicationManager::sendStatus(uint8_t state, uint8_t control_mode,
                                      float position, float velocity,
                                      float current, float voltage,
                                      bool motor_enabled, bool calibrated) {
    // Clear buffer first
    for (uint8_t i = 0; i < 32; i++) {
        transfer.packet.txBuff[i] = 0;
    }

    StatusResponse response;
    response.response_id = RESP_STATUS;
    response.state = state;
    response.control_mode = control_mode;
    response.position = position;
    response.velocity = velocity;
    response.current = current;
    response.voltage = voltage;
    response.motor_enabled = motor_enabled ? 1 : 0;
    response.calibrated = calibrated ? 1 : 0;

    sendData(&response, sizeof(response));

    if (DEBUG_COMM) {
        Serial.println("Sent STATUS response");
    }
}
