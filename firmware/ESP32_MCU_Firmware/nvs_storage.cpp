#include "nvs_storage.h"
#include "config.h"

// Global instance
NVSStorage nvsStorage;

NVSStorage::NVSStorage() : initialized(false) {
}

bool NVSStorage::begin() {
    if (initialized) {
        return true;
    }

    // Open NVS namespace in read-write mode
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        if (DEBUG_MOTOR) {
            Serial.println("[NVS] Failed to open NVS namespace");
        }
        return false;
    }

    initialized = true;

    if (DEBUG_MOTOR) {
        Serial.println("[NVS] Storage initialized");
    }

    return true;
}

bool NVSStorage::hasValidData() {
    if (!initialized) {
        return false;
    }

    // Check magic value
    uint16_t magic = prefs.getUShort(NVS_KEY_VALID, 0);
    if (magic != NVS_VALID_MAGIC) {
        return false;
    }

    // Check version
    uint8_t version = prefs.getUChar(NVS_KEY_VERSION, 0);
    if (version != NVS_STORAGE_VERSION) {
        if (DEBUG_MOTOR) {
            Serial.print("[NVS] Version mismatch: stored=");
            Serial.print(version);
            Serial.print(", expected=");
            Serial.println(NVS_STORAGE_VERSION);
        }
        return false;
    }

    return true;
}

bool NVSStorage::load(CalibrationData& data) {
    if (!initialized || !hasValidData()) {
        return false;
    }

    // Load calibration data
    data.zero_electric_angle = prefs.getFloat(NVS_KEY_ZERO_ANGLE, 0.0f);
    data.sensor_direction = prefs.getInt(NVS_KEY_SENSOR_DIR, 1);

    // Load position PID
    data.pos_p = prefs.getFloat(NVS_KEY_POS_P, PID_P_POSITION);
    data.pos_i = prefs.getFloat(NVS_KEY_POS_I, PID_I_POSITION);
    data.pos_d = prefs.getFloat(NVS_KEY_POS_D, PID_D_POSITION);
    data.pos_ramp = prefs.getFloat(NVS_KEY_POS_RAMP, PID_RAMP_POSITION_DEG);

    // Load velocity PID
    data.vel_p = prefs.getFloat(NVS_KEY_VEL_P, PID_P_VELOCITY);
    data.vel_i = prefs.getFloat(NVS_KEY_VEL_I, PID_I_VELOCITY);
    data.vel_d = prefs.getFloat(NVS_KEY_VEL_D, PID_D_VELOCITY);
    data.vel_ramp = prefs.getFloat(NVS_KEY_VEL_RAMP, PID_RAMP_VELOCITY);

    if (DEBUG_MOTOR) {
        Serial.println("[NVS] Calibration data loaded");
    }

    return true;
}

bool NVSStorage::save(const CalibrationData& data) {
    if (!initialized) {
        return false;
    }

    // Save calibration data
    prefs.putFloat(NVS_KEY_ZERO_ANGLE, data.zero_electric_angle);
    prefs.putInt(NVS_KEY_SENSOR_DIR, data.sensor_direction);

    // Save position PID
    prefs.putFloat(NVS_KEY_POS_P, data.pos_p);
    prefs.putFloat(NVS_KEY_POS_I, data.pos_i);
    prefs.putFloat(NVS_KEY_POS_D, data.pos_d);
    prefs.putFloat(NVS_KEY_POS_RAMP, data.pos_ramp);

    // Save velocity PID
    prefs.putFloat(NVS_KEY_VEL_P, data.vel_p);
    prefs.putFloat(NVS_KEY_VEL_I, data.vel_i);
    prefs.putFloat(NVS_KEY_VEL_D, data.vel_d);
    prefs.putFloat(NVS_KEY_VEL_RAMP, data.vel_ramp);

    // Mark as valid
    prefs.putUShort(NVS_KEY_VALID, NVS_VALID_MAGIC);
    prefs.putUChar(NVS_KEY_VERSION, NVS_STORAGE_VERSION);

    if (DEBUG_MOTOR) {
        Serial.println("[NVS] Calibration data saved");
    }

    return true;
}

bool NVSStorage::clear() {
    if (!initialized) {
        return false;
    }

    // Clear the valid flag (effectively invalidates all data)
    prefs.putUShort(NVS_KEY_VALID, 0);

    if (DEBUG_MOTOR) {
        Serial.println("[NVS] Calibration data cleared");
    }

    return true;
}

void NVSStorage::printStoredData() {
    if (!initialized) {
        Serial.println("[NVS] Not initialized");
        return;
    }

    Serial.println("\n=== NVS Stored Data ===");

    uint16_t magic = prefs.getUShort(NVS_KEY_VALID, 0);
    uint8_t version = prefs.getUChar(NVS_KEY_VERSION, 0);

    Serial.print("Valid: ");
    Serial.println(magic == NVS_VALID_MAGIC ? "YES" : "NO");
    Serial.print("Version: ");
    Serial.println(version);

    if (magic == NVS_VALID_MAGIC) {
        Serial.println("\nCalibration:");
        Serial.print("  zero_electric_angle: ");
        Serial.print(prefs.getFloat(NVS_KEY_ZERO_ANGLE, 0.0f) * 180.0f / PI, 2);
        Serial.println("deg");
        Serial.print("  sensor_direction: ");
        Serial.println(prefs.getInt(NVS_KEY_SENSOR_DIR, 1) == 1 ? "CW" : "CCW");

        Serial.println("\nPosition PID:");
        Serial.print("  P=");
        Serial.print(prefs.getFloat(NVS_KEY_POS_P, 0), 2);
        Serial.print(" I=");
        Serial.print(prefs.getFloat(NVS_KEY_POS_I, 0), 3);
        Serial.print(" D=");
        Serial.print(prefs.getFloat(NVS_KEY_POS_D, 0), 4);
        Serial.print(" Ramp=");
        Serial.println(prefs.getFloat(NVS_KEY_POS_RAMP, 0), 1);

        Serial.println("\nVelocity PID:");
        Serial.print("  P=");
        Serial.print(prefs.getFloat(NVS_KEY_VEL_P, 0), 2);
        Serial.print(" I=");
        Serial.print(prefs.getFloat(NVS_KEY_VEL_I, 0), 2);
        Serial.print(" D=");
        Serial.print(prefs.getFloat(NVS_KEY_VEL_D, 0), 4);
        Serial.print(" Ramp=");
        Serial.println(prefs.getFloat(NVS_KEY_VEL_RAMP, 0), 1);
    }

    Serial.println("=======================\n");
}
