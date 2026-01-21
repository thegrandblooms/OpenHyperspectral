#ifndef NVS_STORAGE_H
#define NVS_STORAGE_H

#include <Arduino.h>
#include <Preferences.h>

//=============================================================================
// NVS Storage for Motor Calibration and PID Parameters
//=============================================================================
// Uses ESP32's Non-Volatile Storage (NVS) to persist:
// - Motor calibration data (zero_electric_angle, sensor_direction)
// - PID parameters for position, velocity, and current control
// - Calibration validity flag
//
// Data is stored in the "motor" namespace and survives power cycles.
//=============================================================================

// NVS namespace and keys
#define NVS_NAMESPACE "motor"
#define NVS_KEY_VALID "valid"
#define NVS_KEY_VERSION "version"
#define NVS_KEY_ZERO_ANGLE "zero_angle"
#define NVS_KEY_SENSOR_DIR "sensor_dir"
#define NVS_KEY_POS_P "pos_p"
#define NVS_KEY_POS_I "pos_i"
#define NVS_KEY_POS_D "pos_d"
#define NVS_KEY_POS_RAMP "pos_ramp"
#define NVS_KEY_VEL_P "vel_p"
#define NVS_KEY_VEL_I "vel_i"
#define NVS_KEY_VEL_D "vel_d"
#define NVS_KEY_VEL_RAMP "vel_ramp"

// Current storage version (increment when format changes)
#define NVS_STORAGE_VERSION 1

// Magic value to indicate valid data
#define NVS_VALID_MAGIC 0xCAFE

/**
 * Calibration data structure
 */
struct CalibrationData {
    float zero_electric_angle;  // Motor electrical zero offset
    int sensor_direction;       // 1 = CW, -1 = CCW

    // Position PID
    float pos_p;
    float pos_i;
    float pos_d;
    float pos_ramp;

    // Velocity PID
    float vel_p;
    float vel_i;
    float vel_d;
    float vel_ramp;
};

/**
 * NVS Storage Manager
 *
 * Handles persistent storage of motor calibration and PID parameters.
 */
class NVSStorage {
public:
    NVSStorage();

    /**
     * Initialize NVS storage
     * Call once during setup()
     * @return true if successful
     */
    bool begin();

    /**
     * Check if valid calibration data exists in NVS
     * @return true if valid data is stored
     */
    bool hasValidData();

    /**
     * Load calibration data from NVS
     * @param data Output structure to fill
     * @return true if data was loaded successfully
     */
    bool load(CalibrationData& data);

    /**
     * Save calibration data to NVS
     * @param data Data to save
     * @return true if data was saved successfully
     */
    bool save(const CalibrationData& data);

    /**
     * Clear all stored calibration data
     * Forces recalibration on next boot
     * @return true if cleared successfully
     */
    bool clear();

    /**
     * Print stored data to Serial (for debugging)
     */
    void printStoredData();

private:
    Preferences prefs;
    bool initialized;
};

// Global instance
extern NVSStorage nvsStorage;

#endif // NVS_STORAGE_H
