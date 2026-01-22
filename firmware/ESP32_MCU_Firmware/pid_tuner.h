#ifndef PID_TUNER_H
#define PID_TUNER_H

#include <Arduino.h>
#include <SimpleFOC.h>

//=============================================================================
// Firmware-based PID Auto-Tuner
//=============================================================================
// Runs step response tests at high sampling rate (1kHz) to find optimal
// PID parameters. Much more accurate than Python-based tuning due to
// no serial latency.
//
// Algorithm:
// 1. Start with conservative P gain, I=0, D=0
// 2. Run step response test, measure overshoot and settling
// 3. Increase P until overshoot exceeds threshold
// 4. Add D to reduce overshoot
// 5. Add small I to eliminate steady-state error
// 6. Save optimal parameters to NVS
//=============================================================================

// Forward declarations
class BLDCMotor;
class MT6701Sensor;

//=============================================================================
// TUNING CONFIGURATION
//=============================================================================

// Step test parameters
#define TUNE_STEP_SIZE_DEG      30.0f    // Step size for testing (degrees)
#define TUNE_SETTLE_TIME_MS     3000     // Time to wait for settling (ms)

// Tolerances
#define TUNE_POSITION_TOL_DEG   0.5f     // Target reached threshold
#define TUNE_SETTLING_TOL_DEG   1.0f     // For settling time calculation
#define TUNE_MAX_OVERSHOOT_DEG  5.0f     // Maximum acceptable overshoot

// P tuning
#define TUNE_P_START            5.0f     // Starting P value
#define TUNE_P_STEP             5.0f     // P increment
#define TUNE_P_MAX              50.0f    // Maximum P to try

// D tuning
#define TUNE_D_START            0.0f     // Starting D value
#define TUNE_D_STEP             0.1f     // D increment
#define TUNE_D_MAX              2.0f     // Maximum D to try

// I tuning
#define TUNE_I_START            0.0f     // Starting I value
#define TUNE_I_STEP             0.1f     // I increment
#define TUNE_I_MAX              2.0f     // Maximum I to try

// Stability margins
#define TUNE_P_MARGIN           0.8f     // Use 80% of best P (stability margin)
#define TUNE_TARGET_ERROR_DEG   0.2f     // Target steady-state error

// Sample buffer size - REDUCED to avoid stack overflow
// 500 samples at 10ms interval = 5 seconds of data (plenty for step response)
#define TUNE_BUFFER_SIZE        500
#define TUNE_SAMPLE_INTERVAL_MS 10       // Sample every 10ms (100Hz) - changed from 1ms

//=============================================================================
// DATA STRUCTURES
//=============================================================================

/**
 * Step response metrics
 */
struct StepMetrics {
    float rise_time_ms;         // Time to reach 90% of target
    float overshoot_deg;        // Peak overshoot
    float settling_time_ms;     // Time to settle within tolerance
    float steady_state_error;   // Final error
    float peak_time_ms;         // Time to reach peak

    // Composite score (lower is better)
    float score() const {
        return overshoot_deg * 10.0f +
               settling_time_ms * 0.002f +
               steady_state_error * 5.0f +
               rise_time_ms * 0.001f;
    }
};

/**
 * PID parameters
 */
struct PIDParams {
    float p;
    float i;
    float d;
    float ramp;
};

//=============================================================================
// PID AUTO-TUNER CLASS
//=============================================================================

class PIDAutoTuner {
public:
    /**
     * Constructor
     * @param motor Reference to BLDCMotor
     * @param sensor Reference to MT6701Sensor
     */
    PIDAutoTuner(BLDCMotor& motor, MT6701Sensor& sensor);

    /**
     * Run full auto-tuning sequence
     * @param verbose Print progress to Serial
     * @return true if tuning successful
     */
    bool runTuning(bool verbose = true);

    /**
     * Get optimal PID parameters after tuning
     */
    void getOptimalPID(float& p, float& i, float& d, float& ramp);

    /**
     * Get tuning metrics
     */
    const StepMetrics& getMetrics() const { return best_metrics; }

private:
    BLDCMotor& motor;
    MT6701Sensor& sensor;

    // Results
    PIDParams best_params;
    StepMetrics best_metrics;

    // Sample buffer - STATIC to avoid stack overflow
    // These are allocated once in global memory, not on stack
    static float position_buffer[TUNE_BUFFER_SIZE];
    static unsigned long time_buffer[TUNE_BUFFER_SIZE];
    int sample_count;

    // Internal methods
    bool tuneP(bool verbose);
    bool tuneD(bool verbose);
    bool tuneI(bool verbose);

    StepMetrics runStepTest(float step_size_deg, bool verbose);
    void applyPID(float p, float i, float d, float ramp);
    void analyzeResponse(float start_pos, float target_pos, StepMetrics& metrics);

    float normalizeAngle(float angle);
};

#endif // PID_TUNER_H
