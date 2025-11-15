#ifndef PID_AUTO_TUNER_H
#define PID_AUTO_TUNER_H

#include <Arduino.h>
#include <SimpleFOC.h>

//=============================================================================
// PID AUTO-TUNER FOR MOTOR POSITION CONTROL
//=============================================================================

/**
 * Auto-tuner for motor PID position control.
 *
 * This class automatically finds optimal PID parameters for motor position
 * tracking by starting with conservative values and gradually increasing
 * gains while preventing overshoot.
 *
 * Algorithm:
 * 1. Start with low P gain to avoid overshoot
 * 2. Test motor response at multiple positions
 * 3. Incrementally increase P until good response (with safety margin)
 * 4. Add D gain to reduce overshoot
 * 5. Add I gain to eliminate steady-state error
 *
 * Usage:
 *   PIDAutoTuner tuner(motor, encoder);
 *   if (tuner.runTuning()) {
 *     float p, i, d, ramp;
 *     tuner.getOptimalPID(p, i, d, ramp);
 *     motor.P_angle.P = p;
 *     motor.P_angle.I = i;
 *     motor.P_angle.D = d;
 *   }
 */
class PIDAutoTuner {
public:
    // Tuning parameters - OPTIMIZED FOR GIMBAL MOTORS (using DEGREES)
    // Test positions chosen to stay within ±20° for 45° total range application
    static constexpr float TEST_POSITIONS_DEG[] = {0.0, 10.0, -10.0, 5.0, 0.0};  // degrees - smaller movements
    static constexpr int NUM_TEST_POSITIONS = 5;
    static constexpr float TUNING_POSITION_TOLERANCE_DEG = 3.0;      // degrees
    static constexpr float TUNING_SETTLING_TOLERANCE_DEG = 1.0;      // degrees
    static constexpr unsigned long TUNING_MOVEMENT_TIMEOUT = 15000;  // ms - increased from 20s for more responsive tuning
    static constexpr unsigned long TUNING_SETTLING_WINDOW = 500;     // ms

    // Initial PID values for GIMBAL MOTORS - Based on research (P=0.5-2.0 typical)
    // Starting at 1.0 based on SimpleFOC gimbal motor examples and research
    static constexpr float TUNING_INITIAL_P = 1.0;   // Higher starting point based on research
    static constexpr float TUNING_INITIAL_I = 0.0;
    static constexpr float TUNING_INITIAL_D = 0.0;
    static constexpr float TUNING_INITIAL_RAMP_DEG = 1000.0;  // deg/s

    // PID increment steps - tuned for gimbal motors
    static constexpr float TUNING_P_STEP = 0.5;      // Larger steps since we're starting higher
    static constexpr float TUNING_D_STEP = 0.02;     // Fine D adjustment
    static constexpr float TUNING_I_STEP = 0.05;     // Fine I adjustment

    // Tuning limits - based on gimbal motor research
    static constexpr float TUNING_MAX_P = 20.0;      // Typically won't exceed 10-12 for gimbal
    static constexpr float TUNING_MAX_I = 2.0;       // Lower I limit for gimbal motors
    static constexpr float TUNING_MAX_D = 1.0;       // Lower D limit

    // Performance thresholds - in DEGREES
    static constexpr float TUNING_MAX_OVERSHOOT_DEG = 3.0;           // degrees - reasonable for gimbal
    static constexpr float TUNING_MAX_SETTLING_TIME = 3.0;           // seconds - faster response expected
    static constexpr float TUNING_TARGET_STEADY_STATE_ERROR_DEG = 0.5; // degrees
    static constexpr float TUNING_STABILITY_MARGIN = 0.8;            // Use 80% of best P

    // Performance metrics
    struct TuningMetrics {
        float overshoot;
        float settling_time;
        float steady_state_error;
        float rise_time;
        float final_position;
        float score;
        bool valid;
    };

    /**
     * Constructor.
     *
     * @param motor Reference to SimpleFOC BLDCMotor (works in RADIANS)
     * @param encoder Reference to encoder sensor
     *
     * NOTE: This tuner works in DEGREES internally but communicates with
     * SimpleFOC (which uses radians) through conversion functions.
     */
    PIDAutoTuner(BLDCMotor& motor, Sensor& encoder);

    /**
     * Run complete PID auto-tuning sequence.
     *
     * @param verbose Print detailed progress to Serial
     * @return true if tuning succeeded, false if failed
     */
    bool runTuning(bool verbose = true);

    /**
     * Get optimal PID parameters from tuning.
     *
     * @param p Output: Proportional gain
     * @param i Output: Integral gain
     * @param d Output: Derivative gain
     * @param ramp_deg_s Output: Ramp limit (degrees/second)
     */
    void getOptimalPID(float& p, float& i, float& d, float& ramp_deg_s);

    /**
     * Get tuning performance metrics.
     */
    TuningMetrics getMetrics() { return final_metrics; }

private:
    BLDCMotor& motor;
    Sensor& encoder;

    // Current PID values
    float current_p;
    float current_i;
    float current_d;
    float current_ramp;

    // Optimal PID values
    float optimal_p;
    float optimal_i;
    float optimal_d;
    float optimal_ramp;

    // Final metrics
    TuningMetrics final_metrics;

    bool verbose_output;

    /**
     * Set PID parameters and wait for them to take effect.
     */
    void setPID(float p, float i, float d, float ramp);

    /**
     * Move to target position and track performance.
     *
     * @param target Target position in radians
     * @param metrics Output: Performance metrics
     * @return true if movement completed, false if timeout
     */
    bool moveAndAnalyze(float target, TuningMetrics& metrics);

    /**
     * Evaluate PID parameters by testing multiple positions.
     *
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @param ramp Ramp limit
     * @param avg_score Output: Average performance score
     * @return true if evaluation succeeded, false if overshoot or error
     */
    bool evaluatePID(float p, float i, float d, float ramp, float& avg_score);

    /**
     * Tune P gain (Phase 1).
     *
     * @return Optimal P gain
     */
    float tunePGain();

    /**
     * Tune D gain (Phase 2).
     *
     * @param p Fixed P gain from Phase 1
     * @return Optimal D gain
     */
    float tuneDGain(float p);

    /**
     * Tune I gain (Phase 3).
     *
     * @param p Fixed P gain from Phase 1
     * @param d Fixed D gain from Phase 2
     * @return Optimal I gain
     */
    float tuneIGain(float p, float d);

    /**
     * Print separator line to Serial.
     */
    void printSeparator();

    /**
     * Print metrics to Serial.
     */
    void printMetrics(const TuningMetrics& metrics, float target);
};

#endif // PID_AUTO_TUNER_H
