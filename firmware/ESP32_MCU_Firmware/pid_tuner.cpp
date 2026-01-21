#include "pid_tuner.h"
#include "motor_control.h"
#include "config.h"

// Static buffer definitions (allocated once in global memory, not on stack)
float PIDAutoTuner::position_buffer[TUNE_BUFFER_SIZE];
unsigned long PIDAutoTuner::time_buffer[TUNE_BUFFER_SIZE];

PIDAutoTuner::PIDAutoTuner(BLDCMotor& motor, MT6701Sensor& sensor)
    : motor(motor), sensor(sensor), sample_count(0) {
    // Initialize with defaults
    best_params = {PID_P_POSITION, PID_I_POSITION, PID_D_POSITION, PID_RAMP_POSITION_DEG};
    memset(&best_metrics, 0, sizeof(best_metrics));
}

bool PIDAutoTuner::runTuning(bool verbose) {
    if (verbose) {
        Serial.println("");
        Serial.println("================================================================");
        Serial.println("         PID AUTO-TUNING (Firmware-based)");
        Serial.println("================================================================");
        Serial.println("");
        Serial.print("Step size: ");
        Serial.print(TUNE_STEP_SIZE_DEG, 1);
        Serial.print("deg, Max overshoot: ");
        Serial.print(TUNE_MAX_OVERSHOOT_DEG, 1);
        Serial.println("deg");
        Serial.println("");
    }

    // Make sure motor is enabled
    if (!motor.enabled) {
        motor.enable();
        delay(200);
    }

    // Phase 1: Tune P
    if (verbose) {
        Serial.println("================================================================");
        Serial.println("[PHASE 1] Tuning Proportional Gain (P)");
        Serial.println("================================================================");
    }

    if (!tuneP(verbose)) {
        if (verbose) {
            Serial.println("P tuning failed!");
        }
        return false;
    }

    // Phase 2: Tune D
    if (verbose) {
        Serial.println("");
        Serial.println("================================================================");
        Serial.println("[PHASE 2] Tuning Derivative Gain (D)");
        Serial.println("================================================================");
    }

    if (!tuneD(verbose)) {
        if (verbose) {
            Serial.println("D tuning failed (continuing with D=0)");
        }
    }

    // Phase 3: Tune I
    if (verbose) {
        Serial.println("");
        Serial.println("================================================================");
        Serial.println("[PHASE 3] Tuning Integral Gain (I)");
        Serial.println("================================================================");
    }

    if (!tuneI(verbose)) {
        if (verbose) {
            Serial.println("I tuning failed (continuing with I=0)");
        }
    }

    // Apply final parameters
    applyPID(best_params.p, best_params.i, best_params.d, best_params.ramp);

    if (verbose) {
        Serial.println("");
        Serial.println("================================================================");
        Serial.println("                    TUNING COMPLETE");
        Serial.println("================================================================");
        Serial.println("");
        Serial.println("Optimal PID Parameters:");
        Serial.print("  P = ");
        Serial.println(best_params.p, 2);
        Serial.print("  I = ");
        Serial.println(best_params.i, 3);
        Serial.print("  D = ");
        Serial.println(best_params.d, 4);
        Serial.print("  Ramp = ");
        Serial.print(best_params.ramp, 1);
        Serial.println(" deg/s");
        Serial.println("");
        Serial.println("Final Metrics:");
        Serial.print("  Rise Time: ");
        Serial.print(best_metrics.rise_time_ms, 1);
        Serial.println(" ms");
        Serial.print("  Overshoot: ");
        Serial.print(best_metrics.overshoot_deg, 2);
        Serial.println(" deg");
        Serial.print("  Settling Time: ");
        Serial.print(best_metrics.settling_time_ms, 1);
        Serial.println(" ms");
        Serial.print("  Steady-State Error: ");
        Serial.print(best_metrics.steady_state_error, 3);
        Serial.println(" deg");
        Serial.print("  Score: ");
        Serial.println(best_metrics.score(), 2);
        Serial.println("");
        Serial.println("To update config.h:");
        Serial.print("  #define PID_P_POSITION   ");
        Serial.println(best_params.p, 2);
        Serial.print("  #define PID_I_POSITION   ");
        Serial.println(best_params.i, 3);
        Serial.print("  #define PID_D_POSITION   ");
        Serial.println(best_params.d, 4);
        Serial.println("================================================================");
    }

    return true;
}

void PIDAutoTuner::getOptimalPID(float& p, float& i, float& d, float& ramp) {
    p = best_params.p;
    i = best_params.i;
    d = best_params.d;
    ramp = best_params.ramp;
}

bool PIDAutoTuner::tuneP(bool verbose) {
    float best_p = TUNE_P_START;
    float best_score = 999999.0f;
    bool found_good_p = false;

    for (float p = TUNE_P_START; p <= TUNE_P_MAX; p += TUNE_P_STEP) {
        if (verbose) {
            Serial.print("Testing P=");
            Serial.print(p, 1);
            Serial.print("... ");
        }

        applyPID(p, 0.0f, 0.0f, PID_RAMP_POSITION_DEG);
        delay(100);

        StepMetrics metrics = runStepTest(TUNE_STEP_SIZE_DEG, false);

        if (verbose) {
            Serial.print("OS=");
            Serial.print(metrics.overshoot_deg, 2);
            Serial.print("deg, Settle=");
            Serial.print(metrics.settling_time_ms, 0);
            Serial.print("ms, Err=");
            Serial.print(metrics.steady_state_error, 3);
            Serial.print("deg, Score=");
            Serial.println(metrics.score(), 2);
        }

        // Check if overshoot exceeded threshold
        if (metrics.overshoot_deg > TUNE_MAX_OVERSHOOT_DEG) {
            if (verbose) {
                Serial.println("  Overshoot exceeded threshold, stopping P search");
            }
            break;
        }

        // Track best score
        float score = metrics.score();
        if (score < best_score) {
            best_score = score;
            best_p = p;
            best_metrics = metrics;
            found_good_p = true;
        }

        // Return to start position
        delay(500);
    }

    if (!found_good_p) {
        best_p = TUNE_P_START;
    }

    // Apply stability margin
    best_params.p = best_p * TUNE_P_MARGIN;
    best_params.i = 0.0f;
    best_params.d = 0.0f;
    best_params.ramp = PID_RAMP_POSITION_DEG;

    if (verbose) {
        Serial.print("\nBest P: ");
        Serial.print(best_p, 2);
        Serial.print(" -> Using ");
        Serial.print(best_params.p, 2);
        Serial.println(" (with 20% stability margin)");
    }

    return found_good_p;
}

bool PIDAutoTuner::tuneD(bool verbose) {
    float best_d = 0.0f;
    float best_score = best_metrics.score();
    bool found_better = false;

    for (float d = TUNE_D_START; d <= TUNE_D_MAX; d += TUNE_D_STEP) {
        if (d == 0.0f) continue;  // Skip D=0, already tested

        if (verbose) {
            Serial.print("Testing D=");
            Serial.print(d, 2);
            Serial.print("... ");
        }

        applyPID(best_params.p, 0.0f, d, best_params.ramp);
        delay(100);

        StepMetrics metrics = runStepTest(TUNE_STEP_SIZE_DEG, false);

        if (verbose) {
            Serial.print("OS=");
            Serial.print(metrics.overshoot_deg, 2);
            Serial.print("deg, Settle=");
            Serial.print(metrics.settling_time_ms, 0);
            Serial.print("ms, Score=");
            Serial.println(metrics.score(), 2);
        }

        float score = metrics.score();

        // D should reduce overshoot
        if (score < best_score && metrics.overshoot_deg < best_metrics.overshoot_deg + 1.0f) {
            best_score = score;
            best_d = d;
            best_metrics = metrics;
            found_better = true;
        }

        // If score gets worse by more than 10%, stop searching
        if (score > best_score * 1.1f && found_better) {
            if (verbose) {
                Serial.println("  Score degrading, stopping D search");
            }
            break;
        }

        delay(500);
    }

    best_params.d = best_d;

    if (verbose) {
        Serial.print("\nBest D: ");
        Serial.println(best_params.d, 3);
    }

    return found_better;
}

bool PIDAutoTuner::tuneI(bool verbose) {
    // Only add I if there's significant steady-state error
    if (best_metrics.steady_state_error < TUNE_TARGET_ERROR_DEG) {
        if (verbose) {
            Serial.print("Steady-state error (");
            Serial.print(best_metrics.steady_state_error, 3);
            Serial.println("deg) is acceptable, skipping I tuning");
        }
        return true;
    }

    float best_i = 0.0f;
    float best_score = best_metrics.score();
    bool found_better = false;

    for (float i = TUNE_I_STEP; i <= TUNE_I_MAX; i += TUNE_I_STEP) {
        if (verbose) {
            Serial.print("Testing I=");
            Serial.print(i, 2);
            Serial.print("... ");
        }

        applyPID(best_params.p, i, best_params.d, best_params.ramp);
        delay(100);

        StepMetrics metrics = runStepTest(TUNE_STEP_SIZE_DEG, false);

        if (verbose) {
            Serial.print("OS=");
            Serial.print(metrics.overshoot_deg, 2);
            Serial.print("deg, Err=");
            Serial.print(metrics.steady_state_error, 3);
            Serial.print("deg, Score=");
            Serial.println(metrics.score(), 2);
        }

        // I can increase overshoot - be careful
        if (metrics.overshoot_deg > TUNE_MAX_OVERSHOOT_DEG * 1.5f) {
            if (verbose) {
                Serial.println("  Overshoot too high, stopping I search");
            }
            break;
        }

        float score = metrics.score();

        if (score < best_score) {
            best_score = score;
            best_i = i;
            best_metrics = metrics;
            found_better = true;
        }

        // Stop if error is good enough
        if (metrics.steady_state_error < TUNE_TARGET_ERROR_DEG) {
            if (verbose) {
                Serial.println("  Target error achieved!");
            }
            break;
        }

        delay(500);
    }

    best_params.i = best_i;

    if (verbose) {
        Serial.print("\nBest I: ");
        Serial.println(best_params.i, 3);
    }

    return found_better;
}

StepMetrics PIDAutoTuner::runStepTest(float step_size_deg, bool verbose) {
    StepMetrics metrics;
    memset(&metrics, 0, sizeof(metrics));

    // Get starting position from encoder (0-360°)
    sensor.update();
    float start_pos = sensor.getDegrees();
    float target_pos = normalizeAngle(start_pos + step_size_deg);

    // CRITICAL: Calculate target relative to motor.shaft_angle, not absolute
    // motor.shaft_angle can be any value (e.g., -302.3°), not just 0-360°
    // We need to normalize the target to be within ±π of current position
    float current_rad = motor.shaft_angle;
    float target_deg_normalized = target_pos;

    // Convert target to radians
    float target_rad_absolute = target_deg_normalized * DEG_TO_RAD;

    // Normalize current angle to 0-2π for comparison
    float current_rad_normalized = fmod(current_rad, TWO_PI);
    if (current_rad_normalized < 0) current_rad_normalized += TWO_PI;

    // Calculate error in radians (shortest path)
    float error_rad = target_rad_absolute - current_rad_normalized;
    while (error_rad > PI) error_rad -= TWO_PI;
    while (error_rad < -PI) error_rad += TWO_PI;

    // Target is current position + error (this stays in motor's coordinate system)
    float target_rad = current_rad + error_rad;

    // Sample position
    sample_count = 0;
    unsigned long start_time = millis();
    unsigned long sample_time = start_time;

    while (millis() - start_time < TUNE_SETTLE_TIME_MS && sample_count < TUNE_BUFFER_SIZE) {
        // Run FOC loop
        motor.loopFOC();
        motor.move(target_rad);

        // Sample every TUNE_SAMPLE_INTERVAL_MS
        if (millis() >= sample_time) {
            sensor.update();
            position_buffer[sample_count] = sensor.getDegrees();
            time_buffer[sample_count] = millis() - start_time;
            sample_count++;
            sample_time += TUNE_SAMPLE_INTERVAL_MS;
        }

        delayMicroseconds(100);  // ~10kHz FOC loop
    }

    // Analyze the response
    analyzeResponse(start_pos, target_pos, metrics);

    return metrics;
}

void PIDAutoTuner::analyzeResponse(float start_pos, float target_pos, StepMetrics& metrics) {
    if (sample_count < 10) {
        return;
    }

    float step_size = target_pos - start_pos;
    // Handle wraparound
    if (step_size > 180.0f) step_size -= 360.0f;
    if (step_size < -180.0f) step_size += 360.0f;

    int direction = (step_size > 0) ? 1 : -1;

    // Find rise time (10% to 90%)
    float threshold_10 = start_pos + step_size * 0.1f;
    float threshold_90 = start_pos + step_size * 0.9f;
    int idx_10 = -1, idx_90 = -1;

    for (int i = 0; i < sample_count; i++) {
        float pos = position_buffer[i];
        float progress = (pos - start_pos) / step_size;

        if (idx_10 < 0 && progress >= 0.1f) {
            idx_10 = i;
        }
        if (idx_90 < 0 && progress >= 0.9f) {
            idx_90 = i;
            break;
        }
    }

    if (idx_10 >= 0 && idx_90 >= 0) {
        metrics.rise_time_ms = time_buffer[idx_90] - time_buffer[idx_10];
    }

    // Find peak and overshoot
    float peak_pos = start_pos;
    int peak_idx = 0;

    for (int i = 0; i < sample_count; i++) {
        float pos = position_buffer[i];
        if (direction > 0) {
            if (pos > peak_pos) {
                peak_pos = pos;
                peak_idx = i;
            }
        } else {
            if (pos < peak_pos) {
                peak_pos = pos;
                peak_idx = i;
            }
        }
    }

    metrics.peak_time_ms = time_buffer[peak_idx];

    // Calculate overshoot (how far past target)
    float overshoot = (peak_pos - target_pos) * direction;
    metrics.overshoot_deg = (overshoot > 0) ? overshoot : 0.0f;

    // Find settling time (last time outside tolerance band)
    int settled_idx = sample_count - 1;
    for (int i = sample_count - 1; i >= 0; i--) {
        float error = abs(position_buffer[i] - target_pos);
        // Handle wraparound
        if (error > 180.0f) error = 360.0f - error;

        if (error > TUNE_SETTLING_TOL_DEG) {
            settled_idx = i + 1;
            break;
        }
    }

    if (settled_idx < sample_count) {
        metrics.settling_time_ms = time_buffer[settled_idx];
    } else {
        metrics.settling_time_ms = time_buffer[sample_count - 1];
    }

    // Calculate steady-state error (average of last 100ms)
    int window_start = sample_count - 100;
    if (window_start < 0) window_start = 0;

    float sum_error = 0;
    int count = 0;
    for (int i = window_start; i < sample_count; i++) {
        float error = abs(position_buffer[i] - target_pos);
        if (error > 180.0f) error = 360.0f - error;
        sum_error += error;
        count++;
    }

    metrics.steady_state_error = (count > 0) ? (sum_error / count) : 0.0f;
}

void PIDAutoTuner::applyPID(float p, float i, float d, float ramp) {
    // Convert ramp from deg/s to rad/s for SimpleFOC
    float ramp_rad = ramp * DEG_TO_RAD;

    motor.P_angle.P = p;
    motor.P_angle.I = i;
    motor.P_angle.D = d;
    motor.P_angle.output_ramp = ramp_rad;
}

float PIDAutoTuner::normalizeAngle(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}
