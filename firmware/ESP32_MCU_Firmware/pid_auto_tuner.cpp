#include "pid_auto_tuner.h"
#include "motor_control.h"  // For MT6701Sensor

// Initialize static constexpr arrays
constexpr float PIDAutoTuner::TEST_POSITIONS_DEG[];

PIDAutoTuner::PIDAutoTuner(BLDCMotor& motor, Sensor& encoder)
    : motor(motor), encoder(encoder),
      current_p(TUNING_INITIAL_P), current_i(TUNING_INITIAL_I),
      current_d(TUNING_INITIAL_D), current_ramp(TUNING_INITIAL_RAMP_DEG),
      optimal_p(TUNING_INITIAL_P), optimal_i(TUNING_INITIAL_I),
      optimal_d(TUNING_INITIAL_D), optimal_ramp(TUNING_INITIAL_RAMP_DEG),
      verbose_output(true) {

    final_metrics.valid = false;
}

void PIDAutoTuner::setPID(float p, float i, float d, float ramp_deg_s) {
    motor.P_angle.P = p;
    motor.P_angle.I = i;
    motor.P_angle.D = d;
    // SIMPLEFOC BOUNDARY: Convert ramp from deg/s to rad/s
    motor.P_angle.output_ramp = ramp_deg_s * (PI / 180.0f);

    current_p = p;
    current_i = i;
    current_d = d;
    current_ramp = ramp_deg_s;

    if (verbose_output) {
        Serial.print("[TUNE] Setting PID: P=");
        Serial.print(p, 2);
        Serial.print(", I=");
        Serial.print(i, 2);
        Serial.print(", D=");
        Serial.print(d, 3);
        Serial.print(", Ramp=");
        Serial.print(ramp_deg_s, 1);
        Serial.println("°/s");
    }

    // Wait for parameters to take effect
    delay(100);
}

bool PIDAutoTuner::moveAndAnalyze(float target_deg, TuningMetrics& metrics) {
    // Initialize metrics
    metrics.overshoot = 0;
    metrics.settling_time = 0;
    metrics.steady_state_error = 0;
    metrics.rise_time = 0;
    metrics.final_position = 0;
    metrics.score = 0;
    metrics.valid = false;

    // CRITICAL: Use ENCODER position, not SimpleFOC shaft_angle!
    // MT6701 absolute encoder is the source of truth for position
    encoder.update();
    float initial_pos_deg = ((MT6701Sensor&)encoder).getDegrees();
    float position_change_deg = target_deg - initial_pos_deg;

    // SIMPLEFOC BOUNDARY: Command movement in radians
    float target_rad = target_deg * (PI / 180.0f);
    motor.target = target_rad;

    unsigned long start_time = millis();
    unsigned long rise_time_ms = 0;
    unsigned long settling_time_ms = 0;
    float peak_overshoot = 0;
    bool reached_90_percent = false;
    bool is_settled = false;

    float target_90_deg = initial_pos_deg + 0.9f * position_change_deg;

    if (verbose_output) {
        Serial.print("[TUNE] Initial encoder: ");
        Serial.print(initial_pos_deg, 2);
        Serial.print("°, Target: ");
        Serial.print(target_deg, 2);
        Serial.print("°, Change: ");
        Serial.print(position_change_deg, 2);
        Serial.println("°");
    }

    // Track movement
    while (millis() - start_time < TUNING_MOVEMENT_TIMEOUT) {
        // Run FOC loop
        motor.loopFOC();
        motor.move();

        // CRITICAL: Use ENCODER position, not SimpleFOC shaft_angle!
        // MT6701 absolute encoder is the source of truth for position
        encoder.update();
        float current_pos_deg = ((MT6701Sensor&)encoder).getDegrees();
        float error_deg = abs(target_deg - current_pos_deg);

        // Calculate overshoot (in degrees)
        if (abs(position_change_deg) > 0.5f) {  // Avoid division by zero
            if (position_change_deg > 0 && current_pos_deg > target_deg) {
                float overshoot_deg = current_pos_deg - target_deg;
                if (overshoot_deg > peak_overshoot) {
                    peak_overshoot = overshoot_deg;
                }
            } else if (position_change_deg < 0 && current_pos_deg < target_deg) {
                float overshoot_deg = target_deg - current_pos_deg;
                if (overshoot_deg > peak_overshoot) {
                    peak_overshoot = overshoot_deg;
                }
            }
        }

        // Check for 90% rise time
        if (!reached_90_percent) {
            if (position_change_deg > 0 && current_pos_deg >= target_90_deg) {
                rise_time_ms = millis() - start_time;
                reached_90_percent = true;
            } else if (position_change_deg < 0 && current_pos_deg <= target_90_deg) {
                rise_time_ms = millis() - start_time;
                reached_90_percent = true;
            }
        }

        // Check if settled (using degrees)
        if (error_deg < TUNING_SETTLING_TOLERANCE_DEG) {
            if (!is_settled) {
                settling_time_ms = millis() - start_time;
                is_settled = true;

                // Wait for settling confirmation
                delay(TUNING_SETTLING_WINDOW);

                // Verify still settled
                motor.loopFOC();
                motor.move();
                encoder.update();
                current_pos_deg = ((MT6701Sensor&)encoder).getDegrees();
                error_deg = abs(target_deg - current_pos_deg);

                if (error_deg < TUNING_SETTLING_TOLERANCE_DEG) {
                    // Movement complete
                    metrics.final_position = current_pos_deg;
                    break;
                } else {
                    // Not truly settled, keep waiting
                    is_settled = false;
                }
            }
        } else {
            is_settled = false;
        }

        delayMicroseconds(100);  // Small delay to avoid overwhelming the motor
    }

    // Calculate final metrics (in degrees)
    metrics.overshoot = peak_overshoot;
    metrics.settling_time = settling_time_ms / 1000.0f;  // Convert to seconds
    metrics.rise_time = rise_time_ms / 1000.0f;

    // Get final position from ENCODER (source of truth)
    motor.loopFOC();
    motor.move();
    encoder.update();
    float final_pos_deg = ((MT6701Sensor&)encoder).getDegrees();
    metrics.final_position = final_pos_deg;
    metrics.steady_state_error = abs(target_deg - final_pos_deg);

    // Calculate score (lower is better)
    metrics.score = (
        metrics.overshoot * 10.0 +              // Heavily penalize overshoot
        metrics.settling_time * 2.0 +           // Penalize slow settling
        metrics.steady_state_error * 5.0 +      // Penalize steady-state error
        metrics.rise_time * 0.5                 // Small penalty for slow rise
    );

    // Check if movement timed out
    if (millis() - start_time >= TUNING_MOVEMENT_TIMEOUT) {
        if (verbose_output) {
            Serial.println("[TUNE] Movement timeout");
        }
        return false;
    }

    metrics.valid = true;
    return true;
}

bool PIDAutoTuner::evaluatePID(float p, float i, float d, float ramp_deg_s, float& avg_score) {
    // Set PID parameters
    setPID(p, i, d, ramp_deg_s);

    float total_score = 0;
    float total_overshoot = 0;
    int valid_tests = 0;

    // Test movements to different positions (in DEGREES)
    for (int idx = 0; idx < NUM_TEST_POSITIONS; idx++) {
        float target_deg = TEST_POSITIONS_DEG[idx];

        if (verbose_output) {
            Serial.print("[TUNE] Test ");
            Serial.print(idx + 1);
            Serial.print("/");
            Serial.print(NUM_TEST_POSITIONS);
            Serial.print(": target=");
            Serial.print(target_deg, 2);
            Serial.println("°");
        }

        TuningMetrics metrics;
        if (!moveAndAnalyze(target_deg, metrics)) {
            if (verbose_output) {
                Serial.println("[TUNE] Movement failed");
            }
            return false;
        }

        if (!metrics.valid) {
            return false;
        }

        // Check for excessive overshoot (safety threshold in degrees)
        if (metrics.overshoot > TUNING_MAX_OVERSHOOT_DEG) {
            if (verbose_output) {
                Serial.print("[TUNE] Excessive overshoot: ");
                Serial.print(metrics.overshoot, 2);
                Serial.println("°");
            }
            return false;
        }

        total_score += metrics.score;
        total_overshoot += metrics.overshoot;
        valid_tests++;

        if (verbose_output) {
            printMetrics(metrics, target_deg);
        }

        // Small delay between movements
        delay(500);
    }

    avg_score = total_score / valid_tests;

    if (verbose_output) {
        Serial.print("[TUNE] Average score: ");
        Serial.print(avg_score, 2);
        Serial.print(", Avg overshoot: ");
        Serial.print(total_overshoot / valid_tests, 2);
        Serial.println("°");
    }

    return true;
}

float PIDAutoTuner::tunePGain() {
    if (verbose_output) {
        printSeparator();
        Serial.println("[PHASE 1] Tuning Proportional Gain (P)");
        printSeparator();
    }

    float p = TUNING_INITIAL_P;
    float best_p = p;
    float best_score = 999999.0;

    while (p <= TUNING_MAX_P) {
        float score;
        if (!evaluatePID(p, 0, 0, current_ramp, score)) {
            // Hit overshoot limit or other issue, back off
            if (verbose_output) {
                Serial.print("[TUNE] Issue at P=");
                Serial.print(p, 2);
                Serial.println(", using previous best");
            }
            break;
        }

        if (score < best_score) {
            best_score = score;
            best_p = p;
            if (verbose_output) {
                Serial.print("[TUNE] New best P: ");
                Serial.print(best_p, 2);
                Serial.print(" (score: ");
                Serial.print(best_score, 2);
                Serial.println(")");
            }
        } else {
            // Score got worse
            if (verbose_output) {
                Serial.print("[TUNE] Score increasing at P=");
                Serial.print(p, 2);
                Serial.println(", using previous best");
            }
            break;
        }

        p += TUNING_P_STEP;
    }

    // Apply stability margin
    float optimal_p = best_p * TUNING_STABILITY_MARGIN;

    if (verbose_output) {
        Serial.print("[TUNE] Optimal P (with ");
        Serial.print((1.0 - TUNING_STABILITY_MARGIN) * 100, 0);
        Serial.print("% margin): ");
        Serial.println(optimal_p, 2);
    }

    return optimal_p;
}

float PIDAutoTuner::tuneDGain(float p) {
    if (verbose_output) {
        printSeparator();
        Serial.println("[PHASE 2] Tuning Derivative Gain (D)");
        printSeparator();
    }

    // Get baseline with P-only
    float baseline_score;
    if (!evaluatePID(p, 0, 0, current_ramp, baseline_score)) {
        if (verbose_output) {
            Serial.println("[TUNE] Failed to get baseline, skipping D tuning");
        }
        return 0.0;
    }

    float d = 0.0;
    float best_d = d;
    float best_score = baseline_score;

    while (d <= TUNING_MAX_D) {
        if (d == 0.0) {
            d += TUNING_D_STEP;
            continue;  // Already tested D=0
        }

        float score;
        if (!evaluatePID(p, 0, d, current_ramp, score)) {
            if (verbose_output) {
                Serial.print("[TUNE] Issue at D=");
                Serial.print(d, 3);
                Serial.println(", using previous best");
            }
            break;
        }

        // Need significant improvement (5%) to justify D gain
        if (score < best_score * 0.95) {
            best_score = score;
            best_d = d;
            if (verbose_output) {
                Serial.print("[TUNE] New best D: ");
                Serial.print(best_d, 3);
                Serial.print(" (score: ");
                Serial.print(best_score, 2);
                Serial.println(")");
            }
        } else {
            // D not helping anymore
            if (verbose_output) {
                Serial.print("[TUNE] D not improving at D=");
                Serial.println(d, 3);
            }
            break;
        }

        d += TUNING_D_STEP;
    }

    if (verbose_output) {
        Serial.print("[TUNE] Optimal D: ");
        Serial.println(best_d, 3);
    }

    return best_d;
}

float PIDAutoTuner::tuneIGain(float p, float d) {
    if (verbose_output) {
        printSeparator();
        Serial.println("[PHASE 3] Tuning Integral Gain (I)");
        printSeparator();
    }

    // Get baseline with PD
    float baseline_score;
    if (!evaluatePID(p, 0, d, current_ramp, baseline_score)) {
        if (verbose_output) {
            Serial.println("[TUNE] Failed to get baseline, skipping I tuning");
        }
        return 0.0;
    }

    float i = 0.0;
    float best_i = i;
    float best_score = baseline_score;

    while (i <= TUNING_MAX_I) {
        if (i == 0.0) {
            i += TUNING_I_STEP;
            continue;  // Already tested I=0
        }

        float score;
        if (!evaluatePID(p, i, d, current_ramp, score)) {
            if (verbose_output) {
                Serial.print("[TUNE] Issue at I=");
                Serial.print(i, 2);
                Serial.println(", using previous best");
            }
            break;
        }

        if (score < best_score) {
            best_score = score;
            best_i = i;
            if (verbose_output) {
                Serial.print("[TUNE] New best I: ");
                Serial.print(best_i, 2);
                Serial.print(" (score: ");
                Serial.print(best_score, 2);
                Serial.println(")");
            }
        } else {
            // Score got worse
            if (verbose_output) {
                Serial.print("[TUNE] I not improving at I=");
                Serial.println(i, 2);
            }
            break;
        }

        i += TUNING_I_STEP;
    }

    if (verbose_output) {
        Serial.print("[TUNE] Optimal I: ");
        Serial.println(best_i, 2);
    }

    return best_i;
}

bool PIDAutoTuner::runTuning(bool verbose) {
    verbose_output = verbose;

    if (verbose_output) {
        printSeparator();
        Serial.println("[TUNER] STARTING PID AUTO-TUNING");
        printSeparator();
        Serial.print("[TUNER] Test positions: ");
        for (int i = 0; i < NUM_TEST_POSITIONS; i++) {
            Serial.print(TEST_POSITIONS_DEG[i], 1);
            if (i < NUM_TEST_POSITIONS - 1) Serial.print(", ");
        }
        Serial.println("°");
        Serial.print("[TUNER] Max overshoot: ");
        Serial.print(TUNING_MAX_OVERSHOOT_DEG, 1);
        Serial.println("°");
        Serial.println();
    }

    // Phase 1: Tune P
    optimal_p = tunePGain();

    // Phase 2: Tune D
    optimal_d = tuneDGain(optimal_p);

    // Phase 3: Tune I
    optimal_i = tuneIGain(optimal_p, optimal_d);

    optimal_ramp = current_ramp;

    // Final evaluation
    if (verbose_output) {
        printSeparator();
        Serial.println("[TUNER] FINAL EVALUATION");
        printSeparator();
    }

    float final_score;
    if (!evaluatePID(optimal_p, optimal_i, optimal_d, optimal_ramp, final_score)) {
        if (verbose_output) {
            Serial.println("[TUNER] Final evaluation failed!");
        }
        return false;
    }

    // Store final metrics (use average from last evaluation)
    final_metrics.score = final_score;
    final_metrics.valid = true;

    // Print results
    if (verbose_output) {
        printSeparator();
        Serial.println("[TUNER] TUNING COMPLETE");
        printSeparator();
        Serial.println("Optimal PID Parameters:");
        Serial.print("  P = ");
        Serial.println(optimal_p, 2);
        Serial.print("  I = ");
        Serial.println(optimal_i, 2);
        Serial.print("  D = ");
        Serial.println(optimal_d, 3);
        Serial.print("  Ramp = ");
        Serial.println(optimal_ramp, 1);
        Serial.println();
        Serial.print("Final Score: ");
        Serial.println(final_score, 2);
        printSeparator();
    }

    return true;
}

void PIDAutoTuner::getOptimalPID(float& p, float& i, float& d, float& ramp_deg_s) {
    p = optimal_p;
    i = optimal_i;
    d = optimal_d;
    ramp_deg_s = optimal_ramp;
}

void PIDAutoTuner::printSeparator() {
    Serial.println("================================================================");
}

void PIDAutoTuner::printMetrics(const TuningMetrics& metrics, float target) {
    Serial.print("  Overshoot: ");
    Serial.print(metrics.overshoot, 2);
    Serial.print("°, Settle: ");
    Serial.print(metrics.settling_time, 2);
    Serial.print("s, Error: ");
    Serial.print(metrics.steady_state_error, 2);
    Serial.print("°, Score: ");
    Serial.println(metrics.score, 2);
}
