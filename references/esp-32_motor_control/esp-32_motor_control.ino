// Include all necessary libraries
#include <Arduino.h>
#include "Display_ST7789.h"
#include "LVGL_Driver.h"
#include "ui.h"
#include "screens.h"
#include "RotaryEncoder.h"

// Include our motor driver abstraction
#include "StepperDriver.h"
#include "L298NDriver.h"
#include "DRV8825Driver.h"
#include "TimerStepperControl.h"

//===============================================
// MOTOR CONFIGURATION
//===============================================
// Base motor characteristics
#define BASE_STEPS_PER_REVOLUTION 200    // Standard for NEMA 17 (1.8° per step)
#define DEFAULT_MICROSTEP_MODE 8         // Using 1/8 microstepping for smooth operation
float gearRatio = 5.0;                   // Default 5:1 gear ratio

//===============================================
// MOTION PARAMETERS
//===============================================
// Speed limits in RPM (at the output shaft after gearing)
#define MIN_RPM 0.1                      // Minimum speed in RPM
#define MAX_RPM 30.0                     // Maximum speed in RPM
#define DEFAULT_RPM 5.0                  // Default starting speed in RPM
#define RPM_FINE_ADJUST 0.1              // Fine adjustment increment in RPM
#define RPM_COARSE_ADJUST 1.0            // Coarse adjustment increment in RPM

// Step/rotation limits
#define MIN_ROTATION_PERCENT 1.0         // Minimum rotation (1% of a full turn)
#define MAX_ROTATION_PERCENT 1000.0      // Maximum rotation (10 full turns)
#define DEFAULT_ROTATION_PERCENT 100.0   // Default rotation (1 full turn)
#define ROTATION_FINE_ADJUST 1.0         // Fine adjustment increment (1% of rotation)
#define ROTATION_COARSE_ADJUST 5.0       // Coarse adjustment increment (5% of rotation)

// Acceleration and timing
#define DEFAULT_ACCELERATION 6400         // Steps per second per second
#define ACCEL_MIN 400 // minimum acceleration in settings
#define ACCEL_MAX 12800 // maximum acceleration in settings
#define MOTOR_IDLE_TIMEOUT_MS 5000       // 5 seconds before motor power saving

// Rotation Direction
#define INVERT_STEP_MODE_DIRECTION true       // Set to true to invert direction in steps mode
#define INVERT_CONTINUOUS_MODE_DIRECTION true // Set to true to invert direction in continuous mode
#define INVERT_JOG_MODE_DIRECTION false       // Set to true to invert direction in jog mode

//===============================================
// ENCODER AND UI SETTINGS
//===============================================
// Encoder sensitivity
#define ENCODER_FINE_SENSITIVITY 1       // For fine adjustments
#define ENCODER_COARSE_SENSITIVITY 3     // For coarse adjustments
#define ENCODER_JOG_STEP_MULTIPLIER 4    // Multiplier for steps per encoder tick in jog mode

//===============================================
// DRIVER CONFIGURATION
//===============================================
// Set this to choose which driver to use
#define USE_L298N_DRIVER  0  // Set to 1 to use L298N
#define USE_DRV8825_DRIVER 1 // Set to 1 to use DRV8825

// L298N Pin definitions
#define L298N_PIN1 1      // Connected to IN1 on L298N
#define L298N_PIN2 4      // Connected to IN2 on L298N
#define L298N_PIN3 19     // Connected to IN3 on L298N
#define L298N_PIN4 20     // Connected to IN4 on L298N
#define L298N_ENABLE_A 18 // Connected to ENA on L298N
#define L298N_ENABLE_B 23 // Connected to ENB on L298N

// DRV8825 Pin definitions
#define DRV8825_ENABLE_PIN 9  // Enable pin
#define DRV8825_M0_PIN 18      // Microstep mode 0
#define DRV8825_M1_PIN 19      // Microstep mode 1
#define DRV8825_M2_PIN 20       // Microstep mode 2
#define DRV8825_SLEEP_RESET_PIN 23 // GPIO23 controls both SLEEP and RESET
#define DRV8825_STEP_PIN 12        // STEP
#define DRV8825_DIR_PIN 13         // Direction
#define DRV8825_SLEEP_PIN -1   // Optional - connect if needed
#define DRV8825_RESET_PIN -1   // Optional - connect if needed
#define DRV8825_FAULT_PIN -1   // Optional - connect if needed

// Create the appropriate driver and controller
#if USE_L298N_DRIVER
    L298NDriver driver(L298N_PIN1, L298N_PIN2, L298N_PIN3, L298N_PIN4, L298N_ENABLE_A, L298N_ENABLE_B);
#elif USE_DRV8825_DRIVER
    DRV8825Driver driver(DRV8825_STEP_PIN, DRV8825_DIR_PIN, DRV8825_ENABLE_PIN, 
                         DRV8825_M0_PIN, DRV8825_M1_PIN, DRV8825_M2_PIN,
                         DRV8825_SLEEP_RESET_PIN);
#else
    #error "No driver selected! Set either USE_L298N_DRIVER or USE_DRV8825_DRIVER to 1"
#endif

//===============================================
// GLOBAL VARIABLES
//===============================================
// Create the timer-based controller with the selected driver
TimerStepperControl controller(&driver);

// Motor operation state
bool motorRunning = false;
bool continuousMode = false;
bool clockwiseDirection = true;
bool enableMotorPowerSave = true;
bool fineAdjustmentMode = true;          // Toggle between fine/coarse adjustment
bool ultraFineAdjustmentMode = false;  // For 0.01% precision adjustments

// Encoder jog state tracking
bool encoderJogMode = false;
long lastJogEncoderValue = 0;            // Track encoder position for jog mode
static long encoderValueAccumulator = 0;
static bool isFirstJogCheck = true;
static unsigned long jogModeEntryTime = 0;

// Current settings (these get converted to/from user-friendly units)
int targetSteps;                         // Target steps to move
int speedSetting;                        // Current speed in steps/sec
int accelerationSetting = DEFAULT_ACCELERATION; // Current acceleration in steps/sec²

// Timing variables
unsigned long lastMotorActivityTime = 0;

// UI state tracking
bool valueAdjustmentMode = false;        // Whether we're in value adjustment mode
lv_obj_t *currentAdjustmentObject = NULL; // Currently selected UI element for adjustment
int adjustmentSensitivity = ENCODER_FINE_SENSITIVITY; // How much to change per encoder tick

// Spinner references for each screen
lv_obj_t *move_steps_spinner = NULL;
lv_obj_t *manual_jog_spinner = NULL;
lv_obj_t *continuous_rotation_spinner = NULL;
lv_obj_t *sequence_spinner = NULL;
lv_obj_t *sequence_positions_spinner = NULL;

// Forward declarations of event handlers
void on_move_steps_start_clicked();
void on_move_steps_direction_clicked();
void on_move_steps_speed_clicked();
void on_move_steps_steps_clicked();
void on_manual_jog_start_clicked();
void on_manual_jog_speed_clicked();
void on_continuous_rotation_start_clicked();
void on_continuous_rotation_direction_clicked();
void on_continuous_rotation_speed_clicked();
void on_sequence_start_clicked();
void on_sequence_speed_clicked();
void on_sequence_direction_clicked();
void on_sequence_position_clicked(int position);
void on_settings_acceleration_clicked();
void on_settings_microstepping_clicked();
void on_back_clicked();

//===============================================
// MOTOR CONTROL FUNCTIONS
//===============================================
void startStepperMotion(int steps, bool clockwise, int speed) {
    // Ensure motor is in a clean state before starting
    if (motorRunning) {
        safelyStopAndResetMotor();
        delay(25); // Small delay to ensure reset is complete
    }
    
    // Wake the driver if it's using DRV8825
    #if USE_DRV8825_DRIVER
    controller.wake();
    #endif
    
    // Set direction and steps with inversion flag
    bool effectiveDirection = INVERT_STEP_MODE_DIRECTION ? !clockwise : clockwise;
    int relativeSteps = effectiveDirection ? steps : -steps;
    
    // Send command to move
    MotorCommand_t cmd;
    cmd.cmd_type = CMD_MOVE_STEPS;
    cmd.position = relativeSteps;
    cmd.speed = speed;
    cmd.direction = effectiveDirection;
    controller.sendCommand(&cmd);
    
    // Remember state
    continuousMode = false;
    motorRunning = true;
    lastMotorActivityTime = millis();
    
    // Update UI to reflect motor running state
    update_ui_labels();
    
    Serial.print("Starting stepper motion: ");
    Serial.print(steps);
    Serial.print(" steps, direction: ");
    Serial.print(clockwise ? "clockwise" : "counterclockwise");
    Serial.print(", speed: ");
    Serial.println(speed);
}

void startContinuousRotation(bool clockwise, int speed) {
    // Ensure motor is in a clean state before starting
    if (motorRunning) {
        safelyStopAndResetMotor();
        delay(25); // Small delay to ensure reset is complete
    }
    
    // Wake the driver if it's using DRV8825
    #if USE_DRV8825_DRIVER
    controller.wake();
    #endif
    
    // Apply direction inversion if configured
    bool effectiveDirection = INVERT_CONTINUOUS_MODE_DIRECTION ? !clockwise : clockwise;
    
    // Send command for continuous rotation
    MotorCommand_t cmd;
    cmd.cmd_type = CMD_START_CONTINUOUS;
    cmd.speed = speed;
    cmd.direction = effectiveDirection;
    controller.sendCommand(&cmd);
    
    // Remember state
    continuousMode = true;
    motorRunning = true;
    lastMotorActivityTime = millis();
    
    // Update UI to reflect motor running state
    update_ui_labels();
    
    Serial.print("Starting continuous rotation, direction: ");
    Serial.print(clockwise ? "clockwise" : "counterclockwise");
    Serial.print(", speed: ");
    Serial.println(speed);
}

void enterEncoderJogMode() {
    encoderJogMode = true;
    lastJogEncoderValue = encoderValue;  // Explicitly set this
    encoderValueAccumulator = 0;  // Reset accumulator
    Serial.println("Entered encoder jog mode");
}

void checkEncoderJogMode() {
    // Update last activity time
    lastMotorActivityTime = millis();
    static unsigned long lastEncoderUpdateTime = 0;
    
    // Initialize entry time on first call
    if (isFirstJogCheck) {
        jogModeEntryTime = millis();
        isFirstJogCheck = false;
        return; // Skip first check to allow mode to stabilize
    }
    
    // Add debounce period after entering jog mode
    if (millis() - jogModeEntryTime < 100) {
        return; // Wait 100ms before processing any encoder movements
    }

    // Exit jog mode if encoder button is pressed
    if (buttonPressed) {
        buttonPressed = false;
        encoderJogMode = false;
        motorRunning = false;
        stopMotor();
        update_ui_labels();
        Serial.println("Exiting encoder jog mode via button press");
        isFirstJogCheck = true; // Reset for next time
        return;
    }

    // Check for encoder movement
    if (encoderValue != lastJogEncoderValue) {
        // Calculate movement but accumulate it
        long delta = encoderValue - lastJogEncoderValue;
        
        // Only accumulate if not the first movement after entering jog mode
        if (lastJogEncoderValue != encoderValue) {
            encoderValueAccumulator += delta;
        }
        
        // Always update last encoder value
        lastJogEncoderValue = encoderValue;

        // Only process movements periodically to avoid command flooding
        unsigned long currentTime = millis();
        if (currentTime - lastEncoderUpdateTime >= 50) { // 50ms debounce
            lastEncoderUpdateTime = currentTime;

            // Only move if there's significant accumulated encoder change
            if (abs(encoderValueAccumulator) >= 1) {
                // Scale movement by sensitivity factor
                int moveSteps = encoderValueAccumulator * ENCODER_JOG_STEP_MULTIPLIER;

                // Apply direction setting from UI with inversion flag
                bool effectiveDirection = INVERT_JOG_MODE_DIRECTION ? !clockwiseDirection : clockwiseDirection;
                if (!effectiveDirection) {
                    moveSteps = -moveSteps;
                }

                // Reset accumulator
                encoderValueAccumulator = 0;

                // Send command to move steps with no acceleration
                MotorCommand_t cmd;
                cmd.cmd_type = CMD_MOVE_JOG;
                cmd.position = moveSteps;
                cmd.speed = speedSetting;
                controller.sendCommand(&cmd);

                Serial.print("Encoder jog: ");
                Serial.print(moveSteps);
                Serial.println(" steps");
            }
        }
    }
}

void stopMotor() {
    // Use our new safe stopping function
    safelyStopAndResetMotor();
    
    // Put driver to sleep to save power if power save is enabled
    if (enableMotorPowerSave) {
        #if USE_DRV8825_DRIVER
        controller.sleep();
        #endif
    }
    
    // Update UI to reflect motor stopped state
    update_ui_labels();
    
    Serial.println("Motor stopped");
}

void safelyStopAndResetMotor() {
    // First stop the motor
    MotorCommand_t cmd;
    cmd.cmd_type = CMD_STOP_MOTOR;
    controller.sendCommand(&cmd);
    
    // Wait for the command to be processed
    delay(25);
    
    // Now explicitly reset the controller state
    controller.resetMotorState();
    
    // Update state variables
    motorRunning = false;
    continuousMode = false;
    encoderJogMode = false;
}

// Helper functions for unit conversions
int getEffectiveStepsPerRevolution() {
    #if USE_DRV8825_DRIVER
    // Get current microstepping mode from the driver
    int microstepMode = driver.getMicrostepMode();
    return BASE_STEPS_PER_REVOLUTION * microstepMode;
    #else
    // For drivers without microstepping, use base steps
    return BASE_STEPS_PER_REVOLUTION;
    #endif
}

float stepsToRPM(int stepsPerSecond, float ratio) {
    // Convert steps/second to RPM at the output shaft
    int effectiveSteps = getEffectiveStepsPerRevolution();
    return (stepsPerSecond * 60.0) / (effectiveSteps * ratio);
}

float rpmToSteps(float rpm, float ratio) {
    // Convert RPM to steps/second
    int effectiveSteps = getEffectiveStepsPerRevolution();
    return (rpm * effectiveSteps * ratio) / 60.0f;
}

int safeRoundStepsPerSec(float stepsPerSec) {
    if (stepsPerSec <= 0) return 0;  // Handle zero/negative case
    return max(1, (int)round(stepsPerSec));  // Never round down to zero
}

float stepsToRotationPercent(int steps, float ratio) {
    // Convert steps to percentage of output shaft rotation
    int effectiveSteps = getEffectiveStepsPerRevolution();
    return (steps * 100.0) / (effectiveSteps * ratio);
}

int rotationPercentToSteps(float percent, float ratio) {
    // Convert percentage of rotation to steps
    int effectiveSteps = getEffectiveStepsPerRevolution();
    return (percent * effectiveSteps * ratio) / 100.0;
}

// Function to calculate maximum RPM based on current microstepping
float getMaxRpmForCurrentMicrostepping() {
    int microstepMode = driver.getMicrostepMode();
    float maxStepsPerSec = 4000.0f; // Based on 0.25ms timer
    int effectiveStepsPerRev = BASE_STEPS_PER_REVOLUTION * microstepMode * gearRatio;
    
    // Calculate theoretical max based on timer frequency
    float theoreticalMax = (maxStepsPerSec * 60.0f) / effectiveStepsPerRev;
    
    // Return the lower of the theoretical max or hardware MAX_RPM
    return min(theoreticalMax, (float)MAX_RPM);
}

//===============================================
// SEQUENCE MODE
//===============================================

// Define sequence data structure
typedef struct {
    float positions[5];        // Position values (0-4)
    bool initialDirection;     // Initial direction (true = clockwise)
    int speedSetting;          // Speed for the sequence
    bool isRunning;            // Whether sequence is currently running
    int currentStep;           // Current step in the sequence
    bool loopSequence;         // Whether to loop continuously
    float currentPosition;     // Current position of the motor
} SequenceData_t;

// Initialize with default values
SequenceData_t sequenceData = {
    .positions = {0, 25, 50, 75, 100},  // Default position values
    .initialDirection = true,           // Default clockwise
    .speedSetting = 0,                  // Will be set from current speed
    .isRunning = false,
    .currentStep = 0,
    .loopSequence = false,
    .currentPosition = 0.0      // Start at absolute position 0
};

// Sequence state tracking
int currentPositionBeingAdjusted = -1;  // -1 means none
bool lastMoveDirection = true;

// Function prototypes for sequence functions
void startSequence();

void stopSequence();

void moveToNextSequencePosition();

void updateSequencePositionLabels();

void onPositionChange() {
    // Print detailed debug information
    Serial.print("Current position: ");
    Serial.print(sequenceData.currentPosition);
    Serial.print(" (");
    Serial.print(fmod(sequenceData.currentPosition, 100.0));
    Serial.print("% + ");
    Serial.print((int)(sequenceData.currentPosition / 100.0));
    Serial.println(" rotations)");
    
    for (int i = 0; i < 5; i++) {
        Serial.print("Position ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(sequenceData.positions[i]);
        Serial.print(" (");
        Serial.print(fmod(sequenceData.positions[i], 100.0));
        Serial.print("% + ");
        Serial.print((int)(sequenceData.positions[i] / 100.0));
        Serial.println(" rotations)");
    }
}

// Motor control code
void moveToNextSequencePosition() {
    // Increment step counter
    sequenceData.currentStep++;
    
    // Check if we've reached the end
    if (sequenceData.currentStep >= 5) {
        if (sequenceData.loopSequence) {
            sequenceData.currentStep = 1; // Loop back to position 1
        } else {
            stopSequence();
            return;
        }
    }
    
    // Get current and target positions
    float currentPosition = sequenceData.currentPosition;
    float targetPosition = sequenceData.positions[sequenceData.currentStep];
    
    // Determine direction (alternating based on step)
    bool moveClockwise;
    if (sequenceData.currentStep % 2 == 1) {
        moveClockwise = sequenceData.initialDirection;
    } else {
        moveClockwise = !sequenceData.initialDirection;
    }
    
    // Extract normalized positions (0-99%)
    float currentNormalized = fmod(currentPosition, 100.0);
    if (currentNormalized < 0) currentNormalized += 100.0;
    
    float targetNormalized = fmod(targetPosition, 100.0);
    if (targetNormalized < 0) targetNormalized += 100.0;
    
    // Calculate the angular movement needed
    float angularMovement = 0;
    
    // For clockwise movement
    if (moveClockwise) {
        if (targetNormalized >= currentNormalized) {
            angularMovement = targetNormalized - currentNormalized;
        } else {
            angularMovement = (100.0 - currentNormalized) + targetNormalized;
        }
    } 
    // For counter-clockwise movement
    else {
        if (targetNormalized <= currentNormalized) {
            angularMovement = currentNormalized - targetNormalized;
        } else {
            angularMovement = currentNormalized + (100.0 - targetNormalized);
        }
    }
    
    // Extract rotation counts
    int targetRotations = floor(targetPosition / 100.0);
    float rotationMovement = targetRotations * 100.0;
    
    // Calculate total movement
    float totalMovement = angularMovement + rotationMovement;
    
    // Skip if no movement needed
    if (totalMovement < 0.1) {
        Serial.println("Already at target position - no movement needed");
        return;
    }
    
    // Calculate steps
    int stepsToMove = rotationPercentToSteps(totalMovement, gearRatio);
    
    // Execute movement
    MotorCommand_t cmd;
    cmd.cmd_type = CMD_MOVE_STEPS;
    cmd.position = moveClockwise ? -stepsToMove : stepsToMove;
    cmd.speed = sequenceData.speedSetting;
    controller.sendCommand(&cmd);
    
    // Update current position
    sequenceData.currentPosition = targetPosition;
    
    Serial.print("Moving ");
    Serial.print(totalMovement);
    Serial.print("% ");
    Serial.println(moveClockwise ? "CW" : "CCW");
}

void startSequence() {
    if (motorRunning) {
        stopMotor();
        return;
    }
    
    // Initialize sequence
    sequenceData.isRunning = true;
    sequenceData.currentStep = 0;  // Start with current step as 0
    sequenceData.speedSetting = speedSetting;
    sequenceData.currentPosition = sequenceData.positions[0];  // Start at position 0's value
    motorRunning = true;
    
    // Move to the first position in the sequence
    moveToNextSequencePosition();
    
    update_ui_labels();
}

// Stop sequence execution
void stopSequence() {
    sequenceData.isRunning = false;
    motorRunning = false;
    stopMotor();
    update_ui_labels();
}

// Update the display of sequence position values
void updateSequencePositionLabels() {
    lv_obj_t *posButtons[5] = {
        objects.sequence_position_0_button,
        objects.sequence_position_1_button,
        objects.sequence_position_2_button,
        objects.sequence_position_3_button,
        objects.sequence_position_4_button
    };
    
    for (int i = 0; i < 5; i++) {
        lv_obj_t *label = lv_obj_get_child(posButtons[i], 0);
        if (label) {
            char buffer[30];
            float value = sequenceData.positions[i];
            int rotations = (int)(value / 100.0);
            float normalizedPos = fmod(value, 100.0);
            
            // Format string based on rotation count
            if (rotations > 0) {
                if (ultraFineAdjustmentMode && currentPositionBeingAdjusted == i) {
                    snprintf(buffer, sizeof(buffer), "Pos %d: %.2f%% +%d rot.", 
                            i, normalizedPos, rotations);
                } else {
                    snprintf(buffer, sizeof(buffer), "Pos %d: %.1f%% +%d rot.", 
                            i, normalizedPos, rotations);
                }
            } else {
                if (ultraFineAdjustmentMode && currentPositionBeingAdjusted == i) {
                    snprintf(buffer, sizeof(buffer), "Pos %d: %.2f%%", 
                            i, normalizedPos);
                } else {
                    snprintf(buffer, sizeof(buffer), "Pos %d: %.1f%%", 
                            i, normalizedPos);
                }
            }
            
            lv_label_set_text(label, buffer);
        }
    }
}

//===============================================
// UI FUNCTIONS
//===============================================

// Function to adjust values (speed, steps) when in adjustment mode
void adjustValueByEncoder(lv_obj_t* obj, int delta) {
    // Determine sensitivity based on fine/coarse mode
    float speedAdjustment = fineAdjustmentMode ? RPM_FINE_ADJUST : RPM_COARSE_ADJUST;
    float rotationAdjustment = fineAdjustmentMode ? ROTATION_FINE_ADJUST : ROTATION_COARSE_ADJUST;
    float accelAdjustment = fineAdjustmentMode ? 10 : 50; // Fine: 10 steps/s², Coarse: 50 steps/s²

    
    // Apply proper magnitude based on delta and sensitivity
    float speedDelta = delta * speedAdjustment;
    float rotationDelta = delta * rotationAdjustment;
    int accelDelta = delta * accelAdjustment;
    
    // Check which value is being adjusted and modify accordingly
    if (obj == objects.step_num) {
        // Work with percentage of rotation instead of steps
        float currentPercent = stepsToRotationPercent(targetSteps, gearRatio);
        currentPercent += rotationDelta;
        
        // Apply bounds
        if (currentPercent < MIN_ROTATION_PERCENT) {
            currentPercent = MIN_ROTATION_PERCENT;
        } else if (currentPercent > MAX_ROTATION_PERCENT) {
            currentPercent = MAX_ROTATION_PERCENT;
        }
        
        // Convert back to steps
        targetSteps = rotationPercentToSteps(currentPercent, gearRatio);
        
        // Update the label
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Rot: %.1f%%", currentPercent);
        lv_obj_t *label = lv_obj_get_child(obj, 0);
        if (label) {
            lv_label_set_text(label, buffer);
        }
        
        Serial.print("Rotation adjusted to: ");
        Serial.print(currentPercent);
        Serial.println("%");
    }
        // Then modify the adjustValueByEncoder function:
        else if (obj == objects.speed || obj == objects.speed_manual_jog || 
            obj == objects.continuous_rotation_speed_button ||
            obj == objects.sequence_speed_button) {
            // Work with RPM instead of steps/second
            float currentRPM = stepsToRPM(speedSetting, gearRatio);

            // Apply delta to RPM
            currentRPM += speedDelta;

            // Apply bounds with dynamic max
            float dynamicMaxRpm = getMaxRpmForCurrentMicrostepping();
            if (currentRPM < MIN_RPM) {
            currentRPM = MIN_RPM;
            } else if (currentRPM > dynamicMaxRpm) {
            currentRPM = dynamicMaxRpm;
        }
        
        // Convert back to steps/second
        speedSetting = safeRoundStepsPerSec(rpmToSteps(currentRPM, gearRatio));
        
        // Update the label
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %.1f RPM", currentRPM);
        lv_obj_t *label = lv_obj_get_child(obj, 0);
        if (label) {
            lv_label_set_text(label, buffer);
        }
        
        // Update running speed if motor is already running in continuous mode
        if (motorRunning && continuousMode) {
            MotorCommand_t cmd;
            cmd.cmd_type = CMD_SET_SPEED;
            cmd.speed = speedSetting;
            controller.sendCommand(&cmd);
        }
    }

    // Acceleration adjustment
    else if (obj == objects.acceleration_button) {
        // Apply delta to acceleration
        accelerationSetting += accelDelta;
        
        // Apply bounds
        accelerationSetting = constrain(accelerationSetting, ACCEL_MIN, ACCEL_MAX);

        // Apply setting to controller immediately for consistent behavior
        controller.setAcceleration(accelerationSetting);
        
        // Update the label
        char buffer[30];
        snprintf(buffer, sizeof(buffer), "Accel: %d", accelerationSetting);
        lv_obj_t *label = lv_obj_get_child(obj, 0);
        if (label) {
            lv_label_set_text(label, buffer);
        }
    }

    // Microstepping adjustment
    else if (obj == objects.microstepping_button) {
        #if USE_DRV8825_DRIVER
        // Get current microstepping mode
        int currentMode = driver.getMicrostepMode();
        int newMode = currentMode;
        
        // For microstepping, we just want to cycle through modes on each significant encoder change
        // Only make changes when delta accumulates to a threshold
        static int accumulatedDelta = 0;
        accumulatedDelta += delta;
        
        // Threshold for changing microstepping (adjust as needed)
        const int MICROSTEP_CHANGE_THRESHOLD = 2;
        
        if (accumulatedDelta >= MICROSTEP_CHANGE_THRESHOLD) {
            // Cycle forward through modes
            switch(currentMode) {
                case 1: newMode = 2; break;
                case 2: newMode = 4; break;
                case 4: newMode = 8; break;
                case 8: newMode = 16; break;
                case 16: newMode = 32; break;
                case 32: newMode = 1; break;
                default: newMode = DEFAULT_MICROSTEP_MODE; break;
            }
            accumulatedDelta = 0;
        }
        else if (accumulatedDelta <= -MICROSTEP_CHANGE_THRESHOLD) {
            // Cycle backward through modes
            switch(currentMode) {
                case 1: newMode = 32; break;
                case 2: newMode = 1; break;
                case 4: newMode = 2; break;
                case 8: newMode = 4; break;
                case 16: newMode = 8; break;
                case 32: newMode = 16; break;
                default: newMode = DEFAULT_MICROSTEP_MODE; break;
            }
            accumulatedDelta = 0;
        }
        
        // Only update if we're actually changing the mode
        if (newMode != currentMode) {
            driver.setMicrostepMode(newMode);
            
            // Update the label
            char buffer[20];
            snprintf(buffer, sizeof(buffer), "Microstep: 1/%d", newMode);
            lv_obj_t *label = lv_obj_get_child(obj, 0);
            if (label) {
                lv_label_set_text(label, buffer);
            }
            
            Serial.print("Microstepping adjusted to 1/");
            Serial.println(newMode);
        }
        #endif
    }
    
    // Check if we're adjusting a sequence position
    else if (currentPositionBeingAdjusted >= 0 && currentPositionBeingAdjusted < 5) {
        // Determine adjustment size based on mode
        float adjustment;
        
        if (ultraFineAdjustmentMode) {
            adjustment = 0.01; // Ultra-fine: 0.01% increments
        } else if (fineAdjustmentMode) {
            adjustment = 1.0;  // Fine: 1% increments
        } else {
            adjustment = 5.0;  // Coarse: 5% increments
        }
        
        // Apply adjustment
        float newValue = sequenceData.positions[currentPositionBeingAdjusted] + 
                        (delta * adjustment);
        
        // Apply bounds - only constrain the lower bound to 0
        newValue = constrain(newValue, 0.0, 3600.0);  // Allow up to 36 rotations (3600%)
        
        // Store the new value
        sequenceData.positions[currentPositionBeingAdjusted] = newValue;
        
        // Format the position string based on rotation count
        char buffer[30];
        int rotations = (int)(newValue / 100.0);
        float normalizedPos = fmod(newValue, 100.0);
        
        // Display in normalized format
        if (rotations > 0) {
            if (ultraFineAdjustmentMode) {
                snprintf(buffer, sizeof(buffer), "Pos %d: %.2f%% +%d rot.", 
                        currentPositionBeingAdjusted, normalizedPos, rotations);
            } else {
                snprintf(buffer, sizeof(buffer), "Pos %d: %.1f%% +%d rot.", 
                        currentPositionBeingAdjusted, normalizedPos, rotations);
            }
        } else {
            if (ultraFineAdjustmentMode) {
                snprintf(buffer, sizeof(buffer), "Pos %d: %.2f%%", 
                        currentPositionBeingAdjusted, normalizedPos);
            } else {
                snprintf(buffer, sizeof(buffer), "Pos %d: %.1f%%", 
                        currentPositionBeingAdjusted, normalizedPos);
            }
        }
        
        // Update the button label
        lv_obj_t *posButton;
        switch(currentPositionBeingAdjusted) {
            case 0: posButton = objects.sequence_position_0_button; break;
            case 1: posButton = objects.sequence_position_1_button; break;
            case 2: posButton = objects.sequence_position_2_button; break;
            case 3: posButton = objects.sequence_position_3_button; break;
            case 4: posButton = objects.sequence_position_4_button; break;
            default: posButton = NULL; break;
        }
        
        if (posButton) {
            lv_obj_t *label = lv_obj_get_child(posButton, 0);
            if (label) {
                lv_label_set_text(label, buffer);
            }
        }
        
        Serial.print("Position ");
        Serial.print(currentPositionBeingAdjusted);
        Serial.print(" adjusted to: ");
        Serial.print(newValue);
        Serial.print(" (");
        Serial.print(normalizedPos);
        Serial.print("% +");
        Serial.print(rotations);
        Serial.println(" rot.)");
    }
}

// LVGL event handler function that routes to the appropriate handler
static void ui_event_handler(lv_event_t *e) {
    lv_obj_t *target = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        // Main Screen
        if (target == objects.move_steps) {
            loadScreen(SCREEN_ID_MOVE_STEPS_PAGE);
        } 
        else if (target == objects.manual_jog) {
            loadScreen(SCREEN_ID_MANUAL_JOG_PAGE);
        } 
        else if (target == objects.continuous) {
            loadScreen(SCREEN_ID_CONTINUOUS_ROTATION_PAGE);
        }
        else if (target == objects.auto_button) {
            loadScreen(SCREEN_ID_SEQUENCE_PAGE);
        }
        else if (target == objects.settings_button) {
            loadScreen(SCREEN_ID_SETTINGS_PAGE);
        }
        
        else if (target == objects.back) {
            on_back_clicked();
            loadScreen(SCREEN_ID_MAIN);
        }
        else if (target == objects.start) {
            on_move_steps_start_clicked();
        }
        else if (target == objects.step_num) {
            on_move_steps_steps_clicked();
        }
        else if (target == objects.clockwise) {
            on_move_steps_direction_clicked();
        }
        else if (target == objects.speed) {
            on_move_steps_speed_clicked();
        }
        
        // Manual Jog Page
        else if (target == objects.back_1) {
            on_back_clicked();
            loadScreen(SCREEN_ID_MAIN);
        }
        else if (target == objects.start_1) {
            on_manual_jog_start_clicked();
        }
        else if (target == objects.speed_manual_jog) {
            on_manual_jog_speed_clicked();
        }
        
        // Continuous Rotation Page
        else if (target == objects.back_2) {
            on_back_clicked();
            loadScreen(SCREEN_ID_MAIN);
        }
        else if (target == objects.continuous_rotation_start_button) {
            on_continuous_rotation_start_clicked();
        }
        else if (target == objects.continuous_rotation_direction_button) {
            on_continuous_rotation_direction_clicked();
        }
        else if (target == objects.continuous_rotation_speed_button) {
            on_continuous_rotation_speed_clicked();
        }
        
        // Sequence Page
        else if (target == objects.back_4) {
            on_back_clicked();
            loadScreen(SCREEN_ID_MAIN);
        }
        else if (target == objects.continuous_rotation_start_button_1) {
            on_sequence_start_clicked();
        }
        else if (target == objects.sequence_positions_button) {
            loadScreen(SCREEN_ID_SEQUENCE_POSITIONS_PAGE);
        }
        else if (target == objects.sequence_speed_button) {
            on_sequence_speed_clicked();
        }
        else if (target == objects.sequence_direction_button) {
            on_sequence_direction_clicked();
        }
        
        // Sequence Positions Page
        else if (target == objects.back_5) {
            on_back_clicked();
            loadScreen(SCREEN_ID_SEQUENCE_PAGE);
        }
        else if (target == objects.sequence_position_0_button) {
            on_sequence_position_clicked(0);
        }
        else if (target == objects.sequence_position_1_button) {
            on_sequence_position_clicked(1);
        }
        else if (target == objects.sequence_position_2_button) {
            on_sequence_position_clicked(2);
        }
        else if (target == objects.sequence_position_3_button) {
            on_sequence_position_clicked(3);
        }
        else if (target == objects.sequence_position_4_button) {
            on_sequence_position_clicked(4);
        }
        
        // Settings Page
        else if (target == objects.back_3) {
            on_back_clicked();
            loadScreen(SCREEN_ID_MAIN);
        }
        else if (target == objects.acceleration_button) {
            on_settings_acceleration_clicked();
        }
        else if (target == objects.microstepping_button) {
            on_settings_microstepping_clicked();
        }
    }
}

// Function to attach event handlers to UI elements
void attach_event_handlers() {
    // Main Screen
    lv_obj_add_event_cb(objects.move_steps, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.manual_jog, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.continuous, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.auto_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.settings_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    
    // Move Steps Page
    lv_obj_add_event_cb(objects.back, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.start, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.step_num, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.clockwise, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.speed, ui_event_handler, LV_EVENT_CLICKED, NULL);
    
    // Manual Jog Page
    lv_obj_add_event_cb(objects.back_1, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.start_1, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.speed_manual_jog, ui_event_handler, LV_EVENT_CLICKED, NULL);
    
    // Continuous Rotation Page
    lv_obj_add_event_cb(objects.back_2, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.continuous_rotation_start_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.continuous_rotation_direction_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.continuous_rotation_speed_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    
    // Sequence Page
    lv_obj_add_event_cb(objects.back_4, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.continuous_rotation_start_button_1, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.sequence_positions_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.sequence_speed_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.sequence_direction_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    
    // Sequence Positions Page
    lv_obj_add_event_cb(objects.back_5, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.sequence_position_0_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.sequence_position_1_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.sequence_position_2_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.sequence_position_3_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.sequence_position_4_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    
    // Settings Page
    lv_obj_add_event_cb(objects.back_3, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.acceleration_button, ui_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(objects.microstepping_button, ui_event_handler, LV_EVENT_CLICKED, NULL);

    // Find and store references to all spinners
    uint32_t child_count;

    // Find spinner on move steps screen
    child_count = lv_obj_get_child_cnt(objects.move_steps_page);
    for (uint32_t i = 0; i < child_count; i++) {
        lv_obj_t *child = lv_obj_get_child(objects.move_steps_page, i);
        if (lv_obj_check_type(child, &lv_spinner_class)) {
            move_steps_spinner = child;
            lv_obj_add_flag(move_steps_spinner, LV_OBJ_FLAG_HIDDEN);  // Initially hidden
            break;
        }
    }

    // Find spinner on manual jog screen
    child_count = lv_obj_get_child_cnt(objects.manual_jog_page);
    for (uint32_t i = 0; i < child_count; i++) {
        lv_obj_t *child = lv_obj_get_child(objects.manual_jog_page, i);
        if (lv_obj_check_type(child, &lv_spinner_class)) {
            manual_jog_spinner = child;
            lv_obj_add_flag(manual_jog_spinner, LV_OBJ_FLAG_HIDDEN);  // Initially hidden
            break;
        }
    }

    // Find spinner on continuous rotation screen
    child_count = lv_obj_get_child_cnt(objects.continuous_rotation_page);
    for (uint32_t i = 0; i < child_count; i++) {
        lv_obj_t *child = lv_obj_get_child(objects.continuous_rotation_page, i);
        if (lv_obj_check_type(child, &lv_spinner_class)) {
            continuous_rotation_spinner = child;
            lv_obj_add_flag(continuous_rotation_spinner, LV_OBJ_FLAG_HIDDEN);  // Initially hidden
            break;
        }
    }

    // Find spinner on sequence screen
    child_count = lv_obj_get_child_cnt(objects.sequence_page);
    for (uint32_t i = 0; i < child_count; i++) {
        lv_obj_t *child = lv_obj_get_child(objects.sequence_page, i);
        if (lv_obj_check_type(child, &lv_spinner_class)) {
            sequence_spinner = child;
            lv_obj_add_flag(sequence_spinner, LV_OBJ_FLAG_HIDDEN);  // Initially hidden
            break;
        }
    }

    // Find spinner on sequence positions screen
    child_count = lv_obj_get_child_cnt(objects.sequence_positions_page);
    for (uint32_t i = 0; i < child_count; i++) {
        lv_obj_t *child = lv_obj_get_child(objects.sequence_positions_page, i);
        if (lv_obj_check_type(child, &lv_spinner_class)) {
            sequence_positions_spinner = child;
            lv_obj_add_flag(sequence_positions_spinner, LV_OBJ_FLAG_HIDDEN);  // Initially hidden
            break;
        }
    }
}

// Function to update UI labels based on current settings
void update_ui_labels() {
    // Update direction button labels
    lv_obj_t *clockwise_label = lv_obj_get_child(objects.clockwise, 0);
    if (clockwise_label) {
        lv_label_set_text(clockwise_label, clockwiseDirection ? "Clockwise" : "Counter-CW");
    }
    
    lv_obj_t *direction_label = lv_obj_get_child(objects.continuous_rotation_direction_button, 0);
    if (direction_label) {
        lv_label_set_text(direction_label, clockwiseDirection ? "Clockwise" : "Counter-CW");
    }
    
    // Update step number button label - showing as % of rotation
    lv_obj_t *steps_label = lv_obj_get_child(objects.step_num, 0);
    if (steps_label) {
        char buffer[20];
        float percentRotation = stepsToRotationPercent(targetSteps, gearRatio);
        snprintf(buffer, sizeof(buffer), "Rot: %.1f%%", percentRotation);
        lv_label_set_text(steps_label, buffer);
    }
    
    // Calculate RPM from current speed setting
    float rpm = stepsToRPM(speedSetting, gearRatio);
    
    lv_obj_t *speed_label = lv_obj_get_child(objects.speed, 0);
    if (speed_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %.1f RPM", rpm);
        lv_label_set_text(speed_label, buffer);
    }
    
    lv_obj_t *speed_manual_label = lv_obj_get_child(objects.speed_manual_jog, 0);
    if (speed_manual_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %.1f RPM", rpm);
        lv_label_set_text(speed_manual_label, buffer);
    }
    
    lv_obj_t *speed_cont_label = lv_obj_get_child(objects.continuous_rotation_speed_button, 0);
    if (speed_cont_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %.1f RPM", rpm);
        lv_label_set_text(speed_cont_label, buffer);
    }
    
    lv_obj_t *seq_speed_label = lv_obj_get_child(objects.sequence_speed_button, 0);
    if (seq_speed_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Speed: %.1f RPM", rpm);
        lv_label_set_text(seq_speed_label, buffer);
    }

    lv_obj_t *start_label = lv_obj_get_child(objects.start, 0);
    if (start_label) {
        lv_label_set_text(start_label, motorRunning ? "Stop" : "Start");
    }
    
    lv_obj_t *start_manual_label = lv_obj_get_child(objects.start_1, 0);
    if (start_manual_label) {
        lv_label_set_text(start_manual_label, motorRunning ? "Stop" : "Start");
    }
    
    lv_obj_t *start_cont_label = lv_obj_get_child(objects.continuous_rotation_start_button, 0);
    if (start_cont_label) {
        lv_label_set_text(start_cont_label, motorRunning ? "Stop" : "Start");
    }

    lv_obj_t *accel_label = lv_obj_get_child(objects.acceleration_button, 0);
    if (accel_label) {
        char buffer[30];
        snprintf(buffer, sizeof(buffer), "Accel: %d", accelerationSetting);
        lv_label_set_text(accel_label, buffer);
    }

    // Update spinners based on motor state
    if (move_steps_spinner) {
        if (motorRunning) {
            lv_obj_clear_flag(move_steps_spinner, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(move_steps_spinner, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (manual_jog_spinner) {
        if (encoderJogMode) {
            lv_obj_clear_flag(manual_jog_spinner, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(manual_jog_spinner, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (continuous_rotation_spinner) {
        if (motorRunning) {
            lv_obj_clear_flag(continuous_rotation_spinner, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(continuous_rotation_spinner, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (sequence_spinner) {
        if (sequenceData.isRunning) {
            lv_obj_clear_flag(sequence_spinner, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(sequence_spinner, LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (sequence_positions_spinner) {
        if (sequenceData.isRunning) {
            lv_obj_clear_flag(sequence_positions_spinner, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(sequence_positions_spinner, LV_OBJ_FLAG_HIDDEN);
        }
    }

    #if USE_DRV8825_DRIVER
    lv_obj_t *microstepping_label = lv_obj_get_child(objects.microstepping_button, 0);
    if (microstepping_label) {
        int currentMode = driver.getMicrostepMode();
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Microstep: 1/%d", currentMode);
        lv_label_set_text(microstepping_label, buffer);
    }
    #endif

    // Update sequence direction button
    lv_obj_t *seq_dir_label = lv_obj_get_child(objects.sequence_direction_button, 0);
    if (seq_dir_label) {
        lv_label_set_text(seq_dir_label, sequenceData.initialDirection ? 
                          "Initial: CW" : "Initial: CCW");
    }
    
    // Update sequence positions
    updateSequencePositionLabels();
}

//===============================================
// UI EVENT HANDLERS (Connected to LVGL UI)
//===============================================

// Common back button handler
void on_back_clicked() {
    // Stop the motor when going back to main screen
    safelyStopAndResetMotor();
    
    // If coming from settings page, make sure to update values
    if (currentScreenIndex == 6) { // Settings page
        // Recalculate speed and steps based on current user-friendly values
        // This accounts for microstepping changes
        float currentRPM = stepsToRPM(speedSetting, gearRatio);
        float currentRotationPercent = stepsToRotationPercent(targetSteps, gearRatio);
        
        // Update internal values with new microstepping factor
        speedSetting = safeRoundStepsPerSec(rpmToSteps(currentRPM, gearRatio));
        targetSteps = rotationPercentToSteps(currentRotationPercent, gearRatio);
        
        // Update UI to reflect potentially changed values
        update_ui_labels();
    }
}

// Move Steps Functions
void on_move_steps_start_clicked() {
    // Toggle between start and stop
    if (motorRunning) {
        safelyStopAndResetMotor();
        update_ui_labels();
    } else {
        // Make sure we start from a clean state
        safelyStopAndResetMotor();
        delay(25); // Small delay to ensure reset is complete
        
        startStepperMotion(targetSteps, clockwiseDirection, speedSetting);
    }
}

void on_move_steps_direction_clicked() {
    clockwiseDirection = !clockwiseDirection;
    update_ui_labels();
    Serial.print("Direction changed to: ");
    Serial.println(clockwiseDirection ? "clockwise" : "counterclockwise");
}

void on_move_steps_speed_clicked() {
    // This is now handled in the value adjustment mode
    // But kept for compatibility if not using that mode
    if (speedSetting >= 400) {
        speedSetting = 100;
    } else {
        speedSetting += 100;
    }
    update_ui_labels();
    Serial.print("Speed set to: ");
    Serial.println(speedSetting);
}

void on_move_steps_steps_clicked() {
    // This is now handled in the value adjustment mode
    // But kept for compatibility if not using that mode
    if (targetSteps >= 1000) {
        targetSteps = 50;
    } else {
        targetSteps *= 2;
    }
    update_ui_labels();
    Serial.print("Steps set to: ");
    Serial.println(targetSteps);
}

// Manual Jog Functions
void on_manual_jog_start_clicked() {
    // If already in encoder jog mode, exit it
    if (encoderJogMode) {
        safelyStopAndResetMotor();
        update_ui_labels();
        return;
    }
    
    // If motor is running in any other mode, stop it
    if (motorRunning) {
        safelyStopAndResetMotor();
        update_ui_labels();
        delay(50); // Ensure complete stop before continuing
        return;
    }
    
    // Make sure we start from a clean state
    safelyStopAndResetMotor();
    
    // Now set up jog mode with fresh state
    encoderJogMode = true;
    motorRunning = true;
    lastJogEncoderValue = encoderValue;
    encoderValueAccumulator = 0;
    
    // Update UI
    lv_obj_t *start_manual_label = lv_obj_get_child(objects.start_1, 0);
    if (start_manual_label) {
        lv_label_set_text(start_manual_label, "Jogging...");
    }
    
    // Enable the motor for jogging
    MotorCommand_t cmd;
    cmd.cmd_type = CMD_START_JOG;
    cmd.speed = speedSetting;
    controller.sendCommand(&cmd);
    
    Serial.println("Entering encoder jog mode");
}

void on_manual_jog_speed_clicked() {
    // This is now handled in the value adjustment mode
    // But kept for compatibility if not using that mode
    if (speedSetting >= 400) {
        speedSetting = 100;
    } else {
        speedSetting += 100;
    }
    update_ui_labels();
    Serial.print("Speed set to: ");
    Serial.println(speedSetting);
    
    // Update running speed if motor is already running
    if (motorRunning && continuousMode) {
        MotorCommand_t cmd;
        cmd.cmd_type = CMD_SET_SPEED;
        cmd.speed = speedSetting;
        controller.sendCommand(&cmd);
    }
}

// Continuous Rotation Mode Functions
void on_continuous_rotation_start_clicked() {
    // Toggle between start and stop
    if (motorRunning) {
        safelyStopAndResetMotor();
        update_ui_labels();
    } else {
        // Make sure we start from a clean state
        safelyStopAndResetMotor();
        delay(25); // Small delay to ensure reset is complete
        
        startContinuousRotation(clockwiseDirection, speedSetting);
    }
}

void on_continuous_rotation_direction_clicked() {
    clockwiseDirection = !clockwiseDirection;
    update_ui_labels();
    Serial.print("Direction changed to: ");
    Serial.println(clockwiseDirection ? "clockwise" : "counterclockwise");
    
    // Update running direction if motor is already running
    if (motorRunning && continuousMode) {
        // Stop current movement
        MotorCommand_t stopCmd;
        stopCmd.cmd_type = CMD_STOP_MOTOR;
        controller.sendCommand(&stopCmd);
        
        // Start with new direction
        MotorCommand_t startCmd;
        startCmd.cmd_type = CMD_START_CONTINUOUS;
        startCmd.direction = clockwiseDirection;
        startCmd.speed = speedSetting;
        controller.sendCommand(&startCmd);
    }
}

void on_continuous_rotation_speed_clicked() {
    // This is now handled in the value adjustment mode
    // But kept for compatibility if not using that mode
    if (speedSetting >= 400) {
        speedSetting = 100;
    } else {
        speedSetting += 100;
    }
    update_ui_labels();
    Serial.print("Speed set to: ");
    Serial.println(speedSetting);
    
    // Update running speed if motor is already running
    if (motorRunning && continuousMode) {
        MotorCommand_t cmd;
        cmd.cmd_type = CMD_SET_SPEED;
        cmd.speed = speedSetting;
        controller.sendCommand(&cmd);
    }
}

// Sequence Mode Functions
void on_sequence_start_clicked() {
    if (sequenceData.isRunning) {
        stopSequence();
    } else {
        // Make sure we start from a clean state
        safelyStopAndResetMotor();
        delay(25); // Small delay to ensure reset is complete
        
        startSequence();
    }
}

void on_sequence_speed_clicked() {
    // For now, using a simple toggling behavior like the other screens
    if (speedSetting >= 400) {
        speedSetting = 100;
    } else {
        speedSetting += 100;
    }
    update_ui_labels();
}

void on_sequence_direction_clicked() {
    sequenceData.initialDirection = !sequenceData.initialDirection;
    update_ui_labels();
}

void on_sequence_position_clicked(int position) {
    // Log more information to help debug
    Serial.print("Position ");
    Serial.print(position);
    Serial.println(" clicked"); 
  
    // If already in adjustment mode, exit it
    if (valueAdjustmentMode && currentPositionBeingAdjusted == position) {
      Serial.println("Exiting adjustment mode");
      valueAdjustmentMode = false;
      currentPositionBeingAdjusted = -1;
      currentAdjustmentObject = NULL;
      
      // Restore original style to the button
      lv_obj_t *posButton;
      switch(position) {
          case 0: posButton = objects.sequence_position_0_button; break;
          case 1: posButton = objects.sequence_position_1_button; break;
          case 2: posButton = objects.sequence_position_2_button; break;
          case 3: posButton = objects.sequence_position_3_button; break;
          case 4: posButton = objects.sequence_position_4_button; break;
          default: posButton = NULL; break;
      }
      lv_obj_set_style_bg_color(posButton, lv_color_hex(0x656565), LV_PART_MAIN | LV_STATE_DEFAULT);
      return;
    }
    
    // Enter adjustment mode for this position
    Serial.println("Entering adjustment mode");
    valueAdjustmentMode = true;
    currentPositionBeingAdjusted = position;
    
    // Determine which button we're adjusting
    lv_obj_t *posButton;
    switch(position) {
      case 0: posButton = objects.sequence_position_0_button; break;
      case 1: posButton = objects.sequence_position_1_button; break;
      case 2: posButton = objects.sequence_position_2_button; break;
      case 3: posButton = objects.sequence_position_3_button; break;
      case 4: posButton = objects.sequence_position_4_button; break;
      default: posButton = NULL; break;
    }
    
    // Set the current adjustment object
    currentAdjustmentObject = posButton;
    
    // Highlight the button
    lv_obj_set_style_bg_color(posButton, lv_color_hex(0x2196F3), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    // For sequence positions, always use fine mode initially
    fineAdjustmentMode = true;
    ultraFineAdjustmentMode = false;
    
    // Format the position string based on rotation count
    char buffer[30];
    float value = sequenceData.positions[position];
    int rotations = (int)(value / 100.0);
    float normalizedPos = fmod(value, 100.0);
    
    // Display in normalized format
    if (rotations > 0) {
      snprintf(buffer, sizeof(buffer), "Pos %d: %.1f%% +%d rot.", 
              position, normalizedPos, rotations);
    } else {
      snprintf(buffer, sizeof(buffer), "Pos %d: %.1f%%", 
              position, normalizedPos);
    }
    
    lv_obj_t *label = lv_obj_get_child(posButton, 0);
    if (label) {
      lv_label_set_text(label, buffer);
    }
    
    Serial.print("Adjusting sequence position ");
    Serial.println(position);
  }

// Settings button handlers
void on_settings_acceleration_clicked() {
    // If we're already in adjustment mode, exit it
    if (valueAdjustmentMode && currentAdjustmentObject == objects.acceleration_button) {
        valueAdjustmentMode = false;
        currentAdjustmentObject = NULL;
        
        // Restore original style
        lv_obj_set_style_bg_color(objects.acceleration_button, lv_color_hex(0x656565), LV_PART_MAIN | LV_STATE_DEFAULT);
        
        // Apply the acceleration setting directly to the controller
        controller.setAcceleration(accelerationSetting);
        return;
    }
    
    // Get current acceleration from the controller
    accelerationSetting = controller.getAcceleration();
    
    // Enter adjustment mode
    valueAdjustmentMode = true;
    currentAdjustmentObject = objects.acceleration_button;
    
    // Highlight the button
    lv_obj_set_style_bg_color(objects.acceleration_button, lv_color_hex(0x2196F3), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    // Update the button text
    char buffer[30];
    snprintf(buffer, sizeof(buffer), "Accel: %d", accelerationSetting);
    lv_obj_t *label = lv_obj_get_child(objects.acceleration_button, 0);
    if (label) {
        lv_label_set_text(label, buffer);
    }
}

void update_microstepping_label() {
    // Only applicable for DRV8825 driver
    #if USE_DRV8825_DRIVER
    // Get current microstepping mode
    int currentMode = driver.getMicrostepMode();
    
    // Update the label on the microstepping button
    lv_obj_t *microstepping_label = lv_obj_get_child(objects.microstepping_button, 0);
    if (microstepping_label) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "Microstep: 1/%d", currentMode);
        lv_label_set_text(microstepping_label, buffer);
    }
    #endif
}

void on_settings_microstepping_clicked() {
    // The actual adjustment is now handled through the encoder in value adjustment mode
    // This is just needed for direct button click handling if not using the encoder
    #if USE_DRV8825_DRIVER
    if (!valueAdjustmentMode) {  // Only act on direct click if not in adjustment mode
        int currentMode = driver.getMicrostepMode();
        int newMode;
        
        // Cycle through microstepping modes
        switch(currentMode) {
            case 1: newMode = 2; break;
            case 2: newMode = 4; break;
            case 4: newMode = 8; break;
            case 8: newMode = 16; break;
            case 16: newMode = 32; break;
            case 32: default: newMode = DEFAULT_MICROSTEP_MODE; break;
        }
        
        driver.setMicrostepMode(newMode);
        update_ui_labels();
    }
    #endif
}

//===============================================
// SETUP & LOOP
//===============================================
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Initialize the display and UI
    LCD_Init();
    Set_Backlight(50); // Set LCD backlight to 50%
    Lvgl_Init();
    ui_init();
    update_ui_labels();

    // Attach event handlers to UI elements
    attach_event_handlers();

    // Initialize the rotary encoder
    setupEncoder();

    // Initialize our timer-based motor controller
    controller.init();
    
    // Set microstepping mode for DRV8825 if used
    #if USE_DRV8825_DRIVER
    driver.setMicrostepMode(DEFAULT_MICROSTEP_MODE);
    // Explicitly wake the driver
    controller.wake();
    #endif

    // Convert RPM to steps/sec for initial values
    speedSetting = safeRoundStepsPerSec(rpmToSteps(DEFAULT_RPM, gearRatio));
    targetSteps = rotationPercentToSteps(DEFAULT_ROTATION_PERCENT, gearRatio);

    // Set acceleration
    MotorCommand_t cmd;
    cmd.cmd_type = CMD_SET_ACCELERATION;
    cmd.acceleration = accelerationSetting;
    controller.sendCommand(&cmd);
    
    Serial.println("System ready!");
}

void loop() {
    // Current time tracking
    unsigned long currentMillis = millis();
    
    // Handle UI updates
    Timer_Loop();
    ui_tick();
    
    // Handle encoder input (includes UI navigation and value adjustment)
    handleEncoder();
    
    // Check if a long press was detected for toggling fine/coarse adjustment
    if (longPressDetected) {
        longPressDetected = false;
        
        // Only toggle if in value adjustment mode
        if (valueAdjustmentMode) {
            toggleAdjustmentPrecision();
        }
    }
    
    if (motorRunning && encoderJogMode) {
        checkEncoderJogMode();
    }

    // Check if sequence is running and motor has stopped (completed a step)
    if (sequenceData.isRunning && !controller.isRunning() && 
        sequenceData.currentStep > 0 && sequenceData.currentStep < 5) {
        // Short delay to ensure the motor is really stopped
        delay(50);
        moveToNextSequencePosition();
    }
    
    // Check for motor idle timeout - automatic shutdown after inactivity
    if (enableMotorPowerSave && motorRunning && 
        !encoderJogMode && !continuousMode && 
        controller.isRunning() == false &&
        currentMillis - lastMotorActivityTime > MOTOR_IDLE_TIMEOUT_MS) {
        // Motor has been idle too long, disable it
        motorRunning = false;
        controller.sleep();
        update_ui_labels();
    }

    // Poll for motor status updates (completed movements)
    if (motorRunning && !encoderJogMode && !controller.isRunning()) {
        motorRunning = false;
        Serial.println("Motor stopped (reached target)");
        update_ui_labels();
    }
}