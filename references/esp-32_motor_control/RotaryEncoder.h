#pragma once
#include <Arduino.h>
#include "screens.h"

#define ENCODER_PIN_A 5      // CLK pin
#define ENCODER_PIN_B 4      // DT pin
#define ENCODER_BUTTON_PIN 3 // SW pin
#define LONG_PRESS_DURATION 800 // 800ms for long press

// Encoder sensitivity settings
#define ENCODER_FINE_SENSITIVITY 1       // For fine adjustments
#define ENCODER_COARSE_SENSITIVITY 3     // For coarse adjustments
#define ENCODER_JOG_STEP_MULTIPLIER 4    // Multiplier for steps per encoder tick in jog mode

// Navigation states
extern int8_t currentScreenIndex;
extern int8_t currentFocusIndex;
extern bool buttonPressed;
extern bool encoderJogMode;
extern volatile long encoderValue;
extern volatile long lastEncoderValue;
extern bool valueAdjustmentMode;
extern lv_obj_t *currentAdjustmentObject;
extern int adjustmentSensitivity;
extern bool fineAdjustmentMode;
extern volatile bool longPressDetected;
extern bool ultraFineAdjustmentMode;
extern int currentPositionBeingAdjusted;

// Function declarations - make sure these are declared here
void adjustValueByEncoder(lv_obj_t* obj, int delta);
void toggleAdjustmentPrecision();  // This is the function causing the conflict
void showModeChangeIndicator();
void setupEncoder();
void setupFocusableObjects();
void handleEncoder();
void navigateUI(int8_t direction);
void setFocus(lv_obj_t* obj);
void selectCurrentItem();
void setupFocusStyles();
void showPrecisionIndicator();
void resetPrecisionIndicator();