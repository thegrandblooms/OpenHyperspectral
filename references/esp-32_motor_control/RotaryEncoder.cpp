#include "RotaryEncoder.h"
#include "ui.h"
#include "screens.h"
#include "LVGL_Driver.h"

// Configuration options
const bool REVERSE_ENCODER_DIRECTION = true;  // Set to true to reverse encoder direction
const int encoderThreshold = 4;  // Sensitivity Adjustment 1 - required encoder events
const unsigned long encoderDebounceTime = 0; // Sensitivity Adjustment 2 - milliseconds between encoder events
const unsigned long SCREEN_PRE_RENDER_DELAY_MS = 5;    // Delay before LVGL rendering
const unsigned long SCREEN_POST_RENDER_DELAY_MS = 150; // Delay after LVGL rendering

// Long press settings
#define LONG_PRESS_DURATION 400  // 800ms for long press
volatile unsigned long buttonPressStartTime = 0;
volatile bool longPressDetected = false;

// Navigation state variables
int8_t currentScreenIndex = 0;
int8_t currentFocusIndex = 0;
bool buttonPressed = false;
extern bool valueAdjustmentMode;
extern lv_obj_t *currentAdjustmentObject;
extern int adjustmentSensitivity;
extern void update_ui_labels();
lv_obj_t *precision_indicator = NULL;
lv_timer_t *precision_indicator_timer = NULL;
const uint32_t PRECISION_INDICATOR_DURATION_MS = 1000;  // 1 second
unsigned long precisionIndicatorEndTime = 0;  // Time when indicator should disappear

// Encoder state tracking
volatile bool buttonCurrentlyPressed = false;
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile long lastEncoderValue = 0;
volatile unsigned long lastButtonPress = 0;
const unsigned long debounceTime = 200; // Debounce time in ms
unsigned long lastEncoderTime = 0;

// Screen navigation information
// Define the focusable objects for each screen
lv_obj_t** focusableObjects[7]; // Array for each screen (7 total screens)
int focusableObjectsCount[7]; // Count for each screen

// Forward declaration
void setupFocusableObjects();
void setFocus(lv_obj_t* obj);

void setupFocusStyles() {
  // Apply to all button styles on focus state
  for (int i = 0; i < 7; i++) { 
    for (int j = 0; j < focusableObjectsCount[i]; j++) {
      lv_obj_t* obj = focusableObjects[i][j];
      
      // Style for focused state - highlight with blue border or outline
      lv_obj_set_style_border_color(obj, lv_color_hex(0x2196F3), LV_STATE_FOCUSED);  // Blue border
      lv_obj_set_style_border_width(obj, 3, LV_STATE_FOCUSED);                       // Thicker border when focused
      lv_obj_set_style_border_opa(obj, 255, LV_STATE_FOCUSED);                       // Full opacity
      
      // Optional: You can also change the background color slightly
      lv_obj_set_style_bg_color(obj, lv_color_hex(0x808080), LV_STATE_FOCUSED);     // Darker background
      
      // Enable focus tracking on the object
      lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    }
  }
}

void resetPrecisionIndicator() {
  // Remove any existing indicator and timer
  if (precision_indicator != NULL) {
      lv_obj_del(precision_indicator);
      precision_indicator = NULL;
  }
  
  if (precision_indicator_timer != NULL) {
      lv_timer_del(precision_indicator_timer);
      precision_indicator_timer = NULL;
  }
}

// Function to handle screen transitions with UI refresh
void transitionToScreen(enum ScreensEnum screenId, int8_t newScreenIndex, int8_t newFocusIndex) {
  // Reset any active precision indicator
  resetPrecisionIndicator();
  
  // Load the new screen
  loadScreen(screenId);
  
  // Update the current indices
  currentScreenIndex = newScreenIndex;
  currentFocusIndex = newFocusIndex;
  
  // Wait for screen to load and refresh UI
  delay(SCREEN_PRE_RENDER_DELAY_MS);
  lv_timer_handler();  // Process any pending LVGL tasks
  delay(SCREEN_POST_RENDER_DELAY_MS);
  
  // Update UI labels with current values
  update_ui_labels();
  
  // Set focus on the first item
  setFocus(focusableObjects[currentScreenIndex][currentFocusIndex]);
}

void showPrecisionIndicator() {
  // Set the end time for the indicator
  precisionIndicatorEndTime = millis() + PRECISION_INDICATOR_DURATION_MS;
  
  // Always recreate the indicator on the current screen
  if (precision_indicator != NULL) {
      lv_obj_del(precision_indicator);  // Delete the old indicator
      precision_indicator = NULL;
  }
  
  // Create a new indicator as a top-level object
  precision_indicator = lv_label_create(lv_layer_top());  // Create on top layer
  
  // Set styling
  lv_obj_set_style_bg_color(precision_indicator, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(precision_indicator, 180, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_color(precision_indicator, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_radius(precision_indicator, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_all(precision_indicator, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_align(precision_indicator, LV_ALIGN_BOTTOM_MID, 0, -10);
  
  // Set the text based on the current mode
  if (ultraFineAdjustmentMode) {
      lv_label_set_text(precision_indicator, "ULTRA-FINE (0.01%)");
      lv_obj_set_style_bg_color(precision_indicator, lv_color_hex(0x0000FF), LV_PART_MAIN | LV_STATE_DEFAULT); // Blue
  } else if (fineAdjustmentMode) {
      lv_label_set_text(precision_indicator, "FINE ADJUSTMENT");
      lv_obj_set_style_bg_color(precision_indicator, lv_color_hex(0x00AA00), LV_PART_MAIN | LV_STATE_DEFAULT); // Green
  } else {
      lv_label_set_text(precision_indicator, "COARSE ADJUSTMENT");
      lv_obj_set_style_bg_color(precision_indicator, lv_color_hex(0xFF6600), LV_PART_MAIN | LV_STATE_DEFAULT); // Orange
  }
  
  // Cancel any existing timers
  if (precision_indicator_timer != NULL) {
      lv_timer_del(precision_indicator_timer);
  }
  
  // We'll handle timer in handleEncoder instead of using LVGL timer
  precision_indicator_timer = NULL;
}

void toggleAdjustmentPrecision() {
  // For sequence positions, toggle between fine and ultra-fine only
  if (currentPositionBeingAdjusted >= 0) {
    // Just toggle between ultra-fine and fine
    ultraFineAdjustmentMode = !ultraFineAdjustmentMode;
    
    // Always keep fineAdjustmentMode true for sequence positions
    // This prevents it from ever entering coarse mode
    fineAdjustmentMode = true;
  } else {
    // For other adjustments, toggle between fine and coarse as before
    fineAdjustmentMode = !fineAdjustmentMode;
    // Make sure ultra-fine is off for non-sequence adjustments
    ultraFineAdjustmentMode = false;
  }
  
  // Show feedback
  showPrecisionIndicator();
}

void IRAM_ATTR handleEncoderInterrupt() {
  int MSB = digitalRead(ENCODER_PIN_A);
  int LSB = digitalRead(ENCODER_PIN_B);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;
  
  lastEncoded = encoded;
}

void IRAM_ATTR handleButtonInterrupt() {
  // This gets called on both rising and falling edges
  bool buttonState = digitalRead(ENCODER_BUTTON_PIN);
  unsigned long currentTime = millis();
  
  // Button pressed (LOW since it's pulled up)
  if (buttonState == LOW && !buttonCurrentlyPressed) {
      buttonCurrentlyPressed = true;
      buttonPressStartTime = currentTime;
  }
  // Button released (HIGH)
  else if (buttonState == HIGH && buttonCurrentlyPressed) {
      buttonCurrentlyPressed = false;
      
      // Check if it was a long press
      if (currentTime - buttonPressStartTime > LONG_PRESS_DURATION) {
          longPressDetected = true;
          // Debug message for long press detection
          Serial.println("DEBUG: Long press detected");
      } 
      // Otherwise it's a normal press if it's past debounce time
      else if (currentTime - buttonPressStartTime > 20) { // 20ms debounce
          buttonPressed = true;
          // Debug message for button press detection
          Serial.println("DEBUG: Button press detected");
      }
  }
}

void setupEncoder() {
  // Configure pins
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), handleEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BUTTON_PIN), handleButtonInterrupt, CHANGE);
  
  // Set up focusable objects for each screen
  setupFocusableObjects();
  
  // Set up styles for focus states
  setupFocusStyles();
  
  // Set initial focus on the first item of the current screen
  if (focusableObjectsCount[currentScreenIndex] > 0) {
    setFocus(focusableObjects[currentScreenIndex][0]);
  }
}

void setupFocusableObjects() {
  // Main screen objects
  static lv_obj_t* mainScreenObjects[] = {
    objects.move_steps,
    objects.manual_jog,
    objects.continuous,
    objects.auto_button,
    objects.settings_button
  };
  focusableObjects[0] = mainScreenObjects;
  focusableObjectsCount[0] = 5;
  
  // Move steps screen
  static lv_obj_t* moveStepsScreenObjects[] = {
    objects.back,
    objects.start,
    objects.step_num,
    objects.clockwise,
    objects.speed
  };
  focusableObjects[1] = moveStepsScreenObjects;
  focusableObjectsCount[1] = 5;
  
  // Manual jog screen
  static lv_obj_t* manualJogScreenObjects[] = {
    objects.back_1,
    objects.start_1,
    objects.speed_manual_jog
  };
  focusableObjects[2] = manualJogScreenObjects;
  focusableObjectsCount[2] = 3;
  
  // Continuous rotation screen
  static lv_obj_t* continuousRotationScreenObjects[] = {
    objects.back_2,
    objects.continuous_rotation_start_button,
    objects.continuous_rotation_speed_button,
    objects.continuous_rotation_direction_button
  };
  focusableObjects[3] = continuousRotationScreenObjects;
  focusableObjectsCount[3] = 4;
  
  // Sequence screen
  static lv_obj_t* sequenceScreenObjects[] = {
    objects.back_4,
    objects.continuous_rotation_start_button_1,
    objects.sequence_positions_button,
    objects.sequence_speed_button,
    objects.sequence_direction_button
  };
  focusableObjects[4] = sequenceScreenObjects;
  focusableObjectsCount[4] = 5;
  
  // Sequence positions screen
  static lv_obj_t* sequencePositionsScreenObjects[] = {
    objects.back_5,
    objects.sequence_position_0_button,
    objects.sequence_position_1_button,
    objects.sequence_position_2_button,
    objects.sequence_position_3_button,
    objects.sequence_position_4_button
  };
  focusableObjects[5] = sequencePositionsScreenObjects;
  focusableObjectsCount[5] = 6;
  
  // Settings screen
  static lv_obj_t* settingsScreenObjects[] = {
    objects.back_3,
    objects.acceleration_button,
    objects.microstepping_button
  };
  focusableObjects[6] = settingsScreenObjects;
  focusableObjectsCount[6] = 3;
}

void handleEncoder() {
  unsigned long currentTime = millis();

  // Check if it's time to hide the precision indicator
  if (precision_indicator != NULL && currentTime > precisionIndicatorEndTime) {
      lv_obj_del(precision_indicator);
      precision_indicator = NULL;
  }
  
  // Check for long press to toggle adjustment mode
  if (longPressDetected) {
      longPressDetected = false;
      
      // Only toggle if in value adjustment mode
      if (valueAdjustmentMode) {
          toggleAdjustmentPrecision();
      }
      return;
  }

  // Skip regular UI navigation when in jog mode
  if (encoderJogMode) {
      return;
  }
  
  if (encoderValue != lastEncoderValue) {
      // Handle value adjustment mode
      if (valueAdjustmentMode && currentAdjustmentObject != NULL) {
          // Only register a change if enough encoder ticks have accumulated
          if (abs(encoderValue - lastEncoderValue) >= 1) {
              // Determine direction based on encoder values
              int rawDirection = encoderValue > lastEncoderValue ? 1 : -1;
              int direction = REVERSE_ENCODER_DIRECTION ? -rawDirection : rawDirection;
              
              // Debugging for position adjustment
              if (currentPositionBeingAdjusted >= 0) {
                Serial.print("Adjusting position ");
                Serial.print(currentPositionBeingAdjusted);
                Serial.print(" with direction ");
                Serial.println(direction);
              }
              
              // Adjust the appropriate value based on which object is being adjusted
              adjustValueByEncoder(currentAdjustmentObject, direction * adjustmentSensitivity);
              
              // Update tracking variables
              lastEncoderValue = encoderValue;
              lastEncoderTime = currentTime;
          }
          return;
      }
      
      // Regular UI navigation (only when not in adjustment mode)
      // Only register a change if:
      // 1. Enough pulses have accumulated since last navigation
      // 2. Enough time has passed since the last navigation
      if ((abs(encoderValue - lastEncoderValue) >= encoderThreshold) && 
          (currentTime - lastEncoderTime > encoderDebounceTime)) {
          
          // Determine direction based on encoder values
          int rawDirection = encoderValue > lastEncoderValue ? 1 : -1;
          
          // Apply direction reversal if configured
          int direction = REVERSE_ENCODER_DIRECTION ? -rawDirection : rawDirection;
          
          navigateUI(direction);
          
          // Update tracking variables
          lastEncoderValue = encoderValue;
          lastEncoderTime = currentTime;
      }
  }

  if (buttonPressed) {
    selectCurrentItem();
    buttonPressed = false;
  }

  // After processing encoder movements, ensure the precision indicator stays visible
  if (precision_indicator != NULL) {
    // Make sure it stays on top after any UI updates
    lv_obj_move_foreground(precision_indicator);
  }
}

void selectCurrentItem() {
  lv_obj_t *currentObj = focusableObjects[currentScreenIndex][currentFocusIndex];
  
  // For sequence positions screen, directly send the click event to the standard handler
  if (currentScreenIndex == 5) { // Sequence Positions Page
    if (currentObj == objects.sequence_position_0_button ||
        currentObj == objects.sequence_position_1_button ||
        currentObj == objects.sequence_position_2_button ||
        currentObj == objects.sequence_position_3_button ||
        currentObj == objects.sequence_position_4_button) {
      
      // Just forward the click to the standard event handler
      lv_event_send(currentObj, LV_EVENT_CLICKED, NULL);
      return;
    }
  }
  
  // If already in adjustment mode, exit it
  if (valueAdjustmentMode) {
    valueAdjustmentMode = false;
    currentPositionBeingAdjusted = -1;
    currentAdjustmentObject = NULL;
    
    // Restore original style (remove highlight)
    lv_obj_set_style_bg_color(currentObj, lv_color_hex(0x656565), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    return;
  }
  
  // Handle other adjustable values
  bool isAdjustableValue = false;
  
  if (currentScreenIndex == 1) { // Move Steps Page
    isAdjustableValue = (currentObj == objects.step_num || currentObj == objects.speed);
  }
  else if (currentScreenIndex == 2) { // Manual Jog Page
    isAdjustableValue = (currentObj == objects.speed_manual_jog);
  }
  else if (currentScreenIndex == 3) { // Continuous Rotation Page
    isAdjustableValue = (currentObj == objects.continuous_rotation_speed_button);
  }
  else if (currentScreenIndex == 4) { // Sequence Page
    isAdjustableValue = (currentObj == objects.sequence_speed_button);
  }
  else if (currentScreenIndex == 6) { // Settings Page
    isAdjustableValue = (currentObj == objects.microstepping_button || 
                          currentObj == objects.acceleration_button);
  }
  
  // If this is an adjustable value, enter adjustment mode
  if (isAdjustableValue) {
    valueAdjustmentMode = true;
    currentAdjustmentObject = currentObj;
    
    // Set the default adjustment modes
    fineAdjustmentMode = true;
    ultraFineAdjustmentMode = false;
    
    // Highlight the button to indicate adjustment mode
    lv_obj_set_style_bg_color(currentObj, lv_color_hex(0x2196F3), LV_PART_MAIN | LV_STATE_DEFAULT);
    
    return;
  }
  
  // For screen navigation and other non-adjustable buttons
  // Clear focus states first
  for (int i = 0; i < focusableObjectsCount[currentScreenIndex]; i++) {
    lv_obj_clear_state(focusableObjects[currentScreenIndex][i], LV_STATE_FOCUSED);
  }
  
  // Handle navigation between screens 
  if (currentScreenIndex == 0) { // Main screen
    if (currentObj == objects.move_steps) {
      transitionToScreen(SCREEN_ID_MOVE_STEPS_PAGE, 1, 0);
    } 
    else if (currentObj == objects.manual_jog) {
      transitionToScreen(SCREEN_ID_MANUAL_JOG_PAGE, 2, 0);
    } 
    else if (currentObj == objects.continuous) {
      transitionToScreen(SCREEN_ID_CONTINUOUS_ROTATION_PAGE, 3, 0);
    } 
    else if (currentObj == objects.auto_button) {
      transitionToScreen(SCREEN_ID_SEQUENCE_PAGE, 4, 0);
    } 
    else if (currentObj == objects.settings_button) {
      transitionToScreen(SCREEN_ID_SETTINGS_PAGE, 6, 0);
    }
  }
  else if (currentScreenIndex == 4 && currentObj == objects.sequence_positions_button) {
    transitionToScreen(SCREEN_ID_SEQUENCE_POSITIONS_PAGE, 5, 0);
  }
  else {
    // Handle back buttons
    if ((currentScreenIndex == 1 && currentObj == objects.back) ||
        (currentScreenIndex == 2 && currentObj == objects.back_1) ||
        (currentScreenIndex == 3 && currentObj == objects.back_2) ||
        (currentScreenIndex == 6 && currentObj == objects.back_3)) {
      transitionToScreen(SCREEN_ID_MAIN, 0, 0);
    } 
    else if (currentScreenIndex == 4 && currentObj == objects.back_4) {
      transitionToScreen(SCREEN_ID_MAIN, 0, 0);
    }
    else if (currentScreenIndex == 5 && currentObj == objects.back_5) {
      transitionToScreen(SCREEN_ID_SEQUENCE_PAGE, 4, 0);
    }
    else {
      // Forward any other button clicks to the event handler
      lv_event_send(currentObj, LV_EVENT_CLICKED, NULL);
    }
  }
}

void setFocus(lv_obj_t* obj) {
  // Remove focus from all objects
  for (int i = 0; i < focusableObjectsCount[currentScreenIndex]; i++) {
    lv_obj_clear_state(focusableObjects[currentScreenIndex][i], LV_STATE_FOCUSED);
  }
  
  // Set focus on new object
  lv_obj_add_state(obj, LV_STATE_FOCUSED);
}

void navigateUI(int8_t direction) {
  // Save old index for debugging
  int8_t oldFocusIndex = currentFocusIndex;
  
  // Update focus index
  currentFocusIndex += direction;
  
  // Handle wrapping
  if (currentFocusIndex < 0) {
    currentFocusIndex = focusableObjectsCount[currentScreenIndex] - 1;
  } else if (currentFocusIndex >= focusableObjectsCount[currentScreenIndex]) {
    currentFocusIndex = 0;
  }
  
  // Debug output
  Serial.print("Focus changed: Screen ");
  Serial.print(currentScreenIndex);
  Serial.print(", index ");
  Serial.print(oldFocusIndex);
  Serial.print(" -> ");
  Serial.print(currentFocusIndex);
  Serial.print(", object ptr: 0x");
  Serial.println((uint32_t)focusableObjects[currentScreenIndex][currentFocusIndex], HEX);
  
  // Focus the new object
  setFocus(focusableObjects[currentScreenIndex][currentFocusIndex]);
}