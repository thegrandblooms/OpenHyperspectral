// TimerStepperControl.cpp
#include "TimerStepperControl.h"

// Initialize static instance pointer
TimerStepperControl* TimerStepperControl::instance = nullptr;

// Constructor
TimerStepperControl::TimerStepperControl(StepperDriver* driver) :
    _driver(driver),
    _isRunning(false),
    _isContinuous(false),
    _direction(true),
    _speed(0),
    _currentPosition(0),
    _targetPosition(0),
    _acceleration(6400), // Default acceleration
    _gptimer(nullptr),
    _commandQueue(nullptr),
    _motorTaskHandle(nullptr),
    _minStepInterval(1000),
    _lastStepTime(0),
    _stepAccumulator(0.0f),
    _stepsPerMs(0.0f),
    _currentSpeed(0.0f),
    _lastAccelUpdateTime(0),
    _jogMode(false)  // Initialize jog mode flag
{
    // Store instance pointer for ISR
    instance = this;
}

// Initialize hardware timer and FreeRTOS components
void TimerStepperControl::init() {
    // Initialize the driver
    _driver->init();
    
    // Create command queue
    _commandQueue = xQueueCreate(10, sizeof(MotorCommand_t));
    
    // Create motor control task
    xTaskCreate(
        motorControlTask,
        "motor_task",
        4096,
        this,
        10, // Higher priority
        &_motorTaskHandle
    );
    
    // Configure timer
    gptimer_config_t timer_config;
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    timer_config.direction = GPTIMER_COUNT_UP;
    timer_config.resolution_hz = 1000000;  // 1MHz = 1us resolution
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &_gptimer));
    
    // Configure timer alarm
    gptimer_alarm_config_t alarm_config;
    alarm_config.reload_count = 0;
    alarm_config.alarm_count = 250; // 1ms intervals
    alarm_config.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(_gptimer, &alarm_config));
    
    // Register timer callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timerCallback,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(_gptimer, &cbs, this));
    
    // Start the timer
    ESP_ERROR_CHECK(gptimer_enable(_gptimer));
    ESP_ERROR_CHECK(gptimer_start(_gptimer));
}

// Function for clearing move queue
void TimerStepperControl::clearCommandQueue() {
    // Stop the motor immediately
    _isRunning = false;
    _isContinuous = false;
    _jogMode = false;
    _currentSpeed = 0;
    _stepAccumulator = 0.0f;
    
    // Empty the queue if it exists
    if (_commandQueue != NULL) {
        // Create a dummy command to receive and discard items
        MotorCommand_t dummyCmd;
        
        // Empty the queue by receiving all available items
        while (xQueueReceive(_commandQueue, &dummyCmd, 0) == pdTRUE) {
            // Just discard the commands
        }
    }
}

void TimerStepperControl::resetMotorState() {
    // Reset all state variables
    _isRunning = false;
    _isContinuous = false;
    _jogMode = false;
    _currentSpeed = 0.0f;
    _stepAccumulator = 0.0f;
    
    // Very important: reset the timestamp to prevent elapsed time jumps
    _lastAccelUpdateTime = micros();
    _lastStepTime = _lastAccelUpdateTime;
}

// Timer callback (ISR)
bool IRAM_ATTR TimerStepperControl::timerCallback(gptimer_handle_t timer, 
        const gptimer_alarm_event_data_t *edata, 
        void *user_data) {
    // Get instance pointer
    TimerStepperControl* obj = (TimerStepperControl*)user_data;

    // If running, process a step if needed
    if (obj->_isRunning) {
    obj->processStep();
    }

    // Return false to avoid waking up a high-priority task
    return false;
}

// Process a single step if needed
void TimerStepperControl::processStep() {
    // Only process if we're supposed to be running
    if (!_isRunning) return;
    
    // Get current time
    unsigned long currentTime = micros();
    unsigned long elapsedTime = currentTime - _lastAccelUpdateTime;
    
    // Update acceleration timestamp
    _lastAccelUpdateTime = currentTime;
    
    // Update speed based on acceleration (but not in jog mode)
    if (!_jogMode) {
        // Existing acceleration logic...
        if (_currentSpeed < _speed && _acceleration > 0) {
            // Accelerating
            _currentSpeed += _acceleration * (elapsedTime / 1000000.0f);
            if (_currentSpeed > _speed) _currentSpeed = _speed; // Cap at target speed
        } else if (_currentSpeed > _speed && _acceleration > 0) {
            // Decelerating 
            _currentSpeed -= _acceleration * (elapsedTime / 1000000.0f);
            if (_currentSpeed < 0) _currentSpeed = 0; // Don't go negative
        }
    } else {
        // In jog mode, use target speed directly - no acceleration
        _currentSpeed = _speed;
    }
    
    // Update acceleration timestamp
    _lastAccelUpdateTime = currentTime;
    
    // Calculate step interval based on current speed (protect against division by zero)
    unsigned long stepInterval = (_currentSpeed > 0) ? (1000000 / _currentSpeed) : 1000000;
    
    // Add step accumulation based on current speed and timer interval
    _stepAccumulator += _currentSpeed * (elapsedTime / 1000000.0f);
    
    // Check if we've accumulated enough for a step
    if (_stepAccumulator >= 1.0f) {
        // Reset accumulator but keep the fractional part
        _stepAccumulator -= 1.0f;
        
        // For continuous rotation mode
        if (_isContinuous) {
            // Apply direction and take step
            _driver->setDirection(_direction);
            _driver->step();
            
            // Update position counter
            if (_direction) {
                _currentPosition++;
            } else {
                _currentPosition--;
            }
            return;
        }
        
        // For position control mode, check if we've reached the target
        if (_currentPosition == _targetPosition) {
            _isRunning = false;
            _driver->disable();
            return;
        }
        
        // Determine direction based on position difference
        if (_currentPosition < _targetPosition) {
            _driver->setDirection(true);
            _driver->step();
            _currentPosition++;
        } else {
            _driver->setDirection(false);
            _driver->step();
            _currentPosition--;
        }
    }
}

// Send a command to the motor control task
bool TimerStepperControl::sendCommand(MotorCommand_t* cmd) {
    // Send command to queue with timeout
    return xQueueSend(_commandQueue, cmd, pdMS_TO_TICKS(100)) == pdTRUE;
}

// Motor control task
void TimerStepperControl::motorControlTask(void* pvParameters) {
    TimerStepperControl* obj = (TimerStepperControl*)pvParameters;
    MotorCommand_t cmd;
    
    while (1) {
        // Check for new commands
        if (xQueueReceive(obj->_commandQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
            obj->handleCommand(&cmd);
        }
        
        // Allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Handle a command
void TimerStepperControl::handleCommand(MotorCommand_t* cmd) {
    switch (cmd->cmd_type) {
        case CMD_MOVE_TO:
            _targetPosition = cmd->position;
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            _stepsPerMs = _speed / 1000.0f; // Add this line
            _stepAccumulator = 0.0f;        // Add this line
            _isRunning = true;
            _isContinuous = false;
            _driver->enable();
            _jogMode = false;  // Clear jog mode flag
            break;
            
        case CMD_MOVE_STEPS:
            _targetPosition = _currentPosition + cmd->position;
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            _stepsPerMs = _speed / 1000.0f; // Add this line
            _stepAccumulator = 0.0f;        // Add this line
            _isRunning = true;
            _isContinuous = false;
            _driver->enable();
            _currentSpeed = 0; // Start from standstill
            _lastAccelUpdateTime = micros(); // Initialize timestamp
            _jogMode = false;  // Clear jog mode flag
            break;
            
        case CMD_SET_SPEED:
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            _stepsPerMs = _speed / 1000.0f; // Add this line
            break;
            
        case CMD_START_JOG:
            // Enable motor for jogging but don't change position targets
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            _stepsPerMs = _speed / 1000.0f;
            _stepAccumulator = 0.0f;
            _isRunning = true;
            _isContinuous = false;
            _jogMode = true;  // Set jog mode flag
            _driver->enable();
            break;
        
        case CMD_MOVE_JOG:
            // Make sure the new command completely replaces any pending movement
            _targetPosition = _currentPosition + cmd->position;
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            _stepsPerMs = _speed / 1000.0f;
            _stepAccumulator = 0.0f;
            _isRunning = true;
            _isContinuous = false;
            _jogMode = true;  // Important - ensures we bypass acceleration
            _driver->enable();
            break;
            
        case CMD_START_CONTINUOUS:
            _direction = cmd->direction;
            _speed = cmd->speed;
            _driver->setSpeed(_speed);
            _minStepInterval = _speed > 0 ? 1000000 / _speed : 1000000;
            _stepsPerMs = _speed / 1000.0f; // Add this line
            _stepAccumulator = 0.0f;        // Add this line
            _isRunning = true;
            _isContinuous = true;
            _driver->setDirection(_direction);
            _driver->enable();
            _currentSpeed = 0; // Start from standstill
            _lastAccelUpdateTime = micros(); // Initialize timestamp
            _jogMode = false;  // Clear jog mode flag
            break;
            
        case CMD_STOP_MOTOR:
            _isRunning = false;
            _isContinuous = false;
            _driver->disable();
            _jogMode = false;  // Clear jog mode flag
            break;
            
        default:
            break;
    }
}

// Check if motor is currently running
bool TimerStepperControl::isRunning() {
    return _isRunning;
}

// Set current position
void TimerStepperControl::setCurrentPosition(long position) {
    _currentPosition = position;
    _targetPosition = position;
}

// Get current position
long TimerStepperControl::getCurrentPosition() {
    return _currentPosition;
}

// For power management
void TimerStepperControl::sleep() {
    _driver->disable();
    
    #if USE_DRV8825_DRIVER
    // If using DRV8825, we can safely call sleep directly
    DRV8825Driver* drv8825 = static_cast<DRV8825Driver*>(_driver);
    drv8825->sleep();
    #endif
}

void TimerStepperControl::wake() {
    #if USE_DRV8825_DRIVER
    // If using DRV8825, we can safely call wake directly
    DRV8825Driver* drv8825 = static_cast<DRV8825Driver*>(_driver);
    drv8825->wake();
    #endif
}