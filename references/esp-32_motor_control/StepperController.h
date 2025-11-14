#ifndef STEPPER_CONTROLLER_H
#define STEPPER_CONTROLLER_H

#include "StepperDriver.h"
#include <Arduino.h>

class StepperController {
public:
    // Constructor
    StepperController(StepperDriver* driver) : 
        _driver(driver),
        _targetPosition(0),
        _currentPosition(0),
        _acceleration(1000),
        _minStepInterval(1000), // microseconds
        _lastStepTime(0),
        _isRunning(false),
        _isContinuous(false) {}
    
    // Initialize the controller
    void init() {
        _driver->init();
    }
    
    // Set target position (relative to current position)
    void move(long steps) {
        _targetPosition = _currentPosition + steps;
        _isRunning = true;
        _isContinuous = false;
        
        // Set direction based on movement direction
        _driver->setDirection(steps >= 0);
        _driver->enable();
    }
    
    // Start continuous rotation
    void startContinuous(bool clockwise, int speed) {
        _driver->setDirection(clockwise);
        _driver->setSpeed(speed);
        _isRunning = true;
        _isContinuous = true;
        
        // Calculate step interval based on speed
        // speed is in steps per second
        if (speed > 0) {
            _minStepInterval = 1000000 / speed; // convert to microseconds
        }
        
        _driver->enable();
    }
    
    // Stop motion
    void stop() {
        _isRunning = false;
        _isContinuous = false;
        _driver->disable();
    }
    
    // Run the stepper motor (call frequently in main loop)
    void run() {
        if (!_isRunning) return;
        
        // Get current time
        unsigned long currentTime = micros();
        
        // Check if it's time for the next step
        if (currentTime - _lastStepTime >= _minStepInterval) {
            
            // For continuous rotation, just keep stepping
            if (_isContinuous) {
                _driver->step();
                _lastStepTime = currentTime;
                return;
            }
            
            // For position control, check if we've reached the target
            if (_currentPosition == _targetPosition) {
                _isRunning = false;
                _driver->disable();
                return;
            }
            
            // Determine direction to move
            if (_currentPosition < _targetPosition) {
                _driver->setDirection(true); // clockwise
                _driver->step();
                _currentPosition++;
            } else {
                _driver->setDirection(false); // counterclockwise
                _driver->step();
                _currentPosition--;
            }
            
            _lastStepTime = currentTime;
        }
    }
    
    // Check if motor is still running
    bool isRunning() {
        return _isRunning;
    }
    
    // Get distance to target position
    long distanceToGo() {
        return _targetPosition - _currentPosition;
    }
    
    // Set acceleration (in steps/second^2)
    void setAcceleration(float acceleration) {
        _acceleration = acceleration;
    }
    
    // Set maximum speed (in steps/second)
    void setMaxSpeed(float maxSpeed) {
        _driver->setMaxSpeed(maxSpeed);
        // Recalculate minimum step interval
        if (maxSpeed > 0) {
            _minStepInterval = 1000000 / maxSpeed; // convert to microseconds
        }
    }
    
    // Set current speed (steps/second)
    void setSpeed(float speed) {
        _driver->setSpeed(speed);
        // Calculate step interval based on speed
        if (speed > 0) {
            _minStepInterval = 1000000 / speed; // convert to microseconds
        }
    }
    
    // Get driver instance
    StepperDriver* getDriver() {
        return _driver;
    }
    
    // Set current position
    void setCurrentPosition(long position) {
        _currentPosition = position;
        _targetPosition = position;
    }
    
    // Get current position
    long getCurrentPosition() {
        return _currentPosition;
    }
    
private:
    StepperDriver* _driver; // Motor driver
    
    long _targetPosition;   // Target position (in steps)
    long _currentPosition;  // Current position (in steps)
    
    float _acceleration;     // Acceleration (steps/second^2)
    unsigned long _minStepInterval; // Minimum time between steps (microseconds)
    unsigned long _lastStepTime;    // Last step time (microseconds)
    
    bool _isRunning;     // Whether the motor is running
    bool _isContinuous;  // Whether in continuous rotation mode
};

#endif // STEPPER_CONTROLLER_H