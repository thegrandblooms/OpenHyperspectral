#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <Arduino.h>

// Base class for all stepper motor drivers
class StepperDriver {
public:
    // Constructor & destructor
    StepperDriver() : _enabled(false), _direction(true), _speed(0), _maxSpeed(1000) {}
    virtual ~StepperDriver() {}
    
    // Common interface methods that all drivers must implement
    virtual void init() = 0;                      // Initialize the driver
    virtual void setDirection(bool clockwise) = 0; // Set rotation direction
    virtual void setSpeed(int speed) = 0;         // Set motor speed
    virtual void step() = 0;                      // Execute one step
    virtual void enable() = 0;                    // Enable the driver
    virtual void disable() = 0;                   // Disable the driver
    
    // Common methods with default implementations
    virtual void setMaxSpeed(int maxSpeed) { _maxSpeed = maxSpeed; }
    virtual int getMaxSpeed() { return _maxSpeed; }
    virtual bool isEnabled() { return _enabled; }
    virtual bool getDirection() { return _direction; }
    virtual int getSpeed() { return _speed; }
    
    // For drivers that support microstepping
    virtual void setMicrostepMode(int mode) { /* Default does nothing */ }
    virtual int getMicrostepMode() { return 1; } // Default is full step
    
protected:
    bool _enabled;    // Driver enabled state
    bool _direction;  // Rotation direction (true = clockwise)
    int _speed;       // Current speed setting
    int _maxSpeed;    // Maximum allowed speed
};

#endif // STEPPER_DRIVER_H