// This code is only used if the DRV8825 driver is selected in motorcontrol.ino

#ifndef DRV8825_DRIVER_H
#define DRV8825_DRIVER_H

#include "StepperDriver.h"

class DRV8825Driver : public StepperDriver {
public:
    // New methods for Sleep/Reset control
    void sleep() {
        // Put driver to sleep to save power
        if (_sleepResetPin != -1) {
            digitalWrite(_sleepResetPin, LOW);
            _isAsleep = true;
        }
    }

    void wake() {
        // Wake up the driver
        if (_sleepResetPin != -1) {
            digitalWrite(_sleepResetPin, HIGH);
            _isAsleep = false;
            
            // The DRV8825 needs a short delay to fully wake up
            delayMicroseconds(1000);
        }
    }

    bool isAsleep() {
        return _isAsleep;
    }

    // Microstepping modes
    enum MicrostepMode {
        FULL_STEP = 1,
        HALF_STEP = 2,
        QUARTER_STEP = 4,
        EIGHTH_STEP = 8,
        SIXTEENTH_STEP = 16,
        THIRTYSECOND_STEP = 32
    };
    
    // Constructor
    DRV8825Driver(int stepPin, int dirPin, int enablePin, 
        int m0Pin = -1, int m1Pin = -1, int m2Pin = -1, 
        int sleepResetPin = -1) : 
    _stepPin(stepPin), _dirPin(dirPin), _enablePin(enablePin),
    _m0Pin(m0Pin), _m1Pin(m1Pin), _m2Pin(m2Pin),
    _sleepResetPin(sleepResetPin),
    _microstepMode(FULL_STEP), _pulseWidthUs(5), _isAsleep(true) {}
        
    // Initialize the driver
    void init() override {
        // Configure basic control pins
        pinMode(_stepPin, OUTPUT);
        pinMode(_dirPin, OUTPUT);
        pinMode(_enablePin, OUTPUT);
        
        // Configure microstepping pins if connected
        if (_m0Pin != -1) pinMode(_m0Pin, OUTPUT);
        if (_m1Pin != -1) pinMode(_m1Pin, OUTPUT);
        if (_m2Pin != -1) pinMode(_m2Pin, OUTPUT);
        
        // Configure SLEEP/RESET pin if connected
        if (_sleepResetPin != -1) {
            pinMode(_sleepResetPin, OUTPUT);
            digitalWrite(_sleepResetPin, HIGH); // Wake up the driver
            _isAsleep = false;
        }
        
        // Start with driver disabled
        disable();
        
        // Set initial direction
        digitalWrite(_dirPin, _direction ? HIGH : LOW);
        
        // Set initial microstepping mode
        setMicrostepMode(_microstepMode);
    }
    
    // Set rotation direction
    void setDirection(bool clockwise) override {
        _direction = clockwise;
        digitalWrite(_dirPin, _direction ? HIGH : LOW);
    }
    
    // Set motor speed
    void setSpeed(int speed) override {
        _speed = constrain(speed, 0, _maxSpeed);
    }
    
    // Execute one step
    void step() override {
        if (!_enabled) return;
        
        // Generate a pulse on the step pin
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(_pulseWidthUs); // Minimum pulse width
        digitalWrite(_stepPin, LOW);
    }
    
    // Enable the driver
    void enable() override {
        // Make sure driver is awake before enabling
        if (_isAsleep && _sleepResetPin != -1) {
            wake();
        }
        
        // DRV8825 uses active LOW for enable pin
        digitalWrite(_enablePin, LOW);
        _enabled = true;
    }
    
    // Disable the driver
    void disable() override {
        // DRV8825 uses active LOW for enable pin
        digitalWrite(_enablePin, HIGH);
        _enabled = false;
        
        // Optionally put the driver to sleep to save even more power
        if (_sleepPin != -1) digitalWrite(_sleepPin, LOW);
    }
    
    // Set microstepping mode
    void setMicrostepMode(int mode) override {
        // Convert integer to one of the valid enum values
        switch (mode) {
            case 1: _microstepMode = FULL_STEP; break;
            case 2: _microstepMode = HALF_STEP; break;
            case 4: _microstepMode = QUARTER_STEP; break;
            case 8: _microstepMode = EIGHTH_STEP; break;
            case 16: _microstepMode = SIXTEENTH_STEP; break;
            case 32: _microstepMode = THIRTYSECOND_STEP; break;
            default: _microstepMode = FULL_STEP; break;
        }
        
        // Only apply if microstepping pins are connected
        if (_m0Pin != -1 && _m1Pin != -1 && _m2Pin != -1) {
            switch (_microstepMode) {
                case FULL_STEP:
                    digitalWrite(_m0Pin, LOW);
                    digitalWrite(_m1Pin, LOW);
                    digitalWrite(_m2Pin, LOW);
                    break;
                case HALF_STEP:
                    digitalWrite(_m0Pin, HIGH);
                    digitalWrite(_m1Pin, LOW);
                    digitalWrite(_m2Pin, LOW);
                    break;
                case QUARTER_STEP:
                    digitalWrite(_m0Pin, LOW);
                    digitalWrite(_m1Pin, HIGH);
                    digitalWrite(_m2Pin, LOW);
                    break;
                case EIGHTH_STEP:
                    digitalWrite(_m0Pin, HIGH);
                    digitalWrite(_m1Pin, HIGH);
                    digitalWrite(_m2Pin, LOW);
                    break;
                case SIXTEENTH_STEP:
                    digitalWrite(_m0Pin, LOW);
                    digitalWrite(_m1Pin, LOW);
                    digitalWrite(_m2Pin, HIGH);
                    break;
                case THIRTYSECOND_STEP:
                    digitalWrite(_m0Pin, HIGH);
                    digitalWrite(_m1Pin, LOW);
                    digitalWrite(_m2Pin, HIGH);
                    break;
                default:
                    digitalWrite(_m0Pin, LOW);
                    digitalWrite(_m1Pin, LOW);
                    digitalWrite(_m2Pin, LOW);
            }
        }
    }
    
    // Get current microstepping mode
    int getMicrostepMode() override {
        return _microstepMode;
    }
    
    // Set the step pulse width (in microseconds)
    void setPulseWidth(int microseconds) {
        _pulseWidthUs = microseconds;
    }
    
    // Check if the driver is reporting a fault
    bool hasFault() {
        if (_faultPin != -1) {
            // DRV8825 fault pin is active LOW
            return digitalRead(_faultPin) == LOW;
        }
        return false;
    }
    
private:
    // Pin definitions
    int _stepPin;    // Step pin
    int _dirPin;     // Direction pin
    int _enablePin;  // Enable pin
    int _m0Pin;      // Microstep mode pin 0
    int _m1Pin;      // Microstep mode pin 1
    int _m2Pin;      // Microstep mode pin 2
    int _sleepResetPin; // Combined SLEEP and RESET pin
    int _sleepPin;   // Sleep pin (optional)
    int _resetPin;   // Reset pin (optional)
    int _faultPin;   // Fault pin (optional)

    // Driver state
    bool _isAsleep;     // Sleep state tracking
    int _microstepMode; // Current microstepping mode
    int _pulseWidthUs;  // Step pulse width in microseconds
};

#endif // DRV8825_DRIVER_H