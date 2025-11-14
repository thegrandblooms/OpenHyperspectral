// This code is only used if the L298NDriver is selected in motorcontrol.ino

#ifndef L298N_DRIVER_H
#define L298N_DRIVER_H

#include "StepperDriver.h"

class L298NDriver : public StepperDriver {
public:
    // Constructor
    L298NDriver(int pin1, int pin2, int pin3, int pin4, int enablePinA, int enablePinB) : 
        _pin1(pin1), _pin2(pin2), _pin3(pin3), _pin4(pin4),
        _enablePinA(enablePinA), _enablePinB(enablePinB),
        _stepCount(0) {}
    
    // Initialize the driver
    void init() override {
        pinMode(_pin1, OUTPUT);
        pinMode(_pin2, OUTPUT);
        pinMode(_pin3, OUTPUT);
        pinMode(_pin4, OUTPUT);
        pinMode(_enablePinA, OUTPUT);
        pinMode(_enablePinB, OUTPUT);
        
        // Start with driver disabled
        disable();
        
        // Initialize pins to starting position
        digitalWrite(_pin1, LOW);
        digitalWrite(_pin2, LOW);
        digitalWrite(_pin3, LOW);
        digitalWrite(_pin4, LOW);
    }
    
    // Set rotation direction
    void setDirection(bool clockwise) override {
        _direction = clockwise;
    }
    
    // Set motor speed (this just stores the value, doesn't affect the actual stepping)
    void setSpeed(int speed) override {
        _speed = constrain(speed, 0, _maxSpeed);
    }
    
    // Execute one step in the current direction
    void step() override {
        if (!_enabled) return;
        
        // If direction is counterclockwise, reverse the sequence
        if (!_direction) {
            _stepCount--;
            if (_stepCount < 0) _stepCount = 3;
        } else {
            _stepCount++;
            if (_stepCount > 3) _stepCount = 0;
        }
        
        // Four-step sequence for full stepping
        switch (_stepCount) {
            case 0:  // 1010
                digitalWrite(_pin1, HIGH);
                digitalWrite(_pin2, LOW);
                digitalWrite(_pin3, HIGH);
                digitalWrite(_pin4, LOW);
                break;
            case 1:  // 0110
                digitalWrite(_pin1, LOW);
                digitalWrite(_pin2, HIGH);
                digitalWrite(_pin3, HIGH);
                digitalWrite(_pin4, LOW);
                break;
            case 2:  // 0101
                digitalWrite(_pin1, LOW);
                digitalWrite(_pin2, HIGH);
                digitalWrite(_pin3, LOW);
                digitalWrite(_pin4, HIGH);
                break;
            case 3:  // 1001
                digitalWrite(_pin1, HIGH);
                digitalWrite(_pin2, LOW);
                digitalWrite(_pin3, LOW);
                digitalWrite(_pin4, HIGH);
                break;
        }
    }
    
    // Enable the driver
    void enable() override {
        digitalWrite(_enablePinA, HIGH);
        digitalWrite(_enablePinB, HIGH);
        _enabled = true;
    }
    
    // Disable the driver
    void disable() override {
        digitalWrite(_enablePinA, LOW);
        digitalWrite(_enablePinB, LOW);
        _enabled = false;
    }
    
private:
    int _pin1;       // Motor pin 1
    int _pin2;       // Motor pin 2
    int _pin3;       // Motor pin 3
    int _pin4;       // Motor pin 4
    int _enablePinA; // Enable pin A
    int _enablePinB; // Enable pin B
    int _stepCount;  // Current step in the sequence
};

#endif // L298N_DRIVER_H