# SimpleFOC Diagnostic - What's Broken?

## Symptoms
1. ✗ SimpleFOC shaft_angle always 0.00° (should match encoder)
2. ✗ Motor doesn't move during calibration
3. ✓ MT6701 encoder works fine (reads 1.67°, 349.72°, etc.)

## Root Cause Analysis

### SimpleFOC shaft_angle = 0.00° means:
- `motor.loopFOC()` isn't updating shaft_angle
- OR `motor.init()` / `motor.initFOC()` never succeeded
- Motor object is in a failed/uninitialized state

### Required SimpleFOC Initialization Steps (Canonical):
```cpp
// 1. Create objects
BLDCMotor motor = BLDCMotor(pole_pairs);
BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, enable);
MagneticSensorI2C sensor = MagneticSensorI2C(...);

// 2. Initialize sensor
sensor.init();

// 3. Link sensor to motor
motor.linkSensor(&sensor);

// 4. Initialize driver
driver.voltage_power_supply = 12;
driver.init();

// 5. Link driver to motor
motor.linkDriver(&driver);

// 6. Configure motor parameters
motor.voltage_limit = 6;
motor.controller = MotionControlType::angle;

// 7. Initialize motor (CRITICAL!)
motor.init();

// 8. Align motor and sensor (CRITICAL!)
motor.initFOC();  // Returns 1 on success, 0 on failure

// 9. In loop(), call these CONTINUOUSLY:
void loop() {
  motor.loopFOC();      // Run FOC algorithm
  motor.move(target);   // Run motion control
}
```

## What We're Missing

### Check 1: Is motor.init() succeeding?
From grep output, we call it at motor_control.cpp:402, but we don't check the return value!

### Check 2: Is motor.initFOC() succeeding?
From logs: **initFOC() returned: 0** ← FAILED!

### Check 3: After failed initFOC(), can we still use motor?
**NO!** If initFOC() fails, SimpleFOC won't work. Motor stays in uninitialized state.

## The Critical Issue

**motor.initFOC() is failing** (returns 0), which means:
- zero_electric_angle is never calculated
- SimpleFOC doesn't know mechanical→electrical mapping
- motor.loopFOC() won't work (shaft_angle stays 0)
- Motor won't move

## Why initFOC() is Failing

From logs:
```
MOT: Align sensor.
MOT: Failed to notice movement
MOT: Init FOC failed.
```

SimpleFOC applies voltage, but its movement detection logic fails.

## The Fix Strategy

We have TWO options:

### Option A: Fix the alignment detection (what we tried)
- Force needsSearch()=1 during calibration
- But user is running old firmware, so this didn't get tested

### Option B: Skip alignment entirely (use pre-calculated values)
- Run alignment ONCE manually
- Save zero_electric_angle and sensor_direction
- On future boots, set them before initFOC():
```cpp
motor.zero_electric_angle = 2.15;  // From successful calibration
motor.sensor_direction = Direction::CW;
motor.initFOC();  // Will skip alignment and return success
```

## Immediate Next Steps

1. **Flash the latest firmware** (with calibration mode fix)
2. **Add debug output** to see if motor.init() succeeded
3. **If initFOC() still fails**, manually measure zero_electric_angle
4. **Hardcode the values** to skip alignment entirely

## Manual Calibration Procedure (Backup Plan)

If automatic alignment keeps failing, we can manually calculate zero_electric_angle:

1. Place motor at a known position (e.g., 0° mechanical)
2. Apply voltage at known electrical angle (e.g., 3π/2)
3. Motor will move to align with that electrical angle
4. Read encoder position after alignment
5. Calculate: zero_electric_angle = encoder_reading - (electrical_angle / pole_pairs)
6. Save this value and use it in code

This is what the broken manual calibration TRIED to do, but it assumed motor was already aligned (it wasn't).
