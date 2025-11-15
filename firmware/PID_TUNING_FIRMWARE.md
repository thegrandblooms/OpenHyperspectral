# PID Auto-Tuning for Motor Control (Firmware)

This document describes the PID auto-tuning functionality implemented in the ESP32 firmware for optimal motor position tracking.

## Overview

The firmware includes a built-in PID auto-tuner that automatically finds optimal PID parameters for motor position control. The tuner prevents overshoot by starting with conservative values and gradually increasing gains while monitoring performance.

## Quick Start

### Using the Integration Test

1. Upload the integration test sketch to your ESP32:
   ```
   firmware/tests/integration_test/integration_test.ino
   ```

2. Open Serial Monitor at 115200 baud

3. Run the calibration sequence:
   ```
   c    (or 'calibrate')
   ```

4. Run PID auto-tuning:
   ```
   p    (or 'pidtune')
   ```

5. The tuner will automatically:
   - Test motor response at multiple positions (0°, 90°, 180°, 270°, 360°)
   - Start with low P gain (2.0) to avoid overshoot
   - Gradually increase gains for optimal performance
   - Apply tuned parameters to the motor
   - Display recommended config.h values

### Using the MotorController API

If you're using the `MotorController` class in your firmware:

```cpp
#include "motor_control.h"

MotorController motor_controller;

void setup() {
    Serial.begin(115200);

    // Initialize motor controller
    motor_controller.begin();

    // Run calibration
    if (motor_controller.calibrate()) {
        Serial.println("Calibration successful");

        // Run PID auto-tuning
        if (motor_controller.autoTunePID(true)) {
            Serial.println("PID tuning successful");
            // Tuned parameters are automatically applied
        }
    }
}
```

## Algorithm Details

### Three-Phase Tuning Approach

#### Phase 1: Proportional Gain (P)
- Starts with P = 2.0
- Incrementally increases by steps of 2.0
- Tests motor response at each value
- Stops when overshoot is detected or performance degrades
- Applies 20% stability margin to best P value

#### Phase 2: Derivative Gain (D)
- Fixes P at optimal value from Phase 1
- Incrementally increases D by steps of 0.05
- D gain reduces overshoot and oscillation
- Stops when improvement is less than 5%

#### Phase 3: Integral Gain (I)
- Fixes P and D at optimal values
- Incrementally increases I by steps of 0.1
- I gain eliminates steady-state error
- Stops when target error is achieved

### Performance Metrics

Each PID configuration is evaluated using:

- **Overshoot**: Peak position error beyond target (max allowed: 0.1 rad / 5.7°)
- **Settling Time**: Time to reach and stay within tolerance (target: < 3.0 seconds)
- **Steady-State Error**: Final position error after settling (target: < 0.01 rad / 0.57°)
- **Rise Time**: Time to reach 90% of target position

**Score Calculation** (lower is better):
```
score = overshoot * 10.0 + settling_time * 2.0 + steady_state_error * 5.0 + rise_time * 0.5
```

### Test Positions

The tuner tests movements to:
- 0.0 rad (0°)
- 1.57 rad (90°)
- 3.14 rad (180°)
- 4.71 rad (270°)
- 0.0 rad (0° - full rotation)

## Configuration

### Tuning Parameters

You can customize tuning behavior by modifying constants in `pid_auto_tuner.h`:

```cpp
// Test positions
static constexpr float TEST_POSITIONS[] = {0.0, 1.57, 3.14, 4.71, 0.0};

// Tolerances
static constexpr float POSITION_TOLERANCE = 0.05;      // rad (~2.9°)
static constexpr float SETTLING_TOLERANCE = 0.02;      // rad (~1.1°)

// Initial values (conservative start)
static constexpr float INITIAL_P = 2.0;
static constexpr float INITIAL_I = 0.0;
static constexpr float INITIAL_D = 0.0;

// Increment steps
static constexpr float P_STEP = 2.0;
static constexpr float D_STEP = 0.05;
static constexpr float I_STEP = 0.1;

// Maximum values
static constexpr float MAX_P = 50.0;
static constexpr float MAX_I = 5.0;
static constexpr float MAX_D = 2.0;

// Performance thresholds
static constexpr float MAX_OVERSHOOT = 0.1;              // rad (5.7°)
static constexpr float MAX_SETTLING_TIME = 3.0;          // seconds
static constexpr float TARGET_STEADY_STATE_ERROR = 0.01; // rad (0.57°)
static constexpr float STABILITY_MARGIN = 0.8;           // Use 80% of best P
```

## Integration Test Commands

After uploading the integration test sketch:

| Command | Description |
|---------|-------------|
| `h` or `help` | Show help menu |
| `c` or `calibrate` | Run motor calibration |
| `p` or `pidtune` | Run PID auto-tuning (after calibration) |
| `e` or `enable` | Enable motor |
| `d` or `disable` | Disable motor |
| `t` or `test` | Run automated test sequence |
| `s` or `status` | Show current motor status |
| `r` or `results` | Show test results |
| `m <angle>` | Move to specific angle in radians |

## Example Output

```
================================================================
[TUNER] STARTING PID AUTO-TUNING
================================================================
[TUNER] Test positions: 0.00, 1.57, 3.14, 4.71, 0.00 rad
[TUNER] Max overshoot: 0.1000 rad

================================================================
[PHASE 1] Tuning Proportional Gain (P)
================================================================
[TUNE] Setting PID: P=2.00, I=0.00, D=0.00, Ramp=1000.0
[TUNE] Test 1/5: target=0.00 rad
  Overshoot: 0.0012 rad, Settle: 1.45s, Error: 0.0034 rad, Score: 8.23
...
[TUNE] New best P: 4.00 (score: 7.21)
...
[TUNE] Optimal P (with 20% margin): 6.40

================================================================
[PHASE 2] Tuning Derivative Gain (D)
================================================================
...

================================================================
[PHASE 3] Tuning Integral Gain (I)
================================================================
...

================================================================
[TUNER] TUNING COMPLETE
================================================================
Optimal PID Parameters:
  P = 6.40
  I = 0.20
  D = 0.150
  Ramp = 1000.0

Final Score: 5.32
================================================================

╔════════════════════════════════════════════════════════════════╗
║             PID TUNING SUCCESSFUL!                             ║
╚════════════════════════════════════════════════════════════════╝

Optimal PID parameters have been applied.

To make these values permanent, update config.h:
  #define PID_P_POSITION 6.40
  #define PID_I_POSITION 0.20
  #define PID_D_POSITION 0.150
```

## Making Tuned Values Permanent

### Option 1: Update Firmware Configuration (Recommended)

After successful tuning, update `firmware/ESP32_MCU_Firmware/config.h`:

```cpp
// Position control PID parameters
#define PID_P_POSITION   6.40f    // Update with tuned value
#define PID_I_POSITION   0.20f    // Update with tuned value
#define PID_D_POSITION   0.150f   // Update with tuned value
#define PID_RAMP_POSITION 1000.0f
```

Then recompile and upload the firmware. The tuned values will be used automatically on every boot.

### Option 2: Store in EEPROM (Future Enhancement)

Future versions may support saving tuned values to EEPROM for automatic loading on boot.

## API Reference

### PIDAutoTuner Class

```cpp
#include "pid_auto_tuner.h"

// Constructor
PIDAutoTuner tuner(motor, encoder);

// Run tuning (verbose output to Serial)
bool success = tuner.runTuning(true);

// Get optimal PID values
float p, i, d, ramp;
tuner.getOptimalPID(p, i, d, ramp);

// Get performance metrics
PIDAutoTuner::TuningMetrics metrics = tuner.getMetrics();
```

### MotorController Class

```cpp
#include "motor_control.h"

MotorController controller;

// Run PID auto-tuning
bool success = controller.autoTunePID(true);  // verbose = true

// Tuned parameters are automatically applied
```

## Troubleshooting

### Tuning Takes Too Long

- Reduce number of test positions in `TEST_POSITIONS`
- Increase step sizes (`P_STEP`, `D_STEP`, `I_STEP`)
- Reduce maximum gain limits (`MAX_P`, `MAX_I`, `MAX_D`)

### Excessive Overshoot Detected

- Lower `MAX_OVERSHOOT` threshold
- Increase stability margin (lower `STABILITY_MARGIN` from 0.8 to 0.7)
- Reduce `P_STEP` for finer control

### Motor Oscillates

- Increase `D_STEP` to find damping faster
- Lower `MAX_P` limit
- Check mechanical connections and ensure motor moves freely

### Poor Steady-State Accuracy

- Increase `I_STEP` for faster I gain search
- Raise `MAX_I` limit
- Lower `TARGET_STEADY_STATE_ERROR` threshold

### Tuning Fails Immediately

**Check:**
- Motor is calibrated (`c` command first)
- Motor is enabled and can move freely
- Encoder is reading correctly (`s` command to check status)
- Power supply is adequate (12V, 2A+)
- No mechanical obstructions

### Compilation Errors

If you get compilation errors about `PIDAutoTuner`:

1. Ensure `pid_auto_tuner.h` and `pid_auto_tuner.cpp` are in the same directory as your sketch
2. For integration test, the include path is relative: `../../ESP32_MCU_Firmware/pid_auto_tuner.h`
3. Verify SimpleFOC library is installed

## Performance Expectations

With properly tuned PID values:

- **Overshoot**: < 0.1 rad (5.7°)
- **Settling Time**: 1-2 seconds
- **Steady-State Error**: < 0.01 rad (0.57°)
- **Repeatability**: ±0.02 rad (±1.1°)

## Safety Notes

1. **Motor Movement**: Tuning moves the motor through multiple positions
   - Ensure motor is securely mounted
   - Clear any obstructions
   - Adequate power supply (12V, 2A+)

2. **Emergency Stop**: Send `d` (disable) command to stop motor immediately

3. **Overshoot Protection**: Tuner automatically backs off if overshoot exceeds safety threshold

4. **Calibration Required**: Motor must be calibrated before PID tuning

## Files

| File | Description |
|------|-------------|
| `firmware/ESP32_MCU_Firmware/pid_auto_tuner.h` | PID auto-tuner class definition |
| `firmware/ESP32_MCU_Firmware/pid_auto_tuner.cpp` | PID auto-tuner implementation |
| `firmware/ESP32_MCU_Firmware/motor_control.h` | Motor controller with `autoTunePID()` method |
| `firmware/ESP32_MCU_Firmware/motor_control.cpp` | Motor controller implementation |
| `firmware/tests/integration_test/integration_test.ino` | Integration test with PID tuning command |
| `firmware/ESP32_MCU_Firmware/config.h` | Default PID configuration values |

## References

- [SimpleFOC Library](https://docs.simplefoc.com/)
- [PID Controller Theory](https://en.wikipedia.org/wiki/PID_controller)
- Motor: Mitoot 2804 (7 pole pairs, 100kV)
- Encoder: MT6701 14-bit magnetic encoder
- Driver: SimpleFOC Mini v1 (DRV8313)
