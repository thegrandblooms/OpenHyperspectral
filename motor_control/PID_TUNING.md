# PID Auto-Tuning for Motor Position Control

This document describes the PID auto-tuning system for optimizing motor position tracking performance in the OpenHyperspectral project.

## Overview

The PID auto-tuning system automatically finds optimal PID parameters for motor position control by:
1. Starting with conservative (low) PID gains to prevent overshoot
2. Testing motor response at multiple target positions
3. Incrementally increasing gains while monitoring performance
4. Detecting and preventing overshoot conditions
5. Optimizing for minimal overshoot, fast settling, and low steady-state error

## Quick Start

### Option 1: Integrated Calibration Workflow (Recommended)

Run the complete calibration with PID tuning:

```bash
cd motor_control
python calibrate_with_pid_tuning.py
```

This will:
- Connect to the motor controller
- Run motor calibration if needed
- Automatically tune PID parameters
- Apply the optimized values
- Save results to `pid_tuning_results.json`

### Option 2: Standalone PID Tuning

If your motor is already calibrated, run PID tuning only:

```bash
cd motor_control
python pid_tuner.py
```

### Option 3: Programmatic Usage

Use the API in your own scripts:

```python
from controller import MotorController

# Connect and calibrate
controller = MotorController()
controller.connect()
controller.calibrate()

# Run PID auto-tuning
results = controller.auto_tune_pid(apply_results=True)

if results:
    print(f"Optimal PID: P={results['optimal_p']:.2f}, "
          f"I={results['optimal_i']:.2f}, "
          f"D={results['optimal_d']:.3f}")
```

## How It Works

### Tuning Algorithm

The auto-tuner uses a three-phase sequential tuning approach:

#### Phase 1: Proportional Gain (P)
- Starts with low P gain (default: 2.0)
- Incrementally increases P by steps of 2.0
- Tests motor response at each value
- Stops when overshoot is detected or score degrades
- Uses 80% of best P value for stability margin

#### Phase 2: Derivative Gain (D)
- Fixes P at optimal value from Phase 1
- Incrementally increases D by steps of 0.05
- D gain reduces overshoot and oscillation
- Stops when improvement is less than 5%

#### Phase 3: Integral Gain (I)
- Fixes P and D at optimal values
- Incrementally increases I by steps of 0.1
- I gain eliminates steady-state error
- Stops when target error is achieved or score degrades

### Performance Metrics

Each PID configuration is evaluated using:

**Overshoot**: Peak position error beyond target
- Maximum allowed: 0.1 rad (5.7°)
- Weighted heavily in scoring (10x multiplier)

**Settling Time**: Time to enter and stay within tolerance
- Target: < 3.0 seconds
- Weighted moderately (2x multiplier)

**Steady-State Error**: Final position error after settling
- Target: < 0.01 rad (0.57°)
- Weighted heavily (5x multiplier)

**Rise Time**: Time to reach 90% of target
- Weighted lightly (0.5x multiplier)

### Test Positions

The tuner tests movements to multiple positions to ensure consistent performance:
- 0.0 rad (0°)
- 1.57 rad (90°)
- 3.14 rad (180°)
- 4.71 rad (270°)
- 0.0 rad (0° - full rotation)

## Configuration

### Tuning Parameters

You can customize tuning behavior by modifying `PIDTuner` class constants in `pid_tuner.py`:

```python
# Test positions (radians)
TEST_POSITIONS = [0.0, 1.57, 3.14, 4.71, 0.0]

# Position tolerance for movement completion
POSITION_TOLERANCE = 0.05  # rad (~2.9°)

# Settling tolerance
SETTLING_TOLERANCE = 0.02  # rad (~1.1°)

# Initial PID values (conservative start)
INITIAL_P = 2.0
INITIAL_I = 0.0
INITIAL_D = 0.0

# Increment steps
P_STEP = 2.0
D_STEP = 0.05
I_STEP = 0.1

# Maximum values
MAX_P = 50.0
MAX_I = 5.0
MAX_D = 2.0

# Performance thresholds
MAX_OVERSHOOT = 0.1  # rad (5.7°)
MAX_SETTLING_TIME = 3.0  # seconds
TARGET_STEADY_STATE_ERROR = 0.01  # rad (0.57°)
```

## Command-Line Options

### calibrate_with_pid_tuning.py

```bash
python calibrate_with_pid_tuning.py [OPTIONS]

Options:
  --port PORT          Serial port for motor controller
  --skip-tuning        Skip PID auto-tuning
  --save-results FILE  File to save results (default: pid_tuning_results.json)
  -v, --verbose        Enable debug logging
```

### pid_tuner.py

```bash
python pid_tuner.py [OPTIONS]

Options:
  --port PORT          Serial port for motor controller
  -v, --verbose        Enable debug logging
```

## Output

### Console Output

The tuner provides detailed progress information:

```
============================================================
STARTING PID AUTO-TUNING
============================================================
Test positions: [0.0, 1.57, 3.14, 4.71, 0.0]
Position tolerance: 0.0500 rad
Max overshoot allowed: 0.1000 rad

============================================================
PHASE 1: Tuning Proportional Gain (P)
============================================================
Setting PID: P=2.00, I=0.00, D=0.00, Ramp=1000.0
Test movement 1/5: target=0.00 rad
Movement score: 8.45 (overshoot=0.0012, settle=1.23s, error=0.0034)
...
New best P gain: 4.00 (score: 7.21)
...
Optimal P gain (with 20% margin): 6.40

============================================================
PHASE 2: Tuning Derivative Gain (D)
============================================================
...

============================================================
TUNING COMPLETE
============================================================
Optimal PID Parameters:
  P = 6.40
  I = 0.20
  D = 0.150
  Ramp = 1000.0

Performance Metrics:
  Average overshoot: 0.0023 rad (0.13°)
  Maximum overshoot: 0.0045 rad (0.26°)
  Average settling time: 1.45 s
  Average steady-state error: 0.0067 rad (0.38°)
  Final score: 5.32
============================================================

COPY THESE VALUES TO config.h:
============================================================
#define PID_ANGLE_P 6.40f
#define PID_ANGLE_I 0.20f
#define PID_ANGLE_D 0.150f
============================================================
```

### JSON Results File

Results are saved to `pid_tuning_results.json`:

```json
{
  "optimal_p": 6.4,
  "optimal_i": 0.2,
  "optimal_d": 0.15,
  "ramp": 1000.0,
  "final_score": 5.32,
  "avg_overshoot": 0.0023,
  "max_overshoot": 0.0045,
  "avg_settling_time": 1.45,
  "avg_steady_state_error": 0.0067
}
```

## Applying Tuned Values

### Temporary Application (Current Session)

The tuned values are automatically applied by default and will remain active until:
- Motor is power-cycled
- Firmware is reset
- Different PID values are set

### Permanent Application (Firmware)

To make tuned values permanent, update the firmware configuration:

1. Copy the PID values from the tuner output
2. Edit `firmware/ESP32_MCU_Firmware/config.h`
3. Update these lines:

```cpp
// PID Parameters - Position Controller
#define PID_ANGLE_P 6.40f   // Update with tuned value
#define PID_ANGLE_I 0.20f   // Update with tuned value
#define PID_ANGLE_D 0.150f  // Update with tuned value
#define PID_ANGLE_RAMP 1000.0f
```

4. Recompile and upload firmware
5. Run calibration (PID tuning not needed after firmware update)

## Integration with Test Scripts

### Python Tests

```python
from controller import MotorController

def test_with_tuning():
    controller = MotorController()
    controller.connect()

    # Calibrate motor
    controller.calibrate()

    # Auto-tune PID
    results = controller.auto_tune_pid(apply_results=True)

    # Now run your tests with optimized PID
    controller.move_to(1.57)  # Move to 90°
    # ... rest of test

    controller.disconnect()
```

### Integration Test (Arduino)

For the Arduino integration test, you can add a Python wrapper:

```python
from controller import MotorController

# Connect and tune
controller = MotorController()
controller.connect()
controller.calibrate()

# Run PID tuning
results = controller.auto_tune_pid(apply_results=True)

# Now the integration test will use optimized PID values
# Continue with your Arduino-based testing...
```

## Troubleshooting

### Tuning Takes Too Long

- Reduce the number of test positions
- Increase step sizes (P_STEP, D_STEP, I_STEP)
- Reduce maximum gain limits

### Excessive Overshoot

- Lower MAX_OVERSHOOT threshold
- Increase stability margin (use 70% instead of 80% of best P)
- Reduce P_STEP for finer control

### Motor Oscillates

- Increase D gain step size to find damping faster
- Lower maximum P gain limit
- Add more aggressive overshoot detection

### Poor Steady-State Accuracy

- Increase I gain step size
- Raise MAX_I limit
- Lower TARGET_STEADY_STATE_ERROR threshold

### Connection Issues

```bash
# List available serial ports
python -m serial.tools.list_ports

# Specify port explicitly
python pid_tuner.py --port /dev/ttyUSB0
```

## Performance Expectations

With properly tuned PID values, you should achieve:

- **Overshoot**: < 0.1 rad (5.7°)
- **Settling Time**: 1-2 seconds
- **Steady-State Error**: < 0.01 rad (0.57°)
- **Repeatability**: ±0.02 rad (±1.1°)

## Safety Notes

1. **Motor Movement**: The tuning process will move the motor through multiple positions. Ensure:
   - Motor is securely mounted
   - No obstructions in movement path
   - Power supply is adequate (12V, 2A+)

2. **Emergency Stop**: Press Ctrl+C to interrupt tuning at any time

3. **Overshoot Protection**: The tuner automatically backs off if overshoot exceeds safety threshold

4. **Calibration Required**: Motor must be calibrated before PID tuning

## Advanced Usage

### Custom Test Positions

```python
from pid_tuner import PIDTuner

# Create custom tuner with specific test positions
tuner = PIDTuner(controller)
tuner.TEST_POSITIONS = [0.0, 0.785, 1.57, 2.356, 3.14]  # 45° increments
results = tuner.run_tuning()
```

### Manual PID Testing

```python
# Test specific PID values
p, i, d = 10.0, 0.5, 0.2
score, test_results = tuner.evaluate_pid(p, i, d, 1000.0)

print(f"Score: {score}")
for result in test_results:
    print(f"Target: {result['target']:.2f}, "
          f"Overshoot: {result['overshoot']:.4f}, "
          f"Settle: {result['settling_time']:.2f}s")
```

### Logging

Enable verbose logging for detailed debugging:

```bash
python pid_tuner.py --verbose
```

This provides step-by-step details including:
- Position samples during each movement
- Detailed performance calculations
- Real-time motor status updates

## References

- [PID Controller Theory](https://en.wikipedia.org/wiki/PID_controller)
- [Ziegler-Nichols Method](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method)
- [SimpleFOC Library Documentation](https://docs.simplefoc.com/)
- Motor Controller API: `controller.py`
- Firmware Configuration: `firmware/ESP32_MCU_Firmware/config.h`
