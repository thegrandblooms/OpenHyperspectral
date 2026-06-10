# Integration Test for Motor Control System

This test verifies the complete motor control system with encoder feedback, including PID auto-tuning.

## Hardware Setup

- **Motor**: Mitoot 2804 (7 pole pairs, 100kV)
- **Driver**: SimpleFOC Mini v1 (DRV8313)
- **Encoder**: MT6701 14-bit magnetic encoder (I2C)
- **Power**: 12V, 2A+ supply
- **MCU**: ESP32

## Pin Connections

| Component | Pin |
|-----------|-----|
| Motor Enable | GPIO 15 |
| Motor PWM A | GPIO 13 |
| Motor PWM B | GPIO 11 |
| Motor PWM C | GPIO 12 |
| Encoder SDA | GPIO 47 |
| Encoder SCL | GPIO 48 |

## Upload Instructions

1. Open `integration_test.ino` in Arduino IDE
2. Select board: **ESP32 Dev Module** (or your specific ESP32 board)
3. Select correct COM port
4. Click Upload

**Note**: The sketch includes `pid_auto_tuner.h` and `pid_auto_tuner.cpp` - Arduino IDE will automatically compile these files as they're in the same directory.

## Usage Workflow

### 1. Open Serial Monitor
- Baud rate: **115200**
- Line ending: **Newline** or **Both NL & CR**

### 2. Recommended Workflow

Follow this sequence for best results:

```
Step 1: c         → Run motor/encoder calibration (REQUIRED)
Step 2: p         → Auto-tune PID parameters (HIGHLY RECOMMENDED)
Step 3: t         → Run automated position tests
```

### 3. Available Commands

| Command | Description |
|---------|-------------|
| `h` or `help` | Show help menu |
| `c` or `calibrate` | Run motor/encoder calibration |
| `p` or `pidtune` | Run PID auto-tuning |
| `e` or `enable` | Enable motor |
| `d` or `disable` | Disable motor (emergency stop) |
| `t` or `test` | Run automated test sequence |
| `s` or `status` | Show current motor/encoder status |
| `r` or `results` | Show test results |
| `m <angle>` | Move to angle in radians (e.g., `m 3.14`) |

## PID Auto-Tuning

The auto-tuning process:
1. Starts with conservative PID values (P=2.0)
2. Tests motor response at multiple positions (0°, 90°, 180°, 270°, 360°)
3. Incrementally increases P gain until optimal response
4. Adds D gain to reduce overshoot
5. Adds I gain to eliminate steady-state error
6. Applies optimal values automatically

**Duration**: 2-5 minutes depending on motor response

**Output**: Displays optimal PID values and instructions to update `config.h` for permanent storage.

## Safety Features

### Oscillation Detection
- Monitors motor position every 100ms
- Detects rapid back-and-forth movements
- Auto-disables motor if oscillation detected
- Requires recalibration after emergency stop

### Emergency Stop
- Press `d` at any time to immediately disable motor
- Oscillation detector acts as automatic safety net

### Default Conservative PID
- System starts with P=2.0 (very conservative)
- Prevents wild movements on first power-up
- May be slow - run PID tuning for optimal performance

## Troubleshooting

### "Commands not recognized"
- Check Serial Monitor line ending is set to **Newline** or **Both NL & CR**
- Look for `[CMD] Received:` debug output showing what was received
- Commands are case-insensitive (`P`, `p`, `PIDTUNE` all work)

### "Motor oscillates wildly"
- Emergency stop: Press `d`
- Run PID auto-tuning: `p`
- Check mechanical connections for obstructions
- Verify encoder is reading correctly: `s` for status

### "Calibration fails"
- Check encoder I2C connections (SDA=GPIO47, SCL=GPIO48)
- Verify motor driver connections and power supply
- Check motor enable pin (GPIO15)
- View detailed debug output in Serial Monitor

### "PID tuning fails"
- Ensure motor is calibrated first: `c`
- Check motor can move freely (no obstructions)
- Verify adequate power supply (12V, 2A+)
- Try lowering initial P value if motor is very responsive

### "Test sequence fails"
- Run PID tuning first: `p`
- Conservative default PID may be too slow for tests
- Check position tolerance settings (currently 0.1 rad)

## Expected Performance

With properly tuned PID:
- **Overshoot**: < 0.1 rad (5.7°)
- **Settling time**: 1-2 seconds
- **Steady-state error**: < 0.01 rad (0.57°)
- **Repeatability**: ±0.02 rad (±1.1°)

## Files in this Directory

| File | Description |
|------|-------------|
| `integration_test.ino` | Main test sketch |
| `pid_auto_tuner.h` | PID auto-tuner class definition |
| `pid_auto_tuner.cpp` | PID auto-tuner implementation |
| `README.md` | This file |

**Note**: `pid_auto_tuner.*` files are copies from `../ESP32_MCU_Firmware/` for Arduino IDE compatibility.

## Making PID Values Permanent

After successful tuning, the test will display:

```
To make these values permanent, update config.h:
  #define PID_P_POSITION 6.40
  #define PID_I_POSITION 0.20
  #define PID_D_POSITION 0.150
```

Edit `firmware/ESP32_MCU_Firmware/config.h` with these values, then recompile/upload your main firmware.

## Example Session

```
> c
[Calibration runs...]
CALIBRATION SUCCESSFUL!
Run 'p' to auto-tune PID...

> p
[PID tuning runs for 2-3 minutes...]
TUNING COMPLETE!
Optimal PID: P=6.40, I=0.20, D=0.150

> t
[Test sequence runs...]
Test 1/7: Moving to 0.00 rad... PASS
Test 2/7: Moving to 1.57 rad... PASS
...
Tests Passed: 7 / 7
Average Error: 0.0067 rad (0.38°)
```

## Support

For issues or questions, check:
- Firmware documentation: `firmware/PID_TUNING_FIRMWARE.md`
- Main motor control code: `firmware/ESP32_MCU_Firmware/motor_control.cpp`
- SimpleFOC documentation: https://docs.simplefoc.com/
