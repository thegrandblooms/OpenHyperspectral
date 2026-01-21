# Integration Test for Motor Control System

This test verifies the complete motor control system with encoder feedback.

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

## Usage Workflow

### 1. Open Serial Monitor
- Baud rate: **115200**
- Line ending: **Newline** or **Both NL & CR**

### 2. Recommended Workflow

Follow this sequence for best results:

```
Step 1: c         → Run motor/encoder calibration (REQUIRED)
Step 2: e         → Enable motor
Step 3: t         → Run automated position tests
```

### 3. PID Tuning (via Python)

For PID tuning, use the Python script which provides better analysis and visualization:

```bash
# Install dependencies
pip install pyserial matplotlib numpy

# Run PID tuning
python motor_control/pid_tuner.py /dev/ttyUSB0 --mode autotune --plot
```

Available tuning modes:
- `step` - Step response analysis with plotting
- `autotune` - Ziegler-Nichols auto-tuning
- `sweep` - Parameter sweep optimization
- `manual` - Interactive tuning

### 4. Available Commands

| Command | Description |
|---------|-------------|
| `h` or `help` | Show help menu |
| `c` or `calibrate` | Run motor/encoder calibration |
| `e` or `enable` | Enable motor |
| `d` or `disable` | Disable motor (emergency stop) |
| `t` or `test` | Run automated test sequence |
| `s` or `status` | Show current motor/encoder status |
| `r` or `results` | Show test results |
| `m <angle>` | Move to angle in degrees (e.g., `m 90`) |
| `pid 0 <P> <I> <D> <ramp>` | Set position PID parameters |

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
- System starts with P=20.0, I=0.0, D=0.0 (config.h defaults)
- Run PID tuning for optimal performance

## Troubleshooting

### "Commands not recognized"
- Check Serial Monitor line ending is set to **Newline** or **Both NL & CR**
- Look for `[CMD] Received:` debug output showing what was received
- Commands are case-insensitive

### "Motor oscillates wildly"
- Emergency stop: Press `d`
- Reduce P gain or add D gain
- Check mechanical connections for obstructions
- Verify encoder is reading correctly: `s` for status

### "Calibration fails"
- Check encoder I2C connections (SDA=GPIO47, SCL=GPIO48)
- Verify motor driver connections and power supply
- Check motor enable pin (GPIO15)
- View detailed debug output in Serial Monitor

### "Test sequence fails"
- Run PID tuning first via Python script
- Check position tolerance settings (currently 0.5°)

## Expected Performance

With properly tuned PID:
- **Overshoot**: < 5°
- **Settling time**: 1-2 seconds
- **Steady-state error**: < 0.5°
- **Repeatability**: ±1°

## Files in this Directory

| File | Description |
|------|-------------|
| `integration_test.ino` | Main test sketch |
| `README.md` | This file |

## Making PID Values Permanent

After tuning with the Python script, update `firmware/ESP32_MCU_Firmware/config.h`:

```cpp
#define PID_P_POSITION   <tuned_value>
#define PID_I_POSITION   <tuned_value>
#define PID_D_POSITION   <tuned_value>
#define PID_RAMP_POSITION_DEG <tuned_value>
```

Then recompile and upload the firmware.

## Support

For issues or questions, check:
- Main motor control code: `firmware/ESP32_MCU_Firmware/motor_control.cpp`
- PID tuning script: `motor_control/pid_tuner.py`
- SimpleFOC documentation: https://docs.simplefoc.com/
