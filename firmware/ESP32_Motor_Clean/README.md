# ESP32 Motor Control - Clean Implementation

Minimal motor control using SimpleFOC without fighting its internal systems.

## Key Design Principles

### 1. Trust SimpleFOC's Rotation Tracking

SimpleFOC's `Sensor` base class already handles continuous angle tracking:
- `getSensorAngle()` returns raw 0-2π mechanical angle
- `update()` detects wraparound and maintains `full_rotations` counter
- `getAngle()` returns continuous angle: `full_rotations * 2π + angle`

**We don't override `motor.shaft_angle`** - SimpleFOC manages it.

### 2. Shortest-Path Target Calculation

For position commands, we:
1. Get current position from SimpleFOC (`motor.shaft_angle`)
2. Calculate shortest path error to target (±180°)
3. Add error to current position to get target in SimpleFOC's space

```cpp
float error = target_deg - current_wrapped;  // 0-360 space
if (error > 180) error -= 360;               // Shortest path
if (error < -180) error += 360;
float target_rad = current_rad + error;      // SimpleFOC space
motor.move(target_rad);
```

### 3. User Interface vs Internal Tracking

- **User sees:** 0-360° (from absolute encoder)
- **SimpleFOC uses:** Continuous angles (can be -1000° or +5000°)
- **Target calculation:** Bridges these two spaces

## Files

- `config.h` - Hardware pins and PID settings
- `motor_control.h` - MotorController class with MT6701Sensor
- `ESP32_Motor_Clean.ino` - Main sketch with serial commands

## Commands

| Command | Description |
|---------|-------------|
| `c` | Calibrate motor (required first) |
| `e` | Enable motor |
| `d` | Disable motor |
| `m <deg>` | Move to position (0-360°) |
| `s` | Show status |
| `diag` | Run diagnostic test |
| `h` | Help |

## What's Different from Original

| Original | Clean |
|----------|-------|
| Own continuous tracking | Trust SimpleFOC |
| Sync logic fighting SimpleFOC | No sync needed |
| Override `motor.shaft_angle` | Never touch it |
| ~1200 lines motor_control.cpp | ~150 lines total |

## SimpleFOC's Rotation Tracking (Reference)

From SimpleFOC's `Sensor.cpp`:
```cpp
void Sensor::update() {
    float val = getSensorAngle();
    float d_angle = val - angle_prev;
    if(abs(d_angle) > (0.8f*_2PI))
        full_rotations += (d_angle > 0) ? -1 : 1;
    angle_prev = val;
}

float Sensor::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}
```

## Troubleshooting

**Motor moves wrong direction:**
- Run calibration again (`c`)
- SimpleFOC detects direction automatically

**Motor doesn't move:**
- Check calibration completed
- Check encoder readings (`s`)
- Run `diag` for detailed test

**Position drifts over time:**
- SimpleFOC's `full_rotations` may accumulate if motor spun while disabled
- Re-calibrate to reset
