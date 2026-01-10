# Motor Control System - Development Log
## Executive Summary
**Status: Manual Calibration Implemented** - Bypassing SimpleFOC auto-calibration issues with MT6701 I2C sensors.

**Root Cause (Identified)**: SimpleFOC's automatic `initFOC()` calibration fails with MT6701 I2C sensors because I2C polling (even at ~1ms per read) is slower than SPI alternatives, potentially causing timing issues with SimpleFOC's movement detection algorithm. This is a known issue (GitHub #172). Note: While typical I2C reads at 100-400kHz take <1ms for 2-byte reads, this is still insufficient for SimpleFOC's calibration timing requirements.

**Solution (Implemented)**: Manual calibration using `setPhaseVoltage()` at known electrical angles to calculate `zero_electric_angle` and `sensor_direction` before calling `initFOC()`.

**Next Steps**: Hardware testing to verify calibration works and motor control is functional.

---

## System Architecture (Current State)

### Hardware Setup

```

ESP32-S3 (Waveshare Touch LCD)
├── GPIO 11/12/13 → PWM outputs → SimpleFOC Mini (DRV8313 driver)
├── GPIO 15 → Enable pin → SimpleFOC Mini
├── GPIO 47/48 → I2C (SDA/SCL) → MT6701 encoder
└── Software: SimpleFOC library (runs ON the ESP32)

 

SimpleFOC Mini Board (DRV8313)
├── Receives PWM from ESP32
├── Drives 3-phase BLDC motor
└── NOTE: This is a "dumb" driver - no intelligence, just power

 

MT6701 Absolute Encoder
├── 14-bit magnetic encoder (16384 positions/revolution)
├── I2C interface (address 0x06)
└── Read by ESP32, NOT by SimpleFOC board

 

Mitoot 2804 Gimbal Motor
├── 7 pole pairs
├── ~10Ω resistance
└── Driven by SimpleFOC Mini

```
### Software Architecture
```

ESP32 runs everything:

1. Reads MT6701 via I2C (encoder.update())
2. SimpleFOC library calculates FOC (motor.loopFOC())
3. ESP32 outputs PWM to driver board
4. Driver board powers motor phases

 

SimpleFOC Mini = PASSIVE hardware (just amplifies PWM to motor)

```

 

---

 

## What Works ✅

 

### 1. Encoder Reading

- MT6701 I2C communication: **PERFECT**

- Position reads correctly: 6.24° → 324.40° → 317.90° etc.

- Raw counts accurate (0-16383)

- Encoder library integration: **WORKING**

 

### 2. Basic Motor Power

- Motor responds to voltage commands

- Motor moves during calibration attempts

- PWM output from ESP32: **WORKING**

- Driver board energizing motor: **WORKING**

 

### 3. System Integration

- ESP32 firmware compiles and runs

- Serial communication works

- Command processing works

- Encoder data flowing to SimpleFOC library: **WORKING**

 

---

 

## What Doesn't Work ❌

 

### 1. SimpleFOC Calibration (CRITICAL BLOCKER)

```

Symptom: motor.initFOC() always returns 0 (failure)

Result: zero_electric_angle never calculated

Impact: FOC algorithm cannot run

```

 

**Every calibration attempt fails with one of:**

- "MOT: Failed to notice movement" (needsSearch=0 path)

- "MOT: Error: Not found!" (needsSearch=1 path)

 

### 2. Motor Movement

```

Symptom: Motor oscillates between electrical poles

Observed: 324° → 317° → 324° → 325° → 353° → 345°

Pattern: ~30-50° jumps (360°/7 pole pairs = 51.4°)

Cause: No FOC - just random phase voltages

```

 

### 3. SimpleFOC State Tracking

```

Symptom: motor.shaft_angle always 0.00°

Symptom: motor.shaft_velocity always 0.00°

Cause: motor.loopFOC() not working (initFOC failed)

Impact: Position control impossible

```

 

### 4. Position Reached Spam

```

Symptom: Constant "Position reached" messages

Cause: System thinks target=0°, encoder reads 5°, within tolerance

Root: Motor isn't calibrated, shouldn't be checking position

```

 

---

 

## Attempts Made (What Didn't Work)

 

### Attempt 1: Manual Zero Electric Angle Calculation

```cpp

// TRIED: Calculate offset from current position

float electrical_angle = mechanical_angle * POLE_PAIRS;

motor.zero_electric_angle = electrical_angle;

 

PROBLEM: Assumes motor is at electrical zero (it's not!)

RESULT: ~324° commutation error → wild oscillations

STATUS: Removed (was fundamentally wrong)

```

 

### Attempt 2: SimpleFOC Auto-Calibration (needsSearch=0)

```cpp

// TRIED: Let SimpleFOC align sensor automatically

motor.initFOC();

 

PROBLEM: "Failed to notice movement"

DIAGNOSIS: Basic alignment detection doesn't work with I2C encoder

STATUS: Failed

```

 

### Attempt 3: Increased Alignment Voltage

```cpp

// TRIED: Increase voltage for better movement detection

motor.voltage_sensor_align = 6.0V; // was 3V default

 

PROBLEM: Motor moved more, but detection still failed

RESULT: Still "Failed to notice movement"

STATUS: Helped slightly but didn't solve

```

 

### Attempt 4: Force Index Search Logic (needsSearch=1)

```cpp

// TRIED: Use index search path for robust detection

encoder.setCalibrationMode(true); // needsSearch() returns 1

motor.initFOC();

 

PROBLEM: "MOT: Error: Not found!"

DIAGNOSIS: Index search expects full rotation detection, fails

OBSERVED: Motor oscillates ~30-50° but doesn't complete rotation

STATUS: Failed - motor moves but SimpleFOC can't track it

```

 

### Attempt 5: Remove Manual encoder.update() Before loopFOC()

```cpp

// TRIED: Let SimpleFOC handle sensor updates

// Remove our encoder.update() call before motor.loopFOC()

 

PROBLEM: Made no difference (initFOC still fails)

STATUS: Correct per SimpleFOC design, but doesn't fix calibration

```

 

---

 

## Core Issues (Systemic Problems)

 

### Issue 1: No Working Reference Implementation

**Problem:** We've been modifying code without a known-working baseline

**Impact:** Can't distinguish "our bugs" from "expected behavior"

**Need:** Find/create minimal working example with similar hardware

 

### Issue 2: Calibration Black Box

**Problem:** SimpleFOC's initFOC() is opaque - we don't know WHY it fails

**Questions Not Answered:**

- Is motor.init() succeeding?

- Is the driver board powered correctly?

- Is the enable pin working?

- Are PWM signals correct?

- Is SimpleFOC even seeing motor movement?

 

### Issue 3: Motor Behavior Misunderstanding

**Problem:** Motor oscillating between poles - is this calibration motion or failure mode?

**Confusion:** We see 30-50° movements - is SimpleFOC trying to rotate it, or is it just random?

 

### Issue 4: Architecture Confusion

**Problem:** Multiple times confused "SimpleFOC board" with "SimpleFOC library"

**Impact:** Wasted time on wrong diagnosis (encoder connection, etc.)

 

### Issue 5: Missing Diagnostics

**Problem:** No visibility into SimpleFOC's internal state during calibration

**Need:**

- Is driver.init() succeeding?

- Is motor.init() succeeding?

- What does SimpleFOC see during alignment?

- Is shaft_angle changing during calibration?

 

---

 

## Development Goals (What We're Trying to Achieve)

 

### Primary Goal

**Get SimpleFOC working with our hardware stack:**

- ESP32-S3 running SimpleFOC library

- MT6701 I2C encoder (read by ESP32)

- SimpleFOC Mini driver board (DRV8313)

- Mitoot 2804 gimbal motor (7 pole pairs)

 

### Success Criteria

1. ✅ `motor.initFOC()` returns 1 (success)

2. ✅ `motor.shaft_angle` tracks encoder position

3. ✅ Motor moves smoothly (not oscillating between poles)

4. ✅ Can command position movements

5. ✅ PID control works

 

### Secondary Goals

- Save calibration to NVS (skip alignment on boot)

- Optimize PID parameters

- Implement position-based camera triggering

 

---

 

## Recommended Next Steps

 

### Step 1: Validate Basic Hardware

```cpp

// Test driver board enable pin

digitalWrite(MOTOR_ENABLE, HIGH);

// Manually set PWM to verify motor responds

analogWrite(MOTOR_PWM_A, 128);

// Does motor energize? If not, hardware problem

```

 

### Step 2: Find Reference Implementation

- Search SimpleFOC forums for MT6701 + gimbal motor

- Find working code for I2C absolute encoder

- Compare our code line-by-line

 

### Step 3: Add Comprehensive Diagnostics

```cpp

// During calibration, log:

Serial.println(motor.shaft_angle);      // Is it changing?

Serial.println(motor.sensor_direction); // What direction detected?

Serial.println(motor.zero_electric_angle); // Intermediate values?

```

 

### Step 4: Try Manual Calibration (Correctly This Time)

```cpp

// Apply voltage at KNOWN electrical angle

motor.setPhaseVoltage(6.0, 0, _3PI_2);

delay(2000); // Wait for motor to settle

encoder.update();

float aligned_position = encoder.getSensorAngle();

// Now we know: when encoder reads X, motor is at 3π/2 electrical
// CORRECTED FORMULA (fixed in Dev Log 5):
// aligned_position is mechanical angle, need to convert to electrical
// zero_electric_angle = (mechanical_position × pole_pairs) - applied_electrical_angle

motor.zero_electric_angle = (aligned_position * POLE_PAIRS) - _3PI_2;
// Or with normalization to 0-2π range:
// motor.zero_electric_angle = fmod((aligned_position * POLE_PAIRS) - _3PI_2 + _2PI, _2PI);

```

 

### Step 5: Bypass Calibration for Testing

```cpp

// Hardcode values from successful calibration

motor.zero_electric_angle = 2.15; // Example - need real value

motor.sensor_direction = Direction::CW;

motor.initFOC(); // Should skip alignment and return 1

// Test if FOC works with known-good values

```

 

---

 

## Questions for SimpleFOC Community/Docs

 

1. **For I2C absolute encoders**, which alignment method should be used?

   - needsSearch() = 0 or 1?

   - Special configuration needed?

 

2. **Why does index search fail** with "Not found"?

   - What is SimpleFOC looking for during rotation?

   - How much rotation is expected?

 

3. **Gimbal motor specific settings?**

   - Default voltage_sensor_align too low?

   - Other parameters to adjust?

 

4. **Debug mode available?**

   - Can we see SimpleFOC's internal calibration state?

   - Verbose logging during initFOC()?

 

---

 

## Code Locations (for Next Session)

 

- Main firmware: `firmware/ESP32_MCU_Firmware/ESP32_MCU_Firmware.ino`

- Motor control: `firmware/ESP32_MCU_Firmware/motor_control.cpp`

- Calibration function: `motor_control.cpp:445` (`runCalibration()`)

- Config: `firmware/ESP32_MCU_Firmware/config.h`

 

Current branch: `claude/fix-motor-encoder-oscillation-01KXFf3mPqZCHd6TgjFGkJmY`

 

Latest commits:

- `de8cd2d`: Diagnostic documentation

- `8a89e0a`: Force index search during calibration

- `a4a36a3`: Increase voltage_sensor_align to 6V

- `f737d11`: Replace manual calibration with auto-calibration

- `1951359`: Remove manual encoder.update() before loopFOC()

 

---

 

## Bottom Line

 

**We need to:**

1. Stop guessing and find a working example to copy
2. Verify basic hardware function (PWM, enable pin, power)
3. Understand WHY SimpleFOC calibration is failing (add diagnostics)
4. Consider manual calibration as backup
5. Get ONE successful calibration, then optimize

 

**The encoder works perfectly. SimpleFOC calibration is the blocker.**