# OpenHyperspectral

An open source, modular **1D line-scanning hyperspectral imaging system** with
synchronized motor control.

## Overview

A pushbroom hyperspectral scanner: a motorized rotary stage sweeps the imaging
line across the scene while a camera captures spectra, and frames are matched
to encoder positions to build the data cube.

```
Main Computer ── SpectrumBoi UI (spectrumboi.py)
     │
     ├── USB ──> Mightex monochrome camera (spectral line capture)
     │
     └── USB ──> ESP32-S3-Touch-LCD-2 (Waveshare)
                      │
                      ├── STEP/DIR + UART ──> TMC2209 ──> NEMA17 stepper
                      │                                       │
                      │                                  belt reduction
                      │                                       │
                      └────────── SPI ◄── MT6835 21-bit ◄── output shaft
                                          absolute encoder
```

**Motor architecture (since 2026-06):** NEMA17 stepper + TMC2209 (StealthChop,
1/16 µsteps with 1/256 interpolation) through a belt reduction, with an MT6835
21-bit absolute encoder on the **output shaft** for closed-loop position trim.
Moves execute open-loop, then the encoder trims residual error (belt lash,
compliance) to ≤0.02°; holding is passive, so there is no hunting at rest.

> The previous design (Mitoot 2804 gimbal motor + SimpleFOC) could not meet
> the ≤0.1° precision requirement: iron-core cogging produced limit cycles at
> scan speeds and the MT6701's ±1° INL capped absolute accuracy. The full
> investigation is in `Dev_Log/motor/`; the gimbal-era code lives on in
> `archive/` and `firmware/ESP32_MCU_Firmware/`.

## Scan modes

Two capture strategies are built into the firmware (both under evaluation —
see `firmware/ESP32_Stepper_Firmware/README.md`):

- **Stepped capture** (`scan_step <start> <end> <inc> [dwell_ms]`): settle +
  encoder-trim at each position, emit a `$FRAME` marker, dwell for capture.
- **Smooth video scan** (`scan_smooth <start> <end> <deg/s>`): constant-rate
  sweep with `$ENC` position streaming; record video and frame-match
  afterwards using the `sync` clock handshake.

Both are also driveable from the SpectrumBoi UI (Motor Control → Scan).

## Repository layout

```
OpenHyperspectral/
├── spectrumboi.py              # Main UI: camera preview, spectra, motor control
├── camera_control/             # Mightex camera streaming + driver
├── firmware/
│   ├── ESP32_Stepper_Firmware/ # CURRENT firmware (stepper + MT6835) — see its README
│   ├── ESP32_MCU_Firmware/     # LEGACY gimbal/SimpleFOC firmware (reference)
│   └── libraries/              # Vendored Arduino libraries
├── tools/
│   ├── flash_stepper.bat       # Compile + upload CURRENT firmware (COM3 default)
│   ├── flash.bat               # Compile + upload legacy gimbal firmware
│   ├── motor_test.py           # Serial automation / test presets
│   ├── sweep_validation.py     # Sweep smoothness analysis from $ENC logs
│   └── test_scan_commands.py   # sync / sweep / sweep_stop checks
├── Dev_Log/                    # Development state + history (motor, scan, datacube)
├── archive/                    # Gimbal-era code and tuning data (see its README)
├── references/                 # Third-party reference implementations
└── logs/                       # Generated encoder logs (gitignored)
```

## Getting started

### Flash the firmware

Requires `tools/arduino-cli.exe` (run `tools/install_arduino_cli.ps1` once)
and the TMCStepper + SerialTransfer Arduino libraries.

```bat
tools\flash_stepper.bat          :: COM3 default; pass another port as arg
```

Before first power-up, work through the bring-up checklist in
`firmware/ESP32_Stepper_Firmware/README.md` (pin assignments, GEAR_RATIO,
motor current, encoder verification).

### Run the UI

```bash
pip install -r requirements.txt
python spectrumboi.py
```

Motor Control section: connect to the COM port, then `c` (calibrate), `e`
(enable), `m 90` (move). The Scan and Motor Settings sub-panels expose the
scan modes and firmware parameters.

### Command-line testing

```bash
py -3.9 tools/motor_test.py --preset baseline   # full T1-T5 diagnostic
py -3.9 tools/sweep_validation.py               # sweep smoothness metrics
```

## Development log

`Dev_Log/` holds per-subsystem `STATE.md` (current status + next steps) and
`HISTORY.md` (what was tried and why it failed — read before re-proposing an
approach). The motor log documents the full gimbal→stepper decision.

## License

See [LICENSE](LICENSE) file.
