# Archive — gimbal-motor era (superseded 2026-06)

Everything here belongs to the retired **Mitoot 2804 gimbal motor + SimpleFOC**
motor architecture. It was replaced by the stepper design
(`firmware/ESP32_Stepper_Firmware/`) because the gimbal path could not meet
the ≤0.1° output-shaft precision requirement — full rationale in
`Dev_Log/motor/STATE.md`.

Kept for reference; nothing in the live codebase imports from here.

| Folder | What it is |
|---|---|
| `motor_control_gimbal/` | Python package: serial controller, Optuna PID auto-tuner (needs `optuna`), stream logger, scan pseudocode. Was imported by SpectrumBoi's PID-tune panel (removed). |
| `firmware_tests_gimbal/` | Arduino bring-up sketches for the gimbal hardware (I2C scanner, MT6701 encoder test, DRV8313 driver fault test, loop timing, etc.). |
| `tools_gimbal/` | Cogging-map scripts (`run_cogmap.py`, `run_cogmap_dense.py`) — the firmware commands they drive only exist in the gimbal firmware. |
| `tuning_logs/` | Raw data from the tuning campaign: `sweep_validation_*.csv`, cogmap CSVs, PID tuning report HTML. Referenced by the analyses in `Dev_Log/motor/`. |

The legacy firmware itself stays at `firmware/ESP32_MCU_Firmware/` so it
remains flashable (`tools/flash.bat`) while the gimbal hardware still exists.
