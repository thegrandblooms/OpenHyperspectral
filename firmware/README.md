# Firmware

| Folder | Status | Hardware |
|---|---|---|
| `ESP32_Stepper_Firmware/` | **CURRENT** — flash with `tools\flash_stepper.bat` | NEMA17 + TMC2209 + belt reduction + MT6835 21-bit encoder (output shaft) |
| `ESP32_MCU_Firmware/` | LEGACY (reference) — flash with `tools\flash.bat` | Mitoot 2804 gimbal + SimpleFOC Mini (DRV8313) + MT6701 encoder |
| `libraries/` | Vendored Arduino libraries (legacy firmware deps) | — |

Both firmwares speak the same serial protocol (text commands + `$ENC` tagged
CSV streaming + SerialTransfer binary protocol), so SpectrumBoi and the
`tools/` scripts work with either. The stepper firmware adds `scan_step` /
`scan_smooth` / `scan_stop` and drops the gimbal-only commands (`cogmap`,
`pidtune`, `tune`, `set pid_*`).

Why the switch: see `Dev_Log/motor/STATE.md` (decision, 2026-06-10) and
`ESP32_Stepper_Firmware/README.md` (new architecture + bring-up checklist).
Gimbal-era bring-up test sketches moved to `archive/firmware_tests_gimbal/`.
