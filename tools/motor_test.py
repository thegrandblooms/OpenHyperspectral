#!/usr/bin/env python3
# Run with: py -3.9 motor_test.py [args]
"""
Motor controller serial automation.
Sends commands to the ESP32, captures output, saves to last_run.txt.
Claude reads last_run.txt to analyze results without copy-paste.

Usage:
  python motor_test.py calibrate enable test
  python motor_test.py "m 90" status
  python motor_test.py --port COM4 calibrate enable sweep
  python motor_test.py --stream 10    # capture streaming data for 10 seconds
  python motor_test.py --interactive  # interactive passthrough (human in the loop)

Preset sequences:
  python motor_test.py --preset baseline
  python motor_test.py --preset oscillation
  python motor_test.py --preset sweep
"""

import serial
import time
import sys
import argparse
import os
from datetime import datetime

# Fix Windows console encoding (ESP32 outputs ✓ and similar chars)
if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

# ── Config ────────────────────────────────────────────────────────────────────
PORT    = "COM3"
BAUD    = 115200
OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Per-command done markers — script stops reading when it sees one of these.
# Keep these specific so internal sub-steps don't prematurely end capture.
# Strings match the STEPPER firmware (firmware/ESP32_Stepper_Firmware).
DONE_MARKERS = {
    "test":          ["RESULT: ALL TESTS PASSED", "RESULT: PARTIAL", "RESULT: FAILED"],
    "diag":          ["RESULT: ALL TESTS PASSED", "RESULT: PARTIAL", "RESULT: FAILED"],
    "position_sweep":["Result:"],                       # "Result: N/5 PASS"
    "motor_test":    ["PASS", "FAIL"],
    "encoder_test":  ["Done. CRC errors:"],
    "calibrate":     ["Calibrating... OK", "Calibrating... FAILED"],
    "c":             ["Calibrating... OK", "Calibrating... FAILED"],
    "recalibrate":   ["OK", "FAILED"],
    "enable":        ["Motor enabled"],   # stop before streaming floods the buffer
    "e":             ["Motor enabled"],
    "m":             ["AT_TARGET", "[TRIM] Gave up", "not enabled"],
    "status":        ["Trim:"],                         # last line of printStatus
    "s":             ["Trim:"],
}

# Per-command timeouts in seconds (generous — T5 visits 24 positions)
TIMEOUTS = {
    "test":           300,
    "diag":           300,
    "position_sweep":  90,
    "calibrate":       30,
    "c":               30,
    "recalibrate":     60,
    "enable":          10,
    "e":               10,
    "motor_test":      20,
    "m":               20,
    "status":           5,
    "s":                5,
    "info":             3,
    "i":                3,
    "default":         10,
}

# ── Preset sequences ──────────────────────────────────────────────────────────
PRESETS = {
    # Baseline: full system diagnostic (T1-T5, encoder-verified accuracy)
    "baseline": ["debug 0", "calibrate", "test"],

    # Hold characterization: move to 4 positions, check status at each.
    # Pair with --stream to capture $ENC, data during holds.
    "oscillation": ["debug 0", "calibrate", "enable", "m 0", "status", "m 90", "status", "m 180", "status", "m 270", "status"],

    # Position accuracy only (faster than full baseline)
    "sweep": ["debug 0", "calibrate", "enable", "position_sweep"],

    # Quick sanity: does it move and arrive?
    "quick": ["debug 0", "calibrate", "enable", "motor_test"],

    # Backlash measurement (moves a few degrees, saves result to NVS)
    "backlash": ["debug 0", "calibrate", "enable", "recalibrate", "status"],
}

# ── Helpers ───────────────────────────────────────────────────────────────────

def open_port(port, baud):
    ser = serial.Serial(
        port=port, baudrate=baud,
        timeout=0.1,
        dsrdtr=False,   # Don't assert DSR — avoids resetting hwcdc boards
        rtscts=False,
    )
    return ser


# Per-command idle timeouts (seconds of silence = command done)
# Stepper firmware: no FOC alignment, so calibrate is near-instant; T5 moves
# settle+trim in ~1s each but print after every position.
IDLE_TIMEOUTS = {
    "test":           20.0,
    "diag":           20.0,
    "recalibrate":    15.0,  # backlash measurement legs are silent while moving
    "position_sweep":  8.0,
    "m":               8.0,  # silence while moving; AT_TARGET prints on settle
    "default":         2.5,
}

def read_until_done(ser, cmd, verbose=True):
    """Read serial output until command completes or timeout."""
    key = cmd.strip().split()[0].lower()
    timeout = TIMEOUTS.get(key, TIMEOUTS["default"])
    idle_timeout = IDLE_TIMEOUTS.get(key, IDLE_TIMEOUTS["default"])

    markers = DONE_MARKERS.get(key, [])
    lines = []
    t_start = time.time()
    t_last_data = time.time()

    while True:
        raw = ser.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").rstrip()
            lines.append(line)
            t_last_data = time.time()
            if verbose:
                print(f"  {line}")
            if markers and any(marker in line for marker in markers):
                time.sleep(0.05)
                ser.reset_input_buffer()  # discard any queued streaming data
                break
        else:
            elapsed = time.time() - t_start
            idle = time.time() - t_last_data
            if idle >= idle_timeout and lines:
                break
            if elapsed >= timeout:
                lines.append(f"[TIMEOUT after {timeout}s]")
                if verbose:
                    print(f"  [TIMEOUT after {timeout}s]")
                break

    return lines


def send_command(ser, cmd, verbose=True):
    if verbose:
        print(f"\n>>> {cmd}")
    ser.write((cmd + "\n").encode("utf-8"))
    ser.flush()
    time.sleep(0.05)
    return read_until_done(ser, cmd, verbose)


def capture_stream(ser, seconds, verbose=True):
    """Capture $ENC, streaming data for N seconds."""
    print(f"\n>>> [streaming for {seconds}s]")
    lines = []
    t_end = time.time() + seconds
    while time.time() < t_end:
        raw = ser.readline()
        if raw:
            line = raw.decode("utf-8", errors="replace").rstrip()
            lines.append(line)
            if verbose:
                print(f"  {line}")
    return lines


def save_results(all_output, commands, port):
    out_path = os.path.join(OUT_DIR, "last_run.txt")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(f"=== Motor Test Run ===\n")
        f.write(f"Time:     {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Port:     {port}\n")
        f.write(f"Commands: {' | '.join(commands)}\n")
        f.write("=" * 40 + "\n\n")
        for line in all_output:
            f.write(line + "\n")
    print(f"\n[Saved to {out_path}]")
    return out_path


def interactive_mode(ser):
    """Pass-through interactive terminal."""
    print("Interactive mode — type commands, Ctrl+C to exit\n")
    import threading

    def reader():
        while True:
            try:
                raw = ser.readline()
                if raw:
                    print(raw.decode("utf-8", errors="replace").rstrip())
            except Exception:
                break

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    try:
        while True:
            cmd = input()
            ser.write((cmd + "\n").encode("utf-8"))
    except KeyboardInterrupt:
        print("\nExiting interactive mode.")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Motor controller serial automation")
    parser.add_argument("commands", nargs="*", help="Commands to send (in order)")
    parser.add_argument("--port", default=PORT, help=f"Serial port (default: {PORT})")
    parser.add_argument("--preset", choices=PRESETS.keys(), help="Run a preset command sequence")
    parser.add_argument("--stream", type=float, metavar="SECONDS", help="Capture streaming data for N seconds")
    parser.add_argument("--interactive", action="store_true", help="Interactive passthrough mode")
    parser.add_argument("--quiet", action="store_true", help="Don't print output to console")
    args = parser.parse_args()

    commands = args.commands
    if args.preset:
        commands = PRESETS[args.preset]

    if not commands and not args.interactive and not args.stream:
        parser.print_help()
        sys.exit(1)

    print(f"Connecting to {args.port} at {BAUD} baud...")
    try:
        ser = open_port(args.port, BAUD)
    except serial.SerialException as e:
        print(f"ERROR: Could not open {args.port}: {e}")
        print("Is the ESP32 connected? Is the serial monitor in Arduino IDE closed?")
        sys.exit(1)

    # Brief settle — hwcdc doesn't reset on port open, but give it a moment
    time.sleep(0.5)
    ser.reset_input_buffer()
    print(f"Connected.\n")

    all_output = []
    verbose = not args.quiet

    try:
        if args.interactive:
            interactive_mode(ser)
            return

        if args.stream:
            # Enable stream then capture
            all_output += send_command(ser, "stream on", verbose)
            all_output += capture_stream(ser, args.stream, verbose)
            all_output += send_command(ser, "stream off", verbose)
        else:
            for cmd in commands:
                lines = send_command(ser, cmd, verbose)
                all_output += lines
                # Small gap between commands
                time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n[Interrupted]")
    finally:
        ser.close()

    if all_output:
        save_results(all_output, commands or [f"stream {args.stream}s"], args.port)


if __name__ == "__main__":
    main()
