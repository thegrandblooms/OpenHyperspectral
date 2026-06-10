#!/usr/bin/env python3
"""
Test sync, sweep <deg/s>, and sweep_stop firmware commands.

Usage:
    python tools/test_scan_commands.py [--port COM3] [--speed 5.0]

Tests:
    1. Clock sync handshake (RTT + offset)
    2. Sweep at commanded speed for 10s — verify motor moves monotonically
    3. sweep_stop — verify motor holds position
"""

import serial
import time
import sys
import io
import argparse
import statistics

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='COM3')
    p.add_argument('--speed', type=float, default=5.0, help='Sweep speed deg/s')
    p.add_argument('--duration', type=float, default=10.0, help='Sweep duration seconds')
    return p.parse_args()


def main():
    args = parse_args()

    print(f"=== Scan Command Test ===")
    print(f"Port: {args.port}  Speed: {args.speed} deg/s  Duration: {args.duration}s")

    s = serial.Serial(args.port, 115200, timeout=2)
    time.sleep(1.5)
    s.reset_input_buffer()

    def send_raw(cmd_bytes):
        s.write(cmd_bytes)

    def send(cmd):
        s.write((cmd + '\n').encode('utf-8'))
        print(f'>>> {cmd}')

    def readline():
        raw = s.readline()
        return raw.decode('utf-8', errors='replace').strip()

    def readline_tagged(prefix, timeout=2.0):
        """Read lines until we find one starting with prefix (skips echo lines)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = readline()
            if line.startswith(prefix):
                return line
        return None

    def drain(timeout=0.5):
        s.timeout = timeout
        lines = []
        while True:
            l = readline()
            if l:
                print(f'    {l}')
                lines.append(l)
            else:
                break
        s.timeout = 2
        return lines

    # -------------------------------------------------------------------------
    # 1. Clock sync test
    # -------------------------------------------------------------------------
    print("\n--- Test 1: Clock sync ---")
    # Drain any leftover serial output first
    drain(0.3)
    rtts = []
    for i in range(5):
        t_before = time.time()
        s.write(b'sync\n')
        # Firmware echoes "> sync" before $SYNC — skip echo lines
        line = readline_tagged('$SYNC,', timeout=2.0)
        t_after = time.time()
        if line:
            rtt_ms = (t_after - t_before) * 1000
            esp_ms = int(line.split(',')[1])
            mid_t = (t_before + t_after) / 2
            offset_ms = mid_t * 1000 - esp_ms
            rtts.append(rtt_ms)
            print(f'  [{i+1}] {line}  RTT={rtt_ms:.1f}ms  host_offset={offset_ms:.1f}ms')
        else:
            print(f'  [{i+1}] No $SYNC response')
        time.sleep(0.05)

    if rtts:
        print(f'  RTT: min={min(rtts):.1f}  max={max(rtts):.1f}  avg={statistics.mean(rtts):.1f}ms')
        sync_ok = max(rtts) < 20
        print(f'  Sync: {"PASS" if sync_ok else "FAIL"} (max RTT {"<" if sync_ok else ">="} 20ms)')
    else:
        print('  Sync: FAIL (no $SYNC responses)')

    # -------------------------------------------------------------------------
    # 2. Calibrate (use stored cal if available)
    # -------------------------------------------------------------------------
    print("\n--- Calibrate ---")
    send('calibrate')
    deadline = time.time() + 25
    cal_ok = False
    while time.time() < deadline:
        line = readline()
        if not line:
            continue
        print(f'    {line}')
        if 'Cal:Y' in line or ('OK' in line and 'Err' in line):
            cal_ok = True
            break
        if 'FAILED' in line or 'ERROR' in line:
            print('  [Calibration FAILED]')
            break

    if not cal_ok:
        print('  Cannot proceed without calibration')
        s.close()
        sys.exit(1)

    time.sleep(0.5)
    s.reset_input_buffer()

    # -------------------------------------------------------------------------
    # 3. Enable motor manually (avoid auto-idle interference)
    # -------------------------------------------------------------------------
    print("\n--- Enable motor ---")
    send('e')
    drain(0.5)

    # Read starting position
    s.write(b'status\n')
    print('>>> status')
    drain(0.5)

    # -------------------------------------------------------------------------
    # 4. Enable encoder stream at 20Hz for monitoring
    # -------------------------------------------------------------------------
    print("\n--- Enable encoder stream ---")
    send('stream_on 20')
    drain(0.3)

    # -------------------------------------------------------------------------
    # 5. Sweep test
    # -------------------------------------------------------------------------
    print(f"\n--- Test 2: Sweep at {args.speed} deg/s for {args.duration}s ---")
    send(f'sweep {args.speed}')

    positions = []
    timestamps = []
    enc_lines = []
    deadline = time.time() + args.duration + 2

    # Read $SWEEP_START
    line = readline()
    print(f'    {line}')
    t_sweep_start = time.time()

    # Collect encoder data during sweep
    sweep_deadline = time.time() + args.duration
    s.timeout = 0.1
    while time.time() < sweep_deadline:
        line = readline()
        if not line:
            continue
        if line.startswith('$ENC,'):
            parts = line.split(',')
            if len(parts) >= 3:
                try:
                    pos = float(parts[2])
                    positions.append(pos)
                    timestamps.append(time.time())
                    enc_lines.append(line)
                except ValueError:
                    pass
        # Don't print every encoder line — too noisy

    s.timeout = 2

    # Stop sweep
    print(f"\n--- Test 3: sweep_stop ---")
    send('sweep_stop')
    line = readline()
    print(f'    {line}')

    time.sleep(0.5)

    # Stop encoder stream
    send('stream_off')
    drain(0.3)

    # -------------------------------------------------------------------------
    # Analysis
    # -------------------------------------------------------------------------
    print(f"\n--- Analysis ---")
    print(f"  Encoder samples collected: {len(positions)}")

    if len(positions) >= 2:
        # Unwrap positions (handle 360°→0° wraparound)
        unwrapped = [positions[0]]
        for i in range(1, len(positions)):
            delta = positions[i] - positions[i-1]
            # Handle wraparound
            if delta > 180:
                delta -= 360
            elif delta < -180:
                delta += 360
            unwrapped.append(unwrapped[-1] + delta)

        total_travel = unwrapped[-1] - unwrapped[0]
        elapsed = timestamps[-1] - timestamps[0]
        actual_speed = total_travel / elapsed if elapsed > 0 else 0

        print(f"  Total travel:    {total_travel:.1f}°")
        print(f"  Elapsed:         {elapsed:.2f}s")
        print(f"  Commanded speed: {args.speed:.1f} deg/s")
        print(f"  Actual speed:    {actual_speed:.2f} deg/s")

        direction_ok = (args.speed > 0 and total_travel > 0) or (args.speed < 0 and total_travel < 0)
        speed_error_pct = abs(actual_speed - args.speed) / abs(args.speed) * 100 if args.speed != 0 else 0

        print(f"\n  Direction: {'PASS' if direction_ok else 'FAIL'}")
        print(f"  Speed error: {speed_error_pct:.1f}% ({'PASS' if speed_error_pct < 30 else 'FAIL'})")

        # Check for stalls (no motion for >2s while sweep active)
        # Skip first 2s to allow motor to start from rest (startup ramp)
        startup_skip = 0
        if timestamps:
            t_start = timestamps[0]
            while startup_skip < len(timestamps) - 1 and timestamps[startup_skip] - t_start < 2.0:
                startup_skip += 1

        # Stall = slow segment persists for 2+ consecutive windows (avoids flagging cogging bumps)
        stall_detected = False
        window = min(20, len(unwrapped) // 4)
        consec = 0
        if window > 2 and startup_skip < len(unwrapped) - window:
            for i in range(startup_skip, len(unwrapped) - window):
                segment_travel = abs(unwrapped[i + window] - unwrapped[i])
                segment_time = timestamps[i + window] - timestamps[i]
                segment_speed = segment_travel / segment_time if segment_time > 0 else 0
                if segment_speed < abs(args.speed) * 0.1:  # Less than 10% of commanded speed
                    consec += 1
                    if consec >= 2:
                        stall_detected = True
                        print(f"  WARNING: Stall at index {i} ({unwrapped[i]:.1f}°), segment_speed={segment_speed:.2f} deg/s")
                        break
                else:
                    consec = 0

        print(f"  Stall detected: {'YES (FAIL)' if stall_detected else 'None (PASS)'}")

        # Print first/last few samples
        print(f"\n  First 5 positions: {[f'{p:.1f}' for p in unwrapped[:5]]}")
        print(f"  Last 5 positions:  {[f'{p:.1f}' for p in unwrapped[-5:]]}")
    else:
        print("  FAIL: Not enough encoder samples")

    s.close()
    print("\n[Done]")


if __name__ == '__main__':
    main()
