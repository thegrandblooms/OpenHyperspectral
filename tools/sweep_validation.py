#!/usr/bin/env python3
"""
Comprehensive sweep validation for hyperspectral scan readiness.

Tests:
  1. Speed accuracy at 1 deg/s (actual scan speed) — 60 second run from 0°
  2. Speed accuracy at 5 deg/s from 0° and 270° (strong cogging detents)
  3. Monotonicity — counts backward steps, max backward excursion
  4. Cogging signature — speed vs. position, worst-case slowdowns
  5. Full 360°+ revolution at 1 deg/s

Usage:
    python tools/sweep_validation.py            # full suite
    python tools/sweep_validation.py --quick    # single 1 deg/s run from 0deg, 60s
    python tools/sweep_validation.py --speed 1 --duration 30 --from-pos 270
"""

import serial
import time
import sys
import io
import argparse
import csv
import os
from datetime import datetime

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace',
                              line_buffering=True)

PORT = 'COM3'
BAUD = 115200


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default=PORT)
    p.add_argument('--out', default=None, help='Save raw CSV to this path')
    p.add_argument('--quick', action='store_true',
                   help='Single 1 deg/s from 0deg, 60s')
    p.add_argument('--speed', type=float, default=None)
    p.add_argument('--duration', type=float, default=None)
    p.add_argument('--from-pos', type=float, default=None,
                   dest='from_pos')
    return p.parse_args()


# ── Serial helpers ─────────────────────────────────────────────────────────────

class ESP32:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=2)
        time.sleep(1.5)
        self.ser.reset_input_buffer()

    def send(self, cmd, echo=True):
        self.ser.write((cmd + '\n').encode('utf-8'))
        if echo:
            print(f'  >>> {cmd}')

    def readline(self, timeout=None):
        if timeout is not None:
            self.ser.timeout = timeout
        raw = self.ser.readline()
        return raw.decode('utf-8', errors='replace').strip()

    def readline_tagged(self, prefix, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.readline(0.3)
            if line.startswith(prefix):
                return line
        return None

    def drain(self, secs=0.4):
        self.ser.timeout = 0.05
        t_end = time.time() + secs
        while time.time() < t_end:
            raw = self.ser.readline()
            # silently discard
        self.ser.timeout = 2

    def reset_state(self):
        """Send sweep_stop + drain to put ESP32 in a known non-sweep state."""
        self.send('sweep_stop', echo=False)
        self.drain(0.3)
        self.send('stream_off', echo=False)
        self.drain(0.3)

    def calibrate(self):
        self.reset_state()  # clear any stuck sweep state first
        self.send('calibrate')
        deadline = time.time() + 25
        while time.time() < deadline:
            line = self.readline(0.5)
            if not line:
                continue
            print(f'    {line}')
            if 'Cal:Y' in line or ('OK' in line and 'Err' in line):
                return True
            if 'FAILED' in line or 'ERROR' in line:
                return False
        return False

    def enable(self):
        self.send('stream_off', echo=False)  # stop stream before enable
        self.drain(0.2)
        self.send('e')
        deadline = time.time() + 5
        while time.time() < deadline:
            line = self.readline(0.5)
            if 'Motor enabled' in line or 'enabled at' in line.lower():
                print(f'    {line}')
                break
        self.send('stream_off', echo=False)  # stop auto-stream from enable
        self.drain(0.3)

    def move_to_and_wait(self, deg, wait_s=8.0):
        """Send move command, wait fixed time for motor to settle."""
        print(f'  Moving to {deg}°, waiting {wait_s}s...')
        self.send('stream_off', echo=False)
        self.drain(0.2)
        self.send(f'm {deg:.1f}')
        # Drain stream during move — don't try to parse AT_TARGET
        self.drain(wait_s)
        self.drain(0.3)
        # Read current position from a status line
        self.send('status', echo=False)
        self.drain(0.3)

    def sweep_start(self, speed_deg_s):
        self.send(f'sweep {speed_deg_s:.2f}')
        line = self.readline_tagged('$SWEEP_START,', timeout=3.0)
        print(f'    {line}')
        return line

    def sweep_stop(self):
        self.send('sweep_stop')
        line = self.readline_tagged('$SWEEP_END,', timeout=3.0)
        print(f'    {line}')

    def collect_enc(self, duration_s, stream_hz=100, label=''):
        """Collect $ENC, samples for duration_s at stream_hz. Returns list of dicts."""
        self.send(f'stream_on {stream_hz}', echo=False)
        self.drain(0.1)

        samples = []
        deadline = time.time() + duration_s
        self.ser.timeout = 0.02
        last_progress = time.time()
        while time.time() < deadline:
            raw = self.ser.readline()
            line = raw.decode('utf-8', errors='replace').strip()
            if line.startswith('$ENC,'):
                parts = line.split(',')
                if len(parts) >= 6:
                    try:
                        samples.append({
                            'esp_ms':     int(parts[1]),
                            'pos_deg':    float(parts[2]),
                            'vel_deg_s':  float(parts[3]),
                            'target_deg': float(parts[4]),
                            # field 5: commanded position (stepper fw) / voltage_q (legacy gimbal fw)
                            'cmd_pos_deg': float(parts[5]),
                            'os_time':    time.time(),
                        })
                    except ValueError:
                        pass
            # Progress dot every 5s
            if time.time() - last_progress >= 5:
                remaining = deadline - time.time()
                print(f'    ... {remaining:.0f}s remaining ({len(samples)} samples)')
                last_progress = time.time()

        self.ser.timeout = 2
        self.send('stream_off', echo=False)
        self.drain(0.3)
        return samples

    def sync(self):
        self.drain(0.1)
        t0 = time.time()
        self.ser.write(b'sync\n')
        line = self.readline_tagged('$SYNC,', timeout=2.0)
        t1 = time.time()
        if not line:
            return None
        esp_ms = int(line.split(',')[1])
        rtt_ms = (t1 - t0) * 1000
        return rtt_ms, (t0 + t1) / 2 * 1000 - esp_ms

    def close(self):
        self.reset_state()
        self.ser.close()


# ── Analysis ───────────────────────────────────────────────────────────────────

def unwrap(positions):
    if not positions:
        return []
    result = [positions[0]]
    for i in range(1, len(positions)):
        delta = positions[i] - positions[i - 1]
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        result.append(result[-1] + delta)
    return result


def analyze_sweep(samples, commanded_speed, label=''):
    if len(samples) < 20:
        print(f'  [!] Only {len(samples)} samples — analysis skipped')
        return None

    positions = [s['pos_deg'] for s in samples]
    times = [s['os_time'] for s in samples]
    uw = unwrap(positions)

    # Skip first 3s (startup ramp, motor accelerating)
    t0 = times[0]
    skip = next((i for i, t in enumerate(times) if t - t0 >= 3.0), 5)

    aw = uw[skip:]    # analysis window (unwrapped)
    at = times[skip:] # analysis times (os_time, for startup skip only)

    # ESP32 timestamps — accurate 100Hz tick, not subject to Windows scheduling jitter
    esp_ms_all = [s['esp_ms'] for s in samples]
    esp_ms_w = esp_ms_all[skip:]

    if len(aw) < 10:
        print(f'  [!] Not enough samples after startup skip')
        return None

    total_travel = aw[-1] - aw[0]
    # Use ESP32 timestamps for elapsed time
    elapsed = (esp_ms_w[-1] - esp_ms_w[0]) / 1000.0
    actual_speed = total_travel / elapsed if elapsed > 0 else 0
    speed_err_pct = abs(actual_speed - commanded_speed) / abs(commanded_speed) * 100

    # Per-sample instantaneous speed — use ESP32 timestamps (accurate 100Hz tick),
    # NOT os_time (Windows scheduling jitter causes <1ms bursts → inflated speeds).
    inst_speeds = []
    for i in range(1, len(aw)):
        dt_ms = esp_ms_w[i] - esp_ms_w[i - 1]
        if 1 <= dt_ms <= 500:   # sanity: 1–500ms between samples
            dt_s = dt_ms / 1000.0
            inst_speeds.append((aw[i] - aw[i - 1]) / dt_s)

    # Backward steps (encoder position moving opposite to commanded direction)
    sign = 1 if commanded_speed > 0 else -1
    # Use physical backward motion threshold: >0.1° backward in one sample period
    backward_steps = 0
    max_back = 0.0
    for i in range(1, len(aw)):
        dt_ms = esp_ms_w[i] - esp_ms_w[i - 1]
        if dt_ms > 0:
            delta = aw[i] - aw[i - 1]
            if delta * sign < -0.1:   # >0.1° backward
                backward_steps += 1
                max_back = max(max_back, abs(delta))

    backward_pct = backward_steps / max(len(aw) - 1, 1) * 100

    # Worst sustained speed over 2s window — use esp_ms for timing
    worst_2s = float('inf')
    for i in range(len(aw)):
        j = i
        while j < len(aw) and (esp_ms_w[j] - esp_ms_w[i]) < 2000:
            j += 1
        if j > i + 1:
            seg_ms = esp_ms_w[j - 1] - esp_ms_w[i]
            if seg_ms >= 1500:
                seg_v = abs(aw[j - 1] - aw[i]) / (seg_ms / 1000.0)
                worst_2s = min(worst_2s, seg_v)
    if worst_2s == float('inf'):
        worst_2s = 0.0

    stall_threshold = abs(commanded_speed) * 0.20  # <20% of commanded = stall
    stall = worst_2s < stall_threshold

    return {
        'label': label,
        'commanded': commanded_speed,
        'actual': actual_speed,
        'speed_err_pct': speed_err_pct,
        'total_travel': total_travel,
        'elapsed': elapsed,
        'n_samples': len(samples),
        'n_analyzed': len(aw),
        'backward_steps': backward_steps,
        'backward_pct': backward_pct,
        'max_back_deg': max_back,
        'min_inst': min(inst_speeds) if inst_speeds else 0,
        'max_inst': max(inst_speeds) if inst_speeds else 0,
        'worst_2s': worst_2s,
        'stall': stall,
        'inst_speeds': inst_speeds,   # indexed same as analysis_positions[:-1]
        'analysis_positions': positions[skip:],
    }


def print_metrics(m):
    ok = lambda b: 'PASS' if b else 'FAIL'
    speed_ok = m['speed_err_pct'] < 15
    mono_ok = m['backward_pct'] < 2.0 and m['max_back_deg'] < 2.0
    stall_ok = not m['stall']
    overall = speed_ok and mono_ok and stall_ok

    print(f"\n  Results for: {m['label']}")
    print(f"  Samples:     {m['n_samples']} total / {m['n_analyzed']} analyzed (3s startup skip)")
    print(f"  Travel:      {m['total_travel']:.1f}° in {m['elapsed']:.1f}s")
    print(f"  Speed:       {m['commanded']:.2f} cmd → {m['actual']:.3f} actual deg/s  "
          f"({m['speed_err_pct']:.1f}% error)  [{ok(speed_ok)}]")
    print(f"  Speed range: {m['min_inst']:.2f} – {m['max_inst']:.2f} deg/s (instantaneous)")
    print(f"  Worst 2s:    {m['worst_2s']:.3f} deg/s  [{ok(stall_ok)}]"
          f"{'  ← STALL' if m['stall'] else ''}")
    print(f"  Monotone:    {m['backward_steps']} backward steps ({m['backward_pct']:.2f}%), "
          f"max back = {m['max_back_deg']:.3f}°  [{ok(mono_ok)}]")
    print(f"  OVERALL:     {'PASS' if overall else 'FAIL'}")
    return overall


def cogging_heatmap(m, n_bins=36):
    """Print avg speed per 10° bin to show cogging signature."""
    if not m.get('inst_speeds') or not m.get('analysis_positions'):
        return
    speeds = m['inst_speeds']
    positions = m['analysis_positions']
    # inst_speeds[i] = delta between positions[i] and positions[i+1]
    bin_speeds = [[] for _ in range(n_bins)]
    for i, spd in enumerate(speeds):
        if i < len(positions):
            pos = positions[i] % 360.0
            b = int(pos / 360.0 * n_bins) % n_bins
            bin_speeds[b].append(spd)

    cmd = m['commanded']
    print(f"\n  Cogging profile ({n_bins} bins, commanded {cmd:.1f} deg/s):")
    print(f"  {'Pos':>6}  {'Avg':>6}  {'Min':>6}  {'N':>4}  Speed ratio")
    has_slow = False
    for b, sp in enumerate(bin_speeds):
        if not sp:
            continue
        pos = b * (360 // n_bins)
        avg = sum(sp) / len(sp)
        mn = min(sp)
        ratio = avg / abs(cmd) if cmd else 1
        ratio_clamped = max(0.0, min(ratio, 2.0))
        bar = '█' * int(ratio_clamped * 12) + '░' * (24 - int(ratio_clamped * 12))
        warn = ''
        if avg < abs(cmd) * 0.5:
            warn = ' ← SLOW'
            has_slow = True
        print(f"  {pos:>5}°  {avg:>6.2f}  {mn:>6.2f}  {len(sp):>4}  {bar}{warn}")
    if not has_slow:
        print("  (no positions with avg speed < 50% of commanded)")


# ── Test runner ────────────────────────────────────────────────────────────────

def run_test(esp, speed, duration, from_pos, label):
    print(f'\n{"═"*60}')
    print(f'  {label}')
    print(f'{"═"*60}')

    if from_pos is not None:
        esp.move_to_and_wait(from_pos, wait_s=10.0)

    esp.sweep_start(speed)
    samples = esp.collect_enc(duration, stream_hz=100)
    esp.sweep_stop()

    print(f'  Collected: {len(samples)} samples')
    m = analyze_sweep(samples, speed, label)
    if m:
        passed = print_metrics(m)
        cogging_heatmap(m)
    else:
        passed = False
        m = {}

    return samples, m, passed


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    print(f'{"═"*62}')
    print(f'  Sweep Validation  {timestamp}')
    print(f'{"═"*62}')

    esp = ESP32(args.port)

    # Always start by clearing any stuck sweep state
    print('\n[Reset state]')
    esp.reset_state()
    print('  Sent sweep_stop + stream_off to clear any stuck state')

    # Sync
    print('\n[Clock sync]')
    r = esp.sync()
    if r:
        print(f'  RTT={r[0]:.1f}ms  offset={r[1]:.1f}ms')
    else:
        print('  [!] No sync response')

    # Calibrate
    print('\n[Calibrate]')
    if not esp.calibrate():
        print('  CALIBRATION FAILED — aborting')
        esp.close()
        sys.exit(1)
    time.sleep(0.5)
    esp.drain(0.3)

    # Enable
    print('\n[Enable motor]')
    esp.enable()

    all_samples = []
    all_metrics = []
    all_passed = []

    if args.speed is not None:
        # Custom single run
        dur = args.duration or 30.0
        label = f'{args.speed} deg/s'
        if args.from_pos is not None:
            label += f' from {args.from_pos}°'
        s, m, passed = run_test(esp, args.speed, dur, args.from_pos, label)
        all_samples += s
        if m:
            all_metrics.append(m)
        all_passed.append(passed)

    elif args.quick:
        s, m, passed = run_test(esp, 1.0, 60.0, 0.0, '1 deg/s from 0° — 60s')
        all_samples += s
        if m:
            all_metrics.append(m)
        all_passed.append(passed)

    else:
        # Full validation suite
        tests = [
            # (speed, duration, from_pos, label)
            (1.0, 60.0, 0.0,   '1 deg/s from 0° — 60s (actual scan params)'),
            (1.0, 30.0, 270.0, '1 deg/s from 270° — 30s (strong detent start)'),
            (5.0, 20.0, 0.0,   '5 deg/s from 0° — 20s'),
            (5.0, 20.0, 270.0, '5 deg/s from 270° — 20s'),
            (1.0, 90.0, 0.0,   '1 deg/s full revolution+ — 90s'),
        ]
        for spd, dur, fp, lbl in tests:
            s, m, passed = run_test(esp, spd, dur, fp, lbl)
            if s:
                all_samples += [dict(run=lbl, **x) for x in s]
            if m:
                all_metrics.append(m)
            all_passed.append(passed)
            time.sleep(1.0)

    # Save raw data
    out_path = args.out or f'sweep_validation_{timestamp}.csv'
    if all_samples:
        fieldnames = [k for k in all_samples[0].keys() if k != 'inst_speeds']
        with open(out_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(all_samples)
        print(f'\n[Data saved → {out_path}]')

    # Summary
    if len(all_metrics) > 1:
        print(f'\n{"═"*62}')
        print(f'  FINAL SUMMARY')
        print(f'{"═"*62}')
        for m, passed in zip(all_metrics, all_passed):
            icon = '✓' if passed else '✗'
            print(f'  {icon}  {m["label"]}')
            print(f'       speed {m["speed_err_pct"]:.1f}% err | '
                  f'backward {m["backward_pct"]:.2f}% | '
                  f'worst 2s: {m["worst_2s"]:.2f} deg/s')
        scan_ready = all(all_passed)
        print(f'\n  SCAN READY: {"YES" if scan_ready else "NO — see failures above"}')

    esp.close()
    print('\n[Done]')


if __name__ == '__main__':
    main()
