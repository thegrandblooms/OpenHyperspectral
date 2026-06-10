#!/usr/bin/env python3
"""
Run cogmap_dense: measure cogging force at 180 positions (2° steps, full revolution).
Stores Vq into firmware LUT, which auto-enables after the run.

Takes ~4–6 minutes (180 positions × ~1.5s per position: fast settle + 500ms dwell).

Output:
  $COG,<target_deg>,<actual_deg>,<error_deg>,<voltage_q>,<elec_angle_deg>  (×180)
  [COG_LUT] summary line from firmware
  Python analysis: Vq stats, worst positions, LUT plot (ASCII bar chart)
"""
import serial, time, sys, io, csv, os
from datetime import datetime

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)

PORT = 'COM3'
BAUD = 115200
EXPECTED = 360   # 1° steps × 360 = 360°

def main():
    print(f'=== Dense Cogging Map (180 positions, 2° steps) ===')
    print(f'Port: {PORT}  Expected duration: ~5 min')
    print()

    s = serial.Serial(PORT, BAUD, timeout=2)
    time.sleep(1.5)
    s.reset_input_buffer()

    def send(cmd):
        s.write((cmd + '\n').encode())
        print(f'>>> {cmd}')

    def readline(timeout=2.0):
        s.timeout = timeout
        return s.readline().decode('utf-8', errors='replace').strip()

    def readline_tagged(prefix, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            l = readline(0.3)
            if l.startswith(prefix):
                return l
        return None

    # Clear any stuck state
    send('sweep_stop')
    send('stream_off')
    time.sleep(0.3)
    s.reset_input_buffer()

    # Calibrate
    send('calibrate')
    deadline = time.time() + 25
    while time.time() < deadline:
        l = readline(0.5)
        if not l:
            continue
        print(f'  {l}')
        if 'Cal:Y' in l or ('OK' in l and 'Err' in l):
            break

    time.sleep(0.3)
    s.reset_input_buffer()

    # Dense cogmap
    send('cogmap_dense')
    start_line = readline_tagged('$COGMAP_START', timeout=5.0)
    if not start_line:
        print('[!] No $COGMAP_START — is motor calibrated?')
        s.close()
        sys.exit(1)
    print(f'  {start_line}')
    print(f'  (measuring {EXPECTED} positions — ~5 min, please wait...)')
    print()

    cog_results = []   # (target, actual, err, vq, elec_d)
    s.timeout = 0.5
    deadline = time.time() + 900   # 15 min hard timeout
    t_start = time.time()

    while time.time() < deadline:
        l = s.readline().decode('utf-8', errors='replace').strip()
        if l.startswith('$COG,'):
            parts = l.split(',')
            if len(parts) >= 5:
                try:
                    target = float(parts[1])
                    actual = float(parts[2])
                    err    = float(parts[3])
                    vq     = float(parts[4])
                    elec_d = float(parts[5]) if len(parts) > 5 else 0.0
                    cog_results.append((target, actual, err, vq, elec_d))
                    elapsed = time.time() - t_start
                    eta = (elapsed / len(cog_results)) * (EXPECTED - len(cog_results)) if len(cog_results) > 0 else 0
                    print(f'  [{len(cog_results):3d}/{EXPECTED}]  {l}  (ETA {eta:.0f}s)')
                except ValueError:
                    pass
        elif l == '$COGMAP_END':
            print(f'  {l}')
            break
        elif l and not l.startswith('$ENC'):
            print(f'  {l}')

    s.close()

    if not cog_results:
        print('[!] No $COG results captured')
        sys.exit(1)

    # ── Analysis ──────────────────────────────────────────────────────────────
    n = len(cog_results)
    print(f'\n=== Dense Cogging Map Analysis ({n} positions) ===\n')

    vqs = [r[3] for r in cog_results]
    max_vq    = max(vqs)
    min_vq    = min(vqs)
    mean_vq   = sum(vqs) / n
    ptp       = max_vq - min_vq
    ff_amp    = ptp / 2.0
    rms_vq    = (sum(v*v for v in vqs) / n) ** 0.5

    print(f'Vq stats:')
    print(f'  Max:            {max_vq:+.4f} V')
    print(f'  Min:            {min_vq:+.4f} V')
    print(f'  Peak-to-peak:   {ptp:.4f} V')
    print(f'  Mean:           {mean_vq:+.4f} V  (should be ~0; nonzero = systematic motor bias)')
    print(f'  RMS:            {rms_vq:.4f} V')
    print(f'  Sinusoidal ff_amp (half p-p): {ff_amp:.3f} V  (for reference; LUT is active)')

    # ASCII bar chart of Vq vs angle
    print(f'\nCogging profile (Vq vs angle):')
    print(f'  {"Angle":>5}  {"Vq":>7}  {"Bar"}')
    bar_scale = 20.0 / max(abs(min_vq), abs(max_vq)) if max(abs(min_vq), abs(max_vq)) > 0 else 1.0
    for target, actual, err, vq, elec_d in cog_results:
        bar_len = int(abs(vq) * bar_scale)
        direction = '+' if vq >= 0 else '-'
        bar = direction + '#' * bar_len
        print(f'  {target:5.0f}°  {vq:+7.4f}V  {bar}')

    # Worst 10 positions
    sorted_by_vq = sorted(cog_results, key=lambda r: abs(r[3]), reverse=True)
    print(f'\nWorst 10 positions (hardest cogging detents):')
    for target, actual, err, vq, elec_d in sorted_by_vq[:10]:
        print(f'  {target:5.1f}°  actual={actual:.1f}°  Vq={vq:+.4f}V  elec={elec_d:.1f}°')

    # Save CSV for offline analysis
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = os.path.join(os.path.dirname(__file__), f'cogmap_dense_{ts}.csv')
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['target_deg', 'actual_deg', 'error_deg', 'vq_V', 'elec_deg'])
        w.writerows(cog_results)
    print(f'\nSaved: {out_path}')
    print(f'\nLUT is now ACTIVE in firmware — run sweep_validation to verify improvement.')
    print(f'To disable: send "cogreset" command.  LUT is lost on power-cycle (re-run cogmap_dense).')


if __name__ == '__main__':
    main()
