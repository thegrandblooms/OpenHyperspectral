#!/usr/bin/env python3
"""
Run cogmap command, capture all 24 $COG measurements, analyze cogging pattern.

Output:
  $COG,<target_deg>,<actual_deg>,<error_deg>,<voltage_q>,<elec_angle_deg>
"""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)

PORT = 'COM3'
BAUD = 115200

def main():
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

    # Clear stuck state
    send('sweep_stop')
    send('stream_off')
    time.sleep(0.3)
    s.reset_input_buffer()

    # Calibrate
    send('calibrate')
    deadline = time.time() + 25
    while time.time() < deadline:
        l = readline(0.5)
        if not l: continue
        print(f'  {l}')
        if 'Cal:Y' in l or ('OK' in l and 'Err' in l): break

    time.sleep(0.3)
    s.reset_input_buffer()

    # Cogmap (enable is built into the command)
    send('cogmap')
    start_line = readline_tagged('$COGMAP_START', timeout=5.0)
    if not start_line:
        print('[!] No $COGMAP_START — is motor calibrated?')
        s.close()
        sys.exit(1)
    print(f'  {start_line}')
    print('  (measuring 24 positions, ~2 minutes...)')

    cog_results = []
    s.timeout = 0.5
    deadline = time.time() + 180  # 3 min max
    while time.time() < deadline:
        l = s.readline().decode('utf-8', errors='replace').strip()
        if l.startswith('$COG,'):
            parts = l.split(',')
            if len(parts) >= 5:
                target  = float(parts[1])
                actual  = float(parts[2])
                err     = float(parts[3])
                vq      = float(parts[4])
                elec_d  = float(parts[5]) if len(parts) > 5 else 0.0
                cog_results.append((target, actual, err, vq, elec_d))
                print(f'  {l}')
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
    print(f'\n=== Cogging Map Analysis ({len(cog_results)} positions) ===\n')
    print(f'{"Target":>8}  {"Actual":>8}  {"Err":>6}  {"Vq":>7}  {"Elec":>7}  {"Cogging"}')
    print(f'{"(deg)":>8}  {"(deg)":>8}  {"(deg)":>6}  {"(V)":>7}  {"(deg)":>7}')
    print('-' * 65)

    vqs = [r[3] for r in cog_results]
    max_vq = max(abs(v) for v in vqs)

    for target, actual, err, vq, elec_d in cog_results:
        bar_len = int(abs(vq) / max_vq * 20) if max_vq > 0 else 0
        direction = '+' if vq >= 0 else '-'
        bar = direction + '#' * bar_len
        print(f'{target:>8.1f}  {actual:>8.2f}  {err:>6.3f}  {vq:>7.4f}  {elec_d:>7.1f}  {bar}')

    print(f'\nVoltage.q stats:')
    print(f'  Max: {max(vqs):.4f} V')
    print(f'  Min: {min(vqs):.4f} V')
    print(f'  Peak-to-peak: {max(vqs) - min(vqs):.4f} V')
    print(f'  Mean: {sum(vqs)/len(vqs):.4f} V')

    # Estimate feedforward amplitude = half peak-to-peak
    ff_amp = (max(vqs) - min(vqs)) / 2
    print(f'\nRecommended starting cog_ff_amp: {ff_amp:.3f} V  (half peak-to-peak)')
    print(f'(Tune cog_ff_phase by sweeping 0–6.28 rad at the worst position)')

    # Worst positions (highest |vq|)
    sorted_by_vq = sorted(cog_results, key=lambda r: abs(r[3]), reverse=True)
    print(f'\nWorst 5 positions (hardest to hold):')
    for target, actual, err, vq, elec_d in sorted_by_vq[:5]:
        print(f'  {target:.0f}°  actual={actual:.1f}°  Vq={vq:.4f}V  elec={elec_d:.1f}°')

if __name__ == '__main__':
    main()
