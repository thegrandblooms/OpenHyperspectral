#!/usr/bin/env python3
"""Run calibrate + sweep test on COM3, print results."""
import serial
import time
import sys
import io

# Force UTF-8 output on Windows
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

PORT = 'COM3'
BAUD = 115200

def main():
    s = serial.Serial(PORT, BAUD, timeout=2)
    time.sleep(1.5)

    # Drain any pending output
    s.reset_input_buffer()

    def send(cmd):
        s.write((cmd + '\n').encode('utf-8'))
        print(f'>>> {cmd}')

    def readline():
        raw = s.readline()
        return raw.decode('utf-8', errors='replace').strip()

    # Calibrate
    send('calibrate')
    deadline = time.time() + 20
    while time.time() < deadline:
        line = readline()
        if not line:
            continue
        print(line)
        if 'Cal:Y' in line or ('OK' in line and 'Err' in line):
            print('[Calibration complete]')
            break
        if 'FAILED' in line or 'ERROR' in line:
            print('[Calibration FAILED - retrying]')
            time.sleep(1)
            s.reset_input_buffer()
            send('calibrate')
            deadline = time.time() + 20

    time.sleep(0.5)
    s.reset_input_buffer()

    # Run sweep
    send('sweep')
    deadline = time.time() + 200
    while time.time() < deadline:
        line = readline()
        if not line:
            continue
        print(line)
        if 'Result:' in line:
            time.sleep(3)
            # drain remaining
            s.timeout = 0.5
            while True:
                l = readline()
                if l:
                    print(l)
                else:
                    break
            break

    s.close()
    print('[Done]')

if __name__ == '__main__':
    main()
