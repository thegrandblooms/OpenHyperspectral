# Scan Module — Testing Guide

**Status:** Module not yet built. This file describes how to test once built.

---

## Prerequisites

- ESP32 firmware flashed with `sync`, `sweep`, and `sweep_stop` commands
- Mightex camera connected via USB
- `pyserial`, `opencv-python`, `pandas`, `numpy` installed
- Motor calibrated and running (see `Dev_Log/motor/TESTING.md`)

---

## Step 1: Test `sync` Command (Standalone)

Before running a full scan, verify the clock handshake works.

```bash
python3 -c "
import serial, time
ser = serial.Serial('COM3', 115200, timeout=2)
t_before = time.time()
ser.write(b'sync\n')
line = ser.readline().decode().strip()
t_after = time.time()
print('Response:', line)
rtt_ms = (t_after - t_before) * 1000
esp_ms = int(line.split(',')[1])
offset_ms = (t_before + t_after) / 2 * 1000 - esp_ms
print(f'RTT: {rtt_ms:.1f}ms, Offset: {offset_ms:.1f}ms, Uncertainty: {rtt_ms/2:.1f}ms')
"
```

Expected output:
```
Response: $SYNC,12345
RTT: 3.2ms, Offset: 1234567.4ms, Uncertainty: 1.6ms
```

Pass criteria: RTT < 10ms. If >10ms, check for other processes holding COM3.

---

## Step 2: Test `sweep` Command (Standalone)

Verify velocity mode at scan speed before running with camera.

```bash
python3 -c "
import serial, time
ser = serial.Serial('COM3', 115200, timeout=2)
time.sleep(0.5)
ser.write(b'stream on\n')
time.sleep(0.1)
ser.write(b'sweep 1.0\n')  # 1 deg/s
t_start = time.time()
while time.time() - t_start < 5:
    line = ser.readline().decode().strip()
    if line.startswith('\$ENC,'):
        parts = line.split(',')
        print(f't={float(parts[1])/1000:.2f}s pos={parts[2]}° vel={parts[3]}°/s')
ser.write(b'sweep_stop\n')
"
```

Pass criteria:
- Velocity reads ~1.0°/s (±0.3°/s acceptable)
- Motion is monotonic (position increases continuously)
- No oscillation or direction reversals

---

## Step 3: Test EncoderReader (Standalone)

```bash
python3 -m scan.encoder_reader --port COM3 --output /tmp/test_enc.csv --duration 10
```

Then inspect the CSV:
```bash
python3 -c "
import pandas as pd
df = pd.read_csv('/tmp/test_enc.csv', comment='#')
print(df.head(10))
print(f'Rows: {len(df)}, Duration: {(df.os_time_ms.max()-df.os_time_ms.min())/1000:.1f}s')
gaps = df.os_time_ms.diff()
print(f'Max gap between samples: {gaps.max():.0f}ms (expected ~10ms at 100Hz)')
"
```

Pass criteria:
- ~100 rows/second
- Max gap < 50ms (occasional gaps OK, just not sustained)
- OS timestamps increasing monotonically

---

## Step 4: Full Scan (First Milestone)

```bash
python3 -m scan.run_scan --port COM3 --speed 1 --range 60 --output scans/
```

Expected output:
```
[sync] Clock offset: 1234567.4ms (RTT: 3.2ms, uncertainty: ±1.6ms)
[video] Starting recording: scans/scan_20260327_143022/video.avi
[encoder] Starting log: scans/scan_20260327_143022/encoder_log.csv
[sweep] Sending sweep 1.0 deg/s...
[sweep] $SWEEP_START received at esp_ms=12345
[recording] 10s elapsed, pos=10.1°
[recording] 20s elapsed, pos=20.2°
...
[sweep] $SWEEP_END received
[done] Session saved: scans/scan_20260327_143022/
```

After scan, verify session directory:
```
scans/scan_20260327_143022/
  video.avi          — should be ~60s, check with: cv2.VideoCapture('video.avi').get(cv2.CAP_PROP_FRAME_COUNT)
  encoder_log.csv    — should have ~6000 rows (100Hz × 60s)
  metadata.json      — check clock_offset_ms is present
```

---

## Step 5: Frame Extraction (Second Milestone)

```bash
python3 -m scan.frame_extractor scans/scan_20260327_143022/ --pixel-width 0.08 --output-dir scans/scan_20260327_143022/frames/
```

Expected output:
```
Extracting 751 frames (0.00° to 60.00° at 0.08°/pixel)...
Frame 0000: target=0.00°, best=0.03°, error=0.03°, frame_idx=5
Frame 0001: target=0.08°, best=0.09°, error=0.01°, frame_idx=7
...
Mean error: 0.018°  Max error: 0.041°
Frames written to: scans/scan_20260327_143022/frames/
```

Pass criteria:
- All frames extracted (no target angles skipped)
- Mean error < 0.04° (half-pixel)
- Max error < 0.08° (full pixel)
- Frames look visually sequential (no jumps or repeats)

---

## Diagnosing Problems

### Motor not moving monotonically
- Check `sweep` velocity: too slow (<0.5°/s) → cogging detents can stall motion
- Try increasing sweep speed to 2°/s and re-test monotonicity
- Check encoder log: look for velocity sign changes

### Large clock offset uncertainty (>10ms RTT)
- Other process may be holding COM3 (check Task Manager)
- Try `sync` command 5 times and average — single measurement can have outliers

### Frames with large angular error (>0.08°)
- Check encoder log for gaps during video recording
- Check that `video_start_os_ms` in metadata aligns with first frame
- Try reducing sweep speed (more oversampling)

### Video and encoder out of sync
- Verify `video_start_os_ms` is logged immediately when OpenCV first returns a frame
- Check encoder log os_time_ms column: timestamps should span the video duration
