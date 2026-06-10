# Scan Module — Current State & Plan

**Last updated:** 2026-03-27
**Status:** 🟡 FIRMWARE COMPLETE — scan commands implemented and tested; Python scan code not yet built

---

## What This Module Does

Orchestrates a hyperspectral scan: the motor sweeps continuously while the camera records video and the encoder position is logged. Post-processing extracts frames corresponding to desired angular positions.

**This module sits between:**
- ⬅ Motor module (ESP32 firmware, `$ENC,` serial stream) — inputs
- ➡ Datacube module (frame extraction, cube assembly) — outputs

---

## Architecture Decision: Video + Encoder Log

**Chosen approach:** Continuous sweep + simultaneous video recording + encoder logging.
Post-processing correlates encoder positions to video frame timestamps.

**Why not step-and-hold:**
The old `motor_control/SCAN_ARCHITECTURE.md` and `IMPLEMENTATION_ROADMAP.md` describe
a step-and-hold approach (move to angle, verify, capture). This is superseded. At ±0.64°
positioning accuracy, step-and-hold gives 8-pixel positioning error at 0.08°/pixel.
The video approach makes positioning accuracy largely irrelevant — encoder readout
accuracy and monotonic motion are what matter, both of which the current system handles well.

**Why this works:**
- Encoder repeatability (same-direction reads) is much better than absolute accuracy (±0.1° vs ±0.64°)
- At 1°/s sweep, 30fps video gives ~2.4 frames per 0.08° pixel-width — comfortable oversampling
- Velocity-only forward frames eliminate oscillation artifacts
- Post-processing can be re-run with different target angles without re-scanning

---

## Precision Budget (POC)

```
Camera:            Mightex 752×480 (MT9V032, global shutter)
Sweep range:       60° (configurable)
Pixel resolution:  60° ÷ 752 = 0.0798°/pixel
Target accuracy:   ±0.04° (half-pixel) — achievable with encoder readout
Sweep speed:       1°/s (gives 2.4× oversampling at 30fps)
Sweep duration:    ~60 seconds
```

**Encoder readout accuracy at 1°/s:**
- Serial log at 100Hz → 10ms between samples → 0.01° between samples at 1°/s
- Clock correlation uncertainty: ~5ms → ~0.005° positional uncertainty
- Both well below 0.04° half-pixel target ✓

---

## Module Boundary: Inputs & Outputs

### Inputs (from Motor module)
```
Serial stream (COM3, 115200):
  $ENC,<esp_ms>,<pos_deg>,<vel_deg_s>,<target_deg>,<vq>,<elec_deg>   @ 100Hz
  $SYNC,<esp_ms>                                                        on demand
  $SWEEP_START,<esp_ms>                                                 on sweep begin
  $SWEEP_END,<esp_ms>                                                   on sweep end
```

### Outputs (to Datacube module)
```
scans/<session_id>/
  video.avi                 raw video (all frames, Mightex native)
  encoder_log.csv           esp_ms, pos_deg, vel_deg_s, vq, os_time_ms
  metadata.json             session params, clock offset, sweep config
  frames/                   (optional: pre-extracted frames)
    frame_0000.tiff         extracted at 0.00°
    frame_0001.tiff         extracted at 0.08°
    ...
```

### `metadata.json` schema
```json
{
  "session_id": "scan_20260327_143022",
  "timestamp_iso": "2026-03-27T14:30:22",
  "clock_offset_ms": 12450.3,
  "sweep_start_deg": 0.0,
  "sweep_end_deg": 60.0,
  "sweep_speed_deg_s": 1.0,
  "camera": "Mightex MT9V032 752x480",
  "fps_nominal": 30,
  "encoder_log_hz": 100,
  "pixel_width_deg": 0.0798,
  "notes": ""
}
```

---

## Clock Synchronization

**Method: Software handshake at scan start**

```
Desktop                          ESP32
  │                                │
  │── "sync\n" ──────────────────>│
  t_send = time.time()             │ logs $SYNC,<esp_ms>
  │<── "$SYNC,<esp_ms>\n" ─────── │
  t_recv = time.time()             │
  │                                │
  os_midpoint_ms = (t_send + t_recv)/2 * 1000
  clock_offset_ms = os_midpoint_ms - esp_ms
  uncertainty ≈ (t_recv - t_send)/2 * 1000   # typically 1–5ms
```

**Drift over 60s sweep:** USB clock drift on modern PC is <20ms over 60s.
At 1°/s, 20ms = 0.02° = 0.25 pixel — acceptable for POC.

**Encoder log OS timestamp:** Append `time.time()` at the moment each `$ENC,` line
is received by the desktop reader. This gives two timestamps per sample (ESP32 + OS)
allowing drift to be measured and corrected if needed.

---

## Firmware Commands (Implemented & Tested)

All three commands are implemented in firmware and validated 2026-03-27.

### 1. `sync` → `$SYNC,<esp_ms>`
Clock sync handshake. RTT 0–2ms, uncertainty ±1ms. Well below 5ms target.

### 2. `sweep <deg_per_sec>` → `$SWEEP_START,<esp_ms>`
Moving-target position sweep. Advances position setpoint at `deg_per_sec` per second;
the existing position PID tracks it. The PID applies up to 6V transiently to push through
cogging detents — no stalling, no special tuning.

**Why not SimpleFOC velocity mode:** At 1°/s (= 0.017 rad/s), velocity PID with Kp=0.3
produces 0.005V output — far below the ~0.8V cogging threshold. Also `motor.shaft_velocity`
at 1kHz gives 0.001°/sample at 1°/s, below MT6701 encoder resolution (0.022°/count).
Moving-target position control sidesteps all of this.

**Tested speed accuracy:**
| Start position | Commanded | Actual | Error |
|---|---|---|---|
| 270.9° (strong cogging detent) | 5.0 deg/s | 5.14 deg/s | 2.8% |
| 347.5° (near 0° boundary) | 5.0 deg/s | 4.96 deg/s | 0.9% |
| 61.4° | 5.0 deg/s | 5.15 deg/s | 3.0% |

Motor successfully crossed 0° and 270° (previously stall dead zones with fixed voltage).
Speed accuracy at 1°/s (actual scan speed) not yet measured but expected similar.

### 3. `sweep_stop` → `$SWEEP_END,<esp_ms>`
Freezes position setpoint at current encoder position. Motor holds in place.

---

## Python Scan Session Code

### File structure (new, under `scan/`)
```
scan/
  __init__.py
  session.py        ScanSession: orchestrates recording + sync
  encoder_reader.py EncoderReader: threaded $ENC, log writer
  frame_extractor.py FrameExtractor: post-processing, angle→frame
  run_scan.py       CLI entry point: python -m scan.run_scan
```

### `encoder_reader.py` (core reliability piece)
```python
class EncoderReader:
    """Reads $ENC, lines from serial, appends OS timestamp, writes to CSV."""
    def __init__(self, port, baud=115200, output_path=None):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.out = open(output_path, 'w')
        self.out.write('esp_ms,pos_deg,vel_deg_s,target_deg,vq,elec_deg,os_time_ms\n')
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while self._running:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                os_time_ms = time.time() * 1000
                if line.startswith('$ENC,'):
                    self.out.write(line[5:] + f',{os_time_ms:.1f}\n')
                    self.out.flush()  # flush every line — if process dies, log survives
                elif line.startswith('$SYNC,') or line.startswith('$SWEEP'):
                    # also log sync/sweep markers
                    self.out.write(f'# {line},{os_time_ms:.1f}\n')
                    self.out.flush()
            except Exception:
                pass  # don't let reader die on a bad line

    def stop(self):
        self._running = False
        self._thread.join(timeout=2)
        self.out.close()
```

Key design choices:
- `flush()` every line → if crash/ctrl-C, partial log is still valid
- Dropped serial lines just leave a gap in the CSV (timestamps tell you the gap size)
- Bad/garbled lines are silently skipped — next good line resumes
- Non-`$ENC,` lines logged as comments for debugging

### `frame_extractor.py` (post-processing)
```python
def extract_frames(video_path, encoder_log_path, metadata_path, target_angles):
    meta = json.load(open(metadata_path))
    clock_offset_ms = meta['clock_offset_ms']
    fps = meta['fps_nominal']

    enc = pd.read_csv(encoder_log_path)
    enc['os_time_ms'] = enc['os_time_ms']  # already in log

    cap = cv2.VideoCapture(str(video_path))
    video_start_os_ms = meta['video_start_os_ms']  # logged at cv2.VideoWriter start

    results = {}
    for target_deg in target_angles:
        best_frame_idx = None
        best_error = float('inf')

        for frame_idx in range(int(cap.get(cv2.CAP_PROP_FRAME_COUNT))):
            frame_os_ms = video_start_os_ms + (frame_idx / fps) * 1000
            # interpolate encoder position at this frame time
            pos = np.interp(frame_os_ms, enc['os_time_ms'], enc['pos_deg'])
            vel = np.interp(frame_os_ms, enc['os_time_ms'], enc['vel_deg_s'])
            error = abs(pos - target_deg)
            if vel > 0.2 and error < best_error:  # forward motion only
                best_error = error
                best_frame_idx = frame_idx

        if best_frame_idx is not None:
            cap.set(cv2.CAP_PROP_POS_FRAMES, best_frame_idx)
            _, frame = cap.read()
            results[target_deg] = {'frame': frame, 'error_deg': best_error}

    return results
```

---

## Build Order

1. ~~**Firmware: `sync` command**~~ ✅ Done
2. ~~**Firmware: `sweep` / `sweep_stop` commands**~~ ✅ Done (moving-target position control)
3. **`encoder_reader.py`** (1 hr) — threaded logger, test standalone before scan
4. **`session.py`** (2 hr) — glue: sync handshake + start reader + start camera + start sweep
5. **`frame_extractor.py`** (1–2 hr) — post-processing, test on recorded data
6. **`run_scan.py`** (30 min) — CLI wrapper to run a full scan from command line

**First milestone:** `python -m scan.run_scan --speed 1 --range 60 --output scans/`
Records video + encoder log to a session directory, prints clock offset.

**Second milestone:** `python -m scan.frame_extractor scans/<session>/ --pixel-width 0.08`
Extracts one frame per pixel-width, writes to `frames/` subdir.

---

## Open Questions

- **Video start timestamp:** How to get the OS time precisely when the first video frame
  is captured by OpenCV? Options: (a) `time.time()` just before `VideoCapture.read()` first
  returns True, (b) use Mightex SDK directly for timestamped frames.
- **Camera control:** Does the existing `camera_control/` code work for the Mightex?
  Need to check `camera_control/mightex_driver/` before building new abstractions.
- **Sweep speed at 1°/s:** Tested at 5°/s (3% error). Accuracy at 1°/s (actual scan speed)
  not yet verified. Moving-target position control has no theoretical lower bound on speed,
  but verify before committing to 1°/s in scan protocol.
- **Cogging micro-bumps at 1°/s:** At slow speed, the PID may produce brief slowdowns at
  cogging detents (not stalls — the overall position catches up). The encoder log captures
  actual position, so frame extraction handles this automatically.
