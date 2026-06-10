# Scan Module — Decision History

---

## 2026-03-27: Architecture Decision — Video + Encoder Correlation

**Decision:** Use continuous sweep + simultaneous video recording + encoder logging.
Post-process to extract frames nearest desired angles.

**Alternatives considered:**

### Step-and-hold (rejected)
Move to angle → verify position → trigger camera capture → repeat.
Documented in `motor_control/SCAN_ARCHITECTURE.md` (now superseded).

Why rejected:
- Motor positioning accuracy is ±0.64° (MT6701 INL + cogging detents at 4.3° period)
- At 0.08°/pixel that's 8-pixel positioning error per capture
- Step-and-hold accuracy is bounded by absolute positioning, which has a software floor
- Software paths tried and exhausted: CalibratedSensor (failed — introduces 0.49° offset), anti-cogging feedforward (failed — resonance at standstill), I_vel tuning (0.1 is optimal, can't increase without oscillation)

Why video+encoder works:
- Encoder *repeatability* (same-direction reads) is ±0.1° — far better than absolute accuracy
- Monotonic sweep eliminates hysteresis in encoder readings
- Post-processing picks the frame nearest each target angle — no real-time synchronization needed
- Clock sync uncertainty (~5ms = 0.005°) is well below 0.04° half-pixel target
- Re-runs of frame extraction require no re-scan

---

## 2026-03-27: Camera Research

**Mightex MT9V032 (selected for POC)**
- 752×480, global shutter, 38fps
- USB2 interface via Mightex SDK
- Hardware trigger input + strobe output on 4-pin GPIO header
- 3.3V trigger compatible with ESP32 direct
- Global shutter eliminates rolling-shutter skew (critical for line-scan)

**OV2311 (future — higher res)**
- 2MP (1600×1300), global shutter, 50fps
- USB2
- Hardware trigger via XVS pin at 1.8V → needs level shifter from ESP32 3.3V
- Higher resolution would improve angular precision: 60° ÷ 1600 = 0.0375°/pixel

**POC precision math:**
- 752px across 60° → 0.0798°/pixel
- Target accuracy: ±0.04° (half-pixel)
- At 1°/s sweep + 30fps → 2.4× oversampling (one sample every 0.033°)
- Encoder readout uncertainty: ±0.005° (5ms clock error at 1°/s) — well under target

**Hardware trigger decision:** Not needed for POC. Software handshake + continuous video
is sufficient at 1°/s. Hardware trigger becomes relevant if sweep speed increases or
if sub-frame timing is required.

---

## 2026-03-27: Clock Synchronization Method

**Method selected:** Software handshake at scan start.

Send `sync\n` to ESP32, receive `$SYNC,<esp_ms>`, bracket with `time.time()` calls.
Midpoint gives clock offset estimate; RTT/2 gives uncertainty estimate.

Typical RTT over USB serial: 1–5ms → ±0.5–2.5ms uncertainty.
At 1°/s: 2.5ms → 0.0025° → 0.03 pixel — acceptable.

**Why not hardware sync pulse:** Hardware pulse from ESP32 → camera trigger requires
additional wiring and camera API calls at scan start. Not needed at 1°/s.
Can be added later for faster sweeps.

**Encoder log dual timestamp:** Each `$ENC,` line gets both ESP32 timestamp (from line)
and OS timestamp (appended by desktop reader). Two timestamps per sample enables
drift measurement and correction if needed.

---

## 2026-03-27: Decoupled Architecture Decision

**Decision:** Camera and motor systems run independently; desktop orchestrates both
but does not require tight real-time coupling.

Context: ESP32 on USB, Mightex on separate USB. Desktop Python coordinates both.

Why decoupled works:
- OpenCV VideoCapture writes frames to disk continuously
- EncoderReader thread writes `$ENC,` lines to CSV continuously
- Correlation happens in post-processing, not in real time
- If either system drops a sample, timestamps expose the gap — no silent corruption
- Desktop process crash → partial log + partial video both recoverable

Alternative (real-time frame capture on encoder trigger) was rejected:
- Required tight USB-serial coordination
- Any timing jitter in desktop Python → missed triggers
- More failure modes, harder to debug
