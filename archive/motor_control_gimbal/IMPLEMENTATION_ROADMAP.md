# Scan Session Management Implementation Roadmap

This document provides a step-by-step implementation plan for completing the Python scanning workflow.

## Overview

The scanning system consists of three layers:

```
┌─────────────────────────────────────────────┐
│  UI Layer (SpectrumBoi)                     │
│  - User controls                            │
│  - Progress display                         │
│  - Live preview                             │
└──────────────────┬──────────────────────────┘
                   │ Callbacks
┌──────────────────┴──────────────────────────┐
│  Orchestration Layer (scan.py)              │
│  - HyperspectralScanner                     │
│  - ScanSession                              │
│  - Workflow coordination                    │
└──────────────────┬──────────────────────────┘
                   │ Commands
┌──────────────────┴──────────────────────────┐
│  Motor Primitives (controller.py)           │
│  - move_and_verify()                        │
│  - get_encoder_position()                   │
│  - Serial communication                     │
└──────────────────┬──────────────────────────┘
                   │ Serial protocol
                   ↓
            ESP32 Firmware
```

## File Organization

```
motor_control/
├── __init__.py                              # Package exports
├── controller.py                            # ✅ EXISTS - needs additions
├── config.py                                # ❌ TO CREATE
├── scan.py                                  # ❌ TO CREATE
├── camera_adapter.py                        # ❌ TO CREATE
│
├── SCAN_ARCHITECTURE.md                     # ✅ Design documentation
├── controller_additions.pseudo.py           # ✅ Pseudocode for controller additions
├── scan.pseudo.py                           # ✅ Pseudocode for scan.py
└── spectrumboi_integration.pseudo.py        # ✅ Pseudocode for UI integration
```

## Implementation Steps

### Phase 1: Motor Primitives (controller.py additions)

**Estimated time:** 2-3 hours

**Files to modify:**
- `motor_control/controller.py`

**Additions needed:**

1. **Add CMD_GET_ENCODER command constant:**
   ```python
   CMD_GET_ENCODER = 0x0F  # New command ID
   ```

2. **Add RESP_ENCODER response constant:**
   ```python
   RESP_ENCODER = 0x88  # New response ID
   ```

3. **Implement `get_encoder_position()` method:**
   - See `controller_additions.pseudo.py` lines 16-47
   - Sends CMD_GET_ENCODER to MCU
   - Receives RESP_ENCODER with float position
   - Returns position in degrees

4. **Implement `move_and_verify()` method:**
   - See `controller_additions.pseudo.py` lines 50-138
   - Core primitive for scanning
   - Retry loop with encoder verification
   - Returns (success: bool, actual_position: float)

5. **Optional: Implement `move_and_verify_with_callback()`:**
   - See `controller_additions.pseudo.py` lines 141-177
   - Enhanced version with retry callbacks for UI

**Testing:**
```python
# Test encoder reading
motor = MotorController('/dev/ttyUSB0')
motor.connect()
motor.enable()

position = motor.get_encoder_position()
print(f"Current position: {position:.2f}°")

# Test move-and-verify
success, actual = motor.move_and_verify(90.0, tolerance=0.5)
if success:
    print(f"Position confirmed: {actual:.2f}°")
```

**Prerequisites:**
- MCU firmware must be updated to support CMD_GET_ENCODER/RESP_ENCODER
- See firmware integration notes in `controller_additions.pseudo.py` lines 182-206

---

### Phase 2: Configuration Module (config.py)

**Estimated time:** 1 hour

**Files to create:**
- `motor_control/config.py`

**Implementation:**

```python
from dataclasses import dataclass
from pathlib import Path

@dataclass
class ScanConfig:
    """Configuration for hyperspectral scans."""
    # Copy from scan.pseudo.py lines 22-69
    # Includes: scan range, position control, camera, storage, error handling

# Optional: Add validation and calculated properties
```

**Testing:**
```python
from motor_control.config import ScanConfig

config = ScanConfig(
    start_angle_deg=0,
    end_angle_deg=360,
    increment_deg=1.0
)

print(f"Total positions: {config.num_positions}")
print(f"Scan angles: {config.scan_angles[:5]}...")  # First 5 angles
```

---

### Phase 3: Camera Adapter (camera_adapter.py)

**Estimated time:** 2-3 hours

**Files to create:**
- `motor_control/camera_adapter.py`

**Implementation:**

1. **Abstract interface:**
   - See `scan.pseudo.py` lines 74-100 for `CameraInterface`

2. **Mightex adapter:**
   - See `scan.pseudo.py` lines 103-123 for `MightexCameraAdapter`
   - Wraps existing `camera_control.mightex_driver.camera.Camera`

3. **Mock adapter for testing:**
   - See `scan.pseudo.py` lines 126-142 for `MockCameraAdapter`
   - Returns random noise for testing without hardware

**Testing:**
```python
from motor_control.camera_adapter import MockCameraAdapter

camera = MockCameraAdapter(width=1920, height=1200)
camera.set_exposure(100)

image = camera.capture()
print(f"Captured image shape: {image.shape}")
```

---

### Phase 4: Scan Orchestration (scan.py)

**Estimated time:** 4-6 hours (most complex component)

**Files to create:**
- `motor_control/scan.py`

**Implementation:**

1. **ScanSession dataclass:**
   - See `scan.pseudo.py` lines 158-251
   - Stores captures, metadata, statistics
   - Methods for saving data

2. **CaptureMetadata dataclass:**
   - See `scan.pseudo.py` lines 148-155
   - Metadata for each capture

3. **HyperspectralScanner class:**
   - See `scan.pseudo.py` lines 256-474
   - Main orchestrator
   - Callback registration
   - `run_scan()` method with full workflow

**Key implementation notes:**

- **Threading:** `run_scan()` blocks but can be called from UI thread
- **Callbacks:** Use `after()` in tkinter to update UI safely
- **Error handling:** Configurable retry/skip/abort behavior
- **Data storage:** JSON metadata + TIFF/PNG/NPY images

**Testing (without UI):**
```python
from motor_control import MotorController
from motor_control.scan import HyperspectralScanner, ScanConfig
from motor_control.camera_adapter import MockCameraAdapter

# Setup
motor = MotorController('/dev/ttyUSB0')
motor.connect()
motor.enable()

camera = MockCameraAdapter()
scanner = HyperspectralScanner(motor, camera)

# Configure scan
config = ScanConfig(
    start_angle_deg=0,
    end_angle_deg=10,  # Small test scan
    increment_deg=1.0,
    save_images=True
)

# Run scan
session = scanner.run_scan(config)

print(f"Captured: {session.num_successful_captures} images")
print(f"Mean error: {session.mean_position_error:.3f}°")
```

---

### Phase 5: SpectrumBoi Integration

**Estimated time:** 3-4 hours

**Files to modify:**
- `spectrumboi.py` (or wherever the main UI is)

**Implementation:**

1. **Add scan control panel:**
   - See `spectrumboi_integration.pseudo.py` lines 22-236
   - Parameter inputs, start/stop buttons, progress display

2. **Register scanner callbacks:**
   - See `spectrumboi_integration.pseudo.py` lines 136-158
   - Update UI on progress, captures, errors

3. **Add results window:**
   - See `spectrumboi_integration.pseudo.py` lines 246-329
   - Plot position errors, browse images

**Integration points:**
- Connect to existing motor controller instance
- Connect to existing camera instance
- Add to UI layout (sidebar or tab)
- Add menu items for scan control

**Testing:**
- Start SpectrumBoi
- Configure scan parameters
- Click "Start Scan"
- Monitor progress bar and live preview
- View results when complete

---

### Phase 6: Firmware Update (ESP32)

**Estimated time:** 1-2 hours

**Files to modify:**
- `firmware/ESP32_MCU_Firmware/commands.h`
- `firmware/ESP32_MCU_Firmware/communication.cpp`

**Implementation:**

1. **Add command/response IDs:**
   ```cpp
   #define CMD_GET_ENCODER 0x0F
   #define RESP_ENCODER 0x88
   ```

2. **Add command handler:**
   ```cpp
   case CMD_GET_ENCODER:
       encoder.update();
       float position_deg = encoder.getDegrees();

       txBuff[0] = RESP_ENCODER;
       memcpy(&txBuff[1], &position_deg, sizeof(float));
       link.send(1 + sizeof(float));
       break;
   ```

**Testing:**
```bash
# Upload firmware
pio run --target upload

# Test from Python
python3 -c "
from motor_control import MotorController
motor = MotorController('/dev/ttyUSB0')
motor.connect()
print(f'Position: {motor.get_encoder_position():.2f}°')
"
```

---

## Testing Strategy

### Unit Tests

**test_controller_additions.py:**
```python
def test_get_encoder_position():
    motor = MotorController('/dev/ttyUSB0')
    motor.connect()

    pos = motor.get_encoder_position()
    assert 0 <= pos <= 360

def test_move_and_verify():
    motor.enable()
    success, actual = motor.move_and_verify(90.0)
    assert success
    assert abs(actual - 90.0) < 0.5
```

**test_scan.py:**
```python
def test_scan_config():
    config = ScanConfig(start_angle_deg=0, end_angle_deg=360, increment_deg=1.0)
    assert config.num_positions == 361
    assert len(config.scan_angles) == 361

def test_scan_session_metadata():
    session = ScanSession(config=config)
    metadata = CaptureMetadata(
        index=0,
        commanded_angle_deg=90.0,
        actual_angle_deg=90.1,
        position_error_deg=0.1,
        timestamp=datetime.now(),
        move_retries=0,
        exposure_ms=100,
        gain=1.0
    )
    session.add_capture(np.zeros((100, 100)), metadata)
    assert session.num_successful_captures == 1
```

### Integration Tests

**test_full_scan.py:**
```python
def test_small_scan_with_mock_camera():
    """Test complete scan workflow with mock hardware."""
    motor = MotorController('/dev/ttyUSB0')
    motor.connect()
    motor.enable()

    camera = MockCameraAdapter()
    scanner = HyperspectralScanner(motor, camera)

    config = ScanConfig(
        start_angle_deg=0,
        end_angle_deg=10,
        increment_deg=2.0
    )

    session = scanner.run_scan(config)

    assert session.num_successful_captures == 6  # 0, 2, 4, 6, 8, 10
    assert session.success_rate == 1.0
    assert session.mean_position_error < 0.5
```

---

## Dependencies to Add

**Python packages:**
```bash
pip install numpy matplotlib tifffile
```

**Update requirements.txt:**
```txt
pySerialTransfer>=1.0.0
numpy>=1.20.0
matplotlib>=3.5.0
tifffile>=2021.0.0
```

---

## Completion Checklist

### Phase 1: Motor Primitives ✓
- [ ] Add CMD_GET_ENCODER/RESP_ENCODER to controller.py
- [ ] Implement get_encoder_position()
- [ ] Implement move_and_verify()
- [ ] Test encoder reading
- [ ] Test move-and-verify with real motor
- [ ] Update firmware with GET_ENCODER command

### Phase 2: Configuration ✓
- [ ] Create config.py
- [ ] Implement ScanConfig dataclass
- [ ] Add validation and properties
- [ ] Test configuration creation

### Phase 3: Camera Adapter ✓
- [ ] Create camera_adapter.py
- [ ] Implement CameraInterface ABC
- [ ] Implement MightexCameraAdapter
- [ ] Implement MockCameraAdapter for testing
- [ ] Test mock camera captures

### Phase 4: Scan Orchestration ✓
- [ ] Create scan.py
- [ ] Implement CaptureMetadata dataclass
- [ ] Implement ScanSession dataclass
- [ ] Implement HyperspectralScanner class
- [ ] Test scan with mock camera
- [ ] Test metadata saving
- [ ] Test image saving

### Phase 5: UI Integration ✓
- [ ] Add ScanControlPanel to SpectrumBoi
- [ ] Connect motor and camera instances
- [ ] Implement progress callbacks
- [ ] Implement live preview updates
- [ ] Add results visualization window
- [ ] Test full UI workflow

### Phase 6: Polish ✓
- [ ] Add comprehensive error handling
- [ ] Add logging throughout
- [ ] Write unit tests
- [ ] Write integration tests
- [ ] Update documentation
- [ ] Add usage examples

---

## Success Metrics

**When implementation is complete, you should be able to:**

1. ✅ Start SpectrumBoi UI
2. ✅ Configure scan parameters (range, increment, tolerance)
3. ✅ Click "Start Scan" button
4. ✅ Watch progress bar update in real-time
5. ✅ See live preview of captured images
6. ✅ Monitor position errors during scan
7. ✅ Scan completes successfully with all images captured
8. ✅ View results window with position error plots
9. ✅ Find saved images and metadata in output directory
10. ✅ Position errors consistently < 0.5° tolerance

**Performance targets:**
- Position accuracy: < 0.5° error on 95% of captures
- Success rate: > 98% for full 360° scan
- Scan speed: ~2-3 seconds per position (depends on settling time)
- Full 360° scan @ 1° increment: ~12 minutes

---

## Next Steps

**Recommended implementation order:**

1. **Start with firmware** (Phase 6) - Add GET_ENCODER command
2. **Then controller primitives** (Phase 1) - Test immediately with real hardware
3. **Then configuration** (Phase 2) - Quick and foundational
4. **Then camera adapter** (Phase 3) - Start with mock, add Mightex later
5. **Then scan orchestration** (Phase 4) - Test with mock camera first
6. **Finally UI integration** (Phase 5) - Brings it all together

**First working milestone:**
Complete Phases 1-4 to have a working command-line scanner:
```python
python3 examples/test_scan.py
```

**Second milestone:**
Complete Phase 5 for full UI integration with SpectrumBoi.

---

## Questions & Decisions

**Before starting implementation, decide:**

1. **Image storage format:** TIFF (lossless, large) vs PNG (compressed) vs NPY (fastest)?
   - Recommendation: TIFF for archival, NPY for processing

2. **Memory management:** Keep all images in RAM vs save immediately and discard?
   - Recommendation: Save immediately for large scans (360+ images)

3. **Error handling:** Skip failed positions vs abort entire scan?
   - Recommendation: Configurable via ScanConfig.skip_failed_positions

4. **Data cube assembly:** Real-time vs post-processing?
   - Recommendation: Post-processing (out of scope for initial implementation)

5. **Preview updates:** Every image vs every Nth image to reduce overhead?
   - Recommendation: Every image, but use threading to avoid blocking

---

## Support & References

**Pseudocode files:**
- `controller_additions.pseudo.py` - Motor primitives
- `scan.pseudo.py` - Scan orchestration
- `spectrumboi_integration.pseudo.py` - UI integration

**Documentation:**
- `SCAN_ARCHITECTURE.md` - Overall architecture design
- `system_architecture_scope.md` - Original design document

**Existing code to reference:**
- `controller.py` - Existing motor control implementation
- `camera_control/` - Existing camera drivers and UI

**For questions:**
- Check architecture docs first
- Review pseudocode for implementation details
- Refer to existing controller.py for patterns
- Test incrementally with mock hardware before real hardware
