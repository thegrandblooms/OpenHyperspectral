# Scan Session Architecture

This document outlines the architecture for hyperspectral scanning orchestration.

## File Structure & Responsibilities

```
motor_control/
├── __init__.py                 # Package exports
├── controller.py               # LOW-LEVEL: MCU communication & primitives
├── scan.py                     # HIGH-LEVEL: Scan orchestration
└── config.py                   # Configuration & constants
```

---

## Separation of Concerns

### controller.py - Motor Primitives Layer
**Purpose**: Low-level motor operations and MCU communication

**Responsibilities**:
- Serial communication with ESP32
- Individual motor commands (enable, disable, move_to)
- Position reading from encoder
- Command/response protocol handling
- Position monitoring thread
- **NEW**: Move-and-verify primitive (single position)
- **NEW**: Encoder position query

**Does NOT**:
- Know about scans or sessions
- Interact with camera
- Manage metadata
- Track scan progress

### scan.py - Scan Orchestration Layer
**Purpose**: High-level scanning workflow coordination

**Responsibilities**:
- Scan session management (ScanSession class)
- Multi-position scan execution
- Camera trigger coordination
- Metadata collection and storage
- Progress tracking and callbacks
- Error recovery strategies
- **Integration point** between motor, camera, and UI

**Does NOT**:
- Communicate directly with MCU serial port
- Handle low-level motor commands
- Implement camera drivers

### config.py - Configuration
**Purpose**: Centralized configuration

**Responsibilities**:
- Scan parameters (tolerance, settling time, retries)
- Serial port settings
- Camera settings
- File paths and naming conventions

---

## Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    UI (SpectrumBoi)                          │
│  - Start scan button                                         │
│  - Progress updates                                          │
│  - Live preview                                              │
└────────────────┬────────────────────────────────────────────┘
                 │
                 │ start_scan(config)
                 │ progress_callback()
                 ↓
┌─────────────────────────────────────────────────────────────┐
│                 HyperspectralScanner                         │
│                    (scan.py)                                 │
│                                                              │
│  For each scan position:                                    │
│    1. motor.move_and_verify(angle)                          │
│    2. time.sleep(settling_time)                             │
│    3. camera.capture()                                      │
│    4. session.add_capture(image, metadata)                  │
│    5. callback(progress)                                    │
└──────┬────────────────────────────┬──────────────────────────┘
       │                            │
       │ move_and_verify(angle)     │ capture()
       │ get_position()             │
       ↓                            ↓
┌─────────────────┐         ┌─────────────────┐
│ MotorController │         │ CameraInterface │
│ (controller.py) │         │ (camera.py)     │
│                 │         │                 │
│ - move_to()     │         │ - capture()     │
│ - get_encoder() │         │ - set_exposure()│
│ - enable()      │         │                 │
└────────┬────────┘         └─────────────────┘
         │
         │ Serial commands
         ↓
┌─────────────────┐
│  ESP32 MCU      │
│  (firmware)     │
└─────────────────┘
```

---

## Integration Points

### 1. Motor Controller → Scan Session
**Interface**: `MotorController` class

```python
# Scan session uses these motor primitives:
actual_angle = motor.move_and_verify(target_angle, tolerance=0.5, max_retries=3)
current_position = motor.get_encoder_position()
motor.enable()
motor.disable()
```

### 2. Camera → Scan Session
**Interface**: `CameraInterface` abstract base class (adapter pattern)

```python
# Scan session uses camera through abstraction:
camera = MightexCameraAdapter(underlying_camera)
image = camera.capture()
camera.set_exposure(100)
```

### 3. Scan Session → UI
**Interface**: Callback functions and event system

```python
# UI registers callbacks:
scanner = HyperspectralScanner(motor, camera)
scanner.on_progress(lambda progress: ui.update_progress(progress))
scanner.on_capture(lambda img, meta: ui.show_preview(img))
scanner.on_complete(lambda session: ui.show_results(session))

# Start scan:
session = scanner.start_scan(config)
```

---

## Key Design Patterns

### 1. Separation of Concerns
- Motor layer knows nothing about scans
- Scan layer knows nothing about serial protocols
- Clean interfaces between layers

### 2. Dependency Injection
- Scanner receives motor and camera instances
- Allows easy testing with mock objects
- Supports multiple camera implementations

### 3. Observer Pattern (Callbacks)
- UI subscribes to scan events
- Scanner emits progress, capture, error events
- Decouples UI from scan logic

### 4. Adapter Pattern (Camera)
- Abstract camera interface
- Different adapters for Mightex, mock camera, etc.
- Scanner works with any camera implementation

---

## Error Handling Strategy

### Layer-Specific Error Handling

**controller.py** (Motor primitives):
- Raises exceptions for communication failures
- Retries at command level (configurable)
- Returns None or raises on unrecoverable errors

**scan.py** (Scan orchestration):
- Catches motor/camera exceptions
- Implements retry strategies for position errors
- Decides whether to skip frame, retry, or abort scan
- Logs all errors with context
- Notifies UI via error callbacks

**UI** (SpectrumBoi):
- Catches scan exceptions
- Displays user-friendly error messages
- Offers recovery options (retry, skip, abort)

---

## Threading Model

### controller.py Threading
```
Main Thread                 Position Monitor Thread
    │                               │
    │ send_command() ──────────────>│
    │                               │ reads serial
    │                               │ detects RESP_POSITION_REACHED
    │                               │ puts in queue
    │<──────────────────────────────│
    │ wait_for_position()           │
    │ (blocks on queue)             │
```

### scan.py Threading
```
Scan Thread                 UI Thread
    │                           │
    │ for each position:        │
    │   move_and_verify()       │
    │   capture()               │
    │   callback() ────────────>│ update UI
    │                           │ (non-blocking)
    │                           │
```

**Options**:
1. **Blocking scan** (simpler): UI starts scan in separate thread
2. **Async scan** (advanced): Scanner runs in background, yields control

**Recommendation**: Start with blocking scan in UI thread (simpler for now).

---

## Configuration Example

```python
# config.py
class ScanConfig:
    # Position control
    position_tolerance_deg = 0.5        # Max acceptable error
    settling_time_ms = 500              # Vibration damping wait
    max_move_retries = 3                # Retry failed moves

    # Scan pattern
    start_angle_deg = 0.0
    end_angle_deg = 360.0
    increment_deg = 0.5

    # Camera
    exposure_ms = 100
    gain = 1.0

    # Data storage
    output_dir = "./scans"
    session_name_format = "scan_{timestamp}"
    image_format = "tiff"  # or "png", "npy"
```

---

## Next Steps for Implementation

1. **Add to controller.py**: `move_and_verify()` and `get_encoder_position()`
2. **Create scan.py**: `HyperspectralScanner` and `ScanSession` classes
3. **Create config.py**: Configuration dataclass
4. **Create camera adapter**: Abstract interface + Mightex implementation
5. **Update SpectrumBoi**: Integrate scanner with UI callbacks
6. **Add tests**: Unit tests for each layer, integration test for full scan

See pseudocode files for detailed implementation.
