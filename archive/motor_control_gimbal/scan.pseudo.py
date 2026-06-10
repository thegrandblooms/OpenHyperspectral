"""
PSEUDOCODE: scan.py - Hyperspectral Scan Orchestration

This module coordinates motor movements with camera captures for hyperspectral
line-scanning imaging. It sits above the motor controller and camera driver,
orchestrating the stop-and-verify scanning workflow.

Architecture:
    - HyperspectralScanner: Main orchestrator class
    - ScanSession: Data container for scan metadata and results
    - ScanConfig: Configuration dataclass
    - CameraInterface: Abstract base class for camera adapters

Dependencies:
    - motor_control.controller.MotorController
    - camera_control (via adapter pattern)
"""

import time
import logging
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Callable, Any
from datetime import datetime
from pathlib import Path
from abc import ABC, abstractmethod
import json
import numpy as np

logger = logging.getLogger("HyperspectralScanner")


#=============================================================================
# CONFIGURATION
#=============================================================================

@dataclass
class ScanConfig:
    """Configuration for a hyperspectral scan session."""

    # Scan range
    start_angle_deg: float = 0.0
    end_angle_deg: float = 360.0
    increment_deg: float = 0.5

    # Position control
    position_tolerance_deg: float = 0.5      # Max acceptable position error
    settling_time_sec: float = 0.5           # Vibration damping wait
    max_move_retries: int = 3                # Retry attempts for failed moves

    # Camera settings
    exposure_ms: float = 100.0
    gain: float = 1.0
    pre_capture_delay_sec: float = 0.1       # Additional settling before capture

    # Data storage
    output_dir: Path = Path("./scans")
    session_name: Optional[str] = None       # Auto-generated if None
    save_images: bool = True
    image_format: str = "tiff"               # "tiff", "png", or "npy"

    # Error handling
    skip_failed_positions: bool = False      # If True, skip; if False, abort scan
    max_consecutive_failures: int = 5        # Abort if this many failures in a row

    def __post_init__(self):
        """Validate configuration."""
        if self.start_angle_deg >= self.end_angle_deg:
            raise ValueError("start_angle must be less than end_angle")
        if self.increment_deg <= 0:
            raise ValueError("increment must be positive")
        if self.position_tolerance_deg <= 0:
            raise ValueError("tolerance must be positive")

    @property
    def num_positions(self) -> int:
        """Calculate total number of scan positions."""
        return int((self.end_angle_deg - self.start_angle_deg) / self.increment_deg) + 1

    @property
    def scan_angles(self) -> List[float]:
        """Generate list of all scan angles."""
        return [
            self.start_angle_deg + i * self.increment_deg
            for i in range(self.num_positions)
        ]


#=============================================================================
# CAMERA INTERFACE (Adapter Pattern)
#=============================================================================

class CameraInterface(ABC):
    """
    Abstract interface for camera adapters.

    This allows the scanner to work with any camera implementation
    (Mightex, mock camera for testing, etc.)
    """

    @abstractmethod
    def capture(self) -> np.ndarray:
        """
        Capture a single image.

        Returns:
            np.ndarray: Captured image data
        """
        pass

    @abstractmethod
    def set_exposure(self, exposure_ms: float):
        """Set camera exposure time in milliseconds."""
        pass

    @abstractmethod
    def set_gain(self, gain: float):
        """Set camera gain."""
        pass

    @abstractmethod
    def get_frame_shape(self) -> tuple:
        """Return (height, width) of captured frames."""
        pass


class MightexCameraAdapter(CameraInterface):
    """Adapter for Mightex S-Series cameras."""

    def __init__(self, mightex_camera):
        """
        Args:
            mightex_camera: Instance of camera_control.mightex_driver.camera.Camera
        """
        self.camera = mightex_camera

    def capture(self) -> np.ndarray:
        # IMPLEMENTATION: Call underlying Mightex camera API
        # return self.camera.capture_frame()
        pass

    def set_exposure(self, exposure_ms: float):
        # IMPLEMENTATION: Set exposure via Mightex API
        # self.camera.set_exposure(int(exposure_ms))
        pass

    def set_gain(self, gain: float):
        # IMPLEMENTATION: Set gain via Mightex API
        # self.camera.set_gain(gain)
        pass

    def get_frame_shape(self) -> tuple:
        # IMPLEMENTATION: Return frame dimensions
        # return (self.camera.height, self.camera.width)
        pass


class MockCameraAdapter(CameraInterface):
    """Mock camera for testing without hardware."""

    def __init__(self, width=1920, height=1200):
        self.width = width
        self.height = height
        self.exposure = 100.0
        self.gain = 1.0

    def capture(self) -> np.ndarray:
        # Return random noise as test data
        return np.random.randint(0, 255, (self.height, self.width), dtype=np.uint8)

    def set_exposure(self, exposure_ms: float):
        self.exposure = exposure_ms

    def set_gain(self, gain: float):
        self.gain = gain

    def get_frame_shape(self) -> tuple:
        return (self.height, self.width)


#=============================================================================
# SCAN SESSION (Data Container)
#=============================================================================

@dataclass
class CaptureMetadata:
    """Metadata for a single image capture."""
    index: int                          # Sequential capture index
    commanded_angle_deg: float          # Target angle sent to motor
    actual_angle_deg: float             # Encoder-confirmed angle
    position_error_deg: float           # |actual - commanded|
    timestamp: datetime
    move_retries: int                   # Number of retries needed
    exposure_ms: float
    gain: float
    image_path: Optional[Path] = None   # Path to saved image file


@dataclass
class ScanSession:
    """
    Container for scan session data and metadata.

    Tracks all captures, position errors, timing, and provides
    methods for saving results.
    """
    config: ScanConfig
    session_id: str = field(default_factory=lambda: datetime.now().strftime("%Y%m%d_%H%M%S"))
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None
    captures: List[CaptureMetadata] = field(default_factory=list)
    failed_positions: List[float] = field(default_factory=list)
    images: List[np.ndarray] = field(default_factory=list)  # Optional: keep in memory

    @property
    def duration_seconds(self) -> float:
        """Total scan duration."""
        if self.start_time and self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0

    @property
    def num_successful_captures(self) -> int:
        return len(self.captures)

    @property
    def num_failed_positions(self) -> int:
        return len(self.failed_positions)

    @property
    def success_rate(self) -> float:
        total = self.num_successful_captures + self.num_failed_positions
        if total == 0:
            return 0.0
        return self.num_successful_captures / total

    @property
    def mean_position_error(self) -> float:
        """Average position error across all captures."""
        if not self.captures:
            return 0.0
        return np.mean([c.position_error_deg for c in self.captures])

    @property
    def max_position_error(self) -> float:
        """Maximum position error encountered."""
        if not self.captures:
            return 0.0
        return max(c.position_error_deg for c in self.captures)

    def add_capture(self, image: np.ndarray, metadata: CaptureMetadata):
        """Add a successful capture to the session."""
        self.captures.append(metadata)
        if self.config.save_images:
            self.images.append(image)

    def add_failed_position(self, angle: float):
        """Record a position that failed verification."""
        self.failed_positions.append(angle)

    def save_metadata(self, output_dir: Path):
        """Save session metadata to JSON file."""
        metadata_dict = {
            'session_id': self.session_id,
            'start_time': self.start_time.isoformat() if self.start_time else None,
            'end_time': self.end_time.isoformat() if self.end_time else None,
            'duration_seconds': self.duration_seconds,
            'config': {
                'start_angle_deg': self.config.start_angle_deg,
                'end_angle_deg': self.config.end_angle_deg,
                'increment_deg': self.config.increment_deg,
                'position_tolerance_deg': self.config.position_tolerance_deg,
                'settling_time_sec': self.config.settling_time_sec,
                'exposure_ms': self.config.exposure_ms,
                'gain': self.config.gain,
            },
            'statistics': {
                'num_successful_captures': self.num_successful_captures,
                'num_failed_positions': self.num_failed_positions,
                'success_rate': self.success_rate,
                'mean_position_error_deg': self.mean_position_error,
                'max_position_error_deg': self.max_position_error,
            },
            'captures': [
                {
                    'index': c.index,
                    'commanded_angle_deg': c.commanded_angle_deg,
                    'actual_angle_deg': c.actual_angle_deg,
                    'position_error_deg': c.position_error_deg,
                    'timestamp': c.timestamp.isoformat(),
                    'move_retries': c.move_retries,
                    'image_path': str(c.image_path) if c.image_path else None,
                }
                for c in self.captures
            ],
            'failed_positions': self.failed_positions,
        }

        metadata_path = output_dir / f"{self.session_id}_metadata.json"
        with open(metadata_path, 'w') as f:
            json.dump(metadata_dict, f, indent=2)

        logger.info(f"Saved session metadata to {metadata_path}")

    def save_images(self, output_dir: Path, format: str = "tiff"):
        """Save all captured images to disk."""
        if not self.images:
            logger.warning("No images to save (not stored in memory)")
            return

        for i, (image, capture) in enumerate(zip(self.images, self.captures)):
            filename = f"{self.session_id}_img_{i:04d}_angle_{capture.actual_angle_deg:.2f}.{format}"
            filepath = output_dir / filename

            # IMPLEMENTATION: Save based on format
            # if format == "tiff":
            #     tifffile.imwrite(filepath, image)
            # elif format == "png":
            #     cv2.imwrite(str(filepath), image)
            # elif format == "npy":
            #     np.save(filepath, image)

            capture.image_path = filepath

        logger.info(f"Saved {len(self.images)} images to {output_dir}")


#=============================================================================
# HYPERSPECTRAL SCANNER (Main Orchestrator)
#=============================================================================

class HyperspectralScanner:
    """
    Hyperspectral scanning orchestrator.

    Coordinates motor movements with camera captures using a stop-and-verify
    workflow optimized for position accuracy over speed.

    Example:
        >>> from motor_control import MotorController
        >>> from camera_control import Camera
        >>>
        >>> motor = MotorController('/dev/ttyUSB0')
        >>> camera = MightexCameraAdapter(Camera())
        >>> scanner = HyperspectralScanner(motor, camera)
        >>>
        >>> config = ScanConfig(
        ...     start_angle_deg=0,
        ...     end_angle_deg=360,
        ...     increment_deg=1.0
        ... )
        >>>
        >>> session = scanner.run_scan(config)
        >>> print(f"Captured {session.num_successful_captures} images")
    """

    def __init__(self, motor: 'MotorController', camera: CameraInterface):
        """
        Args:
            motor: Connected MotorController instance
            camera: Camera adapter implementing CameraInterface
        """
        self.motor = motor
        self.camera = camera

        # Callback hooks (set via on_* methods)
        self._on_progress: Optional[Callable] = None
        self._on_capture: Optional[Callable] = None
        self._on_error: Optional[Callable] = None
        self._on_complete: Optional[Callable] = None

        # State
        self._scanning = False
        self._session: Optional[ScanSession] = None

    #=========================================================================
    # CALLBACK REGISTRATION
    #=========================================================================

    def on_progress(self, callback: Callable[[int, int, float], None]):
        """
        Register progress callback.

        Args:
            callback: Function(current_index, total_positions, current_angle)
        """
        self._on_progress = callback

    def on_capture(self, callback: Callable[[np.ndarray, CaptureMetadata], None]):
        """
        Register capture callback (called after each successful capture).

        Args:
            callback: Function(image, metadata)
        """
        self._on_capture = callback

    def on_error(self, callback: Callable[[str, Exception], None]):
        """
        Register error callback.

        Args:
            callback: Function(error_message, exception)
        """
        self._on_error = callback

    def on_complete(self, callback: Callable[[ScanSession], None]):
        """
        Register completion callback.

        Args:
            callback: Function(session)
        """
        self._on_complete = callback

    #=========================================================================
    # MAIN SCAN EXECUTION
    #=========================================================================

    def run_scan(self, config: ScanConfig) -> ScanSession:
        """
        Execute a hyperspectral scan.

        Workflow:
            1. Initialize session and output directory
            2. Configure camera settings
            3. Enable motor
            4. For each scan position:
                a. Move and verify position
                b. Wait for settling
                c. Capture image
                d. Store with metadata
                e. Emit progress callback
            5. Disable motor
            6. Save session data
            7. Return session object

        Args:
            config: Scan configuration

        Returns:
            ScanSession: Completed scan session with all data

        Raises:
            RuntimeError: If scan fails unrecoverably
        """
        # INITIALIZATION
        logger.info("=" * 70)
        logger.info("Starting Hyperspectral Scan")
        logger.info("=" * 70)

        self._scanning = True
        session = ScanSession(config=config)
        session.start_time = datetime.now()

        # Create output directory
        output_dir = config.output_dir / session.session_id
        output_dir.mkdir(parents=True, exist_ok=True)
        logger.info(f"Output directory: {output_dir}")

        # Configure camera
        self.camera.set_exposure(config.exposure_ms)
        self.camera.set_gain(config.gain)
        logger.info(f"Camera configured: exposure={config.exposure_ms}ms, gain={config.gain}")

        # Enable motor
        self.motor.enable()
        logger.info("Motor enabled")

        # Generate scan positions
        scan_angles = config.scan_angles
        total_positions = len(scan_angles)
        logger.info(f"Scan range: {config.start_angle_deg}° to {config.end_angle_deg}°")
        logger.info(f"Increment: {config.increment_deg}°")
        logger.info(f"Total positions: {total_positions}")
        logger.info("-" * 70)

        # SCAN LOOP
        consecutive_failures = 0

        try:
            for idx, target_angle in enumerate(scan_angles):
                if not self._scanning:
                    logger.warning("Scan aborted by user")
                    break

                logger.info(f"[{idx + 1}/{total_positions}] Moving to {target_angle:.2f}°...")

                # Emit progress callback
                if self._on_progress:
                    self._on_progress(idx, total_positions, target_angle)

                # STEP 1: Move and verify position
                try:
                    success, actual_angle = self.motor.move_and_verify(
                        target_angle,
                        tolerance=config.position_tolerance_deg,
                        max_retries=config.max_move_retries,
                        settling_time=config.settling_time_sec
                    )

                    if not success:
                        # Position verification failed
                        position_error = abs(actual_angle - target_angle)
                        logger.error(f"Position verification failed: "
                                    f"error={position_error:.3f}° > tolerance={config.position_tolerance_deg}°")

                        session.add_failed_position(target_angle)
                        consecutive_failures += 1

                        if self._on_error:
                            self._on_error(f"Position verification failed at {target_angle:.2f}°", None)

                        # Check failure threshold
                        if consecutive_failures >= config.max_consecutive_failures:
                            raise RuntimeError(f"Scan aborted: {consecutive_failures} consecutive position failures")

                        # Skip or abort based on config
                        if config.skip_failed_positions:
                            logger.warning(f"Skipping position {target_angle:.2f}°")
                            continue
                        else:
                            raise RuntimeError(f"Scan aborted due to position verification failure")

                except Exception as e:
                    logger.error(f"Motor error at position {target_angle:.2f}°: {e}")
                    session.add_failed_position(target_angle)
                    consecutive_failures += 1

                    if self._on_error:
                        self._on_error(f"Motor error at {target_angle:.2f}°", e)

                    if not config.skip_failed_positions:
                        raise

                    continue

                # Position reached successfully
                consecutive_failures = 0  # Reset failure counter
                position_error = abs(actual_angle - target_angle)

                # STEP 2: Additional pre-capture settling
                if config.pre_capture_delay_sec > 0:
                    time.sleep(config.pre_capture_delay_sec)

                # STEP 3: Capture image
                try:
                    logger.info(f"Capturing image at {actual_angle:.2f}°...")
                    image = self.camera.capture()

                    # Create metadata
                    metadata = CaptureMetadata(
                        index=idx,
                        commanded_angle_deg=target_angle,
                        actual_angle_deg=actual_angle,
                        position_error_deg=position_error,
                        timestamp=datetime.now(),
                        move_retries=config.max_move_retries,  # TODO: track actual retries
                        exposure_ms=config.exposure_ms,
                        gain=config.gain
                    )

                    # Store capture
                    session.add_capture(image, metadata)

                    # Emit capture callback
                    if self._on_capture:
                        self._on_capture(image, metadata)

                    logger.info(f"✓ Captured image {idx + 1}/{total_positions} "
                               f"(error: {position_error:.3f}°)")

                except Exception as e:
                    logger.error(f"Camera capture failed at {actual_angle:.2f}°: {e}")
                    if self._on_error:
                        self._on_error(f"Camera error at {actual_angle:.2f}°", e)

                    if not config.skip_failed_positions:
                        raise

        finally:
            # CLEANUP
            self.motor.disable()
            logger.info("Motor disabled")

            session.end_time = datetime.now()
            self._scanning = False

        # SAVE RESULTS
        logger.info("-" * 70)
        logger.info("Scan Complete")
        logger.info(f"Duration: {session.duration_seconds:.1f} seconds")
        logger.info(f"Successful captures: {session.num_successful_captures}")
        logger.info(f"Failed positions: {session.num_failed_positions}")
        logger.info(f"Success rate: {session.success_rate * 100:.1f}%")
        logger.info(f"Mean position error: {session.mean_position_error:.3f}°")
        logger.info(f"Max position error: {session.max_position_error:.3f}°")

        # Save data
        if config.save_images:
            session.save_images(output_dir, format=config.image_format)
        session.save_metadata(output_dir)

        # Emit completion callback
        if self._on_complete:
            self._on_complete(session)

        logger.info(f"Results saved to {output_dir}")
        logger.info("=" * 70)

        return session

    def abort_scan(self):
        """Stop the current scan (call from another thread)."""
        self._scanning = False
        logger.warning("Scan abort requested")


#=============================================================================
# USAGE EXAMPLE
#=============================================================================

if __name__ == "__main__":
    """
    Example usage demonstrating the scanning workflow.
    """

    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # PSEUDOCODE - actual imports would be:
    # from motor_control.controller import MotorController
    # from camera_control import Camera

    # Initialize motor
    # motor = MotorController('/dev/ttyUSB0')
    # motor.connect()
    # motor.calibrate()  # If not already calibrated

    # Initialize camera (via adapter)
    # camera = MightexCameraAdapter(Camera())
    # OR for testing:
    # camera = MockCameraAdapter()

    # Create scanner
    # scanner = HyperspectralScanner(motor, camera)

    # Register callbacks for UI updates
    def on_progress(current, total, angle):
        print(f"Progress: {current}/{total} - Moving to {angle:.2f}°")

    def on_capture(image, metadata):
        print(f"Captured: {metadata.actual_angle_deg:.2f}° "
              f"(error: {metadata.position_error_deg:.3f}°)")

    # scanner.on_progress(on_progress)
    # scanner.on_capture(on_capture)

    # Configure scan
    # config = ScanConfig(
    #     start_angle_deg=0.0,
    #     end_angle_deg=360.0,
    #     increment_deg=1.0,
    #     position_tolerance_deg=0.5,
    #     settling_time_sec=0.5,
    #     exposure_ms=100.0,
    #     output_dir=Path("./scans"),
    # )

    # Run scan
    # session = scanner.run_scan(config)

    # Print results
    # print(f"\nScan complete!")
    # print(f"Captured: {session.num_successful_captures} images")
    # print(f"Failed: {session.num_failed_positions} positions")
    # print(f"Mean error: {session.mean_position_error:.3f}°")

    # Cleanup
    # motor.disconnect()

    print("See pseudocode above for implementation")
