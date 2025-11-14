"""
Mightex S-Series USB Camera Controller
Cross-platform Python implementation for Raspberry Pi OS and Windows

Should be compatible with camera models:
- SCN/SCE-BG04-U, SCN/SCE-CG04-U (752x480)
- SCN/SCE-B013-U, SCN/SCE-C013-U (1280x1024)
- SCN/SCE-C030-U (2048x1536)
"""

import usb.core
import usb.util
import numpy as np
import time
import struct
import threading
import platform
import queue
from enum import Enum
from typing import Optional, Tuple, List
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Platform detection
IS_WINDOWS = platform.system().lower() == "windows"


class CameraMode(Enum):
    NORMAL = 0x00
    TRIGGER = 0x01


class SensorClock(Enum):
    SLOW = 0     # Slow Clock (Not recommended)
    NORMAL = 1   # Normal Clock
    FAST = 2     # Fast Clock


class HBlanking(Enum):
    SHORT = 0    # Short HBlanking Time
    LONG = 1     # Long HBlanking Time  
    LONGEST = 2  # Longest HBlanking Time


class DeviceInfo:
    """Device information structure"""
    def __init__(self, data: bytes = None):
        if data and len(data) >= 43:
            self.config_revision = data[0]
            self.module_no = data[1:15].decode('ascii', errors='ignore').strip('\x00')
            self.serial_no = data[15:29].decode('ascii', errors='ignore').strip('\x00')
            self.manufacture_date = data[29:43].decode('ascii', errors='ignore').strip('\x00')
        else:
            self.config_revision = 0
            self.module_no = "Unknown"
            self.serial_no = "Unknown"
            self.manufacture_date = "Unknown"
    
    def __str__(self):
        return f"Module: {self.module_no}, Serial: {self.serial_no}, Date: {self.manufacture_date}"


class FrameProperty:
    """Frame property structure returned after capture"""
    def __init__(self, data: bytes = None):
        if data and len(data) >= 18:
            # Parse frame property according to protocol
            self.row_size = struct.unpack('>H', data[0:2])[0]
            self.column_size = struct.unpack('>H', data[2:4])[0]
            self.bin_mode = data[4]
            self.exposure_time = struct.unpack('>H', data[5:7])[0]  # in 50us units
            self.red_gain = data[7]
            self.green_gain = data[8]
            self.blue_gain = data[9]
            self.x_start = struct.unpack('>H', data[10:12])[0]
            self.y_start = struct.unpack('>H', data[12:14])[0]
            self.frame_invalid = data[14]
            # data[15] is reserved
            self.timestamp = struct.unpack('>H', data[16:18])[0]
        else:
            # Default values
            self.row_size = 0
            self.column_size = 0
            self.bin_mode = 0
            self.exposure_time = 0
            self.red_gain = 16
            self.green_gain = 16
            self.blue_gain = 16
            self.x_start = 0
            self.y_start = 0
            self.frame_invalid = 0
            self.timestamp = 0


class MightexCamera:
    """
    Cross-platform controller for Mightex S-Series USB cameras
    
    Key Features:
    - Direct USB protocol implementation (no SDK dependency)
    - Cross-platform compatibility (Windows, Linux, Raspberry Pi)
    - Proper resolution command interpretation
    - Threaded USB reading to prevent endpoint starvation
    - BG04/CG04 invalid frame handling
    - Full camera control (exposure, gains, GPIO, triggers)
    
    Usage:
        camera = MightexCamera(0)
        if camera.connect():
            camera.set_resolution(752, 480)
            frame = camera.capture_frame()
            camera.disconnect()
    """
    
    # USB identifiers
    VENDOR_ID = 0x04B4  # Cypress
    PRODUCT_ID = 0x0228
    
    # Endpoints (from protocol doc)
    EP_CMD_OUT = 0x01      # Command output
    EP_CMD_IN = 0x81       # Command input (response)
    EP_DATA_IN_EVEN = 0x82 # Even rows data
    EP_DATA_IN_ODD = 0x86  # Odd rows data
    
    # Commands from protocol documentation
    CMD_QUERY_FW_VERSION = 0x01
    CMD_QUERY_DEVICE_INFO = 0x21
    CMD_USER_EEPROM_WRITE = 0x25
    CMD_USER_EEPROM_READ = 0x26
    CMD_SET_WORK_MODE = 0x30
    CMD_SET_SENSOR_CLOCK = 0x32
    CMD_GET_FRAME_PROPERTY = 0x33
    CMD_GET_IMAGE_DATA = 0x34
    CMD_GET_TRIGGER_STATE = 0x35
    CMD_SET_HBLANKING = 0x36
    CMD_GPIO_WRITE = 0x40
    CMD_GPIO_READ = 0x41
    CMD_SET_RESOLUTION = 0x60
    CMD_SET_XY_START = 0x61
    CMD_SET_GAINS = 0x62
    CMD_SET_EXPOSURE = 0x63
    CMD_SOFT_TRIGGER = 0x65
    
    def __init__(self, device_index: int = 0):
        """
        Initialize camera controller
        
        Args:
            device_index: Index of camera if multiple are connected (0-based)
        """
        self.device = None
        self.device_index = device_index
        self.device_info = None
        self.current_mode = CameraMode.NORMAL
        self.current_resolution = (0, 0)
        self.current_bin_mode = 0
        self.max_resolution = (752, 480)  # Default, updated based on model
        
    def connect(self) -> bool:
        """
        Connect to the camera
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            # Find all Mightex cameras
            devices = list(usb.core.find(
                find_all=True,
                idVendor=self.VENDOR_ID,
                idProduct=self.PRODUCT_ID
            ))
            
            if not devices:
                logger.error("No Mightex cameras found")
                return False
                
            if self.device_index >= len(devices):
                logger.error(f"Device index {self.device_index} out of range (found {len(devices)} devices)")
                return False
                
            self.device = devices[self.device_index]
            
            # Handle kernel driver (Linux only)
            if not IS_WINDOWS:
                try:
                    if self.device.is_kernel_driver_active(0):
                        self.device.detach_kernel_driver(0)
                except Exception as e:
                    logger.warning(f"Could not detach kernel driver: {e}")
                
            # Set configuration
            try:
                self.device.set_configuration()
            except usb.core.USBError as e:
                logger.warning(f"Configuration warning: {e}")
                
            # Query device info and determine capabilities
            self._query_device_info()
            
            logger.info(f"Connected to camera: {self.device_info}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from the camera"""
        if self.device:
            try:
                usb.util.dispose_resources(self.device)
            except Exception as e:
                logger.warning(f"Disconnect warning: {e}")
            finally:
                self.device = None
            logger.info("Disconnected from camera")
            
    def _send_command(self, cmd: int, data: bytes = b'', expect_response: bool = None) -> bytes:
        """
        Send command to camera following the protocol:
        EP(0x01) → CommandID Length Data
        EP(0x81) ← Result Length Data
        
        Args:
            cmd: Command ID
            data: Command data payload
            expect_response: Whether to expect a response (auto-detected if None)
            
        Returns:
            bytes: Response data (empty if no response expected)
        """
        if not self.device:
            raise RuntimeError("Camera not connected")
            
        # Commands that expect responses (from protocol doc)
        response_commands = {
            self.CMD_QUERY_FW_VERSION,
            self.CMD_QUERY_DEVICE_INFO,
            self.CMD_USER_EEPROM_READ,
            self.CMD_GET_FRAME_PROPERTY,
            self.CMD_GET_TRIGGER_STATE,
            self.CMD_GPIO_READ
        }
        
        if expect_response is None:
            expect_response = cmd in response_commands
            
        # Prepare command packet: CommandID + Length + Data
        packet = struct.pack('BB', cmd, len(data)) + data
        
        try:
            # Send command on EP1 OUT
            self.device.write(self.EP_CMD_OUT, packet, timeout=5000)
            
            if expect_response:
                # Read response on EP1 IN
                try:
                    response = self.device.read(self.EP_CMD_IN, 512, timeout=5000)
                    if len(response) >= 2:
                        result = response[0]  # 0x01 = OK, 0x00 = Error
                        length = response[1]  # Length of response data
                        
                        if result == 0x01:  # Success
                            return bytes(response[2:2+length]) if length > 0 else b''
                        else:
                            logger.warning(f"Command 0x{cmd:02X} returned error")
                            return b''
                    else:
                        logger.warning(f"Command 0x{cmd:02X} response too short")
                        return b''
                        
                except usb.core.USBTimeoutError:
                    logger.warning(f"Command 0x{cmd:02X} response timeout")
                    return b''
                    
            return b''
            
        except Exception as e:
            logger.error(f"Command 0x{cmd:02X} failed: {e}")
            return b''
    
    def _query_device_info(self):
        """Query device information and determine camera capabilities"""
        try:
            data = self._send_command(self.CMD_QUERY_DEVICE_INFO, b'\x00')
            self.device_info = DeviceInfo(data)
            
            # Determine max resolution based on module
            module = self.device_info.module_no.upper()
            if 'BG04' in module or 'CG04' in module:
                self.max_resolution = (752, 480)
            elif 'B013' in module or 'C013' in module:
                self.max_resolution = (1280, 1024)
            elif 'C030' in module:
                self.max_resolution = (2048, 1536)
            else:
                self.max_resolution = (752, 480)  # Conservative default
                    
        except Exception as e:
            logger.warning(f"Failed to query device info: {e}")
            self.device_info = DeviceInfo()  # Use defaults
            
    def get_firmware_version(self) -> Tuple[int, int, int]:
        """
        Get camera firmware version
        
        Returns:
            Tuple[int, int, int]: (major, minor, revision)
        """
        data = self._send_command(self.CMD_QUERY_FW_VERSION, b'\x01')
        if len(data) >= 3:
            return (data[0], data[1], data[2])
        return (0, 0, 0)
        
    def set_mode(self, mode: CameraMode):
        """
        Set camera work mode
        
        Args:
            mode: NORMAL for continuous capture, TRIGGER for external trigger
        """
        self._send_command(self.CMD_SET_WORK_MODE, bytes([mode.value]))
        self.current_mode = mode
        logger.info(f"Set camera mode to {mode.name}")
        
    def set_sensor_clock(self, clock: SensorClock):
        """
        Set sensor clock frequency
        
        Args:
            clock: SLOW/NORMAL/FAST (FAST degrades SNR on BG04/CG04)
        """
        self._send_command(self.CMD_SET_SENSOR_CLOCK, bytes([clock.value]))
        # Protocol specifies >200ms wait after this command
        time.sleep(0.25)
        logger.info(f"Set sensor clock to {clock.name}")
        
    def set_hblanking(self, hblanking: HBlanking):
        """
        Set horizontal blanking time
        
        Args:
            hblanking: SHORT/LONG/LONGEST (longer = slower frame rate, better SNR)
        """
        self._send_command(self.CMD_SET_HBLANKING, bytes([hblanking.value]))
        logger.info(f"Set HBlanking to {hblanking.name}")
        
    def set_resolution(self, width: int, height: int, binning: bool = False):
        """
        Set camera resolution
        
        CRITICAL FIX: Protocol expects RowSize=width, ColumnSize=height
        (Previous implementation had this reversed)
        
        Args:
            width: Image width in pixels (must be multiple of 4)
            height: Image height in pixels (must be multiple of 4)
            binning: Enable 2x2 binning for higher sensitivity
        """
        # Validate and align to 4-pixel boundaries
        width = max(4, (width // 4) * 4)
        height = max(4, (height // 4) * 4)
        
        if binning:
            width = max(8, width)
            height = max(8, height)
            
        if width > self.max_resolution[0] or height > self.max_resolution[1]:
            raise ValueError(f"Resolution {width}x{height} exceeds maximum {self.max_resolution}")
            
        bin_mode = 1 if binning else 0
        
        # CRITICAL: Based on protocol documentation example
        # "set camera to 1024x768: 0x60 7 4 0 3 0 1"
        # (4 0) = 0x0400 = 1024 = RowSize = WIDTH
        # (3 0) = 0x0300 = 768 = ColumnSize = HEIGHT
        row_size = width      # RowSize = width (pixels per row)
        column_size = height  # ColumnSize = height (number of rows)
        
        data = struct.pack('>HHB', row_size, column_size, bin_mode)
        self._send_command(self.CMD_SET_RESOLUTION, data)
        
        self.current_resolution = (width, height)
        self.current_bin_mode = bin_mode
        logger.info(f"Set resolution to {width}x{height} (binning: {binning})")
        
        # Wait for camera to process the resolution change
        time.sleep(0.1)
        
        # Verify the setting
        trigger_state, rows, cols, bin_mode_reported = self.get_trigger_state()
        if (rows, cols, bin_mode_reported) != (row_size, column_size, bin_mode):
            logger.warning(f"Camera setting mismatch! Expected ({row_size}, {column_size}, {bin_mode}), got ({rows}, {cols}, {bin_mode_reported})")
        
    def set_xy_start(self, x_start: int, y_start: int):
        """
        Set ROI starting position
        
        Args:
            x_start: X offset for region of interest
            y_start: Y offset for region of interest
        """
        data = struct.pack('>HH', x_start, y_start)
        self._send_command(self.CMD_SET_XY_START, data)
        logger.info(f"Set ROI position to ({x_start}, {y_start})")
        
    def set_exposure_time(self, exposure_ms: float):
        """
        Set exposure time
        
        Args:
            exposure_ms: Exposure time in milliseconds (0.05 - 750ms)
        """
        if not 0.05 <= exposure_ms <= 750:
            raise ValueError("Exposure time must be between 0.05 and 750 ms")
            
        # Convert to 50us units
        exposure_units = int(exposure_ms / 0.05)
        data = struct.pack('>H', exposure_units)
        self._send_command(self.CMD_SET_EXPOSURE, data)
        logger.info(f"Set exposure time to {exposure_ms} ms")
        
    def set_gains(self, red: int, green: int, blue: int):
        """
        Set camera gains
        
        Args:
            red: Red channel gain
            green: Green channel gain  
            blue: Blue channel gain
            
        Note:
            - BG04/CG04: 8-32 (1x-4x)
            - Others: 1-64 (0.25x-8x)
            - For monochrome cameras, only green gain is used
        """
        # Determine valid range based on model
        module = self.device_info.module_no.upper() if self.device_info else ""
        if 'BG04' in module or 'CG04' in module:
            min_gain, max_gain = 8, 32
        else:
            min_gain, max_gain = 1, 64
            
        red = max(min_gain, min(red, max_gain))
        green = max(min_gain, min(green, max_gain))
        blue = max(min_gain, min(blue, max_gain))
        
        data = bytes([red, green, blue])
        self._send_command(self.CMD_SET_GAINS, data)
        logger.info(f"Set gains to R:{red} G:{green} B:{blue}")
        
    def soft_trigger(self):
        """Send software trigger (for TRIGGER mode)"""
        self._send_command(self.CMD_SOFT_TRIGGER, b'\x01')
        
    def get_trigger_state(self) -> Tuple[int, int, int, int]:
        """
        Get camera trigger state
        
        Returns:
            Tuple[int, int, int, int]: (trigger_state, row_size, column_size, bin_mode)
        """
        data = self._send_command(self.CMD_GET_TRIGGER_STATE, b'\x01')
        if len(data) >= 6:
            trigger_state = data[0]
            row_size = struct.unpack('>H', data[1:3])[0]
            column_size = struct.unpack('>H', data[3:5])[0]
            bin_mode = data[5]
            return (trigger_state, row_size, column_size, bin_mode)
        return (0, 0, 0, 0)
        
    def get_frame_property(self) -> FrameProperty:
        """
        Get frame property after capture
        
        Returns:
            FrameProperty: Frame metadata including exposure, gains, etc.
        """
        data = self._send_command(self.CMD_GET_FRAME_PROPERTY, b'\x00')
        return FrameProperty(data)
    
    def write_user_eeprom(self, address: int, data: bytes) -> bool:
        """
        Write data to user EEPROM area
        
        Args:
            address: EEPROM address (0x3C00-0x3FFF, 32-byte aligned)
            data: Data to write (exactly 32 bytes)
            
        Returns:
            bool: True if write successful
        """
        if not (0x3C00 <= address <= 0x3FFF):
            raise ValueError("Address must be in user EEPROM area (0x3C00-0x3FFF)")
        
        if len(data) != 32:
            raise ValueError("Data must be exactly 32 bytes")
            
        if address % 32 != 0:
            raise ValueError("Address must be aligned to 32-byte boundary")
            
        addr_data = struct.pack('>H', address) + data
        response = self._send_command(self.CMD_USER_EEPROM_WRITE, addr_data, expect_response=True)
        
        # Protocol specifies 5-10ms wait between EEPROM writes
        time.sleep(0.01)
        
        return len(response) > 0 and response[0] == 0x01
        
    def read_user_eeprom(self, address: int, size: int) -> bytes:
        """
        Read data from user EEPROM area
        
        Args:
            address: EEPROM address (0x3C00-0x3FFF)
            size: Number of bytes to read (1-32)
            
        Returns:
            bytes: Data read from EEPROM
        """
        if not (0x3C00 <= address <= 0x3FFF):
            raise ValueError("Address must be in user EEPROM area (0x3C00-0x3FFF)")
            
        if not (1 <= size <= 32):
            raise ValueError("Size must be between 1 and 32 bytes")
            
        addr_size_data = struct.pack('>HB', address, size)
        return self._send_command(self.CMD_USER_EEPROM_READ, addr_size_data)
    
    def gpio_config(self, config_byte: int):
        """
        Configure GPIO pins
        
        Args:
            config_byte: Configuration byte (bit 0=GPIO1, bit 1=GPIO2, etc.)
                        0=output, 1=input
        """
        # Write to configuration register (0x03) of PCA9536
        data = bytes([0x03, config_byte])
        self._send_command(self.CMD_GPIO_WRITE, data)
        logger.info(f"Configured GPIO: 0x{config_byte:02X}")
        
    def gpio_set_read(self, output_byte: int) -> int:
        """
        Set GPIO outputs and read inputs
        
        Args:
            output_byte: Output values (1=high, 0=low for output pins)
            
        Returns:
            int: Current pin states
        """
        # Set outputs (register 0x01)
        self._send_command(self.CMD_GPIO_WRITE, bytes([0x01, output_byte]))
        
        # Read inputs (register 0x00)
        data = self._send_command(self.CMD_GPIO_READ, bytes([0x00]))
        return data[0] if data else 0

    def wait_for_trigger(self, timeout_seconds: float = 5.0) -> bool:
        """
        Wait for trigger to be ready (TRIGGER mode only)
        
        Args:
            timeout_seconds: Maximum time to wait
            
        Returns:
            bool: True if trigger ready, False if timeout
        """
        if self.current_mode != CameraMode.TRIGGER:
            return True  # Always ready in normal mode
            
        start_time = time.time()
        while time.time() - start_time < timeout_seconds:
            trigger_state, _, _, _ = self.get_trigger_state()
            if trigger_state == 1:
                return True
            time.sleep(0.001)  # 1ms polling interval
            
        logger.warning(f"Trigger timeout after {timeout_seconds}s")
        return False
        
    def capture_frame(self, max_retries: int = 3) -> Optional[np.ndarray]:
        """
        Capture a single frame
        
        This method implements the critical fixes for:
        1. Proper USB endpoint reading using threading
        2. BG04/CG04 invalid frame re-capture handling
        3. Correct frame reconstruction from interlaced data
        
        Args:
            max_retries: Maximum number of retry attempts
            
        Returns:
            Optional[np.ndarray]: Captured frame as uint8 array, or None if failed
        """
        for retry in range(max_retries + 1):
            try:
                # Step 1: Check trigger state and parameters
                trigger_state, rows, cols, bin_mode = self.get_trigger_state()
                
                # In trigger mode, wait for trigger to be ready
                if self.current_mode == CameraMode.TRIGGER:
                    if not self.wait_for_trigger():
                        logger.error("Trigger not ready")
                        return None
                
                # With corrected resolution: rows=width, cols=height
                actual_width = rows   # Camera reports width as rows
                actual_height = cols  # Camera reports height as cols
                
                # Step 2: Request image data
                self._send_command(self.CMD_GET_IMAGE_DATA, b'\x01', expect_response=False)
                
                # Step 3: Calculate expected data size
                if bin_mode:
                    pixel_width = actual_width // 2
                    pixel_height = actual_height // 2
                else:
                    pixel_width = actual_width
                    pixel_height = actual_height
                    
                rows_per_endpoint = pixel_height // 2
                pixels_per_row = pixel_width
                bytes_per_endpoint = rows_per_endpoint * pixels_per_row
                
                # Step 4: Read both endpoints simultaneously using threads
                # This prevents EP6 from being starved while EP2 is being read
                even_data, odd_data = self._read_bulk_data_threaded(bytes_per_endpoint)
                
                # Step 5: Get frame property
                frame_prop = self.get_frame_property()
                
                # Step 6: Handle invalid frames for BG04/CG04
                if frame_prop.frame_invalid:
                    module = self.device_info.module_no.upper() if self.device_info else ""
                    if ('BG04' in module or 'CG04' in module) and retry < max_retries:
                        logger.warning(f"Invalid frame detected, re-capturing...")
                        self._send_command(self.CMD_GET_IMAGE_DATA, b'\x01', expect_response=False)
                        even_data, odd_data = self._read_bulk_data_threaded(bytes_per_endpoint)
                        frame_prop = self.get_frame_property()
                        
                        if not frame_prop.frame_invalid:
                            logger.info("Re-capture successful")
                        else:
                            continue  # Try full retry
                            
                # Step 7: Reconstruct frame
                if even_data and odd_data and len(even_data) > 1000 and len(odd_data) > 1000:
                    frame = self._reconstruct_frame(even_data, odd_data, pixel_width, pixel_height)
                    if frame is not None:
                        return frame
                        
                # If we get here, try again
                if retry < max_retries:
                    time.sleep(0.1)
                    
            except Exception as e:
                logger.error(f"Frame capture failed on attempt {retry + 1}: {e}")
                if retry < max_retries:
                    time.sleep(0.1)
                    continue
                else:
                    break
                    
        logger.error("All capture attempts failed")
        return None      
        
    def capture_multiple_frames(self, num_frames: int, mode: str = "sum", frame_delay: float = 0.1) -> Optional[np.ndarray]:
        """
        Capture multiple frames and combine them for extended exposure or noise reduction
        
        This method captures multiple frames and combines them to either:
        - Simulate longer exposure times beyond the 750ms hardware limit
        - Reduce noise through frame averaging
        
        Args:
            num_frames: Number of frames to capture and combine
            mode: Combination mode:
                  "average" - Average frames for noise reduction
                  "sum" - Add frames together for simulated long exposure
                  "mean" - Same as average (alias)
            frame_delay: Delay between captures in seconds (default 0.1s)
            
        Returns:
            Optional[np.ndarray]: Combined frame as uint8 array, or None if failed
            
        Examples:
            # Simulate 4.5 second exposure (6 × 750ms frames)
            long_exposure = camera.capture_multiple_frames(6, "sum")
            
            # Average 10 frames for noise reduction
            clean_frame = camera.capture_multiple_frames(10, "average")
        """
        if num_frames < 1:
            logger.error("Number of frames must be at least 1")
            return None
            
        if mode not in ["average", "sum", "mean"]:
            logger.error("Mode must be 'average', 'sum', or 'mean'")
            return None
            
        # Normalize mode names
        if mode == "mean":
            mode = "average"
            
        logger.info(f"Capturing {num_frames} frames for {mode} combination...")
        
        frames = []
        successful_captures = 0
        
        # Capture all frames
        for i in range(num_frames):
            logger.info(f"Capturing frame {i+1}/{num_frames}...")
            
            frame = self.capture_frame()
            if frame is not None:
                frames.append(frame)
                successful_captures += 1
                logger.info(f"Frame {i+1} captured successfully")
            else:
                logger.warning(f"Failed to capture frame {i+1}")
                
            # Add delay between captures (except for last frame)
            if i < num_frames - 1 and frame_delay > 0:
                time.sleep(frame_delay)
        
        if successful_captures == 0:
            logger.error("No frames were captured successfully")
            return None
            
        if successful_captures < num_frames:
            logger.warning(f"Only {successful_captures}/{num_frames} frames captured successfully")
        
        # Combine the frames
        try:
            if mode == "average":
                # Average the frames (noise reduction)
                logger.info(f"Averaging {successful_captures} frames...")
                
                # Convert to float32 for precise averaging
                accumulated = frames[0].astype(np.float32)
                for frame in frames[1:]:
                    accumulated += frame.astype(np.float32)
                    
                # Calculate average and convert back to uint8
                averaged = accumulated / successful_captures
                result = np.clip(averaged, 0, 255).astype(np.uint8)
                
                logger.info(f"Frame averaging complete. Effective noise reduction: ~{np.sqrt(successful_captures):.1f}x")
                
            elif mode == "sum":
                # Sum the frames (simulated long exposure)
                logger.info(f"Summing {successful_captures} frames for long exposure effect...")
                
                # Use uint32 to prevent overflow during accumulation
                accumulated = frames[0].astype(np.uint32)
                for frame in frames[1:]:
                    accumulated += frame.astype(np.uint32)
                
                # Handle potential overflow - we have a few options:
                # Option 1: Clip to 255 (may lose highlight detail)
                # Option 2: Scale down to preserve dynamic range
                
                max_value = np.max(accumulated)
                if max_value > 255:
                    # Scale down to preserve dynamic range
                    scale_factor = 255.0 / max_value
                    logger.info(f"Scaling down by {scale_factor:.3f} to prevent overflow")
                    result = (accumulated * scale_factor).astype(np.uint8)
                else:
                    # No overflow, direct conversion
                    result = accumulated.astype(np.uint8)
                
                # Calculate effective exposure time
                frame_prop = self.get_frame_property()
                single_exposure_ms = frame_prop.exposure_time * 0.05  # Convert from 50us units
                total_exposure_ms = single_exposure_ms * successful_captures
                
                logger.info(f"Long exposure simulation complete. Effective exposure: {total_exposure_ms:.1f}ms ({total_exposure_ms/1000:.2f}s)")
                
            return result
            
        except Exception as e:
            logger.error(f"Failed to combine frames: {e}")
            return None

    def _read_bulk_data_threaded(self, expected_size: int) -> Tuple[bytes, bytes]:
        """
        Read bulk data from both endpoints simultaneously using threads
        
        CRITICAL FIX: This prevents EP6 from being starved while EP2 is being read,
        which was causing the original issue where EP6 only received 2048 bytes.
        
        Args:
            expected_size: Expected bytes per endpoint
            
        Returns:
            Tuple[bytes, bytes]: (even_data, odd_data)
        """
        even_queue = queue.Queue()
        odd_queue = queue.Queue()
        
        def read_endpoint(endpoint, name):
            try:
                data = self._read_bulk_data_improved(endpoint, expected_size)
                if endpoint == self.EP_DATA_IN_EVEN:
                    even_queue.put(data)
                else:
                    odd_queue.put(data)
            except Exception as e:
                logger.error(f"Thread {name}: error {e}")
                if endpoint == self.EP_DATA_IN_EVEN:
                    even_queue.put(b'')
                else:
                    odd_queue.put(b'')
        
        # Start both reading threads
        thread_even = threading.Thread(target=read_endpoint, args=(self.EP_DATA_IN_EVEN, "EP2"))
        thread_odd = threading.Thread(target=read_endpoint, args=(self.EP_DATA_IN_ODD, "EP6"))
        
        thread_even.start()
        thread_odd.start()
        
        # Wait for both to complete
        thread_even.join(timeout=10)
        thread_odd.join(timeout=10)
        
        # Get results
        even_data = even_queue.get() if not even_queue.empty() else b''
        odd_data = odd_queue.get() if not odd_queue.empty() else b''
        
        return even_data, odd_data

    def _read_bulk_data_improved(self, endpoint: int, expected_size: int) -> bytes:
        """
        Improved bulk data reading with proper timeout handling
        
        Args:
            endpoint: USB endpoint to read from
            expected_size: Expected number of bytes
            
        Returns:
            bytes: Data read from endpoint
        """
        data = b''
        timeout = 1000  # 1 second timeout per chunk
        chunk_size = 512  # Standard USB packet size
        consecutive_timeouts = 0
        max_consecutive_timeouts = 10
        
        try:
            while len(data) < expected_size and consecutive_timeouts < max_consecutive_timeouts:
                try:
                    chunk = self.device.read(endpoint, chunk_size, timeout=timeout)
                    if chunk and len(chunk) > 0:
                        data += bytes(chunk)
                        consecutive_timeouts = 0  # Reset on successful read
                    else:
                        consecutive_timeouts += 1
                        
                except Exception:
                    consecutive_timeouts += 1
                        
        except Exception as e:
            logger.error(f"Read error from endpoint {endpoint:#x}: {e}")
            
        return data

    def _reconstruct_frame(self, even_data: bytes, odd_data: bytes, width: int, height: int) -> Optional[np.ndarray]:
        """
        Reconstruct full frame from even/odd row data
        
        Protocol: EP2 has rows 0,2,4... EP6 has rows 1,3,5...
        Each endpoint contains PixelData[RowNumber/2][ColumnNumber]
        
        Args:
            even_data: Data from EP2 (even rows)
            odd_data: Data from EP6 (odd rows)
            width: Frame width
            height: Frame height
            
        Returns:
            Optional[np.ndarray]: Reconstructed frame or None if failed
        """
        try:
            rows_per_endpoint = height // 2
            pixels_per_row = width
            
            # Use the minimum available data
            min_size = min(len(even_data), len(odd_data))
            available_rows_per_endpoint = min_size // pixels_per_row
            
            if available_rows_per_endpoint == 0:
                logger.error("Not enough data for frame reconstruction")
                return None
            
            # Extract the data we can use
            even_bytes_to_use = available_rows_per_endpoint * pixels_per_row
            odd_bytes_to_use = available_rows_per_endpoint * pixels_per_row
            
            # Convert to numpy arrays
            even_pixels = np.frombuffer(even_data[:even_bytes_to_use], dtype=np.uint8)
            odd_pixels = np.frombuffer(odd_data[:odd_bytes_to_use], dtype=np.uint8)
            
            # Reshape into rows
            even_rows = even_pixels.reshape(available_rows_per_endpoint, pixels_per_row)
            odd_rows = odd_pixels.reshape(available_rows_per_endpoint, pixels_per_row)
            
            # Interleave even and odd rows
            total_rows = available_rows_per_endpoint * 2
            full_frame = np.zeros((total_rows, pixels_per_row), dtype=np.uint8)
            full_frame[0::2] = even_rows  # Even indices: 0, 2, 4...
            full_frame[1::2] = odd_rows   # Odd indices: 1, 3, 5...
            
            return full_frame
            
        except Exception as e:
            logger.error(f"Frame reconstruction failed: {e}")
            return None
            
    def initialize_camera(self):
        """
        Initialize camera with recommended settings
        
        This sets up the camera with standard settings suitable for most applications.
        Call this after connect() for basic operation.
        """
        logger.info("Initializing camera with recommended settings...")
        
        # Set work mode to normal
        self.set_mode(CameraMode.NORMAL)
        time.sleep(0.1)
        
        # Set normal sensor clock (good balance of speed and quality)
        self.set_sensor_clock(SensorClock.NORMAL)
        
        # Set short HBlanking for maximum frame rate
        self.set_hblanking(HBlanking.SHORT)
        
        # Set native resolution
        if 'BG04' in self.device_info.module_no or 'CG04' in self.device_info.module_no:
            self.set_resolution(752, 480, binning=False)
        elif 'B013' in self.device_info.module_no or 'C013' in self.device_info.module_no:
            self.set_resolution(1280, 1024, binning=False)
        elif 'C030' in self.device_info.module_no:
            self.set_resolution(2048, 1536, binning=False)
        else:
            self.set_resolution(640, 480, binning=False)
        
        # Set reasonable exposure time
        self.set_exposure_time(50.0)
        
        # Set moderate gains
        self.set_gains(16, 16, 16)
        
        # Set ROI to full frame
        self.set_xy_start(0, 0)
        
        # Wait for camera to stabilize
        time.sleep(0.5)
        
        logger.info("Camera initialization complete")
        
    @staticmethod
    def list_devices() -> List[str]:
        """
        List all connected Mightex cameras
        
        Returns:
            List[str]: List of camera descriptions
        """
        devices = list(usb.core.find(
            find_all=True,
            idVendor=MightexCamera.VENDOR_ID,
            idProduct=MightexCamera.PRODUCT_ID
        ))
        
        device_list = []
        for i, dev in enumerate(devices):
            try:
                device_list.append(f"{i}: Mightex Camera (Bus {dev.bus}, Address {dev.address})")
            except:
                device_list.append(f"{i}: Mightex Camera (Unknown)")
                
        return device_list


# # Example usage with different camera settings and frame averaging
# if __name__ == "__main__":
#     import cv2
    
#     # List available cameras
#     print("Available cameras:")
#     devices = MightexCamera.list_devices()
#     for dev in devices:
#         print(f"  {dev}")
    
#     if not devices:
#         print("No cameras found!")
#         exit(1)
    
#     # Connect to first camera
#     camera = MightexCamera(0)
    
#     if not camera.connect():
#         print("Failed to connect to camera")
#         exit(1)
    
#     try:
#         print(f"Connected to: {camera.device_info}")
#         print(f"Firmware version: {camera.get_firmware_version()}")
        
#         # =================================================================
#         # CAMERA SETTINGS - MODIFY THESE VALUES FOR DIFFERENT USE CASES
#         # =================================================================
        
#         # Choose your use case by uncommenting ONE of the following sections:
        
#         ## --- USE CASE 1: STANDARD PHOTOGRAPHY ---
#         ## Good balance of quality and speed for general use
#         # exposure_time_ms = 1.00        # 10ms exposure (moderate)
#         # red_gain = 8                  # Red channel gain (BG04: 8-32, Others: 1-64)
#         # green_gain = 8                # Green channel gain (BG04: 8-32, Others: 1-64) 
#         # blue_gain = 8                 # Blue channel gain (BG04: 8-32, Others: 1-64)
#         # width = 752                    # Image width in pixels
#         # height = 480                   # Image height in pixels
#         # use_binning = False            # False=full resolution, True=2x2 binning
#         # sensor_speed = SensorClock.NORMAL  # SLOW/NORMAL/FAST
#         # frame_rate_mode = HBlanking.SHORT  # SHORT/LONG/LONGEST
#         # use_frame_averaging = False    # Disable frame averaging for single shots
#         # num_avg_frames = 1             # Not used when averaging disabled
#         # averaging_mode = "average"     # Not used when averaging disabled
#         # filename = "standard_photo.png"
        
#         ## --- USE CASE 2: LOW LIGHT PHOTOGRAPHY WITH NOISE REDUCTION ---
#         ## Higher gains and frame averaging for noise reduction in dim conditions
#         # exposure_time_ms = 100.0       # 100ms exposure (longer for more light)
#         # red_gain = 28                  # Higher gain for low light (BG04: 8-32 range)
#         # green_gain = 28                # Higher gain for low light  
#         # blue_gain = 28                 # Higher gain for low light
#         # width = 752                    # Full resolution
#         # height = 480                   
#         # use_binning = True             # Binning increases sensitivity
#         # sensor_speed = SensorClock.NORMAL  # Normal speed for better quality
#         # frame_rate_mode = HBlanking.LONG   # Longer blanking for stability
#         # use_frame_averaging = True     # Enable frame averaging for noise reduction
#         # num_avg_frames = 8             # Average 8 frames to reduce noise by ~2.8x
#         # averaging_mode = "average"     # Use averaging mode for noise reduction
#         # filename = "low_light_averaged.png"
        
#         ## --- USE CASE 3: VERY LONG EXPOSURE (>750ms) ---
#         ## Use frame summing to simulate exposures longer than hardware limit
#         exposure_time_ms = 700       # Maximum hardware exposure (750ms)
#         red_gain = 16                  # Moderate gain (BG04: 8-32 range)
#         green_gain = 16                # Moderate gain
#         blue_gain = 16                 # Moderate gain
#         width = 752                    
#         height = 480                   
#         num_avg_frames = 1             # 8 × 750ms = 6.0 second effective exposure
#         averaging_mode = "sum"         # Use "sum" for long exposure simulation
#         filename = "test.png"

#         # Advanced
#         use_binning = True             # Binning for higher sensitivity
#         use_frame_averaging = True     # Enable frame averaging
#         sensor_speed = SensorClock.SLOW    # Slow for best quality
#         frame_rate_mode = HBlanking.LONGEST # Longest blanking for quality
        
#         ## --- USE CASE 4: ASTROPHOTOGRAPHY ---
#         ## Maximum exposure with summing for deep space imaging
#         # exposure_time_ms = 750.0       # Maximum hardware exposure
#         # red_gain = 32                  # Maximum gain for faint objects (BG04 max)
#         # green_gain = 32                # Maximum gain
#         # blue_gain = 32                 # Maximum gain
#         # width = 752                    
#         # height = 480                   
#         # use_binning = True             # 2x2 binning for maximum sensitivity
#         # sensor_speed = SensorClock.SLOW    # Slowest for best SNR
#         # frame_rate_mode = HBlanking.LONGEST # Longest blanking for stability
#         # use_frame_averaging = True     # Essential for astrophotography
#         # num_avg_frames = 20            # 20 × 750ms = 15 second exposure
#         # averaging_mode = "sum"         # Sum for long exposure effect
#         # filename = "astrophoto_15s.png"
        
#         # =================================================================
#         # APPLY THE SETTINGS TO THE CAMERA
#         # =================================================================
        
#         print(f"\nApplying camera settings:")
#         print(f"  Resolution: {width}x{height} (binning: {use_binning})")
#         print(f"  Exposure: {exposure_time_ms}ms")
#         print(f"  Gains: R={red_gain}, G={green_gain}, B={blue_gain}")
#         print(f"  Sensor speed: {sensor_speed.name}")
#         print(f"  Frame rate mode: {frame_rate_mode.name}")
        
#         if use_frame_averaging:
#             if averaging_mode == "sum":
#                 effective_exp = exposure_time_ms * num_avg_frames
#                 print(f"  Frame summing: {num_avg_frames} frames (simulated long exposure)")
#                 print(f"  Effective exposure time: ~{effective_exp}ms ({effective_exp/1000:.1f}s)")
#             else:
#                 noise_reduction = num_avg_frames ** 0.5
#                 print(f"  Frame averaging: {num_avg_frames} frames (noise reduction)")
#                 print(f"  Estimated noise reduction: ~{noise_reduction:.1f}x improvement")
#         else:
#             print(f"  Single frame capture")
        
#         # Set camera mode
#         camera.set_mode(CameraMode.NORMAL)
        
#         # Set sensor performance
#         camera.set_sensor_clock(sensor_speed)
#         camera.set_hblanking(frame_rate_mode)
        
#         # Set image properties
#         camera.set_resolution(width, height, use_binning)
#         camera.set_exposure_time(exposure_time_ms)
#         camera.set_gains(red_gain, green_gain, blue_gain)
        
#         # Set ROI to full frame (starting at top-left corner)
#         camera.set_xy_start(0, 0)
        
#         # Wait for camera to settle
#         time.sleep(0.5)
        
#         # =================================================================
#         # CAPTURE AND SAVE THE IMAGE
#         # =================================================================
        
#         if use_frame_averaging:
#             if averaging_mode == "sum":
#                 print(f"\nCapturing {num_avg_frames} frames for long exposure simulation...")
#                 print(f"This will take approximately {(exposure_time_ms * num_avg_frames + 200 * num_avg_frames) / 1000:.1f} seconds...")
#             else:
#                 print(f"\nCapturing {num_avg_frames} frames for noise reduction averaging...")
#                 print(f"This will take approximately {(exposure_time_ms * num_avg_frames + 200 * num_avg_frames) / 1000:.1f} seconds...")
            
#             # Capture multiple frames with appropriate delay
#             frame_delay = 0.2  # 200ms delay between frames
#             frame = camera.capture_multiple_frames(num_avg_frames, averaging_mode, frame_delay)
#         else:
#             print(f"\nCapturing single image with current settings...")
#             frame = camera.capture_frame()
        
#         if frame is not None:
#             print(f"SUCCESS! Captured frame: {frame.shape}")
#             print(f"Frame data range: {frame.min()} - {frame.max()}")
            
#             # Save the image
#             cv2.imwrite(filename, frame)
#             print(f"Image saved as: {filename}")
            
#             # Display statistics
#             if not use_frame_averaging:
#                 # Single frame statistics
#                 frame_prop = camera.get_frame_property()
#                 print(f"\nFrame statistics:")
#                 print(f"  Actual exposure time: {frame_prop.exposure_time * 0.05:.1f}ms")
#                 print(f"  Actual gains: R={frame_prop.red_gain}, G={frame_prop.green_gain}, B={frame_prop.blue_gain}")
#                 print(f"  Frame timestamp: {frame_prop.timestamp}ms")
#                 if frame_prop.frame_invalid:
#                     print("  WARNING: Frame marked as invalid by camera")
#             else:
#                 # Multi-frame statistics
#                 print(f"\nMulti-frame capture statistics:")
#                 if averaging_mode == "sum":
#                     effective_exposure = exposure_time_ms * num_avg_frames
#                     print(f"  Effective exposure time: {effective_exposure:.1f}ms ({effective_exposure/1000:.2f}s)")
#                     print(f"  Light gathering improvement: ~{num_avg_frames}x")
#                 else:
#                     noise_reduction = num_avg_frames ** 0.5
#                     print(f"  Noise reduction factor: ~{noise_reduction:.1f}x")
#                     print(f"  Signal-to-noise ratio improvement: ~{noise_reduction:.1f}x")
                
#                 print(f"  Total capture time: ~{(exposure_time_ms * num_avg_frames) / 1000:.1f}s")
            
#             # Image analysis
#             mean_brightness = frame.mean()
#             std_brightness = frame.std()
#             print(f"\nImage analysis:")
#             print(f"  Mean brightness: {mean_brightness:.1f}")
#             print(f"  Standard deviation: {std_brightness:.1f}")
#             print(f"  Dynamic range utilization: {(frame.max() / 255.0) * 100:.1f}%")
            
#         else:
#             print("Failed to capture frame")
#             print("Try adjusting the settings or check camera connection")
#             print("\nTroubleshooting tips:")
#             print("- Ensure camera is properly connected")
#             print("- Check USB 2.0 connection")
#             print("- Verify camera is not in use by another application")
#             print("- Try reducing resolution or frame count")
        
#     except Exception as e:
#         print(f"Error: {e}")
#         print("Check that the camera is properly connected and settings are valid")
#         import traceback
#         traceback.print_exc()
#     finally:
#         print("\nDisconnecting camera...")
#         camera.disconnect()
#         print("Done!")