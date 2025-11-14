"""
Streaming wrapper for Mightex S-Series USB Camera
Provides OpenCV-compatible interface and live streaming capabilities

This module extends the base MightexCamera class with:
- Continuous frame capture in background thread
- Frame buffering with configurable queue size
- OpenCV VideoCapture-like interface
- Frame rate control and monitoring
- Integration examples for OpenCV display and MJPEG streaming
"""

import threading
import queue
import time
import numpy as np
from typing import Optional, Tuple, Callable
from collections import deque
import logging

# Import from mightex_driver folder
from mightex_driver.camera import MightexCamera, CameraMode, SensorClock, HBlanking

logger = logging.getLogger(__name__)


class MightexStreamingCamera:
    """
    Streaming wrapper for Mightex camera with OpenCV compatibility
    
    This class provides continuous frame capture in a background thread
    with a buffer queue, making it suitable for live display, recording,
    or streaming applications.
    
    Usage:
        # Basic streaming
        stream = MightexStreamingCamera()
        stream.connect()
        stream.start_capture()
        
        while True:
            frame = stream.read()
            if frame is not None:
                cv2.imshow('Camera', frame)
                if cv2.waitKey(1) == ord('q'):
                    break
        
        stream.stop_capture()
        stream.disconnect()
        
        # Context manager style
        with MightexStreamingCamera() as stream:
            stream.start_capture()
            for frame in stream.frame_generator():
                # Process frame
                pass
    """
    
    def __init__(self, device_index: int = 0, buffer_size: int = 5):
        """
        Initialize streaming camera
        
        Args:
            device_index: Camera index if multiple cameras connected
            buffer_size: Number of frames to buffer (larger = more latency, smaller = more drops)
        """
        self.camera = MightexCamera(device_index)
        self.buffer_size = buffer_size
        self.frame_queue = queue.Queue(maxsize=buffer_size)
        
        # Streaming control
        self._capture_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._is_capturing = False
        
        # Frame rate tracking - rolling window for responsive FPS
        self._frame_count = 0
        self._start_time = None
        self._last_frame_time = None
        self._frame_timestamps = deque(maxlen=5)  # Last 5 frame timestamps
        
        # Frame callback (optional)
        self._frame_callback: Optional[Callable] = None
        
    def connect(self) -> bool:
        """Connect to the camera"""
        return self.camera.connect()
    
    def disconnect(self):
        """Disconnect from camera (stops capture if running)"""
        if self._is_capturing:
            self.stop_capture()
        self.camera.disconnect()
    
    def configure(self, 
                  width: int = 752, 
                  height: int = 480,
                  exposure_ms: float = 10.0,
                  red_gain: int = 16,
                  green_gain: int = 16,
                  blue_gain: int = 16,
                  use_binning: bool = False,
                  sensor_speed: SensorClock = SensorClock.NORMAL,
                  frame_rate_mode: HBlanking = HBlanking.SHORT) -> bool:
        """
        Configure camera parameters for streaming
        
        Args:
            width, height: Resolution
            exposure_ms: Exposure time in milliseconds
            red_gain, green_gain, blue_gain: Color channel gains
            use_binning: Enable 2x2 pixel binning
            sensor_speed: Sensor clock speed (SLOW/NORMAL/FAST)
            frame_rate_mode: Frame rate mode (SHORT/LONG/LONGEST)
        
        Returns:
            bool: True if configuration successful
        """
        try:
            self.camera.set_mode(CameraMode.NORMAL)
            self.camera.set_sensor_clock(sensor_speed)
            self.camera.set_hblanking(frame_rate_mode)
            self.camera.set_resolution(width, height, use_binning)
            self.camera.set_exposure_time(exposure_ms)
            self.camera.set_gains(red_gain, green_gain, blue_gain)
            self.camera.set_xy_start(0, 0)
            
            # Give camera time to settle
            time.sleep(0.3)
            
            logger.info(f"Camera configured: {width}x{height}, {exposure_ms}ms exposure")
            return True
        except Exception as e:
            logger.error(f"Configuration failed: {e}")
            return False
    
    def start_capture(self, frame_callback: Optional[Callable] = None):
        """
        Start continuous frame capture in background thread
        
        Args:
            frame_callback: Optional callback function(frame, frame_number, timestamp)
                          called for each captured frame
        """
        if self._is_capturing:
            logger.warning("Capture already running")
            return
        
        self._frame_callback = frame_callback
        self._stop_event.clear()
        self._is_capturing = True
        self._start_time = time.time()
        self._frame_count = 0
        
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        
        logger.info("Capture thread started")
    
    def stop_capture(self, timeout: float = 5.0):
        """
        Stop continuous frame capture
        
        Args:
            timeout: Maximum time to wait for thread to stop
        """
        if not self._is_capturing:
            return
        
        logger.info("Stopping capture...")
        self._stop_event.set()
        
        if self._capture_thread:
            self._capture_thread.join(timeout=timeout)
            
        self._is_capturing = False
        
        # Clear any remaining frames in queue
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        
        logger.info("Capture stopped")
    
    def _capture_loop(self):
        """Background thread that continuously captures frames"""
        while not self._stop_event.is_set():
            try:
                # Capture frame from camera
                frame = self.camera.capture_frame()
                
                if frame is not None:
                    current_time = time.time()
                    self._frame_count += 1
                    
                    # Add timestamp to rolling window for FPS calculation
                    self._frame_timestamps.append(current_time)
                    
                    # Call user callback if provided
                    if self._frame_callback:
                        try:
                            self._frame_callback(frame, self._frame_count, current_time)
                        except Exception as e:
                            logger.error(f"Frame callback error: {e}")
                    
                    # Add frame to queue (non-blocking)
                    try:
                        self.frame_queue.put(frame, block=False)
                    except queue.Full:
                        # Queue full - drop oldest frame and add new one
                        try:
                            self.frame_queue.get_nowait()
                            self.frame_queue.put(frame, block=False)
                        except queue.Empty:
                            pass
                    
                    self._last_frame_time = current_time
                else:
                    logger.warning("Frame capture returned None")
                    time.sleep(0.01)  # Brief pause on error
                    
            except Exception as e:
                logger.error(f"Capture loop error: {e}")
                time.sleep(0.1)  # Pause on error to avoid spinning
    
    def read(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """
        Read next available frame from buffer (OpenCV-compatible interface)
        
        Args:
            timeout: Maximum time to wait for frame
            
        Returns:
            Frame as numpy array or None if timeout/stopped
        """
        try:
            frame = self.frame_queue.get(timeout=timeout)
            return frame
        except queue.Empty:
            return None
    
    def read_latest(self) -> Optional[np.ndarray]:
        """
        Read the most recent frame, discarding any older buffered frames
        
        Returns:
            Latest frame or None if no frames available
        """
        latest_frame = None
        
        # Drain queue to get latest frame
        try:
            while True:
                latest_frame = self.frame_queue.get_nowait()
        except queue.Empty:
            pass
        
        return latest_frame
    
    def get_fps(self) -> float:
        """
        Calculate current frame rate from recent frames (rolling window)
        
        Returns:
            Frames per second (based on last ~5 frames)
        """
        if len(self._frame_timestamps) < 2:
            return 0.0
        
        # Calculate FPS from time between first and last timestamp in window
        time_span = self._frame_timestamps[-1] - self._frame_timestamps[0]
        num_intervals = len(self._frame_timestamps) - 1
        
        if time_span > 0:
            return num_intervals / time_span
        return 0.0
    
    def get_stats(self) -> dict:
        """
        Get capture statistics
        
        Returns:
            Dictionary with capture statistics
        """
        return {
            'frame_count': self._frame_count,
            'fps': self.get_fps(),
            'buffer_size': self.frame_queue.qsize(),
            'is_capturing': self._is_capturing,
            'elapsed_time': time.time() - self._start_time if self._start_time else 0
        }
    
    def frame_generator(self, timeout: float = 1.0):
        """
        Generator that yields frames continuously
        
        Args:
            timeout: Timeout for each frame read
            
        Yields:
            Frame as numpy array
        """
        while self._is_capturing:
            frame = self.read(timeout=timeout)
            if frame is not None:
                yield frame
            else:
                # Check if we've been stopped
                if not self._is_capturing:
                    break
    
    def is_capturing(self) -> bool:
        """Check if capture is currently running"""
        return self._is_capturing
    
    # Context manager support
    def __enter__(self):
        if not self.camera.device:
            self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


# ============================================================================
# INTEGRATION EXAMPLES
# ============================================================================

def example_opencv_display():
    """Example: Display camera feed using OpenCV"""
    import cv2
    
    print("OpenCV Display Example")
    print("Press 'q' to quit")
    
    stream = MightexStreamingCamera(buffer_size=3)
    
    if not stream.connect():
        print("Failed to connect to camera")
        return
    
    # Configure for good live preview
    stream.configure(
        width=752,
        height=480,
        exposure_ms=10.0,
        red_gain=16,
        green_gain=16,
        blue_gain=16,
        use_binning=False,
        sensor_speed=SensorClock.FAST,
        frame_rate_mode=HBlanking.SHORT
    )
    
    stream.start_capture()
    
    try:
        while True:
            frame = stream.read(timeout=1.0)
            
            if frame is not None:
                # Add FPS overlay
                fps = stream.get_fps()
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                cv2.imshow('Mightex Camera', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Timeout waiting for frame")
    
    finally:
        stream.stop_capture()
        stream.disconnect()
        cv2.destroyAllWindows()


def example_mjpeg_server():
    """Example: Stream camera feed as MJPEG over HTTP"""
    import cv2
    from http.server import BaseHTTPRequestHandler, HTTPServer
    import threading
    
    class MJPEGHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/stream.mjpg':
                self.send_response(200)
                self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                
                try:
                    while stream.is_capturing():
                        frame = stream.read(timeout=1.0)
                        if frame is not None:
                            # Encode frame as JPEG
                            _, jpeg = cv2.imencode('.jpg', frame, 
                                                  [cv2.IMWRITE_JPEG_QUALITY, 80])
                            
                            # Send MJPEG frame
                            self.wfile.write(b'--frame\r\n')
                            self.send_header('Content-type', 'image/jpeg')
                            self.send_header('Content-length', len(jpeg))
                            self.end_headers()
                            self.wfile.write(jpeg.tobytes())
                            self.wfile.write(b'\r\n')
                except:
                    pass
            else:
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                html = """
                <html>
                <head><title>Mightex Camera Stream</title></head>
                <body>
                <h1>Mightex Camera Stream</h1>
                <img src="/stream.mjpg" width="752" height="480"/>
                </body>
                </html>
                """
                self.wfile.write(html.encode())
    
    global stream
    stream = MightexStreamingCamera(buffer_size=5)
    
    if not stream.connect():
        print("Failed to connect to camera")
        return
    
    stream.configure(
        width=752,
        height=480,
        exposure_ms=15.0,
        sensor_speed=SensorClock.NORMAL,
        frame_rate_mode=HBlanking.SHORT
    )
    
    stream.start_capture()
    
    server = HTTPServer(('0.0.0.0', 8080), MJPEGHandler)
    print("MJPEG server running at http://localhost:8080")
    print("Press Ctrl+C to stop")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop_capture()
        stream.disconnect()
        server.shutdown()


def example_video_recording():
    """Example: Record camera feed to video file"""
    import cv2
    
    print("Video Recording Example")
    print("Recording 10 seconds of video...")
    
    stream = MightexStreamingCamera(buffer_size=10)
    
    if not stream.connect():
        print("Failed to connect to camera")
        return
    
    # Configure camera
    width, height = 752, 480
    stream.configure(
        width=width,
        height=height,
        exposure_ms=10.0,
        sensor_speed=SensorClock.NORMAL,
        frame_rate_mode=HBlanking.SHORT
    )
    
    # Create video writer
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (width, height))
    
    stream.start_capture()
    
    start_time = time.time()
    duration = 10.0  # Record for 10 seconds
    
    try:
        while time.time() - start_time < duration:
            frame = stream.read(timeout=1.0)
            
            if frame is not None:
                out.write(frame)
                
                # Display progress
                elapsed = time.time() - start_time
                print(f"\rRecording: {elapsed:.1f}/{duration:.1f}s", end='')
        
        print("\nRecording complete!")
        
    finally:
        out.release()
        stream.stop_capture()
        stream.disconnect()


def example_frame_callback():
    """Example: Process frames using callback function"""
    import cv2
    
    def process_frame(frame, frame_num, timestamp):
        """Custom frame processing callback"""
        # Calculate average brightness
        brightness = frame.mean()
        print(f"Frame {frame_num}: brightness={brightness:.1f}, time={timestamp:.2f}")
        
        # Could do more processing here:
        # - Save specific frames
        # - Detect motion
        # - Apply filters
        # - Trigger actions based on content
    
    print("Frame Callback Example")
    print("Capturing 100 frames with custom processing...")
    
    with MightexStreamingCamera(buffer_size=5) as stream:
        stream.configure(
            width=752,
            height=480,
            exposure_ms=10.0
        )
        
        # Start capture with callback
        stream.start_capture(frame_callback=process_frame)
        
        # Let it run for a bit
        time.sleep(5)
        
        stats = stream.get_stats()
        print(f"\nCapture stats: {stats}")


if __name__ == "__main__":
    # Run the OpenCV display example by default
    example_opencv_display()
    
    # Uncomment to try other examples:
    # example_mjpeg_server()
    # example_video_recording()
    # example_frame_callback()
