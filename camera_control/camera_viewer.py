"""
SpectrumBoi Camera Viewer
Clean, simple viewer with modern UI and mouse-interactive sliders
"""

import cv2
import sys
import os
from threading import Lock

# Add parent directory to path (for running as standalone script)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from .camera_streaming import MightexStreamingCamera
from .mightex_driver.camera import SensorClock, HBlanking
from .camera_ui import ModernUI, create_modern_ui


class CameraViewer:
    """
    Camera viewer with modern UI
    """
    
    def __init__(self, sidebar_width: int = 300):
        """
        Initialize viewer
        
        Args:
            sidebar_width: Width of sidebar
        """
        # Camera setup
        self.stream = None
        self.window_name = "SpectrumBoi Camera"
        
        # UI
        self.ui = create_modern_ui(sidebar_width)
        self.sidebar_width = sidebar_width
        
        # Camera parameters
        self.width = 752
        self.height = 480
        self.exposure_ms = 10.0
        self.gain = 16
        self.sensor_speed = SensorClock.NORMAL
        self.frame_rate_mode = HBlanking.SHORT
        
        # State
        self.control_lock = Lock()
        self.frame_count = 0
        self.screenshot_count = 0
        
    def update_exposure(self, value):
        """Update camera exposure"""
        with self.control_lock:
            self.exposure_ms = max(1.0, min(750.0, float(value)))
            if self.stream:
                self.stream.camera.set_exposure_time(self.exposure_ms)
    
    def update_gain(self, value):
        """Update camera gain"""
        with self.control_lock:
            self.gain = max(1, min(64, int(value)))
            if self.stream:
                self.stream.camera.set_gains(self.gain, self.gain, self.gain)
    
    def is_window_open(self):
        """Check if window is still open"""
        try:
            return cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) >= 1
        except:
            return False
    
    def connect_camera(self):
        """Connect and configure camera"""
        self.stream = MightexStreamingCamera(buffer_size=3)
        
        if not self.stream.connect():
            print("ERROR: Failed to connect to camera")
            return False
        
        self.stream.configure(
            width=self.width,
            height=self.height,
            exposure_ms=self.exposure_ms,
            red_gain=self.gain,
            green_gain=self.gain,
            blue_gain=self.gain,
            sensor_speed=self.sensor_speed,
            frame_rate_mode=self.frame_rate_mode
        )
        
        return True
    
    def run(self):
        """Main viewer loop"""
        
        # Connect camera
        if not self.connect_camera():
            print("Camera connection failed")
            return False
        
        # Start capture
        self.stream.start_capture()
        
        # Create window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        # Calculate initial window size
        window_width = self.width + self.sidebar_width
        cv2.resizeWindow(self.window_name, window_width, self.height)
        
        # Set mouse callback for interactive sliders
        cv2.setMouseCallback(self.window_name, self.ui.handle_mouse)
        
        print(f"\n{'='*60}")
        print("SPECTRUMBOI CAMERA")
        print(f"{'='*60}\n")
        print("Camera connected! Controls are shown in the sidebar.")
        print(f"\n{'='*60}\n")
        
        # Keep last frame for display even when no new frame arrives
        last_frame = None
        ui_refresh_rate = 30  # Hz - UI updates at 30 FPS regardless of camera rate
        
        try:
            while True:
                # Check if window closed
                if not self.is_window_open():
                    print("\nWindow closed")
                    break
                
                # Try to get latest frame (non-blocking)
                new_frame = self.stream.read_latest()
                if new_frame is not None:
                    self.frame_count += 1
                    
                    # Convert grayscale to BGR if needed (monochrome camera)
                    if len(new_frame.shape) == 2:
                        new_frame = cv2.cvtColor(new_frame, cv2.COLOR_GRAY2BGR)
                    
                    last_frame = new_frame
                
                # Always update UI even if no new frame
                if last_frame is not None:
                    # Get current window size
                    try:
                        window_rect = cv2.getWindowImageRect(self.window_name)
                        window_width = max(window_rect[2], self.width + self.sidebar_width)
                        window_height = max(window_rect[3], self.height)
                    except:
                        window_width = self.width + self.sidebar_width
                        window_height = self.height
                    
                    # Create display with sidebar
                    display_frame, sidebar_x = self.ui.create_frame_with_sidebar(
                        last_frame, window_width, window_height
                    )
                    
                    # Draw sidebar content
                    display_frame = self.ui.draw_sidebar_content(
                        display_frame,
                        sidebar_x,
                        fps=self.stream.get_fps(),
                        exposure_ms=self.exposure_ms,
                        gain=self.gain,
                        brightness=last_frame.mean(),
                        frame_count=self.frame_count,
                        exposure_callback=self.update_exposure,
                        gain_callback=self.update_gain
                    )
                    
                    # Show frame
                    cv2.imshow(self.window_name, display_frame)
                
                # Handle keyboard - wait time determines UI refresh rate
                key = cv2.waitKey(int(1000 / ui_refresh_rate)) & 0xFF
                
                if key == ord('q') or key == ord('Q') or key == 27:  # Q or ESC
                    print("\nQuitting...")
                    break
                
                elif key == ord('s') or key == ord('S'):
                    if last_frame is not None:
                        filename = f"screenshot_{self.screenshot_count:03d}.png"
                        cv2.imwrite(filename, last_frame)
                        print(f"✓ Saved {filename}")
                        self.screenshot_count += 1
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user")
        
        except Exception as e:
            print(f"\nError: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # Cleanup
            if self.stream:
                self.stream.stop_capture()
                self.stream.disconnect()
            cv2.destroyAllWindows()
            
            # Stats
            print(f"\nSession Statistics:")
            print(f"  Frames captured: {self.frame_count}")
            print(f"  Screenshots saved: {self.screenshot_count}")
            print(f"  Average FPS: {self.stream.get_fps():.1f}")
            print("\nDone!")
        
        return True


def main():
    """Entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='SpectrumBoi Camera Viewer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python camera_viewer.py                 # Default settings
  python camera_viewer.py --exposure 30   # Custom exposure
  python camera_viewer.py --sidebar 350   # Wider sidebar
        """
    )
    
    parser.add_argument('--sidebar', type=int, default=300,
                       help='Sidebar width in pixels (default: 300)')
    parser.add_argument('--exposure', type=float, default=10.0,
                       help='Initial exposure in ms (default: 10.0)')
    parser.add_argument('--gain', type=int, default=16,
                       help='Initial gain (default: 16)')
    
    args = parser.parse_args()
    
    # Create and run viewer
    viewer = CameraViewer(sidebar_width=args.sidebar)
    viewer.exposure_ms = args.exposure
    viewer.gain = args.gain
    
    viewer.run()


if __name__ == "__main__":
    main()
