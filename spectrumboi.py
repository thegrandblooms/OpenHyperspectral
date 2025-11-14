"""
SpectrumBoi Spectral Viewer - ImGui Edition
Professional UI with Dear ImGui for proper controls, scaling, and organization

CAMERA EXPOSURE NOTE:
The camera exposure is hardware-controlled via set_exposure_time() and runs in a 
separate capture thread. UI processing delays should NOT affect actual exposure timing.
However, if exposure appears to change during heavy UI load:
  1. Check if frames are being dropped (FPS counter)
  2. Verify the hardware exposure timer is truly independent
  3. Consider if camera auto-exposure is enabled accidentally
  4. Monitor if USB bandwidth/timing is being affected
"""

# Debug mode - enables testing features like default calibration points
DEBUG = True

import cv2
import numpy as np
import sys
import os
import time
from typing import Optional, List, Tuple
from dataclasses import dataclass
import OpenGL.GL as gl
import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import seaborn as sns
from io import BytesIO

# Set seaborn style
sns.set_theme(style="darkgrid")
plt.style.use('dark_background')

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera_streaming import MightexStreamingCamera
from camera import SensorClock, HBlanking


@dataclass
class CalibrationPoint:
    """A single wavelength calibration point"""
    pixel_x: int
    wavelength_nm: float
    label: str = ""
    
    def __post_init__(self):
        if not self.label:
            self.label = f"{self.wavelength_nm:.1f}nm"


@dataclass
class ScanLine:
    """A Y-coordinate scan line for multi-line spectroscopy"""
    pixel_y: int
    label: str = ""
    
    def __post_init__(self):
        if not self.label:
            self.label = f"Line {self.pixel_y}"


class WavelengthCalibrator:
    """Manages wavelength calibration with polynomial fitting"""
    
    def __init__(self):
        self.calibration_points: List[CalibrationPoint] = []
        self.polynomial_coeffs = None
        self.calibrated = False
        self.cached_scale_data = None  # Cache for wavelength scale overlay
        
    def add_point(self, pixel_x: int, wavelength_nm: float, label: str = ""):
        """Add a calibration point"""
        point = CalibrationPoint(pixel_x, wavelength_nm, label)
        self.calibration_points.append(point)
        self.calibrated = False  # Need to refit
        self.cached_scale_data = None  # Invalidate cache
        
    def remove_point(self, index: int):
        """Remove calibration point by index"""
        if 0 <= index < len(self.calibration_points):
            self.calibration_points.pop(index)
            self.calibrated = False
            self.cached_scale_data = None  # Invalidate cache
            
    def clear_points(self):
        """Clear all calibration points"""
        self.calibration_points.clear()
        self.polynomial_coeffs = None
        self.calibrated = False
        self.cached_scale_data = None  # Invalidate cache
        
    def fit_calibration(self, order: int = None) -> Tuple[bool, str]:
        """
        Fit polynomial to calibration points using exact interpolation.
        Calibration points are ground truth - the polynomial MUST pass through all of them.
        
        Returns:
            (success, message)
        """
        if len(self.calibration_points) < 2:
            return False, "Need at least 2 points"
        
        pixels = np.array([p.pixel_x for p in self.calibration_points])
        wavelengths = np.array([p.wavelength_nm for p in self.calibration_points])
        
        # Use polynomial order = n-1 for n points to guarantee exact fit through all points
        # This is interpolation, not regression
        poly_order = len(self.calibration_points) - 1
        
        self.polynomial_coeffs = np.polyfit(pixels, wavelengths, poly_order)
        self.calibrated = True
        
        # Verify exact fit (should be near-zero error for all points)
        predicted = np.polyval(self.polynomial_coeffs, pixels)
        max_error = np.max(np.abs(wavelengths - predicted))
        
        return True, f"Calibrated ({len(self.calibration_points)} points, max error: {max_error:.4f} nm)"
        
    def pixel_to_wavelength(self, pixel_x: int) -> float:
        """Convert pixel position to wavelength"""
        if not self.calibrated or self.polynomial_coeffs is None:
            return float(pixel_x)
        return float(np.polyval(self.polynomial_coeffs, pixel_x))
    
    def compute_scale_overlay(self, camera_width: int):
        """
        Pre-compute wavelength scale tick positions and labels for fast rendering.
        Called once after calibration is fitted.
        """
        if not self.calibrated:
            self.cached_scale_data = None
            return
        
        # Determine wavelength range
        wl_start = self.pixel_to_wavelength(0)
        wl_end = self.pixel_to_wavelength(camera_width - 1)
        wl_range = abs(wl_end - wl_start)
        
        # Nice tick increments based on range
        if wl_range < 50:
            tick_spacing_nm = 5
        elif wl_range < 100:
            tick_spacing_nm = 10
        elif wl_range < 200:
            tick_spacing_nm = 20
        elif wl_range < 500:
            tick_spacing_nm = 50
        else:
            tick_spacing_nm = 100
        
        # Pre-compute all pixel-to-wavelength mappings (vectorized, fast!)
        all_pixels = np.arange(camera_width)
        all_wavelengths = np.polyval(self.polynomial_coeffs, all_pixels)
        
        # Find tick positions
        first_tick_wl = int(min(wl_start, wl_end) / tick_spacing_nm) * tick_spacing_nm
        max_wl = max(wl_start, wl_end)
        
        ticks = []
        current_wl = first_tick_wl
        while current_wl <= max_wl:
            # Find closest pixel to this wavelength
            idx = np.argmin(np.abs(all_wavelengths - current_wl))
            if 0 <= idx < camera_width:
                ticks.append((int(idx), int(current_wl)))
            current_wl += tick_spacing_nm
        
        self.cached_scale_data = {
            'ticks': ticks,
            'camera_width': camera_width
        }


class ROISelector:
    """ROI selection - only active when mouse is over camera view"""
    
    def __init__(self):
        self.roi = None  # (x, y, width, height)
        self.selecting = False
        self.start_point = None
        self.current_point = None
        
    def start_selection(self, x: int, y: int):
        """Begin ROI selection"""
        self.selecting = True
        self.start_point = (x, y)
        self.current_point = (x, y)
        
    def update_selection(self, x: int, y: int):
        """Update selection"""
        if self.selecting:
            self.current_point = (x, y)
            
    def finish_selection(self):
        """Finalize ROI"""
        if not self.selecting or not self.start_point or not self.current_point:
            return
            
        self.selecting = False
        x1, y1 = self.start_point
        x2, y2 = self.current_point
        
        x = min(x1, x2)
        y = min(y1, y2)
        w = abs(x2 - x1)
        h = abs(y2 - y1)
        
        if w >= 10 and h >= 5:
            self.roi = (x, y, w, h)
    
    def draw_overlay(self, frame: np.ndarray, calibrator: WavelengthCalibrator, show_roi: bool = True) -> np.ndarray:
        """Draw ROI on frame"""
        overlay = frame.copy()
        
        if not show_roi:
            return overlay
        
        # Grey color for B&W camera display
        roi_color = (180, 180, 180)  # Medium-light grey for visibility
        
        # Current selection
        if self.selecting and self.start_point and self.current_point:
            cv2.rectangle(overlay, self.start_point, self.current_point, roi_color, 1)
        
        # Finalized ROI
        if self.roi:
            x, y, w, h = self.roi
            cv2.rectangle(overlay, (x, y), (x + w, y + h), roi_color, 1)
            
            # Label with wavelength range if calibrated
            if calibrator.calibrated:
                wl_start = calibrator.pixel_to_wavelength(x)
                wl_end = calibrator.pixel_to_wavelength(x + w)
                label = f"ROI: {wl_start:.1f}-{wl_end:.1f} nm"
            else:
                label = f"ROI: {w}x{h} px"
            
            cv2.putText(overlay, label, (x, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, roi_color, 1)
        
        return overlay
    
    def reset(self):
        """Clear ROI"""
        self.roi = None
        self.selecting = False
        self.start_point = None
        self.current_point = None


class SpectralViewerImGui:
    """Main application with Dear ImGui UI"""
    
    def __init__(self, width=1280, height=720):
        self.width = width
        self.height = height
        self.window = None
        self.impl = None
        
        # Camera
        self.stream = None
        self.camera_width = 752
        self.camera_height = 480
        self.exposure_ms = 10.0
        self.gain = 16
        self.frame_count = 0
        self.paused = False
        
        # Components
        self.calibrator = WavelengthCalibrator()
        self.roi_selector = ROISelector()
        
        # Debug mode: Add default calibration points for testing
        if DEBUG:
            # Mercury lamp lines for testing (common spectral calibration)
            self.calibrator.add_point(150, 435.8, "Hg Blue")
            self.calibrator.add_point(376, 546.1, "Hg Green")
            self.calibrator.add_point(600, 632.8, "He-Ne Red")
            # Auto-fit calibration (uses exact interpolation)
            self.calibrator.fit_calibration()
            # Pre-compute scale overlay
            self.calibrator.compute_scale_overlay(self.camera_width)
        
        # UI state
        self.new_cal_label = ""
        self.new_cal_pixel = 0
        self.new_cal_wavelength = 400.0
        self.calibration_message = ""
        
        # Visibility toggles
        self.show_calibration_lines = True
        self.show_roi = True
        self.show_nm_scale = True
        self.show_scan_lines = True
        
        # Spectrometer data
        self.spectrum_mode = "ROI Average"  # or "Scan Lines"
        self.captured_spectrum = None  # (wavelengths, intensities) or list of (wavelengths, intensities, label)
        self.spectrum_label = "Spectrum"
        self.spectrum_plot_texture = None  # OpenGL texture for plot
        self.spectrum_plot_needs_update = False  # Flag to regenerate plot
        
        # Scan lines for multi-line spectroscopy
        self.scan_lines: List[ScanLine] = []
        self.new_scan_y = 240  # Default to middle
        self.new_scan_label = ""
        
        # Frame averaging for scan lines
        self.frame_averaging_enabled = False
        self.num_frames_to_average = 3
        
        # OpenGL texture for camera frame
        self.texture_id = None
        
        # Camera view position (for mouse handling)
        self.camera_view_pos = (0, 0)
        self.camera_view_size = (0, 0)
        self.mouse_over_camera = False
        
    def init_glfw(self):
        """Initialize GLFW and ImGui"""
        if not glfw.init():
            return False
        
        # Create window
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)
        
        self.window = glfw.create_window(self.width, self.height, 
                                         "SpectrumBoi Spectral Viewer", None, None)
        if not self.window:
            glfw.terminate()
            return False
        
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)  # VSync
        
        # Initialize ImGui
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)
        
        # Set up mouse callbacks
        glfw.set_mouse_button_callback(self.window, self.mouse_button_callback)
        glfw.set_cursor_pos_callback(self.window, self.cursor_pos_callback)
        
        return True
    
    def create_texture(self, frame: np.ndarray) -> int:
        """Create or update OpenGL texture from frame"""
        if frame is None:
            return None
            
        # Convert to RGB if needed
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        elif frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
        else:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        if self.texture_id is None:
            # Create texture
            self.texture_id = gl.glGenTextures(1)
            gl.glBindTexture(gl.GL_TEXTURE_2D, self.texture_id)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
            gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
        
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.texture_id)
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB, frame.shape[1], frame.shape[0],
                        0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE, frame)
        
        return self.texture_id
    
    def mouse_button_callback(self, window, button, action, mods):
        """Handle mouse button events"""
        if button == glfw.MOUSE_BUTTON_LEFT and self.mouse_over_camera:
            x, y = glfw.get_cursor_pos(window)
            # Convert to camera coordinates
            cam_x = int((x - self.camera_view_pos[0]) / self.camera_view_size[0] * self.camera_width)
            cam_y = int((y - self.camera_view_pos[1]) / self.camera_view_size[1] * self.camera_height)
            
            if action == glfw.PRESS:
                self.roi_selector.start_selection(cam_x, cam_y)
            elif action == glfw.RELEASE:
                self.roi_selector.finish_selection()
    
    def cursor_pos_callback(self, window, x, y):
        """Handle mouse movement"""
        if self.roi_selector.selecting and self.mouse_over_camera:
            # Convert to camera coordinates
            cam_x = int((x - self.camera_view_pos[0]) / self.camera_view_size[0] * self.camera_width)
            cam_y = int((y - self.camera_view_pos[1]) / self.camera_view_size[1] * self.camera_height)
            self.roi_selector.update_selection(cam_x, cam_y)
    
    def connect_camera(self):
        """Connect to camera"""
        self.stream = MightexStreamingCamera(buffer_size=3)
        
        if not self.stream.connect():
            print("ERROR: Failed to connect to camera")
            return False
        
        self.stream.configure(
            width=self.camera_width,
            height=self.camera_height,
            exposure_ms=self.exposure_ms,
            red_gain=self.gain,
            green_gain=self.gain,
            blue_gain=self.gain,
            sensor_speed=SensorClock.NORMAL,
            frame_rate_mode=HBlanking.SHORT
        )
        
        self.stream.start_capture()
        return True
    
    def render_visibility_toggle(self, label_id: str, is_visible: bool) -> bool:
        """
        Render a visibility toggle button aligned to the right.
        Must be called immediately after collapsing_header with same_line().
        Returns new visibility state.
        """
        imgui.same_line()
        # Get remaining space and position button on far right
        avail = imgui.get_content_region_available()[0]
        imgui.set_cursor_pos_x(imgui.get_cursor_pos_x() + avail - 30)
        
        button_text = "[V]" if is_visible else "[  ]"
        if imgui.small_button(f"{button_text}##{label_id}"):
            is_visible = not is_visible
        
        return is_visible
    
    def capture_spectrum_from_roi(self, frame: np.ndarray) -> bool:
        """
        Capture spectrum by averaging ROI.
        Returns True if successful.
        """
        if not self.calibrator.calibrated:
            return False
        
        if not self.roi_selector.roi:
            return False
        
        x, y, w, h = self.roi_selector.roi
        
        # Extract ROI from frame
        roi_data = frame[y:y+h, x:x+w]
        
        # Average along Y axis to get 1D spectrum
        if len(roi_data.shape) == 3:
            # Color image - convert to grayscale first
            roi_data = cv2.cvtColor(roi_data, cv2.COLOR_BGR2GRAY)
        
        spectrum_intensities = np.mean(roi_data, axis=0)
        
        # Get wavelengths for each pixel in ROI
        pixel_positions = np.arange(x, x + w)
        wavelengths = np.array([self.calibrator.pixel_to_wavelength(px) 
                               for px in pixel_positions])
        
        self.captured_spectrum = (wavelengths, spectrum_intensities)
        self.spectrum_plot_needs_update = True  # Flag for plot regeneration
        return True
    
    def capture_spectrum_from_scan_lines(self, frame: np.ndarray) -> bool:
        """
        Capture spectra from multiple Y-coordinate scan lines.
        Supports optional frame averaging for better signal-to-noise.
        Returns True if successful.
        """
        if not self.calibrator.calibrated:
            return False
        
        if not self.scan_lines:
            return False
        
        # Determine if we need to average frames
        if self.frame_averaging_enabled and self.stream:
            print(f"Averaging {self.num_frames_to_average} frames (waiting for new captures)...")
            
            # Collect frames by waiting for new ones to arrive
            frames = []
            last_frame = None
            timeout = time.time() + (self.exposure_ms / 1000.0) * self.num_frames_to_average * 3  # 3x expected time
            
            while len(frames) < self.num_frames_to_average:
                # Check timeout
                if time.time() > timeout:
                    print(f"Timeout: Only captured {len(frames)}/{self.num_frames_to_average} frames")
                    break
                
                # Read latest frame
                f = self.stream.read_latest()
                if f is None:
                    time.sleep(0.001)
                    continue
                
                # Check if this is a new frame (compare to last collected)
                if last_frame is not None:
                    # Simple check: see if frames are identical
                    if np.array_equal(f, last_frame):
                        time.sleep(0.001)
                        continue
                
                # New frame! Convert and collect
                if len(f.shape) == 3:
                    f = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
                frames.append(f.astype(np.float32))
                last_frame = f
                print(f"  Collected frame {len(frames)}/{self.num_frames_to_average}")
            
            if not frames:
                print("ERROR: No frames captured during averaging")
                return False
            
            # Average frames
            frame_gray = np.mean(frames, axis=0).astype(np.uint8)
            print(f"✓ Averaged {len(frames)} frames")
        else:
            # Convert to grayscale if needed (single frame)
            if len(frame.shape) == 3:
                frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                frame_gray = frame
        
        # Capture spectrum from each scan line
        spectra = []
        for scan_line in self.scan_lines:
            y = scan_line.pixel_y
            if 0 <= y < self.camera_height:
                # Extract full horizontal line
                line_data = frame_gray[y, :]
                
                # Get wavelengths for each pixel
                pixel_positions = np.arange(self.camera_width)
                wavelengths = np.array([self.calibrator.pixel_to_wavelength(px) 
                                       for px in pixel_positions])
                
                spectra.append((wavelengths, line_data, scan_line.label))
        
        if spectra:
            self.captured_spectrum = spectra  # List of (wavelengths, intensities, label)
            self.spectrum_plot_needs_update = True
            return True
        
        return False
    
    def export_spectrum(self, filename: str = "spectrum.csv") -> bool:
        """Export captured spectrum to CSV file"""
        if self.captured_spectrum is None:
            return False
        
        try:
            with open(filename, 'w') as f:
                # Check if single spectrum or multiple
                if isinstance(self.captured_spectrum, tuple):
                    # Single spectrum (ROI Average mode)
                    wavelengths, intensities = self.captured_spectrum
                    f.write("Wavelength_nm,Intensity\n")
                    for wl, intensity in zip(wavelengths, intensities):
                        f.write(f"{wl:.2f},{intensity:.2f}\n")
                else:
                    # Multiple spectra (Scan Lines mode)
                    spectra = self.captured_spectrum
                    
                    # Write header
                    f.write("Wavelength_nm")
                    for _, _, label in spectra:
                        f.write(f",{label}")
                    f.write("\n")
                    
                    # Assume all have same wavelength axis (first one)
                    wavelengths = spectra[0][0]
                    
                    # Write data rows
                    for i, wl in enumerate(wavelengths):
                        f.write(f"{wl:.2f}")
                        for wl_arr, intensities, _ in spectra:
                            f.write(f",{intensities[i]:.2f}")
                        f.write("\n")
            
            return True
        except Exception as e:
            print(f"Export failed: {e}")
            return False
    
    def create_spectrum_plot_texture(self):
        """Create OpenGL texture from spectrum plot"""
        if self.captured_spectrum is None:
            return None
        
        # Create matplotlib figure with seaborn dark style
        fig, ax = plt.subplots(figsize=(6, 3), dpi=100, facecolor='#2b2b2b')
        ax.set_facecolor('#1e1e1e')
        
        # Check if single or multiple spectra
        if isinstance(self.captured_spectrum, tuple):
            # Single spectrum (ROI Average mode)
            wavelengths, intensities = self.captured_spectrum
            ax.plot(wavelengths, intensities, color='#5DA5DA', linewidth=2, label='ROI Average')
        else:
            # Multiple spectra (Scan Lines mode)
            spectra = self.captured_spectrum
            colors = ['#5DA5DA', '#FAA43A', '#60BD68', '#F17CB0', '#B2912F', '#B276B2', '#DECF3F', '#F15854']
            
            for i, (wavelengths, intensities, label) in enumerate(spectra):
                color = colors[i % len(colors)]
                ax.plot(wavelengths, intensities, color=color, linewidth=1.5, label=label, alpha=0.9)
            
            # Add legend for multiple lines
            ax.legend(fontsize=8, loc='best', facecolor='#2b2b2b', edgecolor='#555555', labelcolor='white')
        
        ax.set_xlabel('Wavelength (nm)', fontsize=10, color='white')
        ax.set_ylabel('Intensity', fontsize=10, color='white')
        ax.set_title('Spectrum', fontsize=11, color='white')
        ax.grid(True, alpha=0.2, color='#555555')
        
        # Set x limits based on first spectrum
        if isinstance(self.captured_spectrum, tuple):
            wavelengths = self.captured_spectrum[0]
        else:
            wavelengths = self.captured_spectrum[0][0]
        ax.set_xlim(wavelengths[0], wavelengths[-1])
        
        # Style the axes
        ax.tick_params(colors='white', labelsize=8)
        for spine in ax.spines.values():
            spine.set_color('#555555')
        
        # Tight layout
        fig.tight_layout()
        
        # Convert to image
        buf = BytesIO()
        fig.savefig(buf, format='png', dpi=100, facecolor='#2b2b2b')
        buf.seek(0)
        plt.close(fig)
        
        # Read image with OpenCV
        img_array = np.frombuffer(buf.read(), dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Create or update texture
        if self.spectrum_plot_texture is None:
            self.spectrum_plot_texture = gl.glGenTextures(1)
        
        gl.glBindTexture(gl.GL_TEXTURE_2D, self.spectrum_plot_texture)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB, img.shape[1], img.shape[0],
                        0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE, img)
        
        return self.spectrum_plot_texture
    
    def render_ui(self, frame):
        """Render ImGui interface"""
        imgui.new_frame()
        
        # Main menu bar
        if imgui.begin_main_menu_bar():
            if imgui.begin_menu("File"):
                clicked, _ = imgui.menu_item("Quit", "Esc")
                if clicked:
                    glfw.set_window_should_close(self.window, True)
                imgui.end_menu()
            
            if imgui.begin_menu("View"):
                _, self.show_roi = imgui.menu_item("Show ROI", None, self.show_roi)
                _, self.show_calibration_lines = imgui.menu_item("Show Calibration Lines", None, self.show_calibration_lines)
                _, self.show_nm_scale = imgui.menu_item("Show Wavelength Scale", None, self.show_nm_scale)
                _, self.show_scan_lines = imgui.menu_item("Show Scan Lines", None, self.show_scan_lines)
                imgui.end_menu()
            
            imgui.end_main_menu_bar()
        
        # Sidebar on right
        sidebar_width = 350
        win_width, win_height = glfw.get_window_size(self.window)
        imgui.set_next_window_position(win_width - sidebar_width, 20)
        imgui.set_next_window_size(sidebar_width, win_height - 20)
        imgui.begin("Controls", flags=imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_COLLAPSE)
        
        # === Camera Controls Section ===
        if imgui.collapsing_header("Camera Controls")[0]:
            # Exposure
            changed, new_exposure = imgui.slider_float("Exposure (ms)", self.exposure_ms, 1.0, 750.0)
            if changed and self.stream:
                self.exposure_ms = new_exposure
                self.stream.camera.set_exposure_time(self.exposure_ms)
            
            # Gain
            changed, new_gain = imgui.slider_int("Gain", self.gain, 1, 64)
            if changed and self.stream:
                self.gain = new_gain
                self.stream.camera.set_gains(self.gain, self.gain, self.gain)
            
            # Pause button
            if imgui.button("Pause" if not self.paused else "Resume"):
                self.paused = not self.paused
            
            imgui.text(f"FPS: {self.stream.get_fps():.1f}" if self.stream else "FPS: --")
            imgui.text(f"Frames: {self.frame_count}")
        
        # === ROI Section ===
        if imgui.collapsing_header("Region of Interest")[0]:
            imgui.text_wrapped("Click and drag on camera view to select ROI")
            
            if self.roi_selector.roi:
                x, y, w, h = self.roi_selector.roi
                
                # Editable position
                imgui.text("Position:")
                changed_x, new_x = imgui.input_int("X##roi_x", x)
                changed_y, new_y = imgui.input_int("Y##roi_y", y)
                
                # Editable size
                imgui.text("Size:")
                changed_w, new_w = imgui.input_int("Width##roi_w", w)
                changed_h, new_h = imgui.input_int("Height##roi_h", h)
                
                # Apply changes if any
                if changed_x or changed_y or changed_w or changed_h:
                    # Clamp to valid ranges
                    new_x = max(0, min(new_x, self.camera_width - 1))
                    new_y = max(0, min(new_y, self.camera_height - 1))
                    new_w = max(10, min(new_w, self.camera_width - new_x))
                    new_h = max(5, min(new_h, self.camera_height - new_y))
                    self.roi_selector.roi = (new_x, new_y, new_w, new_h)
                
                # Display wavelength range if calibrated
                if self.calibrator.calibrated:
                    wl_start = self.calibrator.pixel_to_wavelength(x)
                    wl_end = self.calibrator.pixel_to_wavelength(x + w)
                    imgui.text(f"Range: {wl_start:.1f} - {wl_end:.1f} nm")
                
                if imgui.button("Clear ROI"):
                    self.roi_selector.reset()
            else:
                imgui.text_colored("No ROI selected", 0.5, 0.5, 0.5)
        
        # === Calibration Section ===
        if imgui.collapsing_header("Wavelength Calibration")[0]:
            # Status
            if self.calibrator.calibrated:
                imgui.text_colored(f"✓ Calibrated ({len(self.calibrator.calibration_points)} points)", 
                                  0.0, 1.0, 0.0)
            else:
                imgui.text_colored("Not calibrated", 0.8, 0.8, 0.0)
            
            imgui.separator()
            
            # Add new calibration point
            imgui.text("Add Calibration Point:")
            _, self.new_cal_label = imgui.input_text("Label", self.new_cal_label, 32)
            _, self.new_cal_pixel = imgui.input_int("Pixel X", self.new_cal_pixel)
            _, self.new_cal_wavelength = imgui.input_float("Wavelength (nm)", self.new_cal_wavelength)
            
            if imgui.button("Add Point"):
                if 0 <= self.new_cal_pixel < self.camera_width:
                    self.calibrator.add_point(self.new_cal_pixel, self.new_cal_wavelength, 
                                             self.new_cal_label)
            
            imgui.separator()
            
            # Calibration points table
            if self.calibrator.calibration_points:
                imgui.text("Calibration Points:")
                
                if imgui.begin_table("cal_points", 4, imgui.TABLE_BORDERS | imgui.TABLE_ROW_BACKGROUND):
                    # Headers
                    imgui.table_setup_column("Label")
                    imgui.table_setup_column("Pixel")
                    imgui.table_setup_column("λ (nm)")
                    imgui.table_setup_column("Action")
                    imgui.table_headers_row()
                    
                    # Rows
                    to_delete = None
                    for i, point in enumerate(self.calibrator.calibration_points):
                        imgui.table_next_row()
                        
                        imgui.table_next_column()
                        imgui.push_item_width(-1)
                        changed, new_label = imgui.input_text(f"##label{i}", point.label, 32)
                        if changed:
                            point.label = new_label
                        imgui.pop_item_width()
                        
                        imgui.table_next_column()
                        imgui.push_item_width(-1)
                        changed, new_pixel = imgui.input_int(f"##pixel{i}", point.pixel_x)
                        if changed:
                            point.pixel_x = new_pixel
                            self.calibrator.calibrated = False  # Need to refit
                        imgui.pop_item_width()
                        
                        imgui.table_next_column()
                        imgui.push_item_width(-1)
                        changed, new_wl = imgui.input_float(f"##wl{i}", point.wavelength_nm, format="%.1f")
                        if changed:
                            point.wavelength_nm = new_wl
                            self.calibrator.calibrated = False  # Need to refit
                        imgui.pop_item_width()
                        
                        imgui.table_next_column()
                        if imgui.small_button(f"Delete##{i}"):
                            to_delete = i
                    
                    imgui.end_table()
                    
                    if to_delete is not None:
                        self.calibrator.remove_point(to_delete)
                
                imgui.separator()
                
                # Fit button
                if imgui.button("Fit Calibration Curve"):
                    success, message = self.calibrator.fit_calibration()
                    if success:
                        # Pre-compute scale overlay for fast rendering
                        self.calibrator.compute_scale_overlay(self.camera_width)
                        self.calibration_message = message
                        imgui.open_popup("Calibration Success")
                    else:
                        self.calibration_message = message
                        imgui.open_popup("Calibration Failed")
                
                # Popups
                if imgui.begin_popup("Calibration Success"):
                    imgui.text(f"✓ {self.calibration_message}")
                    if imgui.button("OK"):
                        imgui.close_current_popup()
                    imgui.end_popup()
                
                if imgui.begin_popup("Calibration Failed"):
                    imgui.text(f"✗ {self.calibration_message}")
                    if imgui.button("OK"):
                        imgui.close_current_popup()
                    imgui.end_popup()
                
                imgui.same_line()
                if imgui.button("Clear All"):
                    self.calibrator.clear_points()
            else:
                imgui.text_colored("No calibration points", 0.5, 0.5, 0.5)
            
            # Common wavelengths reference (inside calibration section)
            imgui.separator()
            if imgui.tree_node("Common Wavelengths"):
                imgui.text("Mercury lamp:")
                imgui.text("  404.7 nm - Violet")
                imgui.text("  435.8 nm - Blue")
                imgui.text("  546.1 nm - Green")
                imgui.text("  577.0 nm - Yellow")
                imgui.text("He-Ne laser:")
                imgui.text("  632.8 nm - Red")
                imgui.tree_pop()
        
        # === Spectrometer Section ===
        if imgui.collapsing_header("Spectrometer")[0]:
            # Mode selection
            imgui.text("Mode:")
            if imgui.radio_button("ROI Average", self.spectrum_mode == "ROI Average"):
                self.spectrum_mode = "ROI Average"
            imgui.same_line()
            if imgui.radio_button("Scan Lines", self.spectrum_mode == "Scan Lines"):
                self.spectrum_mode = "Scan Lines"
            
            imgui.separator()
            
            # Mode-specific UI
            if self.spectrum_mode == "Scan Lines":
                imgui.text("Add Scan Line:")
                _, self.new_scan_label = imgui.input_text("Label##scan", self.new_scan_label, 32)
                _, self.new_scan_y = imgui.input_int("Y Position##scan", self.new_scan_y)
                
                if imgui.button("Add Scan Line"):
                    if 0 <= self.new_scan_y < self.camera_height:
                        scan_line = ScanLine(self.new_scan_y, self.new_scan_label)
                        self.scan_lines.append(scan_line)
                
                imgui.separator()
                
                # Scan lines table
                if self.scan_lines:
                    imgui.text("Scan Lines:")
                    
                    if imgui.begin_table("scan_lines", 3, imgui.TABLE_BORDERS | imgui.TABLE_ROW_BACKGROUND):
                        imgui.table_setup_column("Label")
                        imgui.table_setup_column("Y Pos")
                        imgui.table_setup_column("Action")
                        imgui.table_headers_row()
                        
                        to_delete = None
                        for i, scan_line in enumerate(self.scan_lines):
                            imgui.table_next_row()
                            
                            imgui.table_next_column()
                            imgui.push_item_width(-1)
                            changed, new_label = imgui.input_text(f"##scan_label{i}", scan_line.label, 32)
                            if changed:
                                scan_line.label = new_label
                            imgui.pop_item_width()
                            
                            imgui.table_next_column()
                            imgui.push_item_width(-1)
                            changed, new_y = imgui.input_int(f"##scan_y{i}", scan_line.pixel_y)
                            if changed:
                                scan_line.pixel_y = new_y
                            imgui.pop_item_width()
                            
                            imgui.table_next_column()
                            if imgui.small_button(f"Delete##{i}"):
                                to_delete = i
                        
                        imgui.end_table()
                        
                        if to_delete is not None:
                            self.scan_lines.pop(to_delete)
                    
                    imgui.separator()
                    
                    if imgui.button("Clear All Lines"):
                        self.scan_lines.clear()
                else:
                    imgui.text_colored("No scan lines defined", 0.5, 0.5, 0.5)
                
                imgui.separator()
                
                # Frame averaging option
                imgui.text("Frame Averaging:")
                _, self.frame_averaging_enabled = imgui.checkbox("Enable Averaging", self.frame_averaging_enabled)
                if self.frame_averaging_enabled:
                    imgui.same_line()
                    imgui.push_item_width(80)
                    _, self.num_frames_to_average = imgui.input_int("##frames", self.num_frames_to_average)
                    self.num_frames_to_average = max(2, min(30, self.num_frames_to_average))  # Clamp 2-30
                    imgui.pop_item_width()
                    imgui.same_line()
                    imgui.text("frames")
                
                imgui.separator()
            
            # Capture button (works for both modes)
            if imgui.button("Capture Spectrum"):
                if frame is not None:
                    success = False
                    if self.spectrum_mode == "ROI Average":
                        success = self.capture_spectrum_from_roi(frame)
                    else:  # Scan Lines
                        success = self.capture_spectrum_from_scan_lines(frame)
                    
                    if not success:
                        imgui.open_popup("Spectrum Capture Failed")
            
            # Capture popup (only for failures)
            if imgui.begin_popup("Spectrum Capture Failed"):
                if self.spectrum_mode == "ROI Average":
                    imgui.text("✗ Need calibration and ROI")
                else:
                    imgui.text("✗ Need calibration and scan lines")
                if imgui.button("OK"):
                    imgui.close_current_popup()
                imgui.end_popup()
            
            imgui.separator()
            
            # Display captured spectrum info
            if self.captured_spectrum is not None:
                # Handle both single and multiple spectra
                if isinstance(self.captured_spectrum, tuple):
                    # Single spectrum
                    wavelengths, intensities = self.captured_spectrum
                    imgui.text(f"Points: {len(wavelengths)}")
                    imgui.text(f"Range: {wavelengths[0]:.1f} - {wavelengths[-1]:.1f} nm")
                    imgui.text(f"Intensity: {intensities.min():.0f} - {intensities.max():.0f}")
                    imgui.text(f"Mean: {intensities.mean():.1f}")
                else:
                    # Multiple spectra
                    spectra = self.captured_spectrum
                    imgui.text(f"Lines captured: {len(spectra)}")
                    wavelengths = spectra[0][0]
                    imgui.text(f"Range: {wavelengths[0]:.1f} - {wavelengths[-1]:.1f} nm")
                
                imgui.separator()
                
                # Export button
                if imgui.button("Export to CSV"):
                    import datetime
                    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"spectrum_{timestamp}.csv"
                    if self.export_spectrum(filename):
                        print(f"Spectrum exported to {filename}")
                        imgui.open_popup("Export Success")
                    else:
                        imgui.open_popup("Export Failed")
                
                # Export popups
                if imgui.begin_popup("Export Success"):
                    imgui.text("✓ Spectrum exported")
                    if imgui.button("OK"):
                        imgui.close_current_popup()
                    imgui.end_popup()
                
                if imgui.begin_popup("Export Failed"):
                    imgui.text("✗ Export failed")
                    if imgui.button("OK"):
                        imgui.close_current_popup()
                    imgui.end_popup()
                
                imgui.separator()
                
                # Plot spectrum (cached for performance)
                imgui.text("Spectrum Plot:")
                
                # Only regenerate plot if needed
                if self.spectrum_plot_needs_update:
                    self.create_spectrum_plot_texture()
                    self.spectrum_plot_needs_update = False
                
                # Display cached plot
                if self.spectrum_plot_texture:
                    imgui.image(self.spectrum_plot_texture, 300, 150)
            else:
                imgui.text_colored("No spectrum captured", 0.5, 0.5, 0.5)
                if self.spectrum_mode == "ROI Average":
                    imgui.text_wrapped("Capture a spectrum from the selected ROI")
                else:
                    imgui.text_wrapped("Capture spectra from defined scan lines")
        
        imgui.end()
        
        # Camera view
        if frame is not None:
            # Apply overlays
            display_frame = self.roi_selector.draw_overlay(frame, self.calibrator, self.show_roi)
            
            # Draw calibration markers (if visible)
            if self.show_calibration_lines:
                for point in self.calibrator.calibration_points:
                    cv2.line(display_frame, (point.pixel_x, 0), 
                            (point.pixel_x, self.camera_height), (255, 0, 255), 1)
                    cv2.putText(display_frame, f"{point.wavelength_nm:.0f}nm", 
                               (point.pixel_x + 3, 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
            
            # Draw scan lines (if in scan lines mode and visible)
            if self.spectrum_mode == "Scan Lines" and self.show_scan_lines:
                colors = [(93, 165, 218), (250, 164, 58), (96, 189, 104), (241, 124, 176)]  # BGR format
                for i, scan_line in enumerate(self.scan_lines):
                    y = scan_line.pixel_y
                    if 0 <= y < self.camera_height:
                        color = colors[i % len(colors)]
                        # Draw horizontal line
                        cv2.line(display_frame, (0, y), (self.camera_width, y), color, 1)
                        # Draw label
                        cv2.putText(display_frame, scan_line.label, 
                                   (5, y - 5),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # Draw wavelength scale when calibrated (if visible)
            if self.show_nm_scale and self.calibrator.calibrated and self.calibrator.cached_scale_data:
                # Draw from pre-computed cache (fast!)
                scale_y = 10
                tick_height = 15
                total_width = self.calibrator.cached_scale_data['camera_width']
                
                # Draw ticks from cache
                for pixel_x, wavelength in self.calibrator.cached_scale_data['ticks']:
                    # Draw tick mark
                    cv2.line(display_frame, (pixel_x, scale_y), 
                            (pixel_x, scale_y + tick_height), (255, 255, 0), 1)
                    
                    # Draw wavelength label
                    label = f"{wavelength}"
                    cv2.putText(display_frame, label, 
                               (pixel_x - 10, scale_y + tick_height + 12),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
                
                # Draw scale line
                cv2.line(display_frame, (0, scale_y), (total_width - 1, scale_y), 
                        (255, 255, 0), 1)
                
                # Draw "nm" label
                cv2.putText(display_frame, "nm", (total_width - 30, scale_y - 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)
            
            # Create/update texture
            texture = self.create_texture(display_frame)
            
            # Camera view window - responsive to window size
            win_width, win_height = glfw.get_window_size(self.window)
            sidebar_width = 350
            imgui.set_next_window_position(0, 20)
            imgui.set_next_window_size(win_width - sidebar_width, win_height - 20)
            imgui.begin("Camera View", flags=imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_COLLAPSE)
            
            # Get camera view position for mouse handling
            self.camera_view_pos = imgui.get_cursor_screen_position()
            available_width, available_height = imgui.get_content_region_available()
            
            # Calculate aspect-preserving size
            aspect = self.camera_width / self.camera_height
            if available_width / aspect <= available_height:
                img_width = available_width
                img_height = available_width / aspect
            else:
                img_height = available_height
                img_width = available_height * aspect
            
            self.camera_view_size = (img_width, img_height)
            
            # Check if mouse is over camera view
            mouse_x, mouse_y = imgui.get_mouse_position()
            self.mouse_over_camera = (
                self.camera_view_pos[0] <= mouse_x <= self.camera_view_pos[0] + img_width and
                self.camera_view_pos[1] <= mouse_y <= self.camera_view_pos[1] + img_height
            )
            
            imgui.image(texture, img_width, img_height)
            
            imgui.end()
        
        imgui.render()
        self.impl.render(imgui.get_draw_data())
    
    def run(self):
        """Main loop"""
        if not self.init_glfw():
            return False
        
        if not self.connect_camera():
            print("Camera connection failed")
            return False
        
        print("\n" + "="*60)
        print("SPECTRUMBOI SPECTRAL VIEWER")
        print("="*60)
        print("\nUI Features:")
        print("  - Collapsible sections for organization")
        print("  - Scrollable sidebar")
        print("  - Table view for calibration points")
        print("  - Window resizing works properly")
        print("  - DPI scaling handled automatically")
        print("\nUsage:")
        print("  1. Adjust camera settings in sidebar")
        print("  2. Click & drag on camera view to select ROI")
        print("  3. Add calibration points (enter pixel X and wavelength)")
        print("  4. Click 'Fit Calibration Curve'")
        print("  5. ROI will show wavelength range!")
        print("="*60 + "\n")
        
        last_frame = None
        
        while not glfw.window_should_close(self.window):
            glfw.poll_events()
            self.impl.process_inputs()
            
            # Get camera frame
            if not self.paused and self.stream:
                new_frame = self.stream.read_latest()
                if new_frame is not None:
                    self.frame_count += 1
                    last_frame = new_frame
            
            # Render
            gl.glClearColor(0.1, 0.1, 0.1, 1.0)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT)
            
            self.render_ui(last_frame)
            
            glfw.swap_buffers(self.window)
        
        # Cleanup
        if self.stream:
            self.stream.stop_capture()
            self.stream.disconnect()
        
        self.impl.shutdown()
        glfw.terminate()
        
        return True


def main():
    """Entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='SpectrumBoi Spectral Viewer (ImGui)')
    parser.add_argument('--width', type=int, default=1280, help='Window width')
    parser.add_argument('--height', type=int, default=720, help='Window height')
    
    args = parser.parse_args()
    
    viewer = SpectralViewerImGui(width=args.width, height=args.height)
    viewer.run()


if __name__ == "__main__":
    main()