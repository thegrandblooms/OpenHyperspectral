"""
Modern UI Module for Mightex Camera
Clean, simple, dark-themed interface with sidebar controls

Design Principles:
- Dark grey sidebar on right
- Light grey text for readability
- Modern, minimal aesthetic
- Clear visual hierarchy
- Responsive layout
"""

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional, Callable


@dataclass
class ColorScheme:
    """Modern dark theme color palette"""
    # Background colors
    sidebar_bg = (45, 45, 45)      # Dark grey sidebar
    sidebar_dark = (35, 35, 35)    # Darker accent
    
    # Text colors
    text_primary = (220, 220, 220)   # Light grey
    text_secondary = (160, 160, 160) # Medium grey
    text_accent = (100, 200, 255)    # Light blue accent
    
    # UI elements
    slider_track = (60, 60, 60)      # Slider background
    slider_fill = (100, 200, 255)    # Slider filled portion
    slider_handle = (255, 255, 255)  # Slider handle
    
    # Status colors
    status_good = (100, 200, 100)    # Green
    status_warning = (255, 200, 100) # Orange
    status_error = (255, 100, 100)   # Red
    
    # Dividers
    divider = (70, 70, 70)           # Subtle line


class ModernUI:
    """
    Modern camera interface with dark sidebar
    
    Features:
    - Dark grey sidebar on right side
    - Clean, modern design
    - Interactive sliders
    - Real-time stats display
    - Responsive to window size
    """
    
    def __init__(self, sidebar_width: int = 300):
        """
        Initialize UI
        
        Args:
            sidebar_width: Width of sidebar in pixels
        """
        self.sidebar_width = sidebar_width
        self.colors = ColorScheme()
        
        # Layout metrics
        self.padding = 20
        self.line_height = 30
        self.section_spacing = 15
        self.slider_height = 8
        self.slider_handle_radius = 12
        
        # Font settings
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale_title = 0.7
        self.font_scale_normal = 0.5
        self.font_scale_small = 0.4
        self.font_thickness = 1
        
        # Slider interaction state
        self.dragging_slider = None
        self.mouse_x = 0
        self.mouse_y = 0
        
        # Slider regions (for mouse interaction)
        self.slider_regions = {}  # {'exposure': (x, y, width, min, max), 'gain': ...}
        
    def create_frame_with_sidebar(self, camera_frame: np.ndarray, 
                                  window_width: int,
                                  window_height: int) -> np.ndarray:
        """
        Create composite frame with camera image and sidebar
        
        Args:
            camera_frame: Camera image
            window_width: Target window width
            window_height: Target window height (minus trackbar space)
            
        Returns:
            Composite frame with sidebar
        """
        # Calculate camera display area
        camera_width = window_width - self.sidebar_width
        
        # Scale camera frame to fit display area
        scaled_camera = self._scale_frame(camera_frame, camera_width, window_height)
        
        # Create canvas
        canvas = np.zeros((window_height, window_width, 3), dtype=np.uint8)
        
        # Place camera frame (left side)
        cam_h, cam_w = scaled_camera.shape[:2]
        y_offset = (window_height - cam_h) // 2
        canvas[y_offset:y_offset+cam_h, 0:cam_w] = scaled_camera
        
        # Draw sidebar (right side)
        sidebar_x = camera_width
        canvas[:, sidebar_x:] = self.colors.sidebar_bg
        
        return canvas, sidebar_x
    
    def draw_sidebar_content(self, canvas: np.ndarray, sidebar_x: int,
                            fps: float,
                            exposure_ms: float,
                            gain: int,
                            brightness: float,
                            frame_count: int,
                            exposure_callback: Optional[Callable] = None,
                            gain_callback: Optional[Callable] = None) -> np.ndarray:
        """
        Draw all sidebar content
        
        Args:
            canvas: Frame with sidebar area
            sidebar_x: X position where sidebar starts
            fps, exposure_ms, gain, brightness, frame_count: Camera stats
            exposure_callback: Function to call when exposure slider changes
            gain_callback: Function to call when gain slider changes
            
        Returns:
            Frame with sidebar content
        """
        y_pos = self.padding
        
        # Title section
        y_pos = self._draw_title(canvas, sidebar_x, y_pos)
        y_pos += self.section_spacing
        
        # Divider
        y_pos = self._draw_divider(canvas, sidebar_x, y_pos)
        y_pos += self.section_spacing
        
        # Controls section
        y_pos = self._draw_section_header(canvas, sidebar_x, y_pos, "CONTROLS")
        y_pos += 10
        
        # Exposure slider
        y_pos = self._draw_slider(canvas, sidebar_x, y_pos, 
                                 "Exposure", exposure_ms, 1, 750, "ms",
                                 "exposure_slider", exposure_callback)
        y_pos += self.section_spacing
        
        # Gain slider
        y_pos = self._draw_slider(canvas, sidebar_x, y_pos,
                                 "Gain", gain, 1, 64, "",
                                 "gain_slider", gain_callback)
        y_pos += self.section_spacing
        
        # Divider
        y_pos = self._draw_divider(canvas, sidebar_x, y_pos)
        y_pos += self.section_spacing
        
        # Stats section
        y_pos = self._draw_section_header(canvas, sidebar_x, y_pos, "STATISTICS")
        y_pos += 10
        
        # FPS with status color
        fps_color = self._get_fps_color(fps)
        y_pos = self._draw_stat(canvas, sidebar_x, y_pos, "FPS", f"{fps:.1f}", fps_color)
        
        # Brightness
        y_pos = self._draw_stat(canvas, sidebar_x, y_pos, "Brightness", f"{brightness:.1f}")
        
        # Frame count
        y_pos = self._draw_stat(canvas, sidebar_x, y_pos, "Frames", f"{frame_count}")
        
        y_pos += self.section_spacing
        
        # Divider
        y_pos = self._draw_divider(canvas, sidebar_x, y_pos)
        y_pos += self.section_spacing
        
        # Controls hint section
        y_pos = self._draw_section_header(canvas, sidebar_x, y_pos, "CONTROLS")
        y_pos += 10
        
        controls = [
            ("Sliders", "Click/drag"),
            ("Q / ESC", "Quit"),
            ("S", "Screenshot"),
        ]
        
        for key, action in controls:
            y_pos = self._draw_shortcut(canvas, sidebar_x, y_pos, key, action)
        
        return canvas
    
    def _draw_title(self, canvas: np.ndarray, sidebar_x: int, y_pos: int) -> int:
        """Draw title section"""
        text = "SpectrumBoi"
        text_size = cv2.getTextSize(text, self.font, self.font_scale_title, 
                                    self.font_thickness + 1)[0]
        x_pos = sidebar_x + (self.sidebar_width - text_size[0]) // 2
        
        cv2.putText(canvas, text, (x_pos, y_pos + 20),
                   self.font, self.font_scale_title, self.colors.text_accent,
                   self.font_thickness + 1, cv2.LINE_AA)
        
        return y_pos + 30
    
    def _draw_section_header(self, canvas: np.ndarray, sidebar_x: int, 
                            y_pos: int, text: str) -> int:
        """Draw section header"""
        x_pos = sidebar_x + self.padding
        
        cv2.putText(canvas, text, (x_pos, y_pos + 15),
                   self.font, self.font_scale_small, self.colors.text_secondary,
                   self.font_thickness, cv2.LINE_AA)
        
        return y_pos + 25
    
    def _draw_divider(self, canvas: np.ndarray, sidebar_x: int, y_pos: int) -> int:
        """Draw horizontal divider line"""
        x1 = sidebar_x + self.padding
        x2 = sidebar_x + self.sidebar_width - self.padding
        
        cv2.line(canvas, (x1, y_pos), (x2, y_pos), self.colors.divider, 1)
        
        return y_pos + 5
    
    def _draw_stat(self, canvas: np.ndarray, sidebar_x: int, y_pos: int,
                  label: str, value: str, value_color=None) -> int:
        """Draw a statistic line"""
        x_pos = sidebar_x + self.padding
        
        if value_color is None:
            value_color = self.colors.text_primary
        
        # Label
        cv2.putText(canvas, label, (x_pos, y_pos + 15),
                   self.font, self.font_scale_small, self.colors.text_secondary,
                   self.font_thickness, cv2.LINE_AA)
        
        # Value (right-aligned)
        value_size = cv2.getTextSize(value, self.font, self.font_scale_normal, 
                                     self.font_thickness)[0]
        value_x = sidebar_x + self.sidebar_width - self.padding - value_size[0]
        
        cv2.putText(canvas, value, (value_x, y_pos + 15),
                   self.font, self.font_scale_normal, value_color,
                   self.font_thickness, cv2.LINE_AA)
        
        return y_pos + self.line_height
    
    def _draw_shortcut(self, canvas: np.ndarray, sidebar_x: int, y_pos: int,
                      key: str, action: str) -> int:
        """Draw a keyboard shortcut"""
        x_pos = sidebar_x + self.padding
        
        # Key in accent color
        cv2.putText(canvas, key, (x_pos, y_pos + 12),
                   self.font, self.font_scale_small, self.colors.text_accent,
                   self.font_thickness, cv2.LINE_AA)
        
        # Action in secondary color
        key_width = cv2.getTextSize(key, self.font, self.font_scale_small,
                                    self.font_thickness)[0][0]
        
        cv2.putText(canvas, action, (x_pos + key_width + 15, y_pos + 12),
                   self.font, self.font_scale_small, self.colors.text_secondary,
                   self.font_thickness, cv2.LINE_AA)
        
        return y_pos + 22
    
    def _draw_slider(self, canvas: np.ndarray, sidebar_x: int, y_pos: int,
                    label: str, value: float, min_val: float, max_val: float,
                    unit: str, slider_id: str, callback: Optional[Callable]) -> int:
        """Draw an interactive slider"""
        x_pos = sidebar_x + self.padding
        slider_width = self.sidebar_width - (2 * self.padding)
        
        # Label and value
        label_text = f"{label}: {value:.0f}{unit}"
        cv2.putText(canvas, label_text, (x_pos, y_pos + 12),
                   self.font, self.font_scale_small, self.colors.text_primary,
                   self.font_thickness, cv2.LINE_AA)
        
        y_pos += 25
        
        # Slider track background
        track_y = y_pos + self.slider_height // 2
        cv2.line(canvas, (x_pos, track_y), (x_pos + slider_width, track_y),
                self.colors.slider_track, self.slider_height, cv2.LINE_AA)
        
        # Slider filled portion
        value_normalized = (value - min_val) / (max_val - min_val)
        fill_width = int(slider_width * value_normalized)
        
        if fill_width > 0:
            cv2.line(canvas, (x_pos, track_y), (x_pos + fill_width, track_y),
                    self.colors.slider_fill, self.slider_height, cv2.LINE_AA)
        
        # Slider handle
        handle_x = x_pos + fill_width
        cv2.circle(canvas, (handle_x, track_y), self.slider_handle_radius,
                  self.colors.slider_handle, -1, cv2.LINE_AA)
        cv2.circle(canvas, (handle_x, track_y), self.slider_handle_radius,
                  self.colors.sidebar_dark, 2, cv2.LINE_AA)
        
        # Store slider region for mouse interaction
        self.slider_regions[slider_id] = {
            'x': x_pos,
            'y': track_y,
            'width': slider_width,
            'min': min_val,
            'max': max_val,
            'callback': callback
        }
        
        return y_pos + 30
    
    def handle_mouse(self, event, x, y, flags, param):
        """Handle mouse events for slider interaction"""
        self.mouse_x = x
        self.mouse_y = y
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # Check if clicking on any slider
            for slider_id, region in self.slider_regions.items():
                slider_y = region['y']
                # Check if within slider vertical range
                if abs(y - slider_y) < self.slider_handle_radius + 5:
                    # Check if within slider horizontal range
                    if region['x'] <= x <= region['x'] + region['width']:
                        self.dragging_slider = slider_id
                        self._update_slider_value(slider_id, x)
                        break
        
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.dragging_slider:
                self._update_slider_value(self.dragging_slider, x)
        
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging_slider = None
    
    def _update_slider_value(self, slider_id: str, mouse_x: int):
        """Update slider value based on mouse position"""
        if slider_id not in self.slider_regions:
            return
        
        region = self.slider_regions[slider_id]
        
        # Calculate normalized position (0.0 to 1.0)
        relative_x = mouse_x - region['x']
        normalized = max(0.0, min(1.0, relative_x / region['width']))
        
        # Calculate actual value
        value_range = region['max'] - region['min']
        new_value = region['min'] + (normalized * value_range)
        
        # Call the callback if it exists
        if region['callback']:
            region['callback'](new_value)
    
    def _get_fps_color(self, fps: float) -> Tuple[int, int, int]:
        """Get color based on FPS performance"""
        if fps >= 15:
            return self.colors.status_good
        elif fps >= 10:
            return self.colors.status_warning
        else:
            return self.colors.status_error
    
    def _scale_frame(self, frame: np.ndarray, target_width: int, 
                    target_height: int) -> np.ndarray:
        """Scale frame to fit display area while maintaining aspect ratio"""
        frame_height, frame_width = frame.shape[:2]
        
        # Calculate scaling to fit
        scale_w = target_width / frame_width
        scale_h = target_height / frame_height
        scale = min(scale_w, scale_h)
        
        # Calculate new dimensions
        new_width = int(frame_width * scale)
        new_height = int(frame_height * scale)
        
        # Choose interpolation method
        if scale > 1.0:
            interpolation = cv2.INTER_LINEAR
        else:
            interpolation = cv2.INTER_AREA
        
        # Resize
        scaled = cv2.resize(frame, (new_width, new_height), 
                          interpolation=interpolation)
        
        return scaled


def create_modern_ui(sidebar_width: int = 300):
    """
    Factory function to create UI instance
    
    Args:
        sidebar_width: Width of sidebar
        
    Returns:
        ModernUI instance
    """
    return ModernUI(sidebar_width)


# Example usage
if __name__ == "__main__":
    import numpy as np
    
    # Create test frame
    test_frame = np.random.randint(0, 255, (480, 752, 3), dtype=np.uint8)
    
    # Create UI
    ui = ModernUI(sidebar_width=300)
    
    # Create composite
    composite, sidebar_x = ui.create_frame_with_sidebar(test_frame, 1052, 480)
    
    # Draw sidebar content
    composite = ui.draw_sidebar_content(
        composite, sidebar_x,
        fps=24.5,
        exposure_ms=15.0,
        gain=24,
        brightness=128.5,
        frame_count=1234
    )
    
    # Display
    cv2.imshow('Modern UI Demo', composite)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    print("UI module ready!")
