"""
PSEUDOCODE: SpectrumBoi UI Integration

This shows how to integrate HyperspectralScanner with the SpectrumBoi UI,
including progress updates, live preview, and user controls.

Integration Points:
    1. Scan control buttons (start/stop/pause)
    2. Progress bar and status display
    3. Live image preview during scan
    4. Real-time position error monitoring
    5. Post-scan results visualization
"""

import tkinter as tk
from tkinter import ttk
import threading
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# PSEUDOCODE - would import:
# from motor_control.controller import MotorController
# from motor_control.scan import HyperspectralScanner, ScanConfig, MightexCameraAdapter
# from camera_control import Camera


#=============================================================================
# SCAN CONTROL PANEL (Add to SpectrumBoi UI)
#=============================================================================

class ScanControlPanel(tk.Frame):
    """
    UI panel for hyperspectral scan control.

    Features:
        - Scan parameter configuration
        - Start/Stop/Abort buttons
        - Progress bar with position/time estimates
        - Live position error display
        - Scan statistics
    """

    def __init__(self, parent, motor_controller, camera):
        super().__init__(parent)

        self.motor = motor_controller
        self.camera = camera
        self.scanner = HyperspectralScanner(motor, MightexCameraAdapter(camera))

        # State
        self.scanning = False
        self.scan_thread = None
        self.current_session = None

        self._build_ui()
        self._register_callbacks()

    def _build_ui(self):
        """Build the scan control UI."""

        # SCAN PARAMETERS GROUP
        param_frame = ttk.LabelFrame(self, text="Scan Parameters", padding=10)
        param_frame.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        # Start angle
        ttk.Label(param_frame, text="Start Angle (°):").grid(row=0, column=0, sticky="w")
        self.start_angle_var = tk.DoubleVar(value=0.0)
        ttk.Entry(param_frame, textvariable=self.start_angle_var, width=10).grid(row=0, column=1)

        # End angle
        ttk.Label(param_frame, text="End Angle (°):").grid(row=1, column=0, sticky="w")
        self.end_angle_var = tk.DoubleVar(value=360.0)
        ttk.Entry(param_frame, textvariable=self.end_angle_var, width=10).grid(row=1, column=1)

        # Increment
        ttk.Label(param_frame, text="Increment (°):").grid(row=2, column=0, sticky="w")
        self.increment_var = tk.DoubleVar(value=1.0)
        ttk.Entry(param_frame, textvariable=self.increment_var, width=10).grid(row=2, column=1)

        # Position tolerance
        ttk.Label(param_frame, text="Tolerance (°):").grid(row=3, column=0, sticky="w")
        self.tolerance_var = tk.DoubleVar(value=0.5)
        ttk.Entry(param_frame, textvariable=self.tolerance_var, width=10).grid(row=3, column=1)

        # Settling time
        ttk.Label(param_frame, text="Settling Time (s):").grid(row=4, column=0, sticky="w")
        self.settling_var = tk.DoubleVar(value=0.5)
        ttk.Entry(param_frame, textvariable=self.settling_var, width=10).grid(row=4, column=1)

        # Calculated total positions
        ttk.Label(param_frame, text="Total Positions:").grid(row=5, column=0, sticky="w")
        self.total_positions_label = ttk.Label(param_frame, text="0")
        self.total_positions_label.grid(row=5, column=1, sticky="w")

        # Update total when parameters change
        for var in [self.start_angle_var, self.end_angle_var, self.increment_var]:
            var.trace_add("write", self._update_total_positions)

        # CONTROL BUTTONS
        control_frame = ttk.Frame(self)
        control_frame.grid(row=1, column=0, padx=5, pady=5, sticky="ew")

        self.start_button = ttk.Button(
            control_frame,
            text="Start Scan",
            command=self._start_scan,
            style="Success.TButton"
        )
        self.start_button.grid(row=0, column=0, padx=5)

        self.stop_button = ttk.Button(
            control_frame,
            text="Stop Scan",
            command=self._stop_scan,
            state="disabled",
            style="Danger.TButton"
        )
        self.stop_button.grid(row=0, column=1, padx=5)

        # PROGRESS DISPLAY
        progress_frame = ttk.LabelFrame(self, text="Scan Progress", padding=10)
        progress_frame.grid(row=2, column=0, padx=5, pady=5, sticky="ew")

        # Progress bar
        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_bar = ttk.Progressbar(
            progress_frame,
            variable=self.progress_var,
            maximum=100.0,
            length=300
        )
        self.progress_bar.grid(row=0, column=0, columnspan=2, pady=5)

        # Current position label
        self.position_label = ttk.Label(progress_frame, text="Position: -- / --")
        self.position_label.grid(row=1, column=0, sticky="w")

        # Current angle label
        self.angle_label = ttk.Label(progress_frame, text="Angle: --°")
        self.angle_label.grid(row=1, column=1, sticky="w")

        # Position error display (color-coded)
        self.error_label = ttk.Label(progress_frame, text="Error: --°")
        self.error_label.grid(row=2, column=0, columnspan=2)

        # Estimated time remaining
        self.time_remaining_label = ttk.Label(progress_frame, text="Time remaining: --")
        self.time_remaining_label.grid(row=3, column=0, columnspan=2)

        # STATISTICS DISPLAY
        stats_frame = ttk.LabelFrame(self, text="Scan Statistics", padding=10)
        stats_frame.grid(row=3, column=0, padx=5, pady=5, sticky="ew")

        self.captures_label = ttk.Label(stats_frame, text="Captures: 0")
        self.captures_label.grid(row=0, column=0, sticky="w")

        self.failures_label = ttk.Label(stats_frame, text="Failures: 0")
        self.failures_label.grid(row=1, column=0, sticky="w")

        self.mean_error_label = ttk.Label(stats_frame, text="Mean Error: --°")
        self.mean_error_label.grid(row=2, column=0, sticky="w")

        self.max_error_label = ttk.Label(stats_frame, text="Max Error: --°")
        self.max_error_label.grid(row=3, column=0, sticky="w")

    def _update_total_positions(self, *args):
        """Calculate and display total number of positions."""
        try:
            start = self.start_angle_var.get()
            end = self.end_angle_var.get()
            inc = self.increment_var.get()

            if inc > 0 and end > start:
                total = int((end - start) / inc) + 1
                self.total_positions_label.config(text=str(total))
            else:
                self.total_positions_label.config(text="Invalid")
        except:
            self.total_positions_label.config(text="--")

    def _register_callbacks(self):
        """Register scanner callbacks for UI updates."""

        # Progress callback - update progress bar and labels
        def on_progress(current, total, angle):
            # This is called from scan thread, so use after() for thread safety
            self.after(0, self._update_progress, current, total, angle)

        # Capture callback - update preview and statistics
        def on_capture(image, metadata):
            self.after(0, self._update_capture, image, metadata)

        # Error callback - display error messages
        def on_error(message, exception):
            self.after(0, self._show_error, message, exception)

        # Completion callback - show results
        def on_complete(session):
            self.after(0, self._scan_complete, session)

        self.scanner.on_progress(on_progress)
        self.scanner.on_capture(on_capture)
        self.scanner.on_error(on_error)
        self.scanner.on_complete(on_complete)

    def _start_scan(self):
        """Start a new scan in a background thread."""

        # Build config from UI parameters
        config = ScanConfig(
            start_angle_deg=self.start_angle_var.get(),
            end_angle_deg=self.end_angle_var.get(),
            increment_deg=self.increment_var.get(),
            position_tolerance_deg=self.tolerance_var.get(),
            settling_time_sec=self.settling_var.get(),
            # ... other camera/storage parameters
        )

        # Update UI state
        self.scanning = True
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        self.progress_var.set(0.0)

        # Start scan in background thread
        self.scan_thread = threading.Thread(
            target=self._run_scan_thread,
            args=(config,),
            daemon=True
        )
        self.scan_thread.start()

    def _run_scan_thread(self, config):
        """Run scan in background thread."""
        try:
            session = self.scanner.run_scan(config)
            self.current_session = session
        except Exception as e:
            # Error will be reported via error callback
            pass
        finally:
            # Reset UI state in main thread
            self.after(0, self._reset_ui_state)

    def _stop_scan(self):
        """Abort current scan."""
        self.scanner.abort_scan()
        self.stop_button.config(state="disabled")

    def _reset_ui_state(self):
        """Reset UI to idle state after scan."""
        self.scanning = False
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")

    #=========================================================================
    # CALLBACK HANDLERS (called from main thread via after())
    #=========================================================================

    def _update_progress(self, current, total, angle):
        """Update progress display."""
        progress_percent = (current / total) * 100.0
        self.progress_var.set(progress_percent)

        self.position_label.config(text=f"Position: {current + 1} / {total}")
        self.angle_label.config(text=f"Angle: {angle:.2f}°")

        # Estimate time remaining (simple linear estimate)
        # TODO: Use actual timing data from session
        # elapsed = ...
        # rate = current / elapsed
        # remaining = (total - current) / rate
        # self.time_remaining_label.config(text=f"Time remaining: {remaining:.0f}s")

    def _update_capture(self, image, metadata):
        """Handle new capture - update preview and statistics."""

        # Update position error display (color-coded)
        error = metadata.position_error_deg
        if error < self.tolerance_var.get() / 2:
            color = "green"
        elif error < self.tolerance_var.get():
            color = "orange"
        else:
            color = "red"

        self.error_label.config(
            text=f"Error: {error:.3f}°",
            foreground=color
        )

        # Update statistics
        if self.current_session:
            self.captures_label.config(
                text=f"Captures: {self.current_session.num_successful_captures}"
            )
            self.failures_label.config(
                text=f"Failures: {self.current_session.num_failed_positions}"
            )
            self.mean_error_label.config(
                text=f"Mean Error: {self.current_session.mean_position_error:.3f}°"
            )
            self.max_error_label.config(
                text=f"Max Error: {self.current_session.max_position_error:.3f}°"
            )

        # Update live preview (if preview panel exists)
        # self.parent.update_preview_image(image)

    def _show_error(self, message, exception):
        """Display error message."""
        # Show in status bar or error dialog
        # self.parent.show_error(message)
        print(f"ERROR: {message}")
        if exception:
            print(f"  Exception: {exception}")

    def _scan_complete(self, session):
        """Handle scan completion."""
        # Show completion dialog
        result_text = (
            f"Scan Complete!\n\n"
            f"Duration: {session.duration_seconds:.1f}s\n"
            f"Successful captures: {session.num_successful_captures}\n"
            f"Failed positions: {session.num_failed_positions}\n"
            f"Success rate: {session.success_rate * 100:.1f}%\n"
            f"Mean position error: {session.mean_position_error:.3f}°\n"
            f"Max position error: {session.max_position_error:.3f}°"
        )

        # PSEUDOCODE - show dialog:
        # messagebox.showinfo("Scan Complete", result_text)

        # Optionally open results visualization
        # self._show_results_window(session)


#=============================================================================
# RESULTS VISUALIZATION WINDOW
#=============================================================================

class ScanResultsWindow(tk.Toplevel):
    """
    Window for visualizing scan results.

    Shows:
        - Position error plot vs scan angle
        - Captured images browser
        - Data cube preview (if assembled)
        - Export options
    """

    def __init__(self, parent, session):
        super().__init__(parent)
        self.session = session

        self.title(f"Scan Results - {session.session_id}")
        self.geometry("900x700")

        self._build_ui()
        self._plot_results()

    def _build_ui(self):
        """Build results visualization UI."""

        # TABS
        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True, padx=5, pady=5)

        # Tab 1: Statistics and plots
        stats_tab = ttk.Frame(notebook)
        notebook.add(stats_tab, text="Statistics")

        # Tab 2: Image browser
        images_tab = ttk.Frame(notebook)
        notebook.add(images_tab, text="Images")

        # Tab 3: Data cube (if available)
        cube_tab = ttk.Frame(notebook)
        notebook.add(cube_tab, text="Data Cube")

        # Build stats tab
        self._build_stats_tab(stats_tab)

    def _build_stats_tab(self, parent):
        """Build statistics and plots tab."""

        # Summary text
        summary_frame = ttk.LabelFrame(parent, text="Summary", padding=10)
        summary_frame.pack(fill="x", padx=5, pady=5)

        summary_text = (
            f"Session ID: {self.session.session_id}\n"
            f"Duration: {self.session.duration_seconds:.1f} seconds\n"
            f"Successful captures: {self.session.num_successful_captures}\n"
            f"Failed positions: {self.session.num_failed_positions}\n"
            f"Success rate: {self.session.success_rate * 100:.1f}%\n"
            f"Mean position error: {self.session.mean_position_error:.3f}°\n"
            f"Max position error: {self.session.max_position_error:.3f}°"
        )

        ttk.Label(summary_frame, text=summary_text, justify="left").pack()

        # Plot frame
        plot_frame = ttk.LabelFrame(parent, text="Position Error Analysis", padding=10)
        plot_frame.pack(fill="both", expand=True, padx=5, pady=5)

        # Matplotlib figure
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

        # Plot 1: Position error vs angle
        angles = [c.commanded_angle_deg for c in self.session.captures]
        errors = [c.position_error_deg for c in self.session.captures]

        ax1.plot(angles, errors, 'o-', markersize=3)
        ax1.axhline(y=self.session.config.position_tolerance_deg,
                   color='r', linestyle='--', label='Tolerance')
        ax1.set_xlabel('Commanded Angle (°)')
        ax1.set_ylabel('Position Error (°)')
        ax1.set_title('Position Error vs Scan Angle')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: Error histogram
        ax2.hist(errors, bins=30, edgecolor='black')
        ax2.axvline(x=self.session.mean_position_error,
                   color='r', linestyle='--', label='Mean')
        ax2.set_xlabel('Position Error (°)')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Position Error Distribution')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        fig.tight_layout()

        # Embed in tkinter
        canvas = FigureCanvasTkAgg(fig, master=plot_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)

    def _plot_results(self):
        """Generate result plots."""
        pass  # Implemented in _build_stats_tab


#=============================================================================
# INTEGRATION WITH EXISTING SPECTRUMBOI
#=============================================================================

"""
To integrate into existing SpectrumBoi UI:

1. Add scan control panel to main window:

    class SpectrumBoiWindow:
        def __init__(self):
            # ... existing initialization ...

            # Add scan control panel
            self.scan_panel = ScanControlPanel(
                self.sidebar,
                motor_controller=self.motor,
                camera=self.camera
            )
            self.scan_panel.pack(fill="x", padx=5, pady=5)

2. Connect motor controller instance:

    # In SpectrumBoi initialization:
    from motor_control import MotorController

    self.motor = MotorController('/dev/ttyUSB0')
    self.motor.connect()

3. Connect camera instance (already exists):

    # Use existing camera instance
    self.camera = Camera()  # or however it's initialized

4. Add menu items:

    scan_menu = tk.Menu(menubar, tearoff=0)
    menubar.add_cascade(label="Scan", menu=scan_menu)
    scan_menu.add_command(label="New Scan", command=self.scan_panel._start_scan)
    scan_menu.add_command(label="View Results", command=self._open_results)

5. Handle cleanup on exit:

    def on_closing(self):
        if self.scan_panel.scanning:
            self.scan_panel._stop_scan()

        self.motor.disconnect()
        self.destroy()
"""


#=============================================================================
# MINIMAL EXAMPLE (Standalone)
#=============================================================================

if __name__ == "__main__":
    """Minimal standalone example for testing."""

    root = tk.Tk()
    root.title("Hyperspectral Scan Control - Test")

    # PSEUDOCODE - would initialize real hardware:
    # motor = MotorController('/dev/ttyUSB0')
    # motor.connect()
    # camera = Camera()

    # For testing, use None (panel would need null checks)
    motor = None
    camera = None

    panel = ScanControlPanel(root, motor, camera)
    panel.pack(fill="both", expand=True, padx=10, pady=10)

    root.mainloop()
