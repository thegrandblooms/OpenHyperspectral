#!/usr/bin/env python3
"""
PID Tuning Script for OpenHyperspectral Motor Control

This script provides comprehensive PID tuning capabilities:
1. Step Response Analysis - Measure rise time, overshoot, settling time, steady-state error
2. Frequency Response Testing - Find bandwidth and phase margin
3. Auto-Tuning (Ziegler-Nichols) - Automatic PID parameter calculation
4. Parameter Sweep - Test ranges of P, I, D values and find optimal settings

Usage:
    python pid_tuner.py <port> [--mode <mode>] [--plot]

Modes:
    step      - Run step response test (default)
    freq      - Run frequency response test
    autotune  - Run Ziegler-Nichols auto-tuning
    sweep     - Run parameter sweep
    manual    - Interactive manual tuning

Requirements:
    pip install matplotlib numpy pySerialTransfer

Based on dev_log_10 specifications for OpenHyperspectral project.
"""

import argparse
import time
import math
import sys
import struct
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Callable
from enum import Enum

# Try to import optional dependencies
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    print("Warning: numpy not installed. Some features may be limited.")

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not installed. Plotting disabled.")

# Import serial communication
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    print("Warning: pyserial not installed.")


#=============================================================================
# CONFIGURATION
#=============================================================================

# Default serial settings
DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT = 2.0

# Step response test parameters
STEP_SIZE_DEG = 30.0           # Default step size in degrees
STEP_SETTLE_TIME = 3.0         # Time to wait for settling (seconds)
SAMPLE_INTERVAL_MS = 10        # Sampling interval (milliseconds)

# Tolerance thresholds (degrees)
POSITION_TOLERANCE_DEG = 0.5   # Target reached threshold
SETTLING_TOLERANCE_DEG = 1.0   # For settling time calculation
STEADY_STATE_WINDOW = 0.5      # Window for steady-state error (seconds)

# Auto-tuning parameters
AUTOTUNE_AMPLITUDE_DEG = 5.0   # Relay amplitude for relay feedback
AUTOTUNE_HYSTERESIS_DEG = 0.5  # Hysteresis for relay switching
AUTOTUNE_CYCLES = 5            # Number of oscillation cycles to measure
AUTOTUNE_TIMEOUT = 60.0        # Timeout for auto-tuning

# Parameter sweep defaults
SWEEP_P_RANGE = (5.0, 30.0, 5.0)    # (min, max, step)
SWEEP_I_RANGE = (0.0, 2.0, 0.5)     # (min, max, step)
SWEEP_D_RANGE = (0.0, 2.0, 0.5)     # (min, max, step)


#=============================================================================
# DATA STRUCTURES
#=============================================================================

@dataclass
class StepResponseMetrics:
    """Metrics from a step response test."""
    rise_time: float = 0.0           # Time to reach 90% of target (seconds)
    overshoot: float = 0.0           # Peak overshoot (degrees)
    overshoot_percent: float = 0.0   # Overshoot as percentage of step size
    settling_time: float = 0.0       # Time to settle within tolerance (seconds)
    steady_state_error: float = 0.0  # Final steady-state error (degrees)
    peak_time: float = 0.0           # Time to reach peak (seconds)
    step_size: float = 0.0           # Actual step size (degrees)

    # Raw data for plotting
    timestamps: List[float] = field(default_factory=list)
    positions: List[float] = field(default_factory=list)
    target: float = 0.0
    start_position: float = 0.0

    def score(self) -> float:
        """Calculate composite score (lower is better)."""
        return (
            self.overshoot * 10.0 +           # Heavily penalize overshoot
            self.settling_time * 2.0 +        # Penalize slow settling
            self.steady_state_error * 5.0 +   # Penalize steady-state error
            self.rise_time * 0.5              # Small penalty for slow rise
        )

    def __str__(self) -> str:
        return (
            f"Step Response Metrics:\n"
            f"  Rise Time (10-90%): {self.rise_time:.3f} s\n"
            f"  Overshoot: {self.overshoot:.2f}° ({self.overshoot_percent:.1f}%)\n"
            f"  Peak Time: {self.peak_time:.3f} s\n"
            f"  Settling Time: {self.settling_time:.3f} s\n"
            f"  Steady-State Error: {self.steady_state_error:.3f}°\n"
            f"  Score: {self.score():.2f}"
        )


@dataclass
class FrequencyResponsePoint:
    """Single point in frequency response."""
    frequency_hz: float = 0.0
    amplitude_ratio: float = 0.0   # Output amplitude / Input amplitude
    phase_lag_deg: float = 0.0     # Phase lag in degrees


@dataclass
class PIDParams:
    """PID controller parameters."""
    p: float = 20.0
    i: float = 0.0
    d: float = 0.0
    ramp: float = 1000.0  # degrees/second

    def __str__(self) -> str:
        return f"P={self.p:.2f}, I={self.i:.2f}, D={self.d:.3f}, Ramp={self.ramp:.1f}"


@dataclass
class SweepResult:
    """Result from parameter sweep."""
    params: PIDParams
    metrics: StepResponseMetrics
    score: float


#=============================================================================
# SERIAL MOTOR INTERFACE
#=============================================================================

class MotorInterface:
    """
    Low-level serial interface to motor controller.

    Uses text commands for simplicity and debugging visibility.
    Commands match the ESP32 firmware's serial command handler.
    """

    def __init__(self, port: str, baud_rate: int = DEFAULT_BAUD_RATE,
                 timeout: float = DEFAULT_TIMEOUT):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        self.verbose = False

    def connect(self) -> bool:
        """Connect to motor controller."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            time.sleep(2.0)  # Wait for ESP32 reset
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            # Test connection
            self.serial.write(b"ping\n")
            time.sleep(0.1)
            response = self._read_response(timeout=1.0)

            if "pong" in response.lower() or "ping" in response.lower():
                self.connected = True
                print(f"Connected to motor controller on {self.port}")
                return True
            else:
                # Try anyway - some firmware versions may not have ping
                self.connected = True
                print(f"Connected to {self.port} (ping response: {response[:50]}...)")
                return True

        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from motor controller."""
        if self.serial:
            try:
                self.disable()
            except:
                pass
            self.serial.close()
            self.serial = None
        self.connected = False

    def _send_command(self, cmd: str) -> str:
        """Send command and get response."""
        if not self.connected:
            raise RuntimeError("Not connected to motor controller")

        if self.verbose:
            print(f">>> {cmd}")

        self.serial.write(f"{cmd}\n".encode())
        time.sleep(0.05)
        response = self._read_response()

        if self.verbose:
            print(f"<<< {response[:100]}")

        return response

    def _read_response(self, timeout: float = None) -> str:
        """Read response from serial."""
        if timeout is None:
            timeout = self.timeout

        response = ""
        start_time = time.time()

        while time.time() - start_time < timeout:
            if self.serial.in_waiting:
                data = self.serial.read(self.serial.in_waiting)
                response += data.decode('utf-8', errors='ignore')
                time.sleep(0.01)
            else:
                if response:
                    break
                time.sleep(0.01)

        return response.strip()

    def calibrate(self) -> bool:
        """Run motor calibration."""
        print("Running calibration...")
        response = self._send_command("c")
        time.sleep(3.0)  # Calibration takes time
        response += self._read_response(timeout=5.0)

        if "complete" in response.lower() or "success" in response.lower():
            print("Calibration successful")
            return True
        else:
            print(f"Calibration response: {response}")
            return "error" not in response.lower()

    def enable(self) -> bool:
        """Enable motor."""
        response = self._send_command("e")
        return "error" not in response.lower()

    def disable(self) -> bool:
        """Disable motor."""
        response = self._send_command("d")
        return True

    def move_to(self, position_deg: float) -> bool:
        """Move to position in degrees."""
        response = self._send_command(f"m {position_deg:.2f}")
        return "error" not in response.lower()

    def get_position(self) -> Optional[float]:
        """Get current position in degrees."""
        response = self._send_command("s")

        # Parse position from status output
        # Expected format includes "Pos=123.45°" or similar
        import re

        # Try different patterns
        patterns = [
            r'Pos[=:]?\s*([-\d.]+)',
            r'Position[=:]?\s*([-\d.]+)',
            r'FOC[=:]?\s*([-\d.]+)',
            r'shaft_angle[=:]?\s*([-\d.]+)',
        ]

        for pattern in patterns:
            match = re.search(pattern, response, re.IGNORECASE)
            if match:
                try:
                    return float(match.group(1))
                except ValueError:
                    continue

        return None

    def get_velocity(self) -> Optional[float]:
        """Get current velocity in degrees/second."""
        response = self._send_command("s")

        import re
        patterns = [
            r'Vel[=:]?\s*([-\d.]+)',
            r'Velocity[=:]?\s*([-\d.]+)',
        ]

        for pattern in patterns:
            match = re.search(pattern, response, re.IGNORECASE)
            if match:
                try:
                    return float(match.group(1))
                except ValueError:
                    continue

        return None

    def set_pid(self, controller_type: int, p: float, i: float, d: float,
                ramp: float) -> bool:
        """
        Set PID parameters.

        controller_type: 0=position, 1=velocity, 2=current
        """
        # Command format: pid <type> <p> <i> <d> <ramp>
        response = self._send_command(f"pid {controller_type} {p:.3f} {i:.3f} {d:.4f} {ramp:.1f}")
        return "error" not in response.lower()

    def set_position_pid(self, params: PIDParams) -> bool:
        """Set position PID parameters."""
        return self.set_pid(0, params.p, params.i, params.d, params.ramp)

    def sample_position(self, duration: float, interval_ms: float = 10) -> Tuple[List[float], List[float]]:
        """
        Sample position over time.

        Returns: (timestamps, positions)
        """
        timestamps = []
        positions = []

        start_time = time.time()
        interval_s = interval_ms / 1000.0

        while time.time() - start_time < duration:
            sample_start = time.time()

            pos = self.get_position()
            if pos is not None:
                timestamps.append(time.time() - start_time)
                positions.append(pos)

            # Maintain consistent sample rate
            elapsed = time.time() - sample_start
            if elapsed < interval_s:
                time.sleep(interval_s - elapsed)

        return timestamps, positions


#=============================================================================
# STEP RESPONSE ANALYSIS
#=============================================================================

class StepResponseAnalyzer:
    """Analyze motor step response."""

    def __init__(self, motor: MotorInterface):
        self.motor = motor

    def run_step_test(self, step_size: float = STEP_SIZE_DEG,
                      settle_time: float = STEP_SETTLE_TIME,
                      sample_interval_ms: float = SAMPLE_INTERVAL_MS) -> StepResponseMetrics:
        """
        Run a step response test.

        Args:
            step_size: Step size in degrees
            settle_time: Time to observe settling
            sample_interval_ms: Sampling interval in milliseconds

        Returns:
            StepResponseMetrics with all measurements
        """
        metrics = StepResponseMetrics()

        # Get starting position
        start_pos = self.motor.get_position()
        if start_pos is None:
            print("Error: Could not read starting position")
            return metrics

        target_pos = start_pos + step_size
        metrics.start_position = start_pos
        metrics.target = target_pos
        metrics.step_size = step_size

        print(f"\nStep test: {start_pos:.2f}° -> {target_pos:.2f}° (step: {step_size:.1f}°)")

        # Start sampling thread-like loop
        timestamps = []
        positions = []

        # Command the step
        sample_start = time.time()
        self.motor.move_to(target_pos)

        # Sample during settling
        interval_s = sample_interval_ms / 1000.0

        while time.time() - sample_start < settle_time:
            loop_start = time.time()

            pos = self.motor.get_position()
            if pos is not None:
                timestamps.append(time.time() - sample_start)
                positions.append(pos)

            # Maintain sample rate
            elapsed = time.time() - loop_start
            if elapsed < interval_s:
                time.sleep(interval_s - elapsed)

        if not positions:
            print("Error: No position samples collected")
            return metrics

        # Store raw data
        metrics.timestamps = timestamps
        metrics.positions = positions

        # Analyze the response
        self._analyze_response(metrics)

        return metrics

    def _analyze_response(self, metrics: StepResponseMetrics):
        """Analyze collected step response data."""
        if not metrics.positions:
            return

        start = metrics.start_position
        target = metrics.target
        step = metrics.step_size
        direction = 1 if step > 0 else -1

        # Convert to numpy if available for easier analysis
        if HAS_NUMPY:
            t = np.array(metrics.timestamps)
            pos = np.array(metrics.positions)
        else:
            t = metrics.timestamps
            pos = metrics.positions

        # Calculate relative position (0 = start, 1 = target)
        if abs(step) > 0.01:
            relative_pos = [(p - start) / step for p in pos]
        else:
            relative_pos = [0.0] * len(pos)

        # Rise time (10% to 90% of step)
        rise_start_idx = None
        rise_end_idx = None

        for i, rp in enumerate(relative_pos):
            if rise_start_idx is None and rp >= 0.1:
                rise_start_idx = i
            if rise_end_idx is None and rp >= 0.9:
                rise_end_idx = i
                break

        if rise_start_idx is not None and rise_end_idx is not None:
            metrics.rise_time = t[rise_end_idx] - t[rise_start_idx]

        # Find peak overshoot
        if direction > 0:
            peak_pos = max(pos)
            peak_idx = pos.index(peak_pos) if isinstance(pos, list) else int(np.argmax(pos))
        else:
            peak_pos = min(pos)
            peak_idx = pos.index(peak_pos) if isinstance(pos, list) else int(np.argmin(pos))

        overshoot = (peak_pos - target) * direction
        if overshoot > 0:
            metrics.overshoot = overshoot
            metrics.overshoot_percent = (overshoot / abs(step)) * 100 if step != 0 else 0
            metrics.peak_time = t[peak_idx]

        # Settling time (time to stay within tolerance band)
        tolerance = SETTLING_TOLERANCE_DEG
        settled_idx = None

        for i in range(len(pos) - 1, -1, -1):
            if abs(pos[i] - target) > tolerance:
                settled_idx = i + 1
                break

        if settled_idx is not None and settled_idx < len(t):
            metrics.settling_time = t[settled_idx]
        else:
            metrics.settling_time = 0  # Settled immediately or never

        # Steady-state error (average of last few samples)
        steady_window = int(STEADY_STATE_WINDOW * 1000 / SAMPLE_INTERVAL_MS)
        if steady_window > 0 and len(pos) > steady_window:
            final_positions = pos[-steady_window:]
            if HAS_NUMPY:
                avg_final = np.mean(final_positions)
            else:
                avg_final = sum(final_positions) / len(final_positions)
            metrics.steady_state_error = abs(avg_final - target)
        else:
            metrics.steady_state_error = abs(pos[-1] - target) if pos else 0

    def plot_response(self, metrics: StepResponseMetrics, title: str = "Step Response"):
        """Plot step response."""
        if not HAS_MATPLOTLIB:
            print("Matplotlib not installed - cannot plot")
            return

        if not metrics.timestamps:
            print("No data to plot")
            return

        fig, ax = plt.subplots(figsize=(10, 6))

        # Plot position vs time
        ax.plot(metrics.timestamps, metrics.positions, 'b-', linewidth=2, label='Position')

        # Plot target line
        ax.axhline(y=metrics.target, color='g', linestyle='--', label='Target')

        # Plot start line
        ax.axhline(y=metrics.start_position, color='gray', linestyle=':', alpha=0.5)

        # Plot tolerance band
        ax.axhline(y=metrics.target + SETTLING_TOLERANCE_DEG, color='r',
                   linestyle=':', alpha=0.3)
        ax.axhline(y=metrics.target - SETTLING_TOLERANCE_DEG, color='r',
                   linestyle=':', alpha=0.3, label=f'Tolerance (±{SETTLING_TOLERANCE_DEG}°)')

        # Mark key points
        if metrics.rise_time > 0:
            ax.axvline(x=metrics.rise_time, color='orange', linestyle='--',
                      alpha=0.5, label=f'Rise time: {metrics.rise_time:.3f}s')

        if metrics.settling_time > 0:
            ax.axvline(x=metrics.settling_time, color='purple', linestyle='--',
                      alpha=0.5, label=f'Settling: {metrics.settling_time:.3f}s')

        # Labels and legend
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (°)')
        ax.set_title(f'{title}\n{metrics}')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()


#=============================================================================
# FREQUENCY RESPONSE ANALYSIS
#=============================================================================

class FrequencyResponseAnalyzer:
    """Analyze motor frequency response."""

    def __init__(self, motor: MotorInterface):
        self.motor = motor

    def run_frequency_sweep(self,
                           center_pos: float = 0.0,
                           amplitude: float = 10.0,
                           freq_start: float = 0.1,
                           freq_end: float = 5.0,
                           num_points: int = 10,
                           cycles_per_freq: int = 3) -> List[FrequencyResponsePoint]:
        """
        Run frequency response sweep.

        Args:
            center_pos: Center position in degrees
            amplitude: Oscillation amplitude in degrees
            freq_start: Starting frequency in Hz
            freq_end: Ending frequency in Hz
            num_points: Number of frequency points
            cycles_per_freq: Cycles to measure at each frequency

        Returns:
            List of FrequencyResponsePoint
        """
        results = []

        if HAS_NUMPY:
            frequencies = np.logspace(np.log10(freq_start), np.log10(freq_end), num_points)
        else:
            # Simple log spacing without numpy
            log_start = math.log10(freq_start)
            log_end = math.log10(freq_end)
            frequencies = [10 ** (log_start + i * (log_end - log_start) / (num_points - 1))
                          for i in range(num_points)]

        print(f"\nFrequency sweep: {freq_start:.2f} Hz to {freq_end:.2f} Hz")
        print(f"Amplitude: ±{amplitude:.1f}° around {center_pos:.1f}°")

        # Move to center position first
        self.motor.move_to(center_pos)
        time.sleep(1.0)

        for freq in frequencies:
            print(f"  Testing {freq:.2f} Hz...", end=" ", flush=True)

            try:
                point = self._measure_at_frequency(
                    center_pos, amplitude, freq, cycles_per_freq
                )
                results.append(point)
                print(f"Gain: {point.amplitude_ratio:.2f}, Phase: {point.phase_lag_deg:.1f}°")
            except Exception as e:
                print(f"Error: {e}")

        return results

    def _measure_at_frequency(self, center: float, amplitude: float,
                              freq: float, cycles: int) -> FrequencyResponsePoint:
        """Measure response at a single frequency."""
        period = 1.0 / freq
        sample_interval = period / 20  # 20 samples per cycle
        total_time = period * cycles

        # Command sinusoidal motion and record response
        timestamps = []
        commands = []
        responses = []

        start_time = time.time()

        while time.time() - start_time < total_time:
            t = time.time() - start_time

            # Command position
            cmd = center + amplitude * math.sin(2 * math.pi * freq * t)
            self.motor.move_to(cmd)

            # Read actual position
            pos = self.motor.get_position()
            if pos is not None:
                timestamps.append(t)
                commands.append(cmd)
                responses.append(pos)

            time.sleep(sample_interval)

        # Analyze amplitude and phase
        if len(responses) < 10:
            return FrequencyResponsePoint(freq, 0, 0)

        # Find amplitude ratio
        cmd_amplitude = (max(commands) - min(commands)) / 2
        resp_amplitude = (max(responses) - min(responses)) / 2

        if cmd_amplitude > 0:
            amplitude_ratio = resp_amplitude / cmd_amplitude
        else:
            amplitude_ratio = 0

        # Estimate phase lag (simple cross-correlation approach)
        # Find time offset where response best matches command
        phase_lag = self._estimate_phase_lag(timestamps, commands, responses, freq)

        return FrequencyResponsePoint(freq, amplitude_ratio, phase_lag)

    def _estimate_phase_lag(self, timestamps: List[float],
                           commands: List[float],
                           responses: List[float],
                           freq: float) -> float:
        """Estimate phase lag between command and response."""
        if not HAS_NUMPY or len(timestamps) < 10:
            return 0.0

        t = np.array(timestamps)
        cmd = np.array(commands)
        resp = np.array(responses)

        # Normalize signals
        cmd_norm = cmd - np.mean(cmd)
        resp_norm = resp - np.mean(resp)

        # Cross-correlation
        correlation = np.correlate(cmd_norm, resp_norm, mode='full')
        max_corr_idx = np.argmax(correlation)

        # Convert to time lag
        time_lag = (max_corr_idx - len(cmd) + 1) * (t[-1] - t[0]) / len(t)

        # Convert to phase (degrees)
        phase_lag = time_lag * freq * 360.0

        return phase_lag

    def plot_bode(self, results: List[FrequencyResponsePoint], title: str = "Bode Plot"):
        """Plot Bode diagram."""
        if not HAS_MATPLOTLIB:
            print("Matplotlib not installed - cannot plot")
            return

        if not results:
            print("No data to plot")
            return

        freqs = [r.frequency_hz for r in results]
        gains_db = [20 * math.log10(r.amplitude_ratio) if r.amplitude_ratio > 0 else -60
                    for r in results]
        phases = [r.phase_lag_deg for r in results]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # Magnitude plot
        ax1.semilogx(freqs, gains_db, 'b-o', linewidth=2)
        ax1.axhline(y=-3, color='r', linestyle='--', alpha=0.5, label='-3dB bandwidth')
        ax1.set_ylabel('Magnitude (dB)')
        ax1.set_title(f'{title} - Magnitude')
        ax1.grid(True, which='both', alpha=0.3)
        ax1.legend()

        # Phase plot
        ax2.semilogx(freqs, phases, 'g-o', linewidth=2)
        ax2.axhline(y=-180, color='r', linestyle='--', alpha=0.5, label='-180° (instability)')
        ax2.set_xlabel('Frequency (Hz)')
        ax2.set_ylabel('Phase (°)')
        ax2.set_title(f'{title} - Phase')
        ax2.grid(True, which='both', alpha=0.3)
        ax2.legend()

        plt.tight_layout()
        plt.show()


#=============================================================================
# AUTO-TUNING (ZIEGLER-NICHOLS)
#=============================================================================

class ZieglerNicholsAutoTuner:
    """
    Ziegler-Nichols auto-tuning using relay feedback method.

    This finds the ultimate gain (Ku) and ultimate period (Tu) by
    inducing sustained oscillations, then calculates PID parameters.
    """

    def __init__(self, motor: MotorInterface):
        self.motor = motor

    def run_autotune(self,
                    setpoint: float = 0.0,
                    relay_amplitude: float = AUTOTUNE_AMPLITUDE_DEG,
                    hysteresis: float = AUTOTUNE_HYSTERESIS_DEG,
                    num_cycles: int = AUTOTUNE_CYCLES,
                    timeout: float = AUTOTUNE_TIMEOUT) -> Optional[PIDParams]:
        """
        Run Ziegler-Nichols relay feedback auto-tuning.

        Args:
            setpoint: Target position in degrees
            relay_amplitude: Relay output amplitude (degrees offset)
            hysteresis: Hysteresis band for relay switching
            num_cycles: Number of oscillation cycles to measure
            timeout: Maximum time for tuning

        Returns:
            Optimal PIDParams or None if failed
        """
        print(f"\nStarting Ziegler-Nichols auto-tuning...")
        print(f"Setpoint: {setpoint:.1f}°, Relay amplitude: ±{relay_amplitude:.1f}°")

        # Disable existing PID (set very low gains)
        self.motor.set_position_pid(PIDParams(p=0.1, i=0, d=0, ramp=1000))
        time.sleep(0.5)

        # Move to setpoint
        self.motor.move_to(setpoint)
        time.sleep(2.0)

        # Relay feedback control
        crossings = []
        peaks = []
        relay_state = 1  # 1 = positive, -1 = negative
        last_position = self.motor.get_position() or setpoint

        start_time = time.time()
        last_crossing_time = start_time

        print("Inducing oscillations...")

        while time.time() - start_time < timeout:
            current_pos = self.motor.get_position()
            if current_pos is None:
                time.sleep(0.01)
                continue

            error = setpoint - current_pos

            # Relay with hysteresis
            if relay_state == 1 and error < -hysteresis:
                relay_state = -1
                crossings.append(time.time())
                print(f"  Crossing {len(crossings)}: t={time.time() - start_time:.2f}s")

            elif relay_state == -1 and error > hysteresis:
                relay_state = 1
                crossings.append(time.time())
                print(f"  Crossing {len(crossings)}: t={time.time() - start_time:.2f}s")

            # Track peaks for amplitude
            if (relay_state == 1 and current_pos > last_position) or \
               (relay_state == -1 and current_pos < last_position):
                peaks.append(current_pos)

            # Apply relay output
            output = setpoint + relay_state * relay_amplitude
            self.motor.move_to(output)

            last_position = current_pos

            # Check if we have enough cycles
            if len(crossings) >= num_cycles * 2:
                break

            time.sleep(0.01)

        # Analyze results
        if len(crossings) < 4:
            print("Error: Not enough oscillation cycles detected")
            return None

        # Calculate ultimate period (average of half-periods)
        periods = []
        for i in range(2, len(crossings)):
            periods.append(crossings[i] - crossings[i-2])

        Tu = sum(periods) / len(periods)  # Ultimate period

        # Calculate oscillation amplitude
        if peaks:
            oscillation_amplitude = (max(peaks) - min(peaks)) / 2
        else:
            oscillation_amplitude = relay_amplitude

        # Calculate ultimate gain
        # Ku = 4 * d / (pi * a) where d = relay amplitude, a = oscillation amplitude
        Ku = (4 * relay_amplitude) / (math.pi * oscillation_amplitude)

        print(f"\nAuto-tune results:")
        print(f"  Ultimate Period (Tu): {Tu:.3f} s")
        print(f"  Ultimate Gain (Ku): {Ku:.3f}")
        print(f"  Oscillation Amplitude: {oscillation_amplitude:.2f}°")

        # Calculate PID parameters using Ziegler-Nichols rules
        params = self._calculate_zn_params(Ku, Tu)

        return params

    def _calculate_zn_params(self, Ku: float, Tu: float) -> PIDParams:
        """
        Calculate PID parameters from Ku and Tu.

        Uses "some overshoot" tuning rule for better damping.
        """
        # Classic Ziegler-Nichols PID
        # Kp = 0.6 * Ku
        # Ki = 1.2 * Ku / Tu  (or Ti = Tu / 2)
        # Kd = 0.075 * Ku * Tu  (or Td = Tu / 8)

        # Modified for less overshoot (Tyreus-Luyben)
        Kp = 0.45 * Ku
        Ti = 2.2 * Tu
        Td = Tu / 6.3

        Ki = Kp / Ti if Ti > 0 else 0
        Kd = Kp * Td

        params = PIDParams(
            p=Kp,
            i=Ki,
            d=Kd,
            ramp=1000.0
        )

        print(f"\nCalculated PID parameters (Tyreus-Luyben):")
        print(f"  P = {params.p:.3f}")
        print(f"  I = {params.i:.3f}")
        print(f"  D = {params.d:.4f}")

        return params


#=============================================================================
# PARAMETER SWEEP
#=============================================================================

class ParameterSweep:
    """Systematic PID parameter sweep and optimization."""

    def __init__(self, motor: MotorInterface):
        self.motor = motor
        self.step_analyzer = StepResponseAnalyzer(motor)

    def run_sweep(self,
                 p_range: Tuple[float, float, float] = SWEEP_P_RANGE,
                 i_range: Tuple[float, float, float] = SWEEP_I_RANGE,
                 d_range: Tuple[float, float, float] = SWEEP_D_RANGE,
                 step_size: float = STEP_SIZE_DEG) -> List[SweepResult]:
        """
        Run parameter sweep.

        Args:
            p_range: (min, max, step) for P gain
            i_range: (min, max, step) for I gain
            d_range: (min, max, step) for D gain
            step_size: Step size for testing

        Returns:
            List of SweepResult sorted by score
        """
        results = []

        # Generate parameter combinations
        p_values = self._generate_range(*p_range)
        i_values = self._generate_range(*i_range)
        d_values = self._generate_range(*d_range)

        total_tests = len(p_values) * len(i_values) * len(d_values)
        print(f"\nParameter sweep: {total_tests} combinations")
        print(f"  P: {p_range[0]:.1f} to {p_range[1]:.1f} (step {p_range[2]:.1f})")
        print(f"  I: {i_range[0]:.2f} to {i_range[1]:.2f} (step {i_range[2]:.2f})")
        print(f"  D: {d_range[0]:.2f} to {d_range[1]:.2f} (step {d_range[2]:.2f})")

        test_num = 0

        for p in p_values:
            for i in i_values:
                for d in d_values:
                    test_num += 1
                    params = PIDParams(p=p, i=i, d=d)

                    print(f"\n[{test_num}/{total_tests}] Testing {params}")

                    # Apply parameters
                    if not self.motor.set_position_pid(params):
                        print("  Failed to set PID")
                        continue

                    time.sleep(0.2)

                    # Run step test
                    metrics = self.step_analyzer.run_step_test(
                        step_size=step_size,
                        settle_time=2.0,
                        sample_interval_ms=20
                    )

                    # Check for dangerous behavior
                    if metrics.overshoot > 20.0:  # degrees
                        print(f"  Excessive overshoot ({metrics.overshoot:.1f}°), skipping...")
                        continue

                    score = metrics.score()
                    print(f"  Score: {score:.2f} (overshoot: {metrics.overshoot:.2f}°, "
                          f"settling: {metrics.settling_time:.2f}s, "
                          f"error: {metrics.steady_state_error:.3f}°)")

                    results.append(SweepResult(
                        params=params,
                        metrics=metrics,
                        score=score
                    ))

                    # Return to start position
                    self.motor.move_to(metrics.start_position)
                    time.sleep(1.0)

        # Sort by score (lower is better)
        results.sort(key=lambda r: r.score)

        return results

    def _generate_range(self, start: float, end: float, step: float) -> List[float]:
        """Generate range of values."""
        values = []
        v = start
        while v <= end + 0.001:  # Small epsilon for float comparison
            values.append(v)
            v += step
        return values

    def print_results(self, results: List[SweepResult], top_n: int = 5):
        """Print top sweep results."""
        print(f"\n{'='*60}")
        print(f"TOP {min(top_n, len(results))} PARAMETER COMBINATIONS")
        print(f"{'='*60}")

        for i, result in enumerate(results[:top_n]):
            print(f"\n#{i+1}: Score = {result.score:.2f}")
            print(f"   Parameters: {result.params}")
            print(f"   Overshoot: {result.metrics.overshoot:.2f}°")
            print(f"   Settling Time: {result.metrics.settling_time:.2f}s")
            print(f"   Steady-State Error: {result.metrics.steady_state_error:.3f}°")
            print(f"   Rise Time: {result.metrics.rise_time:.3f}s")

        if results:
            best = results[0]
            print(f"\n{'='*60}")
            print(f"RECOMMENDED CONFIGURATION (copy to config.h):")
            print(f"{'='*60}")
            print(f"#define PID_P_POSITION   {best.params.p:.2f}")
            print(f"#define PID_I_POSITION   {best.params.i:.2f}")
            print(f"#define PID_D_POSITION   {best.params.d:.3f}")
            print(f"#define PID_RAMP_POSITION_DEG {best.params.ramp:.1f}")


#=============================================================================
# INTERACTIVE MANUAL TUNING
#=============================================================================

class ManualTuner:
    """Interactive manual PID tuning."""

    def __init__(self, motor: MotorInterface):
        self.motor = motor
        self.step_analyzer = StepResponseAnalyzer(motor)
        self.current_params = PIDParams()

    def run(self):
        """Run interactive tuning session."""
        print("\n" + "="*60)
        print("INTERACTIVE PID TUNING")
        print("="*60)
        print("\nCommands:")
        print("  p <value>  - Set P gain")
        print("  i <value>  - Set I gain")
        print("  d <value>  - Set D gain")
        print("  r <value>  - Set ramp (deg/s)")
        print("  step [size]- Run step test (default 30°)")
        print("  plot       - Plot last step response")
        print("  apply      - Apply current parameters to motor")
        print("  show       - Show current parameters")
        print("  save       - Print config.h values")
        print("  q          - Quit")
        print()

        last_metrics = None

        while True:
            try:
                cmd = input(f"[P={self.current_params.p:.1f} I={self.current_params.i:.2f} "
                           f"D={self.current_params.d:.3f}] > ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if not cmd:
                continue

            parts = cmd.split()
            command = parts[0]

            try:
                if command == 'q' or command == 'quit':
                    break

                elif command == 'p' and len(parts) > 1:
                    self.current_params.p = float(parts[1])
                    print(f"P set to {self.current_params.p:.2f}")

                elif command == 'i' and len(parts) > 1:
                    self.current_params.i = float(parts[1])
                    print(f"I set to {self.current_params.i:.3f}")

                elif command == 'd' and len(parts) > 1:
                    self.current_params.d = float(parts[1])
                    print(f"D set to {self.current_params.d:.4f}")

                elif command == 'r' and len(parts) > 1:
                    self.current_params.ramp = float(parts[1])
                    print(f"Ramp set to {self.current_params.ramp:.1f}°/s")

                elif command == 'step':
                    step_size = float(parts[1]) if len(parts) > 1 else STEP_SIZE_DEG
                    self.motor.set_position_pid(self.current_params)
                    time.sleep(0.1)
                    last_metrics = self.step_analyzer.run_step_test(step_size=step_size)
                    print(last_metrics)

                elif command == 'plot':
                    if last_metrics:
                        self.step_analyzer.plot_response(last_metrics)
                    else:
                        print("No step response data. Run 'step' first.")

                elif command == 'apply':
                    self.motor.set_position_pid(self.current_params)
                    print(f"Applied: {self.current_params}")

                elif command == 'show':
                    print(f"Current: {self.current_params}")

                elif command == 'save':
                    print("\n// Copy to config.h:")
                    print(f"#define PID_P_POSITION   {self.current_params.p:.2f}")
                    print(f"#define PID_I_POSITION   {self.current_params.i:.2f}")
                    print(f"#define PID_D_POSITION   {self.current_params.d:.3f}")
                    print(f"#define PID_RAMP_POSITION_DEG {self.current_params.ramp:.1f}")
                    print()

                else:
                    print(f"Unknown command: {command}")

            except ValueError as e:
                print(f"Invalid value: {e}")
            except Exception as e:
                print(f"Error: {e}")


#=============================================================================
# MAIN ENTRY POINT
#=============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="PID Tuning for OpenHyperspectral Motor Control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python pid_tuner.py /dev/ttyUSB0 --mode step --plot
  python pid_tuner.py COM3 --mode autotune
  python pid_tuner.py /dev/ttyACM0 --mode sweep --plot
  python pid_tuner.py /dev/ttyUSB0 --mode manual
        """
    )

    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('--mode', '-m',
                       choices=['step', 'freq', 'autotune', 'sweep', 'manual'],
                       default='step',
                       help='Tuning mode (default: step)')
    parser.add_argument('--plot', '-p', action='store_true',
                       help='Show plots')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Verbose output')
    parser.add_argument('--step-size', type=float, default=STEP_SIZE_DEG,
                       help=f'Step size in degrees (default: {STEP_SIZE_DEG})')
    parser.add_argument('--calibrate', '-c', action='store_true',
                       help='Run calibration before tuning')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD_RATE,
                       help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')

    args = parser.parse_args()

    if not HAS_SERIAL:
        print("Error: pyserial not installed. Run: pip install pyserial")
        sys.exit(1)

    # Connect to motor
    motor = MotorInterface(args.port, baud_rate=args.baud)
    motor.verbose = args.verbose

    if not motor.connect():
        print("Failed to connect to motor controller")
        sys.exit(1)

    try:
        # Calibrate if requested
        if args.calibrate:
            if not motor.calibrate():
                print("Calibration failed")
                sys.exit(1)
            time.sleep(1.0)

        # Enable motor
        motor.enable()
        time.sleep(0.5)

        # Run selected mode
        if args.mode == 'step':
            analyzer = StepResponseAnalyzer(motor)
            metrics = analyzer.run_step_test(step_size=args.step_size)
            print(metrics)

            if args.plot and HAS_MATPLOTLIB:
                analyzer.plot_response(metrics)

        elif args.mode == 'freq':
            analyzer = FrequencyResponseAnalyzer(motor)
            results = analyzer.run_frequency_sweep()

            if args.plot and HAS_MATPLOTLIB:
                analyzer.plot_bode(results)

        elif args.mode == 'autotune':
            tuner = ZieglerNicholsAutoTuner(motor)
            params = tuner.run_autotune()

            if params:
                print("\nApplying tuned parameters...")
                motor.set_position_pid(params)

                # Verify with step test
                print("\nVerification step test:")
                analyzer = StepResponseAnalyzer(motor)
                metrics = analyzer.run_step_test(step_size=args.step_size)
                print(metrics)

                if args.plot and HAS_MATPLOTLIB:
                    analyzer.plot_response(metrics, "Auto-tuned Step Response")

        elif args.mode == 'sweep':
            sweep = ParameterSweep(motor)
            results = sweep.run_sweep()
            sweep.print_results(results)

            if args.plot and HAS_MATPLOTLIB and results:
                # Plot best result
                analyzer = StepResponseAnalyzer(motor)
                motor.set_position_pid(results[0].params)
                metrics = analyzer.run_step_test(step_size=args.step_size)
                analyzer.plot_response(metrics, "Best Parameters Step Response")

        elif args.mode == 'manual':
            tuner = ManualTuner(motor)
            tuner.run()

    except KeyboardInterrupt:
        print("\nInterrupted")

    finally:
        motor.disable()
        motor.disconnect()


if __name__ == '__main__':
    main()
