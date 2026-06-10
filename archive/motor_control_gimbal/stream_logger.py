#!/usr/bin/env python3
"""
Encoder Stream Logger for OpenHyperspectral

Reads serial from ESP32, separates encoder data ($ENC, lines) from debug text.
Logs encoder data to CSV and optionally plots in real-time.

Usage:
    python stream_logger.py /dev/ttyUSB0              # Log to CSV + show debug
    python stream_logger.py /dev/ttyUSB0 --plot        # Add real-time plot
    python stream_logger.py /dev/ttyUSB0 --quiet       # Suppress debug in terminal
    python stream_logger.py /dev/ttyUSB0 -o scan1.csv  # Custom output file

The ESP32 firmware must have streaming enabled:
    Type 'stream on' in serial monitor, or send the command before running this.

Protocol:
    $ENC,<timestamp_ms>,<position_deg>,<velocity_deg_s>,<target_deg>
    $SCAN_START,<timestamp_ms>
    $SCAN_END,<timestamp_ms>
    $STREAM_ON,<timestamp_ms>
    $STREAM_OFF,<timestamp_ms>
    (everything else is debug text)
"""

import argparse
import csv
import os
import signal
import sys
import time
from datetime import datetime

import serial

# CSV column headers
CSV_HEADERS = ["timestamp_ms", "position_deg", "velocity_deg_s", "target_deg", "pc_time"]

# Tag prefixes
TAG_ENC = "$ENC,"
TAG_SCAN_START = "$SCAN_START,"
TAG_SCAN_END = "$SCAN_END,"
TAG_STREAM_ON = "$STREAM_ON,"
TAG_STREAM_OFF = "$STREAM_OFF,"


class StreamLogger:
    """Reads serial, filters encoder data, logs to CSV, optionally plots."""

    def __init__(self, port, baud=115200, output=None, quiet=False):
        self.port = port
        self.baud = baud
        self.quiet = quiet
        self.running = False

        # Output file
        if output is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output = f"encoder_log_{timestamp}.csv"
        self.output_path = output

        # Stats
        self.enc_count = 0
        self.scan_active = False
        self.start_time = None

    def connect(self):
        """Open serial connection."""
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        # Flush any stale data
        self.ser.reset_input_buffer()

    def send_command(self, cmd):
        """Send a text command to the ESP32 (e.g., 'stream on')."""
        self.ser.write((cmd + "\n").encode())

    def run(self, enable_stream=True, plot=False):
        """
        Main loop: read serial, filter, log, and optionally plot.

        Args:
            enable_stream: Send 'stream on' command on start.
            plot: Enable real-time matplotlib plot.
        """
        self.running = True
        self.start_time = time.time()

        # Set up signal handler for clean exit
        signal.signal(signal.SIGINT, self._signal_handler)

        self.connect()
        print(f"Connected to {self.port} @ {self.baud} baud")
        print(f"Logging encoder data to: {self.output_path}")
        print("Press Ctrl+C to stop\n")

        # Enable streaming on ESP32
        if enable_stream:
            time.sleep(0.5)  # Let serial settle
            self.send_command("stream on")

        # Open CSV file
        csv_file = open(self.output_path, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(CSV_HEADERS)

        # Optional plot setup
        plot_data = None
        if plot:
            plot_data = self._setup_plot()

        try:
            while self.running:
                line = self._read_line()
                if line is None:
                    continue

                if line.startswith(TAG_ENC):
                    self._handle_enc_line(line, writer, plot_data)
                elif line.startswith(TAG_SCAN_START):
                    self.scan_active = True
                    ts = line[len(TAG_SCAN_START):]
                    if not self.quiet:
                        print(f"[SCAN START] t={ts}ms")
                elif line.startswith(TAG_SCAN_END):
                    self.scan_active = False
                    ts = line[len(TAG_SCAN_END):]
                    if not self.quiet:
                        print(f"[SCAN END] t={ts}ms")
                elif line.startswith("$"):
                    # Other tagged lines (STREAM_ON, STREAM_OFF, etc.)
                    if not self.quiet:
                        print(f"[TAG] {line}")
                else:
                    # Debug text -- pass through to terminal
                    if not self.quiet:
                        print(line)

                # Update plot if active
                if plot_data is not None:
                    self._update_plot(plot_data)

        finally:
            # Clean shutdown
            if enable_stream:
                try:
                    self.send_command("stream off")
                    time.sleep(0.2)
                except Exception:
                    pass

            csv_file.close()
            self.ser.close()

            elapsed = time.time() - self.start_time
            rate = self.enc_count / elapsed if elapsed > 0 else 0
            print(f"\nLogged {self.enc_count} samples in {elapsed:.1f}s ({rate:.1f} samples/s)")
            print(f"Saved to: {self.output_path}")

    def _read_line(self):
        """Read one line from serial, return stripped string or None."""
        try:
            raw = self.ser.readline()
            if not raw:
                return None
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                return None
            return line
        except serial.SerialException:
            print("\nSerial connection lost!")
            self.running = False
            return None

    def _handle_enc_line(self, line, writer, plot_data):
        """Parse $ENC line, write to CSV, update plot data."""
        try:
            # $ENC,timestamp_ms,position_deg,velocity_deg_s,target_deg
            parts = line[len(TAG_ENC):].split(",")
            if len(parts) < 4:
                return

            timestamp_ms = int(parts[0])
            position_deg = float(parts[1])
            velocity_deg_s = float(parts[2])
            target_deg = float(parts[3])
            pc_time = time.time()

            # Write to CSV
            writer.writerow([timestamp_ms, position_deg, velocity_deg_s, target_deg, pc_time])
            self.enc_count += 1

            # Periodic flush (every 100 samples)
            if self.enc_count % 100 == 0:
                try:
                    writer.writerow  # trigger any buffered writes
                except Exception:
                    pass

            # Update plot buffer
            if plot_data is not None:
                plot_data["timestamps"].append(timestamp_ms / 1000.0)  # seconds
                plot_data["positions"].append(position_deg)
                plot_data["velocities"].append(velocity_deg_s)
                plot_data["targets"].append(target_deg)

                # Keep last N seconds of data in plot
                max_points = plot_data["max_points"]
                if len(plot_data["timestamps"]) > max_points:
                    plot_data["timestamps"] = plot_data["timestamps"][-max_points:]
                    plot_data["positions"] = plot_data["positions"][-max_points:]
                    plot_data["velocities"] = plot_data["velocities"][-max_points:]
                    plot_data["targets"] = plot_data["targets"][-max_points:]

        except (ValueError, IndexError) as e:
            if not self.quiet:
                print(f"[WARN] Bad $ENC line: {line} ({e})")

    def _setup_plot(self):
        """Initialize matplotlib for real-time plotting."""
        try:
            import matplotlib.pyplot as plt
            import matplotlib.animation  # noqa: F401 - needed for interactive mode

            plt.ion()
            fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

            ax_pos.set_ylabel("Position (deg)")
            ax_pos.set_title("Encoder Position Stream")
            ax_pos.grid(True, alpha=0.3)

            ax_vel.set_ylabel("Velocity (deg/s)")
            ax_vel.set_xlabel("Time (s)")
            ax_vel.grid(True, alpha=0.3)

            fig.tight_layout()

            return {
                "fig": fig,
                "ax_pos": ax_pos,
                "ax_vel": ax_vel,
                "timestamps": [],
                "positions": [],
                "velocities": [],
                "targets": [],
                "max_points": 1000,  # ~10s at 100Hz
                "last_draw": 0,
                "plt": plt,
            }

        except ImportError:
            print("[WARN] matplotlib not available, plotting disabled")
            return None

    def _update_plot(self, pd):
        """Redraw plot at ~10 FPS (not every sample)."""
        now = time.time()
        if now - pd["last_draw"] < 0.1:  # 10 FPS
            return
        pd["last_draw"] = now

        if len(pd["timestamps"]) < 2:
            return

        plt = pd["plt"]

        pd["ax_pos"].cla()
        pd["ax_pos"].plot(pd["timestamps"], pd["positions"], "b-", linewidth=0.8, label="Position")
        pd["ax_pos"].plot(pd["timestamps"], pd["targets"], "r--", linewidth=0.8, label="Target")
        pd["ax_pos"].set_ylabel("Position (deg)")
        pd["ax_pos"].legend(loc="upper right", fontsize=8)
        pd["ax_pos"].grid(True, alpha=0.3)

        pd["ax_vel"].cla()
        pd["ax_vel"].plot(pd["timestamps"], pd["velocities"], "g-", linewidth=0.8)
        pd["ax_vel"].set_ylabel("Velocity (deg/s)")
        pd["ax_vel"].set_xlabel("Time (s)")
        pd["ax_vel"].grid(True, alpha=0.3)

        try:
            pd["fig"].canvas.draw_idle()
            pd["fig"].canvas.flush_events()
        except Exception:
            pass

    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully."""
        print("\nStopping...")
        self.running = False


def main():
    parser = argparse.ArgumentParser(
        description="Encoder stream logger for OpenHyperspectral ESP32"
    )
    parser.add_argument("port", help="Serial port (e.g., /dev/ttyUSB0, COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("-o", "--output", help="Output CSV file path")
    parser.add_argument("--quiet", action="store_true", help="Suppress debug text in terminal")
    parser.add_argument("--plot", action="store_true", help="Enable real-time matplotlib plot")
    parser.add_argument(
        "--no-enable",
        action="store_true",
        help="Don't send 'stream on' (assume already enabled)",
    )

    args = parser.parse_args()

    logger = StreamLogger(
        port=args.port,
        baud=args.baud,
        output=args.output,
        quiet=args.quiet,
    )
    logger.run(enable_stream=not args.no_enable, plot=args.plot)


if __name__ == "__main__":
    main()
