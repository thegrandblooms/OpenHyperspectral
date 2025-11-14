#!/usr/bin/env python3
"""
OpenHyperspectral Motor Controller

Python interface for controlling BLDC motor via ESP32-S3 with SimpleFOC.
Based on HyperMicro serial communication protocol.

Version: 1.0.0
"""

import struct
import time
import logging
import threading
import queue
from enum import Enum
from typing import Tuple, Dict, Callable, Optional
import sys
import atexit

#=============================================================================
# CONFIGURATION PARAMETERS
#=============================================================================

DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT = 1.0
DEFAULT_RETRY_COUNT = 3
DEFAULT_RETRY_DELAY = 0.2

ESP32_RESET_DELAY = 2.0

MONITOR_SLEEP_INTERVAL = 0.01
MONITOR_ERROR_DELAY = 0.1

POSITION_QUEUE_TIMEOUT = 0.5

DEFAULT_LOG_LEVEL = logging.INFO
DEFAULT_LOG_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'

# Command IDs (match firmware definitions)
CMD_MOVE_TO = 0x01
CMD_SET_SPEED = 0x03
CMD_SET_ACCEL = 0x04
CMD_STOP = 0x05
CMD_HOME = 0x06
CMD_ENABLE = 0x07
CMD_DISABLE = 0x08
CMD_GET_STATUS = 0x09
CMD_PING = 0x0A
CMD_SET_MODE = 0x0B
CMD_SET_CURRENT_LIMIT = 0x0C
CMD_CALIBRATE = 0x0D
CMD_SET_PID = 0x0E

# Response IDs
RESP_OK = 0x81
RESP_ERROR = 0x82
RESP_POSITION_REACHED = 0x84
RESP_PING = 0x86
RESP_STATUS = 0x87

# Import pySerialTransfer
try:
    from pySerialTransfer import pySerialTransfer as txfer
except ImportError:
    sys.stderr.write("Error: pySerialTransfer library not found.\n")
    sys.stderr.write("Please install it with: pip install pySerialTransfer\n")
    raise

# Configure logging
logging.basicConfig(
    level=DEFAULT_LOG_LEVEL,
    format=DEFAULT_LOG_FORMAT
)
logger = logging.getLogger("MotorController")

#=============================================================================
# CONTROL MODE ENUMERATION
#=============================================================================

class ControlMode(Enum):
    """Motor control modes."""
    POSITION = 0
    VELOCITY = 1
    TORQUE = 2

#=============================================================================
# MOTOR CONTROLLER CLASS
#=============================================================================

class MotorController:
    """
    Controller for OpenHyperspectral BLDC motor via ESP32-S3.

    This implementation uses SimpleFOC with position control in radians,
    closely matching the HyperMicro approach but adapted for BLDC motors.
    """

    def __init__(self, port, baud_rate=DEFAULT_BAUD_RATE, timeout=DEFAULT_TIMEOUT, verbose=False):
        """
        Initialize the motor controller.

        Args:
            port: Serial port (e.g., 'COM3', '/dev/ttyUSB0')
            baud_rate: Baud rate for serial communication
            timeout: Command timeout in seconds
            verbose: Enable verbose logging
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.verbose = verbose

        # Set up logging level
        if verbose:
            logger.setLevel(logging.DEBUG)

        logger.info(f"MotorController v1.0.0 initialized for port {port}")

        # Communication objects
        self.link = None
        self.connected = False
        self.lock = threading.Lock()
        self.next_sequence_id = 1

        # Position monitoring
        self.position_callback = None
        self.position_thread = None
        self.position_thread_running = False
        self.position_queue = queue.Queue()

        # Statistics
        self.stats = {
            'commands_sent': 0,
            'commands_failed': 0,
            'positions_received': 0,
        }

        # Register shutdown handler
        atexit.register(self.safe_shutdown)

    def connect(self) -> bool:
        """
        Connect to the ESP32 motor controller.

        Returns:
            Success status
        """
        logger.info(f"Connecting to motor controller on {self.port}...")

        try:
            # Create SerialTransfer object
            logger.debug("Creating SerialTransfer object...")
            self.link = txfer.SerialTransfer(self.port)

            # Open connection
            logger.debug("Opening connection...")
            self.link.open()
            logger.debug("Connection opened successfully")

            # Set baud rate
            if hasattr(self.link, 'connection') and hasattr(self.link.connection, 'baudrate'):
                logger.debug(f"Current baudrate: {self.link.connection.baudrate}")
                if self.link.connection.baudrate != self.baud_rate:
                    self.link.connection.baudrate = self.baud_rate
                    logger.debug(f"New baudrate: {self.link.connection.baudrate}")

            # Wait for ESP32 to reset
            logger.debug(f"Waiting for ESP32 to reset ({ESP32_RESET_DELAY} seconds)...")
            time.sleep(ESP32_RESET_DELAY)

            # Flush buffers
            if hasattr(self.link, 'connection'):
                if hasattr(self.link.connection, 'reset_input_buffer'):
                    logger.debug("Flushing input/output buffers...")
                    self.link.connection.reset_input_buffer()
                    self.link.connection.reset_output_buffer()

            # Test connection with ping
            logger.debug("Testing connection with ping...")
            ping_value = 0x12345678
            echo_value = self.ping(ping_value)

            if echo_value == ping_value:
                self.connected = True
                logger.info("Connected successfully!")

                # Start position monitoring thread
                self._start_position_monitor()

                return True
            else:
                logger.error(f"Ping test failed: expected {ping_value:08X}, got {echo_value}")
                self.link.close()
                self.link = None
                return False

        except Exception as e:
            logger.error(f"Connection failed: {e}")
            import traceback
            logger.debug(traceback.format_exc())

            if self.link:
                try:
                    self.link.close()
                except:
                    pass
                self.link = None

            return False

    def disconnect(self):
        """Disconnect from the motor controller."""
        self.safe_shutdown()

        # Stop position monitor
        self._stop_position_monitor()

        # Close connection
        if self.link:
            try:
                self.link.close()
                logger.info("Disconnected from motor controller")
            except Exception as e:
                logger.error(f"Error during disconnect: {e}")

            self.link = None

        self.connected = False

    def safe_shutdown(self):
        """Safely shut down the controller, stopping and disabling motor."""
        if self.connected:
            try:
                self.stop()
                self.disable()
            except Exception as e:
                logger.error(f"Error during shutdown: {e}")

    def _start_position_monitor(self):
        """Start the position monitoring thread."""
        if self.position_thread is not None and self.position_thread.is_alive():
            return

        self.position_thread_running = True
        self.position_thread = threading.Thread(
            target=self._position_monitor_worker,
            daemon=True
        )
        self.position_thread.start()
        logger.debug("Position monitoring thread started")

    def _stop_position_monitor(self):
        """Stop the position monitoring thread."""
        if self.position_thread is not None:
            self.position_thread_running = False
            if self.position_thread.is_alive():
                self.position_thread.join(timeout=2.0)
            self.position_thread = None
            logger.debug("Position monitoring thread stopped")

    def _position_monitor_worker(self):
        """Thread that monitors for position notifications."""
        logger.debug("Position monitor started")

        while self.position_thread_running:
            try:
                if not self.connected or not self.link:
                    time.sleep(MONITOR_ERROR_DELAY)
                    continue

                # Check for data
                with self.lock:
                    try:
                        available = self.link.available()
                    except:
                        time.sleep(MONITOR_ERROR_DELAY)
                        continue

                    if available <= 0:
                        time.sleep(MONITOR_SLEEP_INTERVAL)
                        continue

                    raw_data = bytes(self.link.rx_buff[:available])

                # Check for position notification
                if len(raw_data) >= 1 and raw_data[0] == RESP_POSITION_REACHED:
                    if len(raw_data) >= 11:
                        sequence_id = struct.unpack("<H", raw_data[1:3])[0]
                        position = struct.unpack("<f", raw_data[3:7])[0]
                        timestamp = struct.unpack("<I", raw_data[7:11])[0]

                        logger.debug(f"Position notification: seq={sequence_id}, pos={position:.4f}")

                        # Add to position queue
                        self.position_queue.put((sequence_id, position, timestamp))
                        self.stats['positions_received'] += 1

                        # Call callback if registered
                        if self.position_callback:
                            try:
                                point_data = {
                                    'sequence_id': sequence_id,
                                    'position': position,
                                    'timestamp': timestamp / 1000.0  # Convert to seconds
                                }
                                self.position_callback(position, sequence_id, point_data)
                            except Exception as e:
                                logger.error(f"Error in position callback: {e}")

            except Exception as e:
                logger.error(f"Error in position monitor: {e}")
                time.sleep(MONITOR_ERROR_DELAY)

        logger.debug("Position monitor stopped")

    def send_command(self, command_id, data=b'', expected_response_id=None, retry_count=0):
        """
        Send a command to the motor controller and wait for response.

        Args:
            command_id: Command ID byte
            data: Additional data bytes
            expected_response_id: Expected response ID or None
            retry_count: Current retry count

        Returns:
            Response data or None if failed
        """
        if not self.connected or not self.link:
            logger.error("Not connected to motor controller")
            return None

        try:
            # Prepare command data
            full_command = bytes([command_id]) + data

            # Debug log command
            cmd_hex = ' '.join(f'{b:02X}' for b in full_command)
            logger.debug(f"Sending command: {cmd_hex}")

            # Send command
            with self.lock:
                # Clear and copy to tx buffer
                buffer_size = min(len(full_command), len(self.link.tx_buff))
                for i in range(buffer_size):
                    self.link.tx_buff[i] = 0

                for i in range(len(full_command)):
                    if i < len(self.link.tx_buff):
                        self.link.tx_buff[i] = full_command[i]

                send_result = self.link.send(len(full_command))

            if not send_result:
                logger.warning(f"Failed to send command 0x{command_id:02X}")
                if retry_count < DEFAULT_RETRY_COUNT:
                    time.sleep(DEFAULT_RETRY_DELAY)
                    return self.send_command(command_id, data, expected_response_id, retry_count + 1)
                return None

            self.stats['commands_sent'] += 1

            # Wait for response
            start_time = time.time()

            while time.time() - start_time < self.timeout:
                with self.lock:
                    available = self.link.available()

                    if available <= 0:
                        time.sleep(MONITOR_SLEEP_INTERVAL)
                        continue

                    raw_response = bytes(self.link.rx_buff[:available])

                if len(raw_response) < 1:
                    time.sleep(MONITOR_SLEEP_INTERVAL)
                    continue

                # Debug log response
                resp_hex = ' '.join(f'{b:02X}' for b in raw_response)
                logger.debug(f"Response received: {resp_hex}")

                resp_id = raw_response[0]

                # Skip position notifications
                if resp_id == RESP_POSITION_REACHED:
                    time.sleep(MONITOR_SLEEP_INTERVAL)
                    continue

                # Check if expected response
                if expected_response_id is None or resp_id == expected_response_id:
                    return raw_response

                logger.warning(f"Unexpected response: 0x{resp_id:02X}, expected: 0x{expected_response_id:02X}")

            # Timeout
            logger.warning(f"Command 0x{command_id:02X} timed out")
            self.stats['commands_failed'] += 1

            if retry_count < DEFAULT_RETRY_COUNT:
                logger.debug(f"Retrying command (attempt {retry_count + 1})")
                return self.send_command(command_id, data, expected_response_id, retry_count + 1)

            return None

        except Exception as e:
            logger.error(f"Error sending command: {e}")
            import traceback
            logger.debug(traceback.format_exc())
            self.stats['commands_failed'] += 1
            return None

    def set_position_callback(self, callback):
        """
        Set the callback function for position notifications.

        Args:
            callback: Function to call when position is reached
                     Signature: callback(position, sequence_id, point_data)
        """
        self.position_callback = callback

    def ping(self, value=0x12345678):
        """
        Send a ping command to verify connection.

        Args:
            value: Test value to send (will be echoed back)

        Returns:
            Echoed value or None if failed
        """
        ping_command = bytes([CMD_PING]) + struct.pack("<I", value)

        logger.debug(f"Sending ping with value: 0x{value:08X}")

        with self.link:
            for i in range(len(ping_command)):
                self.link.tx_buff[i] = ping_command[i]

            send_result = self.link.send(len(ping_command))

        if not send_result:
            logger.warning("Ping command send failed")
            return None

        # Wait for response
        start_time = time.time()
        ping_timeout = self.timeout * 2

        while time.time() - start_time < ping_timeout:
            with self.lock:
                available = self.link.available()

                if available <= 0:
                    time.sleep(MONITOR_SLEEP_INTERVAL)
                    continue

                raw_response = bytes(self.link.rx_buff[:available])

            if len(raw_response) >= 5 and raw_response[0] == RESP_PING:
                echo_value = struct.unpack("<I", raw_response[1:5])[0]
                logger.debug(f"Echo value: 0x{echo_value:08X}")
                return echo_value

        logger.warning("Ping command timed out")
        return None

    def get_status(self):
        """
        Get the current status of the motor controller.

        Returns:
            Dictionary with status information or None if failed
        """
        response = self.send_command(CMD_GET_STATUS, b'', RESP_STATUS)

        if not response or len(response) < 23:
            logger.warning("Failed to get status")
            return None

        # Parse status response
        state = response[1]
        control_mode = response[2]
        position = struct.unpack("<f", response[3:7])[0]
        velocity = struct.unpack("<f", response[7:11])[0]
        current = struct.unpack("<f", response[11:15])[0]
        voltage = struct.unpack("<f", response[15:19])[0]
        motor_enabled = response[19] > 0
        calibrated = response[20] > 0

        status = {
            'state': state,
            'control_mode': ControlMode(control_mode),
            'position': position,
            'velocity': velocity,
            'current': current,
            'voltage': voltage,
            'motor_enabled': motor_enabled,
            'calibrated': calibrated,
            'connected': self.connected,
            'timestamp': time.time()
        }

        return status

    def move_to(self, position_rad, sequence_id=None):
        """
        Move to a specified position in radians.

        Args:
            position_rad: Target position in radians
            sequence_id: Optional sequence ID for tracking

        Returns:
            Success status
        """
        if sequence_id is None:
            sequence_id = self._get_next_sequence_id()

        data = struct.pack("<fH", position_rad, sequence_id)
        response = self.send_command(CMD_MOVE_TO, data, RESP_OK)

        return response is not None and response[0] == RESP_OK

    def queue_movement(self, position_rad, point_data=None):
        """
        Queue a movement to a specified position.

        Args:
            position_rad: Target position in radians
            point_data: Optional data to associate with this movement

        Returns:
            Sequence ID of the queued movement, or 0 if failed
        """
        sequence_id = self._get_next_sequence_id()

        if self.move_to(position_rad, sequence_id):
            return sequence_id
        else:
            return 0

    def wait_for_position(self, sequence_id, timeout=10.0):
        """
        Wait for a position notification for a specific sequence ID.

        Args:
            sequence_id: Sequence ID to wait for
            timeout: Maximum time to wait in seconds

        Returns:
            Position in radians if reached, None if timeout
        """
        logger.debug(f"Waiting for position notification (sequence ID: {sequence_id})")

        start_time = time.time()

        while time.time() - start_time < timeout:
            try:
                seq, pos, timestamp = self.position_queue.get(timeout=POSITION_QUEUE_TIMEOUT)

                if seq == sequence_id:
                    logger.debug(f"Position reached: {pos:.4f} rad for sequence ID {sequence_id}")
                    return pos
                else:
                    logger.debug(f"Received position for different sequence ID: {seq}")
            except queue.Empty:
                pass

        logger.warning(f"Timeout waiting for position notification (sequence ID: {sequence_id})")
        return None

    def set_velocity(self, velocity_rad_s):
        """
        Set motor velocity in rad/s.

        Args:
            velocity_rad_s: Target velocity in rad/s

        Returns:
            Success status
        """
        data = struct.pack("<f", velocity_rad_s)
        response = self.send_command(CMD_SET_SPEED, data, RESP_OK)

        return response is not None and response[0] == RESP_OK

    def set_acceleration(self, accel_rad_s2):
        """
        Set motor acceleration in rad/s².

        Args:
            accel_rad_s2: Target acceleration in rad/s²

        Returns:
            Success status
        """
        data = struct.pack("<f", accel_rad_s2)
        response = self.send_command(CMD_SET_ACCEL, data, RESP_OK)

        return response is not None and response[0] == RESP_OK

    def stop(self):
        """
        Emergency stop - stop motor immediately.

        Returns:
            Success status
        """
        response = self.send_command(CMD_STOP, b'', RESP_OK)
        return response is not None and response[0] == RESP_OK

    def home(self):
        """
        Set current position as home (zero).

        Returns:
            Success status
        """
        response = self.send_command(CMD_HOME, b'', RESP_OK)
        return response is not None and response[0] == RESP_OK

    def enable(self):
        """
        Enable motor.

        Returns:
            Success status
        """
        response = self.send_command(CMD_ENABLE, b'', RESP_OK)
        return response is not None and response[0] == RESP_OK

    def disable(self):
        """
        Disable motor.

        Returns:
            Success status
        """
        response = self.send_command(CMD_DISABLE, b'', RESP_OK)
        return response is not None and response[0] == RESP_OK

    def set_control_mode(self, mode: ControlMode):
        """
        Set the motor control mode.

        Args:
            mode: Control mode (POSITION, VELOCITY, or TORQUE)

        Returns:
            Success status
        """
        data = bytes([mode.value])
        response = self.send_command(CMD_SET_MODE, data, RESP_OK)
        return response is not None and response[0] == RESP_OK

    def set_current_limit(self, current_limit_a):
        """
        Set maximum current limit in amps.

        Args:
            current_limit_a: Maximum current in amps

        Returns:
            Success status
        """
        data = struct.pack("<f", current_limit_a)
        response = self.send_command(CMD_SET_CURRENT_LIMIT, data, RESP_OK)
        return response is not None and response[0] == RESP_OK

    def calibrate(self):
        """
        Run motor calibration.

        Returns:
            Success status
        """
        logger.info("Starting motor calibration...")
        response = self.send_command(CMD_CALIBRATE, b'', RESP_OK)

        if response is not None and response[0] == RESP_OK:
            logger.info("Calibration successful")
            return True
        else:
            logger.error("Calibration failed")
            return False

    def set_pid(self, controller_type, p, i, d, ramp):
        """
        Set PID parameters for a control loop.

        Args:
            controller_type: 0=position, 1=velocity, 2=current
            p: Proportional gain
            i: Integral gain
            d: Derivative gain
            ramp: Output ramp limit

        Returns:
            Success status
        """
        data = struct.pack("<Bffff", controller_type, p, i, d, ramp)
        response = self.send_command(CMD_SET_PID, data, RESP_OK)
        return response is not None and response[0] == RESP_OK

    def _get_next_sequence_id(self):
        """Get the next sequence ID."""
        sequence_id = self.next_sequence_id
        self.next_sequence_id = (self.next_sequence_id + 1) & 0xFFFF

        if self.next_sequence_id == 0:
            self.next_sequence_id = 1

        return sequence_id

    def get_stats(self):
        """Get communication statistics."""
        return self.stats.copy()


#=============================================================================
# TEST FUNCTION
#=============================================================================

def test_controller(port, verbose=False):
    """Run a comprehensive test of the motor controller."""
    print(f"\n==== TESTING MOTOR CONTROLLER ON {port} ====")

    controller = MotorController(port, verbose=verbose)

    try:
        # Test 1: Connect
        print("\nConnecting to motor controller...")
        if not controller.connect():
            print("❌ CONNECTION FAILED")
            return False

        print("✅ CONNECTION SUCCESSFUL")

        # Test 2: Ping
        print("\nTesting ping command...")
        echo_value = controller.ping(0x87654321)

        if echo_value == 0x87654321:
            print(f"✅ PING SUCCESSFUL - Echo value: 0x{echo_value:08X}")
        else:
            print(f"❌ PING FAILED - Echo value: {echo_value}")
            return False

        # Test 3: Status
        print("\nTesting status command...")
        status = controller.get_status()

        if status:
            print("✅ STATUS SUCCESSFUL")
            print("Status values:")
            for key, value in status.items():
                print(f"  {key}: {value}")
        else:
            print("❌ STATUS FAILED")
            return False

        # Test 4: Calibration (optional)
        calibrated = status['calibrated']
        if not calibrated:
            print("\nMotor not calibrated. Run calibration? (y/n): ", end="")
            if input().lower() == 'y':
                if controller.calibrate():
                    print("✅ CALIBRATION SUCCESSFUL")
                else:
                    print("❌ CALIBRATION FAILED")
                    return False

        # Test 5: Enable motor
        print("\nEnabling motor...")
        if controller.enable():
            print("✅ MOTOR ENABLED")
        else:
            print("❌ MOTOR ENABLE FAILED")
            return False

        # Test 6: Move to position
        print("\nTesting move to position (1.0 rad)...")
        sequence_id = controller.queue_movement(1.0)

        if sequence_id > 0:
            print(f"Move command sent with sequence ID: {sequence_id}")

            position = controller.wait_for_position(sequence_id, timeout=10.0)

            if position is not None:
                print(f"✅ MOVE SUCCESSFUL - Final position: {position:.4f} rad")
            else:
                print("⚠️ NO POSITION NOTIFICATION")
        else:
            print("❌ MOVE COMMAND FAILED")
            return False

        # Test 7: Home
        print("\nTesting home command...")
        if controller.home():
            print("✅ HOME COMMAND SUCCESSFUL")
        else:
            print("❌ HOME COMMAND FAILED")
            return False

        # Test 8: Disable motor
        print("\nDisabling motor...")
        if controller.disable():
            print("✅ MOTOR DISABLED")
        else:
            print("❌ MOTOR DISABLE FAILED")
            return False

        print("\nAll tests passed! ✅")
        return True

    except Exception as e:
        print(f"\nError during testing: {e}")
        import traceback
        print(traceback.format_exc())
        return False

    finally:
        controller.disconnect()


if __name__ == '__main__':
    if len(sys.argv) >= 2:
        port = sys.argv[1]
        test_controller(port, verbose=True)
    else:
        print("Usage: python controller.py COM_PORT")
        sys.exit(1)
