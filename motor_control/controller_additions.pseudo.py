"""
PSEUDOCODE: Additions to controller.py

These methods should be added to the MotorController class in controller.py.
They provide the move-and-verify primitive needed for scanning.
"""

#=============================================================================
# ADD TO MotorController CLASS
#=============================================================================

def get_encoder_position(self) -> float:
    """
    Read current encoder position from MCU.

    Returns absolute position in degrees from MT6701 encoder.
    This is the TRUTH SOURCE for position verification.

    Returns:
        float: Current encoder angle in degrees (0-360)

    Raises:
        ConnectionError: If MCU not connected
        TimeoutError: If no response from MCU
    """
    # IMPLEMENTATION:
    # 1. Send CMD_GET_ENCODER command to MCU
    # 2. Wait for RESP_ENCODER response with position data
    # 3. Decode position (float, degrees)
    # 4. Return position

    if not self.connected:
        raise ConnectionError("Motor controller not connected")

    # Send command
    self.link.txBuff[0] = CMD_GET_ENCODER
    self.link.send(1)

    # Wait for response with timeout
    timeout = time.time() + self.timeout
    while time.time() < timeout:
        if self.link.available():
            resp_id = self.link.rxBuff[0]
            if resp_id == RESP_ENCODER:
                # Unpack float position (degrees)
                position = struct.unpack('f', bytes(self.link.rxBuff[1:5]))[0]
                logger.debug(f"Encoder position: {position:.2f}°")
                return position
        time.sleep(0.01)

    raise TimeoutError("Encoder position request timed out")


def move_and_verify(self,
                   target_angle: float,
                   tolerance: float = 0.5,
                   max_retries: int = 3,
                   settling_time: float = 0.5) -> Tuple[bool, float]:
    """
    Move to target angle and verify position with encoder confirmation.

    This is the CORE PRIMITIVE for hyperspectral scanning. It ensures
    the motor actually reached the commanded position by reading the
    encoder (absolute truth source) after movement.

    Workflow:
        1. Send move command to target angle
        2. Wait for MCU to report movement complete
        3. Read actual encoder position
        4. If error > tolerance, retry up to max_retries times
        5. Return success status and final position

    Args:
        target_angle: Desired position in degrees
        tolerance: Maximum acceptable position error in degrees
        max_retries: Number of correction attempts before giving up
        settling_time: Seconds to wait after movement for vibration damping

    Returns:
        Tuple[bool, float]: (success, actual_position)
            - success: True if position reached within tolerance
            - actual_position: Final encoder reading in degrees

    Raises:
        ConnectionError: If motor not connected
        RuntimeError: If motor not enabled or calibrated

    Example:
        >>> success, actual = motor.move_and_verify(90.0, tolerance=0.5)
        >>> if success:
        ...     print(f"Position confirmed: {actual:.2f}°")
        ... else:
        ...     print("Failed to reach target position")
    """
    # VALIDATION
    if not self.connected:
        raise ConnectionError("Motor controller not connected")

    if not self.motor_enabled:
        raise RuntimeError("Motor must be enabled before movement")

    # RETRY LOOP
    for attempt in range(max_retries):
        logger.debug(f"Move attempt {attempt + 1}/{max_retries}: target={target_angle:.2f}°")

        # 1. Send move command
        seq_id = self.move_to(target_angle)

        # 2. Wait for movement completion
        try:
            reported_position = self.wait_for_position(seq_id, timeout=10.0)
            logger.debug(f"MCU reports position: {reported_position:.2f}°")
        except TimeoutError:
            logger.warning(f"Movement timeout on attempt {attempt + 1}")
            if attempt < max_retries - 1:
                continue
            else:
                return False, self.get_encoder_position()

        # 3. Wait for settling (vibration damping)
        time.sleep(settling_time)

        # 4. Verify position with encoder (TRUTH SOURCE)
        actual_position = self.get_encoder_position()
        position_error = abs(actual_position - target_angle)

        # Handle angle wrapping (e.g., 359° vs 1°)
        if position_error > 180.0:
            position_error = 360.0 - position_error

        logger.debug(f"Encoder verification: actual={actual_position:.2f}°, "
                    f"error={position_error:.3f}°")

        # 5. Check if within tolerance
        if position_error <= tolerance:
            logger.info(f"Position confirmed: {actual_position:.2f}° "
                       f"(target: {target_angle:.2f}°, error: {position_error:.3f}°)")
            return True, actual_position

        # Position error detected
        logger.warning(f"Position error {position_error:.3f}° exceeds tolerance "
                      f"{tolerance:.3f}° (attempt {attempt + 1}/{max_retries})")

    # All retry attempts exhausted
    final_position = self.get_encoder_position()
    final_error = abs(final_position - target_angle)
    if final_error > 180.0:
        final_error = 360.0 - final_error

    logger.error(f"Failed to reach target {target_angle:.2f}° after {max_retries} "
                f"attempts. Final position: {final_position:.2f}° "
                f"(error: {final_error:.3f}°)")

    return False, final_position


def move_and_verify_with_callback(self,
                                  target_angle: float,
                                  on_retry: Optional[Callable[[int, float], None]] = None,
                                  **kwargs) -> Tuple[bool, float]:
    """
    Move and verify with retry callback for UI feedback.

    This is an enhanced version that notifies a callback on each retry,
    useful for UI progress indication.

    Args:
        target_angle: Desired position in degrees
        on_retry: Optional callback(attempt_number, current_error)
        **kwargs: Passed to move_and_verify()

    Returns:
        Tuple[bool, float]: (success, actual_position)

    Example:
        >>> def handle_retry(attempt, error):
        ...     print(f"Retry {attempt}: error {error:.3f}°")
        >>>
        >>> success, pos = motor.move_and_verify_with_callback(
        ...     90.0, on_retry=handle_retry
        ... )
    """
    # Extract parameters
    tolerance = kwargs.get('tolerance', 0.5)
    max_retries = kwargs.get('max_retries', 3)
    settling_time = kwargs.get('settling_time', 0.5)

    # Modified retry loop with callbacks
    for attempt in range(max_retries):
        seq_id = self.move_to(target_angle)

        try:
            self.wait_for_position(seq_id, timeout=10.0)
        except TimeoutError:
            if on_retry:
                on_retry(attempt + 1, float('inf'))
            continue

        time.sleep(settling_time)
        actual_position = self.get_encoder_position()
        position_error = abs(actual_position - target_angle)

        if position_error > 180.0:
            position_error = 360.0 - position_error

        if position_error <= tolerance:
            return True, actual_position

        # Call retry callback
        if on_retry:
            on_retry(attempt + 1, position_error)

    return False, self.get_encoder_position()


#=============================================================================
# FIRMWARE INTEGRATION NOTES
#=============================================================================

"""
To support get_encoder_position(), the MCU firmware needs:

1. Add new command:
   #define CMD_GET_ENCODER 0x0F

2. Add new response:
   #define RESP_ENCODER 0x88

3. In firmware command handler:

   case CMD_GET_ENCODER:
       // Read current encoder position
       encoder.update();
       float position_deg = encoder.getDegrees();

       // Send response
       txBuff[0] = RESP_ENCODER;
       memcpy(&txBuff[1], &position_deg, sizeof(float));
       link.send(1 + sizeof(float));
       break;

This is a simple addition - the encoder reading code already exists.
"""
