#!/usr/bin/env python3
"""
PID Auto-Tuning Script for Motor Position Control

This script automatically tunes PID parameters for motor position tracking,
starting with conservative values and gradually increasing gains while
preventing overshoot. The algorithm finds optimal PID values for accurate
encoder position tracking.

Usage:
    python pid_tuner.py [--port PORT] [--max-iterations N]
"""

import time
import argparse
import logging
from typing import Dict, List, Tuple, Optional
import numpy as np
from controller import MotorController

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class PIDTuner:
    """Auto-tuner for motor PID position control."""

    # Tuning parameters
    TEST_POSITIONS = [0.0, 1.57, 3.14, 4.71, 0.0]  # Test positions in radians
    POSITION_TOLERANCE = 0.05  # rad (~2.9 degrees)
    SETTLING_TOLERANCE = 0.02  # rad (~1.1 degrees) for settling
    MOVEMENT_TIMEOUT = 10.0  # seconds
    SETTLING_TIME_WINDOW = 0.5  # seconds to stay within tolerance
    SAMPLE_RATE = 100  # Hz for position tracking

    # Initial conservative PID values (start low to avoid overshoot)
    INITIAL_P = 2.0
    INITIAL_I = 0.0
    INITIAL_D = 0.0
    INITIAL_RAMP = 1000.0

    # PID increment steps
    P_STEP = 2.0
    D_STEP = 0.05
    I_STEP = 0.1

    # Tuning limits
    MAX_P = 50.0
    MAX_I = 5.0
    MAX_D = 2.0

    # Performance thresholds
    MAX_OVERSHOOT = 0.1  # rad (5.7 degrees) - back off if exceeded
    MAX_SETTLING_TIME = 3.0  # seconds
    TARGET_STEADY_STATE_ERROR = 0.01  # rad (0.57 degrees)

    def __init__(self, controller: MotorController):
        """Initialize PID tuner."""
        self.controller = controller
        self.current_p = self.INITIAL_P
        self.current_i = self.INITIAL_I
        self.current_d = self.INITIAL_D
        self.current_ramp = self.INITIAL_RAMP
        self.best_pid = None
        self.best_score = float('inf')

    def set_pid(self, p: float, i: float, d: float, ramp: float) -> bool:
        """Set PID parameters for position controller."""
        logger.info(f"Setting PID: P={p:.2f}, I={i:.2f}, D={d:.2f}, Ramp={ramp:.1f}")
        success = self.controller.set_pid(0, p, i, d, ramp)  # 0 = position controller
        if success:
            self.current_p = p
            self.current_i = i
            self.current_d = d
            self.current_ramp = ramp
        return success

    def track_movement(self, target_position: float, duration: float = None) -> List[Dict]:
        """
        Track motor position during movement.

        Returns:
            List of position samples with timestamps
        """
        if duration is None:
            duration = self.MOVEMENT_TIMEOUT

        samples = []
        start_time = time.time()

        while time.time() - start_time < duration:
            status = self.controller.get_status()
            if status is None:
                logger.warning("Failed to get status during tracking")
                break

            sample = {
                'timestamp': time.time() - start_time,
                'position': status['position'],
                'velocity': status['velocity'],
                'target': target_position
            }
            samples.append(sample)

            # Check if settled
            error = abs(target_position - status['position'])
            if error < self.SETTLING_TOLERANCE:
                # Wait a bit more to confirm settling
                time.sleep(self.SETTLING_TIME_WINDOW)
                status = self.controller.get_status()
                if status and abs(target_position - status['position']) < self.SETTLING_TOLERANCE:
                    logger.debug(f"Position settled at {status['position']:.4f} rad")
                    break

            time.sleep(1.0 / self.SAMPLE_RATE)

        return samples

    def analyze_response(self, samples: List[Dict], target: float) -> Dict:
        """
        Analyze step response characteristics.

        Returns:
            Dictionary with performance metrics
        """
        if not samples:
            return {
                'overshoot': float('inf'),
                'settling_time': float('inf'),
                'steady_state_error': float('inf'),
                'rise_time': float('inf'),
                'valid': False
            }

        positions = np.array([s['position'] for s in samples])
        timestamps = np.array([s['timestamp'] for s in samples])

        # Calculate initial and final positions
        initial_pos = positions[0]
        final_pos = positions[-1]
        position_change = target - initial_pos

        # Overshoot calculation
        if abs(position_change) > 0.01:  # Avoid division by zero
            if position_change > 0:
                peak_pos = np.max(positions)
                overshoot = max(0, peak_pos - target)
            else:
                peak_pos = np.min(positions)
                overshoot = max(0, target - peak_pos)
        else:
            overshoot = 0
            peak_pos = final_pos

        # Rise time (time to reach 90% of target)
        target_90 = initial_pos + 0.9 * position_change
        rise_idx = np.where(np.abs(positions - target_90) < self.SETTLING_TOLERANCE)[0]
        rise_time = timestamps[rise_idx[0]] if len(rise_idx) > 0 else timestamps[-1]

        # Settling time (time to enter and stay within settling tolerance)
        settling_time = timestamps[-1]
        for i in range(len(samples) - 1, -1, -1):
            if abs(positions[i] - target) > self.SETTLING_TOLERANCE:
                if i < len(samples) - 1:
                    settling_time = timestamps[i + 1]
                break

        # Steady state error
        steady_state_samples = positions[-int(self.SAMPLE_RATE * 0.5):]  # Last 0.5s
        steady_state_error = abs(np.mean(steady_state_samples) - target) if len(steady_state_samples) > 0 else abs(final_pos - target)

        metrics = {
            'overshoot': overshoot,
            'settling_time': settling_time,
            'steady_state_error': steady_state_error,
            'rise_time': rise_time,
            'final_position': final_pos,
            'peak_position': peak_pos,
            'valid': True
        }

        logger.debug(f"Response metrics: overshoot={overshoot:.4f}, settling_time={settling_time:.2f}s, error={steady_state_error:.4f}")

        return metrics

    def evaluate_pid(self, p: float, i: float, d: float, ramp: float) -> Tuple[float, List[Dict]]:
        """
        Evaluate PID parameters by testing movement to multiple positions.

        Returns:
            Tuple of (score, results) where lower score is better
        """
        # Set PID parameters
        if not self.set_pid(p, i, d, ramp):
            logger.error("Failed to set PID parameters")
            return float('inf'), []

        # Wait for parameters to take effect
        time.sleep(0.5)

        results = []
        total_score = 0

        # Test movements to different positions
        for i, target in enumerate(self.TEST_POSITIONS):
            logger.info(f"Test movement {i+1}/{len(self.TEST_POSITIONS)}: target={target:.2f} rad")

            # Command movement
            seq_id = self.controller.queue_movement(target)
            if seq_id == 0:
                logger.error("Failed to queue movement")
                return float('inf'), results

            # Track movement
            samples = self.track_movement(target)

            # Analyze response
            metrics = self.analyze_response(samples, target)

            if not metrics['valid']:
                logger.warning("Invalid response metrics")
                return float('inf'), results

            # Check for excessive overshoot (safety threshold)
            if metrics['overshoot'] > self.MAX_OVERSHOOT:
                logger.warning(f"Excessive overshoot detected: {metrics['overshoot']:.4f} rad")
                return float('inf'), results

            # Calculate score for this movement (weighted sum of metrics)
            # Lower score is better
            score = (
                metrics['overshoot'] * 10.0 +  # Heavily penalize overshoot
                metrics['settling_time'] * 2.0 +  # Penalize slow settling
                metrics['steady_state_error'] * 5.0 +  # Penalize steady-state error
                metrics['rise_time'] * 0.5  # Small penalty for slow rise
            )

            total_score += score

            metrics['target'] = target
            metrics['p'] = p
            metrics['i'] = i
            metrics['d'] = d
            results.append(metrics)

            logger.info(f"Movement score: {score:.2f} (overshoot={metrics['overshoot']:.4f}, settle={metrics['settling_time']:.2f}s, error={metrics['steady_state_error']:.4f})")

            # Small delay between movements
            time.sleep(0.5)

        avg_score = total_score / len(self.TEST_POSITIONS)
        logger.info(f"PID P={p:.2f} I={i:.2f} D={d:.2f} - Average score: {avg_score:.2f}")

        return avg_score, results

    def tune_p_gain(self) -> float:
        """
        Tune proportional gain by gradually increasing until we see good response.

        Returns:
            Optimal P gain
        """
        logger.info("\n" + "="*60)
        logger.info("PHASE 1: Tuning Proportional Gain (P)")
        logger.info("="*60)

        p = self.INITIAL_P
        best_p = p
        best_score = float('inf')

        while p <= self.MAX_P:
            score, results = self.evaluate_pid(p, 0, 0, self.current_ramp)

            if score == float('inf'):
                # Hit overshoot limit or other issue, back off
                logger.info(f"Overshoot or issue at P={p:.2f}, backing off")
                break

            if score < best_score:
                best_score = score
                best_p = p
                logger.info(f"New best P gain: {best_p:.2f} (score: {best_score:.2f})")
            else:
                # Score got worse, we've likely passed optimal
                logger.info(f"Score increasing at P={p:.2f}, using previous best")
                break

            p += self.P_STEP

        # Use 80% of best P to ensure stability margin
        optimal_p = best_p * 0.8
        logger.info(f"Optimal P gain (with 20% margin): {optimal_p:.2f}")

        return optimal_p

    def tune_d_gain(self, p: float) -> float:
        """
        Tune derivative gain to reduce overshoot.

        Returns:
            Optimal D gain
        """
        logger.info("\n" + "="*60)
        logger.info("PHASE 2: Tuning Derivative Gain (D)")
        logger.info("="*60)

        d = 0.0
        best_d = d
        best_score = float('inf')

        # Get baseline with P-only
        baseline_score, _ = self.evaluate_pid(p, 0, 0, self.current_ramp)
        best_score = baseline_score

        while d <= self.MAX_D:
            score, results = self.evaluate_pid(p, 0, d, self.current_ramp)

            if score == float('inf'):
                logger.info(f"Issue at D={d:.2f}, using previous best")
                break

            if score < best_score * 0.95:  # Need significant improvement (5%)
                best_score = score
                best_d = d
                logger.info(f"New best D gain: {best_d:.3f} (score: {best_score:.2f})")
            elif d > 0:
                # D not helping anymore
                logger.info(f"D gain not improving performance at D={d:.2f}")
                break

            d += self.D_STEP

        logger.info(f"Optimal D gain: {best_d:.3f}")

        return best_d

    def tune_i_gain(self, p: float, d: float) -> float:
        """
        Tune integral gain to eliminate steady-state error.

        Returns:
            Optimal I gain
        """
        logger.info("\n" + "="*60)
        logger.info("PHASE 3: Tuning Integral Gain (I)")
        logger.info("="*60)

        i = 0.0
        best_i = i
        best_score = float('inf')

        # Get baseline with PD
        baseline_score, baseline_results = self.evaluate_pid(p, 0, d, self.current_ramp)
        best_score = baseline_score

        # Check if we even need I gain
        avg_ss_error = np.mean([r['steady_state_error'] for r in baseline_results])
        if avg_ss_error < self.TARGET_STEADY_STATE_ERROR:
            logger.info(f"Steady-state error already acceptable ({avg_ss_error:.4f} rad), skipping I tuning")
            return 0.0

        while i <= self.MAX_I:
            score, results = self.evaluate_pid(p, i, d, self.current_ramp)

            if score == float('inf'):
                logger.info(f"Issue at I={i:.2f}, using previous best")
                break

            if score < best_score:
                best_score = score
                best_i = i
                logger.info(f"New best I gain: {best_i:.2f} (score: {best_score:.2f})")

                # Check if steady-state error is acceptable
                avg_ss_error = np.mean([r['steady_state_error'] for r in results])
                if avg_ss_error < self.TARGET_STEADY_STATE_ERROR:
                    logger.info(f"Steady-state error target achieved ({avg_ss_error:.4f} rad)")
                    break
            else:
                # Score got worse
                logger.info(f"I gain not improving performance at I={i:.2f}")
                break

            i += self.I_STEP

        logger.info(f"Optimal I gain: {best_i:.2f}")

        return best_i

    def run_tuning(self) -> Dict:
        """
        Run complete auto-tuning sequence.

        Returns:
            Dictionary with optimal PID parameters and tuning results
        """
        logger.info("\n" + "="*60)
        logger.info("STARTING PID AUTO-TUNING")
        logger.info("="*60)
        logger.info(f"Test positions: {self.TEST_POSITIONS}")
        logger.info(f"Position tolerance: {self.POSITION_TOLERANCE:.4f} rad")
        logger.info(f"Max overshoot allowed: {self.MAX_OVERSHOOT:.4f} rad")
        logger.info("")

        # Phase 1: Tune P
        optimal_p = self.tune_p_gain()

        # Phase 2: Tune D
        optimal_d = self.tune_d_gain(optimal_p)

        # Phase 3: Tune I
        optimal_i = self.tune_i_gain(optimal_p, optimal_d)

        # Final evaluation with optimal PID
        logger.info("\n" + "="*60)
        logger.info("FINAL EVALUATION")
        logger.info("="*60)

        final_score, final_results = self.evaluate_pid(
            optimal_p, optimal_i, optimal_d, self.current_ramp
        )

        # Calculate summary statistics
        avg_overshoot = np.mean([r['overshoot'] for r in final_results])
        avg_settling = np.mean([r['settling_time'] for r in final_results])
        avg_ss_error = np.mean([r['steady_state_error'] for r in final_results])
        max_overshoot = np.max([r['overshoot'] for r in final_results])

        results = {
            'optimal_p': optimal_p,
            'optimal_i': optimal_i,
            'optimal_d': optimal_d,
            'ramp': self.current_ramp,
            'final_score': final_score,
            'avg_overshoot': avg_overshoot,
            'max_overshoot': max_overshoot,
            'avg_settling_time': avg_settling,
            'avg_steady_state_error': avg_ss_error,
            'test_results': final_results
        }

        logger.info("\n" + "="*60)
        logger.info("TUNING COMPLETE")
        logger.info("="*60)
        logger.info(f"Optimal PID Parameters:")
        logger.info(f"  P = {optimal_p:.2f}")
        logger.info(f"  I = {optimal_i:.2f}")
        logger.info(f"  D = {optimal_d:.3f}")
        logger.info(f"  Ramp = {self.current_ramp:.1f}")
        logger.info("")
        logger.info(f"Performance Metrics:")
        logger.info(f"  Average overshoot: {avg_overshoot:.4f} rad ({avg_overshoot*180/np.pi:.2f}°)")
        logger.info(f"  Maximum overshoot: {max_overshoot:.4f} rad ({max_overshoot*180/np.pi:.2f}°)")
        logger.info(f"  Average settling time: {avg_settling:.2f} s")
        logger.info(f"  Average steady-state error: {avg_ss_error:.4f} rad ({avg_ss_error*180/np.pi:.2f}°)")
        logger.info(f"  Final score: {final_score:.2f}")
        logger.info("="*60)

        return results


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='PID Auto-Tuner for Motor Position Control')
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port for motor controller')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose debug logging')

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Connect to motor controller
    logger.info("Connecting to motor controller...")
    controller = MotorController(port=args.port)

    if not controller.connect():
        logger.error("Failed to connect to motor controller")
        return 1

    logger.info("Connected successfully")

    # Check if calibrated
    status = controller.get_status()
    if status is None:
        logger.error("Failed to get motor status")
        controller.disconnect()
        return 1

    if not status['calibrated']:
        logger.info("Motor not calibrated, running calibration...")
        if not controller.calibrate():
            logger.error("Calibration failed")
            controller.disconnect()
            return 1
        logger.info("Calibration successful")
        time.sleep(1)

    # Enable motor
    if not status['motor_enabled']:
        logger.info("Enabling motor...")
        controller.enable()
        time.sleep(0.5)

    try:
        # Run tuning
        tuner = PIDTuner(controller)
        results = tuner.run_tuning()

        # Save results to file
        import json
        output_file = 'pid_tuning_results.json'
        with open(output_file, 'w') as f:
            # Convert numpy types to native Python types for JSON serialization
            json_results = {
                k: float(v) if isinstance(v, (np.floating, np.integer)) else v
                for k, v in results.items()
                if k != 'test_results'  # Skip detailed test results
            }
            json.dump(json_results, f, indent=2)

        logger.info(f"\nResults saved to {output_file}")

        # Print final PID values for easy copying
        print("\n" + "="*60)
        print("COPY THESE VALUES TO config.h:")
        print("="*60)
        print(f"#define PID_ANGLE_P {results['optimal_p']:.2f}f")
        print(f"#define PID_ANGLE_I {results['optimal_i']:.2f}f")
        print(f"#define PID_ANGLE_D {results['optimal_d']:.3f}f")
        print("="*60)

    except KeyboardInterrupt:
        logger.info("\nTuning interrupted by user")
    except Exception as e:
        logger.error(f"Error during tuning: {e}", exc_info=True)
        return 1
    finally:
        controller.disconnect()
        logger.info("Disconnected from motor controller")

    return 0


if __name__ == '__main__':
    exit(main())
