#!/usr/bin/env python3
"""
Motor Calibration with PID Auto-Tuning

This script demonstrates the complete motor calibration workflow including
automatic PID tuning for optimal position tracking performance.

Usage:
    python calibrate_with_pid_tuning.py [--port PORT] [--skip-tuning]
"""

import argparse
import logging
import time
import json
from controller import MotorController

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    """Main calibration workflow."""
    parser = argparse.ArgumentParser(
        description='Motor Calibration with PID Auto-Tuning'
    )
    parser.add_argument(
        '--port', type=str, default=None,
        help='Serial port for motor controller'
    )
    parser.add_argument(
        '--skip-tuning', action='store_true',
        help='Skip PID auto-tuning (use existing PID values)'
    )
    parser.add_argument(
        '--save-results', type=str, default='pid_tuning_results.json',
        help='File to save tuning results (default: pid_tuning_results.json)'
    )
    parser.add_argument(
        '--verbose', '-v', action='store_true',
        help='Enable verbose debug logging'
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    print("\n" + "="*70)
    print("MOTOR CALIBRATION WITH PID AUTO-TUNING")
    print("="*70)
    print()

    # Step 1: Connect to motor controller
    logger.info("Step 1: Connecting to motor controller...")
    controller = MotorController(port=args.port)

    if not controller.connect():
        logger.error("Failed to connect to motor controller")
        return 1

    logger.info("Connected successfully")
    print()

    try:
        # Step 2: Check current status
        logger.info("Step 2: Checking motor status...")
        status = controller.get_status()
        if status is None:
            logger.error("Failed to get motor status")
            return 1

        logger.info(f"Motor calibrated: {status['calibrated']}")
        logger.info(f"Motor enabled: {status['motor_enabled']}")
        logger.info(f"Current position: {status['position']:.4f} rad")
        print()

        # Step 3: Run calibration if needed
        if not status['calibrated']:
            logger.info("Step 3: Running motor calibration...")
            if not controller.calibrate():
                logger.error("Calibration failed")
                return 1
            logger.info("Calibration successful")
            time.sleep(1)
        else:
            logger.info("Step 3: Motor already calibrated, skipping")
        print()

        # Step 4: Enable motor
        if not status['motor_enabled']:
            logger.info("Step 4: Enabling motor...")
            controller.enable()
            time.sleep(0.5)
        else:
            logger.info("Step 4: Motor already enabled")
        print()

        # Step 5: Run PID auto-tuning
        if not args.skip_tuning:
            logger.info("Step 5: Running PID auto-tuning...")
            print()
            print("This will test motor response at multiple positions")
            print("to find optimal PID parameters. This may take several minutes.")
            print()

            tuning_results = controller.auto_tune_pid(apply_results=True)

            if tuning_results is None:
                logger.error("PID tuning failed")
                return 1

            # Save results
            if args.save_results:
                logger.info(f"Saving tuning results to {args.save_results}...")
                with open(args.save_results, 'w') as f:
                    # Filter out non-serializable data
                    save_data = {
                        k: v for k, v in tuning_results.items()
                        if k != 'test_results'
                    }
                    json.dump(save_data, f, indent=2)

            print()
            print("="*70)
            print("PID TUNING RESULTS")
            print("="*70)
            print(f"Optimal P: {tuning_results['optimal_p']:.2f}")
            print(f"Optimal I: {tuning_results['optimal_i']:.2f}")
            print(f"Optimal D: {tuning_results['optimal_d']:.3f}")
            print(f"Ramp: {tuning_results['ramp']:.1f}")
            print()
            print("Performance:")
            print(f"  Average overshoot: {tuning_results['avg_overshoot']:.4f} rad")
            print(f"  Maximum overshoot: {tuning_results['max_overshoot']:.4f} rad")
            print(f"  Average settling time: {tuning_results['avg_settling_time']:.2f} s")
            print(f"  Steady-state error: {tuning_results['avg_steady_state_error']:.4f} rad")
            print()
            print("These PID values have been automatically applied to the motor.")
            print()
            print("To make these values permanent, update config.h with:")
            print(f"  #define PID_ANGLE_P {tuning_results['optimal_p']:.2f}f")
            print(f"  #define PID_ANGLE_I {tuning_results['optimal_i']:.2f}f")
            print(f"  #define PID_ANGLE_D {tuning_results['optimal_d']:.3f}f")
            print("="*70)
            print()

        else:
            logger.info("Step 5: Skipping PID auto-tuning (--skip-tuning flag set)")
            print()

        # Step 6: Verify final status
        logger.info("Step 6: Verifying final configuration...")
        final_status = controller.get_status()
        if final_status:
            logger.info(f"Motor calibrated: {final_status['calibrated']}")
            logger.info(f"Motor enabled: {final_status['motor_enabled']}")
            logger.info(f"Current position: {final_status['position']:.4f} rad")
        print()

        print("="*70)
        print("CALIBRATION COMPLETE")
        print("="*70)
        print()
        print("The motor is now calibrated and ready for use.")
        if not args.skip_tuning:
            print("PID parameters have been optimized for your system.")
        print()

    except KeyboardInterrupt:
        logger.info("\nCalibration interrupted by user")
        return 1
    except Exception as e:
        logger.error(f"Error during calibration: {e}", exc_info=True)
        return 1
    finally:
        controller.disconnect()
        logger.info("Disconnected from motor controller")

    return 0


if __name__ == '__main__':
    exit(main())
