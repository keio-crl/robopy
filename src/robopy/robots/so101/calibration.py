# robopy/robots/so101/calibration.py

"""
Calibration procedure for SO-101 arms using Feetech STS3215 motors.

Matches the latest lerobot calibration approach:
1. Move arm to the middle of its range → compute homing offsets (half-turn centering)
2. Move all joints through their full range → record min/max positions
3. wrist_roll is treated as a full-rotation joint (range 0–4095)

The calibration data is stored as a dict of MotorCalibration per motor.
"""

import logging
from typing import Dict

from robopy.motor.feetech_bus import FeetechBus, MotorCalibration
from robopy.motor.feetech_control_table import STSControlTable
from robopy.robots.common.arm import Arm

logger = logging.getLogger(__name__)

FULL_TURN_MOTOR = "wrist_roll"


def _disable_torque_and_set_position_mode(arm: Arm) -> FeetechBus:
    """Disables torque and sets position mode for all motors. Returns the bus."""
    bus = arm.motors
    if bus is None:
        raise RuntimeError(f"Motors for {arm} are not initialized.")
    if not isinstance(bus, FeetechBus):
        raise TypeError("SO-101 calibration requires a FeetechBus.")

    all_motor_names = arm.motor_names

    # Disable torque
    bus.sync_write(STSControlTable.TORQUE_ENABLE, {name: 0 for name in all_motor_names})

    # Set position mode (mode 0) for all motors
    bus.sync_write(STSControlTable.OPERATING_MODE, {name: 0 for name in all_motor_names})

    return bus


def run_arm_calibration(arm: Arm, arm_type: str) -> Dict[str, MotorCalibration]:
    """
    Runs the interactive calibration procedure for a single SO-101 arm.

    Procedure (matching lerobot):
    1. Move arm to the middle of its range of motion → set homing offsets
    2. Move all joints (except wrist_roll) through their full range → record min/max
    3. wrist_roll gets hardcoded range [0, 4095]
    """
    bus = _disable_torque_and_set_position_mode(arm)
    motor_names = arm.motor_names

    print(f"\n>> Running calibration of the SO-101 {arm_type} arm...")

    # --- Step 1: Set homing offsets (half-turn centering) ---
    print("\n[Step 1/2] Move the arm to the MIDDLE of its range of motion.")
    print("  Each joint should be approximately at the center of how far it can move.")
    input("Press Enter when ready...")

    homing_offsets = bus.set_half_turn_homings()
    logger.info(f"Homing offsets: {homing_offsets}")

    # --- Step 2: Record ranges of motion ---
    print("\n[Step 2/2] Move all joints (except wrist_roll) through their full range of motion.")
    print("  Slowly sweep each joint from one limit to the other.")

    unknown_range_motors = [name for name in motor_names if name != FULL_TURN_MOTOR]
    range_mins, range_maxes = bus.record_ranges_of_motion(unknown_range_motors)

    # wrist_roll is a full-rotation joint
    if FULL_TURN_MOTOR in motor_names:
        resolution = bus.motors[FULL_TURN_MOTOR].resolution
        range_mins[FULL_TURN_MOTOR] = 0
        range_maxes[FULL_TURN_MOTOR] = resolution - 1  # 4095

    print("\nCalibration for this arm is complete. Please move it to a safe rest position.")
    input("Press Enter to continue...")

    # --- Build calibration data ---
    calibration: Dict[str, MotorCalibration] = {}
    for name in motor_names:
        motor = bus.motors[name]
        calibration[name] = MotorCalibration(
            id=motor.id,
            drive_mode=0,
            homing_offset=homing_offsets[name],
            range_min=range_mins[name],
            range_max=range_maxes[name],
        )

    # Write calibration to motor EEPROM
    bus.set_calibration(calibration)
    bus.write_calibration_to_motors()

    return calibration
