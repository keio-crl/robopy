# robopy/robots/so101/calibration.py

"""
Calibration procedure for SO-101 arms using Feetech STS3215 motors.

The STS3215 uses a 12-bit encoder (0-4095 steps per rotation).
Calibration determines a homing offset and drive mode (inversion) for each
motor so that the zero-degree position matches a known physical pose.
"""

from typing import Dict, List, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.motor.feetech_bus import FeetechBus
from robopy.motor.feetech_control_table import STSControlTable
from robopy.robots.common.arm import Arm

ZERO_POSITION_DEGREE = 0
ROTATED_POSITION_DEGREE = 90


def convert_degrees_to_steps(degrees: float, resolutions: List[int]) -> NDArray[np.int_]:
    """Converts degrees to motor steps based on motor resolutions."""
    steps = np.array(degrees) / 180.0 * np.array(resolutions) / 2.0
    return steps.astype(int)


def assert_drive_mode(drive_mode: NDArray[np.int_]) -> None:
    if not np.all(np.isin(drive_mode, [0, 1])):
        raise ValueError(f"`drive_mode` contains values other than 0 or 1: ({drive_mode})")


def apply_drive_mode(position: NDArray[np.int_], drive_mode: NDArray[np.int_]) -> NDArray[np.int_]:
    assert_drive_mode(drive_mode)
    signed_drive_mode = -(drive_mode * 2 - 1)
    position *= signed_drive_mode
    return position


def reset_torque_and_set_mode(arm: Arm) -> None:
    """Disables torque and sets the correct operating modes for calibration."""
    bus = arm.motors
    if bus is None:
        raise RuntimeError(f"Motors for {arm} are not initialized.")
    if not isinstance(bus, FeetechBus):
        raise TypeError("SO-101 calibration requires a FeetechBus.")

    all_motor_names = arm.motor_names

    # Disable torque for all motors
    disable_torque_values: Dict[str, int | float] = {name: 0 for name in all_motor_names}
    bus.sync_write(STSControlTable.TORQUE_ENABLE, disable_torque_values)

    # Set position mode (mode 0) for all motors
    position_mode_values: Dict[str, int | float] = {name: 0 for name in all_motor_names}
    bus.sync_write(STSControlTable.OPERATING_MODE, position_mode_values)


def run_arm_calibration(arm: Arm, arm_type: str) -> Dict[str, Tuple[int, bool]]:
    """
    Runs the interactive calibration procedure for a single SO-101 arm.
    This function determines the homing offset and drive mode for each motor.
    """
    reset_torque_and_set_mode(arm)

    if arm.motors is None:
        raise RuntimeError(f"Motors for {arm_type} arm are not initialized.")

    motor_names = arm.motor_names
    resolutions = [arm.motors.motors[name].resolution for name in motor_names]

    print(f"\n>> Running calibration of the SO-101 {arm_type} arm...")

    print("\n[Step 1/2] Move the arm to the ZERO position.")
    print("  All joints should be at their neutral/zero angle.")
    input("Press Enter when ready...")

    # --- Homing Offset Calculation (First Pass) ---
    zero_pos_steps = convert_degrees_to_steps(ZERO_POSITION_DEGREE, resolutions)

    pos_dict = arm.motors.sync_read(STSControlTable.PRESENT_POSITION, motor_names)
    current_pos = np.array([pos_dict[name] for name in motor_names])

    # Round to nearest quarter turn to reduce manual positioning errors
    quarter_turn_steps = convert_degrees_to_steps(90, resolutions)
    rounded_pos = np.round(current_pos.astype(float) / quarter_turn_steps) * quarter_turn_steps

    homing_offset = zero_pos_steps - rounded_pos.astype(int)

    # --- Drive Mode Calculation ---
    print("\n[Step 2/2] Move the arm to the ROTATED position.")
    print("  Rotate each joint approximately +90 degrees from the zero position.")
    input("Press Enter when ready...")

    rotated_pos_steps = convert_degrees_to_steps(ROTATED_POSITION_DEGREE, resolutions)

    pos_dict = arm.motors.sync_read(STSControlTable.PRESENT_POSITION, motor_names)
    current_pos = np.array([pos_dict[name] for name in motor_names])

    # Apply preliminary offset and round
    pos_with_offset = current_pos + homing_offset
    rounded_pos = np.round(pos_with_offset.astype(float) / quarter_turn_steps) * quarter_turn_steps

    # If the rounded position doesn't match the target, the drive mode is inverted
    drive_mode = (rounded_pos != rotated_pos_steps).astype(np.int32)

    # --- Homing Offset Refinement (Second Pass) ---
    pos_dict = arm.motors.sync_read(STSControlTable.PRESENT_POSITION, motor_names)
    current_pos = np.array([pos_dict[name] for name in motor_names])

    pos_with_drive_mode = apply_drive_mode(current_pos, drive_mode)
    rounded_pos = (
        np.round(pos_with_drive_mode.astype(float) / quarter_turn_steps) * quarter_turn_steps
    )

    homing_offset = rotated_pos_steps - rounded_pos.astype(int)

    print("\nCalibration for this arm is complete. Please move it to a safe rest position.")
    input("Press Enter to continue...")

    # --- Format and Return Calibration Data ---
    calibration_data = {
        name: (int(homing_offset[i]), bool(drive_mode[i])) for i, name in enumerate(motor_names)
    }

    return calibration_data
