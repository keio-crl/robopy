# robopy/motor/feetech_control_table.py

"""
Control table definitions for Feetech STS/SMS series motors (e.g. STS3215).

This module mirrors the structure of control_table.py for Dynamixel motors,
providing Enum-based access to the STS3215 control table and sign-magnitude
encoding utilities required by the Feetech protocol.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Literal, TypedDict

from .control_table import Dtype


@dataclass
class FeetechControlItem:
    """
    Represents an item in the Feetech STS/SMS control table.

    Attributes:
        address: The memory address of the item.
        num_bytes: The size of the data in bytes (1 or 2).
        dtype: The data type of the item.
        access: Access mode ("R" for read, "R/W" for read/write).
        calibration_required: Flag indicating if this item requires calibration.
        sign_magnitude_bit: If set, the value uses sign-magnitude encoding
            at this bit index (e.g. 15 for positions, 11 for homing offset).
    """

    address: int
    num_bytes: Literal[1, 2]
    dtype: Dtype
    access: Literal["R", "R/W"]
    calibration_required: bool = field(default=False, kw_only=True)
    sign_magnitude_bit: int | None = field(default=None, kw_only=True)


class STSControlTable(Enum):
    """Control table for Feetech STS-series motors (STS3215, etc.)."""

    # --- EPROM (persistent) ---
    MODEL_NUMBER = FeetechControlItem(3, 2, Dtype.UINT16, "R")
    ID = FeetechControlItem(5, 1, Dtype.UINT8, "R/W")
    BAUD_RATE = FeetechControlItem(6, 1, Dtype.UINT8, "R/W")
    RETURN_DELAY_TIME = FeetechControlItem(7, 1, Dtype.UINT8, "R/W")
    MIN_POSITION_LIMIT = FeetechControlItem(9, 2, Dtype.UINT16, "R/W")
    MAX_POSITION_LIMIT = FeetechControlItem(11, 2, Dtype.UINT16, "R/W")
    MAX_TEMPERATURE_LIMIT = FeetechControlItem(13, 1, Dtype.UINT8, "R")
    MAX_VOLTAGE_LIMIT = FeetechControlItem(14, 1, Dtype.UINT8, "R")
    MIN_VOLTAGE_LIMIT = FeetechControlItem(15, 1, Dtype.UINT8, "R")
    MAX_TORQUE_LIMIT = FeetechControlItem(16, 2, Dtype.UINT16, "R/W")
    P_COEFFICIENT = FeetechControlItem(21, 1, Dtype.UINT8, "R/W")
    D_COEFFICIENT = FeetechControlItem(22, 1, Dtype.UINT8, "R/W")
    I_COEFFICIENT = FeetechControlItem(23, 1, Dtype.UINT8, "R/W")
    PROTECTION_CURRENT = FeetechControlItem(28, 2, Dtype.UINT16, "R/W")
    HOMING_OFFSET = FeetechControlItem(
        31, 2, Dtype.INT16, "R/W", sign_magnitude_bit=11
    )
    OPERATING_MODE = FeetechControlItem(33, 1, Dtype.UINT8, "R/W")
    PROTECTIVE_TORQUE = FeetechControlItem(34, 1, Dtype.UINT8, "R/W")
    PROTECTION_TIME = FeetechControlItem(35, 1, Dtype.UINT8, "R/W")
    OVERLOAD_TORQUE = FeetechControlItem(36, 1, Dtype.UINT8, "R/W")

    # --- SRAM (runtime) ---
    TORQUE_ENABLE = FeetechControlItem(40, 1, Dtype.UINT8, "R/W")
    ACCELERATION = FeetechControlItem(41, 1, Dtype.UINT8, "R/W")
    GOAL_POSITION = FeetechControlItem(
        42, 2, Dtype.INT16, "R/W",
        calibration_required=True, sign_magnitude_bit=15,
    )
    GOAL_VELOCITY = FeetechControlItem(
        46, 2, Dtype.INT16, "R/W", sign_magnitude_bit=15,
    )
    TORQUE_LIMIT = FeetechControlItem(48, 2, Dtype.UINT16, "R/W")
    LOCK = FeetechControlItem(55, 1, Dtype.UINT8, "R/W")
    PRESENT_POSITION = FeetechControlItem(
        56, 2, Dtype.INT16, "R",
        calibration_required=True, sign_magnitude_bit=15,
    )
    PRESENT_VELOCITY = FeetechControlItem(58, 2, Dtype.INT16, "R")
    PRESENT_LOAD = FeetechControlItem(
        60, 2, Dtype.INT16, "R", sign_magnitude_bit=10,
    )
    PRESENT_VOLTAGE = FeetechControlItem(62, 1, Dtype.UINT8, "R")
    PRESENT_TEMPERATURE = FeetechControlItem(63, 1, Dtype.UINT8, "R")
    PRESENT_CURRENT = FeetechControlItem(69, 2, Dtype.UINT16, "R")


# --- Model Specific Definitions ---


class FeetechModelDefinition(TypedDict):
    """Typed dictionary for a Feetech motor model's definition."""

    model_number: int
    control_table: type[Enum]
    resolution: int


FEETECH_MODEL_DEFINITIONS: Dict[str, FeetechModelDefinition] = {
    "sts3215": {
        "model_number": 777,
        "control_table": STSControlTable,
        "resolution": 4096,
    },
    "sts3250": {
        "model_number": 2825,
        "control_table": STSControlTable,
        "resolution": 4096,
    },
}


def get_feetech_model_definition(model_name: str) -> FeetechModelDefinition:
    """Retrieves the definition for a given Feetech motor model name."""
    if model_name not in FEETECH_MODEL_DEFINITIONS:
        raise ValueError(f"Feetech model '{model_name}' is not defined.")
    return FEETECH_MODEL_DEFINITIONS[model_name]


# --- Sign-Magnitude Encoding Utilities ---


def decode_sign_magnitude(encoded_value: int, sign_bit_index: int) -> int:
    """Decodes a sign-magnitude encoded value to a signed integer.

    In Feetech's encoding, the sign bit indicates direction:
    0 = positive, 1 = negative. The remaining bits are the magnitude.
    """
    direction_bit = (encoded_value >> sign_bit_index) & 1
    magnitude_mask = (1 << sign_bit_index) - 1
    magnitude = encoded_value & magnitude_mask
    return -magnitude if direction_bit else magnitude


def encode_sign_magnitude(value: int, sign_bit_index: int) -> int:
    """Encodes a signed integer into sign-magnitude format.

    The sign bit is placed at sign_bit_index. Magnitude occupies lower bits.
    """
    max_magnitude = (1 << sign_bit_index) - 1
    magnitude = min(abs(value), max_magnitude)
    direction_bit = 1 if value < 0 else 0
    return (direction_bit << sign_bit_index) | magnitude
