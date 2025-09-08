# robopy/control_table.py

"""
Control table definitions for Dynamixel motors.

This module provides structured access to the control table of various Dynamixel
motor series using Enums and dataclasses, enhancing type safety and code clarity.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Literal, TypedDict


class Dtype(Enum):
    """Data types for control table items."""

    UINT8 = "UINT8"
    UINT16 = "UINT16"
    UINT32 = "UINT32"
    INT16 = "INT16"
    INT32 = "INT32"


@dataclass
class ControlItem:
    """
    Represents an item in the Dynamixel control table.

    Attributes:
        address: The memory address of the item.
        num_bytes: The size of the data in bytes (1, 2, or 4).
        dtype: The data type of the item.
        access: Access mode ("R" for read, "R/W" for read/write).
        calibration_required: Flag indicating if this item requires calibration
        (e.g., converting steps to degrees).
    """

    address: int
    num_bytes: Literal[1, 2, 4]
    dtype: Dtype
    access: Literal["R", "R/W"]
    calibration_required: bool = field(default=False, kw_only=True)


class XControlTable(Enum):
    """Control table for Dynamixel X-Series motors."""

    MODEL_NUMBER = ControlItem(0, 2, Dtype.UINT16, "R")
    ID = ControlItem(7, 1, Dtype.UINT8, "R/W")
    BAUD_RATE = ControlItem(8, 1, Dtype.UINT8, "R/W")
    DRIVE_MODE = ControlItem(10, 1, Dtype.UINT8, "R/W")
    OPERATING_MODE = ControlItem(11, 1, Dtype.UINT8, "R/W")
    HOMING_OFFSET = ControlItem(20, 4, Dtype.INT32, "R/W")
    TORQUE_ENABLE = ControlItem(64, 1, Dtype.UINT8, "R/W")
    LED = ControlItem(65, 1, Dtype.UINT8, "R/W")
    GOAL_CURRENT = ControlItem(102, 2, Dtype.INT16, "R/W")
    GOAL_VELOCITY = ControlItem(104, 4, Dtype.INT32, "R/W")
    GOAL_POSITION = ControlItem(116, 4, Dtype.INT32, "R/W", calibration_required=True)
    PRESENT_CURRENT = ControlItem(126, 2, Dtype.INT16, "R")
    PRESENT_VELOCITY = ControlItem(128, 4, Dtype.INT32, "R")
    PRESENT_POSITION = ControlItem(132, 4, Dtype.INT32, "R", calibration_required=True)
    PRESENT_INPUT_VOLTAGE = ControlItem(144, 2, Dtype.UINT16, "R")
    PRESENT_TEMPERATURE = ControlItem(146, 1, Dtype.UINT8, "R")


# --- Model Specific Definitions ---


class ModelDefinition(TypedDict):
    """Typed dictionary for a motor model's definition."""

    model_number: int
    control_table: type[Enum]  # e.g., XControlTable
    resolution: int


# A dictionary mapping model names to their detailed definitions.
# This makes it easy to add support for new motor models in the future.
MODEL_DEFINITIONS: Dict[str, ModelDefinition] = {
    "xl330-m077": {"model_number": 1190, "control_table": XControlTable, "resolution": 4096},
    "xl330-m288": {"model_number": 1200, "control_table": XControlTable, "resolution": 4096},
    "xc330-t288": {"model_number": 1220, "control_table": XControlTable, "resolution": 4096},
    "xl430-w250": {"model_number": 1060, "control_table": XControlTable, "resolution": 4096},
    "xm430-w350": {"model_number": 1020, "control_table": XControlTable, "resolution": 4096},
    "xm540-w270": {"model_number": 1120, "control_table": XControlTable, "resolution": 4096},
    "xc430-w150": {"model_number": 1070, "control_table": XControlTable, "resolution": 4096},
}

# --- Utility Functions ---


def get_model_definition(model_name: str) -> ModelDefinition:
    """
    Retrieves the definition for a given motor model name.

    Raises:
        ValueError: If the model name is not defined.
    """
    if model_name not in MODEL_DEFINITIONS:
        raise ValueError(f"Model '{model_name}' is not defined.")
    return MODEL_DEFINITIONS[model_name]


def cast_value(value: int, dtype: Dtype) -> int:
    """
    Casts a raw integer value from the motor to the correct signed/unsigned type.
    Handles two's complement for signed integers.
    """
    if dtype == Dtype.INT16:
        # If the highest bit (sign bit) is 1, it's a negative number.
        return value - 0x10000 if value & 0x8000 else value
    if dtype == Dtype.INT32:
        return value - 0x100000000 if value & 0x80000000 else value
    # For unsigned types, no conversion is needed.
    return value
