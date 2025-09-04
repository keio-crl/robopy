# robopy/control_table.py

"""
Special thanks to:
- https://github.com/nomutin/robopy/blob/master/src/robopy/control_table.py
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Literal


class Dtype(Enum):
    """Data types"""

    UINT8 = "UINT8"
    UINT16 = "UINT16"
    UINT32 = "UINT32"
    INT16 = "INT16"
    INT32 = "INT32"


@dataclass
class ControlItem:
    """Data structure for an entry in the control table.

    Attributes:
        address: Register address in the control table.
        num_bytes: Number of bytes used by the field (1, 2 or 4).
        dtype: The data type stored at the register.
        access: Read/write access string (e.g. "R", "R/W").
    """

    address: int
    num_bytes: Literal[1, 2, 4]
    dtype: Dtype
    access: Literal["R", "R/W", "R/W(NVM)"]


# --- Control table definitions per series ---
class XControlTable(Enum):
    """Dynamixel X series control table."""

    TORQUE_ENABLE = ControlItem(64, 1, Dtype.UINT8, "R/W")
    LED = ControlItem(65, 1, Dtype.UINT8, "R/W")
    GOAL_POSITION = ControlItem(116, 4, Dtype.INT32, "R/W")
    PRESENT_POSITION = ControlItem(132, 4, Dtype.INT32, "R")
    PRESENT_VELOCITY = ControlItem(128, 4, Dtype.INT32, "R")
    PRESENT_CURRENT = ControlItem(126, 2, Dtype.INT16, "R")


# NOTE:
# class AXControlTable(Enum):
#    """Dynamixel AX series control table."""
#
#    TORQUE_ENABLE = ControlItem(24, 1, Dtype.UINT8, "R/W")
#    LED = ControlItem(25, 1, Dtype.UINT8, "R/W")
#    GOAL_POSITION = ControlItem(30, 2, Dtype.UINT16, "R/W")
#    PRESENT_POSITION = ControlItem(36, 2, Dtype.UINT16, "R")


# Definitions for different Dynamixel models
MODEL_DEFINITIONS = {
    "xm430-w350": {"model_number": 1020, "control_table": XControlTable},
    "xl430-w250": {"model_number": 1060, "control_table": XControlTable},
}


# utility functions
def get_model_definition(model_name: str) -> dict:
    """Return the model definition for a given model name.

    Raises:
        ValueError: If the model name is not present in the definitions.
    """
    if model_name not in MODEL_DEFINITIONS:
        raise ValueError(f"Model '{model_name}' is not defined.")
    return MODEL_DEFINITIONS[model_name]


def cast_value(value: float, dtype: Dtype) -> int:
    """Cast a numeric value to the representation required by `dtype`.

    Signed integer types are handled using two's complement semantics.

    Args:
        value: Numeric value to convert.
        dtype: Target Dtype enumeration value.

    Returns:
        Integer value adjusted for the requested data type.
    """
    if dtype == Dtype.UINT8:
        return int(value) & 0xFF
    if dtype == Dtype.UINT16:
        return int(value) & 0xFFFF
    if dtype == Dtype.UINT32:
        return int(value) & 0xFFFFFFFF
    if dtype == Dtype.INT16:
        val = int(value) & 0xFFFF
        return val - 0x10000 if val & 0x8000 else val
    if dtype == Dtype.INT32:
        val = int(value) & 0xFFFFFFFF
        return val - 0x100000000 if val & 0x80000000 else val
    return int(value)
