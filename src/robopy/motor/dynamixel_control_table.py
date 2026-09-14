# robopy/control_table.py

"""
Control table definitions for Dynamixel motors.

This module provides structured access to the control table of various Dynamixel
motor series using Enums and dataclasses, enhancing type safety and code clarity.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Literal, TypedDict


class Dtype(Enum):
    """Data types for control table items."""

    UINT8 = "UINT8"
    UINT16 = "UINT16"
    UINT32 = "UINT32"
    INT8 = "INT8"
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
    POSITION_P_GAIN = ControlItem(84, 2, Dtype.UINT16, "R/W")
    POSITION_I_GAIN = ControlItem(82, 2, Dtype.UINT16, "R/W")
    POSITION_D_GAIN = ControlItem(80, 2, Dtype.UINT16, "R/W")
    CURRENT_LIMIT = ControlItem(38, 2, Dtype.UINT16, "R/W")
    # --- Added for the servo-loop / bilateral control stack ---
    # Firmware version is read at start-up and checked against the required
    # capabilities, rather than trusting the configured model name.
    FIRMWARE_VERSION = ControlItem(6, 1, Dtype.UINT8, "R")
    # Bus Watchdog is signed: -1 means "an error is latched".  Writing 0
    # disables it; 1..127 sets a 20 ms-per-count timeout.
    # https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#bus-watchdog98
    BUS_WATCHDOG = ControlItem(98, 1, Dtype.INT8, "R/W")
    HARDWARE_ERROR_STATUS = ControlItem(70, 1, Dtype.UINT8, "R")
    VELOCITY_LIMIT = ControlItem(44, 4, Dtype.UINT32, "R/W")
    MAX_POSITION_LIMIT = ControlItem(48, 4, Dtype.INT32, "R/W")
    MIN_POSITION_LIMIT = ControlItem(52, 4, Dtype.INT32, "R/W")
    PROFILE_ACCELERATION = ControlItem(108, 4, Dtype.UINT32, "R/W")
    PROFILE_VELOCITY = ControlItem(112, 4, Dtype.UINT32, "R/W")
    MOVING = ControlItem(122, 1, Dtype.UINT8, "R")


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
    if dtype == Dtype.INT8:
        return value - 0x100 if value & 0x80 else value
    if dtype == Dtype.INT16:
        # If the highest bit (sign bit) is 1, it's a negative number.
        return value - 0x10000 if value & 0x8000 else value
    if dtype == Dtype.INT32:
        return value - 0x100000000 if value & 0x80000000 else value
    # For unsigned types, no conversion is needed.
    return value


def encode_value(value: int, dtype: Dtype) -> int:
    """Encode a signed Python int into the unsigned word the wire format expects.

    This is the inverse of :func:`cast_value`.  It matters for ``GOAL_CURRENT``,
    which is an ``INT16`` and is routinely negative.

    Raises:
        ValueError: If ``value`` does not fit the range of ``dtype``.
    """
    if dtype == Dtype.INT8:
        if not -0x80 <= value <= 0x7F:
            raise ValueError(f"{value} does not fit in INT8.")
        return value & 0xFF
    if dtype == Dtype.INT16:
        if not -0x8000 <= value <= 0x7FFF:
            raise ValueError(f"{value} does not fit in INT16.")
        return value & 0xFFFF
    if dtype == Dtype.INT32:
        if not -0x80000000 <= value <= 0x7FFFFFFF:
            raise ValueError(f"{value} does not fit in INT32.")
        return value & 0xFFFFFFFF
    if dtype == Dtype.UINT8:
        limit = 0xFF
    elif dtype == Dtype.UINT16:
        limit = 0xFFFF
    else:
        limit = 0xFFFFFFFF
    if not 0 <= value <= limit:
        raise ValueError(f"{value} does not fit in {dtype.value}.")
    return value


# --- Physical units and per-model capabilities ---------------------------------
#
# Sharing ``XControlTable`` says nothing about what a model can actually do, and
# nothing about its units.  The values below come from the ROBOTIS e-Manual:
#   XM430-W350 https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
#   XM540-W270 https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/
#   XC330-T288 https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/

#: Encoder counts per output revolution for every X-series model used here.
DEFAULT_COUNTS_PER_REVOLUTION: int = 4096


class OperatingMode:
    """DYNAMIXEL X-series ``OPERATING_MODE`` register values.

    See https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
    """

    CURRENT = 0
    VELOCITY = 1
    POSITION = 3
    EXTENDED_POSITION = 4
    CURRENT_BASED_POSITION = 5
    PWM = 16


class CurrentSense:
    """How a model's ``PRESENT_CURRENT`` measurement relates to motor torque.

    Attributes:
        MOTOR_WINDING: The measurement tracks the motor winding current, so it
            can be used (with a per-joint constant) as a torque observation.
        SUPPLY_INPUT: The measurement is taken on the power-supply input side.
            It is *not* interchangeable with a winding-current measurement and
            must not reuse the XM torque-observation model.
    """

    MOTOR_WINDING = "motor_winding"
    SUPPLY_INPUT = "supply_input"


@dataclass(frozen=True)
class MotorCapabilities:
    """Per-model physical units and supported control modes.

    Attributes:
        model_name: Lower-case model name, e.g. ``"xm430-w350"``.
        current_unit_a: Amperes per raw ``GOAL_CURRENT``/``PRESENT_CURRENT`` count.
        counts_per_revolution: Encoder counts per output revolution.
        supported_operating_modes: ``OPERATING_MODE`` values this model accepts.
        current_sense: Where the current measurement is taken; see
            :class:`CurrentSense`.
        stall_torque_nm: Datasheet stall torque at the rated voltage, recorded
            for reference only.  It is deliberately *not* used to derive a
            torque constant -- a stall-torque / stall-current ratio is not a
            validated torque model.
        rated_voltage_v: Voltage the datasheet figures refer to.
    """

    model_name: str
    current_unit_a: float
    counts_per_revolution: int
    supported_operating_modes: tuple[int, ...]
    current_sense: str
    stall_torque_nm: float | None = None
    rated_voltage_v: float | None = None

    def supports_current_control(self) -> bool:
        """Whether this model supports pure current (torque) control, mode 0."""
        return OperatingMode.CURRENT in self.supported_operating_modes

    def supports_current_based_position(self) -> bool:
        """Whether this model supports current-limited position control, mode 5."""
        return OperatingMode.CURRENT_BASED_POSITION in self.supported_operating_modes


_X_SERIES_MODES: tuple[int, ...] = (
    OperatingMode.CURRENT,
    OperatingMode.VELOCITY,
    OperatingMode.POSITION,
    OperatingMode.EXTENDED_POSITION,
    OperatingMode.CURRENT_BASED_POSITION,
    OperatingMode.PWM,
)

#: Models the Rakuda control stack has verified unit data for.  Models absent
#: from this table can still be driven through the legacy raw API, but the
#: physical-unit API refuses them rather than guessing a current unit.
MOTOR_CAPABILITIES: Dict[str, MotorCapabilities] = {
    "xm430-w350": MotorCapabilities(
        model_name="xm430-w350",
        current_unit_a=0.00269,
        counts_per_revolution=DEFAULT_COUNTS_PER_REVOLUTION,
        supported_operating_modes=_X_SERIES_MODES,
        current_sense=CurrentSense.MOTOR_WINDING,
        stall_torque_nm=4.1,
        rated_voltage_v=12.0,
    ),
    "xm540-w270": MotorCapabilities(
        model_name="xm540-w270",
        current_unit_a=0.00269,
        counts_per_revolution=DEFAULT_COUNTS_PER_REVOLUTION,
        supported_operating_modes=_X_SERIES_MODES,
        current_sense=CurrentSense.MOTOR_WINDING,
        stall_torque_nm=7.3,
        rated_voltage_v=12.0,
    ),
    "xc330-t288": MotorCapabilities(
        model_name="xc330-t288",
        current_unit_a=0.001,
        counts_per_revolution=DEFAULT_COUNTS_PER_REVOLUTION,
        supported_operating_modes=_X_SERIES_MODES,
        # The XC330 senses current on the supply input, not the motor winding.
        current_sense=CurrentSense.SUPPLY_INPUT,
        stall_torque_nm=0.93,
        rated_voltage_v=5.0,
    ),
}


def get_motor_capabilities(model_name: str) -> MotorCapabilities:
    """Return the verified physical units and capabilities of ``model_name``.

    Raises:
        ValueError: If no verified unit data exists for the model.  Guessing a
            current unit would silently scale every torque command.
    """
    key = model_name.lower()
    if key not in MOTOR_CAPABILITIES:
        known = ", ".join(sorted(MOTOR_CAPABILITIES))
        raise ValueError(
            f"No verified physical-unit data for DYNAMIXEL model '{model_name}'. "
            f"Models with verified data: {known}."
        )
    return MOTOR_CAPABILITIES[key]


# --- Bulk state block ----------------------------------------------------------


@dataclass(frozen=True)
class StateBlockField:
    """One field inside a contiguous block read.

    Attributes:
        name: Field name used in the decoded result.
        offset: Byte offset from the block's start address.
        num_bytes: Width of the field.
        dtype: Signed/unsigned interpretation of the field.
    """

    name: str
    offset: int
    num_bytes: Literal[1, 2, 4]
    dtype: Dtype

    @property
    def address(self) -> int:
        """Absolute control-table address of this field."""
        return STATE_BLOCK_START_ADDRESS + self.offset


#: Start address of the contiguous control/state block used by the servo loop.
#: ``PRESENT_CURRENT`` (126) .. ``PRESENT_POSITION`` (132..135) is 10 bytes.
STATE_BLOCK_START_ADDRESS: int = 126
STATE_BLOCK_NUM_BYTES: int = 10

#: Fields inside the state block.  The block is read in one transaction but each
#: field is decoded at its own width -- the SDK's ``getData`` is called per
#: field, never once for all 10 bytes.
STATE_BLOCK_FIELDS: tuple[StateBlockField, ...] = (
    StateBlockField("present_current", 0, 2, Dtype.INT16),
    StateBlockField("present_velocity", 2, 4, Dtype.INT32),
    StateBlockField("present_position", 6, 4, Dtype.INT32),
)
