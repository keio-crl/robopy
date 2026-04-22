# robopy/motor/feetech_bus.py

"""
FeetechBus for managing multiple Feetech STS/SMS series motors.

Mirrors the interface of DynamixelBus, providing sync_read/sync_write,
calibration, and error handling for Feetech servos (e.g. STS3215 used
by the SO-100/SO-101 robot arms).
"""

import logging
from dataclasses import asdict, dataclass
from enum import Enum
from types import TracebackType
from typing import Any, Dict, List, Type

import numpy as np
import scservo_sdk as scs
from numpy.typing import NDArray

from .feetech_control_table import (
    FeetechControlItem,
    STSControlTable,
    decode_sign_magnitude,
    encode_sign_magnitude,
    get_feetech_model_definition,
)

logger = logging.getLogger(__name__)

BAUDRATE = 1_000_000
SCS_PROTOCOL_VERSION = 0  # STS3215 uses protocol version 0
NUM_READ_RETRY = 10
NUM_WRITE_RETRY = 2


class NormMode(Enum):
    """Normalization mode for converting raw motor positions to user-facing values."""

    DEGREES = "degrees"
    RANGE_0_100 = "range_0_100"


@dataclass
class MotorCalibration:
    """Calibration data for a single motor, matching lerobot's format."""

    id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int

    def to_dict(self) -> Dict[str, int]:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: Dict[str, int]) -> "MotorCalibration":
        return cls(**d)


class FeetechCommError(ConnectionError):
    """Exception representing a Feetech communication error."""

    def __init__(self, message: str, comm_result_code: int) -> None:
        packet_handler = scs.PacketHandler(SCS_PROTOCOL_VERSION)
        comm_result = packet_handler.getTxRxResult(comm_result_code)
        super().__init__(f"{message}\n[CommResult: {comm_result}]")


class FeetechMotor:
    """Class that holds the definition and state of an individual Feetech motor."""

    def __init__(
        self,
        motor_id: int,
        motor_name: str,
        model_name: str,
        norm_mode: NormMode = NormMode.DEGREES,
    ) -> None:
        self.id = motor_id
        self.motor_name = motor_name
        self.model_name = model_name
        self.norm_mode = norm_mode

        definition = get_feetech_model_definition(model_name)
        self.control_table: type[Enum] = definition["control_table"]
        self.model_number: int = definition["model_number"]
        self.resolution: int = definition["resolution"]


class FeetechBus:
    """
    Manages communication with multiple Feetech motors on a single bus.
    Handles synchronized reading and writing, calibration, and error handling.
    """

    def __init__(
        self,
        port: str,
        motors: Dict[str, FeetechMotor],
    ) -> None:
        self.port_handler = scs.PortHandler(port)
        self.packet_handler = scs.PacketHandler(SCS_PROTOCOL_VERSION)
        self.motors = motors
        self.calibration: Dict[str, MotorCalibration] = {}

    def open(self, baudrate: int = BAUDRATE) -> None:
        """Opens the communication port."""
        if not self.port_handler.openPort():
            raise ConnectionError(f"Failed to open port {self.port_handler.port_name}.")
        if not self.port_handler.setBaudRate(baudrate):
            raise ConnectionError(f"Failed to set baudrate to {baudrate}.")
        logger.info(f"Opened Feetech port {self.port_handler.port_name} (Baudrate: {baudrate})")

    def close(self) -> None:
        """Closes the communication port."""
        self.port_handler.closePort()
        logger.info(f"Closed Feetech port {self.port_handler.port_name}.")

    def __enter__(self) -> "FeetechBus":
        self.open()
        return self

    def __exit__(
        self,
        exc_type: Type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self.close()

    def set_calibration(self, calibration_data: Dict[str, MotorCalibration]) -> None:
        """Sets the calibration data for the motors."""
        self.calibration = calibration_data
        logger.info("Feetech calibration data set.")

    def write_calibration_to_motors(self) -> None:
        """Writes calibration values (homing_offset, min/max limits) to motor EEPROM."""
        for name, calib in self.calibration.items():
            if name not in self.motors:
                continue
            self.write(STSControlTable.HOMING_OFFSET, name, calib.homing_offset)
            self.write(STSControlTable.MIN_POSITION_LIMIT, name, calib.range_min)
            self.write(STSControlTable.MAX_POSITION_LIMIT, name, calib.range_max)

    def reset_calibration_on_motors(self) -> None:
        """Resets calibration registers on all motors to defaults."""
        for name in self.motors:
            self.write(STSControlTable.HOMING_OFFSET, name, 0)
            self.write(STSControlTable.MIN_POSITION_LIMIT, name, 0)
            self.write(STSControlTable.MAX_POSITION_LIMIT, name, 4095)

    def set_half_turn_homings(self) -> Dict[str, int]:
        """Computes and writes homing offsets so each motor's midpoint reads as 2047.

        Returns the homing offset for each motor.
        """
        self.reset_calibration_on_motors()

        positions = self.sync_read(STSControlTable.PRESENT_POSITION, list(self.motors.keys()))

        homing_offsets: Dict[str, int] = {}
        for name in self.motors:
            raw_pos = int(positions[name])
            resolution = self.motors[name].resolution
            half_turn = (resolution - 1) // 2  # 2047 for 4096-step motors
            offset = raw_pos - half_turn
            homing_offsets[name] = offset
            self.write(STSControlTable.HOMING_OFFSET, name, offset)

        return homing_offsets

    def record_ranges_of_motion(
        self, motor_names: List[str]
    ) -> tuple[Dict[str, int], Dict[str, int]]:
        """Interactively records min/max positions for the specified motors.

        The user moves each joint through its full range of motion while this
        method tracks the extremes. Press Enter to finish.
        """
        import sys
        import termios
        import tty

        range_mins: Dict[str, int] = {}
        range_maxes: Dict[str, int] = {}

        # Initialize with current positions
        positions = self.sync_read(STSControlTable.PRESENT_POSITION, motor_names)
        for name in motor_names:
            val = int(positions[name])
            range_mins[name] = val
            range_maxes[name] = val

        # Set stdin to non-blocking to detect Enter
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            print("\nRecording ranges... Move all joints through their full range.")
            print("Press Enter when done.\n")

            import select

            # Track if this is the first iteration
            first_iteration = True
            num_display_lines = len(motor_names)

            while True:
                positions = self.sync_read(STSControlTable.PRESENT_POSITION, motor_names)
                for name in motor_names:
                    val = int(positions[name])
                    if val < range_mins[name]:
                        range_mins[name] = val
                    if val > range_maxes[name]:
                        range_maxes[name] = val

                # Move cursor up to overwrite previous output (except on first iteration)
                if not first_iteration:
                    # Move cursor up by num_display_lines
                    print(f"\033[{num_display_lines}A", end="")
                first_iteration = False

                # Display current tracking - one line per motor
                for name in motor_names:
                    # Clear the line and print motor info
                    print(
                        f"\033[K  {name:15s}: "
                        f"min={range_mins[name]:4d}, "
                        f"cur={int(positions[name]):4d}, "
                        f"max={range_maxes[name]:4d}"
                    )

                # Check if Enter was pressed
                if select.select([sys.stdin], [], [], 0.02)[0]:
                    sys.stdin.read(1)
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print()

        # Validate that all motors were moved
        for name in motor_names:
            if range_mins[name] == range_maxes[name]:
                raise ValueError(
                    f"Motor '{name}' was not moved during range recording "
                    f"(min == max == {range_mins[name]})."
                )

        return range_mins, range_maxes

    def _split_into_byte_chunks(self, value: int, length: int) -> List[int]:
        """Converts an integer value into a list of bytes for transmission (little-endian)."""
        if length == 1:
            return [value & 0xFF]
        if length == 2:
            return [scs.SCS_LOBYTE(value), scs.SCS_HIBYTE(value)]
        raise ValueError(f"Unsupported byte length: {length}")

    def sync_write(self, item: Enum, values: Dict[str, int | float]) -> None:
        """
        Writes values to a specific control table item for multiple motors simultaneously.

        Example:
            bus.sync_write(STSControlTable.TORQUE_ENABLE, {"motor1": 1, "motor2": 1})
        """
        if not isinstance(item.value, FeetechControlItem):
            raise TypeError("Item must be an Enum member with a FeetechControlItem value.")

        control_item: FeetechControlItem = item.value
        group_sync_write = scs.GroupSyncWrite(
            self.port_handler, self.packet_handler, control_item.address, control_item.num_bytes
        )

        processed_values = np.array([values[name] for name in self.motors if name in values])
        motor_names_to_write = [name for name in self.motors if name in values]

        # Apply inverse calibration if needed (degrees to steps)
        if control_item.calibration_required and self.calibration:
            processed_values = self._revert_calibration(processed_values, motor_names_to_write)

        # Add parameters to the sync write group
        for i, name in enumerate(motor_names_to_write):
            motor = self.motors[name]
            if motor.control_table != item.__class__:
                logger.warning(f"Skipping {name} due to mismatched control table.")
                continue

            raw_value = int(processed_values[i])

            # Apply sign-magnitude encoding if required
            if control_item.sign_magnitude_bit is not None:
                raw_value = encode_sign_magnitude(raw_value, control_item.sign_magnitude_bit)

            data = self._split_into_byte_chunks(raw_value, control_item.num_bytes)
            if not group_sync_write.addParam(motor.id, data):
                logger.error(f"Failed to add parameter for {name} (ID-{motor.id}).")

        # Transmit the packet with retries
        comm_result = scs.COMM_TX_FAIL
        for _ in range(NUM_WRITE_RETRY):
            comm_result = group_sync_write.txPacket()
            if comm_result == scs.COMM_SUCCESS:
                return

        raise FeetechCommError(f"Failed to sync write {item.name}.", comm_result)

    def sync_read(self, item: Enum, motor_names: List[str]) -> Dict[str, Any]:
        """
        Reads values from a specific control table item for multiple motors simultaneously.

        Example:
            positions = bus.sync_read(STSControlTable.PRESENT_POSITION, ["motor1", "motor2"])
        """
        if not isinstance(item.value, FeetechControlItem):
            raise TypeError("Item must be an Enum member with a FeetechControlItem value.")

        control_item: FeetechControlItem = item.value
        group_sync_read = scs.GroupSyncRead(
            self.port_handler, self.packet_handler, control_item.address, control_item.num_bytes
        )

        # Add parameters to the sync read group
        motors_to_read: List[FeetechMotor] = []
        for name in motor_names:
            if name not in self.motors:
                continue
            motor = self.motors[name]
            if motor.control_table == item.__class__:
                group_sync_read.addParam(motor.id)
                motors_to_read.append(motor)

        # Transmit the packet with retries
        comm_result = scs.COMM_TX_FAIL
        for _ in range(NUM_READ_RETRY):
            comm_result = group_sync_read.txRxPacket()
            if comm_result == scs.COMM_SUCCESS:
                break
        else:
            raise FeetechCommError(f"Failed to sync read {item.name}.", comm_result)

        # Process received data
        raw_values = []
        results: Dict[str, Any] = {}
        for motor in motors_to_read:
            if group_sync_read.isAvailable(motor.id, control_item.address, control_item.num_bytes):
                raw_value = group_sync_read.getData(
                    motor.id, control_item.address, control_item.num_bytes
                )

                # Decode sign-magnitude if required
                if control_item.sign_magnitude_bit is not None:
                    raw_value = decode_sign_magnitude(raw_value, control_item.sign_magnitude_bit)

                results[motor.motor_name] = raw_value
                raw_values.append(raw_value)

        if control_item.calibration_required and self.calibration:
            calibrated_values = self._apply_calibration(
                np.array(raw_values), [m.motor_name for m in motors_to_read]
            )
            for i, motor in enumerate(motors_to_read):
                results[motor.motor_name] = calibrated_values[i]

        return results

    def _apply_calibration(
        self,
        values: NDArray[np.int32],
        motor_names: List[str],
    ) -> NDArray[np.float32]:
        """Converts raw motor steps to normalized values using range-based normalization.

        For DEGREES mode: (raw - mid) * 360 / max_resolution
        For RANGE_0_100 mode: linear mapping [range_min, range_max] -> [0, 100]
        """
        float_values = values.astype(np.float32)
        for i, name in enumerate(motor_names):
            if name not in self.calibration:
                continue
            calib = self.calibration[name]
            motor = self.motors[name]

            if motor.norm_mode == NormMode.RANGE_0_100:
                bounded = float(np.clip(float_values[i], calib.range_min, calib.range_max))
                if calib.range_max != calib.range_min:
                    norm = (bounded - calib.range_min) / (calib.range_max - calib.range_min) * 100
                else:
                    norm = 0.0
                if calib.drive_mode:
                    norm = 100 - norm
                float_values[i] = norm
            else:
                # NormMode.DEGREES
                mid = (calib.range_min + calib.range_max) / 2.0
                max_res = motor.resolution - 1  # 4095
                float_values[i] = (float_values[i] - mid) * 360.0 / max_res

        return float_values

    def _revert_calibration(
        self,
        values: NDArray[np.float32],
        motor_names: List[str],
    ) -> NDArray[np.int32]:
        """Converts normalized values back to raw motor steps.

        Inverse of _apply_calibration.
        """
        step_values = values.copy().astype(np.float64)
        for i, name in enumerate(motor_names):
            if name not in self.calibration:
                continue
            calib = self.calibration[name]
            motor = self.motors[name]

            if motor.norm_mode == NormMode.RANGE_0_100:
                bounded = float(np.clip(step_values[i], 0.0, 100.0))
                if calib.drive_mode:
                    bounded = 100 - bounded
                raw = bounded / 100.0 * (calib.range_max - calib.range_min) + calib.range_min
                step_values[i] = raw
            else:
                # NormMode.DEGREES
                mid = (calib.range_min + calib.range_max) / 2.0
                max_res = motor.resolution - 1  # 4095
                step_values[i] = step_values[i] * max_res / 360.0 + mid

        return np.round(step_values).astype(np.int32)

    def read(self, item: Enum, motor_name: str) -> Any:
        """Reads a value from a specific control table item for a single motor."""
        result = self.sync_read(item, [motor_name])
        return result.get(motor_name)

    def write(self, item: Enum, motor_name: str, value: int | float) -> None:
        """Writes a value to a specific control table item for a single motor."""
        self.sync_write(item, {motor_name: value})

    def torque_disabled(self, specific_motor_names: List[str] | None = None) -> None:
        """Disable torque for multiple motors."""
        motor_names: List[str]
        if specific_motor_names is None:
            motor_names = list(self.motors.keys())
        else:
            motor_names = specific_motor_names
        torque_off_values: Dict[str, int | float] = {name: 0 for name in motor_names}
        self.sync_write(STSControlTable.TORQUE_ENABLE, torque_off_values)

    def torque_enabled(self, specific_motor_names: List[str] | None = None) -> None:
        """Enable torque for multiple motors."""
        motor_names: List[str]
        if specific_motor_names is None:
            motor_names = list(self.motors.keys())
        else:
            motor_names = specific_motor_names
        torque_on_values: Dict[str, int | float] = {name: 1 for name in motor_names}
        self.sync_write(STSControlTable.TORQUE_ENABLE, torque_on_values)

    def __repr__(self) -> str:
        motor_list = ", ".join(self.motors.keys())
        return f"FeetechBus(port={self.port_handler.port_name}, motors=[{motor_list}])"
