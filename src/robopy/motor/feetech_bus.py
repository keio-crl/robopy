# robopy/motor/feetech_bus.py

"""
FeetechBus for managing multiple Feetech STS/SMS series motors.

Mirrors the interface of DynamixelBus, providing sync_read/sync_write,
calibration, and error handling for Feetech servos (e.g. STS3215 used
by the SO-100/SO-101 robot arms).
"""

import logging
from enum import Enum
from types import TracebackType
from typing import Any, Dict, List, Tuple, Type

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


class FeetechCommError(ConnectionError):
    """Exception representing a Feetech communication error."""

    def __init__(self, message: str, comm_result_code: int) -> None:
        packet_handler = scs.PacketHandler(SCS_PROTOCOL_VERSION)
        comm_result = packet_handler.getTxRxResult(comm_result_code)
        super().__init__(f"{message}\n[CommResult: {comm_result}]")


class FeetechMotor:
    """Class that holds the definition and state of an individual Feetech motor."""

    def __init__(self, motor_id: int, motor_name: str, model_name: str) -> None:
        self.id = motor_id
        self.motor_name = motor_name
        self.model_name = model_name

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
        # Calibration data: {motor_name: (homing_offset, inverted)}
        self.calibration: Dict[str, Tuple[int, bool]] = {}

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

    def set_calibration(self, calibration_data: Dict[str, Tuple[int, bool]]) -> None:
        """Sets the calibration data for the motors."""
        self.calibration = calibration_data
        logger.info("Feetech calibration data set.")

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
        """Converts raw motor steps (int32) to calibrated degrees (float32)."""
        values = values.astype(np.int32)
        for i, name in enumerate(motor_names):
            if name not in self.calibration:
                continue
            homing_offset, inverted = self.calibration[name]
            if inverted:
                values[i] *= -1
            values[i] += homing_offset

        # Convert from steps to degrees
        float_values = values.astype(np.float32)
        for i, name in enumerate(motor_names):
            resolution = self.motors[name].resolution
            float_values[i] = float_values[i] / (resolution / 2) * 180

        return float_values

    def _revert_calibration(
        self,
        values: NDArray[np.float32],
        motor_names: List[str],
    ) -> NDArray[np.int32]:
        """Converts calibrated degrees (float32) back to raw motor steps (int32)."""
        # Convert from degrees to steps
        step_values = values.astype(np.float32)
        for i, name in enumerate(motor_names):
            resolution = self.motors[name].resolution
            step_values[i] = step_values[i] / 180 * (resolution / 2)

        int_values = np.round(step_values).astype(np.int32)

        for i, name in enumerate(motor_names):
            if name not in self.calibration:
                continue
            homing_offset, inverted = self.calibration[name]
            int_values[i] -= homing_offset
            if inverted:
                int_values[i] *= -1

        return int_values

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
