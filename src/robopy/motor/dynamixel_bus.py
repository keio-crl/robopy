# robopy/motors.py

"""
DynamixelBus for managing multiple motors and DynamixelMotor for holding
individual motor information. This design promotes modularity and easy
integration into larger robotic systems.
"""

from enum import Enum
from typing import Any, Dict, List, Tuple

import dynamixel_sdk as dxl
import numpy as np

from .control_table import ControlItem, cast_value, get_model_definition

# Constants from the original script
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 2.0
NUM_READ_RETRY = 10
NUM_WRITE_RETRY = 10


class DynamixelCommError(ConnectionError):
    """Exception representing a Dynamixel communication error."""

    def __init__(self, message: str, dxl_comm_result_code: int):
        packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
        dxl_comm_result = packet_handler.getTxRxResult(dxl_comm_result_code)
        super().__init__(f"{message}\n[CommResult: {dxl_comm_result}]")


class DynamixelMotor:
    """Class that holds the definition and state of an individual motor."""

    def __init__(self, motor_id: int, model_name: str):
        self.id = motor_id
        self.model_name = model_name

        definition = get_model_definition(model_name)
        self.control_table: type[Enum] = definition["control_table"]
        self.model_number: int = definition["model_number"]
        self.resolution: int = definition["resolution"]


class DynamixelBus:
    """
    Manages communication with multiple Dynamixel motors on a single bus.
    Handles synchronized reading and writing, calibration, and error handling.
    """

    def __init__(self, port: str, motors: Dict[str, DynamixelMotor]):
        self.port_handler = dxl.PortHandler(port)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
        self.motors = motors
        # Calibration data: {motor_name: (homing_offset, inverted)}
        self.calibration: Dict[str, Tuple[int, bool]] = {}

    def open(self, baudrate: int = BAUDRATE) -> None:
        """Opens the communication port."""
        if not self.port_handler.openPort():
            raise ConnectionError(f"Failed to open port {self.port_handler.port_name}.")
        if not self.port_handler.setBaudRate(baudrate):
            raise ConnectionError(f"Failed to set baudrate to {baudrate}.")
        print(f"✅ Opened port {self.port_handler.port_name} (Baudrate: {baudrate})")

    def close(self) -> None:
        """Closes the communication port."""
        self.port_handler.closePort()
        print(f"🛑 Closed port {self.port_handler.port_name}.")

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def set_calibration(self, calibration_data: Dict[str, Tuple[int, bool]]):
        """Sets the calibration data for the motors."""
        self.calibration = calibration_data
        print("🔧 Calibration data set.")

    def _split_into_byte_chunks(self, value: int, length: int) -> List[int]:
        """Converts an integer value into a list of bytes for transmission."""
        if length == 1:
            return [value]
        if length == 2:
            return [dxl.DXL_LOBYTE(value), dxl.DXL_HIBYTE(value)]
        if length == 4:
            return [
                dxl.DXL_LOBYTE(dxl.DXL_LOWORD(value)),
                dxl.DXL_HIBYTE(dxl.DXL_LOWORD(value)),
                dxl.DXL_LOBYTE(dxl.DXL_HIWORD(value)),
                dxl.DXL_HIBYTE(dxl.DXL_HIWORD(value)),
            ]
        raise ValueError(f"Unsupported byte length: {length}")

    def sync_write(self, item: Enum, values: Dict[str, int | float]) -> None:
        """
        Writes values to a specific control table item for multiple motors simultaneously.

        Example:
            bus.sync_write(XControlTable.TORQUE_ENABLE, {"motor1": 1, "motor2": 1})
        """
        if not isinstance(item.value, ControlItem):
            raise TypeError("Item must be an Enum member with a ControlItem value.")

        control_item: ControlItem = item.value
        group_sync_write = dxl.GroupSyncWrite(
            self.port_handler, self.packet_handler, control_item.address, control_item.num_bytes
        )

        processed_values = np.array([values[name] for name in self.motors if name in values])
        motor_names_to_write = [name for name in self.motors if name in values]

        # Apply inverse calibration if needed (e.g., degrees to steps)
        if control_item.calibration_required and self.calibration:
            processed_values = self._revert_calibration(processed_values, motor_names_to_write)

        # Add parameters to the sync write group
        for i, name in enumerate(motor_names_to_write):
            motor = self.motors[name]
            # Ensure the motor uses the same control table
            if motor.control_table != item.__class__:
                print(f"⚠️ Warning: Skipping {name} due to mismatched control table.")
                continue

            data = self._split_into_byte_chunks(int(processed_values[i]), control_item.num_bytes)
            if not group_sync_write.addParam(motor.id, data):
                print(f"❌ Failed to add parameter for {name} (ID-{motor.id}).")

        # Transmit the packet with retries
        comm_result = dxl.COMM_NOT_AVAILABLE  # initialize before loop
        # Transmit the packet with retries
        for _ in range(NUM_WRITE_RETRY):
            comm_result = group_sync_write.txPacket()
            if comm_result == dxl.COMM_SUCCESS:
                return

        raise DynamixelCommError(f"Failed to sync write {item.name}.", comm_result)

    def sync_read(self, item: Enum, motor_names: List[str]) -> Dict[str, Any]:
        """
        Reads values from a specific control table item for multiple motors simultaneously.

        Example:
            positions = bus.sync_read(XControlTable.PRESENT_POSITION, ["motor1", "motor2"])
        """
        if not isinstance(item.value, ControlItem):
            raise TypeError("Item must be an Enum member with a ControlItem value.")

        control_item: ControlItem = item.value
        group_sync_read = dxl.GroupSyncRead(
            self.port_handler, self.packet_handler, control_item.address, control_item.num_bytes
        )

        # Add parameters to the sync read group
        motors_to_read = []
        for name in motor_names:
            if name not in self.motors:
                continue
            motor = self.motors[name]
            if motor.control_table == item.__class__:
                group_sync_read.addParam(motor.id)
                motors_to_read.append(motor)

        # Transmit the packet with retries
        comm_result = dxl.COMM_NOT_AVAILABLE  # ループ前に初期化
        # Transmit the packet with retries
        for _ in range(NUM_READ_RETRY):
            comm_result = group_sync_read.txRxPacket()
            if comm_result == dxl.COMM_SUCCESS:
                break
        else:
            # ループがbreakされずに終了した場合（一度も成功しなかった場合）
            raise DynamixelCommError(f"Failed to sync read {item.name}.", comm_result)

        # Process received data
        raw_values = []
        results: Dict[str, Any] = {}
        for motor in motors_to_read:
            if group_sync_read.isAvailable(motor.id, control_item.address, control_item.num_bytes):
                raw_value = group_sync_read.getData(
                    motor.id, control_item.address, control_item.num_bytes
                )
                results[motor.name] = cast_value(raw_value, control_item.dtype)
                raw_values.append(results[motor.name])

        # Apply calibration if needed (e.g., steps to degrees)
        if control_item.calibration_required and self.calibration:
            calibrated_values = self._apply_calibration(
                np.array(raw_values), [m.name for m in motors_to_read]
            )
            for i, motor in enumerate(motors_to_read):
                results[motor.name] = calibrated_values[i]

        return results

    def _apply_calibration(self, values: np.ndarray, motor_names: List[str]) -> np.ndarray:
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

    def _revert_calibration(self, values: np.ndarray, motor_names: List[str]) -> np.ndarray:
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
