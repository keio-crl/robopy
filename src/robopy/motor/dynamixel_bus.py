# robopy/motors.py

"""
DynamixelBus for managing multiple motors and DynamixelMotor for holding
individual motor information.
"""

from enum import Enum
from typing import Any, Dict, List

import dynamixel_sdk as dxl

from .control_table import ControlItem, XControlTable, cast_value, get_model_definition


class DynamixelCommError(ConnectionError):
    """Exception representing a Dynamixel communication error."""

    def __init__(self, message: str, dxl_comm_result_code: int):
        packet_handler = dxl.Protocol2PacketHandler()
        dxl_comm_result = packet_handler.getTxRxResult(dxl_comm_result_code)
        super().__init__(f"{message}\n[CommResult: {dxl_comm_result}]")


class DynamixelMotor:
    """Class that holds the definition of individual motors."""

    def __init__(self, name: str, motor_id: int, model_name: str):
        self.name = name
        self.id = motor_id
        self.model_name = model_name

        definition = get_model_definition(model_name)
        self.control_table: XControlTable | Any = definition["control_table"]
        self.model_number: int = definition["model_number"]


class DynamixelBus:
    """Class that manages multiple Dynamixel motors and handles synchronized communication."""

    def __init__(self, port: str, motors: Dict[str, DynamixelMotor]):
        self.port_handler = dxl.PortHandler(port)
        self.packet_handler = dxl.PacketHandler(2.0)
        self.motors = motors

    def open_port(self, baudrate: int = 57600) -> None:
        if not self.port_handler.openPort():
            raise ConnectionError(f"Failed to open port {self.port_handler.port_name}.")
        if not self.port_handler.setBaudRate(baudrate):
            raise ConnectionError(f"Failed to set baudrate to {baudrate}.")
        print(f"Opened port {self.port_handler.port_name} (Baudrate: {baudrate})")

    def close_port(self) -> None:
        self.port_handler.closePort()
        print(f"Closed port {self.port_handler.port_name}.")

    def _split_into_byte_chunks(self, value: int, length: int) -> List[int]:
        """Convert a value into a byte array."""
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
        raise ValueError("Unsupported byte length.")

    def sync_write(self, item: Enum, values: Dict[str, int]) -> None:
        """Write values to multiple motors simultaneously."""
        if not isinstance(item.value, ControlItem):
            raise TypeError("Item must be an Enum member with a ControlItem value.")

        control_item: ControlItem = item.value
        group_sync_write = dxl.GroupSyncWrite(
            self.port_handler, self.packet_handler, control_item.address, control_item.num_bytes
        )

        for name, value in values.items():
            if name not in self.motors:
                continue
            motor = self.motors[name]

            # Simple check to see if the motor has the same control table item
            if motor.control_table != item.__class__:
                print(f"Warning: Skipping {name} as it has a different control table.")
                continue

            data = self._split_into_byte_chunks(value, control_item.num_bytes)
            if not group_sync_write.addParam(motor.id, data):
                print(f"Failed to add parameter for ID-{motor.id}.")

        comm_result = group_sync_write.txPacket()
        if comm_result != dxl.COMM_SUCCESS:
            raise DynamixelCommError(f"Failed to sync write {item.name}.", comm_result)

        print(f"Wrote {item.name} to {list(values.keys())}.")

    def sync_read(self, item: Enum, motor_names: List[str]) -> Dict[str, int]:
        """Read values from multiple motors simultaneously."""
        if not isinstance(item.value, ControlItem):
            raise TypeError("Item must be an Enum member with a ControlItem value.")

        control_item: ControlItem = item.value
        group_sync_read = dxl.GroupSyncRead(
            self.port_handler, self.packet_handler, control_item.address, control_item.num_bytes
        )

        for name in motor_names:
            if name not in self.motors:
                continue
            motor = self.motors[name]
            if motor.control_table == item.__class__:
                group_sync_read.addParam(motor.id)

        comm_result = group_sync_read.txRxPacket()
        if comm_result != dxl.COMM_SUCCESS:
            raise DynamixelCommError(f"Failed to sync read {item.name}.", comm_result)

        results = {}
        for name in motor_names:
            if name not in self.motors:
                continue
            motor = self.motors[name]
            if group_sync_read.isAvailable(motor.id, control_item.address, control_item.num_bytes):
                raw_value = group_sync_read.getData(
                    motor.id, control_item.address, control_item.num_bytes
                )
                results[name] = cast_value(raw_value, control_item.dtype)

        return results
