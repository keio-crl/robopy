# robopy/motors.py

"""
DynamixelBus for managing multiple motors and DynamixelMotor for holding
individual motor information. This design promotes modularity and easy
integration into larger robotic systems.
"""

import logging
import time
from dataclasses import dataclass
from enum import Enum
from types import TracebackType
from typing import Any, Dict, List, Sequence, Tuple, Type

import dynamixel_sdk as dxl
import numpy as np
from numpy.typing import NDArray

from .dynamixel_control_table import (
    STATE_BLOCK_FIELDS,
    STATE_BLOCK_NUM_BYTES,
    STATE_BLOCK_START_ADDRESS,
    ControlItem,
    Dtype,
    MotorCapabilities,
    XControlTable,
    cast_value,
    encode_value,
    get_model_definition,
    get_motor_capabilities,
)

logger = logging.getLogger(__name__)

# Constants from the original script
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 2.0
NUM_READ_RETRY = 10
NUM_WRITE_RETRY = 2  # 10から2に削減 (パフォーマンス向上のため)

#: Default time budget for one *control-loop* transaction, in seconds.  The
#: legacy ten-retry read path is kept for configuration-time use only; a control
#: loop that silently retries ten times has already missed its deadline.
DEFAULT_CONTROL_TIMEOUT_S = 0.02


@dataclass(frozen=True)
class MotorStateReading:
    """Decoded state of one motor from a single bulk read.

    Attributes:
        motor_name: Name of the motor on the bus.
        motor_id: DYNAMIXEL ID.
        present_current: Raw signed current count (model-specific unit).
        present_velocity: Raw signed velocity count (0.229 rev/min per count).
        present_position: Raw signed position count.  Multi-turn values are
            returned unwrapped.
        valid: ``False`` when the motor did not return this block; the numeric
            fields are then zero and must not be used.
    """

    motor_name: str
    motor_id: int
    present_current: int
    present_velocity: int
    present_position: int
    valid: bool


class DynamixelTimeoutError(TimeoutError):
    """Raised when a control-loop bus transaction exceeds its deadline."""


class DynamixelCommError(ConnectionError):
    """Exception representing a Dynamixel communication error."""

    def __init__(self, message: str, dxl_comm_result_code: int) -> None:
        packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
        dxl_comm_result = packet_handler.getTxRxResult(dxl_comm_result_code)
        super().__init__(f"{message}\n[CommResult: {dxl_comm_result}]")


class DynamixelMotor:
    """Class that holds the definition and state of an individual motor."""

    def __init__(self, motor_id: int, motor_name: str, model_name: str) -> None:
        self.id = motor_id
        self.motor_name = motor_name
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

    def __init__(
        self,
        port: str,
        motors: Dict[str, DynamixelMotor],
        need_calibration: bool = True,
    ) -> None:
        self.port_handler = dxl.PortHandler(port)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
        self.motors = motors
        # Calibration data: {motor_name: (homing_offset, inverted)}
        self.calibration: Dict[str, Tuple[int, bool]] = {}
        # Reusable GroupSyncRead/Write handles, keyed by (address, num_bytes).
        # Rebuilding a group on every cycle re-allocates the parameter storage
        # for no benefit; the cached groups keep their registered IDs.
        self._read_groups: Dict[Tuple[int, int], Any] = {}
        self._write_groups: Dict[Tuple[int, int], Any] = {}
        self._read_group_ids: Dict[Tuple[int, int], Tuple[int, ...]] = {}

    def open(self, baudrate: int = BAUDRATE) -> None:
        """Opens the communication port."""
        if not self.port_handler.openPort():
            raise ConnectionError(f"Failed to open port {self.port_handler.port_name}.")
        if not self.port_handler.setBaudRate(baudrate):
            raise ConnectionError(f"Failed to set baudrate to {baudrate}.")
        logger.info(f"Opened port {self.port_handler.port_name} (Baudrate: {baudrate})")

    def close(self) -> None:
        """Closes the communication port."""
        self.port_handler.closePort()
        logger.info(f"Closed port {self.port_handler.port_name}.")

    def __enter__(self) -> "DynamixelBus":
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
        logger.info("Calibration data set.")

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
                logger.warning(f"Skipping {name} due to mismatched control table.")
                continue

            data = self._split_into_byte_chunks(int(processed_values[i]), control_item.num_bytes)
            if not group_sync_write.addParam(motor.id, data):
                logger.error(f"Failed to add parameter for {name} (ID-{motor.id}).")

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
        motors_to_read: List[DynamixelMotor] = []
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

        # Process received data.  Only motors that actually answered contribute a
        # value; the calibration below is applied to exactly those names, so a
        # missing motor can no longer shift every subsequent value onto the
        # wrong joint.
        raw_values: List[int] = []
        answered_names: List[str] = []
        results: Dict[str, Any] = {}
        for motor in motors_to_read:
            if group_sync_read.isAvailable(motor.id, control_item.address, control_item.num_bytes):
                raw_value = group_sync_read.getData(
                    motor.id, control_item.address, control_item.num_bytes
                )
                results[motor.motor_name] = cast_value(raw_value, control_item.dtype)
                raw_values.append(results[motor.motor_name])
                answered_names.append(motor.motor_name)

        missing = [m.motor_name for m in motors_to_read if m.motor_name not in results]
        if missing:
            logger.warning(
                "sync_read %s: no response from %s; these motors are omitted from the result "
                "rather than reported as zero.",
                item.name,
                missing,
            )

        if control_item.calibration_required and self.calibration and answered_names:
            calibrated_values = self._apply_calibration(np.array(raw_values), answered_names)
            for i, name in enumerate(answered_names):
                results[name] = calibrated_values[i]

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
        """Reads a value from a specific control table item for a single motor.

        Args:
            item: The control table item enum.
            motor_name: Name of the motor to read from.

        Returns:
            The value read from the motor.
        """
        result = self.sync_read(item, [motor_name])
        return result.get(motor_name)

    def write(self, item: Enum, motor_name: str, value: int | float) -> None:
        """Writes a value to a specific control table item for a single motor.

        Args:
            item: The control table item enum.
            motor_name: Name of the motor to write to.
            value: Value to write.
        """
        self.sync_write(item, {motor_name: value})

    def torque_disabled(self, specific_motor_names: List[str] | None = None) -> None:
        """torque_disabled for multiple motors.

        Args:
            specific_motor_names (List[str] | None, optional): List of motor names to
            disable torque. If None, disables torque for all motors. Defaults to None.
        """
        motor_names: List[str]

        if specific_motor_names is None:
            motor_names = list(self.motors.keys())
        else:
            motor_names = specific_motor_names
        torque_off_values: Dict[str, int | float] = {name: 0 for name in motor_names}

        self.sync_write(XControlTable.TORQUE_ENABLE, torque_off_values)

    def torque_enabled(self, specific_motor_names: List[str] | None = None) -> None:
        """torque_enabled for multiple motors.

        Args:
            specific_motor_names (List[str] | None, optional): List of motor names to
            enable torque. If None, enables torque for all motors. Defaults to None.
        """
        motor_names: List[str]
        if specific_motor_names is None:
            motor_names = list(self.motors.keys())
        else:
            motor_names = specific_motor_names
        torque_on_values: Dict[str, int | float] = {name: 1 for name in motor_names}
        self.sync_write(XControlTable.TORQUE_ENABLE, torque_on_values)

    # ------------------------------------------------------------------
    # Physical-unit / control-loop API
    #
    # These methods are additive: the raw ``sync_read``/``sync_write`` API above
    # keeps its exact previous meaning, including the units its callers already
    # rely on.
    # ------------------------------------------------------------------

    def capabilities(self, motor_name: str) -> MotorCapabilities:
        """Verified physical units and supported modes of ``motor_name``'s model.

        Raises:
            ValueError: If the motor is unknown, or its model has no verified
                unit data.
        """
        if motor_name not in self.motors:
            raise ValueError(f"Unknown motor '{motor_name}' on {self.port_handler.port_name}.")
        return get_motor_capabilities(self.motors[motor_name].model_name)

    def _get_read_group(
        self,
        address: int,
        num_bytes: int,
        motor_ids: Sequence[int],
    ) -> Any:
        """Return a cached ``GroupSyncRead`` registered for exactly ``motor_ids``."""
        key = (address, num_bytes)
        ids = tuple(motor_ids)
        group = self._read_groups.get(key)
        if group is None:
            group = dxl.GroupSyncRead(self.port_handler, self.packet_handler, address, num_bytes)
            self._read_groups[key] = group
            self._read_group_ids[key] = ()
        if self._read_group_ids[key] != ids:
            group.clearParam()
            for motor_id in ids:
                if not group.addParam(motor_id):
                    raise DynamixelCommError(
                        f"Failed to register ID {motor_id} for a block read at {address}.",
                        dxl.COMM_NOT_AVAILABLE,
                    )
            self._read_group_ids[key] = ids
        return group

    def _get_write_group(self, address: int, num_bytes: int) -> Any:
        """Return a cached ``GroupSyncWrite`` for ``(address, num_bytes)``."""
        key = (address, num_bytes)
        group = self._write_groups.get(key)
        if group is None:
            group = dxl.GroupSyncWrite(self.port_handler, self.packet_handler, address, num_bytes)
            self._write_groups[key] = group
        return group

    def read_state_block(
        self,
        motor_names: Sequence[str],
        *,
        timeout_s: float = DEFAULT_CONTROL_TIMEOUT_S,
    ) -> Tuple[Dict[str, MotorStateReading], int, int]:
        """Read current, velocity and position for many motors in one transaction.

        A single 10-byte SyncRead starting at ``PRESENT_CURRENT`` (126) replaces
        three per-item reads.  Each field is then decoded at its own width --
        ``int16`` at offset 0, ``int32`` at offsets 2 and 6 -- rather than
        widening any single ``ControlItem`` to ten bytes.

        SyncRead reduces the number of packets; it does *not* make the motors
        sample simultaneously.  The returned timestamps bound how far apart the
        samples in this snapshot can be, so callers can detect an excessive
        spread instead of assuming simultaneity.

        Args:
            motor_names: Motors to read, in any order.
            timeout_s: Time budget for the whole transaction, including at most
                one retry.  Exceeding it raises rather than retrying ten times.

        Returns:
            ``(readings, start_ns, end_ns)`` where ``readings`` maps motor name
            to :class:`MotorStateReading`.  A motor that did not answer is
            present with ``valid=False``; it is never zero-filled silently.

        Raises:
            DynamixelTimeoutError: If the transaction exceeds ``timeout_s``.
            DynamixelCommError: If no attempt succeeded within the budget.
        """
        selected = [self.motors[name] for name in motor_names if name in self.motors]
        unknown = [name for name in motor_names if name not in self.motors]
        if unknown:
            raise ValueError(f"Unknown motor(s) on this bus: {unknown}")
        if not selected:
            raise ValueError("read_state_block requires at least one known motor.")

        group = self._get_read_group(
            STATE_BLOCK_START_ADDRESS,
            STATE_BLOCK_NUM_BYTES,
            [m.id for m in selected],
        )

        start_ns = time.monotonic_ns()
        deadline_ns = start_ns + int(timeout_s * 1e9)
        comm_result = dxl.COMM_NOT_AVAILABLE
        succeeded = False
        while True:
            comm_result = group.txRxPacket()
            if comm_result == dxl.COMM_SUCCESS:
                succeeded = True
                break
            if time.monotonic_ns() >= deadline_ns:
                break
        end_ns = time.monotonic_ns()

        if not succeeded:
            if end_ns >= deadline_ns:
                raise DynamixelTimeoutError(
                    f"Bulk state read on {self.port_handler.port_name} exceeded its "
                    f"{timeout_s * 1e3:.1f} ms budget."
                )
            raise DynamixelCommError("Failed to read the motor state block.", comm_result)

        readings: Dict[str, MotorStateReading] = {}
        for motor in selected:
            values: Dict[str, int] = {}
            available = True
            for field_def in STATE_BLOCK_FIELDS:
                if not group.isAvailable(motor.id, field_def.address, field_def.num_bytes):
                    available = False
                    break
                raw = group.getData(motor.id, field_def.address, field_def.num_bytes)
                values[field_def.name] = cast_value(raw, field_def.dtype)
            readings[motor.motor_name] = MotorStateReading(
                motor_name=motor.motor_name,
                motor_id=motor.id,
                present_current=values.get("present_current", 0),
                present_velocity=values.get("present_velocity", 0),
                present_position=values.get("present_position", 0),
                valid=available,
            )
        return readings, start_ns, end_ns

    def write_goal_current_raw(
        self,
        values: Dict[str, int],
        *,
        timeout_s: float = DEFAULT_CONTROL_TIMEOUT_S,
    ) -> None:
        """Write signed raw ``GOAL_CURRENT`` counts to several motors at once.

        ``GOAL_CURRENT`` is an ``INT16`` and negative values are normal, so the
        values are two's-complement encoded here rather than being truncated by
        an unsigned split.

        Args:
            values: ``{motor_name: raw_count}``.  Each value is range-checked
                against the model's ``CURRENT_LIMIT`` semantics by the caller;
                this method only enforces the ``INT16`` wire range.
            timeout_s: Budget for the transmission, including retries.

        Raises:
            DynamixelTimeoutError: If the write exceeds ``timeout_s``.
            DynamixelCommError: If the transmission failed within the budget.
        """
        item: ControlItem = XControlTable.GOAL_CURRENT.value
        group = self._get_write_group(item.address, item.num_bytes)
        group.clearParam()
        for name, raw in values.items():
            if name not in self.motors:
                raise ValueError(f"Unknown motor '{name}' on this bus.")
            encoded = encode_value(int(raw), Dtype.INT16)
            data = self._split_into_byte_chunks(encoded, item.num_bytes)
            if not group.addParam(self.motors[name].id, data):
                raise DynamixelCommError(
                    f"Failed to queue GOAL_CURRENT for '{name}'.", dxl.COMM_NOT_AVAILABLE
                )

        start_ns = time.monotonic_ns()
        deadline_ns = start_ns + int(timeout_s * 1e9)
        comm_result = dxl.COMM_NOT_AVAILABLE
        while True:
            comm_result = group.txPacket()
            if comm_result == dxl.COMM_SUCCESS:
                return
            if time.monotonic_ns() >= deadline_ns:
                break
        raise DynamixelTimeoutError(
            f"GOAL_CURRENT write on {self.port_handler.port_name} exceeded its "
            f"{timeout_s * 1e3:.1f} ms budget (last result {comm_result})."
        )

    def write_goal_current_a(
        self,
        currents_a: Dict[str, float],
        *,
        timeout_s: float = DEFAULT_CONTROL_TIMEOUT_S,
    ) -> Dict[str, int]:
        """Write goal currents given in amperes, using each model's own unit.

        The per-model unit is applied here (0.00269 A/count for XM430/XM540,
        0.001 A/count for XC330), so the same ampere value produces a different
        raw count on different models.  This method does *not* apply a joint
        direction: pass a current already expressed in the motor's own positive
        direction (:class:`robopy.control.joint_mapping.JointCalibration` does
        that conversion).

        Args:
            currents_a: ``{motor_name: amperes}``, signed.
            timeout_s: Budget for the transmission.

        Returns:
            The raw counts actually sent, for logging and verification.
        """
        raw: Dict[str, int] = {}
        for name, amps in currents_a.items():
            unit = self.capabilities(name).current_unit_a
            raw[name] = int(round(amps / unit))
        self.write_goal_current_raw(raw, timeout_s=timeout_s)
        return raw

    def write_with_readback(
        self,
        item: Enum,
        values: Dict[str, int | float],
        *,
        tolerance: int = 0,
    ) -> Dict[str, int]:
        """Write a control-table item and read it back to confirm it took effect.

        A successful SyncWrite transmission only says the packet left the host;
        it does not confirm that each motor accepted the value.  Configuration
        writes therefore go through this method, while the periodic control
        writes above deliberately do not (they cannot afford the extra round
        trip).

        Args:
            item: Control-table item to write.
            values: ``{motor_name: value}``.
            tolerance: Allowed absolute difference between the written and the
                read-back value.

        Returns:
            The read-back values.

        Raises:
            DynamixelCommError: If a motor did not answer the read-back.
            ValueError: If a read-back value differs by more than ``tolerance``.
        """
        self.sync_write(item, values)
        readback = self.sync_read(item, list(values.keys()))
        mismatched = {}
        for name, wanted in values.items():
            if name not in readback:
                raise DynamixelCommError(
                    f"Motor '{name}' did not answer the read-back of {item.name}.",
                    dxl.COMM_RX_TIMEOUT,
                )
            got = int(readback[name])
            if abs(got - int(wanted)) > tolerance:
                mismatched[name] = (int(wanted), got)
        if mismatched:
            raise ValueError(
                f"Read-back of {item.name} did not match what was written: {mismatched}"
            )
        return {name: int(readback[name]) for name in values}

    def read_diagnostics(
        self,
        motor_names: Sequence[str] | None = None,
    ) -> Dict[str, Dict[str, int]]:
        """Read temperature, input voltage and hardware error status.

        These are read on their own slower schedule; they are not part of the
        per-cycle state block.

        Args:
            motor_names: Motors to query.  ``None`` queries every motor.

        Returns:
            ``{motor_name: {"temperature_c", "voltage_dv", "hardware_error"}}``,
            containing only motors that answered every item.
        """
        names = list(self.motors) if motor_names is None else list(motor_names)
        temperature = self.sync_read(XControlTable.PRESENT_TEMPERATURE, names)
        voltage = self.sync_read(XControlTable.PRESENT_INPUT_VOLTAGE, names)
        hw_error = self.sync_read(XControlTable.HARDWARE_ERROR_STATUS, names)
        out: Dict[str, Dict[str, int]] = {}
        for name in names:
            if name in temperature and name in voltage and name in hw_error:
                out[name] = {
                    "temperature_c": int(temperature[name]),
                    "voltage_dv": int(voltage[name]),
                    "hardware_error": int(hw_error[name]),
                }
        return out

    def verify_identity(
        self,
        expected: Dict[str, Tuple[int, int]] | None = None,
    ) -> Dict[str, Dict[str, int]]:
        """Read back each motor's ID, model number and firmware version.

        The configuration is not trusted on its own: what is actually on the bus
        is read and compared against what the configuration claims.

        Args:
            expected: Optional ``{motor_name: (motor_id, model_number)}``.  When
                given, a mismatch raises.

        Returns:
            ``{motor_name: {"model_number", "firmware_version"}}``.

        Raises:
            ValueError: If a motor reports a different model number than expected,
                or did not answer.
        """
        names = list(self.motors)
        model_numbers = self.sync_read(XControlTable.MODEL_NUMBER, names)
        firmware = self.sync_read(XControlTable.FIRMWARE_VERSION, names)

        result: Dict[str, Dict[str, int]] = {}
        problems: List[str] = []
        for name in names:
            if name not in model_numbers or name not in firmware:
                problems.append(f"{name}: no response")
                continue
            result[name] = {
                "model_number": int(model_numbers[name]),
                "firmware_version": int(firmware[name]),
            }
            declared = self.motors[name].model_number
            if int(model_numbers[name]) != declared:
                problems.append(
                    f"{name}: configured model number {declared} but the motor reports "
                    f"{int(model_numbers[name])}"
                )
            if expected is not None and name in expected:
                want_id, want_model = expected[name]
                if self.motors[name].id != want_id:
                    problems.append(
                        f"{name}: expected ID {want_id}, configured {self.motors[name].id}"
                    )
                if int(model_numbers[name]) != want_model:
                    problems.append(
                        f"{name}: expected model number {want_model}, got "
                        f"{int(model_numbers[name])}"
                    )
        if problems:
            raise ValueError("Motor identity check failed: " + "; ".join(problems))
        return result

    def __repr__(self) -> str:
        motor_list = ", ".join(self.motors.keys())
        return f"DynamixelBus(port={self.port_handler.port_name}, motors=[{motor_list}])"

    def __len__(self) -> int:
        return len(self.motors)
