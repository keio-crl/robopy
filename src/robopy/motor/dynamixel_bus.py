# robopy/motors.py

"""
DynamixelBus for managing multiple motors and DynamixelMotor for holding
individual motor information. This design promotes modularity and easy
integration into larger robotic systems.

The bus itself only deals with names, calibration and the control table; the
actual protocol traffic is delegated to a
:class:`~robopy.motor.dynamixel_transport.DynamixelTransport`.  Two are
available and are chosen by the ``backend`` argument:

``"python"``
    the pure-Python ``dynamixel_sdk``;
``"native"``
    the optional ``robopy_dxl`` C++ extension (Fast Sync Read, GIL-free
    transfers) built from ``native/robopy_dxl``;
``"auto"`` (default)
    native when it is installed, python otherwise.

Sync-read/-write groups are built once per (control item, motor set) and reused,
so the hot path does not re-serialise its parameters on every call.
"""

import logging
from enum import Enum
from types import TracebackType
from typing import Any, Dict, List, Mapping, Sequence, Tuple, Type

import numpy as np
from numpy.typing import NDArray

from .dynamixel_control_table import ControlItem, Dtype, XControlTable, get_model_definition
from .dynamixel_transport import (
    DEFAULT_BAUDRATE,
    PROTOCOL_VERSION,
    Backend,
    DynamixelCommError,
    DynamixelTransport,
    create_transport,
    native_available,
)
from .dynamixel_transport import sync_read_parallel as _transport_sync_read_parallel
from .port_tuning import PortTuning, set_latency_timer

logger = logging.getLogger(__name__)

# Constants from the original script
BAUDRATE = DEFAULT_BAUDRATE
NUM_READ_RETRY = 10
NUM_WRITE_RETRY = 2  # 10から2に削減 (パフォーマンス向上のため)

_SIGNED_DTYPES = (Dtype.INT16, Dtype.INT32)

__all__ = [
    "BAUDRATE",
    "PROTOCOL_VERSION",
    "NUM_READ_RETRY",
    "NUM_WRITE_RETRY",
    "DynamixelBus",
    "DynamixelCommError",
    "DynamixelMotor",
    "native_available",
    "sync_read_parallel",
]


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
        backend: Backend = "auto",
    ) -> None:
        self.port = port
        self.motors = motors
        # Calibration data: {motor_name: (homing_offset, inverted)}
        self.calibration: Dict[str, Tuple[int, bool]] = {}

        self._transport: DynamixelTransport = create_transport(port, backend)
        # (address, num_bytes, motor names) -> transport group handle
        self._read_handles: Dict[Tuple[int, int, Tuple[str, ...]], int] = {}
        self._write_handles: Dict[Tuple[int, int, Tuple[str, ...]], int] = {}
        # motor names -> (signs, homing offsets, steps-per-degree scale)
        self._calibration_cache: Dict[
            Tuple[str, ...], Tuple[NDArray[np.int32], NDArray[np.int32], NDArray[np.float32]]
        ] = {}

    # -- connection --------------------------------------------------------

    def open(self, baudrate: int = BAUDRATE, latency_timer_ms: int | None = 1) -> None:
        """Opens the communication port.

        Args:
            baudrate: Bus baudrate.
            latency_timer_ms: Target USB latency timer. The Linux default of
                16ms dominates the round-trip time of every transfer, so it is
                lowered by default. Pass ``None`` to leave it untouched.
        """
        if latency_timer_ms is not None:
            self.tune_port(latency_timer_ms)
        self._transport.open(baudrate)
        logger.info(
            f"Opened port {self.port} (Baudrate: {baudrate}, "
            f"backend: {self._transport.backend_name})"
        )

    def close(self) -> None:
        """Closes the communication port."""
        self._transport.close()
        logger.info(f"Closed port {self.port}.")

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

    @property
    def backend(self) -> str:
        """Name of the active transport: ``"python"`` or ``"native"``."""
        return self._transport.backend_name

    @property
    def transport(self) -> DynamixelTransport:
        return self._transport

    def tune_port(self, latency_ms: int = 1) -> PortTuning:
        """Lower the USB latency timer of this bus's port. Never raises."""
        return set_latency_timer(self.port, latency_ms)

    def set_return_delay_time(self, value: int = 0, motor_names: List[str] | None = None) -> None:
        """Set the Return Delay Time of every motor (raw units of 2us).

        The factory default is 250, i.e. 500us *per motor*.  On a plain sync
        read those delays are paid one after another, so on a 17 motor arm the
        default alone costs about 8.5ms per read.  Setting it to 0 is safe for
        a USB host and is the single cheapest latency win available.

        RETURN_DELAY_TIME lives in the EEPROM area, which has a limited write
        endurance, so motors already holding the target value are skipped: after
        the first call this becomes a pure read.
        """
        names = list(self.motors.keys()) if motor_names is None else motor_names
        current = self.sync_read(XControlTable.RETURN_DELAY_TIME, names)
        stale = [name for name in names if current.get(name) != value]
        if not stale:
            logger.debug(f"RETURN_DELAY_TIME already {value} on all motors of {self.port}.")
            return

        self.sync_write(XControlTable.RETURN_DELAY_TIME, {name: value for name in stale})
        logger.info(f"Set RETURN_DELAY_TIME={value} on {len(stale)} motors of {self.port}.")

    def set_calibration(self, calibration_data: Dict[str, Tuple[int, bool]]) -> None:
        """Sets the calibration data for the motors."""
        self.calibration = calibration_data
        self._calibration_cache.clear()
        logger.info("Calibration data set.")

    # -- group management --------------------------------------------------

    @staticmethod
    def _control_item(item: Enum) -> ControlItem:
        if not isinstance(item.value, ControlItem):
            raise TypeError("Item must be an Enum member with a ControlItem value.")
        return item.value

    def _matching_names(self, item: Enum, names: Sequence[str]) -> List[str]:
        """Names that exist on this bus and share the item's control table."""
        matching: List[str] = []
        for name in names:
            motor = self.motors.get(name)
            if motor is None:
                continue
            if motor.control_table is not item.__class__:
                logger.warning(f"Skipping {name} due to mismatched control table.")
                continue
            matching.append(name)
        return matching

    def _read_handle(self, control_item: ControlItem, names: Sequence[str], is_signed: bool) -> int:
        key = (control_item.address, control_item.num_bytes, tuple(names))
        handle = self._read_handles.get(key)
        if handle is None:
            handle = self._transport.make_read_group(
                control_item.address,
                control_item.num_bytes,
                [self.motors[name].id for name in names],
                is_signed,
            )
            self._read_handles[key] = handle
        return handle

    def _write_handle(self, control_item: ControlItem, names: Sequence[str]) -> int:
        key = (control_item.address, control_item.num_bytes, tuple(names))
        handle = self._write_handles.get(key)
        if handle is None:
            handle = self._transport.make_write_group(
                control_item.address,
                control_item.num_bytes,
                [self.motors[name].id for name in names],
            )
            self._write_handles[key] = handle
        return handle

    # -- transfers ---------------------------------------------------------

    def sync_write(self, item: Enum, values: Mapping[str, int | float]) -> None:
        """
        Writes values to a specific control table item for multiple motors simultaneously.

        Example:
            bus.sync_write(XControlTable.TORQUE_ENABLE, {"motor1": 1, "motor2": 1})
        """
        control_item = self._control_item(item)
        names = self._matching_names(item, [name for name in self.motors if name in values])
        if not names:
            return

        raw = np.array([values[name] for name in names], dtype=np.float32)
        if control_item.calibration_required and self.calibration:
            data = self._revert_calibration(raw, names)
        else:
            data = np.round(raw).astype(np.int32)

        self._transport.sync_write(self._write_handle(control_item, names), data, NUM_WRITE_RETRY)

    def sync_read(self, item: Enum, motor_names: List[str]) -> Dict[str, Any]:
        """
        Reads values from a specific control table item for multiple motors simultaneously.

        Example:
            positions = bus.sync_read(XControlTable.PRESENT_POSITION, ["motor1", "motor2"])

        Raises:
            DynamixelCommError: if any of the requested motors failed to answer
                after ``NUM_READ_RETRY`` attempts. A partial result is never
                returned, because a shortened observation vector corrupts
                recorded data silently.
        """
        control_item = self._control_item(item)
        names = self._matching_names(item, motor_names)
        if not names:
            return {}

        is_signed = control_item.dtype in _SIGNED_DTYPES or control_item.calibration_required
        handle = self._read_handle(control_item, names, is_signed)
        raw = self._transport.sync_read(handle, NUM_READ_RETRY)

        if control_item.calibration_required and self.calibration:
            calibrated = self._apply_calibration(raw, names)
            return dict(zip(names, calibrated.tolist()))
        return dict(zip(names, raw.tolist()))

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

    def uses_fast_sync_read(self, item: Enum, motor_names: List[str]) -> bool:
        """Whether this (item, motors) group is currently served by Fast Sync Read."""
        control_item = self._control_item(item)
        names = self._matching_names(item, motor_names)
        key = (control_item.address, control_item.num_bytes, tuple(names))
        handle = self._read_handles.get(key)
        if handle is None:
            return False
        return self._transport.uses_fast_sync_read(handle)

    # -- calibration -------------------------------------------------------

    def _calibration_arrays(
        self, motor_names: Sequence[str]
    ) -> Tuple[NDArray[np.int32], NDArray[np.int32], NDArray[np.float32]]:
        """Per-motor (sign, homing offset, degrees-per-step) as numpy arrays."""
        key = tuple(motor_names)
        cached = self._calibration_cache.get(key)
        if cached is not None:
            return cached

        signs = np.ones(len(key), dtype=np.int32)
        offsets = np.zeros(len(key), dtype=np.int32)
        scales = np.empty(len(key), dtype=np.float32)
        for i, name in enumerate(key):
            homing_offset, inverted = self.calibration.get(name, (0, False))
            signs[i] = -1 if inverted else 1
            offsets[i] = homing_offset
            scales[i] = 360.0 / self.motors[name].resolution

        cached = (signs, offsets, scales)
        self._calibration_cache[key] = cached
        return cached

    def _apply_calibration(
        self,
        values: NDArray[np.int32],
        motor_names: List[str],
    ) -> NDArray[np.float32]:
        """Converts raw motor steps (int32) to calibrated degrees (float32)."""
        signs, offsets, scales = self._calibration_arrays(motor_names)
        steps = values.astype(np.int32) * signs + offsets
        return (steps.astype(np.float32) * scales).astype(np.float32)

    def _revert_calibration(
        self,
        values: NDArray[np.float32],
        motor_names: List[str],
    ) -> NDArray[np.int32]:
        """Converts calibrated degrees (float32) back to raw motor steps (int32)."""
        signs, offsets, scales = self._calibration_arrays(motor_names)
        steps = np.round(values.astype(np.float32) / scales).astype(np.int32)
        return (steps - offsets) * signs

    # -- torque ------------------------------------------------------------

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

    def __repr__(self) -> str:
        motor_list = ", ".join(self.motors.keys())
        return f"DynamixelBus(port={self.port}, backend={self.backend}, motors=[{motor_list}])"

    def __len__(self) -> int:
        return len(self.motors)


def sync_read_parallel(
    targets: Sequence[Tuple["DynamixelBus", Enum, List[str]]],
) -> List[Dict[str, Any]]:
    """Read one control item per bus, with the buses serviced concurrently.

    A leader and a follower arm are two independent USB devices, so there is no
    reason to wait for one before starting the other. With the native transport
    the transfers genuinely overlap (the GIL is released in C++); with the
    Python transport they overlap only inside the blocking read/write syscalls.

    Example:
        leader_pos, follower_pos = sync_read_parallel([
            (leader.motors, XControlTable.PRESENT_POSITION, leader_names),
            (follower.motors, XControlTable.PRESENT_POSITION, follower_names),
        ])
    """
    prepared: List[Tuple[DynamixelBus, ControlItem, List[str], int]] = []
    transport_targets: List[Tuple[DynamixelTransport, int]] = []
    for bus, item, motor_names in targets:
        control_item = DynamixelBus._control_item(item)
        names = bus._matching_names(item, motor_names)
        is_signed = control_item.dtype in _SIGNED_DTYPES or control_item.calibration_required
        handle = bus._read_handle(control_item, names, is_signed)
        prepared.append((bus, control_item, names, handle))
        transport_targets.append((bus.transport, handle))

    raw_results = _transport_sync_read_parallel(transport_targets, NUM_READ_RETRY)

    results: List[Dict[str, Any]] = []
    for (bus, control_item, names, _), raw in zip(prepared, raw_results):
        if control_item.calibration_required and bus.calibration:
            values = bus._apply_calibration(raw, names).tolist()
        else:
            values = raw.tolist()
        results.append(dict(zip(names, values)))
    return results
