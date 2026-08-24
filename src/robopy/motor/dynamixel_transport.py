"""Transports that carry Dynamixel protocol 2.0 traffic for :class:`DynamixelBus`.

Two interchangeable implementations are provided:

``python``
    The pure-Python ``dynamixel_sdk``.  Always available.
``native``
    The optional ``robopy_dxl`` package: a pybind11 extension around the **C++**
    DynamixelSDK.  Install it from ``native/robopy_dxl`` in this repository.

Both expose the same, deliberately narrow interface: a *group* (address, byte
width and motor ids) is registered once at connect time and then transferred by
handle.  That is what makes the hot path cheap -- the Python SDK otherwise
rebuilds its ``GroupSyncRead``/``GroupSyncWrite`` and re-serialises every
parameter on every call.

The native transport additionally uses Fast Sync Read (protocol 2.0 instruction
``0x8A``), which answers a whole group with one broadcast status packet instead
of one status packet per motor.  The Python SDK pinned by robopy does not
implement it.  Motors whose firmware does not answer ``0x8A`` are detected on
the first read and silently downgraded to a plain sync read.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Any, List, Literal, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

logger = logging.getLogger(__name__)

Backend = Literal["auto", "python", "native"]

PROTOCOL_VERSION = 2.0
DEFAULT_BAUDRATE = 1_000_000


class DynamixelCommError(ConnectionError):
    """Raised when a Dynamixel transfer fails after all retries."""

    def __init__(self, message: str, dxl_comm_result_code: int | None = None) -> None:
        if dxl_comm_result_code is None:
            super().__init__(message)
        else:
            import dynamixel_sdk as dxl

            detail = dxl.PacketHandler(PROTOCOL_VERSION).getTxRxResult(dxl_comm_result_code)
            super().__init__(f"{message}\n[CommResult: {detail}]")
        self.comm_result = dxl_comm_result_code


def native_available() -> bool:
    """True when the optional C++ transport (``robopy_dxl``) is importable."""
    try:
        import robopy_dxl  # noqa: F401
    except ImportError:
        return False
    return True


def _sign_extend(values: NDArray[np.int64], num_bytes: int) -> NDArray[np.int32]:
    """Reinterpret raw unsigned control-table values as signed."""
    if num_bytes == 1:
        return values.astype(np.uint8).view(np.int8).astype(np.int32)
    if num_bytes == 2:
        return values.astype(np.uint16).view(np.int16).astype(np.int32)
    return values.astype(np.uint32).view(np.int32).astype(np.int32)


class DynamixelTransport(ABC):
    """Common interface for the Python and C++ Dynamixel transports."""

    backend_name: str

    def __init__(self, port: str) -> None:
        self.port = port

    @abstractmethod
    def open(self, baudrate: int = DEFAULT_BAUDRATE) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @property
    @abstractmethod
    def is_open(self) -> bool: ...

    @abstractmethod
    def make_read_group(
        self, address: int, num_bytes: int, ids: Sequence[int], is_signed: bool
    ) -> int:
        """Register a reusable sync-read group and return its handle."""

    @abstractmethod
    def make_write_group(self, address: int, num_bytes: int, ids: Sequence[int]) -> int:
        """Register a reusable sync-write group and return its handle."""

    @abstractmethod
    def sync_read(self, handle: int, retries: int) -> NDArray[np.int32]:
        """Read a registered group; values follow the registered id order."""

    @abstractmethod
    def sync_write(self, handle: int, values: NDArray[np.int32], retries: int) -> None:
        """Write a registered group; ``values`` must match the registered ids."""

    def uses_fast_sync_read(self, handle: int) -> bool:
        """Whether this group is currently served by Fast Sync Read."""
        return False

    def __repr__(self) -> str:
        return f"{type(self).__name__}(port={self.port!r}, open={self.is_open})"


# --------------------------------------------------------------------------- #
# Pure Python transport
# --------------------------------------------------------------------------- #


class PythonTransport(DynamixelTransport):
    """``dynamixel_sdk`` driven transport with cached group objects."""

    backend_name = "python"

    def __init__(self, port: str) -> None:
        super().__init__(port)
        import dynamixel_sdk as dxl

        self._dxl = dxl
        self.port_handler = dxl.PortHandler(port)
        self.packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
        self._is_open = False
        self._read_groups: List[Tuple[Any, int, int, List[int], bool]] = []
        self._write_groups: List[Tuple[Any, int, int, List[int]]] = []

    def open(self, baudrate: int = DEFAULT_BAUDRATE) -> None:
        if self._is_open:
            return
        if not self.port_handler.openPort():
            raise ConnectionError(f"Failed to open port {self.port}.")
        if not self.port_handler.setBaudRate(baudrate):
            self.port_handler.closePort()
            raise ConnectionError(f"Failed to set baudrate to {baudrate} on {self.port}.")
        self._is_open = True

    def close(self) -> None:
        if not self._is_open:
            return
        self.port_handler.closePort()
        self._is_open = False

    @property
    def is_open(self) -> bool:
        return self._is_open

    def make_read_group(
        self, address: int, num_bytes: int, ids: Sequence[int], is_signed: bool
    ) -> int:
        group = self._dxl.GroupSyncRead(self.port_handler, self.packet_handler, address, num_bytes)
        for motor_id in ids:
            if not group.addParam(motor_id):
                raise RuntimeError(f"GroupSyncRead.addParam failed for id {motor_id}.")
        self._read_groups.append((group, address, num_bytes, list(ids), is_signed))
        return len(self._read_groups) - 1

    def make_write_group(self, address: int, num_bytes: int, ids: Sequence[int]) -> int:
        group = self._dxl.GroupSyncWrite(self.port_handler, self.packet_handler, address, num_bytes)
        zeros = [0] * num_bytes
        for motor_id in ids:
            if not group.addParam(motor_id, zeros):
                raise RuntimeError(f"GroupSyncWrite.addParam failed for id {motor_id}.")
        self._write_groups.append((group, address, num_bytes, list(ids)))
        return len(self._write_groups) - 1

    def sync_read(self, handle: int, retries: int) -> NDArray[np.int32]:
        group, address, num_bytes, ids, is_signed = self._read_groups[handle]
        comm_result = self._dxl.COMM_RX_FAIL
        for _ in range(max(1, retries)):
            comm_result = group.txRxPacket()
            if comm_result != self._dxl.COMM_SUCCESS:
                continue
            if not all(group.isAvailable(motor_id, address, num_bytes) for motor_id in ids):
                # A partial answer is a failed transfer: returning it would
                # silently shorten the observation vector.
                continue
            raw = np.array(
                [group.getData(motor_id, address, num_bytes) for motor_id in ids], dtype=np.int64
            )
            return _sign_extend(raw, num_bytes) if is_signed else raw.astype(np.int32)

        raise DynamixelCommError(
            f"Failed to sync read address {address} on {self.port} (ids: {ids})", comm_result
        )

    def sync_write(self, handle: int, values: NDArray[np.int32], retries: int) -> None:
        group, address, num_bytes, ids = self._write_groups[handle]
        if len(values) != len(ids):
            raise ValueError(f"Expected {len(ids)} values, got {len(values)}.")

        raw = np.asarray(values, dtype=np.int64) & ((1 << (8 * num_bytes)) - 1)
        for motor_id, value in zip(ids, raw.tolist()):
            data = [(value >> (8 * i)) & 0xFF for i in range(num_bytes)]
            if not group.changeParam(motor_id, data):
                raise RuntimeError(f"GroupSyncWrite.changeParam failed for id {motor_id}.")

        comm_result = self._dxl.COMM_TX_FAIL
        for _ in range(max(1, retries)):
            comm_result = group.txPacket()
            if comm_result == self._dxl.COMM_SUCCESS:
                return

        raise DynamixelCommError(
            f"Failed to sync write address {address} on {self.port} (ids: {ids})", comm_result
        )


# --------------------------------------------------------------------------- #
# Native (C++) transport
# --------------------------------------------------------------------------- #


class NativeTransport(DynamixelTransport):
    """``robopy_dxl`` (C++ DynamixelSDK) transport."""

    backend_name = "native"

    def __init__(self, port: str, prefer_fast_sync_read: bool = True) -> None:
        super().__init__(port)
        import robopy_dxl

        self._robopy_dxl = robopy_dxl
        self._prefer_fast = prefer_fast_sync_read
        self.bus = robopy_dxl.Bus(port, DEFAULT_BAUDRATE, PROTOCOL_VERSION)

    def open(self, baudrate: int = DEFAULT_BAUDRATE) -> None:
        if self.bus.is_open:
            return
        # Keep the same Bus object across close/open so that registered group
        # handles stay valid; only the speed changes.
        self.bus.set_baudrate(baudrate)
        self.bus.open()

    def close(self) -> None:
        self.bus.close()

    @property
    def is_open(self) -> bool:
        return bool(self.bus.is_open)

    def make_read_group(
        self, address: int, num_bytes: int, ids: Sequence[int], is_signed: bool
    ) -> int:
        return int(
            self.bus.make_read_group(address, num_bytes, list(ids), is_signed, self._prefer_fast)
        )

    def make_write_group(self, address: int, num_bytes: int, ids: Sequence[int]) -> int:
        return int(self.bus.make_write_group(address, num_bytes, list(ids)))

    def sync_read(self, handle: int, retries: int) -> NDArray[np.int32]:
        try:
            return self.bus.sync_read(handle, retries)  # type: ignore[no-any-return]
        except self._robopy_dxl.DxlCommError as error:
            raise DynamixelCommError(str(error)) from error

    def sync_write(self, handle: int, values: NDArray[np.int32], retries: int) -> None:
        try:
            self.bus.sync_write(handle, np.ascontiguousarray(values, dtype=np.int32), retries)
        except self._robopy_dxl.DxlCommError as error:
            raise DynamixelCommError(str(error)) from error

    def uses_fast_sync_read(self, handle: int) -> bool:
        return bool(self.bus.group_uses_fast(handle))


# --------------------------------------------------------------------------- #
# Selection and parallel helpers
# --------------------------------------------------------------------------- #


def create_transport(port: str, backend: Backend = "auto") -> DynamixelTransport:
    """Instantiate the requested transport for ``port``.

    ``auto`` picks the native transport when ``robopy_dxl`` is installed and
    falls back to the pure-Python one otherwise.
    """
    if backend == "python":
        return PythonTransport(port)
    if backend == "native":
        return NativeTransport(port)
    if backend != "auto":
        raise ValueError(f"Unknown backend {backend!r}; expected 'auto', 'python' or 'native'.")

    if native_available():
        return NativeTransport(port)
    logger.debug(
        "robopy_dxl is not installed; using the pure-Python Dynamixel transport. "
        "Install native/robopy_dxl for Fast Sync Read and GIL-free transfers."
    )
    return PythonTransport(port)


def sync_read_parallel(
    targets: Sequence[Tuple[DynamixelTransport, int]], retries: int = 3
) -> List[NDArray[np.int32]]:
    """Read one group per transport concurrently.

    A leader and a follower arm sit on two independent USB devices, so their
    transfers can overlap.  That only pays off with the native transport, which
    releases the GIL and runs the transfers in C++ worker threads.  The Python
    SDK spins on ``readPort`` in Python bytecode and therefore holds the GIL for
    almost the whole transfer, so threading it costs more than it saves --
    measured at roughly 1.5x *slower* than reading the buses one after another.
    For non-native transports this therefore runs sequentially.
    """
    if not targets:
        return []
    if len(targets) == 1:
        transport, handle = targets[0]
        return [transport.sync_read(handle, retries)]

    distinct = {id(transport) for transport, _ in targets}
    if len(distinct) != len(targets):
        # Two groups on one bus cannot overlap, and the native helper takes the
        # bus lock per target, so running them concurrently would deadlock.
        return [transport.sync_read(handle, retries) for transport, handle in targets]

    if all(isinstance(transport, NativeTransport) for transport, _ in targets):
        import robopy_dxl

        return list(
            robopy_dxl.sync_read_parallel(
                [(transport.bus, handle) for transport, handle in targets],  # type: ignore[attr-defined]
                retries,
            )
        )

    return [transport.sync_read(handle, retries) for transport, handle in targets]
