"""An in-process Dynamixel protocol 2.0 device simulator on a pseudo terminal.

The simulator lets the Dynamixel transports be exercised end to end without any
hardware: it allocates a pty, hands the slave path out as ``port`` (so it can be
passed straight to ``DynamixelBus`` or ``robopy_dxl.Bus``), and answers real
protocol 2.0 instruction packets from a background thread.

Supported instructions: PING, READ, WRITE, SYNC_READ, SYNC_WRITE and
FAST_SYNC_READ.  ``supports_fast_sync_read=False`` makes the devices ignore
``0x8A`` the way pre-fast-sync-read firmware does, which is what the native
transport's automatic downgrade is tested against.

``return_delay_us`` emulates the per-motor Return Delay Time.  It is what makes
a plain sync read cost O(N) delays while a fast sync read costs one, so
benchmarks against the simulator reproduce the shape of the real bus even though
they cannot reproduce its wire time.
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from types import TracebackType
from typing import Dict, Iterable, List, Type

BROADCAST_ID = 0xFE

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_STATUS = 0x55
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83
INST_FAST_SYNC_READ = 0x8A

HEADER = b"\xff\xff\xfd\x00"
CONTROL_TABLE_SIZE = 512


def _make_crc_table() -> List[int]:
    """CRC-16 with polynomial 0x8005, MSB first, as used by protocol 2.0."""
    table = []
    for index in range(256):
        crc = index << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = _make_crc_table()


def crc16(data: bytes, crc: int = 0) -> int:
    for byte in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ byte) & 0xFF]) & 0xFFFF
    return crc


def add_stuffing(payload: bytes) -> bytes:
    """Insert the 0xFD escape byte after every 0xFF 0xFF 0xFD in the payload."""
    out = bytearray()
    for byte in payload:
        out.append(byte)
        if len(out) >= 3 and out[-3] == 0xFF and out[-2] == 0xFF and out[-1] == 0xFD:
            out.append(0xFD)
    return bytes(out)


def remove_stuffing(payload: bytes) -> bytes:
    """Inverse of :func:`add_stuffing`."""
    out = bytearray()
    skip_next = False
    for byte in payload:
        if skip_next:
            skip_next = False
            continue
        out.append(byte)
        if len(out) >= 3 and out[-3] == 0xFF and out[-2] == 0xFF and out[-1] == 0xFD:
            skip_next = True
    return bytes(out)


def build_packet(motor_id: int, instruction: int, params: bytes) -> bytes:
    """Wrap an instruction/status payload in a full protocol 2.0 packet."""
    body = add_stuffing(bytes([instruction]) + params)
    length = len(body) + 2  # payload + CRC
    packet = bytearray(HEADER)
    packet.append(motor_id)
    packet.append(length & 0xFF)
    packet.append((length >> 8) & 0xFF)
    packet += body
    crc = crc16(bytes(packet))
    packet.append(crc & 0xFF)
    packet.append((crc >> 8) & 0xFF)
    return bytes(packet)


@dataclass
class FakeMotor:
    """One simulated motor: a flat, byte addressable control table."""

    motor_id: int
    model_number: int = 1020  # xm430-w350
    firmware_version: int = 52
    memory: bytearray = field(default_factory=lambda: bytearray(CONTROL_TABLE_SIZE))

    def read(self, address: int, length: int) -> bytes:
        return bytes(self.memory[address : address + length])

    def write(self, address: int, data: bytes) -> None:
        self.memory[address : address + len(data)] = data

    def set_value(self, address: int, value: int, length: int) -> None:
        self.write(address, (value & ((1 << (8 * length)) - 1)).to_bytes(length, "little"))

    def get_value(self, address: int, length: int, signed: bool = False) -> int:
        return int.from_bytes(self.read(address, length), "little", signed=signed)


class FakeDynamixelBus:
    """A pty backed bus carrying a set of :class:`FakeMotor` devices."""

    def __init__(
        self,
        motor_ids: Iterable[int],
        *,
        model_number: int = 1020,
        return_delay_us: int = 0,
        supports_fast_sync_read: bool = True,
    ) -> None:
        self.motors: Dict[int, FakeMotor] = {
            motor_id: FakeMotor(motor_id, model_number=model_number) for motor_id in motor_ids
        }
        self.return_delay_us = return_delay_us
        self.supports_fast_sync_read = supports_fast_sync_read

        # Instruction counters, useful for asserting *which* path was taken.
        self.instruction_counts: Dict[int, int] = {}

        self._master_fd, self._slave_fd = os.openpty()
        self.port: str = os.ttyname(self._slave_fd)
        os.set_blocking(self._master_fd, False)

        self._buffer = bytearray()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, name="fake-dxl", daemon=True)

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> "FakeDynamixelBus":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        for fd in (self._master_fd, self._slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass

    def __enter__(self) -> "FakeDynamixelBus":
        return self.start()

    def __exit__(
        self,
        exc_type: Type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self.stop()

    # -- helpers -----------------------------------------------------------

    def set_value(self, motor_id: int, address: int, value: int, length: int) -> None:
        self.motors[motor_id].set_value(address, value, length)

    def set_all(self, address: int, value: int, length: int) -> None:
        for motor in self.motors.values():
            motor.set_value(address, value, length)

    def get_value(self, motor_id: int, address: int, length: int, signed: bool = False) -> int:
        return self.motors[motor_id].get_value(address, length, signed=signed)

    # -- server ------------------------------------------------------------

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = os.read(self._master_fd, 4096)
            except BlockingIOError:
                time.sleep(0.0002)
                continue
            except OSError:
                return
            if not chunk:
                time.sleep(0.0002)
                continue
            self._buffer += chunk
            self._drain()

    def _drain(self) -> None:
        while True:
            start = self._buffer.find(HEADER)
            if start < 0:
                # Keep the last 3 bytes: a header may be split across reads.
                del self._buffer[: max(0, len(self._buffer) - 3)]
                return
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 7:
                return
            length = self._buffer[5] | (self._buffer[6] << 8)
            total = length + 7
            if len(self._buffer) < total:
                return
            packet = bytes(self._buffer[:total])
            del self._buffer[:total]

            expected = crc16(packet[:-2])
            if expected != (packet[-2] | (packet[-1] << 8)):
                continue  # corrupt packet: a real motor stays silent
            payload = remove_stuffing(packet[7:-2])
            self._dispatch(packet[4], payload[0], payload[1:])

    def _dispatch(self, motor_id: int, instruction: int, params: bytes) -> None:
        self.instruction_counts[instruction] = self.instruction_counts.get(instruction, 0) + 1

        if instruction == INST_PING:
            targets = list(self.motors) if motor_id == BROADCAST_ID else [motor_id]
            for target in targets:
                motor = self.motors.get(target)
                if motor is None:
                    continue
                self._respond(
                    target,
                    bytes(
                        [
                            motor.model_number & 0xFF,
                            (motor.model_number >> 8) & 0xFF,
                            motor.firmware_version,
                        ]
                    ),
                )
        elif instruction == INST_READ:
            motor = self.motors.get(motor_id)
            if motor is None:
                return
            address = params[0] | (params[1] << 8)
            length = params[2] | (params[3] << 8)
            self._respond(motor_id, motor.read(address, length))
        elif instruction == INST_WRITE:
            motor = self.motors.get(motor_id)
            if motor is None:
                return
            address = params[0] | (params[1] << 8)
            motor.write(address, params[2:])
            self._respond(motor_id, b"")
        elif instruction == INST_SYNC_READ:
            address = params[0] | (params[1] << 8)
            length = params[2] | (params[3] << 8)
            for target in params[4:]:
                motor = self.motors.get(target)
                if motor is None:
                    continue
                self._respond(target, motor.read(address, length))
        elif instruction == INST_SYNC_WRITE:
            address = params[0] | (params[1] << 8)
            length = params[2] | (params[3] << 8)
            cursor = 4
            while cursor + 1 + length <= len(params):
                target = params[cursor]
                data = params[cursor + 1 : cursor + 1 + length]
                cursor += 1 + length
                motor = self.motors.get(target)
                if motor is not None:
                    motor.write(address, data)
        elif instruction == INST_FAST_SYNC_READ:
            if not self.supports_fast_sync_read:
                return  # pre-0x8A firmware: no answer at all
            address = params[0] | (params[1] << 8)
            length = params[2] | (params[3] << 8)
            self._respond_fast(address, length, list(params[4:]))

    def _respond(self, motor_id: int, data: bytes, error: int = 0) -> None:
        if self.return_delay_us:
            time.sleep(self.return_delay_us / 1_000_000)
        self._write(build_packet(motor_id, INST_STATUS, bytes([error]) + data))

    def _respond_fast(self, address: int, length: int, ids: List[int]) -> None:
        # One broadcast status packet carrying <error, id, data, crc> per motor.
        # Only the trailing CRC is checked by the SDK (it is the packet CRC);
        # the per-device ones are filled in for realism.
        if self.return_delay_us:
            time.sleep(self.return_delay_us / 1_000_000)

        blocks = bytearray()
        for target in ids:
            motor = self.motors.get(target)
            if motor is None:
                continue
            block = bytes([0x00, target]) + motor.read(address, length)
            block_crc = crc16(block)
            blocks += block + bytes([block_crc & 0xFF, (block_crc >> 8) & 0xFF])

        packet = bytearray(HEADER)
        packet.append(BROADCAST_ID)
        # No byte stuffing here: the SDK receives fast status packets with
        # `skip_stuffing=True`, so real devices do not stuff them either.
        body = bytes([INST_STATUS]) + bytes(blocks[:-2])
        # The last block's CRC slot doubles as the packet CRC.
        total_length = len(body) + 2
        packet.append(total_length & 0xFF)
        packet.append((total_length >> 8) & 0xFF)
        packet += body
        crc = crc16(bytes(packet))
        packet.append(crc & 0xFF)
        packet.append((crc >> 8) & 0xFF)
        self._write(bytes(packet))

    def _write(self, data: bytes) -> None:
        try:
            os.write(self._master_fd, data)
        except OSError:
            pass
