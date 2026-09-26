"""A minimal server-side WebSocket (RFC 6455) on top of the stdlib HTTP handler.

The viewer server is a ``ThreadingHTTPServer``; each connection already has a
thread and buffered ``rfile`` / ``wfile``.  Upgrading such a connection needs
only the handshake and frame codec, which is what this module provides -- no
event loop, no extra dependency.  It implements what a browser client uses:
text and binary messages, fragmentation, ping/pong, close, and masking of
client frames.  Extensions (compression) are not negotiated.
"""

from __future__ import annotations

import base64
import hashlib
import os
import struct
import threading
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler
from typing import Any, BinaryIO, Literal, Mapping, cast, overload

__all__ = [
    "OP_BINARY",
    "OP_CLOSE",
    "OP_PING",
    "OP_PONG",
    "OP_TEXT",
    "Message",
    "WebSocket",
    "WebSocketClosed",
    "WebSocketError",
    "accept_key",
    "decode_frame",
    "encode_frame",
    "handshake",
    "is_upgrade_request",
]

_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
OP_CONTINUATION = 0x0
OP_TEXT = 0x1
OP_BINARY = 0x2
OP_CLOSE = 0x8
OP_PING = 0x9
OP_PONG = 0xA


class WebSocketError(Exception):
    """A protocol violation or a broken connection."""


class WebSocketClosed(WebSocketError):
    """The peer closed the connection (or we did)."""

    def __init__(self, code: int | None = None, reason: str = "") -> None:
        super().__init__(f"websocket closed (code={code}, reason={reason!r})")
        self.code = code
        self.reason = reason


@dataclass(frozen=True)
class Message:
    """A complete (defragmented) data message."""

    opcode: int
    data: bytes

    @property
    def text(self) -> str:
        """The payload decoded as UTF-8 (text messages)."""
        return self.data.decode("utf-8")


def accept_key(client_key: str) -> str:
    """``Sec-WebSocket-Accept`` for a client's ``Sec-WebSocket-Key``."""
    digest = hashlib.sha1(client_key.strip().encode("ascii") + _GUID).digest()
    return base64.b64encode(digest).decode("ascii")


def is_upgrade_request(headers: Mapping[str, str] | Any) -> bool:
    """Whether the request headers ask for a WebSocket upgrade."""
    upgrade = headers.get("Upgrade", "").lower()
    connection = headers.get("Connection", "").lower()
    return upgrade == "websocket" and "upgrade" in connection


def handshake(handler: BaseHTTPRequestHandler, *, max_message_bytes: int = 16 << 20) -> "WebSocket":
    """Complete the opening handshake on ``handler`` and return the socket.

    Raises:
        WebSocketError: If the request is not a valid upgrade request.  The
            caller should answer with a 400 in that case.
    """
    headers = handler.headers
    if not is_upgrade_request(headers):
        raise WebSocketError("Not a WebSocket upgrade request.")
    if headers.get("Sec-WebSocket-Version", "") != "13":
        raise WebSocketError("Only WebSocket version 13 is supported.")
    key = headers.get("Sec-WebSocket-Key")
    if not key:
        raise WebSocketError("Missing Sec-WebSocket-Key.")
    try:
        if len(base64.b64decode(key, validate=True)) != 16:
            raise WebSocketError("Sec-WebSocket-Key must decode to 16 bytes.")
    except ValueError as exc:
        raise WebSocketError("Sec-WebSocket-Key is not valid base64.") from exc
    handler.send_response(HTTPStatus.SWITCHING_PROTOCOLS)
    handler.send_header("Upgrade", "websocket")
    handler.send_header("Connection", "Upgrade")
    handler.send_header("Sec-WebSocket-Accept", accept_key(key))
    handler.end_headers()
    handler.wfile.flush()
    handler.close_connection = True
    return WebSocket(
        cast(BinaryIO, handler.rfile),
        cast(BinaryIO, handler.wfile),
        max_message_bytes=max_message_bytes,
    )


def encode_frame(
    opcode: int, payload: bytes, *, fin: bool = True, mask: bytes | None = None
) -> bytes:
    """Serialise one frame.  ``mask`` (4 bytes) is used by clients only."""
    header = bytearray()
    header.append((0x80 if fin else 0x00) | (opcode & 0x0F))
    length = len(payload)
    mask_bit = 0x80 if mask is not None else 0x00
    if length < 126:
        header.append(mask_bit | length)
    elif length < 1 << 16:
        header.append(mask_bit | 126)
        header += struct.pack("!H", length)
    else:
        header.append(mask_bit | 127)
        header += struct.pack("!Q", length)
    if mask is not None:
        if len(mask) != 4:
            raise ValueError("A frame mask is 4 bytes.")
        header += mask
        payload = _apply_mask(payload, mask)
    return bytes(header) + payload


def _apply_mask(payload: bytes, mask: bytes) -> bytes:
    if not payload:
        return payload
    repeats = len(payload) // 4 + 1
    key = int.from_bytes(mask * repeats, "big") >> (8 * (4 * repeats - len(payload)))
    return (int.from_bytes(payload, "big") ^ key).to_bytes(len(payload), "big")


def _read_exact(stream: BinaryIO, n: int) -> bytes:
    data = bytearray()
    while len(data) < n:
        chunk = stream.read(n - len(data))
        if not chunk:
            raise WebSocketError("Connection closed mid-frame.")
        data += chunk
    return bytes(data)


def decode_frame(stream: BinaryIO, *, max_payload_bytes: int) -> tuple[bool, int, bytes]:
    """Read one frame from ``stream``; returns ``(fin, opcode, unmasked payload)``."""
    first, second = _read_exact(stream, 2)
    fin = bool(first & 0x80)
    if first & 0x70:
        raise WebSocketError("Reserved bits set without a negotiated extension.")
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", _read_exact(stream, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", _read_exact(stream, 8))[0]
    if length > max_payload_bytes:
        raise WebSocketError(f"Frame of {length} bytes exceeds the {max_payload_bytes} byte limit.")
    mask = _read_exact(stream, 4) if masked else None
    payload = _read_exact(stream, length) if length else b""
    if mask is not None:
        payload = _apply_mask(payload, mask)
    return fin, opcode, payload


class WebSocket:
    """One upgraded connection.  ``recv`` runs on the connection's thread; sends
    may come from any thread (they are serialised by a lock)."""

    def __init__(
        self, rfile: BinaryIO, wfile: BinaryIO, *, max_message_bytes: int = 16 << 20
    ) -> None:
        self._rfile = rfile
        self._wfile = wfile
        self._max = max_message_bytes
        self._send_lock = threading.Lock()
        self._closed = False
        self.close_code: int | None = None

    @property
    def closed(self) -> bool:
        """Whether a close frame has been sent or received."""
        return self._closed

    # -- sending ----------------------------------------------------------

    def _send_frame(self, opcode: int, payload: bytes) -> None:
        data = encode_frame(opcode, payload)
        with self._send_lock:
            if self._closed and opcode != OP_CLOSE:
                raise WebSocketClosed(self.close_code)
            try:
                self._wfile.write(data)
                self._wfile.flush()
            except (OSError, ValueError) as exc:
                self._closed = True
                raise WebSocketError(f"send failed: {exc}") from exc

    def send_text(self, text: str) -> None:
        """Send a text message."""
        self._send_frame(OP_TEXT, text.encode("utf-8"))

    def send_binary(self, data: bytes) -> None:
        """Send a binary message."""
        self._send_frame(OP_BINARY, bytes(data))

    def ping(self, payload: bytes = b"") -> None:
        """Send a ping."""
        self._send_frame(OP_PING, payload)

    def close(self, code: int = 1000, reason: str = "") -> None:
        """Send a close frame (idempotent)."""
        if self._closed:
            return
        payload = struct.pack("!H", code) + reason.encode("utf-8")[:123]
        try:
            self._send_frame(OP_CLOSE, payload)
        except WebSocketError:
            pass
        self._closed = True
        self.close_code = code

    # -- receiving --------------------------------------------------------

    @overload
    def recv(self) -> Message: ...

    @overload
    def recv(self, *, control_only: Literal[True]) -> Message | None: ...

    def recv(self, *, control_only: bool = False) -> Message | None:
        """Block until a complete data message arrives.

        Control frames are handled here: pings are answered, pongs ignored, and
        a close frame is echoed before :class:`WebSocketClosed` is raised.

        Args:
            control_only: Return ``None`` as soon as one control frame has
                been handled instead of waiting on for a data message.  For a
                sender that only wants to service the peer's pings and closes
                when its socket becomes readable, without a second thread on
                the same (possibly TLS) socket.

        Raises:
            WebSocketClosed: When the peer closes the connection.
            WebSocketError: On a protocol violation or a dead connection.
        """
        if self._closed:
            raise WebSocketClosed(self.close_code)
        fragments: list[bytes] = []
        message_opcode: int | None = None
        total = 0
        while True:
            try:
                fin, opcode, payload = decode_frame(self._rfile, max_payload_bytes=self._max)
            except (OSError, ValueError) as exc:
                self._closed = True
                raise WebSocketError(f"recv failed: {exc}") from exc
            if opcode in (OP_PING, OP_PONG, OP_CLOSE):
                if not fin or len(payload) > 125:
                    raise WebSocketError("Control frames must be unfragmented and short.")
                if opcode == OP_PING:
                    self._send_frame(OP_PONG, payload)
                elif opcode == OP_CLOSE:
                    code = struct.unpack("!H", payload[:2])[0] if len(payload) >= 2 else None
                    reason = payload[2:].decode("utf-8", "replace")
                    if not self._closed:
                        try:
                            self._send_frame(OP_CLOSE, payload[:2])
                        except WebSocketError:
                            pass
                    self._closed = True
                    self.close_code = code
                    raise WebSocketClosed(code, reason)
                if control_only and message_opcode is None:
                    return None
                continue
            if opcode in (OP_TEXT, OP_BINARY):
                if message_opcode is not None:
                    raise WebSocketError("New data frame while a fragmented message is open.")
                message_opcode = opcode
            elif opcode == OP_CONTINUATION:
                if message_opcode is None:
                    raise WebSocketError("Continuation frame without a message.")
            else:
                raise WebSocketError(f"Unknown opcode {opcode:#x}.")
            total += len(payload)
            if total > self._max:
                raise WebSocketError("Message exceeds the size limit.")
            fragments.append(payload)
            if fin:
                assert message_opcode is not None
                return Message(message_opcode, b"".join(fragments))


def client_mask() -> bytes:
    """A random 4-byte mask, for tests that speak the client side."""
    return os.urandom(4)
