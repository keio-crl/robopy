"""The WebSocket codec/handshake and the camera streamer of :mod:`robopy.vr`."""

from __future__ import annotations

import base64
import io
import json
import os
import socket
import struct
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Iterator, Tuple

import cv2
import numpy as np
import pytest

from robopy.vr import websocket as ws
from robopy.vr.camera import (
    CallableFrameSource,
    FrameStreamer,
    JpegEncoder,
    SyntheticFrameSource,
    to_bgr_uint8,
)


class TestCodec:
    def test_accept_key_matches_the_rfc_example(self) -> None:
        assert ws.accept_key("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="

    @pytest.mark.parametrize("length", [0, 5, 125, 126, 300, 65535, 65536, 70000])
    @pytest.mark.parametrize("masked", [False, True])
    def test_frames_round_trip_at_every_length_class(self, length: int, masked: bool) -> None:
        payload = os.urandom(length)
        mask = os.urandom(4) if masked else None
        frame = ws.encode_frame(ws.OP_BINARY, payload, mask=mask)
        header_len = 2 + (0 if length < 126 else 2 if length < 65536 else 8) + (4 if masked else 0)
        assert len(frame) == header_len + length
        fin, opcode, decoded = ws.decode_frame(io.BytesIO(frame), max_payload_bytes=1 << 20)
        assert fin and opcode == ws.OP_BINARY and decoded == payload

    def test_recv_assembles_fragments_answers_pings_and_honours_close(self) -> None:
        mask = os.urandom(4)
        stream = io.BytesIO(
            ws.encode_frame(ws.OP_TEXT, b"hel", fin=False, mask=mask)
            + ws.encode_frame(ws.OP_PING, b"p", mask=mask)  # interleaved control frame
            + ws.encode_frame(0x0, b"lo", fin=True, mask=mask)
            + ws.encode_frame(ws.OP_CLOSE, struct.pack("!H", 1001) + b"bye", mask=mask)
        )
        out = io.BytesIO()
        sock = ws.WebSocket(stream, out)
        message = sock.recv()
        assert message.opcode == ws.OP_TEXT and message.text == "hello"
        # The pong went out before the message completed.
        fin, opcode, payload = ws.decode_frame(io.BytesIO(out.getvalue()), max_payload_bytes=1024)
        assert opcode == ws.OP_PONG and payload == b"p"
        with pytest.raises(ws.WebSocketClosed) as info:
            sock.recv()
        assert info.value.code == 1001 and info.value.reason == "bye"
        assert sock.closed
        # The close was echoed with the same code.
        rest = io.BytesIO(out.getvalue())
        ws.decode_frame(rest, max_payload_bytes=1024)  # pong
        fin, opcode, payload = ws.decode_frame(rest, max_payload_bytes=1024)
        assert opcode == ws.OP_CLOSE and struct.unpack("!H", payload[:2])[0] == 1001

    def test_protocol_violations_are_errors(self) -> None:
        with pytest.raises(ws.WebSocketError, match="Reserved"):
            ws.decode_frame(io.BytesIO(b"\xc1\x00"), max_payload_bytes=10)
        with pytest.raises(ws.WebSocketError, match="exceeds"):
            ws.decode_frame(
                io.BytesIO(ws.encode_frame(ws.OP_TEXT, b"x" * 20)), max_payload_bytes=10
            )
        with pytest.raises(ws.WebSocketError, match="mid-frame"):
            ws.decode_frame(io.BytesIO(b"\x81\x05he"), max_payload_bytes=10)
        sock = ws.WebSocket(io.BytesIO(ws.encode_frame(0x0, b"x")), io.BytesIO())
        with pytest.raises(ws.WebSocketError, match="Continuation"):
            sock.recv()
        sock = ws.WebSocket(io.BytesIO(ws.encode_frame(ws.OP_PING, b"x" * 126)), io.BytesIO())
        with pytest.raises(ws.WebSocketError, match="unfragmented and short"):
            sock.recv()

    def test_send_after_close_is_refused(self) -> None:
        out = io.BytesIO()
        sock = ws.WebSocket(io.BytesIO(), out)
        sock.send_text("hi")
        sock.close(1000, "done")
        assert sock.closed
        with pytest.raises(ws.WebSocketClosed):
            sock.send_text("again")
        sock.close()  # idempotent


class _EchoHandler(BaseHTTPRequestHandler):
    """Upgrades ``/echo`` and echoes every message; anything else is a 404."""

    protocol_version = "HTTP/1.1"

    def log_message(self, *args: Any) -> None:  # noqa: D401 - quiet
        pass

    def do_GET(self) -> None:  # noqa: N802
        if self.path != "/echo":
            self.send_response(HTTPStatus.NOT_FOUND)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        try:
            sock = ws.handshake(self)
        except ws.WebSocketError as exc:
            body = str(exc).encode()
            self.send_response(HTTPStatus.BAD_REQUEST)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        try:
            while True:
                message = sock.recv()
                if message.opcode == ws.OP_TEXT:
                    sock.send_text(message.text.upper())
                else:
                    sock.send_binary(message.data[::-1])
        except ws.WebSocketError:
            pass


class RawClient:
    """A minimal client speaking the wire protocol with its own bytes."""

    def __init__(self, host: str, port: int, path: str, *, upgrade: bool = True) -> None:
        self.sock = socket.create_connection((host, port), timeout=10)
        key = base64.b64encode(os.urandom(16)).decode()
        headers = [f"GET {path} HTTP/1.1", f"Host: {host}:{port}"]
        if upgrade:
            headers += [
                "Upgrade: websocket",
                "Connection: Upgrade",
                f"Sec-WebSocket-Key: {key}",
                "Sec-WebSocket-Version: 13",
            ]
        self.sock.sendall(("\r\n".join(headers) + "\r\n\r\n").encode())
        self.rfile = self.sock.makefile("rb")
        self.status = self.rfile.readline()
        self.headers = {}
        while True:
            line = self.rfile.readline()
            if line in (b"\r\n", b""):
                break
            name, _, value = line.decode().partition(":")
            self.headers[name.strip().lower()] = value.strip()
        self.key = key

    def send(self, opcode: int, payload: bytes) -> None:
        self.sock.sendall(ws.encode_frame(opcode, payload, mask=os.urandom(4)))

    def send_json(self, obj: Any) -> None:
        self.send(ws.OP_TEXT, json.dumps(obj).encode())

    def recv(self) -> Tuple[int, bytes]:
        _fin, opcode, payload = ws.decode_frame(self.rfile, max_payload_bytes=1 << 24)
        return opcode, payload

    def recv_json(self) -> Any:
        while True:
            opcode, payload = self.recv()
            if opcode == ws.OP_TEXT:
                return json.loads(payload)

    def close(self) -> None:
        try:
            self.send(ws.OP_CLOSE, struct.pack("!H", 1000))
        except OSError:
            pass
        self.sock.close()


@pytest.fixture
def echo_server() -> Iterator[Tuple[str, int]]:
    server = ThreadingHTTPServer(("127.0.0.1", 0), _EchoHandler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server.server_address[0], server.server_address[1]
    server.shutdown()
    server.server_close()


class TestHandshakeOverHttp:
    def test_upgrade_and_echo(self, echo_server: Tuple[str, int]) -> None:
        client = RawClient(*echo_server, "/echo")
        assert b"101" in client.status
        assert client.headers["upgrade"] == "websocket"
        assert client.headers["sec-websocket-accept"] == ws.accept_key(client.key)
        client.send_json({"a": 1})
        opcode, payload = client.recv()
        assert opcode == ws.OP_TEXT and payload == b'{"A": 1}'
        client.send(ws.OP_BINARY, b"\x01\x02\x03")
        opcode, payload = client.recv()
        assert opcode == ws.OP_BINARY and payload == b"\x03\x02\x01"
        client.close()

    def test_plain_get_is_rejected(self, echo_server: Tuple[str, int]) -> None:
        client = RawClient(*echo_server, "/echo", upgrade=False)
        assert b"400" in client.status
        client.sock.close()


class TestCameraPipeline:
    def test_to_bgr_uint8_handles_layouts_and_ranges(self) -> None:
        chw = np.zeros((3, 4, 5), dtype=np.float32)
        chw[0] = 1.0  # red channel full, RGB order
        out = to_bgr_uint8(chw, color="rgb")
        assert out.shape == (4, 5, 3) and out.dtype == np.uint8
        assert out[0, 0].tolist() == [0, 0, 255]  # red lands in the B-G-R last slot
        hwc = np.full((2, 2, 3), 200.0, dtype=np.float64)  # already 0..255 floats
        assert to_bgr_uint8(hwc, color="bgr")[0, 0].tolist() == [200, 200, 200]
        with pytest.raises(ValueError):
            to_bgr_uint8(np.zeros((4, 5)))
        with pytest.raises(ValueError):
            to_bgr_uint8(np.zeros((4, 5, 3)), color="hsv")

    def test_synthetic_frames_encode_and_decode(self) -> None:
        captions = iter(["yaw=0.1", "yaw=0.2"])
        source = SyntheticFrameSource(320, 240, caption=lambda: next(captions))
        frame = source.read()
        assert frame is not None and frame.shape == (240, 320, 3) and frame.dtype == np.uint8
        data, w, h = JpegEncoder(quality=80).encode(frame)
        assert data[:2] == b"\xff\xd8" and (w, h) == (320, 240)
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape == (240, 320, 3)
        small, w, h = JpegEncoder(max_width=160).encode(frame)
        assert (w, h) == (160, 120)
        with pytest.raises(ValueError):
            JpegEncoder(quality=0)

    def test_callable_source_adapts_and_passes_none(self) -> None:
        frames: list[Any] = [None, np.zeros((3, 8, 8), dtype=np.float32)]
        closed = []
        source = CallableFrameSource(
            lambda: frames.pop(0), color="rgb", close=lambda: closed.append(1)
        )
        assert source.read() is None
        got = source.read()
        assert got is not None and got.shape == (8, 8, 3)
        source.close()
        assert closed == [1]

    def test_streamer_keeps_only_the_newest_frame(self) -> None:
        streamer = FrameStreamer(SyntheticFrameSource(64, 48), fps=200.0)
        assert streamer.latest is None
        assert streamer.wait_for(0, timeout_s=0.01) is None
        first = streamer.capture_once()
        second = streamer.capture_once()
        assert first is not None and second is not None
        assert (first.seq, second.seq) == (1, 2)
        assert streamer.latest is second
        assert streamer.wait_for(1, timeout_s=0.01) is second
        assert streamer.wait_for(2, timeout_s=0.01) is None  # nothing newer yet
        streamer.start()
        try:
            newer = streamer.wait_for(2, timeout_s=2.0)
            assert newer is not None and newer.seq > 2
            assert streamer.describe()["running"]
        finally:
            streamer.stop()
        assert not streamer.running

    def test_streamer_survives_a_failing_source(self) -> None:
        class Flaky:
            calls = 0

            def read(self) -> Any:
                self.calls += 1
                if self.calls % 2:
                    raise RuntimeError("device hiccup")
                return np.zeros((16, 16, 3), dtype=np.uint8)

            def close(self) -> None:
                pass

        streamer = FrameStreamer(Flaky(), fps=100.0)
        assert streamer.capture_once() is None
        assert streamer.capture_once() is not None
        assert streamer.describe()["read_failures"] == 1
        time.sleep(0)  # no thread was started; nothing to stop
