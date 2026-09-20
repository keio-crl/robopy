"""Camera path: rotation, the single-threaded camera socket, RealSense recovery."""

from __future__ import annotations

import threading
import time
from typing import Any, Iterator, Tuple

import numpy as np
import pytest

pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.viewer.model_bundle import ModelBundle  # noqa: E402
from robopy.viewer.server import IKSetup  # noqa: E402
from robopy.vr import websocket as ws  # noqa: E402
from robopy.vr.__main__ import STREAMING_IK_OVERRIDES  # noqa: E402
from robopy.vr.backend import SimulationBackend  # noqa: E402
from robopy.vr.camera import (  # noqa: E402
    FrameStreamer,
    JpegEncoder,
    RealsenseFrameSource,
    RotatedFrameSource,
    SyntheticFrameSource,
    rotate_frame,
)
from robopy.vr.server import VRServer, VRServerConfig  # noqa: E402

from .test_vr_server import SOFT_LIMITS  # noqa: E402
from .test_vr_transport import RawClient  # noqa: E402


class TestRotation:
    def test_rotate_frame_turns_clockwise(self) -> None:
        frame = np.zeros((2, 3, 3), dtype=np.uint8)
        frame[0, 0] = (1, 2, 3)  # top-left pixel
        assert rotate_frame(frame, 0) is frame
        r180 = rotate_frame(frame, 180)
        assert r180.shape == (2, 3, 3) and tuple(r180[1, 2]) == (1, 2, 3)
        r90 = rotate_frame(frame, 90)
        assert r90.shape == (3, 2, 3) and tuple(r90[0, 1]) == (1, 2, 3)
        assert rotate_frame(frame, 450).shape == (3, 2, 3)  # 450 == 90
        with pytest.raises(ValueError):
            rotate_frame(frame, 45)

    def test_rotation_happens_at_the_source_and_nowhere_downstream(self) -> None:
        source = SyntheticFrameSource(64, 32)
        upright = FrameStreamer(source, fps=30.0, encoder=JpegEncoder(90)).capture_once()
        turned_source = RotatedFrameSource(source, 90)
        turned = FrameStreamer(turned_source, fps=30.0, encoder=JpegEncoder(90)).capture_once()
        assert upright is not None and turned is not None
        assert (upright.width, upright.height) == (64, 32)
        assert (turned.width, turned.height) == (32, 64)
        # The streamer has no rotation setting: only the source can turn a picture.
        assert not hasattr(FrameStreamer(source), "rotate_deg")
        assert "rotate" not in FrameStreamer(source).describe()
        with pytest.raises(ValueError):
            RotatedFrameSource(source, 30)
        assert RotatedFrameSource(source, 360).degrees == 0


class TestRealsenseRecovery:
    def test_failures_are_swallowed_and_the_pipeline_restarts(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        calls: list[str] = []

        class FakeCamera:
            def __init__(self, config: Any) -> None:
                self.dead = False

            def connect(self) -> None:
                calls.append("connect")
                self.dead = False

            def disconnect(self) -> None:
                calls.append("disconnect")

            def async_read(self, timeout_ms: float) -> Any:
                if self.dead:
                    raise RuntimeError("Frame didn't arrive")
                raise TimeoutError("nothing new")

        monkeypatch.setattr("robopy.sensors.visual.realsense_camera.RealsenseCamera", FakeCamera)
        source = RealsenseFrameSource(reconnect_after_s=0.05)
        assert source.read() is None and source.failures == 1  # a timeout: quiet
        source._camera.dead = True
        assert source.read() is None  # a device error: warned, still None
        time.sleep(0.06)
        assert source.read() is None
        assert source.reconnects == 1 and calls == ["connect", "disconnect", "connect"]
        assert source.read() is None and source.reconnects == 1  # one attempt per interval


@pytest.fixture(scope="module")
def camera_server(synthetic_urdf: Any) -> Iterator[VRServer]:
    bundle = ModelBundle.load(synthetic_urdf, soft_limits=SOFT_LIMITS)
    ik = IKSetup(bundle, config_overrides=STREAMING_IK_OVERRIDES)
    camera = FrameStreamer(RotatedFrameSource(SyntheticFrameSource(64, 48), 180), fps=60.0)
    srv = VRServer(
        bundle,
        ik=ik,
        backend=SimulationBackend(bundle, ik),
        head_tracker=None,
        arm_teleop=None,
        camera=camera,
        host="127.0.0.1",
        port=0,
        config=VRServerConfig(state_hz=60.0, head_enabled=False),
    )
    camera.start()
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    yield srv
    camera.stop()
    srv.shutdown()
    srv.server_close()


def _address(server: VRServer) -> Tuple[str, int]:
    return server.server_address[0], server.server_address[1]


class TestCameraSocket:
    def test_pings_are_answered_while_frames_keep_flowing(self, camera_server: VRServer) -> None:
        host, port = _address(camera_server)
        client = RawClient(host, port, "/ws/camera")
        try:
            greeting = client.recv_json()
            assert greeting["type"] == "camera" and "rotate_deg" not in greeting
            frames = 0
            pongs = 0
            deadline = time.monotonic() + 3.0
            client.send(ws.OP_PING, b"hi")
            while time.monotonic() < deadline and (frames < 5 or pongs < 1):
                opcode, payload = client.recv()
                if opcode == ws.OP_BINARY:
                    frames += 1
                    assert payload[:2] == b"\xff\xd8"  # JPEG
                elif opcode == ws.OP_PONG:
                    pongs += 1
                    assert payload == b"hi"
            assert frames >= 5 and pongs == 1
            assert camera_server.status()["camera_clients"] == 1
        finally:
            client.close()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and camera_server.status()["camera_clients"] != 0:
            time.sleep(0.05)
        assert camera_server.status()["camera_clients"] == 0  # the close was honoured

    def test_recv_control_only_returns_after_one_control_frame(self) -> None:
        import io

        # A ping followed by nothing: control_only returns None after the ping.
        ping = ws.encode_frame(ws.OP_PING, b"x", mask=b"\x01\x02\x03\x04")
        sent = io.BytesIO()
        sock = ws.WebSocket(io.BytesIO(ping), sent)
        assert sock.recv(control_only=True) is None
        assert sent.getvalue()[:1] == bytes([0x80 | ws.OP_PONG])
