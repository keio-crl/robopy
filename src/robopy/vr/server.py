"""The VR server: the viewer's HTTP server plus two WebSockets.

Endpoints, in addition to those of :class:`robopy.viewer.server.ViewerServer`
(the page assets, meshes, ``/api/model`` and ``/api/fk`` are shared):

``GET /vr``               the WebXR page
``GET /ws/teleop``        WebSocket; the operator streams poses in, state comes back
``GET /ws/camera``        WebSocket; binary JPEG frames out (newest only)
``GET /api/vr/status``    everything about the session, for humans and tests

Protocol of ``/ws/teleop`` (JSON text messages)
-----------------------------------------------
Client -> server::

    {"type": "hello", "want_poses": true}
    {"type": "pose", "t": <ms>,
     "head":  {"p": [x, y, z], "q": [x, y, z, w]} | null,
     "left":  {"p": [...], "q": [...], "clutch": bool, "trigger": 0..1,
               "buttons": {"a": bool, "b": bool, "stick": bool}} | null,
     "right": {...} | null}
    {"type": "recenter"}
    {"type": "set", "head_enabled": bool, "arms_enabled": bool,
     "position_scale": float, "orientation_enabled": bool, "want_poses": bool}

Poses are WebXR poses (metres, ``xyzw`` quaternion) in the page's reference
space; the server does every frame conversion so the page stays dumb.

Server -> client::

    {"type": "hello", "model": <describe()>, "head": <mapping>, "arms": <config>,
     "backend": "simulation"|"hardware", "camera": {...}, "twin_offset_m": [...]}
    {"type": "state", "seq": n, "t": <echoed ms>, "joints": {...},
     "geometries": [{"p": [...], "q": [...]}, ...]   (only when want_poses),
     "tcp": {"left": {...}, "right": {...}},
     "head": {...}, "arms": {...}, "ik": {...}, "warnings": [...], "server_ms": float}
    {"type": "error", "message": "..."}

Exactly one operator may stream at a time; a second ``/ws/teleop`` connection
is refused.  When the operator's socket closes (or goes silent), both clutches
are released and the backend is told to hold.
"""

from __future__ import annotations

import json
import logging
import socket
import ssl
import threading
import time
import webbrowser
from dataclasses import dataclass
from http import HTTPStatus
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Tuple
from urllib.parse import urlsplit

import numpy as np
from numpy.typing import NDArray

from robopy.viewer.model_bundle import ModelBundle, matrix_to_pose
from robopy.viewer.server import IKSetup, ViewerServer, _Handler

from .arm_teleop import ControllerSample, DualArmTeleop
from .backend import TeleopBackend, TeleopCommand
from .camera import FrameStreamer
from .head_tracking import HeadTracker
from .websocket import (
    OP_TEXT,
    WebSocket,
    WebSocketClosed,
    WebSocketError,
    handshake,
    is_upgrade_request,
)
from .xr_math import OperatorFrame, xr_pose_to_robot

logger = logging.getLogger(__name__)

__all__ = ["TeleopSession", "VRServer", "VRServerConfig", "serve_vr"]

_STATIC_DIR = Path(__file__).parent / "static"


@dataclass
class VRServerConfig:
    """Session-level settings the page and the session share.

    Attributes:
        state_hz: Ceiling on state messages sent back to the operator.
        teleop_timeout_s: A teleop socket silent this long is treated as gone
            (clutches released, backend told to hold).
        camera_fov_deg: Horizontal field of view the page uses to size the
            camera image.  The default is the Intel RealSense D435 colour
            sensor's data-sheet value; it is *not* a calibration of this camera.
        twin_offset_m: Where the page draws the robot's base relative to the
            operator's re-centred origin (x forward, y left, z up).  Purely
            visual; the teleoperation is relative and does not depend on it.
        head_enabled: Whether the head follows the headset at session start.
        arms_enabled: Whether the arms follow the controllers at session start.
    """

    state_hz: float = 30.0
    teleop_timeout_s: float = 5.0
    camera_fov_deg: float = 69.0
    twin_offset_m: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    head_enabled: bool = True
    arms_enabled: bool = True

    def __post_init__(self) -> None:
        if self.state_hz <= 0.0 or self.teleop_timeout_s <= 0.0:
            raise ValueError("state_hz and teleop_timeout_s must be positive.")
        if not 10.0 <= self.camera_fov_deg <= 170.0:
            raise ValueError("camera_fov_deg must be within [10, 170].")


def _pose_entry(entry: Any) -> NDArray[np.float64] | None:
    """A WebXR ``{"p": [3], "q": [4]}`` entry as a robot-frame transform, or ``None``."""
    if not isinstance(entry, dict):
        return None
    p, q = entry.get("p"), entry.get("q")
    if not (isinstance(p, list) and isinstance(q, list) and len(p) == 3 and len(q) == 4):
        return None
    values = [*p, *q]
    if not all(isinstance(v, (int, float)) and np.isfinite(v) for v in values):
        return None
    try:
        return xr_pose_to_robot(p, q)
    except ValueError:
        return None


class TeleopSession:
    """Turn one operator's messages into backend commands.

    Not thread-safe by itself; the server calls it under its model lock.
    """

    def __init__(
        self,
        backend: TeleopBackend,
        *,
        head_tracker: HeadTracker | None,
        arm_teleop: DualArmTeleop | None,
        config: VRServerConfig,
        bundle: ModelBundle | None = None,
    ) -> None:
        """Create a session.

        Args:
            backend: Where commands go.
            head_tracker: Head mapping, or ``None`` for an arms-only session.
            arm_teleop: Arm mapping, or ``None`` for a head-only session.
            config: Session settings.
            bundle: The model bundle, for geometry poses sent to the page.
        """
        self.backend = backend
        self.head_tracker = head_tracker
        self.arm_teleop = arm_teleop
        self.config = config
        self.bundle = bundle
        self.operator = OperatorFrame()
        self.head_enabled = config.head_enabled
        self.arms_enabled = config.arms_enabled
        self.want_poses = False
        self.seq = 0
        self.messages = 0
        self.started_s = time.monotonic()
        self._last_head_robot: NDArray[np.float64] | None = None
        self._last_state_s = -1.0
        self._last_head: Dict[str, Any] = {}
        self._last_arms: Dict[str, Any] = {}
        self._last_report: Any = None
        if head_tracker is not None:
            head_tracker.reset(backend.joint_positions())

    # -- messages -----------------------------------------------------------

    def hello(self) -> Dict[str, Any]:
        """The greeting: everything the page needs to build itself."""
        return {
            "type": "hello",
            "backend": self.backend.name,
            "model": None if self.bundle is None else self.bundle.describe(),
            "head": None if self.head_tracker is None else self.head_tracker.describe(),
            "arms": None if self.arm_teleop is None else self.arm_teleop.describe(),
            "head_enabled": self.head_enabled,
            "arms_enabled": self.arms_enabled,
            "camera_fov_deg": self.config.camera_fov_deg,
            "twin_offset_m": list(self.config.twin_offset_m),
            "state_hz": self.config.state_hz,
            "backend_info": self.backend.describe(),
        }

    def handle(self, message: Mapping[str, Any], now_s: float) -> Dict[str, Any] | None:
        """Process one client message; returns the reply to send, if any."""
        self.messages += 1
        kind = message.get("type")
        if kind == "hello":
            self.want_poses = bool(message.get("want_poses", False))
            return self.hello()
        if kind == "pose":
            return self._handle_pose(message, now_s)
        if kind == "recenter":
            return self._recenter(now_s)
        if kind == "set":
            return self._apply_settings(message)
        return {"type": "error", "message": f"unknown message type {kind!r}"}

    def _apply_settings(self, message: Mapping[str, Any]) -> Dict[str, Any]:
        if "head_enabled" in message:
            self.head_enabled = bool(message["head_enabled"])
        if "arms_enabled" in message:
            self.arms_enabled = bool(message["arms_enabled"])
            if not self.arms_enabled and self.arm_teleop is not None:
                self.arm_teleop.release_all()
        if "want_poses" in message:
            self.want_poses = bool(message["want_poses"])
        if self.arm_teleop is not None:
            for key in ("position_scale", "orientation_enabled"):
                if key in message:
                    for arm in self.arm_teleop.arms.values():
                        value = message[key]
                        if key == "position_scale":
                            value = float(value)
                            if not 0.0 < value <= 5.0:
                                return {
                                    "type": "error",
                                    "message": "position_scale must be in (0, 5]",
                                }
                        setattr(arm.config, key, value)
        return self.hello()

    def _recenter(self, now_s: float) -> Dict[str, Any]:
        if self._last_head_robot is None:
            return {"type": "error", "message": "no head pose received yet; cannot re-centre"}
        self.operator.recenter(self._last_head_robot)
        head_op = self.operator.to_operator(self._last_head_robot)
        if self.head_tracker is not None:
            self.head_tracker.recenter(head_op[:3, :3])
        if self.arm_teleop is not None:
            self.arm_teleop.release_all()
        state = self._state(now_s, echo_t=None, force=True)
        assert state is not None  # force=True always produces a state
        state["recentred"] = True
        return state

    def _handle_pose(self, message: Mapping[str, Any], now_s: float) -> Dict[str, Any] | None:
        head_robot = _pose_entry(message.get("head"))
        head_targets: Dict[str, float] = {}
        if head_robot is not None:
            self._last_head_robot = head_robot
            if not self.operator.recentred:
                self.operator.recenter(head_robot)
            head_op = self.operator.to_operator(head_robot)
            if self.head_tracker is not None and self.head_enabled:
                torso_joint = self.head_tracker.mapping.torso_joint
                torso_angle = None
                if torso_joint is not None:
                    # The head rides on the torso; cancel the torso's heading so
                    # the camera follows the operator in the base frame.
                    torso_angle = self.backend.joint_positions().get(torso_joint)
                command = self.head_tracker.update(
                    head_op[:3, :3], now_s, torso_angle_rad=torso_angle
                )
                head_targets = command.targets_rad
                self._last_head = {
                    "yaw_input_rad": command.yaw_input_rad,
                    "pitch_input_rad": command.pitch_input_rad,
                    "targets_rad": command.targets_rad,
                    "at_limit": list(command.at_limit),
                    "torso_compensation_rad": command.torso_compensation_rad,
                    "tracking": True,
                }
            else:
                self._last_head = {"tracking": False}
        elif self.head_tracker is not None:
            self._last_head = {"tracking": False}

        arm_target = None
        grippers: Dict[str, float] = {}
        if self.arm_teleop is not None and self.arms_enabled and self.operator.recentred:
            samples: Dict[str, ControllerSample | None] = {}
            for side in ("left", "right"):
                entry = message.get(side)
                pose = _pose_entry(entry)
                if pose is None:
                    samples[side] = None
                    continue
                assert isinstance(entry, dict)
                buttons = entry.get("buttons") or {}
                samples[side] = ControllerSample(
                    pose=self.operator.to_operator(pose),
                    clutch=bool(entry.get("clutch", False)),
                    trigger=float(entry.get("trigger", 0.0) or 0.0),
                    buttons={str(k): bool(v) for k, v in buttons.items()}
                    if isinstance(buttons, dict)
                    else {},
                    stamp_s=now_s,
                )
            hand_poses = {side: self.backend.hand_pose(side) for side in ("left", "right")}
            output = self.arm_teleop.update(samples, hand_poses, now_s)
            arm_target = output.target
            grippers = output.gripper_targets_rad
            self._last_arms = {
                side: {
                    "clutched": cmd.clutched,
                    "enabled": cmd.enabled,
                    "tracked": cmd.tracked,
                    "gripper_rad": cmd.gripper_rad,
                    "target": None if cmd.target is None else matrix_to_pose(cmd.target),
                }
                for side, cmd in output.commands.items()
            }
        elif self.arm_teleop is not None:
            self.arm_teleop.release_all()
            self._last_arms = {
                side: {"clutched": False, "enabled": False, "tracked": False}
                for side in ("left", "right")
            }

        self._last_report = self.backend.apply(
            TeleopCommand(
                head_targets_rad=head_targets,
                arm_target=arm_target,
                gripper_targets_rad=grippers,
                stamp_s=now_s,
            )
        )
        return self._state(now_s, echo_t=message.get("t"))

    def _state(self, now_s: float, *, echo_t: Any, force: bool = False) -> Dict[str, Any] | None:
        if not force and now_s - self._last_state_s < 1.0 / self.config.state_hz:
            return None
        self._last_state_s = now_s
        self.seq += 1
        started = time.perf_counter()
        joints = self.backend.joint_positions()
        report = self._last_report
        state: Dict[str, Any] = {
            "type": "state",
            "seq": self.seq,
            "t": echo_t,
            "joints": {k: float(v) for k, v in joints.items()},
            "head": dict(self._last_head),
            "arms": dict(self._last_arms),
            "operator": {
                "recentred": self.operator.recentred,
                "yaw_offset_rad": self.operator.yaw_offset_rad,
            },
            "head_enabled": self.head_enabled,
            "arms_enabled": self.arms_enabled,
            "backend": self.backend.name,
        }
        if report is not None:
            state["ik"] = {
                "status": report.ik_status,
                "message": report.ik_message,
                "commandable": report.ik_commandable,
                "errors": report.errors,
                "compute_ms": report.compute_ms,
            }
            state["tcp"] = {side: matrix_to_pose(T) for side, T in report.hand_poses.items()}
            state["grippers"] = report.gripper_positions_rad
            state["warnings"] = list(report.warnings)
        if self.want_poses and self.bundle is not None:
            poses = self.bundle.poses(joints)
            state["geometries"] = poses["geometries"]
            state.setdefault("tcp", poses["tcp"])
        state["server_ms"] = (time.perf_counter() - started) * 1e3
        return state

    def close(self) -> None:
        """The operator is gone: release the clutches and hold."""
        if self.arm_teleop is not None:
            self.arm_teleop.release_all()
        self.backend.hold()

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly status."""
        return {
            "messages": self.messages,
            "state_seq": self.seq,
            "uptime_s": time.monotonic() - self.started_s,
            "recentred": self.operator.recentred,
            "head_enabled": self.head_enabled,
            "arms_enabled": self.arms_enabled,
            "want_poses": self.want_poses,
            "head": dict(self._last_head),
            "arms": dict(self._last_arms),
        }


class _VRHandler(_Handler):
    """Adds the VR routes; everything else falls through to the viewer handler."""

    server: "VRServer"  # type: ignore[assignment]

    def do_GET(self) -> None:  # noqa: N802 - stdlib naming
        path = urlsplit(self.path).path
        if path in ("/vr", "/vr/", "/vr.html"):
            self._send_file(_STATIC_DIR / "vr.html")
        elif path == "/ws/teleop":
            self._websocket(self.server.run_teleop_socket)
        elif path == "/ws/camera":
            self._websocket(self.server.run_camera_socket)
        elif path == "/api/vr/status":
            self._send_json(self.server.status())
        elif path.startswith("/vr/static/"):
            self._send_vr_static(path[len("/vr/static/") :])
        else:
            super().do_GET()

    def _send_vr_static(self, relative: str) -> None:
        target = (_STATIC_DIR / relative).resolve()
        if _STATIC_DIR.resolve() not in target.parents or not target.is_file():
            self._send_error(HTTPStatus.NOT_FOUND, "No such asset")
            return
        self._send_file(target)

    def _websocket(self, run: Callable[[WebSocket, socket.socket], None]) -> None:
        if not is_upgrade_request(self.headers):
            self._send_error(HTTPStatus.BAD_REQUEST, "This endpoint is a WebSocket.")
            return
        try:
            ws = handshake(self)
        except WebSocketError as exc:
            self._send_error(HTTPStatus.BAD_REQUEST, str(exc))
            return
        try:
            run(ws, self.connection)
        except Exception:  # noqa: BLE001 - a socket thread must not die silently
            logger.exception("websocket handler failed")
        finally:
            ws.close()
            self.close_connection = True


class VRServer(ViewerServer):
    """The viewer server extended with the teleoperation and camera sockets.

    Args:
        bundle: The loaded model.
        ik: The viewer's jog solver (for ``/api/ik``), or ``None``.
        backend: Where commands go.
        head_tracker: Head mapping, or ``None`` for an arms-only session.
        arm_teleop: Arm mapping, or ``None`` for a head-only session.
        camera: The frame streamer, or ``None`` for no picture.
        host: Bind address.  ``0.0.0.0`` for a headset on the LAN.
        port: Bind port.
        ssl_context: A server-side TLS context.  WebXR only runs in a secure
            context, so a headset on the LAN needs this (or an ``adb reverse``
            tunnel to ``localhost``, which browsers treat as secure).
        config: Session settings.
    """

    def __init__(
        self,
        bundle: ModelBundle,
        *,
        ik: IKSetup | None,
        backend: TeleopBackend,
        head_tracker: HeadTracker | None,
        arm_teleop: DualArmTeleop | None,
        camera: FrameStreamer | None = None,
        host: str = "127.0.0.1",
        port: int = 8766,
        ssl_context: ssl.SSLContext | None = None,
        config: VRServerConfig | None = None,
    ) -> None:
        # ThreadingHTTPServer.__init__ binds; swap the handler class first.
        self.RequestHandlerClass = _VRHandler  # type: ignore[misc]
        super().__init__(bundle, host=host, port=port, ik=ik)
        self.RequestHandlerClass = _VRHandler  # type: ignore[misc]
        self.backend = backend
        self.head_tracker = head_tracker
        self.arm_teleop = arm_teleop
        self.camera = camera
        self.vr_config = config or VRServerConfig()
        self.ssl_context = ssl_context
        if ssl_context is not None:
            self.socket = ssl_context.wrap_socket(self.socket, server_side=True)
        self._session: TeleopSession | None = None
        self._session_lock = threading.Lock()
        self._camera_clients = 0
        self._refused_operators = 0

    @property
    def url(self) -> str:
        """Where the pages are served (``https`` when TLS is on)."""
        host, port = self.server_address[0], self.server_address[1]
        if isinstance(host, bytes):
            host = host.decode()
        scheme = "https" if self.ssl_context is not None else "http"
        return f"{scheme}://{host}:{port}/"

    @property
    def vr_url(self) -> str:
        """The WebXR page."""
        return self.url + "vr"

    @property
    def session(self) -> TeleopSession | None:
        """The current operator's session, if any."""
        return self._session

    def describe(self) -> Dict[str, Any]:
        """The viewer description, marked as not simulation-only when hardware is behind it."""
        payload = super().describe()
        payload["simulation_only"] = self.backend.name == "simulation"
        payload["vr"] = True
        return payload

    def status(self) -> Dict[str, Any]:
        """Everything about the session."""
        with self._lock:
            session = self._session
            return {
                "backend": self.backend.describe(),
                "head": None if self.head_tracker is None else self.head_tracker.describe(),
                "arms": None if self.arm_teleop is None else self.arm_teleop.describe(),
                "camera": None if self.camera is None else self.camera.describe(),
                "session": None if session is None else session.describe(),
                "camera_clients": self._camera_clients,
                "refused_operators": self._refused_operators,
                "tls": self.ssl_context is not None,
                "config": {
                    "state_hz": self.vr_config.state_hz,
                    "teleop_timeout_s": self.vr_config.teleop_timeout_s,
                    "camera_fov_deg": self.vr_config.camera_fov_deg,
                    "twin_offset_m": list(self.vr_config.twin_offset_m),
                },
            }

    # -- sockets --------------------------------------------------------------

    def run_teleop_socket(self, ws: WebSocket, connection: socket.socket) -> None:
        """Serve one operator until the socket closes."""
        with self._session_lock:
            if self._session is not None:
                self._refused_operators += 1
                ws.send_text(
                    json.dumps(
                        {
                            "type": "error",
                            "message": "another operator is connected; only one may drive",
                        }
                    )
                )
                ws.close(1013, "operator slot taken")
                return
            with self._lock:
                session = TeleopSession(
                    self.backend,
                    head_tracker=self.head_tracker,
                    arm_teleop=self.arm_teleop,
                    config=self.vr_config,
                    bundle=self.bundle,
                )
            self._session = session
        connection.settimeout(self.vr_config.teleop_timeout_s)
        logger.info("VR operator connected")
        try:
            while True:
                try:
                    message = ws.recv()
                except WebSocketClosed:
                    break
                except (WebSocketError, socket.timeout, TimeoutError, OSError) as exc:
                    logger.info("VR operator stream ended: %s", exc)
                    break
                if message.opcode != OP_TEXT:
                    continue
                try:
                    payload = json.loads(message.text)
                except ValueError:
                    ws.send_text(json.dumps({"type": "error", "message": "invalid JSON"}))
                    continue
                if not isinstance(payload, dict):
                    ws.send_text(json.dumps({"type": "error", "message": "expected an object"}))
                    continue
                now = time.monotonic()
                try:
                    with self._lock:
                        reply = session.handle(payload, now)
                except Exception as exc:  # noqa: BLE001 - report, keep the stream alive
                    logger.exception("teleop message failed")
                    reply = {"type": "error", "message": f"{type(exc).__name__}: {exc}"}
                if reply is not None:
                    try:
                        ws.send_text(json.dumps(reply))
                    except WebSocketError:
                        break
        finally:
            with self._lock:
                try:
                    session.close()
                except Exception:  # noqa: BLE001
                    logger.exception("session close failed")
            with self._session_lock:
                if self._session is session:
                    self._session = None
            logger.info("VR operator disconnected; arms holding")

    def run_camera_socket(self, ws: WebSocket, connection: socket.socket) -> None:
        """Push JPEG frames to one client as they are produced."""
        if self.camera is None:
            ws.send_text(json.dumps({"type": "error", "message": "no camera in this session"}))
            ws.close(1011, "no camera")
            return
        self._camera_clients += 1
        stop = threading.Event()

        def drain() -> None:
            # The client sends nothing but pings/close; read them so a close
            # frame is honoured promptly instead of on the next failed send.
            try:
                while not stop.is_set():
                    ws.recv()
            except (WebSocketError, OSError):
                pass
            finally:
                stop.set()

        reader = threading.Thread(target=drain, name="robopy-camera-ws-reader", daemon=True)
        reader.start()
        try:
            ws.send_text(
                json.dumps(
                    {
                        "type": "camera",
                        "fov_deg": self.vr_config.camera_fov_deg,
                        "source": type(self.camera.source).__name__,
                    }
                )
            )
            last_seq = 0
            while not stop.is_set():
                frame = self.camera.wait_for(last_seq, 1.0)
                if frame is None:
                    continue
                last_seq = frame.seq
                ws.send_binary(frame.data)
        except WebSocketError:
            pass
        finally:
            stop.set()
            self._camera_clients -= 1

    def server_close(self) -> None:
        """Stop the camera thread along with the server."""
        if self.camera is not None:
            self.camera.stop()
        super().server_close()


def serve_vr(
    server: VRServer,
    *,
    open_browser: bool = False,
    on_ready: Callable[[VRServer], None] | None = None,
) -> None:
    """Run a built :class:`VRServer` until interrupted, printing where it is."""
    print(f"robopy VR teleoperation: {server.vr_url}")
    print(f"  viewer page : {server.url}")
    print(f"  backend     : {server.backend.name}")
    print(f"  head        : {'on' if server.head_tracker is not None else 'off'}")
    print(f"  arms        : {'on' if server.arm_teleop is not None else 'off'}")
    print(f"  camera      : {'on' if server.camera is not None else 'off'}")
    if server.ssl_context is None:
        print(
            "  TLS off: WebXR needs a secure context. Open the page on the headset via an "
            "`adb reverse` tunnel to localhost, or start with --cert/--key."
        )
    if server.backend.name == "simulation":
        print("  SIMULATION ONLY -- nothing here talks to a motor. Ctrl+C to stop.")
    else:
        print("  HARDWARE: the follower arm will move. Keep the stop switch in reach.")
    if server.camera is not None:
        server.camera.start()
    if on_ready is not None:
        on_ready(server)
    if open_browser:
        threading.Timer(0.5, lambda: webbrowser.open(server.vr_url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
