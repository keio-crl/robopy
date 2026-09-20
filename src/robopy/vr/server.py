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
    {"type": "ping"}                       keepalive; answered with {"type": "pong"}
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
import select
import socket
import ssl
import threading
import time
import webbrowser
from dataclasses import dataclass
from http import HTTPStatus
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Tuple
from urllib.parse import urlsplit

import numpy as np
from numpy.typing import NDArray

from robopy.viewer.model_bundle import ModelBundle, matrix_to_pose
from robopy.viewer.server import IKSetup, ViewerServer, _Handler

from .arm_teleop import ControllerSample, DualArmTeleop
from .backend import TeleopBackend, TeleopCommand
from .camera import FrameStreamer, SyntheticFrameSource
from .head_tracking import HeadTracker
from .recording import CameraTap, SessionRecorder
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

__all__ = ["TeleopSession", "VRServer", "VRServerConfig", "head_anchor_position", "serve_vr"]

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
        twin_offset_m: Where the page draws the robot's base, in robot axes
            (x forward, y left, z up) from the WebXR floor origin, or ``None``
            to place the twin so that the robot's head anchor sits where the
            operator's head was at re-centring -- the operator then stands in
            the robot and, with the absolute arm mapping, the twin's hands
            come to their own.  Purely visual either way.
        arm_anchor_frame: Model frame whose position is the robot's head
            anchor for the absolute arm mapping; ``"auto"`` takes
            ``head_camera_link`` when the model has it.  Ignored when both
            arms use the relative mapping.
        clutch_button: Which controller button the page treats as the clutch:
            ``"a"`` (A on the right, X on the left; the thumb), ``"grip"``
            (the squeeze) or ``"stick"`` (the thumbstick click).
        head_enabled: Whether the head follows the headset at session start.
        arms_enabled: Whether the arms follow the controllers at session start.
        record_dir: Where session recordings (and their videos) are written,
            or ``None`` to disable recording.
        render_videos: Whether to render each recording to MP4 with
            :mod:`robopy.vr.render` as soon as it is written (needs MuJoCo).
    """

    state_hz: float = 30.0
    teleop_timeout_s: float = 5.0
    camera_fov_deg: float = 69.0
    twin_offset_m: Tuple[float, float, float] | None = None
    arm_anchor_frame: str = "auto"
    clutch_button: str = "a"
    head_enabled: bool = True
    arms_enabled: bool = True
    record_dir: Path | None = Path("recordings")
    render_videos: bool = True

    def __post_init__(self) -> None:
        if self.state_hz <= 0.0 or self.teleop_timeout_s <= 0.0:
            raise ValueError("state_hz and teleop_timeout_s must be positive.")
        if not 10.0 <= self.camera_fov_deg <= 170.0:
            raise ValueError("camera_fov_deg must be within [10, 170].")
        if self.clutch_button not in ("a", "grip", "stick"):
            raise ValueError("clutch_button must be 'a', 'grip' or 'stick'.")


#: Frames tried, in order, for the robot's head anchor when none is named.
HEAD_ANCHOR_FRAMES: Tuple[str, ...] = ("head_camera_link", "head_link", "head")


def head_anchor_position(
    bundle: ModelBundle,
    joints: Mapping[str, float],
    head_tracker: HeadTracker | None,
    frame: str = "auto",
) -> NDArray[np.float64]:
    """Position of the robot's head anchor in the base frame.

    Computed at ``joints`` with the head joints at the forward-looking neutral
    of ``head_tracker`` (when given), so the anchor does not depend on where
    the head happens to point.

    Args:
        bundle: The model.
        joints: Current joint positions, radians.
        head_tracker: Supplies the head joints' neutral angles, or ``None``.
        frame: A model frame, or ``"auto"``.

    Raises:
        ValueError: The frame is unknown (or none of the automatic candidates
            exists).
    """
    model = bundle.model
    if frame == "auto":
        found = next((f for f in HEAD_ANCHOR_FRAMES if model.has_frame(f)), None)
        if found is None:
            raise ValueError(
                f"no head anchor frame among {HEAD_ANCHOR_FRAMES}; name one with "
                "arm_anchor_frame (--arm-anchor) or use the relative arm mapping"
            )
        frame = found
    elif not model.has_frame(frame):
        raise ValueError(f"arm anchor frame {frame!r} is not in the model")
    positions = dict(joints)
    if head_tracker is not None:
        m = head_tracker.mapping
        positions[m.yaw_joint] = m.yaw_neutral_rad
        positions[m.pitch_joint] = m.pitch_neutral_rad
    q = model.q_from_positions(positions, require_all=False)
    return np.asarray(model.frame_pose(q, frame)[:3, 3], dtype=np.float64).copy()


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
        recorder: SessionRecorder | None = None,
        camera_available: bool = True,
    ) -> None:
        """Create a session.

        Args:
            backend: Where commands go.
            head_tracker: Head mapping, or ``None`` for an arms-only session.
            arm_teleop: Arm mapping, or ``None`` for a head-only session.
            config: Session settings.
            bundle: The model bundle, for geometry poses sent to the page.
            recorder: Where ``{"type": "record"}`` messages go, or ``None``
                when recording is off.
            camera_available: Whether the server streams a camera image; the
                page hides the image plane otherwise.
        """
        self.backend = backend
        self.recorder = recorder
        self.camera_available = camera_available
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
        self._last_controller_base: Dict[str, Any] = {}
        self._last_report: Any = None
        self.recorder_joint_names: List[str] = list(backend.joint_positions())
        self.robot_anchor_m: NDArray[np.float64] | None = None
        if arm_teleop is not None and arm_teleop.needs_anchor:
            if bundle is None:
                raise ValueError("the absolute arm mapping needs the model bundle for its anchor")
            self.robot_anchor_m = head_anchor_position(
                bundle, backend.joint_positions(), head_tracker, config.arm_anchor_frame
            )
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
            "twin_offset_m": None
            if self.config.twin_offset_m is None
            else list(self.config.twin_offset_m),
            "twin": self._twin_pose(),
            "clutch_button": self.config.clutch_button,
            "robot_anchor_m": None
            if self.robot_anchor_m is None
            else [float(v) for v in self.robot_anchor_m],
            "state_hz": self.config.state_hz,
            "backend_info": self.backend.describe(),
            "camera_available": self.camera_available,
            "recording": None if self.recorder is None else self.recorder.describe(),
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
        if kind == "record":
            return self._record(message, now_s)
        if kind == "ping":
            # Keepalive from a page that is connected but not yet streaming
            # poses (before Enter VR); answering it also lets the page measure
            # the link.
            return {"type": "pong", "t": message.get("t")}
        return {"type": "error", "message": f"unknown message type {kind!r}"}

    # -- recording ----------------------------------------------------------

    def recording_metadata(self) -> Dict[str, Any]:
        """The header of a recording: enough to render it without the session."""
        joints = list(self.backend.joint_positions())
        head = None
        if self.head_tracker is not None:
            m = self.head_tracker.mapping
            head = {
                "yaw_joint": m.yaw_joint,
                "pitch_joint": m.pitch_joint,
                "yaw_neutral_rad": m.yaw_neutral_rad,
                "pitch_neutral_rad": m.pitch_neutral_rad,
            }
        arms = None
        if self.arm_teleop is not None:
            left = self.arm_teleop.arms["left"].config
            arms = {"mapping": left.mapping, "position_scale": left.position_scale}
        return {
            "robot": None if self.bundle is None else self.bundle.describe().get("robot"),
            "backend": self.backend.name,
            "joint_names": joints,
            "head": head,
            "anchor_m": None
            if self.robot_anchor_m is None
            else [float(v) for v in self.robot_anchor_m],
            "arms": arms,
        }

    def _record(self, message: Mapping[str, Any], now_s: float) -> Dict[str, Any]:
        if self.recorder is None:
            return {"type": "error", "message": "recording is disabled on this server"}
        action = message.get("action", "status")
        if action == "start":
            self.recorder.start(self.recording_metadata(), now_s)
        elif action == "stop":
            self.recorder.stop(now_s)
        elif action == "toggle":
            if self.recorder.active:
                self.recorder.stop(now_s)
            else:
                self.recorder.start(self.recording_metadata(), now_s)
        elif action != "status":
            return {"type": "error", "message": f"unknown record action {action!r}"}
        return {"type": "recording", "recording": self.recorder.describe()}

    def _record_frame(self, now_s: float) -> None:
        """Append this pose step to the recorder (cheap when not recording)."""
        if self.recorder is None or not self.recorder.active:
            return
        joints = self.backend.joint_positions()
        report = self._last_report
        head = None
        if self._last_head.get("tracking"):
            head = {
                "yaw_rad": self._last_head.get("yaw_input_rad"),
                "pitch_rad": self._last_head.get("pitch_input_rad"),
            }
        controllers: Dict[str, Any] = {}
        targets: Dict[str, Any] = {}
        for side in ("left", "right"):
            arm = self._last_arms.get(side) or {}
            p_base = self._last_controller_base.get(side)
            controllers[side] = (
                None
                if not arm.get("tracked")
                else {"p_base": p_base, "clutched": bool(arm.get("clutched"))}
            )
            target = arm.get("target")
            targets[side] = None if not target else list(target["p"])
        ik = None
        if report is not None:
            errors = report.errors or {}
            ik = {
                "status": report.ik_status,
                "left_position_m": errors.get("left_position_m"),
                "right_position_m": errors.get("right_position_m"),
            }
        self.recorder.add(
            {
                "q": [float(joints.get(name, 0.0)) for name in self.recorder_joint_names],
                "head": head,
                "controllers": controllers,
                "targets": targets,
                "ik": ik,
            },
            now_s,
        )

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

    def _recentre_operator(self, head_robot: NDArray[np.float64]) -> None:
        """Re-centre the operator frame and re-anchor the absolute arm mapping."""
        self.operator.recenter(head_robot)
        if self.arm_teleop is not None and self.robot_anchor_m is not None:
            self.arm_teleop.set_anchor(self.robot_anchor_m, self.operator.head_position_m)

    def _twin_pose(self) -> Dict[str, Any] | None:
        """Where the page should draw the robot: ``{"p", "yaw"}`` in robot axes.

        With an explicit ``twin_offset_m`` the page places the twin itself.
        Otherwise the twin is placed so its head anchor coincides with the
        operator's head at re-centring, facing the operator's forward.
        """
        if (
            self.config.twin_offset_m is not None
            or self.robot_anchor_m is None
            or not self.operator.recentred
        ):
            return None
        base_op = np.eye(4)
        base_op[:3, 3] = self.operator.head_position_m - self.robot_anchor_m
        base = self.operator.from_operator(base_op)
        return {"p": [float(v) for v in base[:3, 3]], "yaw": self.operator.yaw_offset_rad}

    def _recenter(self, now_s: float) -> Dict[str, Any]:
        if self._last_head_robot is None:
            return {"type": "error", "message": "no head pose received yet; cannot re-centre"}
        self._recentre_operator(self._last_head_robot)
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
                self._recentre_operator(head_robot)
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
                pose_op = self.operator.to_operator(pose)
                if self.robot_anchor_m is not None:
                    scale = self.arm_teleop.arms[side].config.position_scale
                    p_base = self.robot_anchor_m + scale * (
                        pose_op[:3, 3] - self.operator.head_position_m
                    )
                    self._last_controller_base[side] = [float(v) for v in p_base]
                else:
                    self._last_controller_base[side] = None
                samples[side] = ControllerSample(
                    pose=pose_op,
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
        self._record_frame(now_s)
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
                "head_height_m": self.operator.head_height_m,
            },
            "twin": self._twin_pose(),
            "recording": None if self.recorder is None else self.recorder.describe(),
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
        """The operator is gone: release the clutches, hold, and finish any recording."""
        if self.arm_teleop is not None:
            self.arm_teleop.release_all()
        self.backend.hold()
        if self.recorder is not None and self.recorder.active:
            self.recorder.stop(time.monotonic())

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
        self.recorder: SessionRecorder | None = None
        if self.vr_config.record_dir is not None:
            render = None
            if self.vr_config.render_videos:
                from .render import render_recording

                def render(path: Path, progress: Callable[[float], None]) -> List[Path]:
                    return render_recording(path, progress=progress)

            tap = None
            if self.camera is not None and not isinstance(self.camera.source, SyntheticFrameSource):
                # A real camera: its pictures become the first-person video.
                tap = CameraTap(self.camera)
            self.recorder = SessionRecorder(self.vr_config.record_dir, render=render, camera=tap)

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
                "recording": None if self.recorder is None else self.recorder.describe(),
                "refused_operators": self._refused_operators,
                "tls": self.ssl_context is not None,
                "config": {
                    "state_hz": self.vr_config.state_hz,
                    "teleop_timeout_s": self.vr_config.teleop_timeout_s,
                    "camera_fov_deg": self.vr_config.camera_fov_deg,
                    "twin_offset_m": None
                    if self.vr_config.twin_offset_m is None
                    else list(self.vr_config.twin_offset_m),
                    "arm_anchor_frame": self.vr_config.arm_anchor_frame,
                    "clutch_button": self.vr_config.clutch_button,
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
                    recorder=self.recorder,
                    camera_available=self.camera is not None,
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
        """Push JPEG frames to one client as they are produced.

        One thread does both directions on purpose.  The client sends nothing
        but pings and a close, which are serviced when the socket becomes
        readable; a second thread blocked in ``recv`` while this one sends
        would use the same TLS connection from two threads at once, which
        OpenSSL does not allow and which ended camera streams mid-session.
        """
        if self.camera is None:
            ws.send_text(json.dumps({"type": "error", "message": "no camera in this session"}))
            ws.close(1011, "no camera")
            return
        self._camera_clients += 1
        reason = "client closed"

        def readable() -> bool:
            pending = getattr(connection, "pending", None)
            if pending is not None and pending():
                return True
            ready, _, _ = select.select([connection], [], [], 0.0)
            return bool(ready)

        try:
            ws.send_text(
                json.dumps(
                    {
                        "type": "camera",
                        "fov_deg": self.vr_config.camera_fov_deg,
                        "source": type(self.camera.source).__name__,
                        "rotate_deg": self.camera.rotate_deg,
                    }
                )
            )
            last_seq = 0
            while True:
                if readable():
                    ws.recv(control_only=True)  # answers pings; raises on close
                frame = self.camera.wait_for(last_seq, 0.25)
                if frame is None:
                    continue
                last_seq = frame.seq
                ws.send_binary(frame.data)
        except WebSocketClosed:
            pass
        except (WebSocketError, OSError) as exc:
            reason = f"{type(exc).__name__}: {exc}"
        finally:
            self._camera_clients -= 1
            logger.info("camera client gone (%s); %d still watching", reason, self._camera_clients)

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
