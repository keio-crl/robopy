"""A small standard-library HTTP server behind the viewer page.

No web framework is pulled in: the page needs a handful of JSON endpoints, the
static files, and the STL meshes.  Everything that touches the kinematic model
runs under one lock, because Pinocchio's ``Data`` is not thread-safe and the
server handles requests on threads.

Endpoints
---------
``GET  /``                  the page
``GET  /static/<file>``     page assets (served from the package directory)
``GET  /mesh/<index>``      the STL of geometry ``index`` (index-based, so no path
                            from the network ever reaches the filesystem)
``GET  /api/model``         static model description
``POST /api/fk``            ``{"joints": {name: rad}}`` -> poses
``POST /api/ik``            targets -> solved joints and poses (needs the solver)
``GET  /api/health``        liveness
"""

from __future__ import annotations

import json
import logging
import mimetypes
import threading
import time
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Sequence
from urllib.parse import unquote, urlsplit

import numpy as np

from robopy.control.types import (
    DualArmTarget,
    JointState,
    TorsoPolicy,
    monotonic_ns,
    se3_from_quat_xyzw,
)

from .model_bundle import ModelBundle

logger = logging.getLogger(__name__)

__all__ = ["IKSetup", "ViewerServer", "serve"]

_STATIC_DIR = Path(__file__).parent / "static"
_MAX_BODY_BYTES = 1 << 20


class IKSetup:
    """The dual-arm solver bound to a bundle, plus how its joints were grouped."""

    def __init__(
        self,
        bundle: ModelBundle,
        *,
        torso_joint: str | None = None,
        left_arm_joints: Sequence[str] | None = None,
        right_arm_joints: Sequence[str] | None = None,
        head_joints: Sequence[str] | None = None,
    ) -> None:
        """Build the solver, inferring joint groups from names when not given.

        Inference is by the substrings ``left`` / ``right`` / ``head`` / ``torso``
        in the URDF joint names, in tree (shoulder-to-wrist) order.  That holds
        for the Rakuda export and the synthetic fixture; anything else should
        pass the groups explicitly.  The groups used are recorded in
        :attr:`groups` and shown in the page so an inference is never silent.
        """
        from robopy.kinematics.dual_arm_ik import DualArmIK, DualArmIKConfig  # noqa: PLC0415

        names = bundle.joint_order
        torso = torso_joint or next((n for n in names if "torso" in n), None)
        left = list(left_arm_joints or [n for n in names if "left" in n and "head" not in n])
        right = list(right_arm_joints or [n for n in names if "right" in n and "head" not in n])
        head = list(head_joints or [n for n in names if "head" in n])
        if torso is None or len(left) != 6 or len(right) != 6:
            raise ValueError(
                "Could not infer the IK joint groups: need one torso joint and six joints per "
                f"arm, found torso={torso!r}, left={left}, right={right}. Pass the groups "
                "explicitly (or via --config)."
            )
        if not {"left", "right"} <= set(bundle.tcp_frames):
            raise ValueError("The bundle has no left/right TCP frames; the solver needs both.")

        unbounded = bundle.model.unbounded_joints([torso, *left, *right])
        self.geometric_study_only = bool(unbounded)
        self.solver = DualArmIK(
            bundle.model,
            left_frame=bundle.tcp_frames["left"],
            right_frame=bundle.tcp_frames["right"],
            torso_joint=torso,
            left_arm_joints=left,
            right_arm_joints=right,
            head_joints=head,
            config=DualArmIKConfig(
                # The page iterates to convergence in one request; per-step
                # bounds stay in place so the path is one the machine could take.
                max_joint_step_rad=0.05,
                compute_budget_s=5.0,
                max_state_age_s=5.0,
                # The viewer iterates to convergence in one request; there is no
                # machine integrating these steps, so an acceleration window is
                # meaningless here and only slows the approach to the answer.
                max_joint_acceleration_rad_s2=None,
                # A little more Tikhonov damping than the controller default: it
                # penalises step size without biasing the equilibrium, which
                # keeps a straight (singular) arm from wandering along its
                # null space while a jog converges.
                damping=1e-3,
                require_soft_limits=not unbounded,
            ),
        )
        self.groups: Dict[str, Any] = {
            "torso": torso,
            "left": left,
            "right": right,
            "head": head,
            "unbounded_continuous": unbounded,
        }
        self._bundle = bundle

    def solve(
        self,
        positions: Mapping[str, float],
        targets: Mapping[str, Any],
        *,
        torso_policy: str = "fixed",
        torso_velocity_rad_s: float = 0.0,
        iterations: int = 200,
        dt: float = 0.02,
        orientation_weight: float | None = None,
    ) -> Dict[str, Any]:
        """Iterate the differential solver from ``positions`` until it converges.

        Args:
            positions: Starting joint configuration.
            targets: ``{"left"|"right": {"p": [3], "q": [4]}}`` for the driven hands.
            torso_policy: ``fixed`` / ``manual`` / ``optimize``.
            torso_velocity_rad_s: Torso rate for the ``manual`` policy.
            iterations: Iteration budget.
            dt: Per-iteration step time; with the step bounds this sets the
                largest joint move per iteration.
            orientation_weight: Orientation task weight.  ``0`` makes the jog
                position-only, which is what a two-axis wrist usually needs;
                ``None`` keeps the solver's current weight.

        Returns:
            Status, message, final joints, per-hand errors, the iteration count
            and ``stalled`` -- true when the residual stopped changing before
            converging, i.e. the target is not exactly reachable from here and
            the pose shown is the solver's weighted compromise.  A
            non-commandable status returns the *starting* joints, so the page
            never displays a pose the solver refused.
        """
        from robopy.kinematics.dual_arm_ik import DualArmIKStatus  # noqa: PLC0415

        def se3(entry: Any) -> np.ndarray | None:
            if not entry:
                return None
            return se3_from_quat_xyzw(entry["p"], entry["q"])

        left = se3(targets.get("left"))
        right = se3(targets.get("right"))
        if left is None and right is None:
            raise ValueError(
                "At least one of targets.left / targets.right is required; with both hands "
                "disabled there is nothing to solve."
            )
        policy = TorsoPolicy(torso_policy)
        target = DualArmTarget(
            left_target=left,
            right_target=right,
            left_enabled=left is not None,
            right_enabled=right is not None,
            torso_policy=policy,
            torso_velocity_rad_s=torso_velocity_rad_s if policy is TorsoPolicy.MANUAL else 0.0,
        )

        if orientation_weight is not None:
            if not 0.0 <= float(orientation_weight) <= 10.0:
                raise ValueError("orientation_weight must be within [0, 10].")
            self.solver.set_task_costs(orientation_cost=float(orientation_weight))

        current = {name: 0.0 for name in self._bundle.joint_order}
        current.update({k: float(v) for k, v in positions.items()})
        # Regularise towards where the arm *is*, not towards wherever the solver
        # first saw it: a jog should be the smallest motion that reaches the
        # target, and a stale reference would drag the arm back on every request.
        self.solver.reset(self._bundle.positions_to_q(current))
        result = None
        steps = 0
        history: list[float] = []
        stalled = False
        for steps in range(1, max(1, int(iterations)) + 1):
            result = self.solver.solve_step(self._state(current), target, dt)
            if not result.is_commandable:
                break
            current.update(result.joint_targets_rad)
            if result.status is DualArmIKStatus.CONVERGED:
                break
            # Stall detection: when the position residual has not moved by a
            # micrometre over the last 40 iterations, more iterations will not
            # help -- the solver is at its weighted compromise.
            history.append(
                float(
                    sum(
                        e
                        for e in (result.left_position_error_m, result.right_position_error_m)
                        if e is not None
                    )
                )
            )
            if len(history) > 40 and abs(history[-1] - history[-41]) < 1e-6:
                stalled = True
                break
        assert result is not None
        final = (
            current
            if result.is_commandable
            else {name: 0.0 for name in self._bundle.joint_order}
            | {k: float(v) for k, v in positions.items()}
        )
        return {
            "status": result.status.value,
            "commandable": result.is_commandable,
            "message": result.message,
            "iterations": steps,
            "joints": final,
            "errors": {
                "left_position_m": result.left_position_error_m,
                "left_orientation_rad": result.left_orientation_error_rad,
                "right_position_m": result.right_position_error_m,
                "right_orientation_rad": result.right_orientation_error_rad,
                "left_hold_m": result.left_hold_residual_m,
                "right_hold_m": result.right_hold_residual_m,
            },
            "active_limits": list(result.active_limits),
            "torso_velocity_rad_s": result.torso_velocity_rad_s,
            "stalled": stalled,
            "orientation_weight": self.solver.orientation_cost,
        }

    def _state(self, positions: Mapping[str, float]) -> JointState:
        names = self._bundle.joint_order
        now = monotonic_ns()
        n = len(names)
        return JointState(
            joint_names=names,
            position_rad=np.asarray([positions[name] for name in names], dtype=float),
            velocity_rad_s=np.zeros(n),
            current_a=np.zeros(n),
            valid=np.ones(n, dtype=bool),
            read_start_ns=now,
            read_end_ns=now,
            sequence=0,
            mode_generation=0,
        )


class _Handler(BaseHTTPRequestHandler):
    """Request handler; the server instance is reached through ``self.server``."""

    server: "ViewerServer"  # type: ignore[assignment]
    protocol_version = "HTTP/1.1"

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002 - stdlib signature
        logger.debug("%s - " + format, self.address_string(), *args)

    # -- routing ------------------------------------------------------------

    def do_GET(self) -> None:  # noqa: N802 - stdlib naming
        path = urlsplit(self.path).path
        if path in ("/", "/index.html"):
            self._send_file(_STATIC_DIR / "index.html")
        elif path.startswith("/static/"):
            self._send_static(path[len("/static/") :])
        elif path.startswith("/mesh/"):
            self._send_mesh(path[len("/mesh/") :])
        elif path == "/api/model":
            self._send_json(self.server.describe())
        elif path == "/api/health":
            self._send_json({"ok": True, "ik": self.server.ik is not None})
        else:
            self._send_error(HTTPStatus.NOT_FOUND, f"No route for {path}")

    def do_HEAD(self) -> None:  # noqa: N802 - stdlib naming
        self.do_GET()

    def do_POST(self) -> None:  # noqa: N802 - stdlib naming
        path = urlsplit(self.path).path
        try:
            body = self._read_json()
            if path == "/api/fk":
                self._send_json(self.server.fk(body))
            elif path == "/api/ik":
                self._send_json(self.server.ik_solve(body))
            else:
                self._send_error(HTTPStatus.NOT_FOUND, f"No route for {path}")
        except (KeyError, ValueError, TypeError) as exc:
            self._send_error(HTTPStatus.BAD_REQUEST, str(exc))
        except RuntimeError as exc:
            self._send_error(HTTPStatus.SERVICE_UNAVAILABLE, str(exc))

    # -- helpers --------------------------------------------------------------

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > _MAX_BODY_BYTES:
            raise ValueError(f"Request body must be 1..{_MAX_BODY_BYTES} bytes.")
        payload = json.loads(self.rfile.read(length))
        if not isinstance(payload, dict):
            raise ValueError("Request body must be a JSON object.")
        return payload

    def _send_json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(data)

    def _send_error(self, status: HTTPStatus, message: str) -> None:
        self._send_json({"error": message}, status)

    def _send_static(self, relative: str) -> None:
        # Resolve inside the static directory and refuse anything that escapes it.
        target = (_STATIC_DIR / unquote(relative)).resolve()
        if _STATIC_DIR.resolve() not in target.parents or not target.is_file():
            self._send_error(HTTPStatus.NOT_FOUND, "No such asset")
            return
        self._send_file(target)

    def _send_mesh(self, index_text: str) -> None:
        try:
            index = int(index_text)
            geometry = self.server.bundle.meshes[index]
        except (ValueError, IndexError):
            self._send_error(HTTPStatus.NOT_FOUND, "No such mesh")
            return
        assert geometry.mesh_path is not None
        self._send_file(geometry.mesh_path, content_type="model/stl", cache=True)

    def _send_file(
        self, path: Path, *, content_type: str | None = None, cache: bool = False
    ) -> None:
        if not path.is_file():
            self._send_error(HTTPStatus.NOT_FOUND, "Missing file")
            return
        ctype = content_type or mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        if path.suffix == ".js":
            ctype = "text/javascript; charset=utf-8"
        size = path.stat().st_size
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(size))
        self.send_header("Cache-Control", "public, max-age=3600" if cache else "no-store")
        self.end_headers()
        if self.command == "HEAD":
            return
        with path.open("rb") as handle:
            while True:
                chunk = handle.read(1 << 16)
                if not chunk:
                    break
                self.wfile.write(chunk)


class ViewerServer(ThreadingHTTPServer):
    """The HTTP server owning a :class:`ModelBundle` and, optionally, a solver."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        bundle: ModelBundle,
        *,
        host: str = "127.0.0.1",
        port: int = 8765,
        ik: IKSetup | None = None,
    ) -> None:
        super().__init__((host, port), _Handler)
        self.bundle = bundle
        self.ik = ik
        self._lock = threading.Lock()
        self._fk_calls = 0
        self._fk_seconds = 0.0

    @property
    def url(self) -> str:
        """Where the page is served."""
        host, port = self.server_address[0], self.server_address[1]
        if isinstance(host, bytes):  # AF_UNIX addresses are bytes in typeshed; not used here
            host = host.decode()
        return f"http://{host}:{port}/"

    def describe(self) -> Dict[str, Any]:
        """Static model description plus the solver setup."""
        with self._lock:
            payload = self.bundle.describe()
        payload["ik"] = (
            None
            if self.ik is None
            else {
                "available": True,
                "groups": self.ik.groups,
                "geometric_study_only": self.ik.geometric_study_only,
            }
        )
        payload["simulation_only"] = True
        return payload

    def fk(self, body: Mapping[str, Any]) -> Dict[str, Any]:
        """Poses for the joint configuration in ``body["joints"]``."""
        joints = body.get("joints")
        if not isinstance(joints, dict):
            raise ValueError("'joints' must be an object of {joint: radians}.")
        for name, value in joints.items():
            if not isinstance(value, (int, float)) or not np.isfinite(value):
                raise ValueError(f"Joint '{name}' has a non-finite value.")
        started = time.perf_counter()
        with self._lock:
            poses = self.bundle.poses(joints)
            self._fk_calls += 1
            self._fk_seconds += time.perf_counter() - started
        poses["timing_ms"] = (time.perf_counter() - started) * 1e3
        return poses

    def ik_solve(self, body: Mapping[str, Any]) -> Dict[str, Any]:
        """Solve for ``body["targets"]`` from ``body["joints"]`` and return poses."""
        if self.ik is None:
            raise RuntimeError(
                "The solver is not available: the model has no left/right TCP frames or the "
                "joint groups could not be inferred. See the server log."
            )
        joints = body.get("joints")
        targets = body.get("targets")
        if not isinstance(joints, dict) or not isinstance(targets, dict):
            raise ValueError("'joints' and 'targets' must be objects.")
        started = time.perf_counter()
        with self._lock:
            result = self.ik.solve(
                joints,
                targets,
                torso_policy=str(body.get("torso_policy", "fixed")),
                torso_velocity_rad_s=float(body.get("torso_velocity_rad_s", 0.0)),
                iterations=int(body.get("iterations", 200)),
                dt=float(body.get("dt", 0.02)),
                orientation_weight=(
                    None
                    if body.get("orientation_weight") is None
                    else float(body["orientation_weight"])
                ),
            )
            result["poses"] = self.bundle.poses(result["joints"])
        result["timing_ms"] = (time.perf_counter() - started) * 1e3
        return result


def serve(
    bundle: ModelBundle,
    *,
    host: str = "127.0.0.1",
    port: int = 8765,
    open_browser: bool = True,
    ik: IKSetup | None = None,
    on_ready: Callable[[ViewerServer], None] | None = None,
) -> None:
    """Run the viewer until interrupted."""
    server = ViewerServer(bundle, host=host, port=port, ik=ik)
    print(f"robopy viewer: {server.url}")
    print(f"  model : {bundle.urdf_path}")
    print(
        f"  shapes: {len(bundle.geometries)} ({len(bundle.meshes)} meshes, drawn from "
        f"<{bundle.geometry_source}>)   joints: {len(bundle.joint_order)}"
    )
    print(f"  IK    : {'available' if ik is not None else 'not available'}")
    print("  SIMULATION ONLY -- nothing here talks to a motor. Ctrl+C to stop.")
    if on_ready is not None:
        on_ready(server)
    if open_browser:
        threading.Timer(0.5, lambda: webbrowser.open(server.url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
