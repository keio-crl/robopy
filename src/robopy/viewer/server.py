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
``POST /api/ik``            targets -> a timed joint trajectory (``mode: trajectory``,
                            the page's mode) or the end-point solve of the
                            legacy ``mode: endpoint``; needs the solver
``POST /api/ik/reset``      forget the session's velocity history and posture
                            reference (a manual pose change, a mode switch)
``GET  /api/health``        liveness

Two modes of ``/api/ik`` are kept deliberately apart.  ``trajectory`` is the
continuous-operation mode: the server keeps the solver's velocity history and
posture reference between requests, walks a reference pose towards the goal
under Cartesian velocity and acceleration ceilings and returns every sample
with its time, which the page plays back at that timing.  ``endpoint`` is the
analysis mode the page used before: iterate from the given configuration to
convergence and return only the final joints; it resets the solver first and
keeps nothing.
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
    InactiveArmPolicy,
    JointState,
    TorsoPolicy,
    monotonic_ns,
    se3_from_quat_xyzw,
)
from robopy.kinematics.cartesian_trajectory import (
    JointTrajectory,
    PoseReference,
    TrajectoryLimits,
    run_trajectory,
)

from .model_bundle import ModelBundle, matrix_to_pose

logger = logging.getLogger(__name__)

__all__ = ["SIMULATION_TRAJECTORY_LIMITS", "IKSetup", "ViewerServer", "serve"]

_STATIC_DIR = Path(__file__).parent / "static"
_MAX_BODY_BYTES = 1 << 20

#: Reference ceilings of the viewer's simulation profile.  These are numbers
#: for evaluating the software on a model; the machine's ceilings are a
#: configuration (control.trajectory) and are not filled in from here.
SIMULATION_TRAJECTORY_LIMITS = TrajectoryLimits(
    max_linear_velocity_m_s=0.25,
    max_linear_acceleration_m_s2=1.0,
    max_angular_velocity_rad_s=1.5,
    max_angular_acceleration_rad_s2=6.0,
    lag_tolerance_m=0.02,
)
SIMULATION_SAMPLE_PERIOD_S = 0.02


def _reach_bound(
    bundle: ModelBundle,
    arm_joints: Sequence[str],
    tcp_frame: str,
) -> Dict[str, Any] | None:
    """Where a hand can possibly be: its shoulder, and the arm unfolded.

    The radius is the sum of the arm's segment lengths at ``q = 0``, which is
    an upper bound on how far the TCP can get from the first arm joint -- not
    a reachability claim, just a finite range for the page's target sliders.
    ``None`` when the frames cannot be resolved; the page then falls back to a
    fixed range.
    """
    try:
        chain = [*arm_joints, tcp_frame]
        frames = bundle.model.frame_poses(bundle.model.neutral_q(), chain)
    except (KeyError, ValueError):  # a name that is not a frame, or ambiguous
        logger.warning("No reach bound for %s: its joint frames did not resolve.", tcp_frame)
        return None
    points = [np.asarray(frames[name][:3, 3], dtype=float) for name in chain]
    radius = float(sum(float(np.linalg.norm(b - a)) for a, b in zip(points, points[1:])))
    return {"center": [float(v) for v in points[0]], "radius": radius}


def _jsonable(value: Any) -> Any:
    """Turn mappings, tuples and numpy scalars into JSON-friendly values."""
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


class IKSetup:
    """The dual-arm solver bound to a bundle, plus the operation session around it."""

    def __init__(
        self,
        bundle: ModelBundle,
        *,
        torso_joint: str | None = None,
        left_arm_joints: Sequence[str] | None = None,
        right_arm_joints: Sequence[str] | None = None,
        head_joints: Sequence[str] | None = None,
        config_overrides: Mapping[str, Any] | None = None,
        trajectory_limits: TrajectoryLimits | None = None,
        sample_period_s: float | None = None,
        trajectory_profile: str = "simulation",
    ) -> None:
        """Build the solver, inferring joint groups from names when not given.

        Inference is by the substrings ``left`` / ``right`` / ``head`` / ``torso``
        in the URDF joint names, in tree (shoulder-to-wrist) order.  That holds
        for the Rakuda export and the synthetic fixture; anything else should
        pass the groups explicitly.  The groups used are recorded in
        :attr:`groups` and shown in the page so an inference is never silent.

        ``config_overrides`` replaces fields of the solver configuration; the
        VR server uses it to run the solver as a streaming controller (one step
        per pose sample) instead of iterating a jog to convergence.
        ``trajectory_limits`` and ``sample_period_s`` are the reference
        ceilings of the trajectory mode; the simulation profile is used when
        they are not given, and ``trajectory_profile`` names where they came
        from so the page can say so.
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
        settings: Dict[str, Any] = dict(
            # Per-step bounds stay in place so every path is one the machine
            # could take; the trajectory mode plays them back at their timing.
            max_joint_step_rad=0.05,
            compute_budget_s=5.0,
            max_state_age_s=5.0,
            # The trajectory mode integrates these steps at a fixed sample
            # period, so the acceleration bound means something here (the
            # legacy end-point mode still switches it off per request).
            max_joint_acceleration_rad_s2=8.0,
            # A little more Tikhonov damping than the controller default: it
            # penalises step size without biasing the equilibrium, which
            # keeps a straight (singular) arm from wandering along its
            # null space while a jog converges.
            damping=1e-3,
            # The natural-motion profile: hand tasks first, the posture and
            # the joint-motion costs second; the error is corrected with a
            # time constant so the response does not depend on the period.
            task_priority_mode="hierarchical",
            orientation_mode="position_only",
            gain_time_constant_s=0.08,
            joint_motion_cost={torso: 2.0},
            limit_avoidance_enabled=True,
            require_soft_limits=not unbounded,
        )
        settings.update(config_overrides or {})
        if settings.get("orientation_mode") == "axis_aligned" and not settings.get(
            "approach_axis_tcp"
        ):
            raise ValueError(
                "orientation_mode axis_aligned needs approach_axis_tcp (control.ik): the "
                "gripper's approach axis is stated, not assumed."
            )
        self.solver = DualArmIK(
            bundle.model,
            left_frame=bundle.tcp_frames["left"],
            right_frame=bundle.tcp_frames["right"],
            torso_joint=torso,
            left_arm_joints=left,
            right_arm_joints=right,
            head_joints=head,
            config=DualArmIKConfig(**settings),
        )
        self.groups: Dict[str, Any] = {
            "torso": torso,
            "left": left,
            "right": right,
            "head": head,
            "unbounded_continuous": unbounded,
        }
        self.workspace: Dict[str, Any] = {
            side: _reach_bound(bundle, self.groups[side], bundle.tcp_frames[side])
            for side in ("left", "right")
        }
        self._bundle = bundle
        self.trajectory_limits = trajectory_limits or SIMULATION_TRAJECTORY_LIMITS
        self.sample_period_s = float(sample_period_s or SIMULATION_SAMPLE_PERIOD_S)
        self.trajectory_profile = trajectory_profile
        self.collision_modelled = bundle.model.collision_model is not None and bool(
            len(bundle.model.collision_model.collisionPairs)
        )
        # The operation session: the last trajectory (to resume a re-target
        # from the reference's state at playback time) and its sequence.
        self._last_trajectory: JointTrajectory | None = None
        self._last_seq: int | None = None
        self._resets = 0

    def describe_config(self) -> Dict[str, Any]:
        """The solver settings that shape motion, for logs and the page."""
        cfg = self.solver.config
        out = {
            name: getattr(cfg, name)
            for name in (
                "task_priority_mode",
                "orientation_mode",
                "approach_axis_tcp",
                "posture_cost",
                "joint_motion_cost",
                "velocity_smoothing_cost",
                "limit_avoidance_enabled",
                "gain_time_constant_s",
                "damping",
                "max_joint_velocity_rad_s",
                "max_joint_acceleration_rad_s2",
                "max_joint_step_rad",
                "position_limit_margin_rad",
                "solver",
            )
            if hasattr(cfg, name)
        }
        out["orientation_mode"] = self.solver.orientation_mode
        return out

    def describe(self) -> Dict[str, Any]:
        """What the page needs to know about the solver session."""
        cfg = self.solver.config
        return {
            "available": True,
            "groups": self.groups,
            "workspace": self.workspace,
            "geometric_study_only": self.geometric_study_only,
            "priority_mode": cfg.task_priority_mode,
            "orientation_mode": self.solver.orientation_mode,
            "orientation_modes": ["position_only", "pose"]
            + (["axis_aligned"] if cfg.approach_axis_tcp is not None else []),
            "approach_axis_tcp": None
            if cfg.approach_axis_tcp is None
            else list(cfg.approach_axis_tcp),
            "orientation_weight": self.solver.orientation_cost,
            "collision_modelled": self.collision_modelled,
            "trajectory": {
                "profile": self.trajectory_profile,
                "sample_period_s": self.sample_period_s,
                **self.trajectory_limits.describe(),
            },
            "config": _jsonable(self.describe_config()),
            "resets": self._resets,
        }

    def reset(self, positions: Mapping[str, float] | None = None) -> Dict[str, Any]:
        """Forget the session's velocity history; re-anchor the posture reference.

        This is the explicit reset the plan allows: start-up, a mode switch, a
        manual pose change, resuming after a stop.  A configured preferred
        posture stays; otherwise the given configuration becomes the posture
        the secondary objective settles towards.
        """
        q = None
        if positions is not None:
            current = {name: 0.0 for name in self._bundle.joint_order}
            current.update({k: float(v) for k, v in positions.items()})
            q = self._bundle.positions_to_q(current)
        # A configured preferred posture is kept; otherwise the pose given here
        # becomes the posture the secondary objective settles towards.
        self.solver.reset(q if self.solver.config.posture_reference is None else None)
        self._last_trajectory = None
        self._last_seq = None
        self._resets += 1
        return {"ok": True, "resets": self._resets}

    def _apply_mode(self, orientation_mode: str | None, orientation_weight: float | None) -> None:
        if orientation_mode is not None:
            self.solver.set_orientation_mode(str(orientation_mode))
        if orientation_weight is not None:
            if not 0.0 <= float(orientation_weight) <= 10.0:
                raise ValueError("orientation_weight must be within [0, 10].")
            self.solver.set_task_costs(orientation_cost=float(orientation_weight))

    def solve_trajectory(
        self,
        positions: Mapping[str, float],
        targets: Mapping[str, Any],
        *,
        seq: int | None = None,
        resume: Mapping[str, Any] | None = None,
        torso_policy: str = "optimize",
        torso_velocity_rad_s: float = 0.0,
        inactive_arm_policy: str = "hold_joints",
        orientation_mode: str | None = None,
        orientation_weight: float | None = None,
        max_duration_s: float = 4.0,
        include_samples: bool = True,
    ) -> Dict[str, Any]:
        """Walk a reference from the hands to ``targets`` and return the timed trajectory.

        Args:
            positions: Where the model is now (the page's displayed joints).
            targets: ``{"left"|"right": {"p": [3], "q": [4]}}`` goals.
            seq: The page's request sequence, echoed so a late reply can be
                told from a current one.
            resume: ``{"seq": n, "t": seconds}``: the request arrived while
                the trajectory ``n`` was playing, ``t`` seconds in.  The
                reference resumes from its pose and velocity at that instant
                and the solver from the joint velocity there, so the new goal
                bends the motion instead of restarting it.
            torso_policy: ``fixed`` / ``manual`` / ``optimize``.
            torso_velocity_rad_s: For the manual policy.
            inactive_arm_policy: ``hold_joints`` or ``hold_world``.
            orientation_mode: Switch the hands' mode first (``None`` keeps it).
            orientation_weight: Orientation (or axis) weight (``None`` keeps it).
            max_duration_s: Duration budget of this request; a run still
                moving at the end comes back ``truncated`` and the page asks
                for the continuation.
            include_samples: Whether to return every sample.
        """
        goals: Dict[str, np.ndarray] = {}
        for side in ("left", "right"):
            entry = targets.get(side)
            if entry:
                goals[side] = se3_from_quat_xyzw(entry["p"], entry["q"])
        if not goals:
            raise ValueError(
                "At least one of targets.left / targets.right is required; with both hands "
                "disabled there is nothing to solve."
            )
        policy = TorsoPolicy(torso_policy)
        inactive = InactiveArmPolicy(inactive_arm_policy)
        self._apply_mode(orientation_mode, orientation_weight)
        if not 0.0 < float(max_duration_s) <= 60.0:
            raise ValueError("max_duration_s must be within (0, 60].")

        current = {name: 0.0 for name in self._bundle.joint_order}
        current.update({k: float(v) for k, v in positions.items()})
        references: Dict[str, PoseReference] = {}
        resumed_from = None
        if (
            resume
            and self._last_trajectory is not None
            and self._last_seq is not None
            and int(resume.get("seq", -1)) == self._last_seq
        ):
            t = max(0.0, float(resume.get("t", 0.0)))
            last = self._last_trajectory
            for side in goals:
                try:
                    pose, velocity = last.reference_at(t, side)
                except KeyError:
                    continue
                references[side] = PoseReference(
                    pose, self.trajectory_limits, linear_velocity=[float(v) for v in velocity]
                )
            # The joint velocity at that instant, for the acceleration bound.
            index = 0
            for i, sample in enumerate(last.samples):
                if sample.time_from_start_s <= t:
                    index = i
            if 0 < index < len(last.samples):
                a, b = last.samples[index - 1], last.samples[index]
                dt = b.time_from_start_s - a.time_from_start_s
                if dt > 0.0:
                    self.solver.seed_velocity(
                        {k: (b.joints[k] - a.joints[k]) / dt for k in b.joints}, dt
                    )
            resumed_from = {"seq": self._last_seq, "t": t}
        elif resume:
            # A resume for a trajectory this session no longer holds (or a
            # different one): start the reference at rest where the hands are.
            self.solver.reset()

        trajectory = run_trajectory(
            self.solver,
            current,
            goals,
            self.trajectory_limits,
            dt=self.sample_period_s,
            max_duration_s=float(max_duration_s),
            frames=self._bundle.tcp_frames,
            torso_policy=policy,
            torso_velocity_rad_s=torso_velocity_rad_s if policy is TorsoPolicy.MANUAL else 0.0,
            inactive_arm_policy=inactive,
            references=references or None,
        )
        self._last_trajectory = trajectory
        self._last_seq = None if seq is None else int(seq)
        last_result = trajectory.last_result
        final = trajectory.final_joints or dict(current)
        commandable = bool(trajectory.samples) and len(trajectory.samples) > 1
        errors = (
            last_result.errors()
            if last_result is not None
            else {k: None for k in ("left_position_m", "right_position_m")}
        )
        goal_errors = trajectory.samples[-1].goal_error_m if trajectory.samples else {}
        goal_angles = (
            trajectory.samples[-1].goal_orientation_error_rad if trajectory.samples else {}
        )
        return {
            "mode": "trajectory",
            "seq": seq,
            "resumed_from": resumed_from,
            "status": trajectory.status.value,
            "commandable": commandable,
            "stalled": trajectory.status.is_stall,
            "truncated": trajectory.truncated,
            "message": trajectory.message,
            "duration_s": trajectory.duration_s,
            "dt_s": trajectory.dt_s,
            "n_samples": len(trajectory.samples),
            "samples": [s.describe() for s in trajectory.samples] if include_samples else [],
            "joints": final,
            "errors": errors,
            "goal_error_m": goal_errors,
            "goal_orientation_error_rad": goal_angles,
            "active_limits": list(last_result.active_limits) if last_result else [],
            "min_singular_value": None if last_result is None else last_result.min_singular_value,
            "torso_velocity_rad_s": 0.0
            if last_result is None
            else last_result.torso_velocity_rad_s,
            "orientation_mode": self.solver.orientation_mode,
            "orientation_weight": self.solver.orientation_cost,
            "priority_mode": self.solver.config.task_priority_mode,
            "inactive_arm_policy": inactive.value,
            "torso_policy": policy.value,
            "enabled": sorted(goals),
            "limits": self.trajectory_limits.describe(),
            "collision_modelled": self.collision_modelled,
            "goals": {side: matrix_to_pose(T) for side, T in goals.items()},
            "reference": {
                side: matrix_to_pose(ref.pose) for side, ref in trajectory.references.items()
            },
        }

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
        orientation_mode: str | None = None,
        inactive_arm_policy: str = "hold_joints",
    ) -> Dict[str, Any]:
        """Iterate the differential solver from ``positions`` until it converges (end-point mode).

        Args:
            positions: Starting joint configuration.
            targets: ``{"left"|"right": {"p": [3], "q": [4]}}`` for the driven hands.
            torso_policy: ``fixed`` / ``manual`` / ``optimize``.
            torso_velocity_rad_s: Torso rate for the ``manual`` policy.
            inactive_arm_policy: ``hold_joints`` lets an undriven TCP move with
                the torso; ``hold_world`` requests the legacy world TCP hold.
            iterations: Iteration budget.
            dt: Per-iteration step time; with the step bounds this sets the
                largest joint move per iteration.
            orientation_weight: Orientation task weight; ``None`` keeps the
                solver's current weight.
            orientation_mode: ``position_only`` / ``pose`` / ``axis_aligned``;
                ``None`` keeps the current mode.

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
            inactive_arm_policy=InactiveArmPolicy(inactive_arm_policy),
        )

        self._apply_mode(orientation_mode, orientation_weight)

        current = {name: 0.0 for name in self._bundle.joint_order}
        current.update({k: float(v) for k, v in positions.items()})
        # The end-point mode is stateless by design: regularise towards where
        # the arm *is*, iterate to convergence, keep nothing.  (The trajectory
        # mode is the one that keeps a session.)  No acceleration window
        # either: nothing integrates these steps in time.
        self.solver.reset(self._bundle.positions_to_q(current))
        acceleration = self.solver.config.max_joint_acceleration_rad_s2
        self.solver.config.max_joint_acceleration_rad_s2 = None
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
        self.solver.config.max_joint_acceleration_rad_s2 = acceleration
        self.solver.reset()
        assert result is not None
        final = (
            current
            if result.is_commandable
            else {name: 0.0 for name in self._bundle.joint_order}
            | {k: float(v) for k, v in positions.items()}
        )
        return {
            "mode": "endpoint",
            "status": result.status.value,
            "commandable": result.is_commandable,
            "message": result.message,
            "iterations": steps,
            "joints": final,
            "errors": result.errors(),
            "active_limits": list(result.active_limits),
            "torso_velocity_rad_s": result.torso_velocity_rad_s,
            "stalled": stalled,
            "orientation_weight": self.solver.orientation_cost,
            "orientation_mode": self.solver.orientation_mode,
            "priority_mode": self.solver.config.task_priority_mode,
            "inactive_arm_policy": target.inactive_arm_policy.value,
            "torso_policy": policy.value,
            "min_singular_value": result.min_singular_value,
            "collision_modelled": self.collision_modelled,
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
            elif path == "/api/ik/reset":
                self._send_json(self.server.ik_reset(body))
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
        payload["ik"] = None if self.ik is None else self.ik.describe()
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
        mode = str(body.get("mode", "endpoint"))
        if mode not in ("endpoint", "trajectory"):
            raise ValueError("mode must be 'endpoint' or 'trajectory'.")
        orientation_weight = (
            None if body.get("orientation_weight") is None else float(body["orientation_weight"])
        )
        orientation_mode = (
            None if body.get("orientation_mode") is None else str(body["orientation_mode"])
        )
        started = time.perf_counter()
        with self._lock:
            if mode == "trajectory":
                result = self.ik.solve_trajectory(
                    joints,
                    targets,
                    seq=None if body.get("seq") is None else int(body["seq"]),
                    resume=body.get("resume") if isinstance(body.get("resume"), dict) else None,
                    torso_policy=str(body.get("torso_policy", "optimize")),
                    inactive_arm_policy=str(body.get("inactive_arm_policy", "hold_joints")),
                    torso_velocity_rad_s=float(body.get("torso_velocity_rad_s", 0.0)),
                    orientation_mode=orientation_mode,
                    orientation_weight=orientation_weight,
                    max_duration_s=float(body.get("max_duration_s", 4.0)),
                    include_samples=bool(body.get("include_samples", True)),
                )
            else:
                result = self.ik.solve(
                    joints,
                    targets,
                    torso_policy=str(body.get("torso_policy", "fixed")),
                    inactive_arm_policy=str(body.get("inactive_arm_policy", "hold_joints")),
                    torso_velocity_rad_s=float(body.get("torso_velocity_rad_s", 0.0)),
                    iterations=int(body.get("iterations", 200)),
                    dt=float(body.get("dt", 0.02)),
                    orientation_weight=orientation_weight,
                    orientation_mode=orientation_mode,
                )
            result["poses"] = self.bundle.poses(result["joints"])
        result["timing_ms"] = (time.perf_counter() - started) * 1e3
        return result

    def ik_reset(self, body: Mapping[str, Any]) -> Dict[str, Any]:
        """Forget the solver session's history (see :meth:`IKSetup.reset`)."""
        if self.ik is None:
            raise RuntimeError("The solver is not available.")
        joints = body.get("joints")
        if joints is not None and not isinstance(joints, dict):
            raise ValueError("'joints' must be an object when given.")
        with self._lock:
            return self.ik.reset(joints)


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
