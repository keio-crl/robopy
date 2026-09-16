"""Headset orientation -> ``head_yaw`` / ``head_pitch`` joint targets.

Two things about the head are *not* assumed here but read off the model:

* **the sign of each joint** -- whether a positive ``head_yaw_dof`` turns the
  head left or right depends on which way the URDF axis points (on the Rakuda
  export it is world -Z, so positive yaw turns *right*);
* **the neutral configuration** -- the joint angles at which the head camera
  looks straight ahead.  On the Rakuda export the URDF zero has the head turned
  about 22 degrees to the right, so "zero the joints" is not "look forward".

:meth:`HeadJointMapping.from_model` derives both from forward kinematics, and
records how (``forward_source``) so the derivation is never silent.  The one
remaining assumption is stated in :attr:`HeadJointMapping.notes` when it is
made: without a camera frame, pitch is taken to be level at the URDF zero.

The head sits on the torso, and the torso yaw is a decision variable of the
arm solver.  The operator's head direction is meant in the *base* frame, so
the yaw command is compensated for the torso angle: ``head_yaw = signal +
k * (torso_yaw - torso_reference)`` with ``k`` the head-yaw change that keeps
the camera heading fixed per radian of torso (``-1`` on the Rakuda export,
where both axes point the same way; derived from the model, not assumed).

The tracker itself is deliberately simple -- re-centre, scale, clamp to the
joint limits with a margin, low-pass, rate-limit -- because the operator's
neck is the controller and anything cleverer fights it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

from .xr_math import axis_angle_of, azimuth_elevation, wrap_to_pi, yaw_pitch_of_forward

__all__ = ["HeadCommand", "HeadJointMapping", "HeadTracker", "HeadTrackingConfig"]

_Z = np.array([0.0, 0.0, 1.0])


def _joint_world_axis(
    model: Any, positions: Mapping[str, float], joint: str
) -> NDArray[np.float64]:
    """World-frame rotation axis of ``joint`` at ``positions`` (finite rotation)."""
    delta = 0.1
    moved = dict(positions)
    moved[joint] = float(moved[joint]) + delta
    R0 = model.frame_pose(model.q_from_positions(positions), joint)[:3, :3]
    R1 = model.frame_pose(model.q_from_positions(moved), joint)[:3, :3]
    axis, angle = axis_angle_of(R1 @ R0.T)
    if abs(angle - delta) > 1e-3:
        raise ValueError(
            f"Joint '{joint}' does not behave like a revolute joint (moved {angle:.4f} rad "
            f"for a {delta} rad step)."
        )
    return axis


@dataclass(frozen=True)
class HeadJointMapping:
    """How headset yaw/pitch map onto two head joints of a given model.

    Attributes:
        yaw_joint: URDF joint turning the head about the vertical.
        pitch_joint: URDF joint nodding the head.
        yaw_sign: ``+1`` when a positive joint angle turns the head left (the
            headset's positive yaw), ``-1`` otherwise.
        pitch_sign: ``+1`` when a positive joint angle looks up, ``-1`` otherwise.
        yaw_neutral_rad: Yaw joint angle at which the head faces forward (+X).
        pitch_neutral_rad: Pitch joint angle at which the camera is level.
        yaw_limits_rad: ``(lower, upper)`` of the yaw joint from the model.
        pitch_limits_rad: Same for the pitch joint.
        yaw_axis_world: The yaw joint's world axis at the reference pose.
        pitch_axis_world: The pitch joint's world axis at the reference pose.
        forward_source: How "forward" was established, for the record.
        notes: Assumptions that had to be made, if any.
        torso_joint: The torso yaw joint the head rides on, when the yaw
            command compensates for it; ``None`` otherwise.
        torso_coupling: Head-yaw radians to add per radian of torso yaw so the
            camera heading stays put in the base frame (``-1`` when both axes
            point the same way).
        torso_reference_rad: Torso angle at which ``yaw_neutral_rad`` was
            established.
    """

    yaw_joint: str
    pitch_joint: str
    yaw_sign: int
    pitch_sign: int
    yaw_neutral_rad: float
    pitch_neutral_rad: float
    yaw_limits_rad: Tuple[float, float]
    pitch_limits_rad: Tuple[float, float]
    yaw_axis_world: Tuple[float, float, float] = (0.0, 0.0, 1.0)
    pitch_axis_world: Tuple[float, float, float] = (0.0, -1.0, 0.0)
    forward_source: str = "given"
    notes: Tuple[str, ...] = ()
    torso_joint: str | None = None
    torso_coupling: float = 0.0
    torso_reference_rad: float = 0.0

    def __post_init__(self) -> None:
        if self.yaw_sign not in (1, -1) or self.pitch_sign not in (1, -1):
            raise ValueError("Joint signs must be +1 or -1.")
        for name, (lo, hi) in (
            ("yaw", self.yaw_limits_rad),
            ("pitch", self.pitch_limits_rad),
        ):
            if not (math.isfinite(lo) and math.isfinite(hi)) or lo >= hi:
                raise ValueError(f"{name} limits must be finite with lower < upper, got {lo, hi}.")

    @property
    def joints(self) -> Tuple[str, str]:
        """``(yaw_joint, pitch_joint)``."""
        return self.yaw_joint, self.pitch_joint

    def neutral_positions(self) -> Dict[str, float]:
        """``{joint: rad}`` at which the camera looks straight ahead."""
        return {self.yaw_joint: self.yaw_neutral_rad, self.pitch_joint: self.pitch_neutral_rad}

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly summary."""
        return {
            "yaw_joint": self.yaw_joint,
            "pitch_joint": self.pitch_joint,
            "yaw_sign": self.yaw_sign,
            "pitch_sign": self.pitch_sign,
            "yaw_neutral_rad": self.yaw_neutral_rad,
            "pitch_neutral_rad": self.pitch_neutral_rad,
            "yaw_limits_rad": list(self.yaw_limits_rad),
            "pitch_limits_rad": list(self.pitch_limits_rad),
            "yaw_axis_world": list(self.yaw_axis_world),
            "pitch_axis_world": list(self.pitch_axis_world),
            "forward_source": self.forward_source,
            "notes": list(self.notes),
            "torso_joint": self.torso_joint,
            "torso_coupling": self.torso_coupling,
            "torso_reference_rad": self.torso_reference_rad,
        }

    def torso_compensation_rad(self, torso_angle_rad: float | None) -> float:
        """Head-yaw offset that cancels the torso's contribution to the heading."""
        if self.torso_joint is None or torso_angle_rad is None:
            return 0.0
        return self.torso_coupling * (float(torso_angle_rad) - self.torso_reference_rad)

    @classmethod
    def from_model(
        cls,
        model: Any,
        yaw_joint: str,
        pitch_joint: str,
        *,
        camera_frame: str | None = None,
        camera_forward_axis: str = "auto",
        reference_positions: Mapping[str, float] | None = None,
        torso_joint: str | None = None,
    ) -> "HeadJointMapping":
        """Derive the mapping from a :class:`~robopy.kinematics.urdf_model.WholeBodyModel`.

        Args:
            model: The loaded model.  Both joints must be movable and have
                finite position limits (soft limits count).
            yaw_joint: The head yaw joint.
            pitch_joint: The head pitch joint.
            camera_frame: Frame of the head camera.  When given, the neutral
                configuration is the one at which the camera's forward axis
                points along +X and level.  When ``None``, the head's forward
                is taken perpendicular to the pitch axis and pitch is *assumed*
                level at the reference pose (recorded in ``notes``).
            camera_forward_axis: ``"x"``, ``"y"``, ``"z"``, with an optional
                leading ``-``, naming the camera frame's optical axis; or
                ``"auto"`` to take the frame axis best aligned with the head's
                geometric forward at the reference pose.  Frame conventions
                differ (ROS ``camera_link`` is X-forward, optical frames are
                Z-forward), and the choice is recorded in ``forward_source``.
            reference_positions: Joint configuration at which the axes are
                measured; the model's neutral configuration by default.  The
                yaw axis must be vertical there and the head must face within
                90 degrees of +X.
            torso_joint: The torso yaw joint carrying the head.  When given,
                the yaw command is compensated so the camera heading in the
                *base* frame follows the headset whatever the torso does; the
                coupling factor is measured on the model by finite differences.

        Returns:
            The derived mapping.

        Raises:
            ValueError: If a joint is missing, has no finite limits, or the
                head's geometry does not fit a yaw-then-pitch two-axis neck.
        """
        movable = tuple(model.movable_joint_names)
        for joint in (yaw_joint, pitch_joint):
            if joint not in movable:
                raise ValueError(f"'{joint}' is not a movable joint of the model.")
        lower, upper = model.position_limits([yaw_joint, pitch_joint])
        limits = {
            yaw_joint: (float(lower[0]), float(upper[0])),
            pitch_joint: (float(lower[1]), float(upper[1])),
        }
        for joint, (lo, hi) in limits.items():
            if not (math.isfinite(lo) and math.isfinite(hi)):
                raise ValueError(
                    f"'{joint}' has no finite position limit; set a soft limit so the head "
                    "cannot be commanded beyond its travel."
                )

        positions = {name: 0.0 for name in movable}
        if reference_positions is not None:
            positions.update({k: float(v) for k, v in reference_positions.items()})

        yaw_axis = _joint_world_axis(model, positions, yaw_joint)
        pitch_axis = _joint_world_axis(model, positions, pitch_joint)
        if abs(float(yaw_axis @ _Z)) < math.cos(math.radians(15.0)):
            raise ValueError(
                f"The yaw joint axis {np.round(yaw_axis, 3).tolist()} is not vertical at the "
                "reference pose; headset yaw cannot be mapped onto it."
            )
        if abs(float(pitch_axis @ _Z)) > math.sin(math.radians(15.0)):
            raise ValueError(
                f"The pitch joint axis {np.round(pitch_axis, 3).tolist()} is not horizontal at "
                "the reference pose; headset pitch cannot be mapped onto it."
            )
        yaw_sign = 1 if float(yaw_axis @ _Z) > 0.0 else -1

        # Geometric forward: horizontal, perpendicular to the pitch axis, and on
        # the side of +X (the head is assumed to face broadly forward at the
        # reference pose -- that is what "reference pose" means here).
        candidate = np.cross(_Z, pitch_axis)
        candidate /= np.linalg.norm(candidate)
        if candidate[0] < 0.0:
            candidate = -candidate
        forward0 = candidate
        pitch_sign = 1 if float(np.cross(pitch_axis, forward0) @ _Z) > 0.0 else -1

        notes: list[str] = []
        forward_source: str
        camera_axis_local: NDArray[np.float64] | None = None
        if camera_frame is not None:
            R_cam = model.frame_pose(model.q_from_positions(positions), camera_frame)[:3, :3]
            if camera_forward_axis == "auto":
                best, best_dot = None, -2.0
                for index, name in enumerate("xyz"):
                    for sign, label in ((1.0, name), (-1.0, f"-{name}")):
                        dot = float(sign * R_cam[:, index] @ forward0)
                        if dot > best_dot:
                            best, best_dot = (label, sign, index), dot
                assert best is not None
                if best_dot < math.cos(math.radians(30.0)):
                    notes.append(
                        f"No axis of frame '{camera_frame}' is within 30 degrees of the head's "
                        "geometric forward; the camera frame was ignored."
                    )
                else:
                    label, sign, index = best
                    camera_axis_local = np.zeros(3)
                    camera_axis_local[index] = sign
                    forward_source = f"camera_frame:{camera_frame}/{label} (auto)"
            else:
                label = camera_forward_axis.strip().lower()
                sign = -1.0 if label.startswith("-") else 1.0
                name = label.lstrip("+-")
                if name not in ("x", "y", "z"):
                    raise ValueError(
                        "camera_forward_axis must be x, y, z (optionally negated) or auto."
                    )
                camera_axis_local = np.zeros(3)
                camera_axis_local["xyz".index(name)] = sign
                forward_source = f"camera_frame:{camera_frame}/{label}"

        if camera_axis_local is not None:
            assert camera_frame is not None
            solved = dict(positions)
            converged = False
            for _ in range(30):
                R_cam = model.frame_pose(model.q_from_positions(solved), camera_frame)[:3, :3]
                az, el = azimuth_elevation(R_cam @ camera_axis_local)
                if abs(az) < 1e-7 and abs(el) < 1e-7:
                    converged = True
                    break
                solved[yaw_joint] -= yaw_sign * az
                solved[pitch_joint] -= pitch_sign * el
            if not converged:
                raise ValueError(
                    "Could not find a head configuration where the camera looks forward and "
                    "level; check the joint axes and the camera frame."
                )
            yaw_neutral = float(solved[yaw_joint])
            pitch_neutral = float(solved[pitch_joint])
        else:
            az, _ = azimuth_elevation(forward0)
            yaw_neutral = float(positions[yaw_joint]) - yaw_sign * az
            pitch_neutral = float(positions[pitch_joint])
            forward_source = "pitch_axis"
            notes.append(
                f"Pitch is assumed level at {pitch_joint}={pitch_neutral:.3f} rad (no camera "
                "frame given); if the camera looks up or down there, pass camera_frame."
            )

        torso_coupling = 0.0
        torso_reference = 0.0
        if torso_joint is not None:
            if torso_joint not in movable:
                raise ValueError(f"'{torso_joint}' is not a movable joint of the model.")
            torso_reference = float(positions[torso_joint])

            def heading(torso: float, head_yaw: float) -> float:
                probe = dict(positions)
                probe[torso_joint] = torso
                probe[yaw_joint] = head_yaw
                axis = _joint_world_axis(model, probe, pitch_joint)
                forward = np.cross(_Z, axis)
                if float(forward @ forward0) < 0.0:
                    forward = -forward
                return azimuth_elevation(forward)[0]

            delta = 0.05
            base = heading(torso_reference, float(positions[yaw_joint]))
            d_torso = wrap_to_pi(
                heading(torso_reference + delta, float(positions[yaw_joint])) - base
            )
            d_head = wrap_to_pi(
                heading(torso_reference, float(positions[yaw_joint]) + delta) - base
            )
            if abs(d_head) < 1e-6:
                raise ValueError("The head yaw joint does not change the camera heading.")
            torso_coupling = -d_torso / d_head
            if abs(d_torso) < 1e-6:
                notes.append(
                    f"'{torso_joint}' does not turn the head; no torso compensation applied."
                )
                torso_coupling = 0.0
            elif abs(abs(torso_coupling) - 1.0) > 0.05:
                notes.append(
                    f"Torso coupling {torso_coupling:+.3f} is not +/-1: the torso and head yaw "
                    "axes are not parallel, so the compensation is a linearisation."
                )

        for joint, value in ((yaw_joint, yaw_neutral), (pitch_joint, pitch_neutral)):
            lo, hi = limits[joint]
            if not lo <= value <= hi:
                notes.append(
                    f"The forward-looking configuration {joint}={value:.3f} rad lies outside "
                    f"the joint limits [{lo:.3f}, {hi:.3f}]; the head cannot look straight "
                    "ahead and will be clamped."
                )
        return cls(
            yaw_joint=yaw_joint,
            pitch_joint=pitch_joint,
            yaw_sign=yaw_sign,
            pitch_sign=pitch_sign,
            yaw_neutral_rad=yaw_neutral,
            pitch_neutral_rad=pitch_neutral,
            yaw_limits_rad=limits[yaw_joint],
            pitch_limits_rad=limits[pitch_joint],
            yaw_axis_world=tuple(float(v) for v in yaw_axis),  # type: ignore[arg-type]
            pitch_axis_world=tuple(float(v) for v in pitch_axis),  # type: ignore[arg-type]
            forward_source=forward_source,
            notes=tuple(notes),
            torso_joint=torso_joint,
            torso_coupling=float(torso_coupling),
            torso_reference_rad=torso_reference,
        )


@dataclass
class HeadTrackingConfig:
    """Tunables of :class:`HeadTracker`.

    Attributes:
        yaw_scale: Joint radians per headset radian of yaw (``1`` = one-to-one).
        pitch_scale: Same for pitch.
        filter_hz: First-order low-pass cut-off on the joint targets, or
            ``None`` for none.
        max_rate_rad_s: Largest change of a joint target per second.
        limit_margin_rad: Kept clear of each joint limit.
    """

    yaw_scale: float = 1.0
    pitch_scale: float = 1.0
    filter_hz: float | None = 8.0
    max_rate_rad_s: float = 2.5
    limit_margin_rad: float = 0.02

    def __post_init__(self) -> None:
        if self.yaw_scale <= 0.0 or self.pitch_scale <= 0.0:
            raise ValueError("Head scales must be positive.")
        if self.filter_hz is not None and self.filter_hz <= 0.0:
            raise ValueError("filter_hz must be positive or None.")
        if self.max_rate_rad_s <= 0.0:
            raise ValueError("max_rate_rad_s must be positive.")
        if self.limit_margin_rad < 0.0:
            raise ValueError("limit_margin_rad must not be negative.")


@dataclass(frozen=True)
class HeadCommand:
    """One tracker update.

    Attributes:
        targets_rad: ``{joint: rad}`` to command.
        yaw_input_rad: Headset yaw relative to the re-centred forward.
        pitch_input_rad: Headset pitch relative to the re-centred level.
        at_limit: Joints whose target was clamped this step.
        recentred_now: Whether this update (re-)centred the reference.
        torso_compensation_rad: Head-yaw offset applied to cancel the torso.
    """

    targets_rad: Dict[str, float]
    yaw_input_rad: float
    pitch_input_rad: float
    at_limit: Tuple[str, ...] = ()
    recentred_now: bool = False
    torso_compensation_rad: float = 0.0


@dataclass
class HeadTracker:
    """Map the headset orientation onto the two head joints.

    The first :meth:`update` re-centres automatically, so the head does not
    jump when a session begins with the operator looking somewhere arbitrary;
    :meth:`recenter` does it again on demand.  Call :meth:`reset` with the
    head's measured joint positions before the first update so the rate
    limiter starts from where the head actually is.
    """

    mapping: HeadJointMapping
    config: HeadTrackingConfig = field(default_factory=HeadTrackingConfig)

    def __post_init__(self) -> None:
        self._ref_yaw: float | None = None
        self._ref_pitch: float = 0.0
        self._output: Dict[str, float] | None = None
        self._last_time: float | None = None

    @property
    def recentred(self) -> bool:
        """Whether a reference orientation has been captured."""
        return self._ref_yaw is not None

    def reset(self, positions_rad: Mapping[str, float] | None = None) -> None:
        """Start the filter/rate limiter from ``positions_rad`` (or from nothing)."""
        self._last_time = None
        if positions_rad is None:
            self._output = None
            return
        self._output = {
            joint: float(positions_rad[joint])
            for joint in self.mapping.joints
            if joint in positions_rad
        }
        if len(self._output) != 2:
            self._output = None

    def recenter(self, head_rotation_operator: NDArray[np.float64]) -> None:
        """Take the current headset orientation as "forward and level"."""
        yaw, pitch = yaw_pitch_of_forward(head_rotation_operator)
        self._ref_yaw = yaw
        self._ref_pitch = pitch

    def hold(self) -> Dict[str, float]:
        """The last commanded targets, or the neutral configuration before any."""
        return dict(self._output) if self._output else self.mapping.neutral_positions()

    def update(
        self,
        head_rotation_operator: NDArray[np.float64],
        now_s: float,
        *,
        torso_angle_rad: float | None = None,
    ) -> HeadCommand:
        """Compute joint targets from the headset orientation.

        Args:
            head_rotation_operator: ``(3, 3)`` headset rotation in the operator
                frame (see :class:`~robopy.vr.xr_math.OperatorFrame`).
            now_s: Monotonic time of the sample, seconds.
            torso_angle_rad: Current torso yaw, when the mapping compensates
                for it; the head then keeps the operator's heading in the base
                frame while the arm solver turns the torso.

        Returns:
            The command; its ``targets_rad`` are clamped, filtered and
            rate-limited and can be sent to the joints directly.
        """
        recentred_now = False
        if self._ref_yaw is None:
            self.recenter(head_rotation_operator)
            recentred_now = True
        assert self._ref_yaw is not None
        yaw, pitch = yaw_pitch_of_forward(head_rotation_operator)
        d_yaw = wrap_to_pi(yaw - self._ref_yaw)
        d_pitch = pitch - self._ref_pitch

        m, c = self.mapping, self.config
        compensation = m.torso_compensation_rad(torso_angle_rad)
        raw = {
            m.yaw_joint: m.yaw_neutral_rad + m.yaw_sign * c.yaw_scale * d_yaw + compensation,
            m.pitch_joint: m.pitch_neutral_rad + m.pitch_sign * c.pitch_scale * d_pitch,
        }
        at_limit: list[str] = []
        for joint, (lo, hi) in (
            (m.yaw_joint, m.yaw_limits_rad),
            (m.pitch_joint, m.pitch_limits_rad),
        ):
            lo_m, hi_m = lo + c.limit_margin_rad, hi - c.limit_margin_rad
            if lo_m > hi_m:  # a margin wider than the travel: aim for the middle
                lo_m = hi_m = 0.5 * (lo + hi)
            clamped = min(hi_m, max(lo_m, raw[joint]))
            if clamped != raw[joint]:
                at_limit.append(joint)
            raw[joint] = clamped

        dt = 0.0 if self._last_time is None else max(0.0, now_s - self._last_time)
        self._last_time = now_s
        if self._output is None or dt <= 0.0:
            out = dict(raw) if self._output is None else dict(self._output)
        else:
            out = {}
            alpha = (
                1.0 if c.filter_hz is None else 1.0 - math.exp(-2.0 * math.pi * c.filter_hz * dt)
            )
            max_step = c.max_rate_rad_s * dt
            for joint, target in raw.items():
                previous = self._output[joint]
                desired = previous + alpha * (target - previous)
                step = min(max_step, max(-max_step, desired - previous))
                out[joint] = previous + step
        self._output = out
        return HeadCommand(
            targets_rad=dict(out),
            yaw_input_rad=d_yaw,
            pitch_input_rad=d_pitch,
            at_limit=tuple(at_limit),
            recentred_now=recentred_now,
            torso_compensation_rad=compensation,
        )

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly state."""
        return {
            "mapping": self.mapping.describe(),
            "config": {
                "yaw_scale": self.config.yaw_scale,
                "pitch_scale": self.config.pitch_scale,
                "filter_hz": self.config.filter_hz,
                "max_rate_rad_s": self.config.max_rate_rad_s,
                "limit_margin_rad": self.config.limit_margin_rad,
            },
            "recentred": self.recentred,
            "output_rad": dict(self._output) if self._output else None,
        }


def joint_world_axes(
    model: Any, joints: Sequence[str], positions: Mapping[str, float] | None = None
) -> Dict[str, Tuple[float, float, float]]:
    """World axes of ``joints`` at ``positions`` -- a diagnostic for the docs and tests."""
    full = {name: 0.0 for name in model.movable_joint_names}
    if positions:
        full.update({k: float(v) for k, v in positions.items()})
    return {
        joint: tuple(float(v) for v in _joint_world_axis(model, full, joint))  # type: ignore[misc]
        for joint in joints
    }
