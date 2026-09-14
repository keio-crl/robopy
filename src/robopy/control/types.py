"""SI-unit state, target, result and mode types shared by the Rakuda control stack.

Every quantity in this module is in SI units (radians, radians/second, amperes,
newton-metres, seconds).  Raw DYNAMIXEL counts never appear here: the single
conversion boundary lives in :mod:`robopy.control.joint_mapping`.

Nothing in this module imports hardware drivers, Pinocchio or Pink, so it can be
imported without the ``kinematics`` extra and without a serial port.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

__all__ = [
    "BilateralOutput",
    "ControlMode",
    "DualArmTarget",
    "JointState",
    "LimitFlags",
    "ServoState",
    "TorsoPolicy",
    "monotonic_ns",
    "quat_xyzw_to_matrix",
    "se3_from_quat_xyzw",
]


def monotonic_ns() -> int:
    """Monotonic clock reading in nanoseconds.

    All deadlines and staleness checks in the control stack use this clock.  It
    never runs backwards and is unaffected by wall-clock adjustments.
    """
    return time.monotonic_ns()


class ControlMode(Enum):
    """Control modes implemented in this first iteration.

    Attributes:
        POSITION_TELEOP: Legacy behaviour -- leader joint positions are copied to
            the follower.  No inverse kinematics, no current control.
        CARTESIAN_TELEOP: Left/right TCP targets are converted to follower joint
            position targets by the dual-arm IK.  Uses position control.
        BILATERAL_JOINT: Joint-space virtual spring/damper coupling between
            leader and follower.  Uses current control.  No inverse kinematics.
    """

    POSITION_TELEOP = "position_teleop"
    CARTESIAN_TELEOP = "cartesian_teleop"
    BILATERAL_JOINT = "bilateral_joint"


class TorsoPolicy(Enum):
    """How the shared torso yaw joint is treated by the dual-arm IK.

    Attributes:
        FIXED: The torso is held.  Its IK velocity is constrained to zero, so it
            contributes nothing to either hand task.
        MANUAL: The torso velocity is commanded from outside the IK.  It enters
            both hand tasks as a *known* quantity (feed-forward), and the arm
            joints compensate for it.
        OPTIMIZE: The torso velocity is a free decision variable shared by both
            hand tasks.
    """

    FIXED = "fixed"
    MANUAL = "manual"
    OPTIMIZE = "optimize"


class ServoState(Enum):
    """Lifecycle of a servo loop.

    See :class:`robopy.control.mode_manager.ModeManager` for the allowed
    transitions and for what each state is permitted to write to the bus.
    """

    DISCONNECTED = "disconnected"
    CONFIGURING = "configuring"
    READY = "ready"
    ALIGNING = "aligning"
    RUNNING = "running"
    STOPPING = "stopping"
    FAULT = "fault"


@dataclass(frozen=True)
class JointState:
    """A measured joint-space snapshot in SI units.

    The ordering of every array is given by :attr:`joint_names`; callers must
    never assume a particular order, they must look joint names up.

    Attributes:
        joint_names: Joint names, in the order used by every array below.
        position_rad: Joint positions in radians.
        velocity_rad_s: Joint velocities in radians per second.
        current_a: Measured motor current in amperes (signed, in the joint's own
            positive direction).  This is a *measurement*, not a torque.
        torque_estimate_nm: Optional torque estimate derived from
            :attr:`current_a` through a per-joint model.  ``None`` when no
            validated model is available -- it is deliberately a separate field
            so that a current measurement is never silently read as a torque.
        valid: Per-joint validity.  ``False`` means the motor did not answer, or
            answered with a hardware error; the corresponding array entries are
            not meaningful.
        read_start_ns: Monotonic time just before the read transaction started.
        read_end_ns: Monotonic time just after the read transaction finished.
        sequence: Monotonically increasing snapshot counter for this bus.
        mode_generation: Command generation this snapshot belongs to.  A
            snapshot from an older generation must not be used to produce new
            commands (see :class:`ModeManager`).
    """

    joint_names: Tuple[str, ...]
    position_rad: NDArray[np.float64]
    velocity_rad_s: NDArray[np.float64]
    current_a: NDArray[np.float64]
    valid: NDArray[np.bool_]
    read_start_ns: int
    read_end_ns: int
    sequence: int
    mode_generation: int
    torque_estimate_nm: NDArray[np.float64] | None = None

    def __post_init__(self) -> None:
        n = len(self.joint_names)
        for name in ("position_rad", "velocity_rad_s", "current_a", "valid"):
            array = getattr(self, name)
            if array.shape != (n,):
                raise ValueError(f"JointState.{name} must have shape ({n},), got {array.shape}.")
        if self.torque_estimate_nm is not None and self.torque_estimate_nm.shape != (n,):
            raise ValueError("JointState.torque_estimate_nm must match joint_names length.")
        if self.read_end_ns < self.read_start_ns:
            raise ValueError("JointState.read_end_ns must not precede read_start_ns.")

    @property
    def all_valid(self) -> bool:
        """True when every joint in this snapshot answered correctly."""
        return bool(np.all(self.valid))

    @property
    def acquisition_span_s(self) -> float:
        """Duration of the read transaction in seconds.

        A SyncRead reduces the number of packets but does *not* make the motors
        sample simultaneously; this span bounds how far apart the samples in one
        snapshot can be.
        """
        return (self.read_end_ns - self.read_start_ns) * 1e-9

    def age_s(self, now_ns: int | None = None) -> float:
        """Seconds elapsed since the read finished."""
        now = monotonic_ns() if now_ns is None else now_ns
        return (now - self.read_end_ns) * 1e-9

    def index(self, joint_name: str) -> int:
        """Index of ``joint_name`` within this snapshot."""
        try:
            return self.joint_names.index(joint_name)
        except ValueError as exc:
            raise KeyError(f"Joint '{joint_name}' is not in this snapshot.") from exc

    def select(self, joint_names: Sequence[str]) -> "JointState":
        """Return a snapshot restricted to ``joint_names``, preserving their order."""
        idx = np.asarray([self.index(name) for name in joint_names], dtype=int)
        return JointState(
            joint_names=tuple(joint_names),
            position_rad=self.position_rad[idx],
            velocity_rad_s=self.velocity_rad_s[idx],
            current_a=self.current_a[idx],
            valid=self.valid[idx],
            torque_estimate_nm=(
                None if self.torque_estimate_nm is None else self.torque_estimate_nm[idx]
            ),
            read_start_ns=self.read_start_ns,
            read_end_ns=self.read_end_ns,
            sequence=self.sequence,
            mode_generation=self.mode_generation,
        )

    def positions_dict(self) -> Dict[str, float]:
        """Joint positions as a ``{name: rad}`` mapping."""
        return {name: float(self.position_rad[i]) for i, name in enumerate(self.joint_names)}


def quat_xyzw_to_matrix(quat_xyzw: Sequence[float]) -> NDArray[np.float64]:
    """Rotation matrix from a quaternion in ``(x, y, z, w)`` order.

    ``xyzw`` is the only quaternion order used in this package; it matches
    Pinocchio's convention.

    Args:
        quat_xyzw: Quaternion components ``(x, y, z, w)``.  Need not be unit
            norm; it is normalised here.

    Returns:
        A ``(3, 3)`` rotation matrix.
    """
    q = np.asarray(quat_xyzw, dtype=np.float64)
    if q.shape != (4,):
        raise ValueError("Quaternion must have 4 components in (x, y, z, w) order.")
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("Quaternion norm is too small to normalise.")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def se3_from_quat_xyzw(
    position_m: Sequence[float],
    quat_xyzw: Sequence[float],
) -> NDArray[np.float64]:
    """Build a 4x4 homogeneous transform from a position and an ``xyzw`` quaternion."""
    T = np.eye(4)
    T[:3, :3] = quat_xyzw_to_matrix(quat_xyzw)
    T[:3, 3] = np.asarray(position_m, dtype=np.float64)
    return T


@dataclass(frozen=True)
class DualArmTarget:
    """Left/right TCP targets expressed in the model's ``root`` frame.

    Attributes:
        left_target: ``(4, 4)`` homogeneous transform of the left TCP target in
            the ``root`` frame, or ``None`` when the left task is not driven.
        right_target: Same for the right TCP.
        left_enabled: Whether the left task should track ``left_target``.  When
            ``False`` the solver uses a hold target instead (see
            :class:`robopy.kinematics.dual_arm_ik.DualArmIK`).
        right_enabled: Same for the right task.
        torso_policy: How to treat the shared torso yaw joint.
        torso_velocity_rad_s: Commanded torso velocity, used only when
            ``torso_policy`` is :attr:`TorsoPolicy.MANUAL`.
        created_ns: Monotonic time the target was produced.
        expiry_ns: Monotonic time after which the target must not be used.  A
            consumer that sees an expired target must not issue new motion.
        rebaseline: Request that held (non-driven) TCP targets be re-captured
            from the current measured pose on this step.  Hold targets are
            otherwise latched, never overwritten every cycle.
    """

    left_target: NDArray[np.float64] | None = None
    right_target: NDArray[np.float64] | None = None
    left_enabled: bool = True
    right_enabled: bool = True
    torso_policy: TorsoPolicy = TorsoPolicy.FIXED
    torso_velocity_rad_s: float = 0.0
    created_ns: int = field(default_factory=monotonic_ns)
    expiry_ns: int | None = None
    rebaseline: bool = False

    def __post_init__(self) -> None:
        for name in ("left_target", "right_target"):
            T = getattr(self, name)
            if T is not None and np.asarray(T).shape != (4, 4):
                raise ValueError(f"DualArmTarget.{name} must be a 4x4 transform or None.")
        if self.left_enabled and self.left_target is None:
            raise ValueError("left_enabled is True but left_target is None.")
        if self.right_enabled and self.right_target is None:
            raise ValueError("right_enabled is True but right_target is None.")
        if self.torso_policy is not TorsoPolicy.MANUAL and self.torso_velocity_rad_s != 0.0:
            raise ValueError(
                "torso_velocity_rad_s is only meaningful with TorsoPolicy.MANUAL; "
                f"got policy={self.torso_policy.value}."
            )

    def is_expired(self, now_ns: int | None = None) -> bool:
        """Whether this target's deadline has passed."""
        if self.expiry_ns is None:
            return False
        now = monotonic_ns() if now_ns is None else now_ns
        return now > self.expiry_ns


@dataclass(frozen=True)
class LimitFlags:
    """Which limits were active when a command was produced.

    Each entry maps a joint name to ``True`` when that limit clipped the command
    for that joint on this cycle.  Empty dictionaries mean no clipping.
    """

    torque_saturated: Dict[str, bool] = field(default_factory=dict)
    current_limited: Dict[str, bool] = field(default_factory=dict)
    rate_limited: Dict[str, bool] = field(default_factory=dict)

    @property
    def any_active(self) -> bool:
        """True when at least one limit clipped at least one joint."""
        return any(
            any(flags.values())
            for flags in (self.torque_saturated, self.current_limited, self.rate_limited)
        )


@dataclass(frozen=True)
class BilateralOutput:
    """Result of one bilateral control-law evaluation.

    Torques are in newton-metres, in each machine's own calibrated joint frame.
    Conversion to motor current happens in the motor-side adapter, never here.

    Attributes:
        joint_names: Coupled joint names (follower naming), in array order.
        leader_torque_nm: Commanded leader joint torques.
        follower_torque_nm: Commanded follower joint torques.
        coupling_torque_nm: The coupling term ``u = K e + D e_dot`` itself,
            before gravity compensation and before the sign convention of each
            side is applied.  Reported for diagnostics and energy accounting.
        position_error_rad: ``e = q_s - (S q_m + b)``.
        velocity_error_rad_s: ``e_dot = v_s - S v_m``.
        coupling_scale: Ramp factor in ``[0, 1]`` currently applied to the
            coupling gains (used to fade in after an alignment).
        limits: Which limits clipped this command.
        feedback_active: Whether force feedback to the leader is being issued.
        dt_s: The timestep actually used.
    """

    joint_names: Tuple[str, ...]
    leader_torque_nm: NDArray[np.float64]
    follower_torque_nm: NDArray[np.float64]
    coupling_torque_nm: NDArray[np.float64]
    position_error_rad: NDArray[np.float64]
    velocity_error_rad_s: NDArray[np.float64]
    coupling_scale: float
    limits: LimitFlags
    feedback_active: bool
    dt_s: float

    def as_dict(self, side: str) -> Dict[str, float]:
        """Torque command for ``side`` (``"leader"`` or ``"follower"``) as a mapping."""
        if side == "leader":
            values = self.leader_torque_nm
        elif side == "follower":
            values = self.follower_torque_nm
        else:
            raise ValueError("side must be 'leader' or 'follower'.")
        return {name: float(values[i]) for i, name in enumerate(self.joint_names)}


def ordered_array(
    values: Mapping[str, float],
    joint_names: Sequence[str],
    *,
    field_name: str = "values",
) -> NDArray[np.float64]:
    """Convert a ``{joint: value}`` mapping into an array ordered by ``joint_names``.

    Dictionary insertion order is ignored on purpose: the caller's ``joint_names``
    is the single source of truth for ordering.

    Raises:
        KeyError: If a joint in ``joint_names`` is missing from ``values``.
    """
    missing = [name for name in joint_names if name not in values]
    if missing:
        raise KeyError(f"{field_name} is missing joint(s): {missing}")
    return np.asarray([float(values[name]) for name in joint_names], dtype=np.float64)
