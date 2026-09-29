"""A hand target is never jumped to: a reference pose walks there, and the solver follows it.

Three things are kept apart, as the plan asks: the operator's *goal* (where
the hand should end up), the *reference* (where the hand should be right
now, on its way there) and the *actual* TCP (where the model or the machine
has it).  :class:`PoseReference` produces the reference: a position that
moves towards the goal under a linear velocity and acceleration ceiling, and
a rotation that moves along the geodesic on SO(3) under angular ceilings --
never a difference of Euler components.  From rest a single segment is a
straight line and a shortest-path rotation.  A goal changed while the
reference is moving is re-planned from the reference's current position
*and velocity*: the motion bends smoothly towards the new goal instead of
stopping and starting again.  When the hand lags the reference by more than
a tolerance, the reference brakes rather than running ahead and letting the
error grow without bound.

:func:`run_trajectory` drives a :class:`~robopy.kinematics.dual_arm_ik.DualArmIK`
one step per reference sample and returns the timed joint trajectory --
``time_from_start_s``, joints, TCPs, residuals, active limits per sample --
which the viewer plays back at that timing (no re-interpolation between the
first and last joint angles) and a controller would feed to the machine.
The run ends when the reference has arrived and the hands are converged,
when it stalls, or when the duration budget is spent; the summary status
says which, and a stall is classified by what was binding when it happened:
a position limit (``limits_blocked``), a collision constraint
(``collision_blocked``) or nothing at all (``locally_stalled`` -- the local
method cannot progress from here, which is not a proof the goal is out of
reach).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.control.types import DualArmTarget, InactiveArmPolicy, JointState, TorsoPolicy

from .dual_arm_ik import DualArmIK, DualArmIKResult, DualArmIKStatus

__all__ = [
    "JointTrajectory",
    "PoseReference",
    "ReferenceSample",
    "TrajectoryLimits",
    "TrajectorySample",
    "rotation_log",
    "rotation_exp",
    "run_trajectory",
]


# -- SO(3) helpers ------------------------------------------------------------


def rotation_log(R: NDArray[np.float64]) -> NDArray[np.float64]:
    """Rotation vector (axis times angle, radians) of a rotation matrix."""
    R = np.asarray(R, dtype=np.float64)
    cos = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cos)
    if angle < 1e-12:
        return np.zeros(3)
    if angle < math.pi - 1e-6:
        axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        return axis / (2.0 * math.sin(angle)) * angle
    # Near pi the antisymmetric part vanishes: take the axis from R + I.
    M = R + np.eye(3)
    column = int(np.argmax(np.linalg.norm(M, axis=0)))
    axis = M[:, column]
    return axis / float(np.linalg.norm(axis)) * angle


def rotation_exp(rotation_vector: NDArray[np.float64]) -> NDArray[np.float64]:
    """Rotation matrix of a rotation vector (Rodrigues)."""
    r = np.asarray(rotation_vector, dtype=np.float64)
    angle = float(np.linalg.norm(r))
    if angle < 1e-12:
        return np.eye(3)
    k = r / angle
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


# -- the reference governor ---------------------------------------------------


@dataclass
class TrajectoryLimits:
    """Ceilings of the reference's motion.

    Attributes:
        max_linear_velocity_m_s: Translation speed ceiling.
        max_linear_acceleration_m_s2: Translation acceleration ceiling.
        max_angular_velocity_rad_s: Rotation speed ceiling.
        max_angular_acceleration_rad_s2: Rotation acceleration ceiling.
        lag_tolerance_m: When the hand's position lags the reference by more
            than this, the reference brakes.  ``None`` never brakes.
        lag_tolerance_rad: The same for the rotation.
    """

    max_linear_velocity_m_s: float
    max_linear_acceleration_m_s2: float
    max_angular_velocity_rad_s: float
    max_angular_acceleration_rad_s2: float
    lag_tolerance_m: float | None = 0.02
    lag_tolerance_rad: float | None = None

    def __post_init__(self) -> None:
        for name in (
            "max_linear_velocity_m_s",
            "max_linear_acceleration_m_s2",
            "max_angular_velocity_rad_s",
            "max_angular_acceleration_rad_s2",
        ):
            value = getattr(self, name)
            if not (isinstance(value, (int, float)) and math.isfinite(value) and value > 0.0):
                raise ValueError(f"{name} must be a positive number, got {value!r}.")
        for name in ("lag_tolerance_m", "lag_tolerance_rad"):
            value = getattr(self, name)
            if value is not None and value <= 0.0:
                raise ValueError(f"{name} must be positive or None.")

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly view."""
        return {
            "max_linear_velocity_m_s": self.max_linear_velocity_m_s,
            "max_linear_acceleration_m_s2": self.max_linear_acceleration_m_s2,
            "max_angular_velocity_rad_s": self.max_angular_velocity_rad_s,
            "max_angular_acceleration_rad_s2": self.max_angular_acceleration_rad_s2,
            "lag_tolerance_m": self.lag_tolerance_m,
            "lag_tolerance_rad": self.lag_tolerance_rad,
        }


def _governor_step(
    error: NDArray[np.float64],
    velocity: NDArray[np.float64],
    v_max: float,
    a_max: float,
    dt: float,
    *,
    brake: bool,
) -> NDArray[np.float64]:
    """One step of a bounded-velocity, bounded-acceleration tracker.

    The desired speed towards the goal is the largest that can still stop at
    it under ``a_max`` in discrete time -- ``(-a dt + sqrt(a^2 dt^2 + 8 a d))
    / 2``, which is the exact one-dimensional answer -- capped at ``v_max``.
    The velocity moves towards that desire by at most ``a_max dt``.  When
    braking, the desire is zero.  Applied to a vector the direction of the
    desire is the direction of the error, so from rest the path is straight
    and a mid-motion re-target bends the path instead of restarting it.
    """
    distance = float(np.linalg.norm(error))
    if brake or distance < 1e-12:
        desired = np.zeros(3)
    else:
        stoppable = (-a_max * dt + math.sqrt(a_max * a_max * dt * dt + 8.0 * a_max * distance)) / 2
        speed = min(v_max, stoppable)
        desired = error / distance * speed
    delta = desired - velocity
    step = float(np.linalg.norm(delta))
    max_delta = a_max * dt
    if step > max_delta:
        delta = delta / step * max_delta
    return velocity + delta


@dataclass(frozen=True)
class ReferenceSample:
    """The reference at one instant.

    Attributes:
        pose: ``(4, 4)`` reference pose.
        linear_velocity: ``(3,)`` m/s.
        angular_velocity: ``(3,)`` rad/s, world frame.
        arrived: Position and rotation are both at the goal and at rest.
        braking: The reference was braking this step because the hand lagged.
    """

    pose: NDArray[np.float64]
    linear_velocity: NDArray[np.float64]
    angular_velocity: NDArray[np.float64]
    arrived: bool
    braking: bool = False


class PoseReference:
    """A pose that moves towards a goal under velocity and acceleration ceilings."""

    def __init__(
        self,
        pose: NDArray[np.float64],
        limits: TrajectoryLimits,
        *,
        linear_velocity: Sequence[float] | None = None,
        angular_velocity: Sequence[float] | None = None,
        arrival_tolerance_m: float = 1e-4,
        arrival_tolerance_rad: float = 1e-3,
    ) -> None:
        """Start the reference at ``pose``, at rest unless velocities are given."""
        T = np.asarray(pose, dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError("pose must be a (4, 4) transform.")
        self.limits = limits
        self._position = T[:3, 3].copy()
        self._rotation = T[:3, :3].copy()
        self._v = (
            np.zeros(3)
            if linear_velocity is None
            else np.asarray(linear_velocity, dtype=np.float64)
        )
        self._w = (
            np.zeros(3)
            if angular_velocity is None
            else np.asarray(angular_velocity, dtype=np.float64)
        )
        self._goal: NDArray[np.float64] | None = None
        self.arrival_tolerance_m = arrival_tolerance_m
        self.arrival_tolerance_rad = arrival_tolerance_rad

    @property
    def pose(self) -> NDArray[np.float64]:
        """The current reference pose."""
        T = np.eye(4)
        T[:3, :3] = self._rotation
        T[:3, 3] = self._position
        return T

    @property
    def goal(self) -> NDArray[np.float64] | None:
        """The goal pose, if one is set."""
        return None if self._goal is None else self._goal.copy()

    @property
    def linear_velocity(self) -> NDArray[np.float64]:
        """Current translation velocity, m/s."""
        return self._v.copy()

    @property
    def angular_velocity(self) -> NDArray[np.float64]:
        """Current angular velocity, rad/s (world)."""
        return self._w.copy()

    def set_goal(self, pose: NDArray[np.float64]) -> None:
        """Re-target.  Position and velocity are kept: the path bends, it does not restart."""
        T = np.asarray(pose, dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError("goal must be a (4, 4) transform.")
        self._goal = T.copy()

    def remaining(self) -> Tuple[float, float]:
        """``(distance_m, angle_rad)`` from the reference to the goal."""
        if self._goal is None:
            return 0.0, 0.0
        distance = float(np.linalg.norm(self._goal[:3, 3] - self._position))
        angle = float(np.linalg.norm(rotation_log(self._goal[:3, :3] @ self._rotation.T)))
        return distance, angle

    @property
    def arrived(self) -> bool:
        """At the goal and at rest."""
        if self._goal is None:
            return True
        distance, angle = self.remaining()
        return (
            distance <= self.arrival_tolerance_m
            and angle <= self.arrival_tolerance_rad
            and float(np.linalg.norm(self._v)) < 1e-6
            and float(np.linalg.norm(self._w)) < 1e-6
        )

    def advance(self, dt: float, *, brake: bool = False) -> ReferenceSample:
        """Move the reference by one period towards the goal (or brake to rest)."""
        if dt <= 0.0:
            raise ValueError("dt must be positive.")
        limits = self.limits
        if self._goal is None:
            self._v = np.zeros(3)
            self._w = np.zeros(3)
            return ReferenceSample(self.pose, self._v.copy(), self._w.copy(), True, brake)
        p_error = self._goal[:3, 3] - self._position
        r_error = rotation_log(self._goal[:3, :3] @ self._rotation.T)
        self._v = _governor_step(
            p_error,
            self._v,
            limits.max_linear_velocity_m_s,
            limits.max_linear_acceleration_m_s2,
            dt,
            brake=brake,
        )
        self._w = _governor_step(
            r_error,
            self._w,
            limits.max_angular_velocity_rad_s,
            limits.max_angular_acceleration_rad_s2,
            dt,
            brake=brake,
        )
        step = self._v * dt
        if float(np.linalg.norm(step)) >= float(np.linalg.norm(p_error)) and not brake:
            # Do not step past the goal in the last sample: land on it, and
            # keep the velocity that landing realised so the next step brakes
            # from it within the acceleration ceiling rather than jumping to
            # rest.
            self._position = self._goal[:3, 3].copy()
            self._v = p_error / dt
        else:
            self._position = self._position + step
        turn = self._w * dt
        if float(np.linalg.norm(turn)) >= float(np.linalg.norm(r_error)) and not brake:
            self._rotation = self._goal[:3, :3].copy()
            self._w = r_error / dt
        else:
            self._rotation = rotation_exp(turn) @ self._rotation
        if self._goal is not None:
            # At rest on the goal (within the arrival tolerances) means
            # exactly at rest: a velocity below the acceleration step is
            # dropped rather than dithered about zero.
            distance, angle = self.remaining()
            if distance <= self.arrival_tolerance_m and (
                float(np.linalg.norm(self._v)) <= limits.max_linear_acceleration_m_s2 * dt
            ):
                self._v = np.zeros(3)
            if angle <= self.arrival_tolerance_rad and (
                float(np.linalg.norm(self._w)) <= limits.max_angular_acceleration_rad_s2 * dt
            ):
                self._w = np.zeros(3)
        return ReferenceSample(self.pose, self._v.copy(), self._w.copy(), self.arrived, brake)


# -- the timed joint trajectory -----------------------------------------------


@dataclass(frozen=True)
class TrajectorySample:
    """One sample of a joint trajectory.

    Attributes:
        time_from_start_s: Time of this sample.
        joints: ``{joint: rad}`` after the step.
        tcp: ``{side: (4, 4)}`` actual TCP poses at ``joints``.
        reference: ``{side: (4, 4)}`` reference poses the step tracked.
        goal_error_m: ``{side: m}`` distance from the actual TCP to the goal.
        goal_orientation_error_rad: ``{side: rad}`` angle to the goal.
        reference_error_m: ``{side: m}`` distance from the actual TCP to the reference.
        active_limits: The step's active constraints.
        status: The step's status.
        min_collision_distance_m: As the step reported it.
        braking: Whether the reference braked because the hand lagged.
        compute_time_s: The step's solve time.
    """

    time_from_start_s: float
    joints: Dict[str, float]
    tcp: Dict[str, NDArray[np.float64]]
    reference: Dict[str, NDArray[np.float64]]
    goal_error_m: Dict[str, float]
    goal_orientation_error_rad: Dict[str, float]
    reference_error_m: Dict[str, float]
    active_limits: Tuple[str, ...]
    status: DualArmIKStatus
    min_collision_distance_m: float | None
    braking: bool
    compute_time_s: float

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly view (poses as ``{"p", "q"}``)."""
        from robopy.viewer.model_bundle import matrix_to_pose  # noqa: PLC0415

        return {
            "t": self.time_from_start_s,
            "joints": dict(self.joints),
            "tcp": {side: matrix_to_pose(T) for side, T in self.tcp.items()},
            "reference": {side: matrix_to_pose(T) for side, T in self.reference.items()},
            "goal_error_m": dict(self.goal_error_m),
            "goal_orientation_error_rad": dict(self.goal_orientation_error_rad),
            "reference_error_m": dict(self.reference_error_m),
            "active_limits": list(self.active_limits),
            "status": self.status.value,
            "min_collision_distance_m": self.min_collision_distance_m,
            "braking": self.braking,
            "compute_time_s": self.compute_time_s,
        }


@dataclass
class JointTrajectory:
    """A timed joint trajectory and how it ended.

    Attributes:
        samples: The samples, oldest first; the first is the start (``t = 0``).
        status: Summary: ``CONVERGED`` (reference arrived, hands converged),
            ``TRACKING`` (the duration budget ran out while still making
            progress: continue from the last sample), one of the stall
            statuses, or the failure status of the step that ended the run.
        message: Why it ended, for people.
        last_result: The last solver result.
        limits: The reference ceilings used.
        dt_s: The sample period.
        truncated: The duration budget ended the run.
        references: The final reference governors, to resume from.
    """

    samples: List[TrajectorySample]
    status: DualArmIKStatus
    message: str
    last_result: DualArmIKResult | None
    limits: TrajectoryLimits
    dt_s: float
    truncated: bool = False
    references: Dict[str, PoseReference] = field(default_factory=dict)

    @property
    def duration_s(self) -> float:
        """Time of the last sample."""
        return self.samples[-1].time_from_start_s if self.samples else 0.0

    @property
    def final_joints(self) -> Dict[str, float]:
        """Joints of the last sample."""
        return dict(self.samples[-1].joints) if self.samples else {}

    def reference_at(self, t: float, side: str) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """``(pose, linear_velocity)`` of the reference at time ``t`` (nearest earlier sample).

        The velocity is estimated from the neighbouring samples; it is what a
        re-target during playback resumes from.
        """
        if not self.samples:
            raise ValueError("empty trajectory")
        index = 0
        for i, sample in enumerate(self.samples):
            if sample.time_from_start_s <= t:
                index = i
            else:
                break
        pose = self.samples[index].reference.get(side)
        if pose is None:
            raise KeyError(f"no reference for {side}")
        velocity = np.zeros(3)
        if index + 1 < len(self.samples) and side in self.samples[index + 1].reference:
            later = self.samples[index + 1]
            dt = later.time_from_start_s - self.samples[index].time_from_start_s
            if dt > 0.0:
                velocity = (later.reference[side][:3, 3] - pose[:3, 3]) / dt
        elif index > 0 and side in self.samples[index - 1].reference:
            # At the last sample the reference is still moving when the run was
            # truncated: the velocity it arrived with is the one a
            # continuation resumes from, not zero.
            earlier = self.samples[index - 1]
            dt = self.samples[index].time_from_start_s - earlier.time_from_start_s
            if dt > 0.0:
                velocity = (pose[:3, 3] - earlier.reference[side][:3, 3]) / dt
        return pose.copy(), velocity

    def describe(self, *, include_samples: bool = True) -> Dict[str, Any]:
        """JSON-friendly view."""
        return {
            "status": self.status.value,
            "message": self.message,
            "duration_s": self.duration_s,
            "dt_s": self.dt_s,
            "truncated": self.truncated,
            "n_samples": len(self.samples),
            "limits": self.limits.describe(),
            "samples": [s.describe() for s in self.samples] if include_samples else [],
        }


def _joint_state(names: Sequence[str], positions: Mapping[str, float]) -> JointState:
    from robopy.control.types import monotonic_ns  # noqa: PLC0415

    now = monotonic_ns()
    n = len(names)
    return JointState(
        joint_names=tuple(names),
        position_rad=np.asarray([positions[name] for name in names], dtype=float),
        velocity_rad_s=np.zeros(n),
        current_a=np.zeros(n),
        valid=np.ones(n, dtype=bool),
        read_start_ns=now,
        read_end_ns=now,
        sequence=0,
        mode_generation=0,
    )


def run_trajectory(
    solver: DualArmIK,
    start: Mapping[str, float],
    goals: Mapping[str, NDArray[np.float64]],
    limits: TrajectoryLimits,
    *,
    dt: float,
    max_duration_s: float,
    frames: Mapping[str, str],
    torso_policy: TorsoPolicy = TorsoPolicy.FIXED,
    torso_velocity_rad_s: float = 0.0,
    inactive_arm_policy: InactiveArmPolicy = InactiveArmPolicy.HOLD_JOINTS,
    references: Mapping[str, PoseReference] | None = None,
    settle_samples: int = 40,
    progress_tolerance_m: float = 1e-5,
    step_hook: Callable[[TrajectorySample], None] | None = None,
) -> JointTrajectory:
    """Walk the reference to the goals and follow it with the solver.

    Args:
        solver: The dual-arm solver.  Its velocity history is kept across
            calls; reset it only on the events the plan names.
        start: ``{joint: rad}`` for every movable joint.
        goals: ``{"left"|"right": (4, 4)}`` for the driven hands.
        limits: The reference ceilings.
        dt: Sample period; also the solver step.
        max_duration_s: Duration budget; a run still progressing at the end
            is returned as ``TRACKING`` with ``truncated`` set.
        frames: ``{side: tcp_frame}``.
        torso_policy: Torso policy of every step.
        torso_velocity_rad_s: For the manual policy.
        inactive_arm_policy: For the undriven hand.
        references: Governors to resume from (a re-target during playback);
            a side without one starts at rest at its current TCP.
        settle_samples: How many samples without progress count as a stall,
            once the reference has arrived or is braking for a lagging hand.
        progress_tolerance_m: Improvement of the summed goal error below which
            a sample counts as "no progress" (the default is half a millimetre
            per second at the 20 ms period: a hand creeping slower than that
            against its reach is not making progress).
        step_hook: Called with each sample as it is produced.
    """
    if dt <= 0.0 or max_duration_s <= 0.0:
        raise ValueError("dt and max_duration_s must be positive.")
    if not goals:
        raise ValueError("At least one hand needs a goal.")
    model = solver.model
    names = tuple(model.movable_joint_names)
    current = {name: float(start[name]) for name in names}

    q0 = model.q_from_positions(current)
    poses0 = model.frame_poses(q0, list(frames.values()))
    governors: Dict[str, PoseReference] = {}
    for side, goal in goals.items():
        resumed = None if references is None else references.get(side)
        if resumed is None:
            governor = PoseReference(poses0[frames[side]], limits)
        else:
            governor = resumed
        governor.set_goal(goal)
        governors[side] = governor

    def errors_to(
        poses: Mapping[str, NDArray[np.float64]],
    ) -> Tuple[Dict[str, float], Dict[str, float]]:
        distance: Dict[str, float] = {}
        angle: Dict[str, float] = {}
        for side, goal in goals.items():
            T = poses[frames[side]]
            distance[side] = float(np.linalg.norm(goal[:3, 3] - T[:3, 3]))
            angle[side] = float(np.linalg.norm(rotation_log(goal[:3, :3] @ T[:3, :3].T)))
        return distance, angle

    goal_error, goal_angle = errors_to(poses0)
    samples: List[TrajectorySample] = [
        TrajectorySample(
            time_from_start_s=0.0,
            joints=dict(current),
            tcp={side: poses0[frames[side]].copy() for side in frames},
            reference={side: governors[side].pose for side in goals},
            goal_error_m=goal_error,
            goal_orientation_error_rad=goal_angle,
            reference_error_m={
                side: float(
                    np.linalg.norm(governors[side].pose[:3, 3] - poses0[frames[side]][:3, 3])
                )
                for side in goals
            },
            active_limits=(),
            status=DualArmIKStatus.TRACKING,
            min_collision_distance_m=None,
            braking=False,
            compute_time_s=0.0,
        )
    ]
    if step_hook is not None:
        step_hook(samples[0])

    last_result: DualArmIKResult | None = None
    best_error = sum(goal_error.values())
    stalled_for = 0
    status = DualArmIKStatus.TRACKING
    message = ""
    truncated = False
    n_steps = int(math.ceil(max_duration_s / dt))
    t = 0.0
    for _ in range(n_steps):
        t += dt
        # Brake the reference where the hand lags it.
        lagging = False
        if limits.lag_tolerance_m is not None or limits.lag_tolerance_rad is not None:
            q_now = model.q_from_positions(current)
            now_poses = model.frame_poses(q_now, list(frames.values()))
            for side, governor in governors.items():
                T = now_poses[frames[side]]
                ref = governor.pose
                if limits.lag_tolerance_m is not None:
                    if float(np.linalg.norm(ref[:3, 3] - T[:3, 3])) > limits.lag_tolerance_m:
                        lagging = True
                if limits.lag_tolerance_rad is not None:
                    turn = float(np.linalg.norm(rotation_log(ref[:3, :3] @ T[:3, :3].T)))
                    if turn > limits.lag_tolerance_rad:
                        lagging = True
        references_now: Dict[str, NDArray[np.float64]] = {}
        for side, governor in governors.items():
            references_now[side] = governor.advance(dt, brake=lagging).pose
        target = DualArmTarget(
            left_target=references_now.get("left"),
            right_target=references_now.get("right"),
            left_enabled="left" in references_now,
            right_enabled="right" in references_now,
            torso_policy=torso_policy,
            torso_velocity_rad_s=torso_velocity_rad_s
            if torso_policy is TorsoPolicy.MANUAL
            else 0.0,
            inactive_arm_policy=inactive_arm_policy,
        )
        result = solver.solve_step(_joint_state(names, current), target, dt)
        last_result = result
        if not result.is_commandable:
            status = result.status
            message = result.message
            break
        current.update(result.joint_targets_rad)
        q = model.q_from_positions(current)
        poses = model.frame_poses(q, list(frames.values()))
        goal_error, goal_angle = errors_to(poses)
        sample = TrajectorySample(
            time_from_start_s=t,
            joints=dict(current),
            tcp={side: poses[frames[side]].copy() for side in frames},
            reference=references_now,
            goal_error_m=goal_error,
            goal_orientation_error_rad=goal_angle,
            reference_error_m={
                side: float(
                    np.linalg.norm(references_now[side][:3, 3] - poses[frames[side]][:3, 3])
                )
                for side in goals
            },
            active_limits=result.active_limits,
            status=result.status,
            min_collision_distance_m=result.min_collision_distance_m,
            braking=lagging,
            compute_time_s=result.compute_time_s,
        )
        samples.append(sample)
        if step_hook is not None:
            step_hook(sample)

        arrived = all(g.arrived for g in governors.values())
        if arrived and result.status is DualArmIKStatus.CONVERGED:
            status = DualArmIKStatus.CONVERGED
            message = "the reference arrived and the hands are within tolerance"
            break
        total = sum(goal_error.values())
        if arrived or lagging:
            # The reference is at the goal, or it is waiting for a hand that
            # lags it: either way the hands should now be closing the gap.
            # A gap that stops closing for a settle window is a stall, and
            # the binding constraint names its kind.  (While the reference is
            # still travelling with the hands on it, a pause in the goal error
            # is not evidence of anything.)
            if total < best_error - progress_tolerance_m:
                best_error = total
                stalled_for = 0
            else:
                stalled_for += 1
            if stalled_for >= settle_samples:
                status, message = _classify_stall(result, solver)
                break
        else:
            best_error = min(best_error, total)
            stalled_for = 0
    else:
        truncated = True
        status = DualArmIKStatus.TRACKING
        message = (
            f"the duration budget of {max_duration_s:g} s ran out while still moving; "
            "continue from the last sample"
        )
    return JointTrajectory(
        samples=samples,
        status=status,
        message=message,
        last_result=last_result,
        limits=limits,
        dt_s=dt,
        truncated=truncated,
        references=governors,
    )


def _classify_stall(result: DualArmIKResult, solver: DualArmIK) -> Tuple[DualArmIKStatus, str]:
    """Say *why* a run stopped progressing, from what was binding."""
    limits = [
        name
        for name in result.active_limits
        if name.endswith((":lower", ":upper", ":at_lower", ":at_upper"))
    ]
    if limits:
        return (
            DualArmIKStatus.LIMITS_BLOCKED,
            "the residual stopped improving with a position limit active: " + ", ".join(limits),
        )
    safety = solver.config.collision_safety_distance_m
    if result.min_collision_distance_m is not None and result.min_collision_distance_m <= (
        safety + 1e-3
    ):
        return (
            DualArmIKStatus.COLLISION_BLOCKED,
            f"the residual stopped improving with a collision pair at "
            f"{result.min_collision_distance_m * 1e3:.1f} mm (safety {safety * 1e3:.0f} mm)",
        )
    sigma = result.min_singular_value
    hint = ""
    if sigma is not None and solver.config.singularity_sigma_min > 0.0:
        if sigma < solver.config.singularity_sigma_min:
            hint = f"; the arm is near a singular configuration (sigma_min {sigma:.3g})"
    return (
        DualArmIKStatus.LOCALLY_STALLED,
        "the residual stopped improving with no constraint active: the local method cannot "
        "progress from this configuration (which does not prove the goal is unreachable)" + hint,
    )
