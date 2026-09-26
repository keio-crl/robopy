"""Differential inverse kinematics for two arms sharing one torso yaw joint.

Both hands are solved in a *single* quadratic program over

.. math::

    v = [v_\\mathrm{torso},\\ v_\\mathrm{left}(6),\\ v_\\mathrm{right}(6)]

so the shared torso joint is one decision variable that both hand tasks see.
Solving each arm on its own and averaging the two torso answers afterwards is
not equivalent and is not done here.

Conventions
-----------
Task errors and task Jacobians come from Pink, whose frame-task contribution to
the objective is :math:`\\tfrac{1}{2}\\|J \\Delta q + \\alpha e\\|_W^2` with
:math:`\\Delta q = v\\,\\Delta t`.  The sign and ordering below follow Pink, not
the sign convention of the design note; the two differ only in how the residual
is written.  Orientation error is the SE(3) logarithm, never a difference of
Euler angles.

The head is part of the model -- it moves the hands' collision geometry -- but
never a decision variable: its columns are excluded from the QP, so the solver
cannot command head motion.

What is asked of the hands
--------------------------
:attr:`DualArmIKConfig.orientation_mode` says which part of a hand target is a
task: ``position_only`` (three rows per hand), ``pose`` (six) or
``axis_aligned`` (three for position plus the gripper's approach axis on the
sphere, rotation about that axis free -- see
:mod:`robopy.kinematics.axis_alignment_task`).  Convergence is judged on the
rows the mode asks for, evaluated at the configuration the step *arrives* at:
the position error is the world distance of the TCP from its target, the
orientation error the angle of the SO(3) difference.

Two priorities
--------------
Reaching the hand targets comes first; a comfortable posture, staying away
from the joint limits, moving the torso less than the arms and not changing
velocity abruptly come second.  With
:attr:`DualArmIKConfig.task_priority_mode` ``"weighted"`` (the historical
behaviour) everything is one weighted sum and a heavy posture weight *does*
trade against hand accuracy.  With ``"hierarchical"`` the step is solved in
two stages over the same free variables and the same constraints: first the
hand tasks alone,

.. math::

    x_1 = \\arg\\min_{x \\in C} \\|A x + b\\|^2,

then the secondary objectives subject to keeping the first stage's task
output,

.. math::

    \\min_{x \\in C} C_\\mathrm{posture} + C_\\mathrm{limit} + C_\\mathrm{smooth}
    + C_\\mathrm{motion} \\quad \\text{s.t.} \\quad A x = A x_1 ,

so nothing the second stage prefers can move the hands away from where the
first stage put them.  The equality rows are reduced to a numerically
independent set first (the hand Jacobians are redundant whenever the arms
are), and a second stage that fails numerically falls back to the verified
first-stage step, which is reported.  Both stages carry a small Tikhonov term
(:attr:`DualArmIKConfig.damping`) so the QP is strictly convex for the
backend; its effect on the first stage is bounded by the reported task
residuals rather than assumed away.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.control.types import DualArmTarget, InactiveArmPolicy, JointState, TorsoPolicy

from .axis_alignment_task import AxisAlignmentTask
from .joint_limits import normalised_singular_values
from .urdf_model import WholeBodyModel, require_pinocchio

if TYPE_CHECKING:  # pragma: no cover - typing only
    pass

__all__ = [
    "ORIENTATION_MODES",
    "PRIORITY_MODES",
    "DualArmIK",
    "DualArmIKConfig",
    "DualArmIKResult",
    "DualArmIKStatus",
]

ORIENTATION_MODES: Tuple[str, ...] = ("position_only", "pose", "axis_aligned")
PRIORITY_MODES: Tuple[str, ...] = ("weighted", "hierarchical")


class DualArmIKStatus(Enum):
    """Outcome of one :meth:`DualArmIK.solve_step` call, or of a run of them.

    Attributes:
        CONVERGED: Every enabled hand task is inside tolerance after the step;
            the step is a small correction only.
        TRACKING: A valid step was produced but the targets are not yet reached.
            This is the normal status while following a moving target.
        INFEASIBLE: The constraints admit no step (typically the configuration
            already violates a limit, or a collision constraint cannot be
            satisfied).  No motion command is produced.
        STALE_STATE: The measured state or the target was older than its
            deadline.  No motion command is produced.
        SOLVER_ERROR: The QP solver failed or returned a non-finite solution.
        COLLISION_AT_START: The starting configuration is already in collision,
            so a local distance constraint cannot be trusted.
        DEADLINE_EXCEEDED: The solve itself overran its compute budget.
        LOCALLY_STALLED: Produced by a *sequence* of steps (see
            :mod:`robopy.kinematics.cartesian_trajectory`): the residual has
            stopped improving without any constraint being active.  The local
            method cannot progress from here; that is not a proof the target
            is unreachable.
        LIMITS_BLOCKED: A sequence stalled with a position limit active: the
            target is beyond what the joint ranges allow from here.
        COLLISION_BLOCKED: A sequence stalled with a collision constraint
            active.
    """

    CONVERGED = "converged"
    TRACKING = "tracking"
    INFEASIBLE = "infeasible"
    STALE_STATE = "stale_state"
    SOLVER_ERROR = "solver_error"
    COLLISION_AT_START = "collision_at_start"
    DEADLINE_EXCEEDED = "deadline_exceeded"
    LOCALLY_STALLED = "locally_stalled"
    LIMITS_BLOCKED = "limits_blocked"
    COLLISION_BLOCKED = "collision_blocked"

    @property
    def is_commandable(self) -> bool:
        """Whether a result with this status may be issued as new motion."""
        return self in (DualArmIKStatus.CONVERGED, DualArmIKStatus.TRACKING)

    @property
    def is_stall(self) -> bool:
        """Whether this is one of the "cannot progress" summaries of a run."""
        return self in (
            DualArmIKStatus.LOCALLY_STALLED,
            DualArmIKStatus.LIMITS_BLOCKED,
            DualArmIKStatus.COLLISION_BLOCKED,
        )


@dataclass
class DualArmIKConfig:
    """Weights, limits and policies for the dual-arm solver.

    Attributes:
        position_cost: Weight on each hand's position error.
        orientation_cost: Weight on each hand's orientation error (``pose``
            mode) or approach-axis error (``axis_aligned`` mode).
        left_priority: Extra multiplier on the left hand's costs.
        right_priority: Extra multiplier on the right hand's costs.
        hold_position_cost: Position weight of a non-driven hand's hold task.
        hold_orientation_cost: Orientation weight of a hold task.  A hold with a
            finite weight is a *preference*, not a lock; the residual is
            reported rather than described as exact.
        posture_cost: Weight pulling the arms towards ``posture_reference``,
            a scalar or ``{joint: weight}`` over the active joints.
        posture_reference: ``{joint: radians}`` the posture objective pulls
            towards -- the *preferred* posture.  ``None`` uses the
            configuration the solver was last reset to.
        joint_motion_cost: Quadratic cost on each joint's step, a scalar or
            ``{joint: weight}``.  The usual profile gives the torso more than
            the arms, so a hand is reached by the arm when the arm can.
        velocity_smoothing_cost: Cost on the change of each joint's velocity
            since the previous cycle (``v - v_prev``), scalar or per joint.
        limit_avoidance_enabled: Guide a joint that has entered the band next
            to a limit back inside.  Joints elsewhere feel nothing: this is
            not a pull towards the middle of every range.
        limit_avoidance_band_rad: Width of that band.
        limit_avoidance_cost: Its weight at the limit itself (it ramps up
            quadratically across the band).
        task_priority_mode: ``"weighted"`` or ``"hierarchical"``; see the
            module docstring.
        orientation_mode: ``"position_only"``, ``"pose"`` or ``"axis_aligned"``.
        approach_axis_tcp: The gripper's approach axis in TCP coordinates,
            required by ``axis_aligned``.  Not assumed to be any particular
            axis: state it.
        torso_regularisation: Extra quadratic penalty on torso motion, applied
            only under :attr:`TorsoPolicy.OPTIMIZE`.
        damping: Tikhonov regularisation on the whole step.  Keeps the QP
            strictly convex and bounds the step near singularities.
        singularity_sigma_min: Threshold on the smallest *normalised* singular
            value of an enabled hand's Jacobian (linear rows over
            ``position_tolerance_m``, angular rows over
            ``orientation_tolerance_rad``; see
            :func:`~robopy.kinematics.joint_limits.normalised_singular_values`)
            under which extra damping fades in.
        singularity_damping: The extra damping at a fully singular Jacobian;
            it ramps continuously, there is no switch.
        lm_damping: Levenberg-Marquardt damping passed to the frame tasks.
        gain: Task gain ``alpha`` in Pink's residual, used when
            ``gain_time_constant_s`` is ``None``.
        gain_time_constant_s: When set, the gain becomes
            ``1 - exp(-dt / tau)`` each step, so the error is corrected with
            this time constant whatever the control period.
        max_joint_velocity_rad_s: Per-joint velocity ceiling.  ``None`` falls
            back to the URDF's velocity limit, which for a CAD export is often a
            placeholder and should be replaced.
        max_joint_acceleration_rad_s2: Per-joint bound on ``|v - v_prev| / dt``,
            with ``v_prev`` the previous step divided by *its* period.
        max_joint_step_rad: Hard ceiling on how far a joint target may move in
            one cycle, independent of the velocity limit.
        position_limit_margin_rad: Stay this far inside every position limit.
        position_tolerance_m: Position error under which a hand counts as
            converged.
        orientation_tolerance_rad: Orientation (or approach-axis) error under
            which a hand counts as converged.
        collision_safety_distance_m: Distance the solver tries to keep.
        collision_activation_distance_m: Distance under which a pair gets a
            constraint at all.
        collision_gain: ``eta`` in ``grad(d)^T v >= -eta (d - d_safe)``.
        collision_substeps: Number of interpolated configurations between the
            current and candidate configuration at which the candidate step is
            re-checked.  A local linearisation is not trusted over a big step.
        compute_budget_s: Soft budget for one solve.  Exceeding it produces
            :attr:`DualArmIKStatus.DEADLINE_EXCEEDED` rather than a late command.
        max_state_age_s: Reject a measured state older than this.
        require_soft_limits: Refuse to build a solver whose active continuous
            joints have no explicit soft limit.  Continuous joints carry no URDF
            range; leaving them unbounded lets the solver take a shortest-angle
            path through a region the cabling forbids.
        solver: QP backend name understood by ``qpsolvers``.
        hierarchical_rank_tolerance: Relative singular-value threshold below
            which a row of the first stage's task output counts as redundant
            and is dropped from the second stage's equality constraints.
        secondary_regularisation: Tikhonov term of the second stage (and of
            the weighted mode's secondary part): a minimum-motion preference
            on every free joint, so a joint no secondary objective weighs
            still has a unique, well-conditioned answer -- the backend's
            Cholesky factorisation cannot take a Hessian that spans nine
            orders of magnitude.
    """

    position_cost: float = 1.0
    orientation_cost: float = 0.15
    left_priority: float = 1.0
    right_priority: float = 1.0
    hold_position_cost: float = 1.0
    hold_orientation_cost: float = 0.15
    posture_cost: Mapping[str, float] | float = 1e-3
    posture_reference: Mapping[str, float] | None = None
    joint_motion_cost: Mapping[str, float] | float | None = None
    velocity_smoothing_cost: Mapping[str, float] | float | None = None
    limit_avoidance_enabled: bool = False
    limit_avoidance_band_rad: float = 0.2
    limit_avoidance_cost: float = 1.0
    task_priority_mode: str = "weighted"
    orientation_mode: str = "pose"
    approach_axis_tcp: Tuple[float, float, float] | None = None
    torso_regularisation: float = 1e-2
    damping: float = 1e-6
    singularity_sigma_min: float = 0.0
    singularity_damping: float = 0.0
    lm_damping: float = 1e-6
    gain: float = 1.0
    gain_time_constant_s: float | None = None
    max_joint_velocity_rad_s: Mapping[str, float] | float | None = 1.0
    max_joint_acceleration_rad_s2: Mapping[str, float] | float | None = 8.0
    max_joint_step_rad: float = 0.05
    position_limit_margin_rad: float = 0.01
    position_tolerance_m: float = 1e-3
    orientation_tolerance_rad: float = 1e-2
    collision_safety_distance_m: float = 0.02
    collision_activation_distance_m: float = 0.08
    collision_gain: float = 1.0
    collision_substeps: int = 3
    compute_budget_s: float = 0.05
    max_state_age_s: float = 0.2
    require_soft_limits: bool = True
    solver: str = "quadprog"
    hierarchical_rank_tolerance: float = 1e-8
    secondary_regularisation: float = 1e-3

    def __post_init__(self) -> None:
        if self.task_priority_mode not in PRIORITY_MODES:
            raise ValueError(f"task_priority_mode must be one of {PRIORITY_MODES}.")
        if self.orientation_mode not in ORIENTATION_MODES:
            raise ValueError(f"orientation_mode must be one of {ORIENTATION_MODES}.")
        if self.orientation_mode == "axis_aligned" and self.approach_axis_tcp is None:
            raise ValueError(
                "orientation_mode='axis_aligned' needs approach_axis_tcp: the gripper's approach "
                "axis in TCP coordinates is stated, not assumed to be Z."
            )
        if self.approach_axis_tcp is not None:
            axis = np.asarray(self.approach_axis_tcp, dtype=np.float64)
            if axis.shape != (3,) or float(np.linalg.norm(axis)) < 1e-9:
                raise ValueError("approach_axis_tcp must be a non-zero 3-vector.")
        if self.gain_time_constant_s is not None and self.gain_time_constant_s <= 0.0:
            raise ValueError("gain_time_constant_s must be positive.")
        if self.limit_avoidance_band_rad <= 0.0 or self.limit_avoidance_cost < 0.0:
            raise ValueError("limit_avoidance_band_rad must be positive and its cost >= 0.")
        if self.damping < 0.0 or self.singularity_damping < 0.0:
            raise ValueError("damping terms must be non-negative.")
        if self.secondary_regularisation <= 0.0:
            raise ValueError("secondary_regularisation must be positive.")
        if self.singularity_sigma_min < 0.0:
            raise ValueError("singularity_sigma_min must be non-negative.")


@dataclass(frozen=True)
class DualArmIKResult:
    """Everything one solver step produced, including why it produced nothing.

    Errors are evaluated at the configuration the step *arrives* at (the
    commanded joint targets): the position error is the world distance of the
    TCP from its target, the orientation error the angle of the SO(3)
    difference, the axis error the angle between the approach axis and its
    target direction.  A hand whose mode does not ask for orientation still
    reports it, for display; it just does not decide convergence.

    Attributes:
        status: See :class:`DualArmIKStatus`.
        joint_targets_rad: ``{joint: radians}`` for the active joints.  Empty
            when :attr:`status` is not commandable.
        joint_velocities_rad_s: ``{joint: rad/s}`` implied by the step.
        left_position_error_m: Left hand's position error after the step, or
            ``None`` when the left task was not evaluated.
        left_orientation_error_rad: Left hand's orientation error after the step.
        left_axis_error_rad: Left approach-axis error (``axis_aligned`` only).
        right_position_error_m: Same for the right hand.
        right_orientation_error_rad: Same for the right hand.
        right_axis_error_rad: Same for the right hand.
        left_hold_residual_m: Position residual of the left hold task when the
            left hand is not driven, *as measured* before the step: how far the
            held hand had drifted from its latched target.  A hold is a
            weighted objective, so this is reported instead of claiming the
            hand is pinned.
        right_hold_residual_m: Same for the right hand.
        active_limits: Names of the constraints that were active (clipped).
        min_collision_distance_m: Smallest checked pair distance at the
            candidate configuration, or ``None`` when collision is not modelled.
        min_singular_value: Smallest normalised singular value over the
            enabled hands' Jacobians (see
            :attr:`DualArmIKConfig.singularity_sigma_min`), or ``None`` when no
            hand was driven.
        limit_margin_rad: ``{joint: distance to its nearest limit}`` at the
            arrival configuration, for the active joints with finite limits.
        task_residual_before: Weighted first-priority residual ``||A x + b||``
            evaluated for a zero step -- the task error as the step saw it.
        task_residual_after: The same residual for the step taken.  In the
            hierarchical mode the second stage may not raise it above the
            first stage's; that is checked, not assumed.
        stage1_residual: First-stage residual in the hierarchical mode.
        hierarchical_fallback: The second stage failed numerically and the
            verified first-stage step was issued instead.
        compute_time_s: Wall-clock duration of the solve.
        torso_velocity_rad_s: Commanded torso velocity.
        dt_s: The period the step was computed for.
        orientation_mode: The mode the hands were solved in.
        message: Human-readable diagnosis, especially for failures.
        generation: The state's mode generation, carried through so a consumer
            can reject a command produced from a superseded generation.
    """

    status: DualArmIKStatus
    joint_targets_rad: Dict[str, float] = field(default_factory=dict)
    joint_velocities_rad_s: Dict[str, float] = field(default_factory=dict)
    left_position_error_m: float | None = None
    left_orientation_error_rad: float | None = None
    left_axis_error_rad: float | None = None
    right_position_error_m: float | None = None
    right_orientation_error_rad: float | None = None
    right_axis_error_rad: float | None = None
    left_hold_residual_m: float | None = None
    right_hold_residual_m: float | None = None
    active_limits: Tuple[str, ...] = ()
    min_collision_distance_m: float | None = None
    min_singular_value: float | None = None
    limit_margin_rad: Dict[str, float] = field(default_factory=dict)
    task_residual_before: float | None = None
    task_residual_after: float | None = None
    stage1_residual: float | None = None
    hierarchical_fallback: bool = False
    compute_time_s: float = 0.0
    torso_velocity_rad_s: float = 0.0
    dt_s: float = 0.0
    orientation_mode: str = "pose"
    message: str = ""
    generation: int = 0

    @property
    def is_commandable(self) -> bool:
        """Whether this result may be issued as a new motion command."""
        return self.status.is_commandable and bool(self.joint_targets_rad)

    def errors(self) -> Dict[str, float | None]:
        """The per-hand errors as one mapping (the viewer and VR pages use this)."""
        return {
            "left_position_m": self.left_position_error_m,
            "left_orientation_rad": self.left_orientation_error_rad,
            "left_axis_rad": self.left_axis_error_rad,
            "right_position_m": self.right_position_error_m,
            "right_orientation_rad": self.right_orientation_error_rad,
            "right_axis_rad": self.right_axis_error_rad,
            "left_hold_m": self.left_hold_residual_m,
            "right_hold_m": self.right_hold_residual_m,
        }


def _as_per_joint(
    value: Mapping[str, float] | float | None,
    joints: Sequence[str],
    fallback: NDArray[np.float64] | None,
    *,
    name: str,
) -> NDArray[np.float64]:
    """Expand a scalar / mapping / ``None`` limit specification to a per-joint array."""
    if value is None:
        if fallback is None:
            raise ValueError(f"{name} is None and no fallback is available.")
        return np.asarray(fallback, dtype=np.float64)
    if isinstance(value, Mapping):
        missing = [j for j in joints if j not in value]
        if missing:
            raise ValueError(f"{name} is missing entries for {missing}.")
        return np.asarray([float(value[j]) for j in joints], dtype=np.float64)
    return np.full(len(joints), float(value), dtype=np.float64)


def _as_weights(
    value: Mapping[str, float] | float | None, joints: Sequence[str], *, name: str
) -> NDArray[np.float64]:
    """Expand a cost specification to a per-joint array; a joint left out weighs zero."""
    if value is None:
        return np.zeros(len(joints))
    if isinstance(value, Mapping):
        unknown = sorted(set(value) - set(joints))
        if unknown:
            raise ValueError(f"{name} names joint(s) {unknown} that the solver does not drive.")
        weights = np.asarray([float(value.get(j, 0.0)) for j in joints], dtype=np.float64)
    else:
        weights = np.full(len(joints), float(value), dtype=np.float64)
    if np.any(weights < 0.0):
        raise ValueError(f"{name} must be non-negative.")
    return weights


def _rotation_angle(R: NDArray[np.float64]) -> float:
    """Angle of a rotation matrix, radians in ``[0, pi]``."""
    cos = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    return float(math.acos(cos))


class DualArmIK:
    """Single-QP differential IK for two arms over a shared torso yaw.

    The solver is pure computation: it performs no serial I/O, reads no clock
    other than to measure its own duration, and holds no bus.
    """

    def __init__(
        self,
        model: WholeBodyModel,
        *,
        left_frame: str,
        right_frame: str,
        torso_joint: str,
        left_arm_joints: Sequence[str],
        right_arm_joints: Sequence[str],
        head_joints: Sequence[str] = (),
        config: DualArmIKConfig | None = None,
    ) -> None:
        """Build a solver bound to one model.

        Args:
            model: The whole-body model.
            left_frame: Frame name of the left TCP.
            right_frame: Frame name of the right TCP.
            torso_joint: The shared torso yaw joint.
            left_arm_joints: The six left-arm joints, shoulder to wrist.
            right_arm_joints: The six right-arm joints, shoulder to wrist.
            head_joints: Head joints.  Modelled, never commanded by this solver.
            config: Weights and limits.

        Raises:
            KeyError: If a frame or joint name is unknown.
            ValueError: If an active continuous joint has no soft limit while
                :attr:`DualArmIKConfig.require_soft_limits` is set, or if a
                joint appears in more than one group.
        """
        self._pink = _require_pink()
        self._qpsolvers = _require_qpsolvers()
        self._pin = require_pinocchio()

        self._model = model
        self._config = config or DualArmIKConfig()

        for frame in (left_frame, right_frame):
            if not model.has_frame(frame):
                raise KeyError(
                    f"TCP frame '{frame}' does not exist in {model.source}. Define it with "
                    "WholeBodyModel.add_fixed_frame() from a verified fixed transform."
                )
            # Pink resolves frames by name through Pinocchio's getFrameId, which
            # raises when a URDF gives a joint and its child link the same name.
            # The Rakuda export does that for gripper_left_dof, gripper_right_dof
            # and head_camera_link, so such a name cannot be used as a task frame.
            if len(model.frame_ids(frame)) != 1:
                raise ValueError(
                    f"TCP frame name '{frame}' is carried by more than one frame in "
                    f"{model.source}, so the solver cannot address it by name. Attach a "
                    "uniquely named operational frame with WholeBodyModel.add_fixed_frame() "
                    "and use that instead."
                )
        self._left_frame = left_frame
        self._right_frame = right_frame

        self._torso_joint = torso_joint
        self._left_joints = tuple(left_arm_joints)
        self._right_joints = tuple(right_arm_joints)
        self._head_joints = tuple(head_joints)

        groups = {
            "torso": (torso_joint,),
            "left": self._left_joints,
            "right": self._right_joints,
            "head": self._head_joints,
        }
        seen: Dict[str, str] = {}
        for group, names in groups.items():
            for name in names:
                if not model.has_joint(name):
                    raise KeyError(f"'{name}' is not a movable joint of {model.source}.")
                if name in seen:
                    raise ValueError(
                        f"Joint '{name}' appears in both the '{seen[name]}' and '{group}' groups."
                    )
                seen[name] = group

        #: Active joints, in QP variable order: torso first, then left, then right.
        self._active_joints: Tuple[str, ...] = (
            (torso_joint,) + self._left_joints + self._right_joints
        )
        self._active_v = model.v_indices(self._active_joints)
        self._torso_slot = 0

        if self._config.require_soft_limits:
            unbounded = model.unbounded_joints(self._active_joints)
            if unbounded:
                raise ValueError(
                    f"Active joint(s) {unbounded} have no finite position limit. Continuous URDF "
                    "joints carry no range, so a measured soft limit must be supplied via "
                    "WholeBodyModel.set_soft_limits() before they can be driven. Set "
                    "DualArmIKConfig.require_soft_limits=False only for a deliberately "
                    "unconstrained geometric study."
                )

        cfg = self._config
        # Per-joint secondary weights, validated once.
        self._posture_weights = _as_weights(
            cfg.posture_cost, self._active_joints, name="posture_cost"
        )
        self._motion_weights = _as_weights(
            cfg.joint_motion_cost, self._active_joints, name="joint_motion_cost"
        )
        self._smoothing_weights = _as_weights(
            cfg.velocity_smoothing_cost, self._active_joints, name="velocity_smoothing_cost"
        )

        self._orientation_mode = cfg.orientation_mode
        self._orientation_cost = cfg.orientation_cost
        self._left_task = self._pink.tasks.FrameTask(
            left_frame,
            position_cost=cfg.position_cost * cfg.left_priority,
            orientation_cost=self._frame_orientation_cost(cfg.left_priority),
            lm_damping=cfg.lm_damping,
            gain=cfg.gain,
        )
        self._right_task = self._pink.tasks.FrameTask(
            right_frame,
            position_cost=cfg.position_cost * cfg.right_priority,
            orientation_cost=self._frame_orientation_cost(cfg.right_priority),
            lm_damping=cfg.lm_damping,
            gain=cfg.gain,
        )
        self._axis_tasks: Dict[str, AxisAlignmentTask] = {}
        if cfg.approach_axis_tcp is not None:
            for side, frame, priority in (
                ("left", left_frame, cfg.left_priority),
                ("right", right_frame, cfg.right_priority),
            ):
                self._axis_tasks[side] = AxisAlignmentTask(
                    model,
                    frame,
                    cfg.approach_axis_tcp,
                    cost=cfg.orientation_cost * priority,
                    gain=cfg.gain,
                    lm_damping=cfg.lm_damping,
                )

        self._hold_targets: Dict[str, Any] = {"left": None, "right": None}
        self._hold_tasks = {
            side: self._pink.tasks.FrameTask(
                frame,
                position_cost=cfg.hold_position_cost,
                orientation_cost=cfg.hold_orientation_cost,
                lm_damping=cfg.lm_damping,
                gain=cfg.gain,
            )
            for side, frame in (("left", left_frame), ("right", right_frame))
        }
        self._previous_step: NDArray[np.float64] | None = None
        self._previous_dt: float | None = None
        self._posture_q: NDArray[np.float64] | None = None
        self._has_collision = model.collision_model is not None
        self._stage2_note = ""

    # -- properties ---------------------------------------------------------

    @property
    def active_joints(self) -> Tuple[str, ...]:
        """Active joints in QP variable order (torso, left arm, right arm)."""
        return self._active_joints

    @property
    def config(self) -> DualArmIKConfig:
        """The solver configuration."""
        return self._config

    @property
    def model(self) -> WholeBodyModel:
        """The whole-body model this solver is bound to."""
        return self._model

    @property
    def orientation_mode(self) -> str:
        """Which part of a hand target is a task (see :data:`ORIENTATION_MODES`)."""
        return self._orientation_mode

    @property
    def orientation_cost(self) -> float:
        """The orientation (or axis) weight currently applied to the hand tasks."""
        return self._orientation_cost

    # -- lifecycle ----------------------------------------------------------

    def reset(self, q: NDArray[np.float64] | None = None) -> None:
        """Clear latched state: hold targets, velocity history and posture reference.

        Call this at start-up, on an explicit mode change, after a manual
        posture change and when resuming after a stop.  Not every request: a
        continuous operation keeps its velocity history and its posture
        reference, or the acceleration bound and the posture objective would
        mean nothing.
        """
        self._hold_targets = {"left": None, "right": None}
        self._previous_step = None
        self._previous_dt = None
        if q is not None:
            self._posture_q = np.array(q, dtype=np.float64, copy=True)

    def set_posture_reference(self, positions_rad: Mapping[str, float]) -> None:
        """Set the configuration the posture objective pulls towards."""
        self._posture_q = self._model.q_from_positions(positions_rad, require_all=False)

    def set_orientation_mode(self, mode: str) -> None:
        """Change which part of a hand target is a task, without rebuilding.

        Args:
            mode: One of :data:`ORIENTATION_MODES`.  ``axis_aligned`` needs
                :attr:`DualArmIKConfig.approach_axis_tcp` to have been given.
        """
        if mode not in ORIENTATION_MODES:
            raise ValueError(f"orientation mode must be one of {ORIENTATION_MODES}.")
        if mode == "axis_aligned" and not self._axis_tasks:
            raise ValueError(
                "axis_aligned needs approach_axis_tcp in the solver configuration; the "
                "approach axis is stated, not assumed."
            )
        self._orientation_mode = mode
        self._apply_orientation_cost()

    def set_task_costs(
        self,
        *,
        position_cost: float | None = None,
        orientation_cost: float | None = None,
    ) -> None:
        """Re-weight the hand tasks without rebuilding the solver.

        Args:
            position_cost: New position weight, applied with each side's
                priority multiplier.  ``None`` leaves it unchanged.
            orientation_cost: New orientation (or axis) weight, likewise.  A
                weight of zero makes the hands position-only in effect; the
                cleaner way to say that is :meth:`set_orientation_mode`.
        """
        cfg = self._config
        if position_cost is not None:
            self._left_task.set_position_cost(position_cost * cfg.left_priority)
            self._right_task.set_position_cost(position_cost * cfg.right_priority)
        if orientation_cost is not None:
            if orientation_cost < 0.0:
                raise ValueError("orientation_cost must be non-negative.")
            self._orientation_cost = float(orientation_cost)
            self._apply_orientation_cost()

    def _frame_orientation_cost(self, priority: float) -> float:
        """The FrameTask's orientation weight for the current mode."""
        if self._orientation_mode == "pose":
            return self._orientation_cost * priority
        return 0.0

    def _apply_orientation_cost(self) -> None:
        cfg = self._config
        self._left_task.set_orientation_cost(self._frame_orientation_cost(cfg.left_priority))
        self._right_task.set_orientation_cost(self._frame_orientation_cost(cfg.right_priority))
        for side, task in self._axis_tasks.items():
            priority = cfg.left_priority if side == "left" else cfg.right_priority
            task.cost = self._orientation_cost * priority

    # -- solving ------------------------------------------------------------

    def solve_step(
        self,
        state: JointState,
        target: DualArmTarget,
        dt: float,
    ) -> DualArmIKResult:
        """Solve one differential IK step from a *measured* state.

        Every step starts from the measured configuration, so an unresolved
        tracking error between the internal target and the machine cannot be
        integrated away.  The previous solution is used only for the
        acceleration bound and the smoothing objective.

        Args:
            state: Measured joint state.  It must cover every joint of the
                model, including the head, whose pose the collision check needs.
            target: The left/right TCP targets and torso policy.
            dt: Control period in seconds.

        Returns:
            A :class:`DualArmIKResult`.  A non-commandable status always carries
            a reason in :attr:`DualArmIKResult.message`.
        """
        started = time.perf_counter()
        cfg = self._config
        if dt <= 0.0:
            return self._failure(
                DualArmIKStatus.SOLVER_ERROR, "dt must be positive.", started, state, dt
            )

        stale = self._check_freshness(state, target)
        if stale is not None:
            return self._failure(DualArmIKStatus.STALE_STATE, stale, started, state, dt)

        try:
            q = self._configuration_from_state(state)
        except (KeyError, ValueError) as exc:
            return self._failure(DualArmIKStatus.SOLVER_ERROR, str(exc), started, state, dt)

        if self._posture_q is None:
            self._posture_q = np.array(q, dtype=np.float64, copy=True)
        if cfg.posture_reference is not None:
            self._posture_q = self._model.q_from_positions(
                cfg.posture_reference, base=q, require_all=False
            )

        gain = cfg.gain
        if cfg.gain_time_constant_s is not None:
            gain = 1.0 - math.exp(-dt / cfg.gain_time_constant_s)
        self._set_gain(gain)

        configuration = self._pink.Configuration(self._model.model, self._model.data, q)
        mode = self._orientation_mode
        hand_tasks, axis_tasks = self._update_task_targets(configuration, target)

        # --- first-priority rows: A x + b over the full tangent space ---------
        rows_A: List[NDArray[np.float64]] = []
        rows_b: List[NDArray[np.float64]] = []
        lm = 0.0
        for task in hand_tasks:
            J = np.asarray(task.compute_jacobian(configuration), dtype=np.float64)
            e = np.asarray(task.compute_error(configuration), dtype=np.float64)
            W = np.asarray(task.cost, dtype=np.float64)
            rows_A.append(W[:, None] * J)
            weighted_error = task.gain * W * e
            rows_b.append(weighted_error)
            # Levenberg-Marquardt damping, as Pink adds it per task.
            lm += task.lm_damping * float(weighted_error @ weighted_error)
        for axis in axis_tasks:
            A_t, b_t = axis.weighted_rows(q)
            rows_A.append(A_t)
            rows_b.append(b_t)
            lm += axis.lm_damping * float(b_t @ b_t)
        nv = self._model.nv
        A_full = np.vstack(rows_A) if rows_A else np.zeros((0, nv))
        b_full = np.concatenate(rows_b) if rows_b else np.zeros(0)

        free_slots, known_step = self._variable_layout(target, dt)
        S = np.zeros((nv, len(free_slots)))
        for column, slot in enumerate(free_slots):
            S[self._active_v[slot], column] = 1.0
        # The known (non-optimised) part of the step, lifted to the full tangent
        # space so it can be folded into the objective's linear term.
        known_full = np.zeros(nv)
        known_full[self._active_v] = known_step

        A = A_full @ S  # task rows over the free variables
        b = b_full + A_full @ known_full
        n_free = len(free_slots)

        # --- normalised singular values, extra damping near singularities -----
        sigma_min = self._min_singular_value(q, target, free_slots)
        damping = cfg.damping
        if (
            sigma_min is not None
            and cfg.singularity_sigma_min > 0.0
            and cfg.singularity_damping > 0.0
        ):
            shortfall = max(0.0, 1.0 - sigma_min / cfg.singularity_sigma_min)
            damping += cfg.singularity_damping * shortfall * shortfall

        # --- bounds and constraints ------------------------------------------
        try:
            lb, ub, bound_names, bound_parts = self._step_bounds(q, free_slots, known_step, dt)
        except ValueError as exc:
            return self._failure(DualArmIKStatus.INFEASIBLE, str(exc), started, state, dt)

        if np.any(lb > ub):
            offenders = [bound_names[i] for i in np.flatnonzero(lb > ub)]
            return self._failure(
                DualArmIKStatus.INFEASIBLE,
                f"The configuration already violates the limits of {offenders}; no feasible step "
                "exists. Move the machine back inside its limits before commanding motion.",
                started,
                state,
                dt,
            )

        G: NDArray[np.float64] | None = None
        h: NDArray[np.float64] | None = None
        collision_before: float | None = None
        collision_detail = ""
        if self._has_collision:
            report = self._model.collision_report(q)
            collision_before = report.min_distance
            if collision_before < 0.0:
                return self._failure(
                    DualArmIKStatus.COLLISION_AT_START,
                    f"Starting configuration is already in self-collision "
                    f"(min distance {collision_before:.4f} m at pair {report.closest_pair()}). "
                    "A local distance constraint cannot be trusted from here.",
                    started,
                    state,
                    dt,
                    min_distance=collision_before,
                )
            G, h = self._collision_constraints(q, report, S, known_step, dt)
            collision_detail = self._collision_diagnosis(report)

        if time.perf_counter() - started > cfg.compute_budget_s:
            return self._failure(
                DualArmIKStatus.DEADLINE_EXCEEDED,
                "The solve exceeded its compute budget before reaching the QP.",
                started,
                state,
                dt,
                min_distance=collision_before,
            )

        # --- secondary objective over the free variables ---------------------
        P2, c2 = self._secondary_objective(q, free_slots, dt, gain)
        if target.torso_policy is TorsoPolicy.OPTIMIZE and self._torso_slot in free_slots:
            torso_column = free_slots.index(self._torso_slot)
            P2[torso_column, torso_column] += cfg.torso_regularisation

        residual_before = float(np.linalg.norm(b)) if b.size else 0.0
        stage1_residual: float | None = None
        fallback = False
        try:
            if n_free == 0:
                # Both arms can be inactive with a fixed/manual torso. There
                # is no optimisation variable, but known motion must still
                # satisfy collision constraints and the path checks below.
                solution = np.zeros(0) if h is None or np.all(h >= -1e-10) else None
            elif cfg.task_priority_mode == "hierarchical":
                solution, stage1_residual, fallback = self._solve_hierarchical(
                    A, b, P2, c2, lb, ub, G, h, damping + lm
                )
            else:
                P = A.T @ A + P2 + (damping + lm) * np.eye(n_free)
                P = 0.5 * (P + P.T)
                c = A.T @ b + c2
                solution = self._qpsolvers.solve_qp(P, c, G=G, h=h, lb=lb, ub=ub, solver=cfg.solver)
        except Exception as exc:  # noqa: BLE001 - backend exceptions vary
            return self._failure(
                DualArmIKStatus.SOLVER_ERROR,
                f"QP backend '{cfg.solver}' raised: {exc}",
                started,
                state,
                dt,
                min_distance=collision_before,
            )
        if solution is None:
            return self._failure(
                DualArmIKStatus.INFEASIBLE,
                "The QP has no solution under the current velocity, acceleration, position and "
                "collision constraints." + collision_detail,
                started,
                state,
                dt,
                min_distance=collision_before,
            )
        if not np.all(np.isfinite(solution)):
            return self._failure(
                DualArmIKStatus.SOLVER_ERROR,
                "The QP returned a non-finite solution.",
                started,
                state,
                dt,
                min_distance=collision_before,
            )

        step_active = np.array(known_step, dtype=np.float64, copy=True)
        for column, slot in enumerate(free_slots):
            step_active[slot] = float(solution[column])

        step_full = np.zeros(nv)
        step_full[self._active_v] = step_active
        q_next = self._model.integrate(q, step_full)

        min_distance = collision_before
        if self._has_collision:
            ok, min_distance, detail = self._verify_path(q, step_full)
            if not ok:
                return self._failure(
                    DualArmIKStatus.INFEASIBLE,
                    detail,
                    started,
                    state,
                    dt,
                    min_distance=min_distance,
                )

        active_limits = self._active_bound_names(solution, lb, ub, bound_names, bound_parts)
        elapsed = time.perf_counter() - started
        if elapsed > cfg.compute_budget_s:
            return self._failure(
                DualArmIKStatus.DEADLINE_EXCEEDED,
                f"The solve took {elapsed * 1e3:.1f} ms, over its "
                f"{cfg.compute_budget_s * 1e3:.1f} ms budget.",
                started,
                state,
                dt,
                min_distance=min_distance,
            )

        self._previous_step = step_active
        self._previous_dt = dt
        residual_after = float(np.linalg.norm(A @ solution + b)) if b.size else 0.0
        errors = self._arrival_errors(q_next, target)
        errors.update(self._hold_residuals(configuration, target))
        # Continuous joints decode into (-pi, pi]; keep each command on the
        # same turn as the measurement it was computed from.
        positions_next = self._model.positions_from_q(q_next, reference=state.positions_dict())
        targets = {name: positions_next[name] for name in self._active_joints}
        velocities = {
            name: float(step_active[i] / dt) for i, name in enumerate(self._active_joints)
        }
        margins = self._limit_margins(positions_next)

        converged = self._is_converged(errors, target)
        return DualArmIKResult(
            status=DualArmIKStatus.CONVERGED if converged else DualArmIKStatus.TRACKING,
            joint_targets_rad=targets,
            joint_velocities_rad_s=velocities,
            left_position_error_m=errors.get("left_position"),
            left_orientation_error_rad=errors.get("left_orientation"),
            left_axis_error_rad=errors.get("left_axis"),
            right_position_error_m=errors.get("right_position"),
            right_orientation_error_rad=errors.get("right_orientation"),
            right_axis_error_rad=errors.get("right_axis"),
            left_hold_residual_m=errors.get("left_hold"),
            right_hold_residual_m=errors.get("right_hold"),
            active_limits=tuple(active_limits),
            min_collision_distance_m=min_distance,
            min_singular_value=sigma_min,
            limit_margin_rad=margins,
            task_residual_before=residual_before,
            task_residual_after=residual_after,
            stage1_residual=stage1_residual,
            hierarchical_fallback=fallback,
            compute_time_s=elapsed,
            torso_velocity_rad_s=float(step_active[self._torso_slot] / dt),
            dt_s=dt,
            orientation_mode=mode,
            message=(
                f"second stage skipped ({self._stage2_note}); the first-stage step was issued"
                if fallback
                else ""
            ),
            generation=state.mode_generation,
        )

    # -- internals ----------------------------------------------------------

    def _set_gain(self, gain: float) -> None:
        for task in (self._left_task, self._right_task, *self._hold_tasks.values()):
            task.gain = gain
        for axis in self._axis_tasks.values():
            axis.gain = gain

    def _check_freshness(self, state: JointState, target: DualArmTarget) -> str | None:
        if not state.all_valid:
            bad = [n for i, n in enumerate(state.joint_names) if not state.valid[i]]
            return f"Measured state is invalid for joint(s) {bad}."
        age = state.age_s()
        if age > self._config.max_state_age_s:
            return (
                f"Measured state is {age * 1e3:.1f} ms old, over the "
                f"{self._config.max_state_age_s * 1e3:.1f} ms limit."
            )
        if target.is_expired():
            return "The Cartesian target has passed its expiry time."
        return None

    def _configuration_from_state(self, state: JointState) -> NDArray[np.float64]:
        positions = state.positions_dict()
        known = {
            name: positions[name] for name in self._model.movable_joint_names if name in positions
        }
        missing = sorted(set(self._model.movable_joint_names) - set(known))
        if missing:
            raise KeyError(
                f"The measured state does not cover model joint(s) {missing}. The head is not a "
                "decision variable but its measured angle is still needed, because it moves the "
                "collision geometry."
            )
        return self._model.q_from_positions(known)

    def _update_task_targets(
        self, configuration: Any, target: DualArmTarget
    ) -> Tuple[List[Any], List[AxisAlignmentTask]]:
        """Return the Cartesian tasks requested by this target, in the current mode."""
        pin = self._pin
        tasks: List[Any] = []
        axis_tasks: List[AxisAlignmentTask] = []
        for side, frame, task, enabled, wanted in (
            ("left", self._left_frame, self._left_task, target.left_enabled, target.left_target),
            (
                "right",
                self._right_frame,
                self._right_task,
                target.right_enabled,
                target.right_target,
            ),
        ):
            if enabled:
                assert wanted is not None  # guaranteed by DualArmTarget validation
                T = np.asarray(wanted, dtype=np.float64)
                task.set_target(pin.SE3(T[:3, :3], T[:3, 3]))
                self._hold_targets[side] = None
                tasks.append(task)
                if self._orientation_mode == "axis_aligned":
                    axis = self._axis_tasks[side]
                    axis.set_target_from_pose(T)
                    axis_tasks.append(axis)
                continue
            if target.inactive_arm_policy is InactiveArmPolicy.HOLD_JOINTS:
                # Do not add even a zero-error frame task: its Jacobian would
                # still oppose torso motion. Also discard any previous world
                # hold so switching back captures the current TCP.
                self._hold_targets[side] = None
                continue
            # Not driven: latch a hold target once, and only re-capture it on an
            # explicit re-baseline. Overwriting it from the measurement every
            # cycle would make the hold drift with whatever error is present.
            if self._hold_targets[side] is None or target.rebaseline:
                self._hold_targets[side] = configuration.get_transform_frame_to_world(frame).copy()
            hold_task = self._hold_tasks[side]
            hold_task.set_target(self._hold_targets[side])
            tasks.append(hold_task)
        return tasks, axis_tasks

    def _variable_layout(
        self, target: DualArmTarget, dt: float
    ) -> Tuple[List[int], NDArray[np.float64]]:
        """Return the free variable slots and the known step of the fixed ones."""
        n_active = len(self._active_joints)
        known = np.zeros(n_active)
        slots = list(range(n_active))
        if target.inactive_arm_policy is InactiveArmPolicy.HOLD_JOINTS:
            for enabled, joints in (
                (target.left_enabled, self._left_joints),
                (target.right_enabled, self._right_joints),
            ):
                if not enabled:
                    for joint in joints:
                        slots.remove(self._active_joints.index(joint))
        if target.torso_policy is TorsoPolicy.FIXED:
            slots.remove(self._torso_slot)
        elif target.torso_policy is TorsoPolicy.MANUAL:
            slots.remove(self._torso_slot)
            known[self._torso_slot] = target.torso_velocity_rad_s * dt
        return slots, known

    def _min_singular_value(
        self,
        q: NDArray[np.float64],
        target: DualArmTarget,
        free_slots: Sequence[int],
    ) -> float | None:
        """Smallest normalised singular value over the enabled hands' Jacobians."""
        cfg = self._config
        free_columns = [int(self._active_v[slot]) for slot in free_slots]
        if not free_columns:
            return None
        smallest: float | None = None
        for enabled, frame in (
            (target.left_enabled, self._left_frame),
            (target.right_enabled, self._right_frame),
        ):
            if not enabled:
                continue
            J = self._model.frame_jacobian(q, frame, local=False)[:, free_columns]
            if self._orientation_mode == "position_only":
                sigma = np.linalg.svd(J[:3, :] / cfg.position_tolerance_m, compute_uv=False)
            else:
                sigma = normalised_singular_values(
                    J,
                    position_scale_m=cfg.position_tolerance_m,
                    orientation_scale_rad=cfg.orientation_tolerance_rad,
                )
            value = float(sigma[-1]) if sigma.size else 0.0
            smallest = value if smallest is None else min(smallest, value)
        return smallest

    def _secondary_objective(
        self,
        q: NDArray[np.float64],
        free_slots: Sequence[int],
        dt: float,
        gain: float,
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """``(P, c)`` of the posture, motion, smoothing and limit-avoidance costs.

        Costs are weights on a residual and enter the objective squared, as
        Pink's task costs do, so ``posture_cost`` keeps the meaning it had with
        Pink's ``PostureTask``.  All four are separate quadratic terms in the
        free step ``x``:

        * posture: ``1/2 w_p (x - alpha (q* - q))^2`` -- the step that would
          take the joint towards its preferred angle at the task gain;
        * motion: ``1/2 w_m x^2``;
        * smoothing: ``1/2 w_s (x - v_prev dt)^2`` with ``v_prev`` the previous
          step over *its* period;
        * limit avoidance: inside the band next to a limit, ``1/2 w_l(d)
          (x - x_in)^2`` with ``x_in`` a step back to the band's edge and
          ``w_l`` ramping from zero at the band's edge to the configured cost
          at the limit.  A joint outside every band feels nothing.
        """
        cfg = self._config
        n = len(free_slots)
        P = np.zeros((n, n))
        c = np.zeros(n)
        if n == 0:
            return P, c
        assert self._posture_q is not None
        # Posture: the tangent difference from the current to the preferred
        # configuration, restricted to the active joints.
        toward = self._model.difference(q, self._posture_q)[self._active_v]
        positions = self._model.positions_from_q(q)
        lower, upper = self._model.position_limits(list(self._active_joints))
        for column, slot in enumerate(free_slots):
            w_p = self._posture_weights[slot] ** 2
            if w_p > 0.0:
                P[column, column] += w_p
                c[column] += -w_p * gain * toward[slot]
            w_m = self._motion_weights[slot] ** 2
            if w_m > 0.0:
                P[column, column] += w_m
            w_s = self._smoothing_weights[slot] ** 2
            if w_s > 0.0 and self._previous_step is not None and self._previous_dt:
                v_prev = self._previous_step[slot] / self._previous_dt
                P[column, column] += w_s
                c[column] += -w_s * v_prev * dt
            if cfg.limit_avoidance_enabled:
                name = self._active_joints[slot]
                value = positions[name]
                band = cfg.limit_avoidance_band_rad
                for limit, inward in ((lower[slot], +1.0), (upper[slot], -1.0)):
                    if not np.isfinite(limit):
                        continue
                    distance = abs(value - limit)
                    if distance >= band:
                        continue
                    penetration = (band - distance) / band  # 0 at the edge, 1 on the limit
                    w_l = (cfg.limit_avoidance_cost * penetration) ** 2
                    x_in = inward * (band - distance) * gain
                    P[column, column] += w_l
                    c[column] += -w_l * x_in
        return P, c

    def _solve_hierarchical(
        self,
        A: NDArray[np.float64],
        b: NDArray[np.float64],
        P2: NDArray[np.float64],
        c2: NDArray[np.float64],
        lb: NDArray[np.float64],
        ub: NDArray[np.float64],
        G: NDArray[np.float64] | None,
        h: NDArray[np.float64] | None,
        regularisation: float,
    ) -> Tuple[NDArray[np.float64] | None, float | None, bool]:
        """Two stages over the same constraints; the second keeps the first's task output.

        The second stage is solved in the null space of the first: with
        ``A = U S V^T`` and ``V_r`` the right singular vectors of the
        numerically non-zero singular values, every step ``x = x_1 + N z``
        with ``N`` spanning the complement of ``V_r`` has the same task output
        as ``x_1``.  The bounds and the collision rows become inequalities on
        ``z`` with ``z = 0`` feasible by construction, so the backend never
        sees an equality system it might call inconsistent by rounding.

        Returns:
            ``(step, stage1_residual, fallback)``; ``step`` is ``None`` when
            the first stage itself is infeasible.
        """
        cfg = self._config
        n = A.shape[1]
        eye = np.eye(n)
        P1 = A.T @ A + regularisation * eye
        P1 = 0.5 * (P1 + P1.T)
        c1 = A.T @ b
        x1 = self._qpsolvers.solve_qp(P1, c1, G=G, h=h, lb=lb, ub=ub, solver=cfg.solver)
        if x1 is None or not np.all(np.isfinite(x1)):
            return None, None, False
        # The backend's answer respects the bounds to its own tolerance; make
        # that exact so the second stage starts from a feasible point.
        x1 = np.clip(x1, lb, ub)
        stage1 = float(np.linalg.norm(A @ x1 + b)) if b.size else 0.0
        rho = regularisation + cfg.secondary_regularisation
        if A.shape[0] == 0:
            # No first-priority task at all (nothing enabled with HOLD_JOINTS):
            # the secondary objective is the whole problem.
            P = P2 + rho * eye
            P = 0.5 * (P + P.T)
            x2 = self._qpsolvers.solve_qp(P, c2, G=G, h=h, lb=lb, ub=ub, solver=cfg.solver)
            if x2 is None or not np.all(np.isfinite(x2)):
                return x1, stage1, True
            return x2, stage1, False
        _, s_values, Vt = np.linalg.svd(A, full_matrices=True)
        rank = int(
            np.sum(s_values > cfg.hierarchical_rank_tolerance * max(float(s_values[0]), 1e-300))
        )
        N = Vt[rank:].T  # (n, n - rank): directions that leave the task output alone
        if N.shape[1] == 0:
            return x1, stage1, False
        # Secondary objective in z: 1/2 (x1 + N z)^T P (x1 + N z) + c2^T (x1 + N z).
        P_full = P2 + rho * eye
        P_z = N.T @ P_full @ N
        P_z = 0.5 * (P_z + P_z.T)
        c_z = N.T @ (P_full @ x1 + c2)
        # The minimiser is invariant to a common scale of (P, c); the backend's
        # tolerances are not, and a secondary Hessian of 1e-3 against
        # constraint rows of order one is where they bite.
        scale = 1.0 / max(float(np.max(np.abs(np.diag(P_z)))), 1e-300)
        P_z = P_z * scale
        c_z = c_z * scale
        # Bounds and collision rows as inequalities on z; z = 0 is feasible.
        G_rows = [N, -N]
        h_rows = [np.maximum(ub - x1, 0.0), np.maximum(x1 - lb, 0.0)]
        if G is not None and h is not None:
            G_rows.append(G @ N)
            h_rows.append(np.maximum(h - G @ x1, 0.0))
        try:
            z = self._qpsolvers.solve_qp(
                P_z,
                c_z,
                G=np.vstack(G_rows),
                h=np.concatenate(h_rows),
                solver=cfg.solver,
            )
        except Exception:  # noqa: BLE001 - a numerically awkward second stage is not fatal
            z = None
        if z is None or not np.all(np.isfinite(z)):
            self._stage2_note = "the backend returned no second-stage solution"
            return x1, stage1, True
        x2 = x1 + N @ z
        # Verify rather than trust: the second stage may not worsen the first
        # priority beyond rounding, nor leave the bounds.
        stage2 = float(np.linalg.norm(A @ x2 + b))
        tolerance = 1e-6 * max(1.0, stage1)
        if stage2 > stage1 + tolerance:
            self._stage2_note = (
                f"the second stage would raise the task residual from {stage1:.3e} to {stage2:.3e}"
            )
            return x1, stage1, True
        if np.any(x2 < lb - 1e-9) or np.any(x2 > ub + 1e-9):
            self._stage2_note = "the second stage left the step bounds"
            return x1, stage1, True
        return np.clip(x2, lb, ub), stage1, False

    def _step_bounds(
        self,
        q: NDArray[np.float64],
        free_slots: Sequence[int],
        known_step: NDArray[np.float64],
        dt: float,
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64], List[str], Dict[str, Any]]:
        cfg = self._config
        names = [self._active_joints[slot] for slot in free_slots]

        urdf_velocity = np.asarray(
            [
                float(self._model.model.velocityLimit[self._model.joint_v_index(name)])
                for name in names
            ],
            dtype=np.float64,
        )
        v_max = _as_per_joint(
            cfg.max_joint_velocity_rad_s, names, urdf_velocity, name="max_joint_velocity_rad_s"
        )
        if not np.all(np.isfinite(v_max)):
            offenders = [names[i] for i in np.flatnonzero(~np.isfinite(v_max))]
            raise ValueError(
                f"No finite velocity limit for {offenders}. The URDF value is missing or "
                "infinite; supply a measured limit."
            )
        a_max = _as_per_joint(
            cfg.max_joint_acceleration_rad_s2,
            names,
            np.full(len(names), np.inf),
            name="max_joint_acceleration_rad_s2",
        )

        # Hard bounds: velocity, the per-cycle step ceiling, and position
        # limits. None of these is ever relaxed.
        lb_speed = np.maximum(-v_max * dt, -cfg.max_joint_step_rad)
        ub_speed = np.minimum(v_max * dt, cfg.max_joint_step_rad)

        positions = self._model.positions_from_q(q)
        # Removing an inactive arm from the variables must not remove its
        # limits. A zero step cannot recover an already invalid held joint.
        fixed_names = [
            name
            for slot, name in enumerate(self._active_joints)
            if slot not in free_slots and known_step[slot] == 0.0
        ]
        fixed_lower, fixed_upper = self._model.position_limits(fixed_names)
        for name, lo, hi in zip(fixed_names, fixed_lower, fixed_upper):
            # A stationary joint may sit on a valid limit (Rakuda's elbows
            # do at zero). The motion margin must not force it to move.
            if not (lo - 1e-9 <= positions[name] <= hi + 1e-9):
                amount = positions[name] - hi if positions[name] > hi else lo - positions[name]
                raise ValueError(
                    f"Held joint '{name}' is outside its limits by {amount:.4f} rad "
                    f"(at {positions[name]:.4f}, allowed [{lo:.4f}, {hi:.4f}])."
                )
        lower, upper = self._model.position_limits(names)
        current = np.asarray([positions[name] for name in names])
        # The position bound never *forces* motion. Outside the margin band
        # the step may approach the limit up to the margin; inside the band
        # (a joint parked on its stop, as the Rakuda's right elbow is at the
        # CAD zero) it may only move inwards, and any inward step, however
        # small, is allowed. The old form ``lower + margin - current`` demanded
        # the whole margin back in one cycle, which the speed bound could not
        # give, and the set went empty exactly where a small step home was the
        # answer. A joint genuinely *beyond* its limit is reported below by
        # name and amount, not clamped back silently.
        lb_position = np.minimum(lower + cfg.position_limit_margin_rad - current, 0.0)
        ub_position = np.maximum(upper - cfg.position_limit_margin_rad - current, 0.0)
        # Rounding puts a joint that sits on its stop a few 1e-16 rad past it;
        # that is "on the limit", not a violation.
        tolerance = 1e-9
        beyond = []
        for i, name in enumerate(names):
            if current[i] > upper[i] + tolerance:
                beyond.append((name, float(current[i] - upper[i])))
            elif current[i] < lower[i] - tolerance:
                beyond.append((name, float(lower[i] - current[i])))
        if beyond:
            listed = ", ".join(f"{name} by {amount:.4f} rad" for name, amount in beyond)
            raise ValueError(
                f"The configuration is outside the position limits of: {listed}. No feasible "
                "step exists; move the machine back inside its limits (the amounts above) "
                "before commanding motion."
            )
        lb_hard = np.maximum(lb_speed, lb_position)
        ub_hard = np.minimum(ub_speed, ub_position)

        components: Dict[str, Any] = {
            "lb_speed": lb_speed,
            "ub_speed": ub_speed,
            "lb_position": lb_position,
            "ub_position": ub_position,
        }

        if self._previous_step is None or not self._previous_dt:
            lb, ub = lb_hard, ub_hard
        else:
            # The acceleration bound is a smoothness constraint, not a safety
            # one. Intersecting it with the hard bounds can empty the feasible
            # set exactly when a joint approaching its position limit needs to
            # decelerate -- "you may not slow down" is never the right answer.
            # So the acceleration window is *clipped into* the hard window
            # instead: when it lies entirely outside, the joint brakes as hard
            # as the hard bounds allow, and the set stays non-empty unless the
            # configuration genuinely violates a position limit.  The previous
            # velocity is the previous step over *its own* period: two cycles
            # of different length compare velocities, not displacements.
            v_prev = np.asarray(
                [self._previous_step[slot] / self._previous_dt for slot in free_slots]
            )
            lb_acceleration = (v_prev - a_max * dt) * dt
            ub_acceleration = (v_prev + a_max * dt) * dt
            lb = np.maximum(lb_hard, np.minimum(lb_acceleration, ub_hard))
            ub = np.minimum(ub_hard, np.maximum(ub_acceleration, lb_hard))
            components["lb_acceleration"] = lb_acceleration
            components["ub_acceleration"] = ub_acceleration

        # A MANUAL torso step is a known quantity, but it still has to respect
        # the torso's own limits; reject it rather than silently exceeding them.
        if known_step[self._torso_slot] != 0.0:
            torso = self._active_joints[self._torso_slot]
            t_lower, t_upper = self._model.position_limits([torso])
            candidate = positions[torso] + known_step[self._torso_slot]
            if not (t_lower[0] <= candidate <= t_upper[0]):
                raise ValueError(
                    f"The commanded manual torso step would take '{torso}' to "
                    f"{candidate:.4f} rad, outside [{t_lower[0]:.4f}, {t_upper[0]:.4f}]."
                )
        return lb, ub, names, components

    def _collision_constraints(
        self,
        q: NDArray[np.float64],
        report: Any,
        S: NDArray[np.float64],
        known_step: NDArray[np.float64],
        dt: float,
    ) -> Tuple[NDArray[np.float64] | None, NDArray[np.float64] | None]:
        """Linear self-collision constraints, in terms of the free step variables.

        For each active pair the distance rate is bounded below::

            grad(d)^T v >= -eta (d - d_safe)

        which, with ``dq = v dt`` and the QP's ``G x <= h`` form, becomes
        ``-row @ dq <= eta (d - d_safe) dt``.
        """
        cfg = self._config
        active = np.flatnonzero(report.distances < cfg.collision_activation_distance_m)
        if active.size == 0:
            return None, None

        nv = self._model.nv
        known_full = np.zeros(nv)
        known_full[self._active_v] = known_step

        rows: List[NDArray[np.float64]] = []
        bounds: List[float] = []
        for k in active:
            row = self._model.distance_jacobian_row(
                q,
                report.witness_a[k],
                report.joint_a[k],
                report.witness_b[k],
                report.joint_b[k],
                report.normals[k],
            )
            margin = (
                cfg.collision_gain
                * (float(report.distances[k]) - cfg.collision_safety_distance_m)
                * dt
            )
            rows.append(-(row @ S))
            bounds.append(margin + float(row @ known_full))
        return (
            np.asarray(rows, dtype=np.float64),
            np.asarray(bounds, dtype=np.float64),
        )

    def _collision_diagnosis(self, report: Any) -> str:
        """Name the pairs that are already inside the safety distance.

        When a collision constraint is what makes the step infeasible, the
        useful answer is *which* pairs, because a pair that sits permanently
        inside the safety distance -- a bearing beside its own washer -- is an
        exclusion to record, not an obstacle to avoid.
        """
        cfg = self._config
        inside = np.flatnonzero(report.distances < cfg.collision_safety_distance_m)
        if inside.size == 0:
            return ""
        order = inside[np.argsort(report.distances[inside])][:5]
        worst = "; ".join(
            f"{report.pair_names[k][0]} <-> {report.pair_names[k][1]} at "
            f"{report.distances[k] * 1e3:.1f} mm"
            for k in order
        )
        return (
            f" {inside.size} pair(s) are already inside the "
            f"{cfg.collision_safety_distance_m * 1e3:.0f} mm safety distance, so the constraints "
            f"demand they all separate at once. Closest: {worst}. A pair that is permanently this "
            "close is an exclusion to record, or the safety distance is too large for this model."
        )

    def _verify_path(
        self,
        q: NDArray[np.float64],
        step_full: NDArray[np.float64],
    ) -> Tuple[bool, float, str]:
        """Re-check the candidate step at interpolated configurations.

        The distance constraint is a linearisation around ``q``; it says nothing
        about the interior of a large step.  Sampling the interval catches a step
        that satisfies the constraint at both ends yet passes through contact.
        """
        cfg = self._config
        worst = np.inf
        worst_detail = ""
        substeps = max(1, cfg.collision_substeps)
        for i in range(1, substeps + 1):
            fraction = i / substeps
            report = self._model.collision_report(self._model.integrate(q, step_full * fraction))
            if report.min_distance < worst:
                worst = report.min_distance
                worst_detail = (
                    f"min distance {report.min_distance:.4f} m at {report.closest_pair()} "
                    f"({fraction * 100:.0f}% along the step)"
                )
            if report.min_distance < 0.0:
                return (
                    False,
                    float(worst),
                    f"The candidate step passes through self-collision: {worst_detail}.",
                )
        return True, float(worst), worst_detail

    def _arrival_errors(
        self, q_next: NDArray[np.float64], target: DualArmTarget
    ) -> Dict[str, float]:
        """World-frame errors at the configuration the step arrives at.

        Position: the distance between the TCP and its target.  Orientation:
        the angle of ``R_target^T R_tcp`` -- the SO(3) geodesic, not Pink's
        internal SE(3) residual, whose rotational part is coupled to the
        translation.  Axis: the angle between the approach axis and its target.
        """
        out: Dict[str, float] = {}
        frames = self._model.frame_poses(q_next, [self._left_frame, self._right_frame])
        for side, frame, enabled, wanted in (
            ("left", self._left_frame, target.left_enabled, target.left_target),
            ("right", self._right_frame, target.right_enabled, target.right_target),
        ):
            T = frames[frame]
            if enabled:
                assert wanted is not None
                goal = np.asarray(wanted, dtype=np.float64)
                out[f"{side}_position"] = float(np.linalg.norm(goal[:3, 3] - T[:3, 3]))
                out[f"{side}_orientation"] = _rotation_angle(goal[:3, :3].T @ T[:3, :3])
                if side in self._axis_tasks and self._orientation_mode == "axis_aligned":
                    out[f"{side}_axis"] = self._axis_tasks[side].angle_error(q_next)
                continue
        return out

    def _hold_residuals(self, configuration: Any, target: DualArmTarget) -> Dict[str, float]:
        """How far each held hand had drifted from its latched hold, as measured."""
        out: Dict[str, float] = {}
        if target.inactive_arm_policy is InactiveArmPolicy.HOLD_JOINTS:
            return out
        for side, enabled in (("left", target.left_enabled), ("right", target.right_enabled)):
            if enabled or self._hold_targets[side] is None:
                continue
            error = self._hold_tasks[side].compute_error(configuration)
            out[f"{side}_hold"] = float(np.linalg.norm(error[:3]))
        return out

    def _limit_margins(self, positions: Mapping[str, float]) -> Dict[str, float]:
        lower, upper = self._model.position_limits(list(self._active_joints))
        out: Dict[str, float] = {}
        for i, name in enumerate(self._active_joints):
            if np.isfinite(lower[i]) and np.isfinite(upper[i]):
                out[name] = float(min(positions[name] - lower[i], upper[i] - positions[name]))
        return out

    def _is_converged(self, errors: Mapping[str, float], target: DualArmTarget) -> bool:
        cfg = self._config
        for side, enabled in (("left", target.left_enabled), ("right", target.right_enabled)):
            if not enabled:
                continue
            if errors.get(f"{side}_position", np.inf) > cfg.position_tolerance_m:
                return False
            # Only what the mode asks for can withhold "converged": a
            # position-only jog is not judged on the orientation it never
            # tried to hold, and a weight of zero means the same thing.
            if self._orientation_mode == "pose" and self._orientation_cost > 0.0:
                if errors.get(f"{side}_orientation", np.inf) > cfg.orientation_tolerance_rad:
                    return False
            elif self._orientation_mode == "axis_aligned" and self._orientation_cost > 0.0:
                if errors.get(f"{side}_axis", np.inf) > cfg.orientation_tolerance_rad:
                    return False
        return True

    @staticmethod
    def _active_bound_names(
        solution: NDArray[np.float64],
        lb: NDArray[np.float64],
        ub: NDArray[np.float64],
        names: Sequence[str],
        parts: Mapping[str, Any],
    ) -> List[str]:
        """Name each binding bound *and which constraint it came from*.

        ``joint:lower`` / ``joint:upper`` is a position limit clipping this step,
        ``joint:at_lower`` / ``joint:at_upper`` a joint already parked on its
        limit (zero step), ``joint:speed`` the velocity or per-cycle step
        ceiling, ``joint:accel`` the acceleration window. Reporting all of them
        as "lower"/"upper" would make a joint pinned by its speed limit look
        like one sitting on a hard stop.
        """
        tolerance = 1e-9
        active: List[str] = []
        for i, name in enumerate(names):
            side = None
            if solution[i] <= lb[i] + tolerance:
                side = "lb"
            elif solution[i] >= ub[i] - tolerance:
                side = "ub"
            if side is None:
                # A joint parked on a hard stop takes a zero step, which no bound
                # "clips" -- yet the stop is exactly why the hand is not moving.
                # Report it so a stall can be told apart from a workspace edge.
                if abs(parts["lb_position"][i]) <= 1e-6:
                    active.append(f"{name}:at_lower")
                elif abs(parts["ub_position"][i]) <= 1e-6:
                    active.append(f"{name}:at_upper")
                continue
            bound = lb[i] if side == "lb" else ub[i]
            if abs(parts[f"{side}_position"][i] - bound) <= tolerance:
                # A zero position bound is a joint parked on its stop, held
                # there by the bound; say so rather than "clipped".
                if abs(bound) <= 1e-9:
                    active.append(f"{name}:{'at_lower' if side == 'lb' else 'at_upper'}")
                else:
                    active.append(f"{name}:{'lower' if side == 'lb' else 'upper'}")
            elif abs(parts[f"{side}_speed"][i] - bound) <= tolerance:
                active.append(f"{name}:speed")
            elif f"{side}_acceleration" in parts:
                active.append(f"{name}:accel")
            else:
                active.append(f"{name}:{'lower' if side == 'lb' else 'upper'}")
        return active

    def _failure(
        self,
        status: DualArmIKStatus,
        message: str,
        started: float,
        state: JointState,
        dt: float,
        *,
        min_distance: float | None = None,
    ) -> DualArmIKResult:
        """Build a result that carries no motion command, only a diagnosis."""
        self._previous_step = None
        self._previous_dt = None
        return DualArmIKResult(
            status=status,
            compute_time_s=time.perf_counter() - started,
            min_collision_distance_m=min_distance,
            dt_s=dt,
            orientation_mode=self._orientation_mode,
            message=message,
            generation=state.mode_generation,
        )


def _require_pink() -> Any:
    """Import Pink, with an actionable message when the extra is missing."""
    try:
        import pink  # noqa: PLC0415
        import pink.tasks  # noqa: F401, PLC0415
    except ImportError as exc:  # pragma: no cover - depends on the environment
        from .urdf_model import _INSTALL_HINT, MissingKinematicsExtra  # noqa: PLC0415

        raise MissingKinematicsExtra(_INSTALL_HINT) from exc
    return pink


def _require_qpsolvers() -> Any:
    """Import qpsolvers, with an actionable message when the extra is missing."""
    try:
        import qpsolvers  # noqa: PLC0415
    except ImportError as exc:  # pragma: no cover - depends on the environment
        from .urdf_model import _INSTALL_HINT, MissingKinematicsExtra  # noqa: PLC0415

        raise MissingKinematicsExtra(_INSTALL_HINT) from exc
    return qpsolvers
