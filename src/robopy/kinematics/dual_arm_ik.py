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
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.control.types import DualArmTarget, InactiveArmPolicy, JointState, TorsoPolicy

from .urdf_model import WholeBodyModel, require_pinocchio

if TYPE_CHECKING:  # pragma: no cover - typing only
    pass

__all__ = [
    "DualArmIK",
    "DualArmIKConfig",
    "DualArmIKResult",
    "DualArmIKStatus",
]


class DualArmIKStatus(Enum):
    """Outcome of one :meth:`DualArmIK.solve_step` call.

    Attributes:
        CONVERGED: Both enabled hand tasks are inside tolerance; the step is a
            small correction only.
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
    """

    CONVERGED = "converged"
    TRACKING = "tracking"
    INFEASIBLE = "infeasible"
    STALE_STATE = "stale_state"
    SOLVER_ERROR = "solver_error"
    COLLISION_AT_START = "collision_at_start"
    DEADLINE_EXCEEDED = "deadline_exceeded"

    @property
    def is_commandable(self) -> bool:
        """Whether a result with this status may be issued as new motion."""
        return self in (DualArmIKStatus.CONVERGED, DualArmIKStatus.TRACKING)


@dataclass
class DualArmIKConfig:
    """Weights, limits and policies for the dual-arm solver.

    Attributes:
        position_cost: Weight on each hand's position error.
        orientation_cost: Weight on each hand's orientation error.  The first
            iteration deliberately favours position, leaving orientation as a
            softer objective.
        left_priority: Extra multiplier on the left hand's costs.
        right_priority: Extra multiplier on the right hand's costs.
        hold_position_cost: Position weight of a non-driven hand's hold task.
        hold_orientation_cost: Orientation weight of a hold task.  A hold with a
            finite weight is a *preference*, not a lock; the residual is
            reported rather than described as exact.
        posture_cost: Weight pulling the arms towards ``posture_reference``.
        torso_regularisation: Extra quadratic penalty on torso motion, applied
            only under :attr:`TorsoPolicy.OPTIMIZE`.
        damping: Tikhonov regularisation on the whole step.  Keeps the QP
            strictly convex and bounds the step near singularities.
        lm_damping: Levenberg-Marquardt damping passed to the frame tasks.
        gain: Task gain ``alpha`` in Pink's residual.
        max_joint_velocity_rad_s: Per-joint velocity ceiling.  ``None`` falls
            back to the URDF's velocity limit, which for a CAD export is often a
            placeholder and should be replaced.
        max_joint_acceleration_rad_s2: Per-joint bound on ``|v - v_prev| / dt``.
        max_joint_step_rad: Hard ceiling on how far a joint target may move in
            one cycle, independent of the velocity limit.
        position_limit_margin_rad: Stay this far inside every position limit.
        position_tolerance_m: Position error under which a hand counts as
            converged.
        orientation_tolerance_rad: Orientation error under which a hand counts
            as converged.
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
        posture_reference: ``{joint: radians}`` the posture task pulls towards.
            ``None`` uses the configuration the solver was last reset to.
    """

    position_cost: float = 1.0
    orientation_cost: float = 0.15
    left_priority: float = 1.0
    right_priority: float = 1.0
    hold_position_cost: float = 1.0
    hold_orientation_cost: float = 0.15
    posture_cost: float = 1e-3
    torso_regularisation: float = 1e-2
    damping: float = 1e-6
    lm_damping: float = 1e-6
    gain: float = 1.0
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
    posture_reference: Mapping[str, float] | None = None


@dataclass(frozen=True)
class DualArmIKResult:
    """Everything one solver step produced, including why it produced nothing.

    Attributes:
        status: See :class:`DualArmIKStatus`.
        joint_targets_rad: ``{joint: radians}`` for the active joints.  Empty
            when :attr:`status` is not commandable.
        joint_velocities_rad_s: ``{joint: rad/s}`` implied by the step.
        left_position_error_m: Norm of the left hand's position error *before*
            the step, in metres.  ``None`` when the left task was not evaluated.
        left_orientation_error_rad: Norm of the left hand's orientation error.
        right_position_error_m: Same for the right hand.
        right_orientation_error_rad: Same for the right hand.
        left_hold_residual_m: Position residual of the left hold task when the
            left hand is not driven.  A hold is a weighted objective, so this is
            reported instead of claiming the hand is pinned.
        right_hold_residual_m: Same for the right hand.
        active_limits: Names of the constraints that were active (clipped).
        min_collision_distance_m: Smallest checked pair distance at the
            candidate configuration, or ``None`` when collision is not modelled.
        compute_time_s: Wall-clock duration of the solve.
        torso_velocity_rad_s: Commanded torso velocity.
        message: Human-readable diagnosis, especially for failures.
        generation: The state's mode generation, carried through so a consumer
            can reject a command produced from a superseded generation.
    """

    status: DualArmIKStatus
    joint_targets_rad: Dict[str, float] = field(default_factory=dict)
    joint_velocities_rad_s: Dict[str, float] = field(default_factory=dict)
    left_position_error_m: float | None = None
    left_orientation_error_rad: float | None = None
    right_position_error_m: float | None = None
    right_orientation_error_rad: float | None = None
    left_hold_residual_m: float | None = None
    right_hold_residual_m: float | None = None
    active_limits: Tuple[str, ...] = ()
    min_collision_distance_m: float | None = None
    compute_time_s: float = 0.0
    torso_velocity_rad_s: float = 0.0
    message: str = ""
    generation: int = 0

    @property
    def is_commandable(self) -> bool:
        """Whether this result may be issued as a new motion command."""
        return self.status.is_commandable and bool(self.joint_targets_rad)


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

        self._left_task = self._pink.tasks.FrameTask(
            left_frame,
            position_cost=self._config.position_cost * self._config.left_priority,
            orientation_cost=self._config.orientation_cost * self._config.left_priority,
            lm_damping=self._config.lm_damping,
            gain=self._config.gain,
        )
        self._right_task = self._pink.tasks.FrameTask(
            right_frame,
            position_cost=self._config.position_cost * self._config.right_priority,
            orientation_cost=self._config.orientation_cost * self._config.right_priority,
            lm_damping=self._config.lm_damping,
            gain=self._config.gain,
        )
        self._posture_task = self._pink.tasks.PostureTask(
            cost=self._config.posture_cost,
            lm_damping=self._config.lm_damping,
            gain=self._config.gain,
        )

        self._hold_targets: Dict[str, Any] = {"left": None, "right": None}
        self._hold_tasks = {
            side: self._pink.tasks.FrameTask(
                frame,
                position_cost=self._config.hold_position_cost,
                orientation_cost=self._config.hold_orientation_cost,
                lm_damping=self._config.lm_damping,
                gain=self._config.gain,
            )
            for side, frame in (("left", left_frame), ("right", right_frame))
        }
        self._previous_step: NDArray[np.float64] | None = None
        self._posture_q: NDArray[np.float64] | None = None
        self._has_collision = model.collision_model is not None
        self._orientation_cost = self._config.orientation_cost

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

    # -- lifecycle ----------------------------------------------------------

    def reset(self, q: NDArray[np.float64] | None = None) -> None:
        """Clear latched state: hold targets, warm start and posture reference.

        Call this on every mode change and after any clutch, so that a stale
        hold target or a stale previous step cannot leak across the boundary.
        """
        self._hold_targets = {"left": None, "right": None}
        self._previous_step = None
        if q is not None:
            self._posture_q = np.array(q, dtype=np.float64, copy=True)

    def set_posture_reference(self, positions_rad: Mapping[str, float]) -> None:
        """Set the configuration the posture task pulls towards."""
        self._posture_q = self._model.q_from_positions(positions_rad, require_all=False)

    def set_task_costs(
        self,
        *,
        position_cost: float | None = None,
        orientation_cost: float | None = None,
    ) -> None:
        """Re-weight the hand tasks without rebuilding the solver.

        Rakuda's arms have a two-axis wrist (yaw and pitch), so a pose that
        keeps the hand's full orientation while translating it is often not
        reachable at all; the solver then settles on a weighted compromise.
        Lowering ``orientation_cost`` -- to zero for a position-only jog -- makes
        the translation exact and lets the orientation float, which is the
        useful behaviour for jogging this kind of arm.  An orientation cost of
        zero also removes orientation from the convergence test.

        Args:
            position_cost: New position weight, applied with each side's
                priority multiplier.  ``None`` leaves it unchanged.
            orientation_cost: New orientation weight, likewise.
        """
        cfg = self._config
        for task, priority in (
            (self._left_task, cfg.left_priority),
            (self._right_task, cfg.right_priority),
        ):
            if position_cost is not None:
                task.set_position_cost(position_cost * priority)
            if orientation_cost is not None:
                task.set_orientation_cost(orientation_cost * priority)
        if orientation_cost is not None:
            self._orientation_cost = orientation_cost

    @property
    def orientation_cost(self) -> float:
        """The orientation weight currently applied to the hand tasks."""
        return self._orientation_cost

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
        integrated away.  The previous solution is used only as a warm start for
        the acceleration bound.

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
        if dt <= 0.0:
            return self._failure(
                DualArmIKStatus.SOLVER_ERROR, "dt must be positive.", started, state
            )

        stale = self._check_freshness(state, target)
        if stale is not None:
            return self._failure(DualArmIKStatus.STALE_STATE, stale, started, state)

        try:
            q = self._configuration_from_state(state)
        except (KeyError, ValueError) as exc:
            return self._failure(DualArmIKStatus.SOLVER_ERROR, str(exc), started, state)

        if self._posture_q is None:
            self._posture_q = np.array(q, dtype=np.float64, copy=True)
        if self._config.posture_reference is not None:
            self._posture_q = self._model.q_from_positions(
                self._config.posture_reference, base=q, require_all=False
            )

        configuration = self._pink.Configuration(self._model.model, self._model.data, q)

        hand_tasks = self._update_task_targets(configuration, target)
        self._posture_task.set_target(self._posture_q)

        # --- objective over the active tangent directions --------------------
        nv = self._model.nv
        H_full = np.zeros((nv, nv))
        c_full = np.zeros(nv)
        for task in (*hand_tasks, self._posture_task):
            H_task, c_task = task.compute_qp_objective(configuration)
            H_full += H_task
            c_full += c_task

        free_slots, known_step = self._variable_layout(target, dt)
        S = np.zeros((nv, len(free_slots)))
        for column, slot in enumerate(free_slots):
            S[self._active_v[slot], column] = 1.0
        # The known (non-optimised) part of the step, lifted to the full tangent
        # space so it can be folded into the objective's linear term.
        known_full = np.zeros(nv)
        known_full[self._active_v] = known_step

        P = S.T @ H_full @ S
        c = S.T @ (H_full @ known_full + c_full)
        P += self._config.damping * np.eye(P.shape[0])
        if target.torso_policy is TorsoPolicy.OPTIMIZE and self._torso_slot in free_slots:
            torso_column = free_slots.index(self._torso_slot)
            P[torso_column, torso_column] += self._config.torso_regularisation
        P = 0.5 * (P + P.T)

        # --- bounds and constraints ------------------------------------------
        try:
            lb, ub, bound_names, bound_parts = self._step_bounds(q, free_slots, known_step, dt)
        except ValueError as exc:
            return self._failure(DualArmIKStatus.INFEASIBLE, str(exc), started, state)

        if np.any(lb > ub):
            offenders = [bound_names[i] for i in np.flatnonzero(lb > ub)]
            return self._failure(
                DualArmIKStatus.INFEASIBLE,
                f"The configuration already violates the limits of {offenders}; no feasible step "
                "exists. Move the machine back inside its limits before commanding motion.",
                started,
                state,
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
                    min_distance=collision_before,
                )
            G, h = self._collision_constraints(q, report, S, known_step, dt)
            collision_detail = self._collision_diagnosis(report)

        if time.perf_counter() - started > self._config.compute_budget_s:
            return self._failure(
                DualArmIKStatus.DEADLINE_EXCEEDED,
                "The solve exceeded its compute budget before reaching the QP.",
                started,
                state,
                min_distance=collision_before,
            )

        try:
            if not free_slots:
                # Both arms can be inactive with a fixed/manual torso. There
                # is no optimisation variable, but known motion must still
                # satisfy collision constraints and the path checks below.
                solution = np.zeros(0) if h is None or np.all(h >= -1e-10) else None
            else:
                solution = self._qpsolvers.solve_qp(
                    P, c, G=G, h=h, lb=lb, ub=ub, solver=self._config.solver
                )
        except Exception as exc:  # noqa: BLE001 - backend exceptions vary
            return self._failure(
                DualArmIKStatus.SOLVER_ERROR,
                f"QP backend '{self._config.solver}' raised: {exc}",
                started,
                state,
                min_distance=collision_before,
            )
        if solution is None:
            return self._failure(
                DualArmIKStatus.INFEASIBLE,
                "The QP has no solution under the current velocity, acceleration, position and "
                "collision constraints." + collision_detail,
                started,
                state,
                min_distance=collision_before,
            )
        if not np.all(np.isfinite(solution)):
            return self._failure(
                DualArmIKStatus.SOLVER_ERROR,
                "The QP returned a non-finite solution.",
                started,
                state,
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
                    min_distance=min_distance,
                )

        active_limits = self._active_bound_names(solution, lb, ub, bound_names, bound_parts)
        elapsed = time.perf_counter() - started
        if elapsed > self._config.compute_budget_s:
            return self._failure(
                DualArmIKStatus.DEADLINE_EXCEEDED,
                f"The solve took {elapsed * 1e3:.1f} ms, over its "
                f"{self._config.compute_budget_s * 1e3:.1f} ms budget.",
                started,
                state,
                min_distance=min_distance,
            )

        self._previous_step = step_active
        errors = self._task_errors(configuration, target)
        positions_next = self._model.positions_from_q(q_next)
        targets = {name: positions_next[name] for name in self._active_joints}
        velocities = {
            name: float(step_active[i] / dt) for i, name in enumerate(self._active_joints)
        }

        converged = self._is_converged(errors, target)
        return DualArmIKResult(
            status=DualArmIKStatus.CONVERGED if converged else DualArmIKStatus.TRACKING,
            joint_targets_rad=targets,
            joint_velocities_rad_s=velocities,
            left_position_error_m=errors.get("left_position"),
            left_orientation_error_rad=errors.get("left_orientation"),
            right_position_error_m=errors.get("right_position"),
            right_orientation_error_rad=errors.get("right_orientation"),
            left_hold_residual_m=errors.get("left_hold"),
            right_hold_residual_m=errors.get("right_hold"),
            active_limits=tuple(active_limits),
            min_collision_distance_m=min_distance,
            compute_time_s=elapsed,
            torso_velocity_rad_s=float(step_active[self._torso_slot] / dt),
            message="",
            generation=state.mode_generation,
        )

    # -- internals ----------------------------------------------------------

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

    def _update_task_targets(self, configuration: Any, target: DualArmTarget) -> List[Any]:
        """Return only the Cartesian tasks requested by this target."""
        pin = self._pin
        tasks: List[Any] = []
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
        return tasks

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
            if not (lo <= positions[name] <= hi):
                raise ValueError(f"Held joint '{name}' already violates the limits.")
        lower, upper = self._model.position_limits(names)
        current = np.asarray([positions[name] for name in names])
        lb_position = lower + cfg.position_limit_margin_rad - current
        ub_position = upper - cfg.position_limit_margin_rad - current
        lb_hard = np.maximum(lb_speed, lb_position)
        ub_hard = np.minimum(ub_speed, ub_position)

        components: Dict[str, Any] = {
            "lb_speed": lb_speed,
            "ub_speed": ub_speed,
            "lb_position": lb_position,
            "ub_position": ub_position,
        }

        if self._previous_step is None:
            lb, ub = lb_hard, ub_hard
        else:
            # The acceleration bound is a smoothness constraint, not a safety
            # one. Intersecting it with the hard bounds can empty the feasible
            # set exactly when a joint approaching its position limit needs to
            # decelerate -- "you may not slow down" is never the right answer.
            # So the acceleration window is *clipped into* the hard window
            # instead: when it lies entirely outside, the joint brakes as hard
            # as the hard bounds allow, and the set stays non-empty unless the
            # configuration genuinely violates a position limit.
            previous = np.asarray([self._previous_step[slot] for slot in free_slots])
            lb_acceleration = previous - a_max * dt * dt
            ub_acceleration = previous + a_max * dt * dt
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

    def _task_errors(self, configuration: Any, target: DualArmTarget) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for side, task, enabled in (
            ("left", self._left_task, target.left_enabled),
            ("right", self._right_task, target.right_enabled),
        ):
            if not enabled:
                if target.inactive_arm_policy is InactiveArmPolicy.HOLD_JOINTS:
                    continue
                task = self._hold_tasks[side]
            error = task.compute_error(configuration)
            position = float(np.linalg.norm(error[:3]))
            orientation = float(np.linalg.norm(error[3:]))
            if enabled:
                out[f"{side}_position"] = position
                out[f"{side}_orientation"] = orientation
            else:
                out[f"{side}_hold"] = position
        return out

    def _is_converged(self, errors: Mapping[str, float], target: DualArmTarget) -> bool:
        cfg = self._config
        for side, enabled in (("left", target.left_enabled), ("right", target.right_enabled)):
            if not enabled:
                continue
            if errors.get(f"{side}_position", np.inf) > cfg.position_tolerance_m:
                return False
            # With the orientation weight at zero the orientation is not a goal,
            # so it cannot be a reason to withhold "converged".
            if (
                self._orientation_cost > 0.0
                and errors.get(f"{side}_orientation", np.inf) > cfg.orientation_tolerance_rad
            ):
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
        *,
        min_distance: float | None = None,
    ) -> DualArmIKResult:
        """Build a result that carries no motion command, only a diagnosis."""
        self._previous_step = None
        return DualArmIKResult(
            status=status,
            compute_time_s=time.perf_counter() - started,
            min_collision_distance_m=min_distance,
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
