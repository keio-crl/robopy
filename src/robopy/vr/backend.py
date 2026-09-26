"""Where the teleoperation commands go: the simulated model or the machine.

Both backends take the same :class:`TeleopCommand` -- head joint targets,
an arm :class:`~robopy.control.types.DualArmTarget`, gripper targets -- and
report the resulting joint configuration, so the page and the tests do not
care which one is behind the server.

:class:`SimulationBackend` integrates the dual-arm solver itself, one
differential step per pose sample, and keeps the joint state in memory.
:class:`ControlSystemBackend` only *sets targets* on a running
:class:`~robopy.robots.rakuda.rakuda_control.RakudaControlSystem`; the servo
loop on its own thread does the reading, solving and writing, with its own
freshness and generation checks.  Neither backend invents a measurement: the
hardware backend refuses to be built unless the control system is in
Cartesian teleoperation mode with the head motors mapped to URDF joints.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Protocol

import numpy as np
from numpy.typing import NDArray

from robopy.control.types import DualArmTarget, JointState, monotonic_ns

__all__ = [
    "BackendReport",
    "ControlSystemBackend",
    "SimulationBackend",
    "TeleopBackend",
    "TeleopCommand",
]


@dataclass(frozen=True)
class TeleopCommand:
    """Everything one pose sample asks the robot to do.

    Attributes:
        head_targets_rad: ``{urdf_joint: rad}`` for the head.
        arm_target: The arms' Cartesian target (hands not clutched are held),
            or ``None`` to leave the arms alone entirely.
        gripper_targets_rad: ``{follower_motor: rad}``; empty when the gripper
            travel is unmeasured.
        stamp_s: Monotonic time of the sample.
    """

    head_targets_rad: Mapping[str, float] = field(default_factory=dict)
    arm_target: DualArmTarget | None = None
    gripper_targets_rad: Mapping[str, float] = field(default_factory=dict)
    stamp_s: float = 0.0


@dataclass
class BackendReport:
    """What happened when a command was applied.

    Attributes:
        joints: ``{urdf_joint: rad}`` after the step (simulated) or as last
            measured (hardware).
        ik_status: Solver status name, or ``None`` when no arm was driven.
        ik_message: Solver diagnosis.
        ik_commandable: Whether the solver produced a command.
        errors: Per-hand position/orientation errors reported by the solver.
        hand_poses: ``{"left"|"right": (4, 4)}`` at ``joints``.
        gripper_positions_rad: Gripper angles as commanded (simulation) or
            measured (hardware), keyed by motor.
        compute_ms: Time spent in the solver.
        warnings: Anything the operator should see.
    """

    joints: Dict[str, float]
    ik_status: str | None = None
    ik_message: str = ""
    ik_commandable: bool | None = None
    errors: Dict[str, float | None] = field(default_factory=dict)
    hand_poses: Dict[str, NDArray[np.float64]] = field(default_factory=dict)
    gripper_positions_rad: Dict[str, float] = field(default_factory=dict)
    compute_ms: float = 0.0
    warnings: list[str] = field(default_factory=list)


class TeleopBackend(Protocol):
    """The common face of the simulated and hardware backends."""

    @property
    def name(self) -> str:
        """``"simulation"`` or ``"hardware"``."""
        ...

    @property
    def model(self) -> Any:
        """The :class:`~robopy.kinematics.urdf_model.WholeBodyModel`."""
        ...

    def joint_positions(self) -> Dict[str, float]:
        """Current ``{urdf_joint: rad}`` (every movable joint)."""
        ...

    def hand_pose(self, side: str) -> NDArray[np.float64]:
        """``(4, 4)`` pose of the left/right TCP at the current configuration."""
        ...

    def apply(self, command: TeleopCommand) -> BackendReport:
        """Apply one command."""
        ...

    def hold(self) -> None:
        """Stop driving: arms hold, head stays; called when the stream ends."""
        ...

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly status."""
        ...


def _joint_state(names: tuple[str, ...], positions: Mapping[str, float]) -> JointState:
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


def _errors_of(result: Any) -> Dict[str, float | None]:
    errors = result.errors()
    return {
        key: errors[key]
        for key in (
            "left_position_m",
            "left_orientation_rad",
            "left_axis_rad",
            "right_position_m",
            "right_orientation_rad",
            "right_axis_rad",
        )
    }


class SimulationBackend:
    """Drive the kinematic model in memory -- no hardware anywhere.

    Args:
        bundle: The loaded model (as the viewer uses it).
        ik: The solver bound to the bundle, or ``None`` for a head-only session.
        control_period_s: Nominal time between pose samples; the solver step
            size when a sample carries no usable timestamp.
        ik_substeps: Differential steps per applied command.  More substeps
            track a fast hand more closely at the cost of solver time.
        initial_positions: Starting configuration; zeros otherwise.
    """

    name = "simulation"

    def __init__(
        self,
        bundle: Any,
        ik: Any | None,
        *,
        control_period_s: float = 1.0 / 60.0,
        ik_substeps: int = 2,
        initial_positions: Mapping[str, float] | None = None,
    ) -> None:
        if control_period_s <= 0.0:
            raise ValueError("control_period_s must be positive.")
        if ik_substeps < 1:
            raise ValueError("ik_substeps must be at least 1.")
        self._bundle = bundle
        self._ik = ik
        self._period = control_period_s
        self._substeps = ik_substeps
        self._names: tuple[str, ...] = tuple(bundle.joint_order)
        self._positions: Dict[str, float] = {name: 0.0 for name in self._names}
        if initial_positions:
            for name, value in initial_positions.items():
                if name in self._positions:
                    self._positions[name] = float(value)
        self._grippers: Dict[str, float] = {}
        self._last_enabled = {"left": False, "right": False}
        self._last_stamp: float | None = None
        self._last_report: BackendReport | None = None
        lower, upper = bundle.model.position_limits(self._names)
        self._limits = {
            name: (float(lower[i]), float(upper[i])) for i, name in enumerate(self._names)
        }

    @property
    def model(self) -> Any:
        """The kinematic model."""
        return self._bundle.model

    @property
    def bundle(self) -> Any:
        """The model bundle, for the page's geometry poses."""
        return self._bundle

    def joint_positions(self) -> Dict[str, float]:
        """The simulated configuration."""
        return dict(self._positions)

    def hand_pose(self, side: str) -> NDArray[np.float64]:
        """TCP pose at the simulated configuration."""
        frame = self._bundle.tcp_frames[side]
        return self._bundle.model.frame_pose(self._q(), frame)

    def _q(self) -> NDArray[np.float64]:
        return self._bundle.model.q_from_positions(self._positions)

    def apply(self, command: TeleopCommand) -> BackendReport:
        """Move the head directly and step the arm solver towards the target."""
        started = time.perf_counter()
        warnings: list[str] = []
        for joint, value in command.head_targets_rad.items():
            if joint not in self._positions:
                warnings.append(f"head joint '{joint}' is not in the model; ignored")
                continue
            lo, hi = self._limits[joint]
            clamped = float(value)
            if np.isfinite(lo) and np.isfinite(hi):
                clamped = min(hi, max(lo, clamped))
            self._positions[joint] = clamped
        self._grippers.update({k: float(v) for k, v in command.gripper_targets_rad.items()})

        report = BackendReport(joints={})
        target = command.arm_target
        if target is not None and self._ik is not None and not target.is_expired():
            solver = self._ik.solver
            for side, enabled in (("left", target.left_enabled), ("right", target.right_enabled)):
                if enabled and not self._last_enabled[side]:
                    # A clutch just engaged: regularise towards where the arm
                    # is now, not towards wherever the solver last saw it.
                    solver.reset(self._q())
                self._last_enabled[side] = enabled
            dt = self._period
            if self._last_stamp is not None and command.stamp_s > self._last_stamp:
                dt = min(0.1, command.stamp_s - self._last_stamp)
            sub_dt = dt / self._substeps
            result = None
            for _ in range(self._substeps):
                result = solver.solve_step(
                    _joint_state(self._names, self._positions), target, sub_dt
                )
                if not result.is_commandable:
                    break
                self._positions.update({k: float(v) for k, v in result.joint_targets_rad.items()})
            assert result is not None
            report.ik_status = result.status.value
            report.ik_message = result.message
            report.ik_commandable = result.is_commandable
            report.errors = _errors_of(result)
        elif target is not None and self._ik is None:
            warnings.append("no solver: the arms cannot be driven in this session")
        if command.stamp_s > 0.0:
            self._last_stamp = command.stamp_s

        report.joints = dict(self._positions)
        report.hand_poses = {
            side: self.hand_pose(side)
            for side in self._bundle.tcp_frames
            if side in ("left", "right")
        }
        report.gripper_positions_rad = dict(self._grippers)
        report.compute_ms = (time.perf_counter() - started) * 1e3
        report.warnings = warnings
        self._last_report = report
        return report

    def hold(self) -> None:
        """Nothing moves without a command; just forget the clutch state."""
        self._last_enabled = {"left": False, "right": False}
        self._last_stamp = None

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly status."""
        return {
            "name": self.name,
            "ik": self._ik is not None,
            "control_period_s": self._period,
            "ik_substeps": self._substeps,
            "gripper_positions_rad": dict(self._grippers),
        }


class ControlSystemBackend:
    """Hand the commands to a running :class:`RakudaControlSystem`.

    Args:
        system: A system built in ``cartesian_teleop`` mode.  It must have been
            configured and started (or be stepped by hand, as the tests do); this
            backend never starts or stops it.
        target_ttl_s: Validity of the direct (head/gripper) targets it sets.
    """

    name = "hardware"

    def __init__(self, system: Any, *, target_ttl_s: float = 0.25) -> None:
        from robopy.control.types import ControlMode  # noqa: PLC0415

        if system.mode is not ControlMode.CARTESIAN_TELEOP:
            raise ValueError(
                "VR teleoperation needs cartesian_teleop mode; the system is in "
                f"{system.mode.value}."
            )
        if target_ttl_s <= 0.0:
            raise ValueError("target_ttl_s must be positive.")
        self._system = system
        self._ttl = target_ttl_s
        joint_map = system.follower.joint_map
        self._motor_for_joint: Dict[str, str] = {}
        for motor in system.follower.motor_names:
            urdf_joint = joint_map[motor].urdf_joint
            if urdf_joint is not None:
                self._motor_for_joint[urdf_joint] = motor
        self._last_grippers: Dict[str, float] = {}

    @property
    def model(self) -> Any:
        """The control system's kinematic model."""
        return self._system.model

    @property
    def system(self) -> Any:
        """The wrapped control system."""
        return self._system

    def joint_positions(self) -> Dict[str, float]:
        """Latest measured follower positions, every model joint present."""
        positions = {name: 0.0 for name in self._system.model.movable_joint_names}
        positions.update(self._system.follower_positions_urdf())
        return positions

    def hand_pose(self, side: str) -> NDArray[np.float64]:
        """Measured TCP pose."""
        return self._system.hand_pose(side)

    def apply(self, command: TeleopCommand) -> BackendReport:
        """Set the arm target and the direct head/gripper targets."""
        started = time.perf_counter()
        warnings: list[str] = []
        direct: Dict[str, float] = {}
        for joint, value in command.head_targets_rad.items():
            motor = self._motor_for_joint.get(joint)
            if motor is None:
                warnings.append(
                    f"no follower motor is mapped to head joint '{joint}'; not commanded"
                )
                continue
            direct[motor] = float(value)
        direct.update({k: float(v) for k, v in command.gripper_targets_rad.items()})
        if command.arm_target is not None:
            self._system.set_target(command.arm_target)
        if direct:
            self._system.set_direct_targets(direct, ttl_s=self._ttl)
        self._last_grippers.update({k: float(v) for k, v in command.gripper_targets_rad.items()})

        report = BackendReport(joints=self.joint_positions())
        result = self._system.last_ik_result
        if result is not None:
            report.ik_status = result.status.value
            report.ik_message = result.message
            report.ik_commandable = result.is_commandable
            report.errors = _errors_of(result)
        report.hand_poses = {side: self.hand_pose(side) for side in ("left", "right")}
        report.gripper_positions_rad = dict(self._last_grippers)
        report.compute_ms = (time.perf_counter() - started) * 1e3
        report.warnings = warnings
        return report

    def hold(self) -> None:
        """Hold both hands (the solver's hold task) and let direct targets expire."""
        self._system.set_target(DualArmTarget(left_enabled=False, right_enabled=False))

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly status, including the control system's own report."""
        return {
            "name": self.name,
            "mode": self._system.mode.value,
            "state": self._system.manager.state.value,
            "motor_for_joint": dict(self._motor_for_joint),
            "target_ttl_s": self._ttl,
            "report": self._system.report(),
        }
