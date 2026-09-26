"""Wiring the Rakuda control stack together: config in, running servo loop out.

This module is the only place that knows how a :class:`RakudaControlConfig`
becomes a joint map, a kinematic model, a solver, a controller and a servo loop.
Everything it builds is testable on a simulated bus, and nothing it builds
commands current on real hardware unless the configuration explicitly says so
*and* the calibration has actually been measured.

The legacy position teleoperation path is untouched.  While a control system is
running it holds the command lease for each bus, and
:class:`~robopy.robots.rakuda.rakuda_pair_sys.RakudaPairSys` refuses its own
writes, so the old loop and the new one can never be writing at once.
"""

from __future__ import annotations

import logging
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.config.robot_config.rakuda_config import (
    RAKUDA_GRIPPER_MOTOR_NAMES,
    RAKUDA_HEAD_MOTOR_NAMES,
    RakudaControlConfig,
    RakudaJointCalibrationSpec,
    RakudaTcpSpec,
)
from robopy.control.bilateral import BilateralController, BilateralGains, GravityProvider
from robopy.control.joint_mapping import (
    JointCalibration,
    JointMap,
    JointMapError,
    ValidationLevel,
    check_torque_policy,
)
from robopy.control.mode_manager import ModeManager
from robopy.control.servo_loop import ArmServo, BusLike, ServoLoop, ServoLoopConfig
from robopy.control.types import (
    ControlMode,
    DualArmTarget,
    JointState,
    ServoState,
    monotonic_ns,
    se3_from_quat_xyzw,
)
from robopy.motor.dynamixel_control_table import XControlTable

logger = logging.getLogger(__name__)

__all__ = [
    "RakudaControlSystem",
    "build_arm_servo",
    "build_joint_map",
    "build_model_and_ik",
]


def build_joint_map(
    bus_motors: Mapping[str, Any],
    calibration: Mapping[str, RakudaJointCalibrationSpec],
    *,
    known_urdf_joints: Sequence[str] | None = None,
    side: str = "",
) -> JointMap:
    """Turn a bus's motor dictionary and a config calibration into a joint map.

    Motors with no calibration entry are still registered, with every measured
    field left as ``None``.  They are readable in raw terms but will not pass
    :meth:`JointMap.require`, so they cannot be driven until measured.

    Args:
        bus_motors: ``{motor_name: DynamixelMotor}`` from the bus.
        calibration: The configured per-motor calibration.
        known_urdf_joints: Movable joint names of the loaded model, used to
            reject a URDF joint name that does not exist or is a fixed joint.
        side: ``"leader"`` or ``"follower"``, used in error messages.

    Returns:
        A validated :class:`JointMap`.

    Raises:
        JointMapError: On an unknown motor, a duplicate, or a bad URDF name.
    """
    unknown = sorted(set(calibration) - set(bus_motors))
    if unknown:
        raise JointMapError(
            f"{side or 'joint map'}: calibration names motor(s) {unknown} that are not on the bus."
        )

    entries: List[JointCalibration] = []
    for name, motor in bus_motors.items():
        spec = calibration.get(name)
        if spec is None:
            entries.append(
                JointCalibration(
                    motor_name=name,
                    motor_id=int(motor.id),
                    model=str(motor.model_name),
                    counts_per_revolution=int(getattr(motor, "resolution", 4096)),
                    notes="no calibration entry in the configuration",
                )
            )
            continue
        entries.append(
            JointCalibration(
                motor_name=name,
                motor_id=int(motor.id),
                model=str(motor.model_name),
                counts_per_revolution=int(getattr(motor, "resolution", 4096)),
                urdf_joint=spec.urdf_joint,
                direction=spec.direction,
                zero_count=spec.zero_count,
                drive_mode=spec.drive_mode,
                homing_offset=spec.homing_offset,
                lower_limit_rad=spec.lower_limit_rad,
                upper_limit_rad=spec.upper_limit_rad,
                max_velocity_rad_s=spec.max_velocity_rad_s,
                max_acceleration_rad_s2=spec.max_acceleration_rad_s2,
                torque_constant_nm_per_a=spec.torque_constant_nm_per_a,
                current_limit_a=spec.current_limit_a,
                max_current_rate_a_s=spec.max_current_rate_a_s,
                validated=spec.validated,
                notes=spec.notes,
            )
        )
    return JointMap(entries, known_urdf_joints=known_urdf_joints)


def _tcp_transform(spec: RakudaTcpSpec) -> np.ndarray:
    return se3_from_quat_xyzw(spec.translation_m, spec.quaternion_xyzw)


def build_model_and_ik(config: RakudaControlConfig) -> Tuple[Any, Any]:
    """Load the kinematic model and build the dual-arm solver from ``config``.

    Imports of Pinocchio and Pink happen inside this function, so a project that
    never runs Cartesian teleoperation does not need the ``kinematics`` extra.

    Returns:
        ``(WholeBodyModel, DualArmIK)``.

    Raises:
        ValueError: If the model configuration is incomplete.
        MissingKinematicsExtra: If the optional extra is not installed.
    """
    from robopy.kinematics.dual_arm_ik import DualArmIK, DualArmIKConfig  # noqa: PLC0415
    from robopy.kinematics.urdf_model import WholeBodyModel  # noqa: PLC0415

    spec = config.model
    urdf_path = spec.urdf_path
    package_dirs: List[str] = list(spec.package_dirs)
    if not urdf_path:
        # No URDF configured: use the Rakuda model that ships with robopy.
        from robopy.models import find_rakuda_model  # noqa: PLC0415

        bundled = find_rakuda_model()
        if bundled is None:
            raise ValueError(
                "control.model.urdf_path is not set and the bundled Rakuda model was not found."
            )
        urdf_path = str(bundled.convex_collision_urdf)
        package_dirs = package_dirs or [str(d) for d in bundled.package_dirs]
        logger.info(
            "control.model.urdf_path is unset; using the bundled Rakuda model %s", urdf_path
        )
    if spec.left_tcp is None or spec.right_tcp is None:
        raise ValueError("control.model.left_tcp and right_tcp must both be defined.")
    if not spec.torso_joint:
        raise ValueError("control.model.torso_joint is not set.")

    model = WholeBodyModel.from_urdf(
        urdf_path,
        package_dirs=package_dirs,
        build_collision=spec.build_collision,
        geometry_only=spec.geometry_only,
    )
    model.add_fixed_frame("left_tcp", spec.left_tcp.parent_frame, _tcp_transform(spec.left_tcp))
    model.add_fixed_frame("right_tcp", spec.right_tcp.parent_frame, _tcp_transform(spec.right_tcp))
    if spec.soft_limits_rad:
        model.set_soft_limits(spec.soft_limits_rad)

    if spec.build_collision:
        excluded = [(a, b) for a, b, _reason in spec.collision_exclusions]
        registered = model.add_all_collision_pairs(excluded=excluded)
        logger.info(
            "Registered %d self-collision pairs (%d excluded by recorded decision).",
            registered,
            len(excluded),
        )

    unvalidated = [
        name
        for name, tcp in (("left_tcp", spec.left_tcp), ("right_tcp", spec.right_tcp))
        if not tcp.validated
    ]
    if unvalidated:
        logger.warning(
            "TCP frame(s) %s are configured but marked validated=false. The offset from the "
            "gripper frame to the grasp centre is a measurement, not an assumption; Cartesian "
            "accuracy is unverified until it is made.",
            unvalidated,
        )

    ik = DualArmIK(
        model,
        left_frame="left_tcp",
        right_frame="right_tcp",
        torso_joint=spec.torso_joint,
        left_arm_joints=spec.left_arm_joints,
        right_arm_joints=spec.right_arm_joints,
        head_joints=spec.head_joints,
        config=DualArmIKConfig(
            compute_budget_s=config.ik_period_s,
            max_state_age_s=config.max_state_age_s,
        ),
    )
    return model, ik


def build_arm_servo(
    name: str,
    bus: BusLike,
    joint_map: JointMap,
    manager: ModeManager,
    config: RakudaControlConfig,
    *,
    motor_names: Sequence[str] | None = None,
) -> ArmServo:
    """Create the servo that owns ``bus``, using the periods from ``config``."""
    return ArmServo(
        name=name,
        bus=bus,
        joint_map=joint_map,
        manager=manager,
        config=ServoLoopConfig(
            control_period_s=config.control_period_s,
            read_timeout_s=config.read_timeout_s,
            write_timeout_s=config.write_timeout_s,
            max_state_age_s=config.max_state_age_s,
            max_command_age_s=config.max_command_age_s,
            diagnostics_period_s=config.diagnostics_period_s,
            max_cross_bus_skew_s=config.max_cross_bus_skew_s,
            stop_policy=config.stop_policy,
            bus_watchdog_counts=config.bus_watchdog_counts,
        ),
        motor_names=motor_names,
    )


class RakudaControlSystem:
    """A configured, runnable Rakuda control stack.

    Build it with :meth:`from_buses`, then :meth:`configure` and :meth:`start`.
    The system owns the command leases for both buses while it runs.
    """

    def __init__(
        self,
        config: RakudaControlConfig,
        leader_servo: ArmServo,
        follower_servo: ArmServo,
        manager: ModeManager,
        *,
        leader_gravity: GravityProvider | None = None,
        follower_gravity: GravityProvider | None = None,
    ) -> None:
        """Assemble the system from already-built servos.

        Args:
            config: The control configuration.
            leader_servo: Servo owning the leader bus.
            follower_servo: Servo owning the follower bus.
            manager: The shared mode manager.
            leader_gravity: Validated leader gravity model, if any.
            follower_gravity: Validated follower gravity model, if any.
        """
        self._config = config
        self._leader = leader_servo
        self._follower = follower_servo
        self._manager = manager
        self._mode = ControlMode(config.mode)

        self._model: Any = None
        self._ik: Any = None
        self._bilateral: BilateralController | None = None
        self._target: DualArmTarget | None = None
        self._last_ik_result: Any = None
        self._ik_targets: Dict[str, float] = {}
        self._direct_targets: Dict[str, float] = {}
        self._direct_expiry_ns: int | None = None
        self._direct_counts: Dict[str, int] = {}
        self._direct_counts_expiry_ns: int | None = None
        self._direct_count_writes = 0
        self._cycle = 0

        if self._mode is ControlMode.BILATERAL_JOINT:
            coupled = list(config.bilateral.coupled_motors)
            self._bilateral = BilateralController(
                coupled,
                BilateralGains(
                    stiffness_nm_per_rad=config.bilateral.stiffness_nm_per_rad,
                    damping_nm_s_per_rad=config.bilateral.damping_nm_s_per_rad,
                    scale=config.bilateral.scale,
                    offset_rad=config.bilateral.offset_rad,
                    max_torque_nm=config.bilateral.max_torque_nm,
                    max_torque_rate_nm_s=config.bilateral.max_torque_rate_nm_s,
                    velocity_filter_hz=config.bilateral.velocity_filter_hz,
                    ramp_time_s=config.bilateral.ramp_time_s,
                ),
                leader_gravity=leader_gravity,
                follower_gravity=follower_gravity,
                allow_uncompensated=config.bilateral.allow_uncompensated,
            )

        self._loop = ServoLoop(
            (self._leader, self._follower),
            manager,
            self._step,
            ServoLoopConfig(
                control_period_s=config.control_period_s,
                max_cross_bus_skew_s=config.max_cross_bus_skew_s,
                stop_policy=config.stop_policy,
            ),
        )

    # -- construction -------------------------------------------------------

    @classmethod
    def from_buses(
        cls,
        config: RakudaControlConfig,
        leader_bus: BusLike,
        follower_bus: BusLike,
        *,
        leader_torque_enabled: Sequence[str] | None = None,
        follower_torque_enabled: Sequence[str] | None = None,
        leader_gravity: GravityProvider | None = None,
        follower_gravity: GravityProvider | None = None,
        manager: ModeManager | None = None,
    ) -> "RakudaControlSystem":
        """Build a whole system from two buses and a configuration.

        Args:
            config: The control configuration.
            leader_bus: The leader's bus.  A real ``DynamixelBus`` or a
                ``SimulatedDynamixelBus`` -- the stack does not distinguish.
            follower_bus: The follower's bus.
            leader_torque_enabled: Effective leader torque policy, checked
                against the bilateral coupling set.
            follower_torque_enabled: Effective follower torque policy.
            leader_gravity: Validated leader gravity model, if any.
            follower_gravity: Validated follower gravity model, if any.
            manager: An existing mode manager, or ``None`` to create one.

        Returns:
            The assembled system.

        Raises:
            ValueError: If the configuration is inconsistent with the buses.
        """
        mode = ControlMode(config.mode)
        manager = manager or ModeManager(mode)

        model = None
        ik = None
        known_joints: Sequence[str] | None = None
        if mode is ControlMode.CARTESIAN_TELEOP:
            model, ik = build_model_and_ik(config)
            known_joints = model.movable_joint_names

        leader_map = build_joint_map(
            leader_bus.motors,
            config.leader_joint_calibration,
            known_urdf_joints=known_joints,
            side="leader",
        )
        follower_map = build_joint_map(
            follower_bus.motors,
            config.follower_joint_calibration,
            known_urdf_joints=known_joints,
            side="follower",
        )

        if mode is ControlMode.BILATERAL_JOINT:
            coupled = config.bilateral.coupled_motors
            check_torque_policy(
                coupled,
                list(leader_torque_enabled) if leader_torque_enabled is not None else None,
                all_motor_names=list(leader_bus.motors),
                side="leader",
            )
            check_torque_policy(
                coupled,
                list(follower_torque_enabled) if follower_torque_enabled is not None else None,
                all_motor_names=list(follower_bus.motors),
                side="follower",
            )
            forbidden = sorted(
                set(coupled) & (set(RAKUDA_GRIPPER_MOTOR_NAMES) | set(RAKUDA_HEAD_MOTOR_NAMES))
            )
            if forbidden:
                raise ValueError(
                    f"Bilateral coupling must not include {forbidden}; the head and the grippers "
                    "follow a separate hold/disable policy."
                )

        leader_motors = cls._motors_for_mode(mode, config, list(leader_bus.motors))
        follower_motors = cls._motors_for_mode(mode, config, list(follower_bus.motors))

        system = cls(
            config,
            build_arm_servo(
                "leader", leader_bus, leader_map, manager, config, motor_names=leader_motors
            ),
            build_arm_servo(
                "follower",
                follower_bus,
                follower_map,
                manager,
                config,
                motor_names=follower_motors,
            ),
            manager,
            leader_gravity=leader_gravity,
            follower_gravity=follower_gravity,
        )
        system._model = model
        system._ik = ik
        return system

    @staticmethod
    def _motors_for_mode(
        mode: ControlMode,
        config: RakudaControlConfig,
        all_motors: Sequence[str],
    ) -> List[str]:
        """Motors the servo reads and commands in ``mode``."""
        if mode is ControlMode.BILATERAL_JOINT:
            return list(config.bilateral.coupled_motors)
        return list(all_motors)

    # -- properties ---------------------------------------------------------

    @property
    def mode(self) -> ControlMode:
        """The control mode this system was built for."""
        return self._mode

    @property
    def manager(self) -> ModeManager:
        """The mode manager."""
        return self._manager

    @property
    def loop(self) -> ServoLoop:
        """The servo loop."""
        return self._loop

    @property
    def leader(self) -> ArmServo:
        """The leader servo."""
        return self._leader

    @property
    def follower(self) -> ArmServo:
        """The follower servo."""
        return self._follower

    @property
    def ik(self) -> Any:
        """The dual-arm solver, or ``None`` outside Cartesian mode."""
        return self._ik

    @property
    def model(self) -> Any:
        """The kinematic model, or ``None`` outside Cartesian mode."""
        return self._model

    @property
    def bilateral(self) -> BilateralController | None:
        """The coupling controller, or ``None`` outside bilateral mode."""
        return self._bilateral

    @property
    def last_ik_result(self) -> Any:
        """The most recent IK result, for diagnostics."""
        return self._last_ik_result

    # -- lifecycle ----------------------------------------------------------

    def configure(self, *, allow_hardware_current_output: bool | None = None) -> None:
        """Put both buses into the register state this mode needs.

        Args:
            allow_hardware_current_output: Override the configuration's master
                switch for this call.

        Raises:
            PermissionError: If the mode commands current but the configuration
                has not enabled hardware current output.
            JointMapError: If the calibration is not complete enough.
        """
        allowed = (
            self._config.allow_hardware_current_output
            if allow_hardware_current_output is None
            else allow_hardware_current_output
        )
        if self._mode is ControlMode.BILATERAL_JOINT and not allowed:
            raise PermissionError(
                "Bilateral joint control commands motor current, but "
                "control.allow_hardware_current_output is false. It stays false until the "
                "per-joint zero points, travel limits, torque constants and current limits have "
                "been measured on this machine and marked validated: true. Set it deliberately, "
                "not to get past this message."
            )

        self._manager.transition(ServoState.CONFIGURING)
        current_limits_leader = dict(self._config.bilateral.leader_current_limit_a) or None
        current_limits_follower = dict(self._config.bilateral.follower_current_limit_a) or None
        if self._mode is not ControlMode.BILATERAL_JOINT:
            current_limits_leader = None
            current_limits_follower = None

        if self._mode is ControlMode.BILATERAL_JOINT:
            self._leader.configure_for_mode(self._mode, current_limits_a=current_limits_leader)
        self._follower.configure_for_mode(self._mode, current_limits_a=current_limits_follower)
        self._manager.transition(ServoState.READY)

    def align(self) -> Dict[str, Any]:
        """Check that the two machines are aligned before the coupling engages.

        Returns:
            A report with the alignment error per coupled joint.

        Raises:
            RuntimeError: If the position error exceeds
                :attr:`RakudaBilateralConfig.max_alignment_error_rad`, which
                would make engaging produce an immediate spring torque step.
        """
        self._manager.transition(ServoState.ALIGNING)

        report: Dict[str, Any] = {"mode": self._mode.value}
        if self._bilateral is not None:
            leader_state = self._leader.read_state()
            follower_state = self._follower.read_state()
            error = self._bilateral.alignment_error_rad(leader_state, follower_state)
            worst = float(np.max(np.abs(error))) if error.size else 0.0
            report["alignment_error_rad"] = {
                name: float(error[i]) for i, name in enumerate(self._bilateral.joint_names)
            }
            report["worst_alignment_error_rad"] = worst
            limit = self._config.bilateral.max_alignment_error_rad
            if worst > limit:
                self._manager.transition(ServoState.STOPPING)
                self._manager.transition(ServoState.READY)
                raise RuntimeError(
                    f"The arms are {worst:.4f} rad out of alignment, over the {limit:.4f} rad "
                    "limit. Engaging now would apply a large spring torque immediately: align "
                    "the machines, or re-baseline the offset with the feedback released."
                )
            self._bilateral.reset()
            self._bilateral.engage()
        elif self._ik is not None:
            state = self._state_in_urdf_joints(self._follower.read_state())
            self._ik.reset()
            self._ik.set_posture_reference(state.positions_dict())
            report["posture_reference_set"] = True
        return report

    def set_target(self, target: DualArmTarget) -> None:
        """Set the Cartesian target the IK worker will track."""
        if self._mode is not ControlMode.CARTESIAN_TELEOP:
            raise RuntimeError(f"A Cartesian target is meaningless in {self._mode.value} mode.")
        self._target = target

    def set_direct_targets(self, targets_rad: Mapping[str, float], *, ttl_s: float = 0.25) -> None:
        """Position targets for follower motors the arm IK does not drive.

        The head joints and the grippers are commanded this way (by the VR
        teleoperation, for instance) while the arms follow the Cartesian target.
        The targets expire after ``ttl_s`` so a stalled producer stops moving
        the head rather than freezing a stale command in place.

        Args:
            targets_rad: ``{follower_motor_name: radians}``.
            ttl_s: How long the targets stay valid.

        Raises:
            RuntimeError: Outside Cartesian teleoperation.
            ValueError: For a motor the servo does not own, a motor whose URDF
                joint the IK drives, or a non-finite target.
        """
        if self._mode is not ControlMode.CARTESIAN_TELEOP:
            raise RuntimeError(
                f"Direct targets are only used in {ControlMode.CARTESIAN_TELEOP.value} mode."
            )
        if ttl_s <= 0.0:
            raise ValueError("ttl_s must be positive.")
        active = set(self._ik.active_joints) if self._ik is not None else set()
        clean: Dict[str, float] = {}
        for motor, angle in targets_rad.items():
            if motor not in self._follower.motor_names:
                raise ValueError(f"'{motor}' is not a follower motor.")
            urdf_joint = self._follower.joint_map[motor].urdf_joint
            if urdf_joint in active:
                raise ValueError(
                    f"'{motor}' drives URDF joint '{urdf_joint}', which belongs to the arm IK; "
                    "it cannot also be commanded directly."
                )
            if not np.isfinite(angle):
                raise ValueError(f"'{motor}': non-finite target.")
            clean[motor] = float(angle)
        self._direct_targets = clean
        self._direct_expiry_ns = monotonic_ns() + int(ttl_s * 1e9)

    def set_direct_goal_counts(self, goals: Mapping[str, int], *, ttl_s: float = 0.25) -> None:
        """Goal positions, in encoder counts, for follower motors the servo does not command.

        In bilateral joint control the servo owns only the coupled motors; the
        head (and the grippers) stay in position mode with whatever torque the
        follower was connected with.  This is how the VR teleoperation moves
        the head while the arms are coupled: the goals are written by the
        control loop itself, in its cycle, so no second thread ever touches
        the bus.  They are raw counts on purpose -- the head's zero point is
        not part of the bilateral calibration -- and expire like the other
        direct targets.

        Args:
            goals: ``{follower_motor_name: count}``.
            ttl_s: How long the goals stay valid.

        Raises:
            RuntimeError: Outside bilateral joint control.
            ValueError: For a motor that is not on the follower bus, one the
                servo commands itself, or a non-finite count.
        """
        if self._mode is not ControlMode.BILATERAL_JOINT:
            raise RuntimeError(
                f"Direct goal counts are only used in {ControlMode.BILATERAL_JOINT.value} mode."
            )
        if ttl_s <= 0.0:
            raise ValueError("ttl_s must be positive.")
        owned = set(self._follower.motor_names)
        clean: Dict[str, int] = {}
        for motor, count in goals.items():
            if motor not in self._follower.bus.motors:
                raise ValueError(f"'{motor}' is not a follower motor.")
            if motor in owned:
                raise ValueError(
                    f"'{motor}' is commanded by the bilateral coupling; it cannot also be "
                    "given a goal position."
                )
            if not np.isfinite(count):
                raise ValueError(f"'{motor}': non-finite goal count.")
            clean[motor] = int(round(float(count)))
        self._direct_counts = clean
        self._direct_counts_expiry_ns = monotonic_ns() + int(ttl_s * 1e9)

    def follower_positions_urdf(self) -> Dict[str, float]:
        """Latest follower positions keyed by URDF joint (reads the bus if needed)."""
        state = self._follower.latest_state()
        if state is None:
            state = self._follower.read_state()
        return self._state_in_urdf_joints(state).positions_dict()

    def hand_pose(self, side: str) -> NDArray[np.float64]:
        """``(4, 4)`` pose of the ``left``/``right`` TCP at the latest follower state."""
        if self._model is None:
            raise RuntimeError("No kinematic model: hand poses need cartesian_teleop mode.")
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'.")
        positions = {name: 0.0 for name in self._model.movable_joint_names}
        positions.update(self.follower_positions_urdf())
        return self._model.frame_pose(self._model.q_from_positions(positions), f"{side}_tcp")

    def prepare_running(self) -> None:
        """Enter ``RUNNING`` and take the command lease for each bus.

        The leases are taken *after* the transition, because every transition
        invalidates every open lease -- that is what stops a command computed
        before a mode change from being delivered after it.  Call this instead
        of :meth:`start` when driving :meth:`ServoLoop.run_once` by hand, as the
        tests and the simulated examples do.
        """
        self._manager.transition(ServoState.RUNNING)
        self._leader.acquire_command_path()
        self._follower.acquire_command_path()

    def start(self) -> None:
        """Begin running the control loop in its own thread."""
        self.prepare_running()
        self._loop.start()

    def stop(self) -> List[str]:
        """Stop the loop and apply each servo's stop policy.

        Returns:
            What each servo did, for the record.
        """
        self._loop.stop()
        if self._manager.state is not ServoState.FAULT:
            self._manager.transition(ServoState.STOPPING)
        results: List[str] = []
        for servo in (self._leader, self._follower):
            try:
                results.append(servo.stop())
            except Exception as exc:  # noqa: BLE001 - a dead bus cannot be written to
                results.append(f"{servo.name}: stop command failed ({exc}).")
            servo.release_command_path()
        if self._bilateral is not None:
            self._bilateral.reset()
        if self._manager.state is ServoState.STOPPING:
            self._manager.transition(ServoState.READY)
        return results

    # -- the control cycle --------------------------------------------------

    def _step(self, states: Mapping[str, JointState], dt: float) -> None:
        """One control cycle, dispatched by mode."""
        self._cycle += 1
        if self._mode is ControlMode.BILATERAL_JOINT:
            self._step_bilateral(states, dt)
        elif self._mode is ControlMode.CARTESIAN_TELEOP:
            self._step_cartesian(states, dt)
        else:
            self._step_position(states)

    def _step_position(self, states: Mapping[str, JointState]) -> None:
        leader = states["leader"]
        targets = {
            name: float(leader.position_rad[i])
            for i, name in enumerate(leader.joint_names)
            if name in self._follower.motor_names
        }
        self._follower.command_positions_rad(
            targets, generation=leader.mode_generation, issued_ns=monotonic_ns()
        )

    def _step_cartesian(self, states: Mapping[str, JointState], dt: float) -> None:
        motor_targets: Dict[str, float] = {}
        generation = states["follower"].mode_generation
        target = self._target
        if target is not None and not target.is_expired():
            result = self._ik.solve_step(self._state_in_urdf_joints(states["follower"]), target, dt)
            self._last_ik_result = result
            self._loop.publish_log(
                {
                    "cycle": self._cycle,
                    "ik_status": result.status.value,
                    "compute_time_s": result.compute_time_s,
                    "min_collision_distance_m": result.min_collision_distance_m,
                }
            )
            if result.is_commandable:
                self._ik_targets = dict(result.joint_targets_rad)
                motor_targets.update(self._urdf_targets_to_motors(result.joint_targets_rad))
                generation = result.generation
            else:
                # An invalid result is never issued as new motion; the servo
                # keeps its previous goal and the diagnosis is logged.
                logger.warning(
                    "IK produced no command: %s (%s)", result.status.value, result.message
                )
        # No fresh Cartesian target: the arms hold their last valid joint
        # targets rather than moving on a stale one.  The head and grippers
        # are independent of that and follow their own (also expiring) targets.
        if self._direct_targets and self._direct_expiry_ns is not None:
            if monotonic_ns() <= self._direct_expiry_ns:
                motor_targets.update(self._direct_targets)
        if motor_targets:
            self._follower.command_positions_rad(
                motor_targets, generation=generation, issued_ns=monotonic_ns()
            )

    def _state_in_urdf_joints(self, state: JointState) -> JointState:
        """Re-key a measured snapshot from motor names to URDF joint names.

        The servo reads motors; the solver reasons about URDF joints.  The
        translation is the configured correspondence, not a name match, and
        motors with no URDF counterpart -- the grippers -- are dropped rather
        than invented as joints of the model.
        """
        joint_map = self._follower.joint_map
        pairs: List[Tuple[int, str]] = []
        for index, motor in enumerate(state.joint_names):
            urdf_joint = joint_map[motor].urdf_joint
            if urdf_joint is not None:
                pairs.append((index, urdf_joint))
        if not pairs:
            raise JointMapError(
                "No follower motor is mapped to a URDF joint, so no configuration can be built. "
                "Set `urdf_joint` in control.follower_joint_calibration."
            )
        indices = np.asarray([index for index, _ in pairs], dtype=int)
        names = tuple(name for _, name in pairs)
        return JointState(
            joint_names=names,
            position_rad=state.position_rad[indices],
            velocity_rad_s=state.velocity_rad_s[indices],
            current_a=state.current_a[indices],
            valid=state.valid[indices],
            torque_estimate_nm=(
                None if state.torque_estimate_nm is None else state.torque_estimate_nm[indices]
            ),
            read_start_ns=state.read_start_ns,
            read_end_ns=state.read_end_ns,
            sequence=state.sequence,
            mode_generation=state.mode_generation,
        )

    def _urdf_targets_to_motors(self, targets_rad: Mapping[str, float]) -> Dict[str, float]:
        """Map URDF joint targets onto follower motor names."""
        out: Dict[str, float] = {}
        for urdf_joint, angle in targets_rad.items():
            if not self._follower.joint_map.has_urdf_joint(urdf_joint):
                raise JointMapError(
                    f"No follower motor is mapped to URDF joint '{urdf_joint}'. The mapping is "
                    "configuration, not something inferred from the joint name."
                )
            out[self._follower.joint_map.motor_for_urdf_joint(urdf_joint).motor_name] = float(angle)
        return out

    def _step_bilateral(self, states: Mapping[str, JointState], dt: float) -> None:
        assert self._bilateral is not None
        output = self._bilateral.compute(states["leader"], states["follower"], dt)
        generation = min(states["leader"].mode_generation, states["follower"].mode_generation)
        issued = monotonic_ns()
        self._follower.command_torques_nm(
            output.as_dict("follower"), generation=generation, issued_ns=issued
        )
        self._leader.command_torques_nm(
            output.as_dict("leader"), generation=generation, issued_ns=issued
        )
        # Motors outside the coupling (the head) follow their own goals, in
        # this thread, while those goals are fresh.
        if self._direct_counts and self._direct_counts_expiry_ns is not None:
            if monotonic_ns() <= self._direct_counts_expiry_ns:
                self._follower.bus.sync_write(
                    XControlTable.GOAL_POSITION, dict(self._direct_counts)
                )
                self._direct_count_writes += 1
        self._loop.publish_log(
            {
                "cycle": self._cycle,
                "dt_s": output.dt_s,
                "coupling_scale": output.coupling_scale,
                "max_abs_position_error_rad": float(np.max(np.abs(output.position_error_rad))),
                "stored_energy_j": self._bilateral.stored_energy_j(output),
                "limits_active": output.limits.any_active,
            }
        )

    # -- reporting ----------------------------------------------------------

    def report(self) -> Dict[str, Any]:
        """Timing, calibration completeness and fault state, for the record."""
        level = (
            ValidationLevel.HARDWARE
            if self._mode is ControlMode.BILATERAL_JOINT
            else ValidationLevel.GEOMETRY
        )
        return {
            "mode": self._mode.value,
            "state": self._manager.state.value,
            "generation": self._manager.generation,
            "faults": [
                {"reason": f.reason, "detail": f.detail, "source": f.source}
                for f in self._manager.faults
            ],
            "timing": self._loop.timing_report(),
            "calibration_gaps": {
                "leader": self._leader.joint_map.validation_report(level),
                "follower": self._follower.joint_map.validation_report(level),
            },
            "last_ik_status": (
                None if self._last_ik_result is None else self._last_ik_result.status.value
            ),
            "direct_goal_counts": {
                "goals": dict(self._direct_counts),
                "writes": self._direct_count_writes,
            },
        }
