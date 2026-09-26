import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.config.sensor_config.params_config import AudioParams, CameraParams, TactileParams
from robopy.config.sensor_config.visual_config.camera_config import RealsenseCameraConfig


@dataclass
class RakudaConfig:
    """Configuration class for Rakuda robot."""

    leader_port: str
    follower_port: str
    sensors: "RakudaSensorParams | None" = field(default=None)
    slow_mode: bool = False
    # Torque enable policy (joint names). If None, defaults preserve current behavior.
    # - leader_torque_enabled: default is grippers only
    # - follower_torque_enabled: default is all joints
    leader_torque_enabled: List[str] | None = None
    follower_torque_enabled: List[str] | None = None
    # Dual-arm IK and bilateral control. `None` keeps the legacy position
    # teleoperation behaviour and loads no kinematic model.
    control: "RakudaControlConfig | None" = None


@dataclass
class RakudaSensorParams:
    cameras: List[CameraParams] = field(default_factory=list)
    tactile: List[TactileParams] = field(default_factory=list)
    audio: List[AudioParams] = field(default_factory=list)


@dataclass
class RakudaSensorConfigs:
    cameras: List[RealsenseCameraConfig]
    tactile: List[TactileParams]
    audio: List[AudioParams]


@dataclass
class RakudaArmObs:
    leader: NDArray[np.float32]
    follower: NDArray[np.float32]


@dataclass
class RakudaSensorObs:
    cameras: Dict[str, NDArray[np.float32] | None]
    tactile: Dict[str, NDArray[np.float32] | None]
    audio: Dict[str, NDArray[np.float32] | None]


@dataclass
class RakudaObs:
    """
    Overall observation structure for Rakuda robot.
    arms: Observations from the robot arms (leader and follower).
    sensors: Observations from the sensors (cameras, tactile and audio).
    """

    arms: RakudaArmObs
    sensors: RakudaSensorObs | None


RAKUDA_MOTOR_MAPPING: Dict[str, str] = {
    "torso_yaw": "torso_yaw",
    "head_yaw": "head_yaw",
    "head_pitch": "head_pitch",
    "r_arm_sh_pitch1": "r_arm_sh_pitch1",
    "r_arm_sh_roll": "r_arm_sh_roll",
    "r_arm_sh_pitch2": "r_arm_sh_pitch2",
    "r_arm_el_yaw": "r_arm_el_yaw",
    "r_arm_wr_roll": "r_arm_wr_roll",
    "r_arm_wr_yaw": "r_arm_wr_yaw",
    "r_arm_grip": "r_arm_grip",
    "l_arm_sh_pitch1": "l_arm_sh_pitch1",
    "l_arm_sh_roll": "l_arm_sh_roll",
    "l_arm_sh_pitch2": "l_arm_sh_pitch2",
    "l_arm_el_yaw": "l_arm_el_yaw",
    "l_arm_wr_roll": "l_arm_wr_roll",
    "l_arm_wr_yaw": "l_arm_wr_yaw",
    "l_arm_grip": "l_arm_grip",
}


# Canonical Rakuda joint names (used for validation and config templates).
RAKUDA_JOINT_NAMES: Tuple[str, ...] = tuple(RAKUDA_MOTOR_MAPPING.keys())


#: Travel one Rakuda joint has under leader-follower position teleoperation.
#:
#: The leader and the follower run DYNAMIXEL position control over the whole
#: 0..4095 count range with the zero at 2048 (:class:`RakudaJointCalibrationSpec`
#: ``zero_count``), so every joint can be commanded ``+/-pi`` about that zero.
#: That is what the servos allow, **not** a measurement of where the machine
#: actually stops: a cable, a cover or a neighbouring link may end the travel
#: much sooner, and that number is a measurement which belongs in
#: ``.robopy/rakuda/config.yaml`` as a soft limit.  Anything that narrows the
#: range -- a measured soft limit, the URDF's own range -- still applies on top
#: of this.
RAKUDA_MOTOR_TRAVEL_RAD: Tuple[float, float] = (-math.pi, math.pi)


# --- Dual-arm IK and bilateral control configuration ---------------------------


@dataclass
class RakudaJointCalibrationSpec:
    """Per-motor calibration as it appears in the YAML config.

    Every numeric field defaults to ``None``, meaning "not measured".  Nothing
    here is filled in with a plausible-looking guess: an unmeasured value stays
    ``None`` and the affected feature refuses to run, which is the whole point
    of the distinction.

    Attributes:
        urdf_joint: Name of the movable URDF joint this motor drives.  The
            correspondence is *not* inferred from the motor name -- names like
            ``r_arm_sh_pitch2``, ``r_arm_el_yaw`` and ``r_arm_wr_roll`` do not
            map onto the URDF names by any mechanical rule -- so it has to be
            stated and checked on the machine.
        direction: ``+1`` or ``-1``, the joint axis sign relative to the motor.
        zero_count: ``PRESENT_POSITION`` count at zero radians.
        drive_mode: ``DRIVE_MODE`` the zero point was defined against.
        homing_offset: ``HOMING_OFFSET`` the zero point was defined against.
        lower_limit_rad: Measured lower travel limit.
        upper_limit_rad: Measured upper travel limit.
        max_velocity_rad_s: Measured velocity limit.
        max_acceleration_rad_s2: Measured acceleration limit.
        torque_constant_nm_per_a: Measured torque per ampere for this joint.
        current_limit_a: Current ceiling in amperes.
        max_current_rate_a_s: Ceiling on the rate of change of current.
        validated: Whether these numbers came from the actual machine.
        notes: Provenance note.
    """

    urdf_joint: str | None = None
    direction: int = 1
    zero_count: int | None = None
    drive_mode: int | None = None
    homing_offset: int | None = None
    lower_limit_rad: float | None = None
    upper_limit_rad: float | None = None
    max_velocity_rad_s: float | None = None
    max_acceleration_rad_s2: float | None = None
    torque_constant_nm_per_a: float | None = None
    current_limit_a: float | None = None
    max_current_rate_a_s: float | None = None
    validated: bool = False
    notes: str = ""


@dataclass
class RakudaTcpSpec:
    """A TCP frame defined as a fixed offset from an existing URDF frame.

    The Rakuda URDF's ``gripper_left_dof`` / ``gripper_right_dof`` frames are
    *fixed* joints; whether either coincides with the grasp centre is a
    measurable fact, not an assumption, so the offset is stated explicitly.

    Attributes:
        parent_frame: Existing frame the offset is measured from.
        translation_m: ``(x, y, z)`` offset in the parent frame, in metres.
        quaternion_xyzw: Rotation as ``(x, y, z, w)``.  The ``xyzw`` order is the
            only quaternion order used in robopy.
        validated: Whether the offset was measured on the machine.
    """

    parent_frame: str
    translation_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion_xyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    validated: bool = False


@dataclass
class RakudaSoftLimitSpec:
    """A soft limit as it appears in the YAML: a range that only narrows.

    Attributes:
        lower: Lower bound, radians, URDF joint coordinates.
        upper: Upper bound.
        validated: Whether the range was measured on the machine.  An
            unvalidated value is a simulation stand-in and is reported as one;
            it is never promoted to a machine limit.
        note: Where the number came from.
    """

    lower: float
    upper: float
    validated: bool = False
    note: str = ""


@dataclass
class RakudaLimitOverrideSpec:
    """A recorded replacement of a URDF range known to be wrong.

    Attributes:
        lower: Replacement lower bound, radians, URDF joint coordinates.
        upper: Replacement upper bound.
        reason: Why the exported range is not trusted.  Required: an override
            without a reason is a guess, and guesses are not configuration.
    """

    lower: float
    upper: float
    reason: str


@dataclass
class RakudaIKConfig:
    """The ``control.ik`` section: how the dual-arm solver behaves.

    Every field is optional; ``None`` keeps the solver's own default (see
    :class:`~robopy.kinematics.dual_arm_ik.DualArmIKConfig`).  The same section
    builds the viewer's solver and the machine's, so a behaviour tuned on the
    page is the behaviour the machine gets.

    Attributes:
        task_priority_mode: ``weighted`` (one weighted sum, the historical
            behaviour) or ``hierarchical`` (the hand tasks first, then the
            posture, limit-avoidance and smoothness objectives without
            degrading them).
        orientation_mode: ``position_only``, ``pose`` or ``axis_aligned``.
        approach_axis_tcp: The gripper's approach axis in TCP coordinates,
            needed by ``axis_aligned``.  Not assumed: state it.
        preferred_posture_rad: ``{joint: rad}`` the posture objective pulls
            towards.  ``None`` uses the configuration at alignment.
        posture_cost: Weight of the posture objective (scalar or per joint).
        joint_motion_cost: Per-joint cost on moving at all (scalar or per
            joint); the torso is usually given more than the arms.
        velocity_smoothing_cost: Cost on the change of joint velocity between
            cycles.
        limit_avoidance_enabled: Push joints that enter the band near a limit
            back inside.
        limit_avoidance_band_rad: Width of that band.
        limit_avoidance_cost: Its weight.
        gain_time_constant_s: Task error correction time constant; the per-step
            gain becomes ``1 - exp(-dt / tau)`` so the response does not change
            with the control period.
        torso_policy: Default torso policy name (``fixed`` / ``manual`` /
            ``optimize``) for consumers that do not choose one per target.
        inactive_arm_policy: ``hold_joints`` or ``hold_world``.
        damping: Tikhonov regularisation of every step.
        max_joint_velocity_rad_s: Per-joint or scalar velocity ceiling.
        max_joint_acceleration_rad_s2: Per-joint or scalar acceleration ceiling.
        max_joint_step_rad: Hard ceiling on one cycle's joint move.
        position_limit_margin_rad: Stay this far inside every limit.
        position_tolerance_m: Position error counted as converged.
        orientation_tolerance_rad: Orientation error counted as converged.
        solver: QP backend name (``qpsolvers``).
    """

    task_priority_mode: str | None = None
    orientation_mode: str | None = None
    approach_axis_tcp: Tuple[float, float, float] | None = None
    preferred_posture_rad: Dict[str, float] | None = None
    posture_cost: Dict[str, float] | float | None = None
    joint_motion_cost: Dict[str, float] | float | None = None
    velocity_smoothing_cost: Dict[str, float] | float | None = None
    limit_avoidance_enabled: bool | None = None
    limit_avoidance_band_rad: float | None = None
    limit_avoidance_cost: float | None = None
    gain_time_constant_s: float | None = None
    torso_policy: str | None = None
    inactive_arm_policy: str | None = None
    damping: float | None = None
    max_joint_velocity_rad_s: Dict[str, float] | float | None = None
    max_joint_acceleration_rad_s2: Dict[str, float] | float | None = None
    max_joint_step_rad: float | None = None
    position_limit_margin_rad: float | None = None
    position_tolerance_m: float | None = None
    orientation_tolerance_rad: float | None = None
    solver: str | None = None

    def solver_overrides(self) -> Dict[str, Any]:
        """The non-``None`` fields as keyword overrides for ``DualArmIKConfig``.

        ``preferred_posture_rad`` maps onto the solver's ``posture_reference``;
        ``torso_policy`` and ``inactive_arm_policy`` are consumer defaults, not
        solver fields, and are left out.
        """
        out: Dict[str, Any] = {}
        for name in (
            "task_priority_mode",
            "orientation_mode",
            "approach_axis_tcp",
            "posture_cost",
            "joint_motion_cost",
            "velocity_smoothing_cost",
            "limit_avoidance_enabled",
            "limit_avoidance_band_rad",
            "limit_avoidance_cost",
            "gain_time_constant_s",
            "damping",
            "max_joint_velocity_rad_s",
            "max_joint_acceleration_rad_s2",
            "max_joint_step_rad",
            "position_limit_margin_rad",
            "position_tolerance_m",
            "orientation_tolerance_rad",
            "solver",
        ):
            value = getattr(self, name)
            if value is not None:
                out[name] = value
        if self.preferred_posture_rad is not None:
            out["posture_reference"] = dict(self.preferred_posture_rad)
        return out


@dataclass
class RakudaTrajectoryConfig:
    """The ``control.trajectory`` section: how a hand target is turned into motion.

    A new target is never jumped to.  A reference pose moves from where the
    hand is towards the target under these ceilings, and the solver follows
    the reference one step at a time.  ``None`` means "not set": the viewer
    uses its simulation profile, the machine refuses to run Cartesian mode
    with a ceiling it was not given.

    Attributes:
        sample_period_s: Time between reference samples (and solver steps).
        max_linear_velocity_m_s: Ceiling on the reference's translation speed.
        max_linear_acceleration_m_s2: Ceiling on its translational acceleration.
        max_angular_velocity_rad_s: Ceiling on its rotation speed.
        max_angular_acceleration_rad_s2: Ceiling on its angular acceleration.
        lag_tolerance_m: When the hand lags the reference by more than this,
            the reference waits instead of running ahead.
    """

    sample_period_s: float | None = None
    max_linear_velocity_m_s: float | None = None
    max_linear_acceleration_m_s2: float | None = None
    max_angular_velocity_rad_s: float | None = None
    max_angular_acceleration_rad_s2: float | None = None
    lag_tolerance_m: float | None = None

    def missing(self) -> List[str]:
        """Names of the ceilings still unset."""
        return [
            name
            for name in (
                "sample_period_s",
                "max_linear_velocity_m_s",
                "max_linear_acceleration_m_s2",
                "max_angular_velocity_rad_s",
                "max_angular_acceleration_rad_s2",
            )
            if getattr(self, name) is None
        ]


@dataclass
class RakudaModelConfig:
    """Where the kinematic model lives and how its joints are grouped.

    Attributes:
        urdf_path: Path to the URDF.  ``None`` means the Rakuda model bundled
            with robopy (``robopy/models/rakuda``) is used.
        package_dirs: Directories used to resolve ``package://`` mesh URIs;
            the bundled model's own when empty.
        torso_joint: The shared torso yaw joint.
        left_arm_joints: The six left-arm URDF joints, shoulder to wrist.
        right_arm_joints: The six right-arm URDF joints, shoulder to wrist.
        head_joints: Head joints.  Modelled, never commanded by the arm IK.
        left_tcp: Left TCP definition.
        right_tcp: Right TCP definition.
        soft_limits_rad: Soft position limits ``{joint: (lower, upper)}``.
            Required for continuous joints, which carry no URDF range.  A
            soft limit only narrows the model's range.
        soft_limit_specs: The same limits with their provenance
            (``validated`` / ``note``), for entries written in the long form.
            A joint absent here is an unvalidated, simulation-only value.
        joint_limit_overrides_rad: ``{joint: RakudaLimitOverrideSpec}``
            replacing a URDF range known to be wrong, each with its reason.
            This is the only way a range gets wider than the file's.
        home_positions_rad: ``{joint: rad}`` of a home pose checked against
            the resolved limits (and collision, when modelled).  Offered by
            the viewer as *home*; never applied to the machine by itself.
        build_collision: Load the convex collision geometry.
        collision_exclusions: Approved geometry-name pairs to exclude, each with
            a recorded reason.  Blanket exclusion of every adjacent or nearby
            pair is not the same thing and is not what this is for.
        geometry_only: Declare the model's inertial data untrustworthy.
    """

    urdf_path: str | None = None
    package_dirs: List[str] = field(default_factory=list)
    torso_joint: str | None = None
    left_arm_joints: List[str] = field(default_factory=list)
    right_arm_joints: List[str] = field(default_factory=list)
    head_joints: List[str] = field(default_factory=list)
    left_tcp: RakudaTcpSpec | None = None
    right_tcp: RakudaTcpSpec | None = None
    soft_limits_rad: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    soft_limit_specs: Dict[str, RakudaSoftLimitSpec] = field(default_factory=dict)
    joint_limit_overrides_rad: Dict[str, RakudaLimitOverrideSpec] = field(default_factory=dict)
    home_positions_rad: Dict[str, float] = field(default_factory=dict)
    build_collision: bool = False
    collision_exclusions: List[Tuple[str, str, str]] = field(default_factory=list)
    geometry_only: bool = True

    def soft_limit_entries(self) -> Dict[str, Dict[str, Any]]:
        """Every soft limit as ``{joint: {"lower", "upper", "validated", "note"}}``.

        The form :meth:`WholeBodyModel.set_soft_limits` and
        :meth:`ModelBundle.load` accept, so provenance travels with the value.
        """
        out: Dict[str, Dict[str, Any]] = {}
        for joint, (lower, upper) in self.soft_limits_rad.items():
            spec = self.soft_limit_specs.get(joint)
            out[joint] = {
                "lower": float(lower),
                "upper": float(upper),
                "validated": bool(spec.validated) if spec else False,
                "note": spec.note if spec else "",
            }
        return out

    def override_entries(self) -> Dict[str, Dict[str, Any]]:
        """Every override as ``{joint: {"lower", "upper", "reason"}}``."""
        return {
            joint: {"lower": spec.lower, "upper": spec.upper, "reason": spec.reason}
            for joint, spec in self.joint_limit_overrides_rad.items()
        }


@dataclass
class RakudaBilateralConfig:
    """Joint-space bilateral coupling settings.

    Attributes:
        coupled_motors: Motors to couple.  At most the thirteen arm joints plus
            the shared torso; the head and the grippers are held or disabled by
            a separate policy and are never added implicitly.
        stiffness_nm_per_rad: ``K``, scalar or per joint.
        damping_nm_s_per_rad: ``D``, scalar or per joint.
        scale: ``S``, the leader-to-follower angle ratio.
        offset_rad: ``b``, the calibrated joint offset.
        max_torque_nm: Per-joint torque ceiling.
        max_torque_rate_nm_s: Per-joint torque slew ceiling.
        leader_current_limit_a: Per-motor current ceiling on the leader.
        follower_current_limit_a: Per-motor current ceiling on the follower.
        velocity_filter_hz: Velocity low-pass cut-off.
        ramp_time_s: Time over which the coupling fades in after engaging.
        max_alignment_error_rad: Refuse to engage above this position error.
        allow_uncompensated: Permit running without a validated gravity model.
            Only for a simulated plant or a supported joint, and it must be an
            explicit choice.
    """

    coupled_motors: List[str] = field(default_factory=list)
    stiffness_nm_per_rad: Dict[str, float] | float = 1.0
    damping_nm_s_per_rad: Dict[str, float] | float = 0.05
    scale: Dict[str, float] | float = 1.0
    offset_rad: Dict[str, float] | float = 0.0
    max_torque_nm: Dict[str, float] | float = 0.3
    max_torque_rate_nm_s: Dict[str, float] | float = 10.0
    leader_current_limit_a: Dict[str, float] = field(default_factory=dict)
    follower_current_limit_a: Dict[str, float] = field(default_factory=dict)
    velocity_filter_hz: float | None = 20.0
    ramp_time_s: float = 1.0
    max_alignment_error_rad: float = 0.1
    allow_uncompensated: bool = False


@dataclass
class RakudaControlConfig:
    """Everything the Rakuda control stack needs beyond the legacy settings.

    Attributes:
        mode: Which control mode to run.
        control_period_s: Configured control period.  What was actually achieved
            is measured and reported separately.
        ik_period_s: Period of the slower IK worker.
        read_timeout_s: Budget for one state read.
        write_timeout_s: Budget for one command write.
        max_state_age_s: Staleness limit on a measured snapshot.
        max_command_age_s: Staleness limit on a command.
        max_target_age_s: Staleness limit on an external Cartesian target.  It
            is deliberately separate from the two above: local joint-space
            bilateral control does not require a fresh external target every
            cycle, only fresh data from both machines.
        max_cross_bus_skew_s: Limit on the time difference between the two
            buses' snapshots.
        diagnostics_period_s: Temperature / voltage / error polling period.
        stop_policy: How to bring the machine to rest.  There is no universally
            safe default; choose it against the mechanism and its supports.
        bus_watchdog_counts: ``BUS_WATCHDOG`` value, 20 ms per count, or ``None``
            to leave the register alone.
        leader_joint_calibration: Per-motor calibration on the leader.
        follower_joint_calibration: Per-motor calibration on the follower.
        model: The kinematic model configuration.
        ik: Solver behaviour shared by the viewer and the machine.
        trajectory: Cartesian reference-trajectory ceilings.
        bilateral: Bilateral coupling configuration.
        allow_hardware_current_output: Master switch for commanding current on
            real hardware.  Off by default, so a freshly generated
            configuration cannot start driving current before anything has been
            measured.
    """

    mode: str = "position_teleop"
    control_period_s: float = 0.005
    ik_period_s: float = 0.02
    read_timeout_s: float = 0.02
    write_timeout_s: float = 0.02
    max_state_age_s: float = 0.05
    max_command_age_s: float = 0.05
    max_target_age_s: float = 0.2
    max_cross_bus_skew_s: float = 0.01
    diagnostics_period_s: float = 1.0
    stop_policy: str = "zero_current"
    bus_watchdog_counts: int | None = None
    leader_joint_calibration: Dict[str, RakudaJointCalibrationSpec] = field(default_factory=dict)
    follower_joint_calibration: Dict[str, RakudaJointCalibrationSpec] = field(default_factory=dict)
    model: RakudaModelConfig = field(default_factory=RakudaModelConfig)
    ik: RakudaIKConfig = field(default_factory=RakudaIKConfig)
    trajectory: RakudaTrajectoryConfig = field(default_factory=RakudaTrajectoryConfig)
    bilateral: RakudaBilateralConfig = field(default_factory=RakudaBilateralConfig)
    allow_hardware_current_output: bool = False


#: The thirteen joints the dual-arm IK can drive: the shared torso plus both
#: arms.  The head and the grippers are excluded on purpose.
RAKUDA_IK_MOTOR_NAMES: Tuple[str, ...] = (
    "torso_yaw",
    "r_arm_sh_pitch1",
    "r_arm_sh_roll",
    "r_arm_sh_pitch2",
    "r_arm_el_yaw",
    "r_arm_wr_roll",
    "r_arm_wr_yaw",
    "l_arm_sh_pitch1",
    "l_arm_sh_roll",
    "l_arm_sh_pitch2",
    "l_arm_el_yaw",
    "l_arm_wr_roll",
    "l_arm_wr_yaw",
)

#: Gripper motors.  Their URDF counterparts (``gripper_left_dof`` and
#: ``gripper_right_dof``) are *fixed* joints, so they are not movable joints of
#: the kinematic model and must not be invented as such.
RAKUDA_GRIPPER_MOTOR_NAMES: Tuple[str, ...] = ("l_arm_grip", "r_arm_grip")

#: Head motors: modelled for collision purposes, never driven by the arm IK.
RAKUDA_HEAD_MOTOR_NAMES: Tuple[str, ...] = ("head_yaw", "head_pitch")


@dataclass
class RAKUDA_CONTROLTABLE_VALUES:
    """Control-table values written during Rakuda gripper initialisation.

    The current values below are **raw control-table counts**, not milliamperes.
    They are written straight to ``GOAL_CURRENT`` and ``CURRENT_LIMIT``, which
    are in units of 0.00269 A per count on an XM430/XM540 and 0.001 A per count
    on an XC330.  The older comments said "mA"; that was a mislabel of what the
    code sends, and the *behaviour* is what is preserved here.  ``128`` keeps
    meaning raw 128 -- about 0.34 A on a follower XM430, not 128 mA -- so this
    documentation fix changes no gripping force.  Use
    :attr:`FOLLOWER_GRIP_GOAL_CURRENT_A` when an ampere value is wanted.
    """

    GRIP_OPEN_POSITION: int = 2500  # Open position for gripper
    GRIP_PID: Tuple[int, int, int] = (128, 32, 64)  # PID values for gripper control
    GRIP_PID_SLOW: Tuple[int, int, int] = (
        512,
        64,
        1024,
    )  # PID values for gripper control in slow mode
    # raw control-table counts (see the class docstring)
    FOLLOWER_GRIP_GOAL_CURRENT: int = 128
    FOLLOWER_GRIP_CURRENT_LIMIT: int = 128

    LEADER_GRIP_GOAL_CURRENT: int = 30
    LEADER_GRIP_CURRENT_LIMIT: int = 30

    GRIP_MAX_POSITION: int = 2600  # Maximum position for gripper
    CURRENT_BASED_OPERATING_MODE: int = (
        5  # Operating mode for gripper motors (Current-based position control)
    )
    POSITION_CONTROL_MODE: int = 3  # Operating mode for non-gripper motors (Position control)

    @staticmethod
    def raw_current_to_a(raw: int, model_name: str) -> float:
        """Convert one of the raw current constants above to amperes.

        Args:
            raw: A raw ``GOAL_CURRENT`` / ``CURRENT_LIMIT`` count.
            model_name: The DYNAMIXEL model the value is written to, since the
                unit differs per model.

        Returns:
            The equivalent current in amperes.
        """
        from robopy.motor.dynamixel_control_table import get_motor_capabilities

        return raw * get_motor_capabilities(model_name).current_unit_a
