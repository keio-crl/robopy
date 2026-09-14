"""The single conversion boundary between DYNAMIXEL raw values and SI joint space.

Only this module converts encoder counts, raw velocity units and raw current
units into radians, radians per second and amperes.  Nothing downstream of it
re-applies a calibration, and nothing upstream of it stacks a second calibration
on top of the values produced here.

Zero-point convention
---------------------
``zero_count`` is defined against the value *read back from*
``PRESENT_POSITION``, i.e. after the motor has applied its own ``Drive Mode``
and ``Homing Offset``.  Both of those registers are therefore part of the
calibration and are recorded alongside it, so that a configuration validated on
one machine cannot silently be reused on a machine whose ``Homing Offset``
differs.

Directly coupled rotary joints use::

    q = direction * (count - zero_count) * 2 * pi / counts_per_revolution

``external_gear_ratio`` is only for transmissions *outside* the servo (belts,
extra reductions).  The DYNAMIXEL internal reduction is already reflected in the
encoder resolution, so it must never be multiplied in a second time here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.motor.dynamixel_control_table import (
    DEFAULT_COUNTS_PER_REVOLUTION,
    get_motor_capabilities,
)

__all__ = [
    "JointCalibration",
    "JointMap",
    "JointMapError",
    "RAW_VELOCITY_UNIT_RAD_S",
    "ValidationLevel",
]

#: Raw ``PRESENT_VELOCITY``/``GOAL_VELOCITY`` unit for the X-series models used
#: on Rakuda (XM430-W350, XM540-W270, XC330-T288): 0.229 rev/min per count.
RAW_VELOCITY_UNIT_RAD_S: float = 0.229 * 2.0 * math.pi / 60.0


class JointMapError(ValueError):
    """Raised when a joint map is inconsistent or incomplete."""


class ValidationLevel:
    """How much of a calibration must be measured before it may be used.

    Attributes:
        GEOMETRY: Enough to run forward kinematics and IK in simulation.  Sign,
            zero point and joint limits must be present; torque-related fields
            may still be unmeasured.
        HARDWARE: Everything :attr:`GEOMETRY` requires, plus a per-joint torque
            constant and a current limit, and an explicit ``validated`` flag.
            Required before any current is commanded on real hardware.
    """

    GEOMETRY = "geometry"
    HARDWARE = "hardware"

    ALL = (GEOMETRY, HARDWARE)


@dataclass(frozen=True)
class JointCalibration:
    """Calibration of one motor, and its correspondence to a URDF joint.

    Attributes:
        motor_name: Name of the motor on its bus (e.g. ``"l_arm_sh_pitch1"``).
        motor_id: DYNAMIXEL ID on that bus.
        model: DYNAMIXEL model name (e.g. ``"xm430-w350"``).
        urdf_joint: Name of the corresponding movable URDF joint, or ``None``
            when this motor has no counterpart in the kinematic model (the
            grippers, for instance -- the Rakuda URDF's ``gripper_*_dof``
            frames are *fixed* joints and must not be treated as movable).
        direction: ``+1`` or ``-1``.  Sign of the joint axis relative to the
            motor's own positive direction.  It is applied identically to
            position, velocity and current, so that torque and motion share one
            axis convention.
        zero_count: Encoder count that corresponds to ``q = 0``, expressed
            against ``PRESENT_POSITION`` as read back from the motor.
        counts_per_revolution: Encoder counts per full output revolution.
        external_gear_ratio: Reduction *outside* the servo (output turns per
            encoder turn).  ``1.0`` for a directly coupled joint.
        drive_mode: ``DRIVE_MODE`` register value the zero point was defined
            against, or ``None`` when it was not recorded.
        homing_offset: ``HOMING_OFFSET`` register value the zero point was
            defined against, or ``None`` when it was not recorded.
        lower_limit_rad: Lower joint limit, or ``None`` when unmeasured.
        upper_limit_rad: Upper joint limit, or ``None`` when unmeasured.
        max_velocity_rad_s: Velocity limit, or ``None`` when unmeasured.
        max_acceleration_rad_s2: Acceleration limit, or ``None`` when unmeasured.
        torque_constant_nm_per_a: Validated torque per ampere for this joint, or
            ``None``.  A stall-torque / stall-current ratio taken from a
            datasheet is *not* a validated torque constant and must not be put
            here.
        current_limit_a: Software current ceiling in amperes, or ``None``.
        max_current_rate_a_s: Maximum rate of change of commanded current, or
            ``None``.
        validated: Whether these numbers were measured on the actual machine.
            ``False`` means "written down but unverified"; no current output is
            permitted from an unvalidated calibration.
        notes: Free-form provenance note.
    """

    motor_name: str
    motor_id: int
    model: str
    urdf_joint: str | None = None
    direction: int = 1
    zero_count: int | None = None
    counts_per_revolution: int = DEFAULT_COUNTS_PER_REVOLUTION
    external_gear_ratio: float = 1.0
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

    def __post_init__(self) -> None:
        if self.direction not in (1, -1):
            raise JointMapError(
                f"{self.motor_name}: direction must be +1 or -1, got {self.direction}."
            )
        if not 0 <= self.motor_id <= 252:
            raise JointMapError(f"{self.motor_name}: motor_id {self.motor_id} is out of range.")
        if self.counts_per_revolution <= 0:
            raise JointMapError(f"{self.motor_name}: counts_per_revolution must be positive.")
        if self.external_gear_ratio <= 0.0:
            raise JointMapError(f"{self.motor_name}: external_gear_ratio must be positive.")
        if (
            self.lower_limit_rad is not None
            and self.upper_limit_rad is not None
            and self.lower_limit_rad > self.upper_limit_rad
        ):
            raise JointMapError(
                f"{self.motor_name}: lower_limit_rad exceeds upper_limit_rad "
                f"({self.lower_limit_rad} > {self.upper_limit_rad})."
            )
        for name in ("current_limit_a", "max_current_rate_a_s", "torque_constant_nm_per_a"):
            value = getattr(self, name)
            if value is not None and value <= 0.0:
                raise JointMapError(f"{self.motor_name}: {name} must be positive when set.")
        # Validate the model name eagerly so a typo fails at configuration time
        # rather than on the first bus transaction.
        get_motor_capabilities(self.model)

    # -- unit conversions -------------------------------------------------

    @property
    def rad_per_count(self) -> float:
        """Radians of joint motion per encoder count (sign excluded)."""
        return 2.0 * math.pi / (self.counts_per_revolution * self.external_gear_ratio)

    @property
    def current_unit_a(self) -> float:
        """Amperes per raw current count for this motor model."""
        return get_motor_capabilities(self.model).current_unit_a

    def count_to_rad(self, count: float) -> float:
        """Convert a ``PRESENT_POSITION`` count to a joint angle in radians.

        Multi-turn counts are *not* wrapped into ``[-pi, pi]``: a multi-turn
        joint legitimately reports values outside one revolution, and folding
        them would silently destroy that information.
        """
        if self.zero_count is None:
            raise JointMapError(f"{self.motor_name}: zero_count is not calibrated.")
        return self.direction * (count - self.zero_count) * self.rad_per_count

    def rad_to_count(self, angle_rad: float) -> int:
        """Convert a joint angle in radians to a ``GOAL_POSITION`` count."""
        if self.zero_count is None:
            raise JointMapError(f"{self.motor_name}: zero_count is not calibrated.")
        return int(round(self.zero_count + self.direction * angle_rad / self.rad_per_count))

    def raw_velocity_to_rad_s(self, raw_velocity: float) -> float:
        """Convert a raw ``PRESENT_VELOCITY`` value to radians per second.

        The position zero offset is deliberately *not* applied here: an offset
        is meaningless for a derivative.
        """
        return self.direction * raw_velocity * RAW_VELOCITY_UNIT_RAD_S / self.external_gear_ratio

    def rad_s_to_raw_velocity(self, velocity_rad_s: float) -> int:
        """Convert radians per second to a raw velocity value."""
        return int(
            round(
                self.direction * velocity_rad_s * self.external_gear_ratio / RAW_VELOCITY_UNIT_RAD_S
            )
        )

    def raw_current_to_a(self, raw_current: float) -> float:
        """Convert a raw signed current count to amperes in the joint direction."""
        return self.direction * raw_current * self.current_unit_a

    def a_to_raw_current(self, current_a: float) -> int:
        """Convert amperes in the joint direction to a raw signed current count."""
        return int(round(self.direction * current_a / self.current_unit_a))

    def torque_to_current_a(self, torque_nm: float) -> float:
        """Convert a joint torque to a motor current using the validated constant.

        Raises:
            JointMapError: If no validated torque constant has been measured for
                this joint.  Falling back to a datasheet stall ratio is not
                permitted -- it would turn an unverified number into a current
                command.
        """
        if self.torque_constant_nm_per_a is None:
            raise JointMapError(
                f"{self.motor_name}: torque_constant_nm_per_a is not calibrated, so a "
                "torque cannot be converted into a current command."
            )
        return torque_nm / self.torque_constant_nm_per_a

    # -- validation -------------------------------------------------------

    def missing_fields(self, level: str) -> List[str]:
        """Names of the fields still unmeasured for ``level``.

        Args:
            level: One of :attr:`ValidationLevel.GEOMETRY` or
                :attr:`ValidationLevel.HARDWARE`.

        Returns:
            A sorted list of field names; empty when the calibration is complete
            for that level.
        """
        if level not in ValidationLevel.ALL:
            raise JointMapError(f"Unknown validation level '{level}'.")

        missing: List[str] = []
        if self.zero_count is None:
            missing.append("zero_count")
        if self.lower_limit_rad is None:
            missing.append("lower_limit_rad")
        if self.upper_limit_rad is None:
            missing.append("upper_limit_rad")

        if level == ValidationLevel.HARDWARE:
            if self.max_velocity_rad_s is None:
                missing.append("max_velocity_rad_s")
            if self.torque_constant_nm_per_a is None:
                missing.append("torque_constant_nm_per_a")
            if self.current_limit_a is None:
                missing.append("current_limit_a")
            if not self.validated:
                missing.append("validated")
        return sorted(missing)

    def is_complete(self, level: str) -> bool:
        """Whether this calibration is complete enough for ``level``."""
        return not self.missing_fields(level)


class JointMap:
    """A validated set of :class:`JointCalibration` entries for one bus.

    The map is the only place that knows how a motor name, a DYNAMIXEL ID and a
    URDF joint name relate to each other.  Construction rejects duplicate motor
    names, duplicate IDs, a URDF joint claimed by two motors, and out-of-range
    numbers, so downstream code can assume the correspondence is sound.
    """

    def __init__(
        self,
        calibrations: Iterable[JointCalibration],
        *,
        known_urdf_joints: Sequence[str] | None = None,
    ) -> None:
        """Build and validate a joint map.

        Args:
            calibrations: The per-motor calibrations.
            known_urdf_joints: When given, every non-``None`` ``urdf_joint`` must
                appear in this sequence.  Pass the movable joint names of the
                loaded URDF model to catch a name that does not exist, or one
                that names a *fixed* joint.

        Raises:
            JointMapError: On any duplicate or unknown name.
        """
        entries = list(calibrations)
        if not entries:
            raise JointMapError("A joint map must contain at least one calibration.")

        by_name: Dict[str, JointCalibration] = {}
        by_id: Dict[int, str] = {}
        by_urdf: Dict[str, str] = {}
        for cal in entries:
            if cal.motor_name in by_name:
                raise JointMapError(f"Motor '{cal.motor_name}' is registered more than once.")
            if cal.motor_id in by_id:
                raise JointMapError(
                    f"Motor id {cal.motor_id} is used by both '{by_id[cal.motor_id]}' "
                    f"and '{cal.motor_name}'."
                )
            by_name[cal.motor_name] = cal
            by_id[cal.motor_id] = cal.motor_name
            if cal.urdf_joint is not None:
                if cal.urdf_joint in by_urdf:
                    raise JointMapError(
                        f"URDF joint '{cal.urdf_joint}' is assigned to both "
                        f"'{by_urdf[cal.urdf_joint]}' and '{cal.motor_name}'."
                    )
                by_urdf[cal.urdf_joint] = cal.motor_name

        if known_urdf_joints is not None:
            allowed = set(known_urdf_joints)
            unknown = sorted(name for name in by_urdf if name not in allowed)
            if unknown:
                raise JointMapError(
                    f"Unknown or non-movable URDF joint(s) in the joint map: {unknown}. "
                    f"Movable joints are: {sorted(allowed)}"
                )

        self._entries: Tuple[JointCalibration, ...] = tuple(entries)
        self._by_name = by_name
        self._by_id = by_id
        self._motor_by_urdf = by_urdf

    # -- lookups ----------------------------------------------------------

    def __len__(self) -> int:
        return len(self._entries)

    def __contains__(self, motor_name: object) -> bool:
        return motor_name in self._by_name

    def __iter__(self):  # type: ignore[no-untyped-def]
        return iter(self._entries)

    def __getitem__(self, motor_name: str) -> JointCalibration:
        try:
            return self._by_name[motor_name]
        except KeyError as exc:
            raise JointMapError(f"Unknown motor '{motor_name}' in this joint map.") from exc

    @property
    def motor_names(self) -> Tuple[str, ...]:
        """Motor names in registration order."""
        return tuple(cal.motor_name for cal in self._entries)

    @property
    def urdf_joints(self) -> Tuple[str, ...]:
        """URDF joint names of the motors that have one, in registration order."""
        return tuple(cal.urdf_joint for cal in self._entries if cal.urdf_joint is not None)  # type: ignore[misc]

    def motor_for_urdf_joint(self, urdf_joint: str) -> JointCalibration:
        """Calibration of the motor driving ``urdf_joint``."""
        try:
            return self._by_name[self._motor_by_urdf[urdf_joint]]
        except KeyError as exc:
            raise JointMapError(f"No motor is mapped to URDF joint '{urdf_joint}'.") from exc

    def has_urdf_joint(self, urdf_joint: str) -> bool:
        """Whether some motor drives ``urdf_joint``."""
        return urdf_joint in self._motor_by_urdf

    def with_updates(self, updates: Mapping[str, Mapping[str, object]]) -> "JointMap":
        """Return a new map with per-motor field overrides applied.

        Args:
            updates: ``{motor_name: {field: value}}``.  Unknown motor names raise.
        """
        unknown = sorted(set(updates) - set(self._by_name))
        if unknown:
            raise JointMapError(f"Cannot update unknown motor(s): {unknown}")
        new_entries = [
            replace(cal, **updates[cal.motor_name])  # type: ignore[arg-type]
            if cal.motor_name in updates
            else cal
            for cal in self._entries
        ]
        return JointMap(new_entries)

    # -- vectorised conversions -------------------------------------------

    def counts_to_rad(
        self,
        counts: Mapping[str, float],
        motor_names: Sequence[str],
    ) -> NDArray[np.float64]:
        """Convert counts to radians for ``motor_names``, in that order."""
        return np.asarray(
            [self[name].count_to_rad(float(counts[name])) for name in motor_names],
            dtype=np.float64,
        )

    def rad_to_counts(
        self,
        angles_rad: Mapping[str, float] | Sequence[float],
        motor_names: Sequence[str],
    ) -> Dict[str, int]:
        """Convert radians to counts for ``motor_names``."""
        if isinstance(angles_rad, Mapping):
            values = [float(angles_rad[name]) for name in motor_names]
        else:
            values = [float(v) for v in angles_rad]
            if len(values) != len(motor_names):
                raise JointMapError(f"Expected {len(motor_names)} angles, got {len(values)}.")
        return {name: self[name].rad_to_count(values[i]) for i, name in enumerate(motor_names)}

    def raw_velocity_to_rad_s(
        self,
        raw: Mapping[str, float],
        motor_names: Sequence[str],
    ) -> NDArray[np.float64]:
        """Convert raw velocities to radians per second for ``motor_names``."""
        return np.asarray(
            [self[name].raw_velocity_to_rad_s(float(raw[name])) for name in motor_names],
            dtype=np.float64,
        )

    def raw_current_to_a(
        self,
        raw: Mapping[str, float],
        motor_names: Sequence[str],
    ) -> NDArray[np.float64]:
        """Convert raw currents to amperes for ``motor_names``."""
        return np.asarray(
            [self[name].raw_current_to_a(float(raw[name])) for name in motor_names],
            dtype=np.float64,
        )

    # -- validation --------------------------------------------------------

    def validation_report(self, level: str) -> Dict[str, List[str]]:
        """Missing fields per motor for ``level``.

        Returns:
            ``{motor_name: [missing fields]}``, containing only motors that are
            still incomplete.  An empty dictionary means the whole map is ready
            for that level.
        """
        report: Dict[str, List[str]] = {}
        for cal in self._entries:
            missing = cal.missing_fields(level)
            if missing:
                report[cal.motor_name] = missing
        return report

    def require(self, level: str, *, motor_names: Sequence[str] | None = None) -> None:
        """Raise unless the requested motors are fully calibrated for ``level``.

        Args:
            level: See :class:`ValidationLevel`.
            motor_names: Restrict the check to these motors.  ``None`` checks all.

        Raises:
            JointMapError: Listing every motor and every field still missing.
        """
        report = self.validation_report(level)
        if motor_names is not None:
            wanted = set(motor_names)
            report = {name: miss for name, miss in report.items() if name in wanted}
        if report:
            details = "; ".join(
                f"{name}: {', '.join(miss)}" for name, miss in sorted(report.items())
            )
            raise JointMapError(
                f"Joint map is not complete for validation level '{level}'. "
                f"Unmeasured values must be measured, not guessed. Missing -> {details}"
            )

    def assert_disjoint_from(self, other: "JointMap", *, label: str = "joint map") -> None:
        """Raise if two maps claim the same URDF joint.

        Used to guarantee that an autonomous IK command path and a bilateral
        command path never drive the same joint at the same time.
        """
        shared = sorted(set(self._motor_by_urdf) & set(other._motor_by_urdf))
        if shared:
            raise JointMapError(
                f"{label}: URDF joint(s) {shared} are claimed by two command paths at once."
            )


def check_torque_policy(
    coupled_motor_names: Sequence[str],
    torque_enabled: Sequence[str] | None,
    *,
    all_motor_names: Sequence[str],
    side: str,
) -> None:
    """Validate a bilateral joint selection against a torque-enable policy.

    A joint that is to be driven by the bilateral controller but is configured
    torque-OFF is a configuration error: the coupling set is never silently
    widened or narrowed, and a configured torque-OFF is never overridden.

    Args:
        coupled_motor_names: Motors the bilateral controller will drive.
        torque_enabled: The configured torque-enable list.  ``None`` means "use
            the built-in default", which is resolved by the caller before
            getting here; passing ``None`` skips the check.
        all_motor_names: Every motor on that side's bus.
        side: ``"leader"`` or ``"follower"``, used in the error message.

    Raises:
        JointMapError: On an unknown motor name or a torque-OFF coupled joint.
    """
    unknown = sorted(set(coupled_motor_names) - set(all_motor_names))
    if unknown:
        raise JointMapError(f"{side}: unknown coupled motor name(s) {unknown}.")
    if torque_enabled is None:
        return
    disabled = sorted(set(coupled_motor_names) - set(torque_enabled))
    if disabled:
        raise JointMapError(
            f"{side}: joint(s) {disabled} are selected for bilateral control but are not in "
            f"{side}.torque_enabled. Fix the configuration; the coupling set is not adjusted "
            "automatically and a configured torque-OFF is not overridden."
        )


def build_joint_map_from_motors(
    motors: Mapping[str, object],
    *,
    urdf_joint_by_motor: Mapping[str, str] | None = None,
    defaults: Mapping[str, Mapping[str, object]] | None = None,
) -> JointMap:
    """Build an *uncalibrated* joint map from an existing ``DynamixelBus`` motor dict.

    Every numeric calibration field is left as ``None`` unless it appears in
    ``defaults``.  The result is deliberately unusable for current output until
    real measurements are supplied -- see :meth:`JointMap.require`.

    Args:
        motors: ``{motor_name: DynamixelMotor}`` as held by a ``DynamixelBus``.
        urdf_joint_by_motor: Optional motor-name to URDF-joint correspondence.
        defaults: Optional ``{motor_name: {field: value}}`` overrides.

    Returns:
        A validated :class:`JointMap` with unmeasured values left as ``None``.
    """
    urdf_map = dict(urdf_joint_by_motor or {})
    overrides = dict(defaults or {})
    entries: List[JointCalibration] = []
    for name, motor in motors.items():
        fields: Dict[str, object] = {
            "motor_name": name,
            "motor_id": int(getattr(motor, "id")),
            "model": str(getattr(motor, "model_name")),
            "urdf_joint": urdf_map.get(name),
            "counts_per_revolution": int(
                getattr(motor, "resolution", DEFAULT_COUNTS_PER_REVOLUTION)
            ),
        }
        fields.update(overrides.get(name, {}))
        entries.append(JointCalibration(**fields))  # type: ignore[arg-type]
    return JointMap(entries)
