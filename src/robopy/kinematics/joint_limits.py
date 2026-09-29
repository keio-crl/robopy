"""One place that decides every joint's usable range, and says where it came from.

Three things used to answer "how far may this joint turn" and disagreed: the
viewer's sliders (the actuator's ``+/-pi`` travel on every axis), the solver
(the URDF range narrowed by soft limits), and the machine (whatever its
calibration says).  A pose the sliders allowed could be one the solver refused
to start from.  :func:`resolve_joint_limits` is the single resolver all of
them consult, and it distinguishes what the plan calls for:

* the **URDF** range, which for a CAD export is what the exporter wrote --
  usually right, sometimes a placeholder, and absent for ``continuous`` joints;
* an explicit **override** of that range, ``joint_limit_overrides_rad``, for
  the case where the export is known to be wrong.  An override *replaces* the
  URDF range and is a recorded decision with a stated reason; it is the only
  way a range ever gets wider than the file's, and it never widens a
  validated soft limit;
* a **soft** limit, ``soft_limits_rad``, which only ever *narrows*: the
  cable-limited travel of a continuous joint, a cover, a neighbouring link.
  Each soft limit is either validated (measured on the machine) or a
  provisional stand-in for simulation, and the profile keeps that apart so a
  stand-in is never promoted to a real limit by accident;
* a **display** range for a joint that still has no finite bound at all: the
  page needs a slider, the solver refuses to drive such a joint, and the
  profile says so rather than pretending.

The resolved ``(lower, upper)`` is what the sliders span, what the solver
bounds its steps with, what a home pose is checked against and what the
machine adapter may command.  Every consumer asks the profile; none keeps a
range of its own.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

__all__ = [
    "HomePoseCheck",
    "JointLimit",
    "JointLimitOverride",
    "JointLimitProfile",
    "SoftLimit",
    "check_home_pose",
    "limits_from_model",
    "normalised_singular_values",
    "provisional_soft_limits",
    "resolve_joint_limits",
    "unwrap_towards",
]

Bounds = Tuple[float, float]


@dataclass(frozen=True)
class JointLimitOverride:
    """A recorded replacement of a URDF range known to be wrong.

    Attributes:
        lower: Replacement lower bound, radians, in URDF joint coordinates.
        upper: Replacement upper bound.
        reason: Why the exported range is not trusted (kept with the value, so
            the decision travels with the configuration).
    """

    lower: float
    upper: float
    reason: str = ""

    def __post_init__(self) -> None:
        if not (math.isfinite(self.lower) and math.isfinite(self.upper)):
            raise ValueError("An override must be finite on both sides.")
        if self.lower >= self.upper:
            raise ValueError(f"Override lower {self.lower} must be below upper {self.upper}.")


@dataclass(frozen=True)
class SoftLimit:
    """A limit that narrows the joint's range, measured or provisional.

    Attributes:
        lower: Lower bound, radians.
        upper: Upper bound.
        validated: ``True`` when measured on the machine.  ``False`` marks a
            stand-in for simulation that must never be reported as the
            machine's limit.
        note: Free text: where the number came from.
    """

    lower: float
    upper: float
    validated: bool = False
    note: str = ""

    def __post_init__(self) -> None:
        if not (math.isfinite(self.lower) and math.isfinite(self.upper)):
            raise ValueError("A soft limit must be finite on both sides.")
        if self.lower >= self.upper:
            raise ValueError(f"Soft limit lower {self.lower} must be below upper {self.upper}.")


@dataclass(frozen=True)
class JointLimit:
    """The resolved range of one joint, with its provenance.

    Attributes:
        joint: Joint name.
        lower: Resolved lower bound in radians (``-inf`` when unbounded).
        upper: Resolved upper bound (``+inf`` when unbounded).
        source: What decided the narrowest bound: ``"urdf"``, ``"override"``,
            ``"soft"``, ``"display"`` (no finite bound; ``lower``/``upper`` are
            the display range) or ``"unbounded"`` (no finite bound and no
            display range asked for).
        validated: Whether the resolved range may be trusted on the machine:
            a validated soft limit, or a URDF/override range on a bounded
            joint.  ``False`` for a provisional soft limit and for a display
            range.
        continuous: Whether the joint is a URDF ``continuous`` joint.
        urdf: The URDF's own range, or ``None`` for a continuous joint.
        override: The recorded override, if any.
        soft: The soft limit, if any.
        display_only: ``True`` when the range shown is the display range,
            i.e. not a limit of anything.
        notes: Anything worth telling the operator about this joint's range.
    """

    joint: str
    lower: float
    upper: float
    source: str
    validated: bool
    continuous: bool
    urdf: Bounds | None = None
    override: JointLimitOverride | None = None
    soft: SoftLimit | None = None
    display_only: bool = False
    notes: Tuple[str, ...] = ()

    @property
    def finite(self) -> bool:
        """Whether the resolved range is a real, finite bound."""
        return not self.display_only and math.isfinite(self.lower) and math.isfinite(self.upper)

    @property
    def span(self) -> float:
        """``upper - lower``."""
        return self.upper - self.lower

    def contains(self, value: float, *, margin: float = 0.0) -> bool:
        """Whether ``value`` lies within the resolved range, ``margin`` inside each end."""
        return self.lower + margin <= value <= self.upper - margin

    def violation(self, value: float) -> float:
        """How far ``value`` is outside the resolved range (0 when inside)."""
        if value < self.lower:
            return float(self.lower - value)
        if value > self.upper:
            return float(value - self.upper)
        return 0.0

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly view."""
        return {
            "joint": self.joint,
            "lower": None if not math.isfinite(self.lower) else float(self.lower),
            "upper": None if not math.isfinite(self.upper) else float(self.upper),
            "source": self.source,
            "validated": self.validated,
            "continuous": self.continuous,
            "urdf": None if self.urdf is None else [float(v) for v in self.urdf],
            "override": None
            if self.override is None
            else {
                "lower": self.override.lower,
                "upper": self.override.upper,
                "reason": self.override.reason,
            },
            "soft": None
            if self.soft is None
            else {
                "lower": self.soft.lower,
                "upper": self.soft.upper,
                "validated": self.soft.validated,
                "note": self.soft.note,
            },
            "display_only": self.display_only,
            "notes": list(self.notes),
        }


@dataclass(frozen=True)
class JointLimitProfile:
    """The resolved limits of every movable joint of a model.

    Attributes:
        limits: ``{joint: JointLimit}`` in the model's joint order.
        display_range_rad: The half-width used for display-only ranges.
    """

    limits: Dict[str, JointLimit]
    display_range_rad: float = math.pi

    def __getitem__(self, joint: str) -> JointLimit:
        return self.limits[joint]

    def __contains__(self, joint: object) -> bool:
        return joint in self.limits

    @property
    def joints(self) -> Tuple[str, ...]:
        """Joint names in model order."""
        return tuple(self.limits)

    def bounds(self, joints: Sequence[str]) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """``(lower, upper)`` arrays of the *real* limits for ``joints``.

        A display-only range counts as unbounded here (``+/-inf``): the page
        may draw it, the solver may not lean on it.
        """
        lower = []
        upper = []
        for name in joints:
            limit = self.limits[name]
            if limit.display_only:
                lower.append(-math.inf)
                upper.append(math.inf)
            else:
                lower.append(limit.lower)
                upper.append(limit.upper)
        return np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)

    def slider_bounds(self, joints: Sequence[str]) -> Tuple[List[float], List[float]]:
        """``(lower, upper)`` for the page's sliders: the real limit, or the display range."""
        lower = [self.limits[name].lower for name in joints]
        upper = [self.limits[name].upper for name in joints]
        return lower, upper

    def unbounded(self, joints: Sequence[str]) -> List[str]:
        """Joints among ``joints`` without a finite real limit."""
        return [name for name in joints if not self.limits[name].finite]

    def unvalidated(self, joints: Sequence[str]) -> List[str]:
        """Joints among ``joints`` whose resolved range is not validated for the machine."""
        return [name for name in joints if not self.limits[name].validated]

    def violations(self, positions: Mapping[str, float]) -> Dict[str, float]:
        """``{joint: amount}`` for every position outside its resolved real limit."""
        out: Dict[str, float] = {}
        for name, value in positions.items():
            limit = self.limits.get(name)
            if limit is None or limit.display_only:
                continue
            amount = limit.violation(float(value))
            if amount > 0.0:
                out[name] = amount
        return out

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly view of every joint."""
        return {
            "display_range_rad": self.display_range_rad,
            "joints": [limit.describe() for limit in self.limits.values()],
            "sources": {
                source: [n for n, lim in self.limits.items() if lim.source == source]
                for source in ("urdf", "override", "soft", "display", "unbounded")
            },
        }

    def summary_lines(self) -> List[str]:
        """Human-readable lines for a start-up log."""
        lines = []
        for limit in self.limits.values():
            if limit.display_only:
                rng = f"display +/-{self.display_range_rad:.2f} (NO real limit)"
            elif not limit.finite:
                rng = "unbounded (NO limit)"
            else:
                rng = f"[{limit.lower:+.3f}, {limit.upper:+.3f}]"
            flag = "validated" if limit.validated else "NOT validated"
            lines.append(f"{limit.joint}: {rng} rad from {limit.source} ({flag})")
        return lines


def _coerce_soft(value: Any) -> SoftLimit:
    if isinstance(value, SoftLimit):
        return value
    if isinstance(value, Mapping):
        return SoftLimit(
            lower=float(value["lower"]),
            upper=float(value["upper"]),
            validated=bool(value.get("validated", False)),
            note=str(value.get("note", "")),
        )
    lower, upper = value
    return SoftLimit(lower=float(lower), upper=float(upper))


def _coerce_override(value: Any) -> JointLimitOverride:
    if isinstance(value, JointLimitOverride):
        return value
    if isinstance(value, Mapping):
        return JointLimitOverride(
            lower=float(value["lower"]),
            upper=float(value["upper"]),
            reason=str(value.get("reason", "")),
        )
    lower, upper = value
    return JointLimitOverride(lower=float(lower), upper=float(upper))


def resolve_joint_limits(
    joints: Sequence[str],
    *,
    urdf_limits: Mapping[str, Bounds | None],
    overrides: Mapping[str, Any] | None = None,
    soft_limits: Mapping[str, Any] | None = None,
    display_range_rad: float | None = math.pi,
) -> JointLimitProfile:
    """Resolve every joint's range from the URDF, the overrides and the soft limits.

    Args:
        joints: Movable joint names, in model order.
        urdf_limits: ``{joint: (lower, upper) | None}``; ``None`` for a
            continuous joint (no range in the file).
        overrides: ``{joint: JointLimitOverride | {"lower", "upper", "reason"} |
            (lower, upper)}``.  An override replaces the URDF range.
        soft_limits: ``{joint: SoftLimit | {"lower", "upper", "validated",
            "note"} | (lower, upper)}``.  A plain pair is an *unvalidated*
            soft limit.  A soft limit narrows; where it is wider than the base
            range on a side it does not widen it, and that is noted.
        display_range_rad: Half-width of the range given to a joint that ends
            up with no finite bound, for the page's sliders.  ``None`` leaves
            such a joint unbounded.

    Returns:
        The :class:`JointLimitProfile`.

    Raises:
        KeyError: On an override or soft limit for an unknown joint.
        ValueError: On an inverted or non-finite range.
    """
    overrides = dict(overrides or {})
    soft_limits = dict(soft_limits or {})
    known = set(joints)
    for kind, table in (("override", overrides), ("soft limit", soft_limits)):
        unknown = sorted(set(table) - known)
        if unknown:
            raise KeyError(f"{kind} for unknown joint(s): {unknown}")

    limits: Dict[str, JointLimit] = {}
    for name in joints:
        raw = urdf_limits.get(name)
        continuous = raw is None
        urdf: Bounds | None = None if raw is None else (float(raw[0]), float(raw[1]))
        if urdf is not None:
            if not (math.isfinite(urdf[0]) and math.isfinite(urdf[1])):
                # A revolute joint whose file gives no usable range is treated
                # like a continuous one: nothing to lean on.
                urdf = None
                continuous = False
            elif urdf[0] >= urdf[1]:
                raise ValueError(
                    f"{name}: the URDF range [{urdf[0]}, {urdf[1]}] is inverted or empty; "
                    "give an override with a reason."
                )
        notes: List[str] = []
        override = _coerce_override(overrides[name]) if name in overrides else None
        soft = _coerce_soft(soft_limits[name]) if name in soft_limits else None

        base: Bounds | None
        source: str
        validated: bool
        if override is not None:
            base = (override.lower, override.upper)
            source = "override"
            validated = True
            if urdf is not None:
                notes.append(
                    f"URDF range [{urdf[0]:.3f}, {urdf[1]:.3f}] replaced by an override"
                    + (f": {override.reason}" if override.reason else " (no reason recorded)")
                )
            else:
                notes.append("continuous joint given a range by override")
        elif urdf is not None:
            base = urdf
            source = "urdf"
            validated = True
        else:
            base = None
            source = "unbounded"
            validated = False

        lower: float
        upper: float
        if soft is not None:
            if base is None:
                lower, upper = soft.lower, soft.upper
            else:
                lower, upper = max(base[0], soft.lower), min(base[1], soft.upper)
                if soft.lower < base[0] - 1e-12 or soft.upper > base[1] + 1e-12:
                    notes.append(
                        f"soft limit [{soft.lower:.3f}, {soft.upper:.3f}] is wider than the "
                        f"{source} range on at least one side; a soft limit only narrows (use "
                        "joint_limit_overrides_rad, with a reason, to replace a wrong range)"
                    )
                if lower >= upper:
                    raise ValueError(
                        f"{name}: the soft limit [{soft.lower}, {soft.upper}] does not overlap "
                        f"the {source} range [{base[0]}, {base[1]}]."
                    )
            narrowed = base is None or lower > base[0] + 1e-12 or upper < base[1] - 1e-12
            if narrowed:
                source = "soft"
                validated = soft.validated
                if not soft.validated:
                    notes.append(
                        "soft limit is provisional (not measured on the machine): "
                        "simulation only" + (f"; {soft.note}" if soft.note else "")
                    )
            elif base is not None:
                # The soft limit did not bite; the base range stands and its
                # validation state is the base's.
                pass
        elif base is not None:
            lower, upper = base
        else:
            lower, upper = -math.inf, math.inf

        display_only = False
        if not (math.isfinite(lower) and math.isfinite(upper)):
            if display_range_rad is not None:
                lower, upper = -float(display_range_rad), float(display_range_rad)
                display_only = True
                source = "display"
                notes.append(
                    f"no finite limit: the +/-{display_range_rad:.2f} rad range is for display "
                    "only; the solver will not drive this joint until a soft limit is given"
                )
            validated = False

        limits[name] = JointLimit(
            joint=name,
            lower=float(lower),
            upper=float(upper),
            source=source,
            validated=validated,
            continuous=continuous,
            urdf=urdf,
            override=override,
            soft=soft,
            display_only=display_only,
            notes=tuple(notes),
        )
    return JointLimitProfile(
        limits=limits,
        display_range_rad=math.pi if display_range_rad is None else float(display_range_rad),
    )


@dataclass(frozen=True)
class HomePoseCheck:
    """What :func:`check_home_pose` found.

    Attributes:
        problems: Reasons the pose is not a usable home; empty when it is.
        min_singular_value: Smallest singular value of the normalised arm
            Jacobians at the pose, or ``None`` when no arms were given.
        min_collision_distance_m: Closest registered pair, or ``None`` when
            the model carries no collision geometry.
    """

    problems: Tuple[str, ...] = ()
    min_singular_value: float | None = None
    min_collision_distance_m: float | None = None

    @property
    def ok(self) -> bool:
        """Whether the pose passed every check."""
        return not self.problems


def check_home_pose(
    model: Any,
    profile: JointLimitProfile,
    positions: Mapping[str, float],
    *,
    arms: Mapping[str, Tuple[Sequence[str], str]] | None = None,
    margin_rad: float = 0.0,
    singular_threshold: float = 1e-2,
    position_scale_m: float = 1e-3,
    orientation_scale_rad: float = 1e-2,
) -> HomePoseCheck:
    """Check that a pose is inside the limits, out of collision and not singular.

    Args:
        model: A :class:`~robopy.kinematics.urdf_model.WholeBodyModel`.
        profile: The resolved limits.
        positions: ``{joint: rad}`` for every movable joint.
        arms: ``{side: (arm_joints, tcp_frame)}``; each arm's Jacobian at the
            pose is checked for rank loss.
        margin_rad: How far inside each limit the pose must sit.
        singular_threshold: Smallest normalised singular value accepted.
        position_scale_m: Position tolerance used to normalise the linear
            rows of the Jacobian (see :func:`normalised_singular_values`).
        orientation_scale_rad: Orientation tolerance used for the angular rows.
    """
    problems: List[str] = []
    missing = sorted(set(model.movable_joint_names) - set(positions))
    if missing:
        problems.append(f"missing joint(s): {missing}")
        return HomePoseCheck(problems=tuple(problems))
    for name in model.movable_joint_names:
        limit = profile.limits.get(name)
        if limit is None or limit.display_only:
            continue
        if not limit.contains(float(positions[name]), margin=margin_rad):
            problems.append(
                f"{name} = {positions[name]:+.3f} rad is outside "
                f"[{limit.lower + margin_rad:+.3f}, {limit.upper - margin_rad:+.3f}] "
                f"({limit.source})"
            )
    q = model.q_from_positions(positions)
    min_distance = None
    if getattr(model, "collision_model", None) is not None:
        report = model.collision_report(q)
        min_distance = report.min_distance
        if min_distance < 0.0:
            problems.append(
                f"the pose is in self-collision (min distance {min_distance:.4f} m at "
                f"{report.closest_pair()})"
            )
    min_sigma: float | None = None
    for side, (arm_joints, frame) in (arms or {}).items():
        J = model.frame_jacobian(q, frame, local=False)
        columns = model.v_indices(arm_joints)
        sigma = normalised_singular_values(
            J[:, columns],
            position_scale_m=position_scale_m,
            orientation_scale_rad=orientation_scale_rad,
        )
        smallest = float(sigma[-1]) if sigma.size else 0.0
        min_sigma = smallest if min_sigma is None else min(min_sigma, smallest)
        if smallest < singular_threshold:
            problems.append(
                f"the {side} arm is near a singular configuration (normalised sigma_min "
                f"{smallest:.3g} < {singular_threshold:g})"
            )
    return HomePoseCheck(
        problems=tuple(problems),
        min_singular_value=min_sigma,
        min_collision_distance_m=min_distance,
    )


def normalised_singular_values(
    jacobian: NDArray[np.float64],
    *,
    position_scale_m: float,
    orientation_scale_rad: float,
) -> NDArray[np.float64]:
    """Singular values of a ``(6, n)`` frame Jacobian with its rows made unitless.

    Metres and radians cannot be compared, so a raw singular value mixes the
    two; dividing the linear rows by a position tolerance and the angular rows
    by an orientation tolerance turns every row into "tolerances per radian of
    joint motion", which is a scale on which one threshold makes sense.
    Rows are linear then angular, Pinocchio's ordering.
    """
    J = np.asarray(jacobian, dtype=np.float64)
    if J.shape[0] != 6:
        raise ValueError("Expected a (6, n) frame Jacobian.")
    if position_scale_m <= 0.0 or orientation_scale_rad <= 0.0:
        raise ValueError("Scales must be positive.")
    scaled = J.copy()
    scaled[:3, :] /= position_scale_m
    scaled[3:, :] /= orientation_scale_rad
    return np.asarray(np.linalg.svd(scaled, compute_uv=False), dtype=np.float64)


def unwrap_towards(angle: float, reference: float) -> float:
    """The representative of ``angle`` (mod 2pi) closest to ``reference``.

    A continuous joint decoded from ``(cos, sin)`` comes back in ``(-pi, pi]``;
    a command derived from it must not jump by a turn when the measured angle
    sits near ``+/-pi``.  Choosing the representative nearest the current
    measurement keeps the commanded angle continuous.
    """
    two_pi = 2.0 * math.pi
    return float(angle + two_pi * round((reference - angle) / two_pi))


def limits_from_model(model: Any) -> Dict[str, Bounds | None]:
    """``{joint: urdf_range | None}`` straight from a whole-body model."""
    out: Dict[str, Bounds | None] = {}
    for name in model.movable_joint_names:
        if model.is_continuous(name):
            out[name] = None
        else:
            sl = model.joint_q_slice(name)
            lo = float(model.model.lowerPositionLimit[sl][0])
            hi = float(model.model.upperPositionLimit[sl][0])
            out[name] = (lo, hi)
    return out


def provisional_soft_limits(
    joints: Iterable[str], bounds: Bounds, note: str
) -> Dict[str, SoftLimit]:
    """Unvalidated soft limits for ``joints``, all carrying the same note."""
    return {name: SoftLimit(bounds[0], bounds[1], validated=False, note=note) for name in joints}
