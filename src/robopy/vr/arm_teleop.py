"""Controller poses -> Cartesian arm targets, with a clutch.

Each controller drives one arm only while its clutch button is held; released,
the target freezes and the solver holds the hand.  Two mappings exist:

* ``"absolute"`` (the default): the operator's head and the robot's head are
  made to coincide -- the headset position at re-centring maps to a reference
  point on the robot's head (see :meth:`ArmTeleop.set_anchor`) -- and while
  the clutch is held the hand target *is* the controller's position in that
  correspondence, scaled about the head.  Pressing the button therefore pulls
  the hand towards where the controller is, at the slew-limited speed; the
  operator sees the twin's hands come to their own.  The orientation is still
  applied relatively, as the rotation of the controller since the press: the
  grip's axes and a two-axis wrist's TCP have no natural correspondence.
* ``"relative"``: the controller's displacement and rotation since the press
  are applied to the hand's pose captured at that moment; pressing moves
  nothing.  Used when the operator's and the robot's reach differ so much that
  an absolute correspondence is not usable.

In the absolute mapping the press can pull the hand a long way if the operator's
hand is far from where the robot's is.  An **engagement gate** prevents that:
with an engage radius set (see :attr:`ArmTeleopConfig.engage_radius_m`, or per
sample for a device such as a tracked hand), the clutch is only *requested*
by the press and engages once the mapped target lies within that distance of
the current hand pose -- the operator brings their hand to the robot's, then
the two move together.  Until then the command reports ``waiting`` and the
distance still to go, so the page can show a marker.

The trigger drives the gripper *only* when the gripper's open and closed
angles have been measured on the machine and put in the configuration; with
either missing no gripper command is produced.  The angles are not something
to guess from a data sheet: the Rakuda's gripper joints are fixed in the URDF,
so the model does not know them either.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Literal, Mapping, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.control.types import DualArmTarget, TorsoPolicy, monotonic_ns

from .xr_math import axis_angle_of, matrix_to_quat_xyzw

__all__ = [
    "ArmCommand",
    "ArmTeleop",
    "ArmTeleopConfig",
    "ControllerSample",
    "DualArmTeleop",
    "DualArmTeleopOutput",
]


@dataclass
class ArmTeleopConfig:
    """Per-arm teleoperation tunables.

    Attributes:
        mapping: ``"absolute"`` (hand target = controller position about the
            head anchor) or ``"relative"`` (displacement since the press).
            See the module docstring.
        position_scale: Robot metres per operator metre while clutched.  In
            absolute mapping the scaling is about the head anchor.
        orientation_enabled: Whether the controller's rotation drives the
            hand's orientation.  With a two-axis wrist a translation-only
            mapping is often the usable one; see the orientation weight of
            the solver for the soft version.
        max_speed_m_s: Largest target translation speed; faster operator
            motion is slewed, never jumped to.
        max_angular_speed_rad_s: Same for the target rotation.
        workspace_min_m: Optional lower corner of a box (base frame) the target
            position is clamped into.
        workspace_max_m: Optional upper corner.
        gripper_motor: Follower motor name of this arm's gripper, or ``None``.
        gripper_open_rad: Measured gripper angle at trigger 0, or ``None``.
        gripper_closed_rad: Measured gripper angle at trigger 1, or ``None``.
        engage_radius_m: Absolute mapping only: the clutch engages only once
            the mapped target is within this distance (robot metres) of the
            current hand pose; ``None`` engages at once and lets the hand be
            pulled over at the slew-limited speed.  A sample may carry its own
            radius (:attr:`ControllerSample.engage_radius_m`), which wins.
    """

    mapping: Literal["absolute", "relative"] = "absolute"
    position_scale: float = 1.0
    orientation_enabled: bool = True
    max_speed_m_s: float = 0.6
    max_angular_speed_rad_s: float = 3.0
    workspace_min_m: Tuple[float, float, float] | None = None
    workspace_max_m: Tuple[float, float, float] | None = None
    gripper_motor: str | None = None
    gripper_open_rad: float | None = None
    gripper_closed_rad: float | None = None
    engage_radius_m: float | None = None

    def __post_init__(self) -> None:
        if self.mapping not in ("absolute", "relative"):
            raise ValueError("mapping must be 'absolute' or 'relative'.")
        if self.engage_radius_m is not None and self.engage_radius_m <= 0.0:
            raise ValueError("engage_radius_m must be positive (or None to engage at once).")
        if not 0.0 < self.position_scale <= 5.0:
            raise ValueError("position_scale must be within (0, 5].")
        if self.max_speed_m_s <= 0.0 or self.max_angular_speed_rad_s <= 0.0:
            raise ValueError("Speed limits must be positive.")
        if (self.workspace_min_m is None) != (self.workspace_max_m is None):
            raise ValueError("Give both workspace corners or neither.")
        if self.workspace_min_m is not None and self.workspace_max_m is not None:
            if any(lo >= hi for lo, hi in zip(self.workspace_min_m, self.workspace_max_m)):
                raise ValueError("workspace_min_m must be below workspace_max_m on every axis.")

    @property
    def gripper_available(self) -> bool:
        """Whether the trigger will produce a gripper command."""
        return (
            self.gripper_motor is not None
            and self.gripper_open_rad is not None
            and self.gripper_closed_rad is not None
        )

    def gripper_target(self, trigger: float) -> float | None:
        """Gripper angle for a trigger value in ``[0, 1]``, or ``None`` when unmeasured."""
        if not self.gripper_available:
            return None
        assert self.gripper_open_rad is not None and self.gripper_closed_rad is not None
        t = min(1.0, max(0.0, float(trigger)))
        return self.gripper_open_rad + t * (self.gripper_closed_rad - self.gripper_open_rad)


@dataclass(frozen=True)
class ControllerSample:
    """One controller reading, already in the operator frame.

    Attributes:
        pose: ``(4, 4)`` grip pose in the operator frame, or ``None`` when the
            controller is not tracked this frame.
        clutch: Whether the grip button is squeezed.
        trigger: Trigger value in ``[0, 1]``.
        buttons: Other named buttons (``"a"``, ``"b"``, ``"stick"`` ...).
        stamp_s: Monotonic time of the sample.
        engage_radius_m: What this device requires before its clutch engages
            in the absolute mapping (see :attr:`ArmTeleopConfig.engage_radius_m`),
            or ``None`` to use the arm's setting.
    """

    pose: NDArray[np.float64] | None
    clutch: bool = False
    trigger: float = 0.0
    buttons: Mapping[str, bool] = field(default_factory=dict)
    stamp_s: float = 0.0
    engage_radius_m: float | None = None


@dataclass(frozen=True)
class ArmCommand:
    """What one arm should do this step.

    Attributes:
        target: ``(4, 4)`` hand target in the base frame, or ``None`` before any
            clutch has ever been engaged.
        enabled: ``True`` while the clutch drives the target; ``False`` means
            "hold where you are".
        clutched: Whether the clutch is currently engaged.
        gripper_rad: Gripper angle to command, or ``None``.
        engaged_now: The clutch was engaged on this step.
        released_now: The clutch was released on this step.
        tracked: Whether the controller was tracked this step.
        engage_distance_m: Absolute mapping, controller tracked and clutch not
            engaged: how far the mapped target is from the current hand, i.e.
            how far the operator's hand is from the robot's.  ``None`` otherwise.
        engage_radius_m: The gate radius in force this step, if any.
        waiting: The clutch is requested but the hand is not yet within the
            engage radius; nothing moves until it is.
    """

    target: NDArray[np.float64] | None
    enabled: bool
    clutched: bool
    gripper_rad: float | None = None
    engaged_now: bool = False
    released_now: bool = False
    tracked: bool = True
    engage_distance_m: float | None = None
    engage_radius_m: float | None = None
    waiting: bool = False


class ArmTeleop:
    """Clutch-based teleoperation of one arm (absolute or relative mapping)."""

    def __init__(self, side: str, config: ArmTeleopConfig | None = None) -> None:
        """Bind the teleoperator to one side.

        Args:
            side: ``"left"`` or ``"right"``; used only for reporting.
            config: Tunables; defaults otherwise.
        """
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'.")
        self.side = side
        self.config = config or ArmTeleopConfig()
        self._clutched = False
        self._controller0: NDArray[np.float64] | None = None
        self._hand0: NDArray[np.float64] | None = None
        self._target: NDArray[np.float64] | None = None
        self._last_time: float | None = None
        self._robot_anchor: NDArray[np.float64] | None = None
        self._operator_anchor: NDArray[np.float64] | None = None

    @property
    def clutched(self) -> bool:
        """Whether the clutch is engaged."""
        return self._clutched

    @property
    def target(self) -> NDArray[np.float64] | None:
        """The latched hand target, if any."""
        return None if self._target is None else self._target.copy()

    @property
    def anchor(self) -> Tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        """``(robot_anchor_m, operator_anchor_m)`` of the absolute mapping, if set."""
        if self._robot_anchor is None or self._operator_anchor is None:
            return None
        return self._robot_anchor.copy(), self._operator_anchor.copy()

    def set_anchor(self, robot_anchor_m: Any, operator_anchor_m: Any) -> None:
        """Make two points correspond: one on the robot, one on the operator.

        The absolute mapping sends the controller position ``p`` (operator
        frame) to the hand target ``robot + scale * (p - operator)``.  The
        natural choice is the robot's head (its camera, say) for ``robot`` and
        the headset position at re-centring for ``operator``: the operator
        then stands "inside" the robot, and their hands and the robot's agree.

        Args:
            robot_anchor_m: ``(3,)`` point in the robot's base frame.
            operator_anchor_m: ``(3,)`` point in the operator frame.
        """
        robot = np.asarray(robot_anchor_m, dtype=np.float64).reshape(3)
        operator = np.asarray(operator_anchor_m, dtype=np.float64).reshape(3)
        if not (np.all(np.isfinite(robot)) and np.all(np.isfinite(operator))):
            raise ValueError("Anchor points must be finite.")
        self._robot_anchor = robot.copy()
        self._operator_anchor = operator.copy()

    def release(self) -> None:
        """Disengage the clutch (the target stays latched as a hold)."""
        self._clutched = False
        self._controller0 = None
        self._hand0 = None
        self._last_time = None

    def update(
        self,
        sample: ControllerSample | None,
        current_hand_pose: NDArray[np.float64],
        now_s: float,
    ) -> ArmCommand:
        """Advance one step.

        Args:
            sample: The controller reading, or ``None`` when the controller is
                absent.  An untracked controller releases the clutch: motion
                must never continue on a pose that is no longer measured.
            current_hand_pose: ``(4, 4)`` measured hand pose in the base frame,
                used as the anchor when the clutch engages.
            now_s: Monotonic time, seconds.

        Returns:
            The :class:`ArmCommand` for this step.

        Raises:
            RuntimeError: Absolute mapping with no anchor (see :meth:`set_anchor`).
        """
        c = self.config
        if c.mapping == "absolute" and self._robot_anchor is None:
            raise RuntimeError(
                f"{self.side} arm: absolute mapping needs set_anchor() before update()."
            )
        gripper = None if sample is None else c.gripper_target(sample.trigger)
        if sample is None or sample.pose is None:
            released = self._clutched
            self.release()
            return ArmCommand(
                target=self.target,
                enabled=False,
                clutched=False,
                gripper_rad=gripper,
                released_now=released,
                tracked=False,
            )

        pose = np.asarray(sample.pose, dtype=np.float64)
        engaged_now = released_now = False
        # The gate: where the absolute mapping would send the hand, and how
        # far that is from where the hand is, while the clutch is not engaged.
        radius = sample.engage_radius_m if sample.engage_radius_m is not None else c.engage_radius_m
        distance: float | None = None
        if c.mapping == "absolute" and not self._clutched:
            distance = self._engage_distance(pose, current_hand_pose)
        if radius is not None and c.mapping != "absolute":
            radius = None  # the relative mapping never jumps; nothing to gate
        if sample.clutch and not self._clutched and radius is not None and distance is not None:
            if distance > radius:
                return ArmCommand(
                    target=self.target,
                    enabled=False,
                    clutched=False,
                    gripper_rad=gripper,
                    engage_distance_m=distance,
                    engage_radius_m=radius,
                    waiting=True,
                )
        if sample.clutch and not self._clutched:
            self._clutched = True
            self._controller0 = pose.copy()
            self._hand0 = np.asarray(current_hand_pose, dtype=np.float64).copy()
            self._target = self._hand0.copy()
            self._last_time = now_s
            engaged_now = True
        elif not sample.clutch and self._clutched:
            self.release()
            released_now = True
            if c.mapping == "absolute":
                distance = self._engage_distance(pose, current_hand_pose)

        if not self._clutched:
            return ArmCommand(
                target=self.target,
                enabled=False,
                clutched=False,
                gripper_rad=gripper,
                released_now=released_now,
                engage_distance_m=distance,
                engage_radius_m=radius,
            )

        assert self._controller0 is not None and self._hand0 is not None
        assert self._target is not None
        desired = np.eye(4)
        if c.mapping == "absolute":
            assert self._robot_anchor is not None and self._operator_anchor is not None
            desired[:3, 3] = self._robot_anchor + c.position_scale * (
                pose[:3, 3] - self._operator_anchor
            )
        else:
            desired[:3, 3] = self._hand0[:3, 3] + c.position_scale * (
                pose[:3, 3] - self._controller0[:3, 3]
            )
        if c.orientation_enabled:
            desired[:3, :3] = pose[:3, :3] @ self._controller0[:3, :3].T @ self._hand0[:3, :3]
        else:
            desired[:3, :3] = self._hand0[:3, :3]
        if c.workspace_min_m is not None and c.workspace_max_m is not None:
            desired[:3, 3] = np.clip(
                desired[:3, 3], np.asarray(c.workspace_min_m), np.asarray(c.workspace_max_m)
            )

        dt = 0.0 if self._last_time is None else max(0.0, now_s - self._last_time)
        self._last_time = now_s
        previous = self._target
        if dt > 0.0:
            # Slew towards the desired pose: the operator can move faster than
            # the machine should, and a tracking glitch must not become a leap.
            dp = desired[:3, 3] - previous[:3, 3]
            norm = float(np.linalg.norm(dp))
            max_dp = c.max_speed_m_s * dt
            if norm > max_dp:
                dp *= max_dp / norm
            R_prev = _orthonormalize(previous[:3, :3])
            axis, angle = axis_angle_of(desired[:3, :3] @ R_prev.T)
            max_angle = c.max_angular_speed_rad_s * dt
            if angle > max_angle:
                # Part of the way, about the same axis.  Re-orthonormalised so
                # rounding cannot compound from one step into the next.
                R_new = _orthonormalize(_rotation_about(axis, max_angle) @ R_prev)
            else:
                R_new = desired[:3, :3]
            new = np.eye(4)
            new[:3, 3] = previous[:3, 3] + dp
            new[:3, :3] = R_new
            self._target = new
        elif engaged_now:
            self._target = self._hand0.copy()
        return ArmCommand(
            target=self.target,
            enabled=True,
            clutched=True,
            gripper_rad=gripper,
            engaged_now=engaged_now,
        )

    def _engage_distance(
        self, pose: NDArray[np.float64], current_hand_pose: NDArray[np.float64]
    ) -> float:
        """How far the absolute mapping would move the hand if the clutch engaged now."""
        c = self.config
        assert self._robot_anchor is not None and self._operator_anchor is not None
        mapped = self._robot_anchor + c.position_scale * (pose[:3, 3] - self._operator_anchor)
        current = np.asarray(current_hand_pose, dtype=np.float64)[:3, 3]
        return float(np.linalg.norm(mapped - current))

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly state."""
        target = None
        if self._target is not None:
            target = {
                "p": [float(v) for v in self._target[:3, 3]],
                "q": list(matrix_to_quat_xyzw(self._target[:3, :3])),
            }
        return {
            "side": self.side,
            "mapping": self.config.mapping,
            "anchor": None
            if self._robot_anchor is None or self._operator_anchor is None
            else {
                "robot_m": [float(v) for v in self._robot_anchor],
                "operator_m": [float(v) for v in self._operator_anchor],
            },
            "clutched": self._clutched,
            "target": target,
            "position_scale": self.config.position_scale,
            "orientation_enabled": self.config.orientation_enabled,
            "gripper_available": self.config.gripper_available,
            "engage_radius_m": self.config.engage_radius_m,
        }


def _orthonormalize(R: NDArray[np.float64]) -> NDArray[np.float64]:
    """Nearest proper rotation to ``R`` (polar decomposition via SVD)."""
    U, _, Vt = np.linalg.svd(np.asarray(R, dtype=np.float64))
    out = U @ Vt
    if np.linalg.det(out) < 0.0:
        U[:, -1] *= -1.0
        out = U @ Vt
    return out


def _rotation_about(axis: NDArray[np.float64], angle: float) -> NDArray[np.float64]:
    """Rodrigues' rotation about an axis (normalised here)."""
    k = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(k))
    if norm < 1e-12:
        return np.eye(3)
    k = k / norm
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


@dataclass(frozen=True)
class DualArmTeleopOutput:
    """Both arms' commands for one step.

    Attributes:
        target: The :class:`~robopy.control.types.DualArmTarget` to hand to the
            solver.  Hands whose clutch is not engaged are held.
        gripper_targets_rad: ``{motor: rad}`` for the grippers with measured
            travel; empty otherwise.
        commands: Per-side :class:`ArmCommand`.
    """

    target: DualArmTarget
    gripper_targets_rad: Dict[str, float]
    commands: Dict[str, ArmCommand]


class DualArmTeleop:
    """Both controllers -> one :class:`~robopy.control.types.DualArmTarget`."""

    def __init__(
        self,
        left: ArmTeleopConfig | None = None,
        right: ArmTeleopConfig | None = None,
        *,
        torso_policy: TorsoPolicy = TorsoPolicy.FIXED,
        target_ttl_s: float = 0.25,
    ) -> None:
        """Create both arm teleoperators.

        Args:
            left: Left arm tunables.
            right: Right arm tunables.
            torso_policy: How the solver treats the shared torso yaw.
            target_ttl_s: How long a produced target stays valid.  A consumer
                must not act on an expired one, so a stalled stream stops the
                arms rather than freezing a stale command in place.
        """
        if target_ttl_s <= 0.0:
            raise ValueError("target_ttl_s must be positive.")
        self.arms = {"left": ArmTeleop("left", left), "right": ArmTeleop("right", right)}
        self.torso_policy = torso_policy
        self.target_ttl_s = target_ttl_s

    @property
    def needs_anchor(self) -> bool:
        """Whether either arm uses the absolute mapping (and so needs an anchor)."""
        return any(arm.config.mapping == "absolute" for arm in self.arms.values())

    def set_anchor(self, robot_anchor_m: Any, operator_anchor_m: Any) -> None:
        """Set both arms' anchor; see :meth:`ArmTeleop.set_anchor`."""
        for arm in self.arms.values():
            arm.set_anchor(robot_anchor_m, operator_anchor_m)

    def release_all(self) -> None:
        """Disengage both clutches, e.g. when the operator's stream stops."""
        for arm in self.arms.values():
            arm.release()

    def update(
        self,
        samples: Mapping[str, ControllerSample | None],
        hand_poses: Mapping[str, NDArray[np.float64]],
        now_s: float,
    ) -> DualArmTeleopOutput:
        """Advance both arms one step.

        Args:
            samples: ``{"left"|"right": sample or None}``.
            hand_poses: Measured ``(4, 4)`` hand poses in the base frame.
            now_s: Monotonic time, seconds.

        Returns:
            The combined output.
        """
        commands = {
            side: arm.update(samples.get(side), hand_poses[side], now_s)
            for side, arm in self.arms.items()
        }
        grippers: Dict[str, float] = {}
        for side, command in commands.items():
            motor = self.arms[side].config.gripper_motor
            if command.gripper_rad is not None and motor is not None:
                grippers[motor] = command.gripper_rad
        left, right = commands["left"], commands["right"]
        now_ns = monotonic_ns()
        target = DualArmTarget(
            left_target=left.target if left.enabled else None,
            right_target=right.target if right.enabled else None,
            left_enabled=left.enabled,
            right_enabled=right.enabled,
            torso_policy=self.torso_policy,
            created_ns=now_ns,
            expiry_ns=now_ns + int(self.target_ttl_s * 1e9),
        )
        return DualArmTeleopOutput(target=target, gripper_targets_rad=grippers, commands=commands)

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly state of both arms."""
        return {side: arm.describe() for side, arm in self.arms.items()}
