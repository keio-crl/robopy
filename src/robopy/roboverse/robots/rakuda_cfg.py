"""The Rakuda-2 as a RoboVerse / MetaSim robot.

``get_robot("rakuda")`` finds :class:`RakudaCfg` through the ``metasim.packages``
entry point robopy declares, so installing robopy is all it takes -- nothing in
the RoboVerse checkout needs editing.

The asset is the MJCF that :mod:`robopy.sim.mjcf_export` writes from the
committed CAD export.  Read that module before trusting any number here: the
export carries geometry, not dynamics, and the masses are CAD volumes scaled by
an assumed density.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Tuple
from xml.etree import ElementTree

from metasim.scenario.robot import BaseActuatorCfg, RobotCfg
from metasim.utils import configclass

from robopy.motor.dynamixel_control_table import get_motor_capabilities
from robopy.roboverse.assets import resolve_rakuda_mjcf
from robopy.roboverse.mount import STAND_HEIGHT
from robopy.sim.mjcf_export import (
    RAKUDA_ACTUATED_JOINTS,
    RAKUDA_BASE_BODY,
    RAKUDA_FRAME_SITES,
)

__all__ = [
    "RAKUDA_ARM_JOINTS",
    "home_pose",
    "joint_limits_from_mjcf",
    "RAKUDA_HEAD_JOINTS",
    "RAKUDA_SERVO_BY_JOINT",
    "RAKUDA_STAND_HEIGHT_M",
    "RAKUDA_TORSO_JOINT",
    "RakudaCfg",
]

#: Height the base has to sit at for the robot to stand on ``z = 0``.
#:
#: The CAD origin is at the waist, and the underside of the base plate is
#: 0.25752 m below it, so a Rakuda placed at the origin would be buried to the
#: chest.  Measured off the model's own collision geometry, not guessed; the test
#: suite recomputes it and fails if the model moves.
#:
#: Standing on the floor is rarely what you want, mind: the Rakuda's hands stop
#: 0.112 m above whatever it is bolted to, so on the floor it can reach nothing
#: on the floor.  :class:`robopy.roboverse.mount.RakudaMount` puts it on a
#: pedestal at a height derived from the work surface instead, which is what all
#: the bundled tasks do.
RAKUDA_STAND_HEIGHT_M: float = STAND_HEIGHT

#: The waist joint.  It turns the whole upper body, both arms with it.
RAKUDA_TORSO_JOINT: str = "torso_yaw_dof"

#: Head joints.  Modelled so the head collides and so a camera can be mounted
#: on it; the arm IK never drives them.
RAKUDA_HEAD_JOINTS: Tuple[str, ...] = ("head_yaw_dof", "head_pitch_dof")

#: Each arm, shoulder to wrist.  Same order as the ``left_arm_joints`` /
#: ``right_arm_joints`` lists in ``examples/config/rakuda_control.example.yaml``,
#: so a joint vector means the same thing in simulation and on the real robot.
RAKUDA_ARM_JOINTS: Dict[str, Tuple[str, ...]] = {
    "right": (
        "shoulder_pitch_right_dof",
        "shoulder_roll_right_dof",
        "elbow_yaw_right_dof",
        "elbow_pitch_right_dof",
        "wrist_yaw_right_dof",
        "wrist_pitch_right_dof",
    ),
    "left": (
        "shoulder_pitch_left_dof",
        "shoulder_roll_left_dof",
        "elbow_yaw_left_dof",
        "elbow_pitch_left_dof",
        "wrist_yaw_left_dof",
        "wrist_pitch_left_dof",
    ),
}

#: Which DYNAMIXEL drives each joint, as the follower bus has them.  The
#: exporter derives the same assignment from the CAD part names; this copy is
#: what the actuator limits below are built from.
RAKUDA_SERVO_BY_JOINT: Dict[str, str] = {
    "torso_yaw_dof": "xm540-w270",
    "head_yaw_dof": "xm430-w350",
    "head_pitch_dof": "xm430-w350",
    "shoulder_pitch_right_dof": "xm540-w270",
    "shoulder_roll_right_dof": "xm540-w270",
    "elbow_yaw_right_dof": "xm430-w350",
    "elbow_pitch_right_dof": "xm430-w350",
    "wrist_yaw_right_dof": "xm430-w350",
    "wrist_pitch_right_dof": "xm430-w350",
    "shoulder_pitch_left_dof": "xm540-w270",
    "shoulder_roll_left_dof": "xm540-w270",
    "elbow_yaw_left_dof": "xm430-w350",
    "elbow_pitch_left_dof": "xm430-w350",
    "wrist_yaw_left_dof": "xm430-w350",
    "wrist_pitch_left_dof": "xm430-w350",
}

#: Velocity limit applied to every joint, rad/s.
#:
#: The X-series servos report ``PRESENT_VELOCITY`` in units of 0.229 rev/min and
#: their no-load speeds are around 45 rev/min at 12 V, which is 4.7 rad/s.  What
#: is used here is deliberately slower: the CAD export's own ``velocity="1"`` is
#: an exporter placeholder, nothing on this machine has been clocked, and a
#: simulated robot that slews faster than the real one teaches a policy a habit
#: the hardware cannot repeat.  Raise it if you measure something better.
RAKUDA_VELOCITY_LIMIT_RAD_S: float = 3.0


def _actuators() -> Dict[str, BaseActuatorCfg]:
    """One actuator per joint, with the published stall torque as its limit."""
    actuators: Dict[str, BaseActuatorCfg] = {}
    for joint in RAKUDA_ACTUATED_JOINTS:
        capabilities = get_motor_capabilities(RAKUDA_SERVO_BY_JOINT[joint])
        actuators[joint] = BaseActuatorCfg(
            velocity_limit=RAKUDA_VELOCITY_LIMIT_RAD_S,
            effort_limit_sim=capabilities.stall_torque_nm,
            fully_actuated=True,
            is_ee=False,
        )
    return actuators


def joint_limits_from_mjcf(mjcf_path: str | Path) -> Dict[str, Tuple[float, float]]:
    """Read the joint ranges straight out of the model file.

    They are not repeated here as literals on purpose.  The ranges come from the
    CAD export, except for the three continuous joints, which the exporter gives
    the motor travel; keeping one copy means a re-export cannot leave this file
    quietly disagreeing with the robot the simulator actually loads.

    Args:
        mjcf_path: The model to read.

    Returns:
        Joint name -> ``(lower, upper)`` in radians, for the actuated joints.

    Raises:
        ValueError: If a joint in :data:`RAKUDA_ACTUATED_JOINTS` is missing from
            the model or carries no range.
    """
    root = ElementTree.parse(Path(mjcf_path)).getroot()
    found: Dict[str, Tuple[float, float]] = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        if name not in RAKUDA_ACTUATED_JOINTS:
            continue
        span = joint.get("range")
        if span is None:
            raise ValueError(f"{mjcf_path}: joint {name!r} has no range")
        lower, upper = (float(value) for value in span.split())
        found[name] = (lower, upper)

    missing = [joint for joint in RAKUDA_ACTUATED_JOINTS if joint not in found]
    if missing:
        raise ValueError(f"{mjcf_path} is missing the actuated joint(s) {missing}")
    return {joint: found[joint] for joint in RAKUDA_ACTUATED_JOINTS}


def home_pose(limits: Dict[str, Tuple[float, float]], margin_rad: float = 0.05) -> Dict[str, float]:
    """A safe pose to start from: the CAD zero, nudged inside the joint limits.

    The CAD's zero is the pose the machine was drawn in and is the natural
    starting point, but it is not usable as-is.  ``elbow_pitch_right_dof`` has
    the range ``(-2.7925, 0.0)``, so its zero sits exactly *on* a limit: a
    position servo commanded there fights the limit constraint from the first
    step, and any controller that clamps to the range has no room to move in one
    direction.  Clamping each joint to ``margin_rad`` inside its own range fixes
    that and leaves every other joint exactly at zero.

    This is a starting pose, not a "ready" or "home" pose in the sense the real
    robot's operators would mean -- nobody has defined one of those, and
    inventing a plausible-looking set of angles would be a guess dressed up as a
    configuration.  Tasks that want the arms somewhere specific should say so.

    Args:
        limits: Joint ranges, as :func:`joint_limits_from_mjcf` returns them.
        margin_rad: How far inside each limit to stay.

    Returns:
        Joint name -> angle in radians.
    """
    pose: Dict[str, float] = {}
    for joint, (lower, upper) in limits.items():
        low, high = lower + margin_rad, upper - margin_rad
        if low > high:  # a range narrower than twice the margin: take the middle
            low = high = 0.5 * (lower + upper)
        pose[joint] = min(max(0.0, low), high)
    return pose


@configclass
class RakudaCfg(RobotCfg):
    """Rakuda-2: a bolted-down torso with two 6-DOF arms and a 2-DOF head.

    Fifteen actuated joints, in :data:`RAKUDA_ACTUATED_JOINTS` order -- waist,
    head, right arm, left arm.  That is the order of the action vector.

    The grippers are **not** among them.  The real machine has two
    (``l_arm_grip`` and ``r_arm_grip`` on the follower bus), but the CAD export
    models them as *fixed* frames, so there is nothing to actuate here and no
    simulated grasping.  The ``gripper_left`` and ``gripper_right`` sites mark
    where a hand is; a task that needs to pick something up has to add a gripper
    to the model or fake the grasp with a weld.

    Attributes:
        mjcf_path: Filled in at construction from :func:`resolve_rakuda_mjcf`, so
            an installed wheel uses the self-contained model it ships and a
            repository checkout uses the one that draws the real visual meshes.
    """

    name: str = "rakuda"
    num_joints: int = len(RAKUDA_ACTUATED_JOINTS)

    mjcf_path: str | None = None
    """Set in ``__post_init__``; see :func:`resolve_rakuda_mjcf`."""

    # Redeclared from RobotCfg with their types, all filled in by __post_init__
    # below. They are derived from the model file rather than written out here,
    # so that the asset stays the single source of truth for them.
    actuators: Dict[str, BaseActuatorCfg] | None = None
    joint_limits: Dict[str, Tuple[float, float]] | None = None
    default_joint_positions: Dict[str, float] | None = None
    control_type: Dict[str, str] | None = None

    asset_source: str = ""
    """Where :attr:`mjcf_path` came from, for logging."""

    # Bolted to a bench, and the CAD origin is at the waist rather than the feet.
    fix_base_link: bool = True
    default_position: tuple[float, float, float] = (0.0, 0.0, RAKUDA_STAND_HEIGHT_M)

    enabled_gravity: bool = True

    enabled_self_collisions: str = "mujoco_default"
    """Keep MuJoCo's stock contact filtering. Neither ``True`` nor ``False`` is right here.

    ``True`` makes MetaSim disable ``filterparent`` globally, so a body collides
    with its own parent.  On this model that is ruinous: it is a CAD assembly of
    153 parts in which a cover is *meant* to sit over the servo it covers and a
    hinge horn inside its housing, so nearly every neighbouring pair
    interpenetrates by design.  Turning that into contact produces 264 contacts
    at rest, penetrations of up to 43 mm, and a robot whose servos saturate and
    stall against itself.

    ``False`` goes to the other extreme: MetaSim adds an exclude for every body
    pair in the robot -- 11 628 of them here -- and the arms can then pass
    through the torso and through each other.

    The sentinel leaves both knobs alone: parent-child pairs are filtered, which
    is what makes the assembly sane, and every other pair still collides, so the
    two arms really do stop at the torso and at one another.  It also makes the
    robot behave identically here and under a plain
    ``mujoco.MjModel.from_xml_path`` rollout, which is what the model was
    validated against.  The three pairs that overlap without being parent and
    child -- the waist thrust bearing and its races -- are excluded in the MJCF
    itself by :mod:`robopy.sim.mjcf_export`.
    """

    ee_body_name: str = "wrist_pitch_right"
    """A single "the" end effector is a poor fit for a two-armed robot; this
    names one so tools that insist on one get a sensible answer. Prefer the
    ``gripper_left`` / ``gripper_right`` sites."""

    def __post_init__(self) -> None:
        """Resolve the asset, then read the joint tables out of it."""
        if self.mjcf_path is None:
            path, source = resolve_rakuda_mjcf()
            self.mjcf_path = str(path)
            self.asset_source = source
        if self.actuators is None:
            self.actuators = _actuators()
        if self.control_type is None:
            self.control_type = dict.fromkeys(RAKUDA_ACTUATED_JOINTS, "position")
        if self.joint_limits is None:
            self.joint_limits = joint_limits_from_mjcf(self.mjcf_path)
        if self.default_joint_positions is None:
            self.default_joint_positions = home_pose(self.joint_limits)
        super().__post_init__()


#: Body that everything hangs off, re-exported so tasks need not import the
#: exporter to name it.
BASE_BODY = RAKUDA_BASE_BODY

#: Site name -> the CAD frame it marks.
FRAME_SITES = RAKUDA_FRAME_SITES
