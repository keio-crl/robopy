"""The Rakuda with a hand on each arm, for tasks that need to hold something.

``get_robot("rakuda_gripper")`` resolves to this through the same entry point as
the plain robot.  It is the same arms, torso and head -- the same fifteen
joints, in the same order -- with a parallel jaw added to each gripper frame,
bringing the model to nineteen.

**The hand is borrowed, not this machine's.**  The Rakuda's CAD export models
both grippers as fixed frames with no geometry at all, so the fingers here come
from CALVIN's ``panda_longer_finger``.  Read
:mod:`robopy.sim.panda_gripper` before drawing conclusions from anything this
robot does with an object: the finger shape, the stroke and the grip force are
all invented, and even where they are mounted rests on a TCP offset this
repository records as unmeasured.

Use :class:`~robopy.roboverse.robots.RakudaCfg` when the task does not need to
grasp.  That one is the robot the CAD actually describes.
"""

from __future__ import annotations

from typing import Dict, Tuple

from metasim.scenario.robot import BaseActuatorCfg
from metasim.utils import configclass

from robopy.roboverse.assets import resolve_rakuda_mjcf
from robopy.sim.panda_gripper import (
    FINGER_STROKE_M,
    GRIP_FORCE_N,
    GRIPPER_ACTUATORS,
    PAD_GAP_CLOSED_M,
    PAD_GAP_OPEN_M,
    gripper_targets,
    pad_gap,
)

from .rakuda_cfg import RAKUDA_ACTUATED_JOINTS, RakudaCfg, home_pose, joint_limits_from_mjcf

__all__ = [
    "RAKUDA_FINGER_JOINTS",
    "RAKUDA_GRIPPER_JOINTS",
    "RakudaGripperCfg",
    "gripper_targets",
    "pad_gap",
]

#: The finger joints, per hand.
RAKUDA_GRIPPER_JOINTS: Dict[str, Tuple[str, ...]] = {
    side: tuple(joints) for side, joints in GRIPPER_ACTUATORS.items()
}

#: All four finger joints, in model order: right hand then left, matching the
#: convention the arm joints follow.
RAKUDA_FINGER_JOINTS: Tuple[str, ...] = tuple(
    RAKUDA_GRIPPER_JOINTS["left"] + RAKUDA_GRIPPER_JOINTS["right"]
)

#: Every joint of this robot, arms first so that the first fifteen entries mean
#: exactly what they mean on :class:`RakudaCfg`.
RAKUDA_GRIPPER_ALL_JOINTS: Tuple[str, ...] = RAKUDA_ACTUATED_JOINTS + RAKUDA_FINGER_JOINTS


@configclass
class RakudaGripperCfg(RakudaCfg):
    """Rakuda-2 with a borrowed parallel jaw on each arm.

    Nineteen joints: the fifteen of :class:`RakudaCfg`, then two per hand.  The
    two fingers of a hand are tied together by an equality constraint in the
    model, so despite having an actuator each they are one degree of freedom --
    which is what the real follower bus has, one XM430-W350 per gripper.
    :func:`~robopy.sim.panda_gripper.gripper_targets` sets both at once and is
    the intended way to command a hand.

    Attributes:
        gripper_open_q: Per-finger travel that opens a hand fully.
        gripper_close_q: Per-finger travel that closes it.  Note that closed is
            not *shut*: the pads still leave
            :data:`~robopy.sim.panda_gripper.PAD_GAP_CLOSED_M`, so the jaw
            cannot pinch anything thinner than 8 mm.
    """

    name: str = "rakuda_gripper"
    num_joints: int = len(RAKUDA_GRIPPER_ALL_JOINTS)

    ee_body_name: str = "right_gripper_base"

    gripper_joint_name: str | None = RAKUDA_GRIPPER_JOINTS["right"][0]
    gripper_open_q: list[float] | None = None
    gripper_close_q: list[float] | None = None

    def __post_init__(self) -> None:
        """Resolve the gripper model, then extend the arm's tables with the fingers."""
        if self.mjcf_path is None:
            path, source = resolve_rakuda_mjcf("gripper")
            self.mjcf_path = str(path)
            self.asset_source = source
        if self.gripper_open_q is None:
            self.gripper_open_q = [FINGER_STROKE_M, FINGER_STROKE_M]
        if self.gripper_close_q is None:
            self.gripper_close_q = [0.0, 0.0]

        super().__post_init__()

        # super() filled the fifteen arm joints in; add the fingers.
        limits = joint_limits_from_mjcf(self.mjcf_path, RAKUDA_GRIPPER_ALL_JOINTS)
        self.joint_limits = limits
        actuators = dict(self.actuators or {})
        control = dict(self.control_type or {})
        for joint in RAKUDA_FINGER_JOINTS:
            actuators[joint] = BaseActuatorCfg(
                velocity_limit=0.2,
                effort_limit_sim=GRIP_FORCE_N,
                fully_actuated=True,
                is_ee=True,
            )
            control[joint] = "position"
        self.actuators = actuators
        self.control_type = control
        # Start with the hands open: a task that wants to grasp has to close
        # them, and a hand that starts shut hides whether it ever opened.
        self.default_joint_positions = {
            **home_pose({k: limits[k] for k in RAKUDA_ACTUATED_JOINTS}),
            **gripper_targets(FINGER_STROKE_M),
        }


#: Re-exported so a task need not import the exporter to know what the jaw does.
GRIPPER_STROKE_M = FINGER_STROKE_M
GRIPPER_PAD_GAP_CLOSED_M = PAD_GAP_CLOSED_M
GRIPPER_PAD_GAP_OPEN_M = PAD_GAP_OPEN_M
