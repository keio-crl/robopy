"""Facts about the Rakuda's workspace that the tasks are built on.

Every number here was measured off the exported model rather than chosen: the
script that produced them is ``tests/test_roboverse_rakuda.py``, which
recomputes each one and fails if a re-export moves it.  They are collected in
one place because the robot's geometry is unusually constraining and each task
would otherwise rediscover the same limits.

The shape of the problem:

* The CAD origin sits at the waist, so a Rakuda at ``z = 0`` is buried to the
  chest.  It stands on :data:`STAND_HEIGHT`.
* The pedestal it is bolted to occupies :data:`PEDESTAL_FOOTPRINT` and reaches
  up to ``z = 0.323``, so anything the robot is meant to touch has to clear it.
* The arms are short relative to that pedestal: a hand gets no lower than
  ``z = 0.114``.  **The robot cannot reach the surface it stands on.**  Objects
  have to sit on something raised, which is what :data:`WORK_SURFACE_TOP` is.
* Forward reach runs out around ``x = 0.37``, and each hand works best off its
  own shoulder rather than in front of the chest.

The robot faces ``+x``: the head camera's own frame points along
``(0.923, -0.370, 0.104)``.
"""

from __future__ import annotations

from typing import Dict, Tuple

__all__ = [
    "GRIPPER_BODY",
    "PEDESTAL_FOOTPRINT",
    "REACH_TARGET_BOX",
    "STAND_HEIGHT",
    "WORK_SURFACE_TOP",
    "hand_position",
]

#: Height the base body sits at so the robot stands on ``z = 0``.
STAND_HEIGHT: float = 0.25752

#: Body whose origin is the hand frame, per side.  These are the CAD's own
#: ``gripper_*_dof`` frames -- fixed frames despite the ``_dof`` in the name.
GRIPPER_BODY: Dict[str, str] = {
    "left": "gripper_left_dof",
    "right": "gripper_right_dof",
}

#: World-frame ``(min, max)`` per axis of everything welded to the base.
#: Nothing a task asks the robot to touch should be inside this.
PEDESTAL_FOOTPRINT: Dict[str, Tuple[float, float]] = {
    "x": (-0.122, 0.178),
    "y": (-0.178, 0.122),
    "z": (0.0, 0.323),
}

#: Top of the surface the object tasks put things on, in world ``z``.
#:
#: Chosen by measurement: sampling the joint ranges, roughly 7.5% of poses put a
#: hand in the 10 cm band above this height *and* clear of the pedestal, against
#: 2.7% at ``z = 0.15``.  Higher would be easier still, but the surface has to
#: stay below the chest to be worth calling a table.
WORK_SURFACE_TOP: float = 0.22

#: Where each hand is asked to reach, as world-frame ``(min, max)`` per axis.
#:
#: Each box sits over its own shoulder, which is where that arm actually works
#: -- the left hand's reachable cloud has its median at ``y = +0.15`` and the
#: right hand's at ``y = -0.21``.  About 6% of random joint configurations land
#: a hand inside its box, so the targets are comfortably reachable without being
#: trivially so.
REACH_TARGET_BOX: Dict[str, Dict[str, Tuple[float, float]]] = {
    "left": {"x": (-0.08, 0.12), "y": (0.05, 0.25), "z": (0.30, 0.50)},
    "right": {"x": (-0.12, 0.08), "y": (-0.30, -0.10), "z": (0.30, 0.50)},
}


def hand_position(states, hand: str, robot_name: str = "rakuda"):
    """World position of one hand, as ``[num_envs, 3]``.

    Args:
        states: A ``TensorState`` from ``reset`` or ``step``.
        hand: ``"left"`` or ``"right"``.
        robot_name: Name the robot was given in the scenario.

    Returns:
        The hand frame's world position.

    Raises:
        KeyError: If ``hand`` is not a side, or the model has no such body --
            which would mean the asset and this module have drifted apart.
    """
    if hand not in GRIPPER_BODY:
        raise KeyError(f"hand must be 'left' or 'right', not {hand!r}")
    robot = states.robots[robot_name]
    body = GRIPPER_BODY[hand]
    names = list(robot.body_names)
    if body not in names:
        raise KeyError(
            f"the {robot_name} model has no body {body!r}; it has {len(names)} bodies. "
            "Re-export the model with robopy.sim.mjcf_export."
        )
    return robot.body_state[:, names.index(body), 0:3]
