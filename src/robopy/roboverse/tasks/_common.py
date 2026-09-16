"""Facts about the Rakuda's workspace that the tasks are built on.

Every number here was measured off the exported model rather than chosen, and
``tests/test_roboverse/`` recomputes each one and fails if a re-export moves it.
They live in one place because the robot's geometry is unusually constraining
and each task would otherwise rediscover the same limits.

The shape of the problem:

* The CAD origin is at the waist, so a Rakuda at ``z = 0`` is buried to the
  chest.  Its base body sits :data:`~robopy.roboverse.mount.STAND_HEIGHT` above
  whatever it is bolted to.
* Its shoulders are 0.41 m above its own base plate and each arm is about 0.30 m
  long, so **a hand never gets closer than 0.112 m to the mounting plane**.  The
  robot cannot reach the surface it stands on.
* So it goes on a pedestal, positioned relative to the work surface rather than
  to the floor -- see :mod:`robopy.roboverse.mount`, which is where the heights
  come from and why.
* Forward reach runs out around ``x = 0.365``, and the pedestal occupies ``x``
  out to ``0.178``, which leaves a usable strip of table roughly
  ``x in [0.21, 0.36]``.

Positions here are given **relative to the mounting plane** wherever height is
involved, because that is the frame in which they are constants.  A task turns
them into world coordinates with its :class:`~robopy.roboverse.mount.RakudaMount`.

The robot faces ``+x``: the head camera's own frame points along
``(0.923, -0.370, 0.104)``.
"""

from __future__ import annotations

from typing import Dict, Tuple

from robopy.roboverse.mount import (
    HAND_FLOOR_ABOVE_MOUNT,
    PLATE_CENTRE_XY,
    PLATE_SIZE_XY,
    STAND_HEIGHT,
    WORK_OFFSET_ABOVE_MOUNT,
    RakudaMount,
)

__all__ = [
    "GRIPPER_BODY",
    "HAND_FLOOR_ABOVE_MOUNT",
    "MAX_PALM_REACH_M",
    "OBJECT_ZONE",
    "PEDESTAL_FOOTPRINT",
    "REACH_TARGET_BOX",
    "STAND_HEIGHT",
    "WORK_OFFSET_ABOVE_MOUNT",
    "RakudaMount",
    "hand_position",
]

#: Body whose origin is the hand frame, per side.  These are the CAD's own
#: ``gripper_*_dof`` frames -- fixed frames despite the ``_dof`` in the name.
GRIPPER_BODY: Dict[str, str] = {
    "left": "gripper_left_dof",
    "right": "gripper_right_dof",
}

#: Furthest the hand ever gets from the ``base`` body, in any direction.
#:
#: From 200,000 samples of the seven joints the arm scripts drive, on the
#: ``rakuda_gripper`` model.  It is a hard ceiling rather than a working figure:
#: nothing beyond this sphere can be touched at all, at any height or angle, so
#: it is the cheapest way to rule a target out.  What can be *worked* on is much
#: smaller -- see :data:`OBJECT_ZONE`.
MAX_PALM_REACH_M: float = 0.507

#: Extent of the base plate in the robot's frame, as ``(min, max)`` per axis.
#: A table has to clear this in ``x`` or it runs into the pedestal.
PEDESTAL_FOOTPRINT: Dict[str, Tuple[float, float]] = {
    "x": (
        PLATE_CENTRE_XY[0] - PLATE_SIZE_XY[0] / 2.0,
        PLATE_CENTRE_XY[0] + PLATE_SIZE_XY[0] / 2.0,
    ),
    "y": (
        PLATE_CENTRE_XY[1] - PLATE_SIZE_XY[1] / 2.0,
        PLATE_CENTRE_XY[1] + PLATE_SIZE_XY[1] / 2.0,
    ),
}

#: Where each hand is asked to reach, as ``(min, max)`` per axis, with ``z``
#: **relative to the mounting plane**.
#:
#: Each box sits over its own shoulder, which is where that arm is happiest even
#: though the waist joint lets either hand cross the centre line.  About 6% of
#: random joint configurations land a hand inside its box, so the targets are
#: comfortably reachable without being trivially so.
REACH_TARGET_BOX: Dict[str, Dict[str, Tuple[float, float]]] = {
    "left": {"x": (-0.08, 0.12), "y": (0.05, 0.25), "z": (0.30, 0.50)},
    "right": {"x": (-0.12, 0.08), "y": (-0.30, -0.10), "z": (0.30, 0.50)},
}

#: Where an object may sit on the work surface and still be reachable: ``x`` and
#: ``y`` in the robot's frame, on a table whose top is at the work surface.
#:
#: Measured at the default offset, in front of the pedestal: a hand lands inside
#: this box in about 1.3% of random joint configurations -- and the 2nd
#: percentile of the reachable strip starts at ``x = 0.200``, so the near bound
#: is deliberately well clear of it.
OBJECT_ZONE: Dict[str, Tuple[float, float]] = {
    "x": (0.24, 0.32),
    "y": (-0.12, 0.12),
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
