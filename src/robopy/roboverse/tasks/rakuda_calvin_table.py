"""The Rakuda at CALVIN's play table: once faithfully, once so it can work.

Two tasks share this scene -- CALVIN's scene D, the play table with its sliding
door, drawer, button and switch, plus the three coloured blocks, all at CALVIN's
own ``global_scaling`` of 0.8.

``rakuda.calvin_table``
    A Rakuda bolted exactly where ``calvin_scene_D.yaml`` bolts its Panda.
    **It cannot reach the table from there, and that is the point.**  The
    Rakuda's palm never gets further than 0.507 m from its base body, and from
    ``robot_base_position`` the nearest block is 0.519 m away -- out of reach by
    a centimetre at the very best, and by much more at the height the bench
    actually sits.  This task is for seeing the two robots at one scale, not for
    solving.

``rakuda.calvin_pick``
    The same table and the same blocks, with the Rakuda mounted where it can
    work: turned to face the bench, and standing at the height its own grasping
    envelope wants rather than the Panda's.  The red block then sits inside the
    measured :data:`~robopy.roboverse.tasks._common.OBJECT_ZONE` and the robot
    picks it up.  See :data:`CALVIN_PICK_BASE_POSITION` for where the numbers
    come from.

Everything else is ``calvin_scene_D.yaml`` unchanged, so the layout matches what
``calvin_env`` instantiates.  The scale is baked into the exported MJCF rather
than asked of the simulator, because MetaSim's ``scale`` does not reach an
``ArticulationObjCfg``.
"""

from __future__ import annotations

import torch
from metasim.constants import PhysicStateType
from metasim.scenario.objects import ArticulationObjCfg, PrimitiveCubeCfg
from metasim.scenario.scenario import ScenarioCfg
from metasim.scenario.simulator_params import SimParamCfg
from metasim.task.base import BaseTaskEnv
from metasim.task.registry import register_task

from robopy.roboverse.mount import (
    GRASP_OFFSET_ABOVE_MOUNT,
    PLATE_CENTRE_XY,
    PLATE_SIZE_XY,
    STAND_HEIGHT,
)
from robopy.sim.calvin_table import (
    CALVIN_SCALE,
    CALVIN_TABLE_SURFACE,
    CALVIN_WORK_SURFACE_Z,
    find_calvin_table,
)

from ._common import OBJECT_ZONE, hand_position

__all__ = [
    "BLOCK_MASS_KG",
    "CALVIN_BLOCKS",
    "CALVIN_PICK_BASE_POSITION",
    "CALVIN_PICK_TARGET",
    "PANDA_BASE_POSITION",
    "RakudaAtCalvinTableEnv",
    "RakudaCalvinPickEnv",
    "block_rest_positions",
]

ROBOT = "rakuda"
TABLE = "calvin_table"
PEDESTAL = "rakuda_mount"

#: ``robot_base_position`` from ``calvin_scene_D.yaml``, unchanged.
#:
#: In CALVIN this is where the Panda's base link sits, which is its mounting
#: surface.  The Rakuda's ``base`` body is its waist, so the model goes
#: :data:`~robopy.roboverse.mount.STAND_HEIGHT` higher to put its *feet* here.
#: The scene file's ``robot_base_orientation`` is ``[0, 0, 0]``, so the identity
#: rotation used below is CALVIN's own and not a choice made here.
PANDA_BASE_POSITION = (-0.34, -0.46, 0.24)

#: Where the play table goes, also from the scene file.
TABLE_POSITION = (0.0, 0.0, 0.0)

#: The three movable blocks of ``calvin_scene_D``, as ``size, colour`` taken
#: from the URDFs named there -- ``block_red_middle``, ``block_blue_small`` and
#: ``block_pink_big``.  They are box primitives with flat colours upstream too,
#: so nothing is approximated; the sizes are pre-scaling, like the table's.
CALVIN_BLOCKS = {
    "block_red": ((0.07, 0.05, 0.05), (1.0, 0.0, 0.0)),
    "block_blue": ((0.05, 0.05, 0.05), (0.0, 0.0, 1.0)),
    "block_pink": ((0.10, 0.05, 0.05), (1.0, 0.0, 1.0)),
}

#: Every block URDF gives its mass as 1 kg, and PyBullet's ``globalScaling``
#: shrinks geometry while leaving mass alone, so CALVIN's scaled blocks really
#: are that dense.  It is passed on for the backends that read it.
#:
#: **MuJoCo does not.**  Measured off the compiled model, a block comes out at
#: 0.0896 kg -- its volume times MuJoCo's default density of 1000 -- so on this
#: backend the blocks weigh about a tenth of what CALVIN's do.  Stated here
#: because it is the sort of thing that quietly explains why a grasp holds.
BLOCK_MASS_KG = 1.0

#: Where in CALVIN's drop rectangle each block starts, as ``(x, y)`` fractions.
#: Spread rather than random: this scene is a picture, and one that changes on
#: every reset is harder to hold against CALVIN's own.
_BLOCK_SPOTS = ((0.15, 0.25), (0.50, 0.75), (0.85, 0.35))

#: Clear air under a block at reset, so it settles onto the bench instead of
#: starting a millimetre inside it.
_SETTLE_GAP = 0.002


def _table_cfg() -> ArticulationObjCfg:
    """The play table, as an articulated object MetaSim can drive."""
    table = find_calvin_table()
    if table is None:
        raise FileNotFoundError(
            "the CALVIN play table is missing from the models directory; it is vendored "
            "under calvin_table/, so this is a broken checkout"
        )
    mjcf = table / "mjcf" / "calvin_table.xml"
    if not mjcf.is_file():
        raise FileNotFoundError(
            f"{mjcf} has not been generated; run python -m robopy.sim.calvin_table"
        )
    # No `scale=` here on purpose.  MetaSim's MuJoCo handler loads an
    # articulation's MJCF as it finds it, so a scale set here goes nowhere and
    # the table would come out full size under scaled block positions -- which
    # is exactly how the blocks ended up inside it.  The export bakes CALVIN's
    # global_scaling in instead.
    return ArticulationObjCfg(
        name=TABLE,
        mjcf_path=str(mjcf),
        urdf_path=str(table / "urdf" / "calvin_table_D.urdf"),
        fix_base_link=True,
    )


def _pedestal_cfg() -> PrimitiveCubeCfg:
    """A box holding the robot up to the Panda's base height.

    CALVIN's Panda stands on nothing: ``robot_base_position`` is 0.24 m above
    the floor with no geometry under it, because the mount is not part of the
    scene.  Drawing it keeps the Rakuda from appearing to hover.  It clears the
    drawer, which travels at z 0.30 to 0.40, well above this box's top.
    """
    return PrimitiveCubeCfg(
        name=PEDESTAL,
        size=(PLATE_SIZE_XY[0] + 0.04, PLATE_SIZE_XY[1] + 0.04, PANDA_BASE_POSITION[2]),
        color=(0.32, 0.34, 0.38),
        physics=PhysicStateType.GEOM,
        fix_base_link=True,
    )


def _blocks() -> list[PrimitiveCubeCfg]:
    return [
        PrimitiveCubeCfg(
            name=name,
            size=tuple(v * CALVIN_SCALE for v in size),
            mass=BLOCK_MASS_KG,
            color=color,
            physics=PhysicStateType.RIGIDBODY,
        )
        for name, (size, color) in CALVIN_BLOCKS.items()
    ]


def block_rest_positions() -> dict[str, tuple[float, float, float]]:
    """Where each block starts: spread over the bench, resting on its top.

    The heights come from :data:`~robopy.sim.calvin_table.CALVIN_WORK_SURFACE_Z`
    and each block's own size, not from the scene file's 0.46 -- that is a
    *drop* height, two centimetres of clear air above the bench.
    """
    (x0, y0), (x1, y1) = CALVIN_TABLE_SURFACE
    spots = {}
    for (name, (size, _)), (fx, fy) in zip(CALVIN_BLOCKS.items(), _BLOCK_SPOTS):
        half_height = size[2] * CALVIN_SCALE / 2.0
        spots[name] = (
            x0 + (x1 - x0) * fx,
            y0 + (y1 - y0) * fy,
            CALVIN_WORK_SURFACE_Z + half_height + _SETTLE_GAP,
        )
    return spots


@register_task("rakuda.calvin_table")
class RakudaAtCalvinTableEnv(BaseTaskEnv):
    """CALVIN's scene D with a Rakuda in the Panda's place.

    There is no goal: :meth:`_terminated` is always false and the reward is
    constant.  The table's four joints can be driven through the usual
    ``dof_pos`` route if you want to open the drawer or slide the door, and the
    blocks are ordinary rigid bodies.
    """

    supported_simulators = ("mujoco",)
    max_episode_steps = 500

    scenario = ScenarioCfg(
        objects=[_table_cfg(), _pedestal_cfg(), *_blocks()],
        robots=[ROBOT],
        simulator="mujoco",
        sim_params=SimParamCfg(dt=0.005),
        decimation=4,
        num_envs=1,
        headless=True,
    )

    def _get_initial_states(self) -> list[dict]:
        robot = self.scenario.robots[0]
        upright = torch.tensor([1.0, 0.0, 0.0, 0.0])

        objects = {
            TABLE: {
                "pos": torch.tensor(list(TABLE_POSITION)),
                "rot": upright.clone(),
                # Everything shut, as calvin_scene_D starts it.
                "dof_pos": {
                    "base__button": 0.0,
                    "base__switch": 0.0,
                    "base__slide": 0.0,
                    "base__drawer": 0.0,
                },
            },
            PEDESTAL: {
                "pos": torch.tensor(
                    [
                        PANDA_BASE_POSITION[0] + PLATE_CENTRE_XY[0],
                        PANDA_BASE_POSITION[1] + PLATE_CENTRE_XY[1],
                        PANDA_BASE_POSITION[2] / 2.0,
                    ]
                ),
                "rot": upright.clone(),
            },
        }
        for name, spot in block_rest_positions().items():
            objects[name] = {"pos": torch.tensor(list(spot)), "rot": upright.clone()}

        base = (
            PANDA_BASE_POSITION[0],
            PANDA_BASE_POSITION[1],
            PANDA_BASE_POSITION[2] + STAND_HEIGHT,
        )
        return [
            {
                "objects": objects,
                "robots": {
                    ROBOT: {
                        "pos": torch.tensor(list(base)),
                        "rot": upright.clone(),
                        "dof_pos": dict(robot.default_joint_positions),
                    }
                },
                "cameras": {},
                "extras": {},
            }
            for _ in range(self.num_envs)
        ]

    def _reward(self, states) -> torch.Tensor:
        """No goal, so no reward. Zero rather than something arbitrary."""
        return torch.zeros(self.num_envs, device=self.device)

    def _terminated(self, states) -> torch.Tensor:
        """Never: there is nothing here to succeed at."""
        return torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)


# --------------------------------------------------------------------------- #
# The same table, with the Rakuda mounted where it can actually work.
# --------------------------------------------------------------------------- #

GRIPPER_ROBOT = "rakuda_gripper"
PICK_PEDESTAL = "rakuda_work_mount"

#: Which block the robot goes for.  Only one can sit in the middle of the
#: reachable zone at a time, and the red one is nearest the bench's centre.
CALVIN_PICK_TARGET = "block_red"

#: How far in front of the robot the target block sits, in the robot's own
#: frame.  The middle of :data:`~robopy.roboverse.tasks._common.OBJECT_ZONE`,
#: which is the strip of table a hand can actually come down onto.
_PICK_REACH_X = (OBJECT_ZONE["x"][0] + OBJECT_ZONE["x"][1]) / 2.0

#: Yaw putting the robot's ``+x`` -- the way it faces -- along world ``+y``, so
#: it looks at the front of the bench.  CALVIN's Panda meets the table from the
#: corner; the Rakuda has to square up to it because its grasping strip is
#: narrow in ``y``.
_FACING_THE_BENCH = (0.7071067811865476, 0.0, 0.0, 0.7071067811865476)

#: Height of the robot's feet for this task.
#:
#: Not CALVIN's 0.24.  A downward grasp only works well below this robot's
#: shoulders -- see :data:`~robopy.roboverse.mount.GRASP_OFFSET_ABOVE_MOUNT` --
#: so the bench has to sit that far above the mounting plane, which puts the
#: feet *higher* than the Panda's base rather than lower.
CALVIN_PICK_FEET_Z = CALVIN_WORK_SURFACE_Z - GRASP_OFFSET_ABOVE_MOUNT

#: How far the block has to come up, and how near the hand has to stay, for the
#: pick to count.  The thresholds ``rakuda.lift_block`` uses.
PICK_LIFT_HEIGHT_M = 0.06
PICK_HOLD_RADIUS_M = 0.09


def _pick_base_position() -> tuple[float, float, float]:
    """Where the robot's feet go, derived rather than chosen.

    Facing world ``+y``, a point at ``(x, y)`` in the robot's frame lands at
    ``(-y, x)`` in the world.  Putting the target block at ``(_PICK_REACH_X, 0)``
    in the robot's frame therefore means standing that far behind it along ``y``.
    """
    block_x, block_y, _ = block_rest_positions()[CALVIN_PICK_TARGET]
    return (block_x, block_y - _PICK_REACH_X, CALVIN_PICK_FEET_Z)


#: Where the Rakuda's feet sit for ``rakuda.calvin_pick``.
CALVIN_PICK_BASE_POSITION = _pick_base_position()


def _pick_pedestal_cfg() -> PrimitiveCubeCfg:
    """The stand, from the floor up to the robot's feet."""
    return PrimitiveCubeCfg(
        name=PICK_PEDESTAL,
        size=(PLATE_SIZE_XY[0] + 0.04, PLATE_SIZE_XY[1] + 0.04, CALVIN_PICK_FEET_Z),
        color=(0.32, 0.34, 0.38),
        physics=PhysicStateType.GEOM,
        fix_base_link=True,
    )


@register_task("rakuda.calvin_pick")
class RakudaCalvinPickEnv(BaseTaskEnv):
    """Pick CALVIN's red block off CALVIN's bench.

    Same furniture and same blocks as :class:`RakudaAtCalvinTableEnv`; what
    differs is where the robot stands, which way it faces, and that it has
    hands.  Success needs the block both raised by :data:`PICK_LIFT_HEIGHT_M`
    *and* still within :data:`PICK_HOLD_RADIUS_M` of the hand, so knocking it
    off the bench does not count.
    """

    supported_simulators = ("mujoco",)
    max_episode_steps = 900

    scenario = ScenarioCfg(
        objects=[_table_cfg(), _pick_pedestal_cfg(), *_blocks()],
        robots=[GRIPPER_ROBOT],
        simulator="mujoco",
        sim_params=SimParamCfg(dt=0.005),
        decimation=4,
        num_envs=1,
        headless=True,
    )

    def __init__(self, scenario=None, device=None) -> None:
        self._start_z: torch.Tensor | None = None
        super().__init__(scenario, device)

    def _get_initial_states(self) -> list[dict]:
        robot = self.scenario.robots[0]
        upright = torch.tensor([1.0, 0.0, 0.0, 0.0])
        facing = torch.tensor(list(_FACING_THE_BENCH))
        spots = block_rest_positions()
        self._start_z = torch.full((self.num_envs,), spots[CALVIN_PICK_TARGET][2])

        objects = {
            TABLE: {
                "pos": torch.tensor(list(TABLE_POSITION)),
                "rot": upright.clone(),
                "dof_pos": dict.fromkeys(
                    ("base__button", "base__switch", "base__slide", "base__drawer"), 0.0
                ),
            },
            PICK_PEDESTAL: {
                "pos": torch.tensor(
                    [
                        # The plate is off-centre in the robot's frame, and the
                        # robot is turned, so its offset turns with it.
                        CALVIN_PICK_BASE_POSITION[0] - PLATE_CENTRE_XY[1],
                        CALVIN_PICK_BASE_POSITION[1] + PLATE_CENTRE_XY[0],
                        CALVIN_PICK_FEET_Z / 2.0,
                    ]
                ),
                "rot": upright.clone(),
            },
        }
        for name, spot in spots.items():
            objects[name] = {"pos": torch.tensor(list(spot)), "rot": upright.clone()}

        base = (
            CALVIN_PICK_BASE_POSITION[0],
            CALVIN_PICK_BASE_POSITION[1],
            CALVIN_PICK_BASE_POSITION[2] + STAND_HEIGHT,
        )
        return [
            {
                "objects": objects,
                "robots": {
                    GRIPPER_ROBOT: {
                        "pos": torch.tensor(list(base)),
                        "rot": facing.clone(),
                        "dof_pos": dict(robot.default_joint_positions),
                    }
                },
                "cameras": {},
                "extras": {},
            }
            for _ in range(self.num_envs)
        ]

    def _block_position(self, states) -> torch.Tensor:
        return states.objects[CALVIN_PICK_TARGET].root_state[:, 0:3]

    def _hand_to_block(self, states) -> torch.Tensor:
        return torch.linalg.norm(
            hand_position(states, "right", GRIPPER_ROBOT) - self._block_position(states), dim=-1
        )

    def lift(self, states) -> torch.Tensor:
        """How far the block has risen from where it started, metres."""
        if self._start_z is None:  # pragma: no cover - set by _get_initial_states
            raise RuntimeError("the layout has not been set; reset() the task first")
        return self._block_position(states)[:, 2] - self._start_z.to(self.device)

    def in_hand(self, states) -> torch.Tensor:
        """Whether the block is close enough to the hand to be held by it."""
        return self._hand_to_block(states) < PICK_HOLD_RADIUS_M

    def _reward(self, states) -> torch.Tensor:
        """Get to the block, then get it off the bench."""
        approach = -self._hand_to_block(states)
        return approach + 10.0 * self.lift(states).clamp(min=0.0) * self.in_hand(states)

    def _terminated(self, states) -> torch.Tensor:
        """True once the block is up by :data:`PICK_LIFT_HEIGHT_M` and still held."""
        return (self.lift(states) > PICK_LIFT_HEIGHT_M) & self.in_hand(states)
