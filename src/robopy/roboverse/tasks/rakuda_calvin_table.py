"""The Rakuda standing where CALVIN puts its Panda, at CALVIN's own play table.

Registered as ``rakuda.calvin_table``.  It is CALVIN's scene D -- the play table
with its sliding door, drawer, button and switch, plus the three coloured blocks
-- with a Rakuda bolted at ``robot_base_position`` from ``calvin_scene_D.yaml``
instead of the Panda.

**The Rakuda cannot reach this table from there, and that is expected.**  The
scene is built around a Panda, which works at 0.46 to 0.81 m from its base; the
Rakuda's hands never get further than 0.428 m from its own mounting plane, and
the nearest corner of CALVIN's work surface is 0.46 m away.  Standing clear of
the furniture it can cover about 32 of 40 interaction points, but not from
*this* spot.  This task exists to put the two in the same picture at the same
scale -- to see the size difference, to line a scene up, to start from -- not to
be solved.

For something the Rakuda can actually do, see ``rakuda.lift_block``, which puts
the work surface where its arms are.

Everything here is at CALVIN's own ``global_scaling`` of 0.8 and the positions
come from ``calvin_scene_D.yaml`` unchanged, so the layout matches what
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

from robopy.roboverse.mount import PLATE_CENTRE_XY, PLATE_SIZE_XY, STAND_HEIGHT
from robopy.sim.calvin_table import (
    CALVIN_SCALE,
    CALVIN_TABLE_SURFACE,
    CALVIN_WORK_SURFACE_Z,
    find_calvin_table,
)

__all__ = [
    "BLOCK_MASS_KG",
    "CALVIN_BLOCKS",
    "PANDA_BASE_POSITION",
    "RakudaAtCalvinTableEnv",
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

#: Every block URDF gives its mass as 1 kg.  PyBullet's ``globalScaling`` shrinks
#: geometry and leaves mass alone, so the scaled blocks really are that dense in
#: CALVIN, and matching it keeps contact behaviour comparable.
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
