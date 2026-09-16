"""Pick a cube up off the table with the Rakuda's right hand.

Registered as ``rakuda.lift_cube``.  This is the one bundled task that needs a
gripper, so it runs on ``rakuda_gripper`` rather than ``rakuda`` -- and the
gripper is borrowed from CALVIN's Panda, because the Rakuda's CAD export has no
gripper geometry at all.  :mod:`robopy.sim.panda_gripper` says what that does
and does not mean; the short version is that success here says something about
the task and very little about the real machine.

The cube is 30 mm, which the jaw can hold: its pads sit 8 mm apart closed and
88 mm apart open, so anything from 8 mm up to about 80 mm is graspable in
principle.  Measured on this model, a 30 mm cube held in the proximal two
thirds of the fingers stays put through a 0.9 rad wrist swing, and one held near
the tips works loose -- so the reward encourages closing *on* the cube rather
than merely touching it.
"""

from __future__ import annotations

import torch
from metasim.constants import PhysicStateType
from metasim.scenario.objects import PrimitiveCubeCfg
from metasim.scenario.scenario import ScenarioCfg
from metasim.scenario.simulator_params import SimParamCfg
from metasim.task.base import BaseTaskEnv
from metasim.task.registry import register_task

from ._common import OBJECT_ZONE, RakudaMount, hand_position

__all__ = ["RakudaLiftCubeEnv"]

ROBOT = "rakuda_gripper"
PEDESTAL = "rakuda_mount"
MOUNT = RakudaMount()

TABLE_DEPTH = 0.34
TABLE_WIDTH = 0.60

#: Cube edge. Comfortably inside the jaw's 8 to 88 mm range, and small enough
#: that the fingers reach round it rather than pushing it over.
CUBE_SIZE = 0.03

#: How far above the table the cube has to be raised to count as lifted.
LIFT_HEIGHT_M = 0.06

#: How close the hand has to be for the cube to count as in the hand, rather
#: than merely knocked upwards.
HAND_HOLD_RADIUS_M = 0.09


@register_task("rakuda.lift_cube")
class RakudaLiftCubeEnv(BaseTaskEnv):
    """Close the right hand on the cube and lift it clear of the table.

    Success needs the cube both raised by :data:`LIFT_HEIGHT_M` *and* still
    within :data:`HAND_HOLD_RADIUS_M` of the hand.  Either alone is easy to get
    by accident: a swept arm can flick a cube into the air, and a hand can hover
    over one it never picked up.
    """

    supported_simulators = ("mujoco",)
    max_episode_steps = 500

    scenario = ScenarioCfg(
        objects=[
            MOUNT.pedestal(PEDESTAL),
            MOUNT.table("table", depth=TABLE_DEPTH, width=TABLE_WIDTH),
            PrimitiveCubeCfg(
                name="cube",
                size=(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
                mass=0.03,
                color=(0.20, 0.70, 0.35),
                physics=PhysicStateType.RIGIDBODY,
            ),
        ],
        robots=[ROBOT],
        simulator="mujoco",
        sim_params=SimParamCfg(dt=0.005),
        decimation=4,
        num_envs=1,
        headless=True,
    )

    def __init__(self, scenario=None, device=None) -> None:
        self._start_z: torch.Tensor | None = None
        super().__init__(scenario, device)

    # -- setup ------------------------------------------------------------- #

    def _sample_layout(self, generator: torch.Generator | None = None) -> torch.Tensor:
        unit = torch.rand(self.num_envs, 2, generator=generator)
        return torch.stack(
            [
                OBJECT_ZONE["x"][0] + unit[:, 0] * (OBJECT_ZONE["x"][1] - OBJECT_ZONE["x"][0]),
                OBJECT_ZONE["y"][0] + unit[:, 1] * (OBJECT_ZONE["y"][1] - OBJECT_ZONE["y"][0]),
                torch.full((self.num_envs,), MOUNT.work_surface_z + CUBE_SIZE / 2.0),
            ],
            dim=-1,
        )

    def _get_initial_states(self) -> list[dict]:
        start = self._sample_layout()
        self._start_z = start[:, 2].clone()
        robot = self.scenario.robots[0]
        upright = torch.tensor([1.0, 0.0, 0.0, 0.0])
        return [
            {
                "objects": {
                    PEDESTAL: {
                        "pos": torch.tensor(list(MOUNT.pedestal_position())),
                        "rot": upright.clone(),
                    },
                    "table": {
                        "pos": torch.tensor(list(MOUNT.table_position(TABLE_DEPTH))),
                        "rot": upright.clone(),
                    },
                    "cube": {"pos": start[env].clone(), "rot": upright.clone()},
                },
                "robots": {
                    ROBOT: {
                        "pos": torch.tensor(list(MOUNT.base_position)),
                        "rot": upright.clone(),
                        # Hands open: the task is to close one on the cube.
                        "dof_pos": dict(robot.default_joint_positions),
                    }
                },
                "cameras": {},
                "extras": {},
            }
            for env in range(self.num_envs)
        ]

    # -- task -------------------------------------------------------------- #

    def _cube_position(self, states) -> torch.Tensor:
        return states.objects["cube"].root_state[:, 0:3]

    def _hand_to_cube(self, states) -> torch.Tensor:
        return torch.linalg.norm(
            hand_position(states, "right", ROBOT) - self._cube_position(states), dim=-1
        )

    def lift(self, states) -> torch.Tensor:
        """How far the cube has risen from where it started, metres."""
        if self._start_z is None:  # pragma: no cover - set by _get_initial_states
            raise RuntimeError("the layout has not been sampled; reset() the task first")
        return self._cube_position(states)[:, 2] - self._start_z.to(self.device)

    def in_hand(self, states) -> torch.Tensor:
        """Whether the cube is close enough to the hand to be held by it."""
        return self._hand_to_cube(states) < HAND_HOLD_RADIUS_M

    def _reward(self, states) -> torch.Tensor:
        """Get to the cube, then get it off the table.

        The lift term is gated on the cube still being at the hand, so batting
        it across the table and watching it bounce earns nothing.
        """
        approach = -self._hand_to_cube(states)
        raised = torch.clamp(self.lift(states), min=0.0)
        return approach + 5.0 * torch.where(self.in_hand(states), raised, torch.zeros_like(raised))

    def _terminated(self, states) -> torch.Tensor:
        """True once the cube is up by :data:`LIFT_HEIGHT_M` and still in the hand."""
        return (self.lift(states) > LIFT_HEIGHT_M) & self.in_hand(states)

    def reset(self, states=None, env_ids=None, seed=None):
        """Resample where the cube sits, then reset as usual."""
        if states is None:
            self._initial_states = self._get_initial_states()
        return super().reset(states, env_ids, seed)

    @property
    def mount(self) -> RakudaMount:
        """Where the robot is standing, and the heights that follow from it."""
        return MOUNT
