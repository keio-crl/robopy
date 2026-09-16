"""Push a cube across a table with the Rakuda.

Registered as ``rakuda.push_cube``.  This is the object-manipulation task the
exported model can actually support: the CAD models both grippers as *fixed*
frames, so there is nothing to close on an object and picking anything up is out
of reach.  Pushing is not.

The scene is laid out the way CALVIN lays out its Franka: the robot is fixed at
a height chosen *relative to the work surface*, not stood on the floor.  It has
to be.  The Rakuda's shoulders are 0.41 m above its own base plate and its arms
are about 0.30 m long, so a hand never gets closer than 0.112 m to whatever the
robot is bolted to -- put it on the table and it waves above everything on it.

So it stands on a pedestal whose top is
:data:`~robopy.roboverse.mount.WORK_OFFSET_ABOVE_MOUNT` below the tabletop, which
is the height at which the hands work best, and the table sits in front of the
pedestal rather than under it.  :mod:`robopy.roboverse.mount` has the
measurements behind both.
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

__all__ = ["RakudaPushCubeEnv"]

ROBOT = "rakuda"
PEDESTAL = "rakuda_mount"

#: The robot's stand, and every height that follows from it.
MOUNT = RakudaMount()

#: Extent of the table along ``x``. It starts at ``MOUNT.near_edge_x`` so it
#: never overlaps the pedestal, and reaches past the arm's forward limit (0.365).
TABLE_DEPTH = 0.34
TABLE_WIDTH = 0.60

CUBE_SIZE = 0.04

#: Where the cube starts, in the robot's frame. Measured reachable; see
#: :data:`~robopy.roboverse.tasks._common.OBJECT_ZONE`.
CUBE_START_X = OBJECT_ZONE["x"]
CUBE_START_Y = OBJECT_ZONE["y"]

#: How far the cube has to be pushed, and how close counts as arriving.
GOAL_OFFSET_Y = 0.16
GOAL_RADIUS_M = 0.05


@register_task("rakuda.push_cube")
class RakudaPushCubeEnv(BaseTaskEnv):
    """Push the cube sideways into the goal region without knocking it off.

    The goal is offset in ``y`` rather than ``x``: the arm reaches out over the
    table and sweeps across it, which suits an arm that works off its own
    shoulder far better than asking it to push straight ahead at the limit of
    its forward reach.

    The episode ends when the cube is within :data:`GOAL_RADIUS_M` of the goal.
    Knocking the cube off the table does not end it -- the episode simply runs
    out, and :meth:`cube_on_table` tells the difference.
    """

    supported_simulators = ("mujoco",)
    max_episode_steps = 400

    scenario = ScenarioCfg(
        objects=[
            MOUNT.pedestal(PEDESTAL),
            MOUNT.table("table", depth=TABLE_DEPTH, width=TABLE_WIDTH),
            PrimitiveCubeCfg(
                name="cube",
                size=(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
                mass=0.05,
                color=(0.20, 0.70, 0.35),
                physics=PhysicStateType.RIGIDBODY,
            ),
            PrimitiveCubeCfg(
                name="goal",
                size=(GOAL_RADIUS_M * 2, GOAL_RADIUS_M * 2, 0.002),
                color=(0.90, 0.75, 0.20),
                physics=PhysicStateType.XFORM,
                fix_base_link=True,
                collision_enabled=False,
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
        self._goal: torch.Tensor | None = None
        super().__init__(scenario, device)

    # -- setup ------------------------------------------------------------- #

    def _sample_layout(self, generator: torch.Generator | None = None):
        """Cube start on the table, and a goal offset across it."""
        unit = torch.rand(self.num_envs, 2, generator=generator)
        start = torch.stack(
            [
                CUBE_START_X[0] + unit[:, 0] * (CUBE_START_X[1] - CUBE_START_X[0]),
                CUBE_START_Y[0] + unit[:, 1] * (CUBE_START_Y[1] - CUBE_START_Y[0]),
                torch.full((self.num_envs,), MOUNT.work_surface_z + CUBE_SIZE / 2.0),
            ],
            dim=-1,
        )
        # Always push towards the robot's right, which is where the right arm
        # has room to sweep. A goal on the other side would need the left arm.
        goal = start.clone()
        goal[:, 1] -= GOAL_OFFSET_Y
        goal[:, 2] = MOUNT.work_surface_z
        return start, goal

    def _get_initial_states(self) -> list[dict]:
        start, goal = self._sample_layout()
        self._goal = goal
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
                    "goal": {"pos": goal[env].clone(), "rot": upright.clone()},
                },
                "robots": {
                    ROBOT: {
                        "pos": torch.tensor(list(MOUNT.base_position)),
                        "rot": upright.clone(),
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

    def _cube_to_goal(self, states) -> torch.Tensor:
        if self._goal is None:  # pragma: no cover - _get_initial_states always sets it
            raise RuntimeError("the goal has not been sampled; reset() the task first")
        goal = self._goal.to(self.device)
        return torch.linalg.norm(self._cube_position(states)[:, :2] - goal[:, :2], dim=-1)

    def _hand_to_cube(self, states) -> torch.Tensor:
        return torch.linalg.norm(
            hand_position(states, "right", ROBOT) - self._cube_position(states), dim=-1
        )

    def cube_on_table(self, states) -> torch.Tensor:
        """Whether the cube is still up on the table, as ``[num_envs]`` of bool."""
        return self._cube_position(states)[:, 2] > MOUNT.work_surface_z - CUBE_SIZE

    def _reward(self, states) -> torch.Tensor:
        """Reach the cube first, then move it to the goal.

        The hand-to-cube term is what gets a policy off the ground: until
        something touches the cube it never moves, so a reward built only on
        cube-to-goal distance is flat everywhere a random policy visits.  It is
        weighted below the cube term so that, once contact is made, pushing
        matters more than hugging.
        """
        approach = -0.2 * self._hand_to_cube(states)
        progress = -self._cube_to_goal(states)
        fell = torch.where(
            self.cube_on_table(states), torch.zeros_like(progress), torch.full_like(progress, -1.0)
        )
        return approach + progress + fell

    def _terminated(self, states) -> torch.Tensor:
        """True once the cube is in the goal region and still on the table."""
        return (self._cube_to_goal(states) < GOAL_RADIUS_M) & self.cube_on_table(states)

    def reset(self, states=None, env_ids=None, seed=None):
        """Resample the cube start and goal, then reset as usual."""
        if states is None:
            self._initial_states = self._get_initial_states()
        return super().reset(states, env_ids, seed)

    @property
    def goal(self) -> torch.Tensor | None:
        """Where the cube has to end up, ``[num_envs, 3]``."""
        return None if self._goal is None else self._goal.clone()

    @property
    def mount(self) -> RakudaMount:
        """Where the robot is standing, and the heights that follow from it."""
        return MOUNT
