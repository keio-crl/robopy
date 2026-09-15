"""Reaching tasks for the Rakuda: put a hand on a point in space.

``rakuda.reach`` drives one hand to a sampled target; ``rakuda.bimanual_reach``
drives both at once.  Neither needs a gripper, which matters here: the CAD
export models the Rakuda's two grippers as *fixed* frames, so there is nothing
to open or close and no grasping to be had.  Reaching is the largest useful task
the model supports as exported.

Targets are sampled inside :data:`~robopy.roboverse.tasks._common.REACH_TARGET_BOX`,
a per-hand box measured from the model's own reachable set, so a failure means
the policy missed rather than that it was sent somewhere the arm cannot go.

The target is a marker, not an obstacle: it is drawn where the hand should go
and has collision turned off, so an arm on its way to the goal is never pushed
off course by the goal itself.
"""

from __future__ import annotations

import torch
from metasim.constants import PhysicStateType
from metasim.scenario.objects import PrimitiveSphereCfg
from metasim.scenario.scenario import ScenarioCfg
from metasim.scenario.simulator_params import SimParamCfg
from metasim.task.base import BaseTaskEnv
from metasim.task.registry import register_task

from ._common import REACH_TARGET_BOX, STAND_HEIGHT, hand_position

__all__ = ["RakudaBimanualReachEnv", "RakudaReachEnv"]

ROBOT = "rakuda"

#: How close the hand frame has to get, in metres.
#:
#: The hand frame is the CAD's ``gripper_*_dof`` frame, and the offset from it
#: to an actual grasp point is a measurement nobody has taken (it is still
#: ``null`` in ``examples/config/rakuda_control.example.yaml``).  5 cm is a
#: tolerance that stays meaningful despite that unknown; tighten it once the
#: real tool-centre offset is measured.
SUCCESS_RADIUS_M: float = 0.05

_MARKER_RADIUS = 0.02


def _marker(name: str, color: tuple[float, float, float]) -> PrimitiveSphereCfg:
    """A goal marker: visible, and deliberately not collidable."""
    return PrimitiveSphereCfg(
        name=name,
        radius=_MARKER_RADIUS,
        color=color,
        physics=PhysicStateType.XFORM,
        fix_base_link=True,
        collision_enabled=False,
    )


def _sample_in_box(box, num_envs: int, generator: torch.Generator | None) -> torch.Tensor:
    """Uniform samples in an axis-aligned box, as ``[num_envs, 3]``."""
    low = torch.tensor([box[axis][0] for axis in "xyz"])
    high = torch.tensor([box[axis][1] for axis in "xyz"])
    unit = torch.rand(num_envs, 3, generator=generator)
    return low + unit * (high - low)


class _ReachBase(BaseTaskEnv):
    """Shared machinery: sample targets, measure distance, reward, succeed."""

    #: Hands this task drives, and the marker that belongs to each.
    hands: tuple[str, ...] = ()
    markers: dict[str, str] = {}

    supported_simulators = ("mujoco",)
    max_episode_steps = 250

    def __init__(self, scenario=None, device=None) -> None:
        self._targets: dict[str, torch.Tensor] = {}
        super().__init__(scenario, device)

    # -- setup ------------------------------------------------------------ #

    def _sample_targets(self, generator: torch.Generator | None = None) -> None:
        for hand in self.hands:
            self._targets[hand] = _sample_in_box(REACH_TARGET_BOX[hand], self.num_envs, generator)

    def _get_initial_states(self) -> list[dict]:
        if not self._targets:
            self._sample_targets()
        robot = self.scenario.robots[0]
        states = []
        for env in range(self.num_envs):
            objects = {
                self.markers[hand]: {
                    "pos": self._targets[hand][env].clone(),
                    "rot": torch.tensor([1.0, 0.0, 0.0, 0.0]),
                }
                for hand in self.hands
            }
            states.append(
                {
                    "objects": objects,
                    "robots": {
                        ROBOT: {
                            "pos": torch.tensor([0.0, 0.0, STAND_HEIGHT]),
                            "rot": torch.tensor([1.0, 0.0, 0.0, 0.0]),
                            "dof_pos": dict(robot.default_joint_positions),
                        }
                    },
                    "cameras": {},
                    "extras": {},
                }
            )
        return states

    # -- task -------------------------------------------------------------- #

    def _distances(self, states) -> torch.Tensor:
        """Hand-to-target distance per hand, as ``[num_envs, len(hands)]``."""
        columns = []
        for hand in self.hands:
            target = self._targets[hand].to(self.device)
            columns.append(torch.linalg.norm(hand_position(states, hand, ROBOT) - target, dim=-1))
        return torch.stack(columns, dim=-1)

    def _reward(self, states) -> torch.Tensor:
        """Shaped: ``-distance``, summed over the hands this task drives.

        Dense rather than sparse because the success radius is a 5 cm ball in a
        workspace tens of centimetres across; a sparse reward would almost never
        fire from a random policy.
        """
        return -self._distances(states).sum(dim=-1)

    def _terminated(self, states) -> torch.Tensor:
        """True once *every* driven hand is inside :data:`SUCCESS_RADIUS_M`."""
        return (self._distances(states) < SUCCESS_RADIUS_M).all(dim=-1)

    def reset(self, states=None, env_ids=None, seed=None):
        """Resample the targets, then reset as usual."""
        if states is None:
            generator = None
            if seed is not None:
                generator = torch.Generator().manual_seed(int(seed))
            self._sample_targets(generator)
            self._initial_states = self._get_initial_states()
        return super().reset(states, env_ids, seed)

    @property
    def targets(self) -> dict[str, torch.Tensor]:
        """The current per-hand goal positions, ``[num_envs, 3]`` each."""
        return {hand: value.clone() for hand, value in self._targets.items()}


@register_task("rakuda.reach")
class RakudaReachEnv(_ReachBase):
    """Put the right hand on a sampled target.

    The left arm is not driven towards anything; its actuators still hold it at
    the starting pose, so it hangs where it began rather than going limp.
    """

    hands = ("right",)
    markers = {"right": "target_right"}

    scenario = ScenarioCfg(
        objects=[_marker("target_right", (0.85, 0.25, 0.25))],
        robots=[ROBOT],
        simulator="mujoco",
        sim_params=SimParamCfg(dt=0.005),
        decimation=4,
        num_envs=1,
        headless=True,
    )


@register_task("rakuda.bimanual_reach")
class RakudaBimanualReachEnv(_ReachBase):
    """Put both hands on their own sampled targets at the same time.

    Each target is sampled over its own shoulder, so the two are typically 30 cm
    or more apart and the arms are not asked to occupy the same space.  They can
    still collide on the way -- the model keeps self-collision between parts
    that are not parent and child -- which is the point of doing both at once.
    """

    hands = ("right", "left")
    markers = {"right": "target_right", "left": "target_left"}

    scenario = ScenarioCfg(
        objects=[
            _marker("target_right", (0.85, 0.25, 0.25)),
            _marker("target_left", (0.25, 0.45, 0.85)),
        ],
        robots=[ROBOT],
        simulator="mujoco",
        sim_params=SimParamCfg(dt=0.005),
        decimation=4,
        num_envs=1,
        headless=True,
    )
