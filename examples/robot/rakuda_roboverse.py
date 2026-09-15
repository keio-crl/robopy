"""Put the Rakuda into a RoboVerse scene and drive it.

Nothing in the RoboVerse checkout needs editing: installing robopy registers its
MetaSim content pack through the ``metasim.packages`` entry point, so
``get_robot("rakuda")`` and the ``rakuda.*`` task names resolve on their own.

Run it inside the environment RoboVerse is installed in::

    pip install -e ".[roboverse]"          # from the robopy checkout

    python examples/robot/rakuda_roboverse.py --demo wave
    python examples/robot/rakuda_roboverse.py --demo task --task rakuda.reach
    python examples/robot/rakuda_roboverse.py --demo task --task rakuda.push_cube --video out.mp4

``--demo wave`` builds a bare scene and sweeps a few joints, which is the
shortest thing that proves the import worked.  ``--demo task`` runs one of the
registered tasks with a hand-written proportional controller -- not a policy,
just something that moves towards the goal so the episode does something visible.

What the robot can and cannot do: the CAD export models both grippers as *fixed*
frames, so there are 15 actuated joints and no fingers.  The tasks are built
around that -- reaching and pushing, no grasping.
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import Dict, List

try:
    import torch
    from metasim.scenario.cameras import PinholeCameraCfg
    from metasim.scenario.scenario import ScenarioCfg
    from metasim.scenario.simulator_params import SimParamCfg
    from metasim.task.registry import get_task_class
    from metasim.utils.setup_util import get_handler
except ImportError as exc:  # pragma: no cover - depends on the environment
    raise SystemExit(
        f"this example needs RoboVerse/MetaSim in the current environment ({exc}).\n"
        "From a robopy checkout:  pip install -e '.[roboverse]'"
    ) from exc

from robopy.roboverse.robots import RAKUDA_ARM_JOINTS, RAKUDA_STAND_HEIGHT_M

ROBOT = "rakuda"


def _camera() -> PinholeCameraCfg:
    """Look at the robot's chest from its front-left.

    The Rakuda faces ``+x`` (its head camera's own frame points that way) and is
    about 0.6 m tall standing on the floor, so this frames the whole machine.
    """
    return PinholeCameraCfg(
        width=960,
        height=720,
        pos=(0.9, -0.7, 0.75),
        look_at=(0.0, 0.0, 0.35),
    )


def demo_wave(args: argparse.Namespace) -> int:
    """Build a bare scene and sweep the torso and both elbows."""
    scenario = ScenarioCfg(
        robots=[ROBOT],
        objects=[],
        simulator="mujoco",
        num_envs=1,
        headless=args.headless,
        sim_params=SimParamCfg(dt=0.005),
        decimation=4,
    )
    if args.video:
        scenario.cameras = [_camera()]

    robot = scenario.robots[0]
    print(f"robot   : {robot.name}, {robot.num_joints} joints")
    print(f"asset   : {robot.mjcf_path}")
    print(f"          ({robot.asset_source})")

    handler = get_handler(scenario)
    handler.set_states(
        [
            {
                "objects": {},
                "robots": {
                    ROBOT: {
                        "pos": torch.tensor([0.0, 0.0, RAKUDA_STAND_HEIGHT_M]),
                        "rot": torch.tensor([1.0, 0.0, 0.0, 0.0]),
                        "dof_pos": dict(robot.default_joint_positions),
                    }
                },
            }
        ]
    )

    saver = _make_saver(args.video)
    if saver is not None:
        saver.add(handler.get_states(mode="tensor"))

    joints = handler.get_joint_names(ROBOT, sort=True)
    for step in range(args.steps):
        phase = 2.0 * math.pi * step / max(args.steps - 1, 1)
        targets = dict(robot.default_joint_positions)
        targets["torso_yaw_dof"] = 0.6 * math.sin(phase)
        targets["elbow_pitch_right_dof"] = -0.5 - 0.6 * (0.5 - 0.5 * math.cos(phase))
        targets["elbow_pitch_left_dof"] = 0.5 + 0.6 * (0.5 - 0.5 * math.cos(phase))
        targets = _clamp(targets, robot.joint_limits)
        handler.set_dof_targets([{ROBOT: {"dof_pos_target": targets}}])
        handler.simulate()
        if saver is not None and step % 2 == 0:
            saver.add(handler.get_states(mode="tensor"))

    states = handler.get_states(mode="tensor")
    reached = states.robots[ROBOT].joint_pos[0]
    print("\nfinal joint angles:")
    for name, angle in zip(joints, reached.tolist()):
        print(f"  {name:26} {angle:+.4f}")

    if saver is not None:
        saver.save()
        print(f"\nwrote {args.video}")
    handler.close()
    return 0


def demo_task(args: argparse.Namespace) -> int:
    """Run a registered task with a crude proportional controller.

    The controller is not the point and is not a solution to the task: it nudges
    the arm's joints in whichever direction shortens the distance to the goal,
    measured by finite differences.  It is here so the episode visibly does
    something without dragging in a policy.
    """
    task_class = get_task_class(args.task)
    scenario = task_class.scenario.update(num_envs=1, headless=args.headless)
    if args.video:
        scenario.cameras = [_camera()]
    env = task_class(scenario=scenario, device="cpu")

    robot = scenario.robots[0]
    joints = env.handler.get_joint_names(ROBOT, sort=True)
    states, _ = env.reset(seed=args.seed)

    saver = _make_saver(args.video)
    if saver is not None:
        saver.add(states)

    start = [
        _clamp_one(robot.default_joint_positions[name], robot.joint_limits[name])
        for name in joints
    ]
    action = torch.tensor([start])
    driven = [joints.index(name) for name in RAKUDA_ARM_JOINTS["right"] if name in joints]

    best = float("-inf")
    for step in range(args.steps):
        states, reward, terminated, timeout, _ = env.step(action)
        value = float(reward[0])
        best = max(best, value)
        if saver is not None and step % 2 == 0:
            saver.add(states)
        if bool(terminated[0]):
            print(f"succeeded at step {step}")
            break
        # Coordinate descent: try one joint per step, keep the change if it helped.
        index = driven[step % len(driven)]
        probe = action.clone()
        probe[0, index] += args.probe if (step // len(driven)) % 2 == 0 else -args.probe
        probe[0, index] = _clamp_one(float(probe[0, index]), robot.joint_limits[joints[index]])
        trial_states, trial_reward, *_ = env.step(probe)
        if float(trial_reward[0]) > value:
            action = probe
            states = trial_states
        if saver is not None and step % 2 == 0:
            saver.add(states)
    else:
        print(f"ran {args.steps} steps without reaching the goal")

    print(f"task    : {args.task}")
    print(f"reward  : best {best:.4f}, final {float(reward[0]):.4f}")
    if hasattr(env, "targets"):
        for hand, target in env.targets.items():
            print(f"target[{hand}] = {[round(v, 3) for v in target[0].tolist()]}")

    if saver is not None:
        saver.save()
        print(f"wrote {args.video}")
    env.close()
    return 0


def _make_saver(path: str | None):
    if not path:
        return None
    from metasim.utils.obs_utils import ObsSaver

    return ObsSaver(video_path=path)


def _clamp_one(value: float, limits) -> float:
    low, high = limits
    return min(max(value, low + 1e-3), high - 1e-3)


def _clamp(targets: Dict[str, float], limits) -> Dict[str, float]:
    return {name: _clamp_one(value, limits[name]) for name, value in targets.items()}


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--demo", choices=("wave", "task"), default="wave")
    parser.add_argument("--task", default="rakuda.reach", help="task name for --demo task")
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--probe", type=float, default=0.08, help="coordinate-descent step, rad")
    parser.add_argument("--video", default=None, help="write an mp4 here")
    parser.add_argument(
        "--headless",
        action="store_true",
        default=True,
        help="no on-screen viewer (the default; there is usually no display)",
    )
    parser.add_argument("--gui", dest="headless", action="store_false")
    args = parser.parse_args(argv)

    return demo_wave(args) if args.demo == "wave" else demo_task(args)


if __name__ == "__main__":
    sys.exit(main())
