"""Solve ``rakuda.lift_block``: reach for the block, close on it, pick it up.

Not a policy -- a scripted sequence with differential inverse kinematics, run
through the real task env so the reward and the success flag are the task's own.
It exists to show the environment works end to end, and to be a starting point
for something that learns.

    python examples/robot/rakuda_lift_block.py
    python examples/robot/rakuda_lift_block.py --video docs/robots/assets/lift.mp4
    python examples/robot/rakuda_lift_block.py --seed 3

The arm control -- differential IK on a scratch state, a straight-line hand
path, the finger direction constrained but not the roll -- lives in
:mod:`robopy.roboverse.ik`, along with the reasons each of those matters.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List

os.environ.setdefault("MUJOCO_GL", "egl")  # no DISPLAY on most machines

try:
    import mujoco
    import numpy as np
    from metasim.task.registry import get_task_class
except ImportError as exc:  # pragma: no cover - depends on the environment
    raise SystemExit(
        f"this example needs RoboVerse/MetaSim and MuJoCo ({exc}).\n"
        "From a robopy checkout:  pip install -e '.[sim]'"
    ) from exc

from robopy.roboverse.ik import Arm, ScriptedRun

TASK = "rakuda.lift_block"
ROBOT = "rakuda_gripper"

#: How far along the fingers to hold the block.
#:
#: Two constraints meet here.  The pads have to reach past the block's middle to
#: grip it, and the palm has to stay clear of the block's top face or the hand
#: drives into it on the way down.  The block is 100 mm tall, so anything under
#: 50 mm puts the palm inside it.
GRASP_DEPTH_M = 0.068


def _progress(env):
    """What this task reports after each leg of the path."""

    def note(states):
        return f"lift {float(env.lift(states)[0]):+.4f} m  in_hand {bool(env.in_hand(states)[0])}"

    return note


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--video", default=None, help="write an mp4 here")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args(argv)

    task_cls = get_task_class(TASK)
    scenario = task_cls.scenario.update(num_envs=1, headless=True)
    env = task_cls(scenario=scenario, device="cpu")
    robot = scenario.robots[0]
    states, _ = env.reset(seed=args.seed)

    arm = Arm(env, robot, side="right")
    block = states.objects["cube"].root_state[0, :3].numpy().copy()
    print(f"robot {robot.name}, {robot.num_joints} joints, asset {robot.asset_source}")
    print(f"block at {np.round(block, 4)}")

    # Where the hand has to be, and a path to get there that misses the table.
    for joint, value in arm.home.items():
        arm.scratch.qpos[arm.qadr[joint]] = value
    mujoco.mj_kinematics(arm.model, arm.scratch)
    start = arm.scratch.xpos[arm.palm].copy()

    grasp = block + np.array([0.0, 0.0, GRASP_DEPTH_M])
    above = grasp + np.array([0.0, 0.0, 0.10])
    ceiling = grasp[2] + 0.16
    rise = np.array([(start[0] + above[0]) / 2, (start[1] + above[1]) / 2, ceiling])
    over = np.array([above[0], above[1], ceiling])
    lifted = grasp + np.array([0.0, 0.0, 0.12])

    renderer = camera = None
    if args.video:
        renderer = mujoco.Renderer(arm.model, height=args.height, width=args.width)
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        camera.lookat[:] = [0.22, block[1] * 0.5, 0.85]
        camera.distance = 1.0
        camera.azimuth = 148
        camera.elevation = -16

    run = ScriptedRun(env, arm, camera, renderer, note=_progress(env))
    run.states = states

    open_q = float(robot.gripper_open_q[0])
    run.move(rise, open_q, "rise")
    run.move(over, open_q, "traverse")
    run.move(above, open_q, "descend")
    run.move(grasp, open_q, "settle", waypoints=8)
    run.hold(0.0, 90, "close")
    run.move(lifted, 0.0, "lift", waypoints=10, dwell=10)
    run.hold(0.0, 60, "hold")

    for line in run.log:
        print(" ", line)

    lift = float(env.lift(run.states)[0])
    in_hand = bool(env.in_hand(run.states)[0])
    print(f"\nlifted {lift:+.4f} m, still in the hand: {in_hand}")
    print(f"task reports success: {run.succeeded}")

    if args.video:
        import imageio

        os.makedirs(os.path.dirname(args.video) or ".", exist_ok=True)
        imageio.mimwrite(args.video, run.frames, fps=30, quality=7, macro_block_size=1)
        size = os.path.getsize(args.video) / 1e6
        print(f"wrote {args.video} ({len(run.frames)} frames, {size:.2f} MB)")

    env.close()
    return 0 if run.succeeded else 1


if __name__ == "__main__":
    sys.exit(main())
