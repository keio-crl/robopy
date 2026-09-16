"""Film the Rakuda working in CALVIN's scene, in two acts.

    python examples/robot/rakuda_calvin_motion.py \\
        --video docs/robots/assets/rakuda_calvin_motion.mp4

Act one is ``rakuda.calvin_table``: the robot bolted exactly where
``calvin_scene_D.yaml`` bolts its Panda.  It reaches for the red block, gets as
far as it can, and stops short -- because from there the block is further away
than its arm is long.  The gap is printed and is not hidden in the film.

Act two is ``rakuda.calvin_pick``: the same table, the same blocks, the robot
moved to where its own measured workspace says it can work, and this time it
picks the block up.

Neither act is a policy.  Both drive the arm with the differential IK in
:mod:`robopy.roboverse.ik`, through the real task envs, so the success flag at
the end is the task's own.
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import List

os.environ.setdefault("MUJOCO_GL", "egl")  # no DISPLAY on most machines

try:
    import imageio.v2 as imageio
    import mujoco
    import numpy as np
    from metasim.task.registry import get_task_class
except ImportError as exc:  # pragma: no cover - depends on the environment
    raise SystemExit(
        f"this example needs RoboVerse/MetaSim and MuJoCo ({exc}).\n"
        "From a robopy checkout:  pip install -e '.[roboverse]'"
    ) from exc

from robopy.roboverse.ik import Arm, ScriptedRun, solve_ik
from robopy.roboverse.tasks.rakuda_calvin_table import (
    CALVIN_PICK_TARGET,
    RakudaCalvinPickEnv,
)

#: How far above the block's centre the palm goes to grasp it.
#:
#: CALVIN's blocks are 40 mm tall once scaled, far shorter than the 100 mm one
#: ``rakuda.lift_block`` uses, so the jaw has to be held higher relative to the
#: block or the finger tips drive into the bench.  The pads run from 15 mm to
#: 95 mm below the palm, so 80 mm puts their tips 5 mm clear of the bench and
#: still leaves 35 mm of pad against the block.
GRASP_DEPTH_M = 0.080

#: How high above the grasp the hand travels to clear the bench.
#:
#: 0.16 m is as high as this arm can hold the hand over the block at all: at
#: 0.22 m the IK gives up 0.24 m short.  The finger tips hang 0.095 m below the
#: palm, so from here they pass 0.17 m above the bench top.
CLEARANCE_M = 0.16

#: Camera for each act: ``azimuth, elevation, distance, lookat``.
#:
#: The pick is filmed from further back and higher than feels natural, because
#: closer views put the robot's own torso between the camera and its hand at
#: exactly the moment the jaw closes.
SHORT_VIEW = (118, -16, 1.75, (-0.05, -0.20, 0.45))
PICK_VIEW = (135, -25, 1.50, (0.05, -0.22, 0.48))


def framebuffer(model, width: int, height: int) -> None:
    """Let the renderer exceed MuJoCo's 640x480 default offscreen buffer."""
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)


def make_camera(azimuth, elevation, distance, lookat):
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.azimuth, camera.elevation, camera.distance = azimuth, elevation, distance
    camera.lookat[:] = lookat
    return camera


def furthest_reachable(arm: Arm, toward, tolerance: float = 0.005, steps: int = 18) -> np.ndarray:
    """The far end of the reachable part of the line from the hand to ``toward``.

    Binary search on the IK residual.  Nothing clever: the reachable set is
    convex enough along a single ray that halving works, and it is the honest
    way to ask "how far does this arm actually get" without hand-picking a pose.
    """
    here = arm.forward(arm.home)
    toward = np.asarray(toward)
    low, high, best = 0.0, 1.0, here
    for _ in range(steps):
        middle = (low + high) / 2.0
        point = here + (toward - here) * middle
        _, residual = solve_ik(arm, point, arm.home)
        if residual < tolerance:
            low, best = middle, point
        else:
            high = middle
    return best


def act_one(width: int, height: int) -> List[np.ndarray]:
    """The faithful scene: reach for the block and come up short."""
    cls = get_task_class("rakuda.calvin_table")
    env = cls(scenario=cls.scenario.update(num_envs=1, headless=True), device="cpu")
    robot = env.scenario.robots[0]
    states, _ = env.reset()

    arm = Arm(env, robot, side="right")
    block = states.objects[CALVIN_PICK_TARGET].root_state[0, :3].numpy().copy()
    base = states.robots[robot.name].root_state[0, :3].numpy()
    print("act one -- where CALVIN puts its Panda")
    print(f"  robot base {base.round(4)}, block {block.round(4)}")
    print(f"  block is {np.linalg.norm(block - base):.3f} m from the base body")

    framebuffer(arm.model, width, height)
    renderer = mujoco.Renderer(arm.model, height=height, width=width)
    camera = make_camera(*SHORT_VIEW)

    # Aiming straight at the block makes the solver wander: every waypoint past
    # the arm's limit is unsolvable and it gives up early, so the arm barely
    # moves and the film says nothing.  Reach instead to the furthest point on
    # the way to the block that the IK can genuinely hold, and let the gap that
    # remains be the answer.
    wanted = block + np.array([0.0, 0.0, GRASP_DEPTH_M])
    stretch = furthest_reachable(arm, wanted)

    run = ScriptedRun(env, arm, camera, renderer)
    run.states = states
    run.hold(0.0, 20, "settle")
    run.move(stretch, 0.0, "reach", waypoints=20, dwell=6)
    run.hold(0.0, 60, "strain")

    mujoco.mj_forward(arm.model, arm.live)
    reached = arm.live.xpos[arm.palm].copy()
    short = float(np.linalg.norm(block[:2] - reached[:2]))
    print(f"  furthest the hand can hold: {reached.round(4)}")
    print(f"  still {short:.3f} m short of the block, horizontally")
    frames = run.frames
    env.close()
    return frames


def act_two(width: int, height: int) -> tuple[List[np.ndarray], bool]:
    """Mounted where it can work: actually pick the block up."""
    env = RakudaCalvinPickEnv(
        scenario=RakudaCalvinPickEnv.scenario.update(num_envs=1, headless=True), device="cpu"
    )
    robot = env.scenario.robots[0]
    states, _ = env.reset()

    arm = Arm(env, robot, side="right")
    block = states.objects[CALVIN_PICK_TARGET].root_state[0, :3].numpy().copy()
    base = states.robots[robot.name].root_state[0, :3].numpy()
    print("\nact two -- mounted where its workspace says it can work")
    print(f"  robot base {base.round(4)}, block {block.round(4)}")
    print(f"  block is {np.linalg.norm(block - base):.3f} m from the base body")

    framebuffer(arm.model, width, height)
    renderer = mujoco.Renderer(arm.model, height=height, width=width)
    camera = make_camera(*PICK_VIEW)

    # Go over the bench, never through it.  A straight line from the home pose
    # to a point above the block runs the hand into the front panel and the
    # drawer, and the servos then sit 0.16 m behind the command, straining --
    # which looks like the arm being too short and is not.
    start = arm.forward(arm.home)
    grasp = block + np.array([0.0, 0.0, GRASP_DEPTH_M])
    above = grasp + np.array([0.0, 0.0, 0.10])
    ceiling = grasp[2] + CLEARANCE_M
    rise = np.array([(start[0] + above[0]) / 2, (start[1] + above[1]) / 2, ceiling])
    over = np.array([above[0], above[1], ceiling])
    lifted = grasp + np.array([0.0, 0.0, 0.12])

    def note(states):
        return f"lift {float(env.lift(states)[0]):+.4f} m  in_hand {bool(env.in_hand(states)[0])}"

    run = ScriptedRun(env, arm, camera, renderer, note=note)
    run.states = states
    open_q = float(robot.gripper_open_q[0])
    run.move(rise, open_q, "rise")
    run.move(over, open_q, "traverse")
    run.move(above, open_q, "descend")
    run.move(grasp, open_q, "settle", waypoints=8)
    # The jaw is 0.088 m wide open and the block 0.056 m across, so 0.02 m of
    # servo droop is the difference between closing around it and shoving it.
    run.correct(grasp, open_q, "align")
    run.hold(0.0, 90, "close")
    run.move(lifted, 0.0, "lift", waypoints=10, dwell=10)
    run.hold(0.0, 70, "hold")

    for line in run.log:
        print("  ", line)
    print(
        f"  lifted {float(env.lift(run.states)[0]):+.4f} m, "
        f"in the hand: {bool(env.in_hand(run.states)[0])}"
    )
    print(f"  task reports success: {run.succeeded}")
    frames, ok = run.frames, run.succeeded
    env.close()
    return frames, ok


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--video", default=None, help="write an mp4 here")
    parser.add_argument("--still", default=None, help="write the last frame here as a PNG")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=640)
    parser.add_argument("--skip-act-one", action="store_true", help="film only the pick")
    args = parser.parse_args(argv)

    frames: List[np.ndarray] = []
    if not args.skip_act_one:
        frames += act_one(args.width, args.height)
    picked, succeeded = act_two(args.width, args.height)
    frames += picked

    if args.video:
        os.makedirs(os.path.dirname(args.video) or ".", exist_ok=True)
        imageio.mimwrite(args.video, frames, fps=30, quality=7, macro_block_size=1)
        size = os.path.getsize(args.video) / 1e6
        print(f"\nwrote {args.video} ({len(frames)} frames, {size:.2f} MB)")

    if args.still and frames:
        # The last frame: the block up, held, the arm still on it.
        os.makedirs(os.path.dirname(args.still) or ".", exist_ok=True)
        imageio.imwrite(args.still, frames[-1])
        print(f"wrote {args.still}")

    return 0 if succeeded else 1


if __name__ == "__main__":
    sys.exit(main())
