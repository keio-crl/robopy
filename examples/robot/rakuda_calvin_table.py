"""Look at ``rakuda.calvin_table``: the Rakuda standing where CALVIN puts its Panda.

Nothing is solved here.  The scene is CALVIN's play table at its own
``global_scaling`` of 0.8, with the Rakuda bolted at the ``robot_base_position``
from ``calvin_scene_D.yaml``, and this script renders it so the two can be seen
at the same scale.

    python examples/robot/rakuda_calvin_table.py
    python examples/robot/rakuda_calvin_table.py --video docs/robots/assets/calvin.mp4
    python examples/robot/rakuda_calvin_table.py --stills docs/robots/assets

The video orbits the scene, then opens the sliding door and pulls the drawer
out, which is the quickest way to see that the table is articulated and that the
robot is standing in the furniture rather than beside a picture of it.

**The Rakuda cannot reach this table from here.**  The scene is sized for a
Panda; the Rakuda's hands stop 0.428 m from its mounting plane and the nearest
corner of the work surface is further than that.  The script prints the numbers
rather than hiding them.  ``rakuda.lift_block`` is the task where the robot can
actually work.
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
    import torch
    from metasim.task.registry import get_task_class
except ImportError as exc:  # pragma: no cover - depends on the environment
    print(f"this example needs RoboVerse and MuJoCo: {exc}", file=sys.stderr)
    raise SystemExit(1) from exc

from robopy.roboverse.mount import STAND_HEIGHT
from robopy.roboverse.tasks.rakuda_calvin_table import PANDA_BASE_POSITION

#: Where the camera looks: the middle of the bench, a little above it.
LOOK_AT = (-0.05, -0.12, 0.42)

#: Stills worth having, as ``azimuth, elevation, distance``.
STILLS = {
    "front": (118, -14, 1.9),
    "side": (178, -12, 1.8),
    "top": (128, -46, 1.9),
    "close": (104, -16, 1.15),
}

#: The table's travel, in metres, already scaled.
SLIDE_TRAVEL = 0.28
DRAWER_TRAVEL = 0.22


def settle(env, robot, steps: int = 60):
    """Hold the robot still until the blocks have stopped moving."""
    joints = env.handler.get_joint_names(robot.name, sort=True)
    hold = torch.tensor([[float(robot.default_joint_positions[j]) for j in joints]])
    states = None
    for _ in range(steps):
        states, *_ = env.step(hold)
    return states


def joint_address(model, suffix: str) -> int:
    """``qpos`` index of the table joint ending in ``suffix``.

    MetaSim prefixes an object's joints with its scene name, and the prefix has
    changed shape between versions, so match on the end rather than guessing it.
    """
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
        if name.endswith(suffix):
            return int(model.jnt_qposadr[i])
    raise KeyError(f"no table joint ending in {suffix!r}")


def describe(states, model, data) -> None:
    """Print where everything ended up, and how far short the arms fall."""
    base = states.robots["rakuda"].root_state[0, :3].numpy()
    print(f"robot base   {base.round(4)}  (feet at z={base[2] - STAND_HEIGHT:.3f})")
    print(f"panda base   {np.array(PANDA_BASE_POSITION).round(4)}  from calvin_scene_D.yaml")
    for name in ("block_red", "block_blue", "block_pink"):
        print(f"{name:12} {states.objects[name].root_state[0, :3].numpy().round(4)}")

    low = np.full(3, np.inf)
    high = np.full(3, -np.inf)
    for geom in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom]) or ""
        if not body.startswith("calvin_table/"):
            continue
        low = np.minimum(low, data.geom_xpos[geom])
        high = np.maximum(high, data.geom_xpos[geom])
    corners = np.array([[x, y] for x in (low[0], high[0]) for y in (low[1], high[1])])
    reach = np.linalg.norm(corners - base[:2], axis=1)
    print(f"table corners are {reach.round(3)} m away; the hands reach 0.428 m")


def render(renderer, data, camera, azimuth, elevation, distance):
    camera.azimuth, camera.elevation, camera.distance = azimuth, elevation, distance
    camera.lookat[:] = LOOK_AT
    renderer.update_scene(data, camera)
    return renderer.render()


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--video", default=None, help="write an mp4 here")
    parser.add_argument("--stills", default=None, help="write PNGs into this directory")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=640)
    args = parser.parse_args(argv)

    cls = get_task_class("rakuda.calvin_table")
    env = cls(scenario=cls.scenario.update(num_envs=1, headless=True), device="cpu")
    robot = env.scenario.robots[0]
    env.reset()
    states = settle(env, robot)

    model = env.handler.physics.model.ptr
    data = env.handler.physics.data.ptr
    describe(states, model, data)

    # The offscreen framebuffer defaults to 640x480 and the renderer refuses to
    # exceed it.  It is sized when the render context is built, so growing it
    # has to happen before the Renderer exists.
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)

    if args.stills:
        os.makedirs(args.stills, exist_ok=True)
        for name, (azimuth, elevation, distance) in STILLS.items():
            path = os.path.join(args.stills, f"rakuda_calvin_{name}.png")
            imageio.imwrite(path, render(renderer, data, camera, azimuth, elevation, distance))
            print(f"wrote {path}")

    if args.video:
        slide = joint_address(model, "base__slide")
        drawer = joint_address(model, "base__drawer")
        frames = []
        # One slow lap, so the robot's position relative to the table reads.
        for azimuth in np.linspace(40, 400, 150):
            frames.append(render(renderer, data, camera, azimuth, -16, 1.9))
        # Then open the door and the drawer, from the front.
        for phase in np.concatenate([np.linspace(0, 1, 60), np.linspace(1, 0, 30)]):
            data.qpos[slide] = SLIDE_TRAVEL * min(1.0, phase * 2)
            data.qpos[drawer] = DRAWER_TRAVEL * max(0.0, phase * 2 - 1)
            mujoco.mj_forward(model, data)
            frames.append(render(renderer, data, camera, 118, -16, 1.75))
        imageio.mimwrite(args.video, frames, fps=30, quality=7, macro_block_size=1)
        size = os.path.getsize(args.video) / 1e6
        print(f"wrote {args.video} ({len(frames)} frames, {size:.2f} MB)")

    env.close()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
