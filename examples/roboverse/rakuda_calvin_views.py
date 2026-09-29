"""Render ``rakuda.calvin_table`` through MetaSim's cameras, on whichever backend you name.

``rakuda_calvin_scene.py`` beside this file renders the same scene faster and
with an orbiting video, but it does it by reaching into
``env.handler.physics.model`` and driving ``mujoco.Renderer`` itself, so it is
MuJoCo and nothing else.  This script goes through ``ScenarioCfg.cameras`` and
``states.cameras[...].rgb``, which every backend implements, so the same scene
and the same four viewpoints come out of::

    python examples/roboverse/rakuda_calvin_views.py --sim mujoco
    python examples/roboverse/rakuda_calvin_views.py --sim isaacsim

That is the whole point of it: one scene description, one set of lights, one
set of cameras, and the backend as an argument.  Use it to check that a change
to the scene reads the same way on both, and to get the Isaac Sim renders --
RTX shadows, real image-based lighting from the dome, PBR materials off the
MTLs -- that MuJoCo's fixed-function renderer cannot produce.

Isaac Sim is not part of the RoboVerse environment robopy is installed into;
see ``docs/robots/rakuda_calvin_table.md`` for what it takes.  If it is missing,
this script says so in one line rather than in a stack trace.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, List, Tuple

os.environ.setdefault("MUJOCO_GL", "egl")  # no DISPLAY on most machines

try:
    import imageio.v2 as imageio
    import numpy as np
    import torch
    from metasim.scenario.cameras import PinholeCameraCfg
    from metasim.task.registry import get_task_class
except ImportError as exc:  # pragma: no cover - depends on the environment
    print(f"this example needs RoboVerse: {exc}", file=sys.stderr)
    print("From a robopy checkout:  pip install -e '.[sim]'", file=sys.stderr)
    raise SystemExit(1) from exc

import tasks  # noqa: F401  (registers rakuda.calvin_table)
from rakuda_calvin_scene import LOOK_AT, STILLS

__all__ = ["camera_position", "calvin_cameras", "main"]

#: Vertical field of view, degrees.  MuJoCo's default camera, so the frames
#: line up with ``rakuda_calvin_scene.py``'s.
FOV_Y_DEG = 45.0


def camera_position(
    look_at: Tuple[float, float, float], azimuth: float, elevation: float, distance: float
) -> Tuple[float, float, float]:
    """Turn MuJoCo's orbit parameters into a world position.

    ``rakuda_calvin_scene.STILLS`` is written the way one drives MuJoCo's
    viewer -- an azimuth, an elevation and a distance about a look-at point --
    because that is how the angles were found.  MetaSim's cameras are placed by
    position and look-at, so the two have to be converted rather than picked
    again, or the two scripts drift apart and their renders stop comparing.

    MuJoCo's angles describe the direction *from* the camera *to* the look-at
    point, so the camera sits that far back along it.

    Args:
        look_at: The point the camera looks at.
        azimuth: Degrees about ``+z``, from ``+x``.
        elevation: Degrees above the horizontal; negative looks down.
        distance: Metres from the look-at point.

    Returns:
        The camera position in world coordinates.
    """
    az, el = math.radians(azimuth), math.radians(elevation)
    forward = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
    return tuple(centre - distance * f for centre, f in zip(look_at, forward))


def calvin_cameras(width: int, height: int) -> List[PinholeCameraCfg]:
    """One camera per viewpoint in :data:`rakuda_calvin_scene.STILLS`.

    All four are placed at once and read from one step, rather than one camera
    moved between renders: MuJoCo bakes its cameras into the MJCF when the
    model is built, so moving one afterwards does not move it.
    """
    # PinholeCameraCfg is specified like a physical camera. The vertical
    # aperture is derived from the horizontal one and the aspect ratio, so the
    # focal length that gives FOV_Y_DEG follows from that.
    horizontal_aperture = 20.955  # the config default, in cm
    vertical_aperture = horizontal_aperture * height / width
    focal_length = vertical_aperture / (2.0 * math.tan(math.radians(FOV_Y_DEG) / 2.0))
    return [
        PinholeCameraCfg(
            name=name,
            data_types=["rgb"],
            width=width,
            height=height,
            pos=camera_position(LOOK_AT, azimuth, elevation, distance),
            look_at=LOOK_AT,
            focal_length=focal_length,
            horizontal_aperture=horizontal_aperture,
        )
        for name, (azimuth, elevation, distance) in STILLS.items()
    ]


def _settle(env, robot, steps: int) -> Dict:
    """Hold the robot still until the blocks have stopped moving."""
    joints = env.handler.get_joint_names(robot.name, sort=True)
    hold = torch.tensor([[float(robot.default_joint_positions[j]) for j in joints]])
    states = None
    for _ in range(steps):
        states, *_ = env.step(hold)
    return states


def _as_image(rgb) -> "np.ndarray":
    """One environment's RGB frame as ``uint8`` HxWx3, whatever the backend returned."""
    frame = rgb[0] if rgb.ndim == 4 else rgb
    frame = frame.detach().cpu().numpy() if hasattr(frame, "detach") else np.asarray(frame)
    if frame.dtype != np.uint8:  # some backends hand back floats in 0..1
        frame = (np.clip(frame, 0.0, 1.0) * 255).astype(np.uint8)
    return frame[..., :3]


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--sim",
        default="mujoco",
        help="MetaSim backend: mujoco (default), isaacsim, sapien3, genesis, ...",
    )
    parser.add_argument("--out", default=None, help="write PNGs into this directory")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=640)
    parser.add_argument("--settle-steps", type=int, default=60)
    args = parser.parse_args(argv)

    cls = get_task_class("rakuda.calvin_table")
    if args.sim not in cls.supported_simulators:
        print(
            f"rakuda.calvin_table does not claim to run on {args.sim!r}; "
            f"it supports {', '.join(cls.supported_simulators)}",
            file=sys.stderr,
        )
        return 2

    scenario = cls.scenario.update(
        num_envs=1,
        headless=True,
        simulator=args.sim,
        cameras=calvin_cameras(args.width, args.height),
    )
    try:
        env = cls(scenario=scenario, device="cpu" if args.sim == "mujoco" else "cuda:0")
    except ImportError as exc:
        print(f"the {args.sim} backend is not installed in this environment: {exc}", file=sys.stderr)
        print(
            "Isaac Sim: see docs/robots/rakuda_calvin_table.md, section "
            "'Isaac Lab で動かす'. Every other backend: "
            f"pip install 'roboverse-metasim[{args.sim}]'",
            file=sys.stderr,
        )
        return 1

    env.reset()
    states = _settle(env, env.scenario.robots[0], args.settle_steps)

    out = args.out or os.path.join("/tmp", f"rakuda_calvin_{args.sim}")
    os.makedirs(out, exist_ok=True)
    for name in STILLS:
        camera = states.cameras.get(name)
        if camera is None or camera.rgb is None:
            print(f"{args.sim} returned no RGB for camera {name!r}", file=sys.stderr)
            continue
        path = os.path.join(out, f"rakuda_calvin_{name}.png")
        imageio.imwrite(path, _as_image(camera.rgb))
        print(f"wrote {path}")

    env.close()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
