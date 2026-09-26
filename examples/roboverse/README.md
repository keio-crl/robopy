# Rakuda in RoboVerse — examples

robopy itself ships **only the robot**: `robopy.roboverse.robots` registers
`rakuda` and `rakuda_gripper` with MetaSim, and that is the whole API surface.
Where to stand the machine and what to ask of it are scene-building decisions
that change with the job, so they live here rather than in the package.

Run anything in this directory directly, from the repository root:

```bash
python examples/roboverse/rakuda_roboverse.py --demo wave
python examples/roboverse/rakuda_lift_block.py --seed 1
python examples/roboverse/rakuda_calvin_scene.py --stills docs/robots/assets
python examples/roboverse/rakuda_calvin_motion.py --video /tmp/calvin.mp4
python examples/roboverse/rakuda_calvin_views.py --sim mujoco   # or --sim isaacsim
```

Python puts the script's own directory on the import path, which is why these
modules import each other by plain name (`from mount import ...`). Running them
from somewhere else, or importing them from a test, needs that directory on
`sys.path` — see `tests/test_roboverse/conftest.py`.

## What is here

| File | What it is |
| --- | --- |
| `mount.py` | The pedestal: how high to stand the robot, and why. Every offset is measured off the model. |
| `workspace.py` | What this arm can actually reach, as constants the tasks are built on. |
| `ik.py` | Differential IK on a scratch state, and a way to drive a task along a hand path. |
| `staging.py` | The ground, the sky and the lights every task is built on. `staged()` is what fills in a `ScenarioCfg`. |
| `calvin_table_asset.py` | Turns CALVIN's play table URDF into an MJCF *and* a scaled URDF. Run it to regenerate the checked-in models. |
| `calvin_room_asset.py` | Writes the floor as USD, which is the one asset MetaSim will not convert for you. |
| `tasks/` | The six `rakuda.*` tasks. |
| `rakuda_roboverse.py` | First look: wave the arms, or run a task. |
| `rakuda_lift_block.py` | Scripted pick of a block off a table. |
| `rakuda_calvin_scene.py` | Renders CALVIN's scene with the Rakuda in the Panda's place. MuJoCo only: it drives `mujoco.Renderer` itself. |
| `rakuda_calvin_motion.py` | Films the robot failing to reach from that spot, then picking a block from one it can work at. MuJoCo only, same reason. |
| `rakuda_calvin_views.py` | The same four views through `ScenarioCfg.cameras`, so `--sim isaacsim` works. |

## Backends

`staging.staged()` gives every task a scene and a light rig that both the
MuJoCo and the Isaac Sim backends read, so the CALVIN tasks declare
`supported_simulators = ("mujoco", "isaacsim")` and `rakuda_calvin_views.py`
takes `--sim`. What that needs installed, and the assets behind it, is in
[`docs/robots/rakuda_calvin_table.md`](../../docs/robots/rakuda_calvin_table.md).

Two things are worth knowing before adding a backend:

- **A scene's USD is not generated for you.** MetaSim converts an object's or a
  robot's MJCF/URDF to USD on demand, but `IsaacsimHandler._load_scene` just
  warns and returns if `SceneCfg.usd_path` is `None` -- and then skips its own
  terrain too, because `scene` is not `None`. The result is a stage with no
  floor. Hence `calvin_room_asset.py`.
- **Lights belong in `ScenarioCfg`, not in the MJCF.** MetaSim's light configs
  are UsdLux-shaped and each backend translates them. A `<light>` in a scene
  MJCF lights MuJoCo and nothing else.

## Task names

These tasks are outside robopy's content pack, so MetaSim will not discover them
on its own. Importing the package is what registers them:

```python
import tasks  # noqa: F401
from metasim.task.registry import get_task_class

env_cls = get_task_class("rakuda.lift_block")
```

`register_task` writes into MetaSim's global registry and `get_task_class`
answers from it, so the import is the whole trick. What is lost is only
discovery: a fresh process's `list_tasks()` will not list them until something
imports this package.
