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
| `calvin_table_asset.py` | Turns CALVIN's play table URDF into an MJCF. Run it to regenerate the checked-in model. |
| `tasks/` | The six `rakuda.*` tasks. |
| `rakuda_roboverse.py` | First look: wave the arms, or run a task. |
| `rakuda_lift_block.py` | Scripted pick of a block off a table. |
| `rakuda_calvin_scene.py` | Renders CALVIN's scene with the Rakuda in the Panda's place. |
| `rakuda_calvin_motion.py` | Films the robot failing to reach from that spot, then picking a block from one it can work at. |

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
