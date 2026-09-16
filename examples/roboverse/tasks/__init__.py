"""Four Rakuda tasks, built on robopy's RoboVerse robot.

These are examples, not API.  robopy ships the *robot* -- ``rakuda`` and
``rakuda_gripper``, registered by :mod:`robopy.roboverse.robots` -- and stops
there, because where to stand a robot and what to ask of it are decisions that
change with the job.  Everything those decisions need is in this directory:

=================== ====================================================
``mount``           the pedestal, and the heights it is derived from
``workspace``       measured facts: what this arm can reach, and where
``ik``              differential IK and a way to drive a task with it
``calvin_table_asset`` CALVIN's play table, turned into an MJCF
=================== ====================================================

=========================== =================================================
``rakuda.reach``            right hand to a sampled target
``rakuda.bimanual_reach``   both hands to their own targets at once
``rakuda.push_cube``        push a cube across a table into a goal region
``rakuda.lift_cube``        close a hand on a cube and pick it up
``rakuda.calvin_table``     stand in CALVIN's scene D, where its Panda stands
``rakuda.calvin_pick``      pick a block off CALVIN's bench, mounted to reach it
=========================== =================================================

All of them run on MuJoCo, which is the only backend the exported assets target.

``rakuda.calvin_table`` is the odd one out: it has no goal and the robot cannot
reach the furniture from where CALVIN's scene file puts the arm's base.  It is
there to put the two robots in one picture at one scale.  ``rakuda.calvin_pick``
is the same scene with the robot moved to where its own workspace says it can
work, and is solvable.

The first three run on ``rakuda``, the robot the CAD actually describes, whose
grippers are fixed frames with no fingers to close.  The lifting ones need a
hand, so they run on ``rakuda_gripper`` -- the same arms with a jaw borrowed
from CALVIN's Panda bolted to each gripper frame.  What that is and is not is in
:mod:`robopy.sim.panda_gripper`; briefly, it is not this machine's gripper and
nothing it does should be read as evidence about the real one.

Importing this package is what makes the names resolvable::

    import tasks  # noqa: F401  -- registers them
    from metasim.task.registry import get_task_class

    env_cls = get_task_class("rakuda.lift_block")

``register_task`` writes into MetaSim's global registry and ``get_task_class``
answers from it, so the import is the whole trick.  What living outside the
content pack costs is only automatic *discovery*: a fresh process's
``list_tasks()`` will not find these until something imports them.
"""

from __future__ import annotations

from .rakuda_calvin import RakudaAtCalvinTableEnv, RakudaCalvinPickEnv
from .rakuda_lift_cube import RakudaLiftCubeEnv
from .rakuda_push_cube import RakudaPushCubeEnv
from .rakuda_reach import RakudaBimanualReachEnv, RakudaReachEnv

__all__ = [
    "RakudaAtCalvinTableEnv",
    "RakudaBimanualReachEnv",
    "RakudaCalvinPickEnv",
    "RakudaLiftCubeEnv",
    "RakudaPushCubeEnv",
    "RakudaReachEnv",
]
