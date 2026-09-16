"""Tasks this content pack offers to MetaSim.

MetaSim resolves a task name by scanning this package for ``@register_task``
decorators, so importing the modules here is what makes the names usable:

=========================== =================================================
``rakuda.reach``            right hand to a sampled target
``rakuda.bimanual_reach``   both hands to their own targets at once
``rakuda.push_cube``        push a cube across a table into a goal region
``rakuda.lift_cube``        close a hand on a cube and pick it up
=========================== =================================================

All four run on MuJoCo, which is the only backend the exported assets target.

The first three run on ``rakuda``, the robot the CAD actually describes, whose
grippers are fixed frames with no fingers to close.  ``rakuda.lift_cube`` needs
a hand, so it runs on ``rakuda_gripper`` -- the same arms with a jaw borrowed
from CALVIN's Panda bolted to each gripper frame.  What that is and is not is in
:mod:`robopy.sim.panda_gripper`; briefly, it is not this machine's gripper and
nothing it does should be read as evidence about the real one.
"""

from __future__ import annotations

from .rakuda_lift_cube import RakudaLiftCubeEnv
from .rakuda_push_cube import RakudaPushCubeEnv
from .rakuda_reach import RakudaBimanualReachEnv, RakudaReachEnv

__all__ = [
    "RakudaBimanualReachEnv",
    "RakudaLiftCubeEnv",
    "RakudaPushCubeEnv",
    "RakudaReachEnv",
]
