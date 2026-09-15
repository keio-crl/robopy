"""Tasks this content pack offers to MetaSim.

MetaSim resolves a task name by scanning this package for ``@register_task``
decorators, so importing the modules here is what makes the names usable:

=========================== =================================================
``rakuda.reach``            right hand to a sampled target
``rakuda.bimanual_reach``   both hands to their own targets at once
``rakuda.push_cube``        push a cube across a table into a goal region
=========================== =================================================

All three run on MuJoCo, which is the only backend the exported asset targets.

What is *not* here is grasping.  The CAD export models the Rakuda's two
grippers as fixed frames rather than joints, so the simulated robot has no
fingers to close; every task above is built to need none.  Giving it a working
hand means adding gripper joints to the model, not writing another task.
"""

from __future__ import annotations

from .rakuda_push_cube import RakudaPushCubeEnv
from .rakuda_reach import RakudaBimanualReachEnv, RakudaReachEnv

__all__ = [
    "RakudaBimanualReachEnv",
    "RakudaPushCubeEnv",
    "RakudaReachEnv",
]
