"""Tasks this content pack offers to MetaSim: deliberately none.

MetaSim resolves a task name by scanning a content pack for ``@register_task``
decorators, and it looks for this submodule by name.  It is empty because the
Rakuda tasks are **not** part of robopy's API surface -- they are scene-building
decisions, which change with what you are trying to do, and they live with the
examples instead:

============================ ============================================
``examples/roboverse/tasks`` the four Rakuda tasks
``examples/roboverse``       the mount geometry, workspace facts, and IK
============================ ============================================

What robopy *does* ship is the robot: :mod:`robopy.roboverse.robots` registers
``rakuda`` and ``rakuda_gripper``, and that is all you need to put the machine
into a scenario of your own::

    from metasim.scenario.scenario import ScenarioCfg
    from metasim.utils.setup_util import get_handler

    handler = get_handler(ScenarioCfg(robots=["rakuda"], simulator="mujoco"))
    handler.launch()

The example tasks still resolve by name once their module has been imported --
``register_task`` writes into MetaSim's global registry and ``get_task_class``
answers from it -- so ``import tasks`` in a script under
``examples/roboverse/`` is enough to make ``get_task_class("rakuda.lift_block")``
work.  What is lost by living outside the pack is only *discovery*: a fresh
process's ``list_tasks()`` will not find them on its own.

This module stays rather than being deleted because MetaSim looks for all four
content roles -- ``robots``, ``tasks``, ``scenes``, ``grounds`` -- and an absent
one is reported as an import error inside messages about *other* failures, which
is needlessly confusing.
"""

from __future__ import annotations

__all__: list[str] = []
