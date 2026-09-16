"""robopy's MetaSim content pack: the Rakuda as a RoboVerse robot, and tasks for it.

MetaSim discovers content packages through the ``metasim.packages`` entry point
group, which robopy declares in its ``pyproject.toml`` pointing here.  Installing
robopy into the environment RoboVerse runs in is therefore the whole
installation step:

    pip install robopy
    python -c "from metasim.utils.setup_util import get_robot; print(get_robot('rakuda'))"

What this pack contains is **the robot, and nothing else**.
:mod:`robopy.roboverse.robots` registers ``rakuda`` and ``rakuda_gripper``, and
:mod:`robopy.roboverse.assets` finds the MJCF they point at.  That is the whole
surface: with it you can put the machine into any scenario you like.

``tasks``, ``scenes`` and ``grounds`` are present but empty.  MetaSim looks for
all four roles and an absent one is reported as an import error inside messages
about *other* failures, which is needlessly confusing.

Standing the robot somewhere useful is a scene-building decision, not a fact
about the machine -- the Rakuda cannot reach the surface it is bolted to, so it
has to go on a pedestal positioned relative to the work surface, and how high
depends on whether it is reaching or grasping.  All of that, with the four
bundled tasks and the scripted arm control that drives them, lives with the
examples in ``examples/roboverse/``.

Simulator support: MuJoCo, and MJX and Newton insofar as they read the same
MJCF.  The asset is an MJCF because that is what the CAD export can be turned
into faithfully; see :mod:`robopy.sim.mjcf_export`.  Backends that want USD
(Isaac) or that load the URDF directly (PyBullet, SAPIEN) are not wired up: the
URDF's ``package://`` references and its placeholder dynamics would each need
handling, and neither has been tested here, so no ``urdf_path`` is advertised
rather than advertising one that quietly loads a robot made of grams.
"""

from __future__ import annotations
