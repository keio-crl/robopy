"""robopy's MetaSim content pack: the Rakuda as a RoboVerse robot, and tasks for it.

MetaSim discovers content packages through the ``metasim.packages`` entry point
group, which robopy declares in its ``pyproject.toml`` pointing here.  Installing
robopy into the environment RoboVerse runs in is therefore the whole
installation step:

    pip install robopy
    python -c "from metasim.utils.setup_util import get_robot; print(get_robot('rakuda'))"

``robopy.roboverse.mount`` is the other half of putting this robot in a scene:
the Rakuda cannot reach the surface it is bolted to, so it has to be stood on a
pedestal positioned relative to the work surface rather than to the floor. That
module derives the heights and builds the geometry; the bundled tasks all use it.

MetaSim imports ``robopy.roboverse.robots`` to resolve a robot name and
``robopy.roboverse.tasks`` to resolve a task name, so those two submodules are
the pack's whole surface.  ``scenes`` and ``grounds`` are present but empty --
MetaSim looks for all four roles and an absent one is reported as an import
error in messages about *other* failures, which is needlessly confusing.

Simulator support: MuJoCo, and MJX and Newton insofar as they read the same
MJCF.  The asset is an MJCF because that is what the CAD export can be turned
into faithfully; see :mod:`robopy.sim.mjcf_export`.  Backends that want USD
(Isaac) or that load the URDF directly (PyBullet, SAPIEN) are not wired up: the
URDF's ``package://`` references and its placeholder dynamics would each need
handling, and neither has been tested here, so no ``urdf_path`` is advertised
rather than advertising one that quietly loads a robot made of grams.
"""

from __future__ import annotations
