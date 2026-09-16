"""Model files that travel inside the robopy wheel.

``pip install robopy`` has to be enough to put the Rakuda into a RoboVerse
scene, and a wheel cannot reach the repository's ``models/`` directory.  So one
self-contained MJCF -- geometry inlined, no STL files beside it -- ships here.

It draws the convex hulls rather than the 53 MB of Git LFS visual meshes.  A
checkout that has those gets the prettier model instead:
:func:`resolve_rakuda_mjcf` prefers ``models/rakuda/assembly_2/mjcf/rakuda.xml``
whenever it can find it, and the two are the same robot -- same bodies, same
joints, same masses, same sites -- differing only in what is drawn.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Tuple

__all__ = [
    "PACKAGED_RAKUDA_GRIPPER_MJCF",
    "PACKAGED_RAKUDA_MJCF",
    "RAKUDA_MJCF_ENV_VAR",
    "resolve_rakuda_mjcf",
]

#: Overrides every other source when set, so a user can point the simulated
#: robot at a model of their own without editing anything.
RAKUDA_MJCF_ENV_VAR = "ROBOPY_RAKUDA_MJCF"

#: The self-contained model inside the wheel.
PACKAGED_RAKUDA_MJCF: Path = Path(__file__).resolve().parent / "rakuda" / "rakuda.xml"

#: The same robot with a borrowed hand on each arm; see
#: :mod:`robopy.sim.panda_gripper` for what that is and is not.
PACKAGED_RAKUDA_GRIPPER_MJCF: Path = (
    Path(__file__).resolve().parent / "rakuda" / "rakuda_gripper.xml"
)


def resolve_rakuda_mjcf(variant: str = "plain") -> Tuple[Path, str]:
    """Return the best available Rakuda MJCF and where it came from.

    Args:
        variant: ``"plain"`` for the robot the CAD describes, or ``"gripper"``
            for the one with borrowed fingers on both arms.

    Resolution order:

    1. ``$ROBOPY_RAKUDA_MJCF``, when set -- an explicit choice wins.
    2. ``models/rakuda/assembly_2/mjcf/rakuda.xml`` in a repository checkout,
       which references the real visual meshes when Git LFS has fetched them.
    3. The self-contained model in this package, which always exists.

    Returns:
        ``(path, reason)``, where ``reason`` is a short phrase naming the
        source, suitable for a log line.

    Raises:
        FileNotFoundError: If the override is set but does not exist, or if the
            packaged model is missing (a broken installation).
    """
    if variant not in ("plain", "gripper"):
        raise ValueError(f"variant must be 'plain' or 'gripper', not {variant!r}")
    suffix = "" if variant == "plain" else "_gripper"
    packaged = PACKAGED_RAKUDA_MJCF if variant == "plain" else PACKAGED_RAKUDA_GRIPPER_MJCF

    override = os.environ.get(RAKUDA_MJCF_ENV_VAR)
    if override:
        path = Path(override).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"{RAKUDA_MJCF_ENV_VAR} points at {path}, which does not exist")
        return path, f"${RAKUDA_MJCF_ENV_VAR}"

    from robopy.models import RAKUDA_PACKAGE_NAME, find_rakuda_model

    model = find_rakuda_model()
    if model is not None:
        checkout = model.package_dir / RAKUDA_PACKAGE_NAME / "mjcf" / f"rakuda{suffix}.xml"
        if checkout.is_file():
            which = "visual meshes" if model.visual_meshes_available else "convex hulls"
            return checkout, f"models/ checkout, drawing {which}"

    if not packaged.is_file():
        raise FileNotFoundError(
            f"robopy is installed without its packaged Rakuda model ({packaged}). "
            "Reinstall robopy, or set "
            f"{RAKUDA_MJCF_ENV_VAR} to an MJCF exported with robopy.sim.mjcf_export."
        )
    return packaged, "robopy wheel, drawing convex hulls"
