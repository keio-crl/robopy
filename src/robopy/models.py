"""Locate the robot models that ship with the repository.

The Rakuda CAD export lives under ``models/rakuda/`` at the repository root --
outside the Python package, so wheels stay small.  The URDFs and the 2.9 MB
convex collision meshes are ordinary git files, so a plain clone can always run
the kinematics and the collision-geometry viewer.  The 53 MB of visual meshes
(``assembly_2/meshes/*.stl``) are configured for Git LFS in ``.gitattributes``
and only affect how pretty the viewer looks; a checkout may have them as real
files (``git lfs pull`` done), as LFS pointer files, or not at all (they were
never added to the repository from this environment -- see
``models/rakuda/README.md``).  :class:`RakudaModelFiles` tells the three apart.

Resolution order for the models directory:

1. the ``ROBOPY_MODELS_DIR`` environment variable;
2. an explicit ``base`` argument;
3. a ``models/`` directory found by walking up from this file (a checkout) or
   from the current working directory.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Literal

__all__ = [
    "RAKUDA_PACKAGE_NAME",
    "RakudaModelFiles",
    "VisualMeshStatus",
    "find_models_dir",
    "find_rakuda_model",
    "is_lfs_pointer",
]

RAKUDA_PACKAGE_NAME = "assembly_2"
VisualMeshStatus = Literal["PRESENT", "LFS_POINTERS", "ABSENT"]
_LFS_MAGIC = b"version https://git-lfs.github.com/spec/"


def is_lfs_pointer(path: Path) -> bool:
    """Whether ``path`` is a Git LFS pointer file rather than the real content.

    A pointer is a ~130-byte text file starting with the LFS spec line.  It is
    what a clone without ``git lfs pull`` contains, and feeding it to an STL
    loader fails in a confusing way, so it is detected up front.
    """
    try:
        if path.stat().st_size > 1024:
            return False
        with path.open("rb") as handle:
            return handle.read(len(_LFS_MAGIC)) == _LFS_MAGIC
    except OSError:
        return False


def _candidate_roots(base: Path | None) -> Iterable[Path]:
    env = os.environ.get("ROBOPY_MODELS_DIR")
    if env:
        yield Path(env).expanduser()
    if base is not None:
        yield Path(base)
    for start in (Path(__file__).resolve().parent, Path.cwd().resolve()):
        for parent in (start, *start.parents):
            yield parent / "models"


def find_models_dir(base: Path | None = None) -> Path | None:
    """Return the repository ``models/`` directory, or ``None`` when not found."""
    for candidate in _candidate_roots(base):
        if (candidate / "rakuda" / RAKUDA_PACKAGE_NAME / "urdf").is_dir():
            return candidate
    return None


@dataclass(frozen=True)
class RakudaModelFiles:
    """The Rakuda model as found on disk.

    Attributes:
        package_dir: Directory that resolves ``package://assembly_2/...`` URIs
            (i.e. ``models/rakuda``).
        visual_urdf: ``assembly_2.urdf`` -- full visual meshes (Git LFS).
        convex_collision_urdf: ``assembly_2_convex_collision.urdf`` -- convex
            hulls for both visual and collision geometry (plain git).
        mesh_collision_urdf: ``assembly_2_mesh_collision.urdf``.
        visual_meshes_available: ``True`` only when every visual STL is real
            geometry.
        visual_mesh_status: Why they are unavailable, when they are:
            ``PRESENT``, ``LFS_POINTERS`` (clone without ``git lfs pull``) or
            ``ABSENT`` (the ``meshes/`` directory holds no STL at all).
    """

    package_dir: Path
    visual_urdf: Path
    convex_collision_urdf: Path
    mesh_collision_urdf: Path
    visual_meshes_available: bool
    visual_mesh_status: "VisualMeshStatus" = "ABSENT"

    @property
    def default_urdf(self) -> Path:
        """The best URDF to render: visual when its meshes are present, else convex."""
        return self.visual_urdf if self.visual_meshes_available else self.convex_collision_urdf

    @property
    def meshes_dir(self) -> Path:
        """Where the visual STLs live (or would live): ``assembly_2/meshes``."""
        return self.package_dir / RAKUDA_PACKAGE_NAME / "meshes"

    def missing_visual_meshes(self) -> List[Path]:
        """Visual STLs that are LFS pointers rather than geometry.

        Files that are simply absent are not listed (there is nothing to list);
        ``visual_mesh_status == "ABSENT"`` covers that case.
        """
        return sorted(p for p in self.meshes_dir.glob("*.stl") if is_lfs_pointer(p))

    def visual_mesh_hint(self) -> str | None:
        """One-line, human-readable explanation when the visual meshes are unavailable."""
        if self.visual_mesh_status == "PRESENT":
            return None
        if self.visual_mesh_status == "LFS_POINTERS":
            return (
                "The visual meshes are Git LFS pointers, so the convex collision hulls are "
                "drawn instead. Run `git lfs install && git lfs pull` for the full meshes."
            )
        return (
            f"No visual meshes under {self.meshes_dir}; the convex collision hulls are drawn "
            "instead. Copy assembly_2/meshes/*.stl from Rakuda-2_simulation_ready.zip there "
            "(see models/rakuda/README.md for committing them via Git LFS)."
        )


def find_rakuda_model(base: Path | None = None) -> RakudaModelFiles | None:
    """Locate the committed Rakuda model, or return ``None`` when absent.

    Args:
        base: Optional explicit ``models/`` directory.

    Returns:
        The model files, with ``visual_mesh_status`` telling whether the
        visual STLs are real geometry, LFS pointers, or not there at all.
    """
    models = find_models_dir(base)
    if models is None:
        return None
    package_dir = models / "rakuda"
    urdf_dir = package_dir / RAKUDA_PACKAGE_NAME / "urdf"
    files = RakudaModelFiles(
        package_dir=package_dir,
        visual_urdf=urdf_dir / f"{RAKUDA_PACKAGE_NAME}.urdf",
        convex_collision_urdf=urdf_dir / f"{RAKUDA_PACKAGE_NAME}_convex_collision.urdf",
        mesh_collision_urdf=urdf_dir / f"{RAKUDA_PACKAGE_NAME}_mesh_collision.urdf",
        visual_meshes_available=False,
    )
    if not files.convex_collision_urdf.is_file():
        return None
    stls = list(files.meshes_dir.glob("*.stl"))
    status: VisualMeshStatus
    if not stls:
        status = "ABSENT"
    elif any(is_lfs_pointer(p) for p in stls):
        status = "LFS_POINTERS"
    else:
        status = "PRESENT"
    return RakudaModelFiles(
        package_dir=files.package_dir,
        visual_urdf=files.visual_urdf,
        convex_collision_urdf=files.convex_collision_urdf,
        mesh_collision_urdf=files.mesh_collision_urdf,
        visual_meshes_available=status == "PRESENT",
        visual_mesh_status=status,
    )
