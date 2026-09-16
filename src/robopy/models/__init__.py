"""Robot models that ship with robopy, and where their optional parts live.

robopy is a library: everything the kinematics, the solver, the viewer and the
VR teleoperation need is inside the installed package, so ``pip install
robopy`` is enough.  The Rakuda-2 CAD export lives under
``robopy/models/rakuda/`` as package data:

* the three URDFs and the 137 convex collision meshes (2.9 MB) are always
  installed and are all the model needs to run;
* the 137 *visual* meshes (53 MB of decorative STL) are not in the wheel.  In
  the repository they are Git LFS objects; an installed library fetches them
  on demand into a per-user cache with :func:`fetch_visual_meshes` (or the
  ``robopy-models fetch`` command).  Without them the viewer draws the convex
  hulls, and nothing else changes.

Resolution order for the models directory (the one containing ``rakuda/``):

1. the ``ROBOPY_MODELS_DIR`` environment variable;
2. an explicit ``base`` argument;
3. the package data, :data:`BUNDLED_MODELS_DIR`.

Visual meshes are looked for in the fetch cache first, then next to the URDF,
so a checkout with ``git lfs pull`` done and an installed wheel with a fetched
cache behave the same.
"""

from __future__ import annotations

import os
import shutil
import tempfile
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, List, Literal, Sequence

__all__ = [
    "BUNDLED_MODELS_DIR",
    "RAKUDA_PACKAGE_NAME",
    "RAKUDA_VISUAL_MESH_URL",
    "FetchReport",
    "RakudaModelFiles",
    "VisualMeshStatus",
    "cache_dir",
    "fetch_visual_meshes",
    "find_models_dir",
    "find_rakuda_model",
    "is_lfs_pointer",
    "visual_mesh_cache_package_dir",
    "visual_mesh_names",
]

RAKUDA_PACKAGE_NAME = "assembly_2"
VisualMeshStatus = Literal["PRESENT", "LFS_POINTERS", "ABSENT"]
_LFS_MAGIC = b"version https://git-lfs.github.com/spec/"

#: The models shipped inside the installed package (``robopy/models``).
BUNDLED_MODELS_DIR: Path = Path(__file__).resolve().parent

#: Where GitHub serves the repository's LFS objects.  ``{ref}`` is a branch,
#: tag or commit; ``{name}`` the STL file name.  A private repository needs a
#: token (``GITHUB_TOKEN`` or the ``token`` argument).
RAKUDA_VISUAL_MESH_URL = (
    "https://media.githubusercontent.com/media/keio-crl/robopy/{ref}/"
    "src/robopy/models/rakuda/assembly_2/meshes/{name}"
)


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


def cache_dir() -> Path:
    """robopy's per-user cache: ``$ROBOPY_CACHE_DIR``, else ``$XDG_CACHE_HOME/robopy``."""
    env = os.environ.get("ROBOPY_CACHE_DIR")
    if env:
        return Path(env).expanduser()
    xdg = os.environ.get("XDG_CACHE_HOME")
    base = Path(xdg).expanduser() if xdg else Path.home() / ".cache"
    return base / "robopy"


def visual_mesh_cache_package_dir() -> Path:
    """The cache directory that resolves ``package://assembly_2/meshes/...`` for fetched STLs."""
    return cache_dir() / "models" / "rakuda"


def _candidate_roots(base: Path | None) -> Iterable[Path]:
    env = os.environ.get("ROBOPY_MODELS_DIR")
    if env:
        yield Path(env).expanduser()
    if base is not None:
        yield Path(base)
    yield BUNDLED_MODELS_DIR


def find_models_dir(base: Path | None = None) -> Path | None:
    """Return the models directory (containing ``rakuda/``), or ``None`` when not found."""
    for candidate in _candidate_roots(base):
        if (candidate / "rakuda" / RAKUDA_PACKAGE_NAME / "urdf").is_dir():
            return candidate
    return None


def visual_mesh_names(urdf: Path) -> List[str]:
    """Basenames of the STL files a URDF's ``<visual>`` elements reference."""
    names: List[str] = []
    try:
        root = ET.parse(urdf).getroot()
    except (ET.ParseError, OSError):
        return names
    for visual in root.iter("visual"):
        for mesh in visual.iter("mesh"):
            uri = mesh.get("filename", "")
            name = uri.rsplit("/", 1)[-1]
            if name and name not in names:
                names.append(name)
    return sorted(names)


def _real_mesh(path: Path) -> bool:
    return path.is_file() and not is_lfs_pointer(path)


@dataclass(frozen=True)
class RakudaModelFiles:
    """The Rakuda model as found on disk.

    Attributes:
        package_dir: Directory that resolves ``package://assembly_2/...`` for
            the URDFs and the convex collision meshes (``.../models/rakuda``).
        visual_urdf: ``assembly_2.urdf`` -- the original export, visual meshes only.
        convex_collision_urdf: ``assembly_2_convex_collision.urdf`` -- visual
            meshes in ``<visual>``, convex hulls in ``<collision>``.  The default.
        mesh_collision_urdf: ``assembly_2_mesh_collision.urdf``.
        visual_meshes_available: ``True`` only when every visual STL the URDF
            references is real geometry somewhere in :attr:`package_dirs`.
        visual_mesh_status: ``PRESENT``, ``LFS_POINTERS`` (a checkout without
            ``git lfs pull``) or ``ABSENT`` (an installed wheel before
            :func:`fetch_visual_meshes`).
        visual_mesh_dir: Directory holding the real visual STLs, when present.
        visual_mesh_count: How many visual STLs the URDF references.
    """

    package_dir: Path
    visual_urdf: Path
    convex_collision_urdf: Path
    mesh_collision_urdf: Path
    visual_meshes_available: bool
    visual_mesh_status: VisualMeshStatus = "ABSENT"
    visual_mesh_dir: Path | None = None
    visual_mesh_count: int = 0

    @property
    def default_urdf(self) -> Path:
        """The best URDF to render: visual when its meshes are present, else convex."""
        return self.visual_urdf if self.visual_meshes_available else self.convex_collision_urdf

    @property
    def meshes_dir(self) -> Path:
        """Where the package's own visual STLs live (or would): ``assembly_2/meshes``."""
        return self.package_dir / RAKUDA_PACKAGE_NAME / "meshes"

    @property
    def package_dirs(self) -> List[Path]:
        """Directories to resolve ``package://`` URIs, fetched visual meshes first.

        Pass this (not just :attr:`package_dir`) to anything that loads meshes,
        so visual STLs fetched into the cache are found before the package's
        own ``meshes/`` directory, which in a checkout may hold LFS pointers.
        """
        dirs: List[Path] = []
        if self.visual_mesh_dir is not None:
            candidate = self.visual_mesh_dir.parent.parent  # .../rakuda
            if candidate != self.package_dir:
                dirs.append(candidate)
        dirs.append(self.package_dir)
        return dirs

    def missing_visual_meshes(self) -> List[Path]:
        """Visual STLs in the package directory that are LFS pointers, not geometry."""
        return sorted(p for p in self.meshes_dir.glob("*.stl") if is_lfs_pointer(p))

    def visual_mesh_hint(self) -> str | None:
        """One-line, human-readable explanation when the visual meshes are unavailable."""
        if self.visual_mesh_status == "PRESENT":
            return None
        if self.visual_mesh_status == "LFS_POINTERS":
            return (
                "The visual meshes are Git LFS pointers, so the convex collision hulls are "
                "drawn instead. Run `git lfs install && git lfs pull` in the checkout, or "
                "`robopy-models fetch` to download them into the cache."
            )
        return (
            "The visual meshes are not installed (they are not part of the wheel), so the "
            "convex collision hulls are drawn instead. Run `robopy-models fetch` to download "
            f"them into {visual_mesh_cache_package_dir() / RAKUDA_PACKAGE_NAME / 'meshes'}."
        )


def find_rakuda_model(base: Path | None = None) -> RakudaModelFiles | None:
    """Locate the Rakuda model, or return ``None`` when absent.

    Args:
        base: Optional explicit models directory (the one containing ``rakuda/``).

    Returns:
        The model files, with ``visual_mesh_status`` telling whether the
        visual STLs are real geometry, LFS pointers, or not there at all.
    """
    models = find_models_dir(base)
    if models is None:
        return None
    package_dir = models / "rakuda"
    urdf_dir = package_dir / RAKUDA_PACKAGE_NAME / "urdf"
    convex = urdf_dir / f"{RAKUDA_PACKAGE_NAME}_convex_collision.urdf"
    if not convex.is_file():
        return None
    visual_urdf = urdf_dir / f"{RAKUDA_PACKAGE_NAME}.urdf"
    names = visual_mesh_names(convex if convex.is_file() else visual_urdf)
    own_meshes = package_dir / RAKUDA_PACKAGE_NAME / "meshes"

    status: VisualMeshStatus = "ABSENT"
    visual_dir: Path | None = None
    candidates = [
        visual_mesh_cache_package_dir() / RAKUDA_PACKAGE_NAME / "meshes",
        own_meshes,
    ]
    if names:
        for directory in candidates:
            if all(_real_mesh(directory / name) for name in names):
                status, visual_dir = "PRESENT", directory
                break
    if status != "PRESENT":
        own = list(own_meshes.glob("*.stl"))
        if own and any(is_lfs_pointer(p) for p in own):
            status = "LFS_POINTERS"
        elif own and not names:
            # A URDF that names no visual meshes: whatever STLs sit next to it
            # are taken as the visual set.
            status, visual_dir = "PRESENT", own_meshes
    return RakudaModelFiles(
        package_dir=package_dir,
        visual_urdf=visual_urdf,
        convex_collision_urdf=convex,
        mesh_collision_urdf=urdf_dir / f"{RAKUDA_PACKAGE_NAME}_mesh_collision.urdf",
        visual_meshes_available=status == "PRESENT",
        visual_mesh_status=status,
        visual_mesh_dir=visual_dir,
        visual_mesh_count=len(names),
    )


# --- fetching the visual meshes -------------------------------------------------


@dataclass
class FetchReport:
    """What :func:`fetch_visual_meshes` did.

    Attributes:
        destination: Directory the STLs were written to.
        downloaded: Files fetched this time.
        skipped: Files that were already present (and not pointers).
        failed: ``(name, reason)`` for files that could not be fetched.
    """

    destination: Path
    downloaded: List[str] = field(default_factory=list)
    skipped: List[str] = field(default_factory=list)
    failed: List[tuple[str, str]] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        """Whether every file is now present."""
        return not self.failed


def _looks_like_stl(data: bytes) -> bool:
    if len(data) < 84:
        return False
    if data[: len(_LFS_MAGIC)] == _LFS_MAGIC:
        return False
    if data[:5] == b"solid" and b"facet" in data[:4096]:
        return True
    triangles = int.from_bytes(data[80:84], "little")
    return len(data) == 84 + 50 * triangles


def _default_opener(url: str, token: str | None, timeout_s: float) -> bytes:
    headers = {"User-Agent": "robopy-models"}
    if token:
        headers["Authorization"] = f"token {token}"
    request = urllib.request.Request(url, headers=headers)
    with urllib.request.urlopen(request, timeout=timeout_s) as response:  # noqa: S310 - https only
        return bytes(response.read())


def fetch_visual_meshes(
    destination: Path | None = None,
    *,
    ref: str = "main",
    token: str | None = None,
    force: bool = False,
    names: Sequence[str] | None = None,
    progress: Callable[[str, int, int], None] | None = None,
    opener: Callable[[str, str | None, float], bytes] | None = None,
    timeout_s: float = 60.0,
    url_template: str = RAKUDA_VISUAL_MESH_URL,
) -> FetchReport:
    """Download the Rakuda visual meshes into the cache (or ``destination``).

    The files come from the repository's Git LFS storage through GitHub's
    media endpoint, so the installed wheel stays small and a plain
    ``pip install`` still has everything it needs to *run*; this only adds
    the pretty rendering.

    Args:
        destination: Directory for the STLs.  Default: the cache location
            :func:`find_rakuda_model` looks in first.
        ref: Branch, tag or commit to fetch from.
        token: GitHub token for a private repository; ``GITHUB_TOKEN`` is used
            when ``None``.
        force: Re-download files that are already present.
        names: The STL names to fetch; default: every ``<visual>`` mesh the
            bundled URDF references.
        progress: Called with ``(name, index, total)`` before each download.
        opener: ``(url, token, timeout_s) -> bytes``; replaceable for tests.
        timeout_s: Per-file network timeout.
        url_template: Where to fetch from; ``{ref}`` and ``{name}`` are filled in.

    Returns:
        A :class:`FetchReport`.  Nothing is raised for a failed file; a file
        that arrives as an LFS pointer or is not an STL counts as failed and
        is not kept.
    """
    dest = (
        Path(destination)
        if destination is not None
        else visual_mesh_cache_package_dir() / RAKUDA_PACKAGE_NAME / "meshes"
    )
    dest.mkdir(parents=True, exist_ok=True)
    if names is None:
        rakuda = find_rakuda_model()
        if rakuda is None:
            raise FileNotFoundError(
                "The bundled Rakuda model was not found; cannot list its meshes."
            )
        names = visual_mesh_names(rakuda.convex_collision_urdf)
    token = token if token is not None else os.environ.get("GITHUB_TOKEN")
    fetch = opener or _default_opener
    report = FetchReport(destination=dest)
    total = len(names)
    for index, name in enumerate(names, start=1):
        target = dest / name
        if not force and _real_mesh(target):
            report.skipped.append(name)
            continue
        if progress is not None:
            progress(name, index, total)
        url = url_template.format(ref=ref, name=name)
        try:
            data = fetch(url, token, timeout_s)
        except urllib.error.HTTPError as exc:
            report.failed.append((name, f"HTTP {exc.code} for {url}"))
            continue
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            report.failed.append((name, f"{type(exc).__name__}: {exc}"))
            continue
        if not _looks_like_stl(data):
            report.failed.append(
                (
                    name,
                    "the response is not an STL (an LFS pointer or an error page); for a "
                    "private repository set GITHUB_TOKEN",
                )
            )
            continue
        with tempfile.NamedTemporaryFile(dir=dest, delete=False) as handle:
            handle.write(data)
            temporary = Path(handle.name)
        shutil.move(str(temporary), str(target))
        report.downloaded.append(name)
    return report


def status_text(rakuda: RakudaModelFiles | None) -> str:
    """Human-readable summary for the command line."""
    if rakuda is None:
        return "Rakuda model: not found (ROBOPY_MODELS_DIR unset and no package data?)"
    lines = [
        f"Rakuda model     : {rakuda.convex_collision_urdf}",
        f"package dirs     : {', '.join(str(d) for d in rakuda.package_dirs)}",
        f"visual meshes    : {rakuda.visual_mesh_status} ({rakuda.visual_mesh_count} referenced)",
    ]
    if rakuda.visual_mesh_dir is not None:
        lines.append(f"visual mesh dir  : {rakuda.visual_mesh_dir}")
    hint = rakuda.visual_mesh_hint()
    if hint:
        lines.append(f"note             : {hint}")
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    """``robopy-models``: show where the models are, fetch the visual meshes."""
    import argparse

    parser = argparse.ArgumentParser(
        prog="robopy-models",
        description="Locate robopy's bundled robot models and fetch their optional visual meshes.",
    )
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("status", help="where the model is and whether its visual meshes are present")
    path_cmd = sub.add_parser("path", help="print a path")
    path_cmd.add_argument(
        "which",
        choices=["urdf", "package-dir", "cache"],
        nargs="?",
        default="urdf",
    )
    fetch_cmd = sub.add_parser("fetch", help="download the visual meshes into the cache")
    fetch_cmd.add_argument("--ref", default="main", help="git ref to fetch from (default: main)")
    fetch_cmd.add_argument("--dest", type=Path, default=None, help="directory instead of the cache")
    fetch_cmd.add_argument("--force", action="store_true", help="re-download existing files")
    fetch_cmd.add_argument("--token", default=None, help="GitHub token (default: $GITHUB_TOKEN)")
    args = parser.parse_args(argv)

    rakuda = find_rakuda_model()
    if args.command in (None, "status"):
        print(status_text(rakuda))
        return 0 if rakuda is not None else 1
    if args.command == "path":
        if args.which == "cache":
            print(visual_mesh_cache_package_dir() / RAKUDA_PACKAGE_NAME / "meshes")
            return 0
        if rakuda is None:
            print("model not found")
            return 1
        print(rakuda.convex_collision_urdf if args.which == "urdf" else rakuda.package_dir)
        return 0
    if args.command == "fetch":
        started = time.monotonic()

        def show(name: str, index: int, total: int) -> None:
            print(f"[{index:3d}/{total}] {name}")

        report = fetch_visual_meshes(
            args.dest, ref=args.ref, token=args.token, force=args.force, progress=show
        )
        print(
            f"downloaded {len(report.downloaded)}, skipped {len(report.skipped)}, "
            f"failed {len(report.failed)} in {time.monotonic() - started:.1f} s -> "
            f"{report.destination}"
        )
        for name, reason in report.failed[:10]:
            print(f"  FAILED {name}: {reason}")
        if len(report.failed) > 10:
            print(f"  ... and {len(report.failed) - 10} more")
        return 0 if report.ok else 1
    parser.error("unknown command")
    return 2


def _unused(*_: Any) -> None:  # pragma: no cover - keeps the import list honest for mypy
    return None
