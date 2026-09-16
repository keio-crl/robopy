"""Robot models that ship with robopy, and where their optional parts live.

robopy is a library: everything the kinematics, the solver, the viewer and the
VR teleoperation need is inside the installed package, so ``pip install
robopy`` is enough.  The Rakuda-2 CAD export lives under
``robopy/models/rakuda/`` as package data:

* the three URDFs and the 137 convex collision meshes (2.9 MB) are always
  installed and are all the model needs to run;
* the 137 *visual* meshes (53 MB of decorative STL) are not in the wheel.  In
  the repository they are Git LFS objects; for an installed library they are
  published as one zip attached to a GitHub Release, and
  :func:`fetch_visual_meshes` (the ``robopy-models fetch`` command) downloads
  that asset into a per-user cache.  Without them the viewer draws the convex
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

import hashlib
import io
import json
import os
import shutil
import tempfile
import time
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, List, Literal, Sequence

__all__ = [
    "BUNDLED_MODELS_DIR",
    "GITHUB_REPO",
    "RAKUDA_PACKAGE_NAME",
    "RAKUDA_VISUAL_MESH_ASSET",
    "RAKUDA_VISUAL_MESH_RELEASE_TAG",
    "FetchReport",
    "RakudaModelFiles",
    "VisualMeshStatus",
    "cache_dir",
    "fetch_visual_meshes",
    "find_models_dir",
    "find_rakuda_model",
    "is_lfs_pointer",
    "release_asset_url",
    "visual_mesh_cache_package_dir",
    "visual_mesh_names",
]

RAKUDA_PACKAGE_NAME = "assembly_2"
VisualMeshStatus = Literal["PRESENT", "LFS_POINTERS", "ABSENT"]
_LFS_MAGIC = b"version https://git-lfs.github.com/spec/"

#: The models shipped inside the installed package (``robopy/models``).
BUNDLED_MODELS_DIR: Path = Path(__file__).resolve().parent

#: The repository whose GitHub Releases carry the visual-mesh asset.
GITHUB_REPO = "keio-crl/robopy"
#: Release tag the installed package fetches from.  Bumped only when the CAD
#: export (and so the bundled URDF) changes; ``scripts/build_visual_mesh_asset.py``
#: builds the asset and the ``release-visual-meshes`` workflow attaches it.
RAKUDA_VISUAL_MESH_RELEASE_TAG = "rakuda-visual-meshes-v1"
#: Name of the zip attached to that release: ``assembly_2/meshes/*.stl`` plus a
#: ``MANIFEST.json`` with a SHA-256 per file.
RAKUDA_VISUAL_MESH_ASSET = "rakuda_visual_meshes.zip"
MANIFEST_NAME = "MANIFEST.json"


def release_asset_url(
    tag: str = RAKUDA_VISUAL_MESH_RELEASE_TAG,
    asset: str = RAKUDA_VISUAL_MESH_ASSET,
    *,
    repo: str = GITHUB_REPO,
) -> str:
    """Direct download URL of a release asset (public repositories)."""
    return f"https://github.com/{repo}/releases/download/{tag}/{asset}"


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
            "the release asset into "
            f"{visual_mesh_cache_package_dir() / RAKUDA_PACKAGE_NAME / 'meshes'}."
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
        source: URL the archive was downloaded from (``None`` when nothing was).
        downloaded: Files extracted this time.
        skipped: Files that were already present (and not pointers).
        failed: ``(name, reason)`` for files that could not be obtained.
        archive_bytes: Size of the downloaded archive.
    """

    destination: Path
    source: str | None = None
    downloaded: List[str] = field(default_factory=list)
    skipped: List[str] = field(default_factory=list)
    failed: List[tuple[str, str]] = field(default_factory=list)
    archive_bytes: int = 0

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
    """GET ``url`` and return the body; redirects (GitHub's asset CDN) are followed."""
    headers = {"User-Agent": "robopy-models"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
        if "api.github.com" in url and "/releases/assets/" in url:
            headers["Accept"] = "application/octet-stream"
    request = urllib.request.Request(url, headers=headers)
    with urllib.request.urlopen(request, timeout=timeout_s) as response:  # noqa: S310 - https only
        return bytes(response.read())


def _api_asset_url(
    repo: str,
    tag: str,
    asset: str,
    token: str,
    opener: Callable[[str, str | None, float], bytes],
    timeout_s: float,
) -> str:
    """Resolve a release asset through the API (needed for private repositories)."""
    listing = json.loads(
        opener(f"https://api.github.com/repos/{repo}/releases/tags/{tag}", token, timeout_s)
    )
    for entry in listing.get("assets", []):
        if entry.get("name") == asset and entry.get("url"):
            return str(entry["url"])
    raise FileNotFoundError(f"Release {tag!r} of {repo} has no asset named {asset!r}.")


def fetch_visual_meshes(
    destination: Path | None = None,
    *,
    tag: str = RAKUDA_VISUAL_MESH_RELEASE_TAG,
    asset: str = RAKUDA_VISUAL_MESH_ASSET,
    url: str | None = None,
    repo: str = GITHUB_REPO,
    token: str | None = None,
    force: bool = False,
    names: Sequence[str] | None = None,
    progress: Callable[[str, int, int], None] | None = None,
    opener: Callable[[str, str | None, float], bytes] | None = None,
    timeout_s: float = 300.0,
) -> FetchReport:
    """Download the Rakuda visual meshes (one release asset) into the cache.

    The STLs are published as a single zip attached to a GitHub Release, so a
    ``pip install`` needs one HTTP request to get them and the wheel stays
    small.  Every extracted file is checked to be an STL (never an LFS pointer
    or an error page), against the archive's ``MANIFEST.json`` SHA-256 when
    present, and the set is checked against the visual meshes the bundled URDF
    references.

    Args:
        destination: Directory for the STLs.  Default: the cache location
            :func:`find_rakuda_model` looks in first.
        tag: Release tag to fetch from.
        asset: Asset file name on that release.
        url: Fetch this URL instead of the release asset (a mirror, a lab
            file server, a local ``file://`` path).
        repo: ``owner/name`` of the GitHub repository.
        token: GitHub token for a private repository; the release is then
            resolved through the API.  ``GITHUB_TOKEN`` is used when ``None``.
        force: Re-extract even when every file is already present.
        names: The STL names expected; default: every ``<visual>`` mesh of
            the bundled URDF.
        progress: Called with ``(name, index, total)`` while extracting.
        opener: ``(url, token, timeout_s) -> bytes``; replaceable for tests.
        timeout_s: Network timeout for the archive download.

    Returns:
        A :class:`FetchReport`.  Nothing is raised for a failed download or a
        bad archive; ``failed`` says what went wrong and nothing invalid is kept.
    """
    dest = (
        Path(destination)
        if destination is not None
        else visual_mesh_cache_package_dir() / RAKUDA_PACKAGE_NAME / "meshes"
    )
    if names is None:
        rakuda = find_rakuda_model()
        if rakuda is None:
            raise FileNotFoundError(
                "The bundled Rakuda model was not found; cannot list its meshes."
            )
        names = visual_mesh_names(rakuda.convex_collision_urdf)
    wanted = list(names)
    report = FetchReport(destination=dest)
    if not force and wanted and all(_real_mesh(dest / name) for name in wanted):
        report.skipped = wanted
        return report

    token = (token if token is not None else os.environ.get("GITHUB_TOKEN")) or None
    fetch = opener or _default_opener
    try:
        if url is None:
            url = (
                _api_asset_url(repo, tag, asset, token, fetch, timeout_s)
                if token
                else release_asset_url(tag, asset, repo=repo)
            )
        report.source = url
        data = fetch(url, token, timeout_s)
    except urllib.error.HTTPError as exc:
        report.source = url
        hint = (
            " (no such release/asset: build it with scripts/build_visual_mesh_asset.py and "
            "attach it, or pass --tag/--url)"
            if exc.code == 404
            else ""
        )
        report.failed = [(n, f"HTTP {exc.code} for {url}{hint}") for n in wanted]
        return report
    except (urllib.error.URLError, OSError, TimeoutError, ValueError) as exc:
        report.source = url
        report.failed = [(n, f"{type(exc).__name__}: {exc}") for n in wanted]
        return report
    report.archive_bytes = len(data)

    try:
        archive = zipfile.ZipFile(io.BytesIO(data))
    except zipfile.BadZipFile:
        reason = "the download is not a zip archive"
        if data[:1] == b"<":
            reason += " (an HTML page: wrong URL, or a private repository without GITHUB_TOKEN)"
        report.failed = [(n, reason) for n in wanted]
        return report

    members: dict[str, zipfile.ZipInfo] = {}
    manifest: dict[str, Any] = {}
    for info in archive.infolist():
        if info.is_dir():
            continue
        base = info.filename.rsplit("/", 1)[-1]
        if base == MANIFEST_NAME:
            try:
                manifest = json.loads(archive.read(info)).get("files", {})
            except (ValueError, AttributeError):
                manifest = {}
            continue
        if not base.lower().endswith(".stl") or ".." in info.filename or base != base.strip():
            continue  # not a mesh, or a name we will not write to disk
        members[base] = info

    dest.mkdir(parents=True, exist_ok=True)
    targets = wanted or sorted(members)
    total = len(targets)
    for index, name in enumerate(targets, start=1):
        if progress is not None:
            progress(name, index, total)
        member = members.get(name)
        if member is None:
            report.failed.append((name, "not in the archive"))
            continue
        content = archive.read(member)
        if not _looks_like_stl(content):
            report.failed.append((name, "archive entry is not an STL (an LFS pointer?)"))
            continue
        expected = manifest.get(name, {}).get("sha256") if isinstance(manifest, dict) else None
        if expected and hashlib.sha256(content).hexdigest() != expected:
            report.failed.append((name, "SHA-256 does not match the archive's manifest"))
            continue
        with tempfile.NamedTemporaryFile(dir=dest, delete=False) as handle:
            handle.write(content)
            temporary = Path(handle.name)
        shutil.move(str(temporary), str(dest / name))
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
    lines.append(f"release asset    : {release_asset_url()}")
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
    fetch_cmd = sub.add_parser(
        "fetch", help="download the visual-mesh release asset into the cache"
    )
    fetch_cmd.add_argument(
        "--tag", default=RAKUDA_VISUAL_MESH_RELEASE_TAG, help="release tag (default: %(default)s)"
    )
    fetch_cmd.add_argument(
        "--asset", default=RAKUDA_VISUAL_MESH_ASSET, help="asset name (default: %(default)s)"
    )
    fetch_cmd.add_argument("--url", default=None, help="fetch this zip URL instead of the release")
    fetch_cmd.add_argument("--repo", default=GITHUB_REPO, help="owner/name (default: %(default)s)")
    fetch_cmd.add_argument("--dest", type=Path, default=None, help="directory instead of the cache")
    fetch_cmd.add_argument("--force", action="store_true", help="re-extract existing files")
    fetch_cmd.add_argument(
        "--token",
        default=None,
        help="GitHub token for a private repository (default: $GITHUB_TOKEN)",
    )
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
        print(f"fetching {args.url or release_asset_url(args.tag, args.asset, repo=args.repo)} ...")

        def show(name: str, index: int, total: int) -> None:
            if index in (1, total) or index % 25 == 0:
                print(f"[{index:3d}/{total}] {name}")

        report = fetch_visual_meshes(
            args.dest,
            tag=args.tag,
            asset=args.asset,
            url=args.url,
            repo=args.repo,
            token=args.token,
            force=args.force,
            progress=show,
        )
        print(
            f"extracted {len(report.downloaded)}, already present {len(report.skipped)}, "
            f"failed {len(report.failed)} ({report.archive_bytes / 1e6:.1f} MB archive) in "
            f"{time.monotonic() - started:.1f} s -> {report.destination}"
        )
        reasons = sorted({reason for _, reason in report.failed})
        for reason in reasons[:5]:
            count = sum(1 for _, r in report.failed if r == reason)
            print(f"  FAILED x{count}: {reason}")
        return 0 if report.ok else 1
    parser.error("unknown command")
    return 2
