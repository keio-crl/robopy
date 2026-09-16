"""Build the ``rakuda_visual_meshes.zip`` release asset.

The visual meshes of the Rakuda model are Git LFS objects in this repository
and are not part of the wheel.  An installed robopy fetches them from a
GitHub Release asset (see :mod:`robopy.models`).  This script builds that
asset from a checkout whose LFS content has been pulled::

    git lfs install && git lfs pull
    uv run python scripts/build_visual_mesh_asset.py --out dist/rakuda_visual_meshes.zip
    gh release create rakuda-visual-meshes-v1 dist/rakuda_visual_meshes.zip \\
        --title "Rakuda visual meshes v1" --notes "Visual STLs of the Rakuda-2 export."

The ``release-visual-meshes`` GitHub Actions workflow does the same when a
``rakuda-visual-meshes-v*`` tag is pushed.  The archive holds
``assembly_2/meshes/<name>.stl`` for every visual mesh the bundled URDF
references, plus ``MANIFEST.json`` with a SHA-256 per file; the fetcher
checks both.  LFS pointer files are refused: an asset built from a checkout
without ``git lfs pull`` would be worse than no asset.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import zipfile
from pathlib import Path
from typing import Sequence

from robopy.models import (
    MANIFEST_NAME,
    RAKUDA_PACKAGE_NAME,
    RAKUDA_VISUAL_MESH_ASSET,
    RAKUDA_VISUAL_MESH_RELEASE_TAG,
    find_rakuda_model,
    is_lfs_pointer,
    visual_mesh_names,
)


def build(source: Path, out: Path, names: Sequence[str]) -> dict[str, dict[str, object]]:
    """Write the archive; return the manifest's ``files`` table."""
    missing = [n for n in names if not (source / n).is_file()]
    pointers = [n for n in names if (source / n).is_file() and is_lfs_pointer(source / n)]
    if missing:
        raise SystemExit(
            f"{len(missing)} visual mesh(es) missing under {source}: {missing[:3]} ..."
        )
    if pointers:
        raise SystemExit(
            f"{len(pointers)} file(s) under {source} are Git LFS pointers, not meshes. "
            "Run `git lfs install && git lfs pull` first."
        )
    files: dict[str, dict[str, object]] = {}
    out.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for name in names:
            data = (source / name).read_bytes()
            files[name] = {"sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data)}
            archive.writestr(f"{RAKUDA_PACKAGE_NAME}/meshes/{name}", data)
        manifest = {
            "asset": out.name,
            "release_tag": RAKUDA_VISUAL_MESH_RELEASE_TAG,
            "package": RAKUDA_PACKAGE_NAME,
            "count": len(files),
            "files": files,
        }
        archive.writestr(MANIFEST_NAME, json.dumps(manifest, indent=1, sort_keys=True))
    return files


def main(argv: Sequence[str] | None = None) -> int:
    """Build the asset and print a summary."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--source",
        type=Path,
        default=None,
        help="directory with the visual STLs (default: the package's assembly_2/meshes)",
    )
    parser.add_argument(
        "--out", type=Path, default=Path("dist") / RAKUDA_VISUAL_MESH_ASSET, help="zip to write"
    )
    args = parser.parse_args(argv)

    rakuda = find_rakuda_model()
    if rakuda is None:
        print("bundled Rakuda model not found", file=sys.stderr)
        return 1
    names = visual_mesh_names(rakuda.convex_collision_urdf)
    source = args.source or rakuda.meshes_dir
    files = build(source, args.out, names)
    total = sum(int(entry["bytes"]) for entry in files.values())  # type: ignore[call-overload]
    print(
        f"{args.out}: {len(files)} meshes, {total / 1e6:.1f} MB raw, "
        f"{args.out.stat().st_size / 1e6:.1f} MB compressed; manifest {MANIFEST_NAME}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
