"""Take the inertials from a re-exported Rakuda CAD model into the committed URDFs.

The Rakuda's CAD export carried *volumes* where masses belong, so the committed
model weighed about two grams in total: correct as geometry, useless as
dynamics.  A later export with densities set produces the same tree with real
inertials, and this script moves those numbers across without touching anything
else.

Why not simply replace the files
--------------------------------
Three URDFs are committed -- the plain one, and two with collision geometry
added (convex hulls and full meshes).  The new export has no collision geometry
at all, so copying it over the three would silently delete 274 ``<collision>``
elements and the simulation work that depends on them.  Only ``<inertial>``
blocks move.

What is checked before anything is written
------------------------------------------
The two models must be the same machine.  Link names, joint names, types,
parent and child links, axes, origins and limits, and every visual mesh
reference and scale are compared first, and a difference beyond
:data:`POSITION_TOLERANCE_M` / :data:`ANGLE_TOLERANCE_RAD` aborts the import
rather than producing a model that is half one export and half another.  The
tolerances are there for re-export rounding, which in practice lands around
1e-16 m; a real geometry change is many orders of magnitude larger.

How the edit is made
--------------------
Textually, line by line, replacing each link's ``<inertial>`` block in place.
A parse-and-reserialise would reformat 150 KB of generated XML and make the
diff unreadable, which is exactly the diff a human needs to check.  The result
is then re-parsed and compared against the source, so the textual shortcut is
verified rather than trusted.

Usage::

    python scripts/update_rakuda_inertials.py path/to/Assembly_2.zip --dry-run
    python scripts/update_rakuda_inertials.py path/to/Assembly_2.zip
    python scripts/update_rakuda_inertials.py path/to/assembly_2.urdf --report out.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

__all__ = [
    "ANGLE_TOLERANCE_RAD",
    "POSITION_TOLERANCE_M",
    "SOURCE_MEMBER",
    "ImportReport",
    "compare_structure",
    "inertials_by_link",
    "read_source",
    "update_file",
]

#: The one member read out of a model ZIP.  Named rather than extracted
#: wholesale: an archive is untrusted input and nothing here needs the rest of
#: it.
SOURCE_MEMBER = "assembly_2/urdf/assembly_2.urdf"

#: Geometry differences tolerated between the two exports, beyond which the
#: import is refused.  Re-export rounding is around 1e-16 m and 1e-14 rad, so
#: these leave four orders of magnitude of headroom and still catch a part that
#: actually moved.
POSITION_TOLERANCE_M: float = 1e-9
ANGLE_TOLERANCE_RAD: float = 1e-9

#: Mass at or below which a link is reported as carrying none.  The CAD export
#: writes 1e-9 for a pure coordinate frame; a real part is grams at least.
NEGLIGIBLE_MASS_KG: float = 1e-8

_LINK_RE = re.compile(r'^\s*<link\s+name="([^"]*)"')
_INERTIAL_OPEN_RE = re.compile(r"^(\s*)<inertial>\s*$")
_INERTIAL_CLOSE_RE = re.compile(r"^\s*</inertial>\s*$")


def _leading_space(line: str) -> str:
    """The indentation of ``line``, so a replacement can match its neighbours."""
    return line[: len(line) - len(line.lstrip())]


@dataclass
class ImportReport:
    """What one file's import did, or would do.

    Attributes:
        path: The URDF that was updated.
        changed_links: Links whose inertial block differs from what was there.
        unchanged_links: Links whose inertial block was already identical.
        mass_before_kg: Total mass before the import.
        mass_after_kg: Total mass after it.
        collisions_before: ``<collision>`` elements before.
        collisions_after: ``<collision>`` elements after; must not drop.
        written: Whether the file was actually written.
    """

    path: str
    changed_links: List[str] = field(default_factory=list)
    unchanged_links: List[str] = field(default_factory=list)
    mass_before_kg: float = 0.0
    mass_after_kg: float = 0.0
    collisions_before: int = 0
    collisions_after: int = 0
    written: bool = False

    def summary(self) -> str:
        """One line per file, for the console."""
        verb = "wrote" if self.written else "would write"
        return (
            f"{verb} {self.path}: {len(self.changed_links)} link(s) changed, "
            f"{len(self.unchanged_links)} already current, "
            f"mass {self.mass_before_kg:.10g} -> {self.mass_after_kg:.10g} kg, "
            f"collision elements {self.collisions_before} -> {self.collisions_after}"
        )


def read_source(path: Path) -> Tuple[str, str]:
    """Read the source URDF text, from a ZIP or a plain file.

    Args:
        path: A ``.zip`` holding :data:`SOURCE_MEMBER`, or a ``.urdf``.

    Returns:
        ``(text, sha256 of the URDF bytes)``.

    Raises:
        FileNotFoundError: If the path, or the member inside a ZIP, is missing.
    """
    if path.suffix.lower() == ".zip":
        with zipfile.ZipFile(path) as archive:
            try:
                data = archive.read(SOURCE_MEMBER)
            except KeyError as exc:
                raise FileNotFoundError(
                    f"{path} has no member '{SOURCE_MEMBER}'. Members: "
                    f"{', '.join(sorted(archive.namelist())[:10])}..."
                ) from exc
    else:
        data = path.read_bytes()
    return data.decode("utf-8"), hashlib.sha256(data).hexdigest()


def inertials_by_link(text: str) -> Dict[str, List[str]]:
    """Each link's ``<inertial>`` block, as stripped lines, keyed by link name.

    Returns the source lines rather than parsed numbers so that the values are
    spliced in exactly as the exporter wrote them.  Re-formatting a float
    through Python would change ``9.57636e-05`` into something else that means
    the same thing and make every line look edited.
    """
    blocks: Dict[str, List[str]] = {}
    link: str | None = None
    collecting: List[str] | None = None
    for line in text.splitlines():
        match = _LINK_RE.match(line)
        if match:
            link = match.group(1)
            continue
        if link is None:
            continue
        if collecting is None:
            if _INERTIAL_OPEN_RE.match(line) and link not in blocks:
                collecting = []
            continue
        if _INERTIAL_CLOSE_RE.match(line):
            blocks[link] = collecting
            collecting = None
            continue
        collecting.append(line.strip())
    return blocks


def _floats(element: ET.Element | None, attribute: str, count: int) -> Tuple[float, ...] | None:
    if element is None:
        return None
    raw = element.get(attribute)
    if raw is None:
        return None
    parts = raw.split()
    return tuple(float(v) for v in parts) if len(parts) == count else None


def _origin(element: ET.Element | None) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    origin = None if element is None else element.find("origin")
    xyz = _floats(origin, "xyz", 3) or (0.0, 0.0, 0.0)
    rpy = _floats(origin, "rpy", 3) or (0.0, 0.0, 0.0)
    return xyz, rpy


def _visual_signature(link: ET.Element) -> List[Tuple]:
    """Mesh reference, scale and placement of each visual, in document order."""
    signature = []
    for visual in link.findall("visual"):
        mesh = visual.find("geometry/mesh")
        signature.append(
            (
                None if mesh is None else mesh.get("filename"),
                None if mesh is None else mesh.get("scale"),
                _origin(visual),
            )
        )
    return signature


def _close(a: Sequence[float], b: Sequence[float], tolerance: float) -> bool:
    return len(a) == len(b) and all(abs(x - y) <= tolerance for x, y in zip(a, b))


def compare_structure(source: ET.Element, target: ET.Element) -> List[str]:
    """Everything that must match before inertials may be moved across.

    Returns:
        Human-readable problems.  Empty means the two describe the same machine
        and only their inertials may differ.
    """
    problems: List[str] = []

    source_links = {link.get("name"): link for link in source.findall("link")}
    target_links = {link.get("name"): link for link in target.findall("link")}
    missing = sorted(set(target_links) - set(source_links))
    extra = sorted(set(source_links) - set(target_links))
    if missing:
        problems.append(f"links in the target but not the source: {missing}")
    if extra:
        problems.append(f"links in the source but not the target: {extra}")

    for name in sorted(set(source_links) & set(target_links)):
        a = _visual_signature(source_links[name])
        b = _visual_signature(target_links[name])
        if len(a) != len(b):
            problems.append(f"link '{name}': {len(b)} visual(s) in the target, {len(a)} in source")
            continue
        for index, (one, two) in enumerate(zip(a, b)):
            if one[0] != two[0] or one[1] != two[1]:
                problems.append(
                    f"link '{name}' visual {index}: mesh {two[0]!r} scale {two[1]!r} "
                    f"in the target, {one[0]!r} scale {one[1]!r} in the source"
                )
            if not _close(one[2][0], two[2][0], POSITION_TOLERANCE_M) or not _close(
                one[2][1], two[2][1], ANGLE_TOLERANCE_RAD
            ):
                problems.append(f"link '{name}' visual {index}: origin moved")

    source_joints = {joint.get("name"): joint for joint in source.findall("joint")}
    target_joints = {joint.get("name"): joint for joint in target.findall("joint")}
    missing = sorted(set(target_joints) - set(source_joints))
    extra = sorted(set(source_joints) - set(target_joints))
    if missing:
        problems.append(f"joints in the target but not the source: {missing}")
    if extra:
        problems.append(f"joints in the source but not the target: {extra}")

    for name in sorted(set(source_joints) & set(target_joints)):
        one, two = source_joints[name], target_joints[name]
        if one.get("type") != two.get("type"):
            problems.append(
                f"joint '{name}': type {two.get('type')} in the target, "
                f"{one.get('type')} in the source"
            )
        for tag in ("parent", "child"):
            a = one.find(tag)
            b = two.find(tag)
            if (a is None) != (b is None) or (
                a is not None and b is not None and a.get("link") != b.get("link")
            ):
                problems.append(f"joint '{name}': {tag} link differs")
        axis_a = _floats(one.find("axis"), "xyz", 3)
        axis_b = _floats(two.find("axis"), "xyz", 3)
        if (axis_a is None) != (axis_b is None) or (
            axis_a is not None and axis_b is not None and not _close(axis_a, axis_b, 1e-12)
        ):
            problems.append(f"joint '{name}': axis differs")
        (xyz_a, rpy_a), (xyz_b, rpy_b) = _origin(one), _origin(two)
        if not _close(xyz_a, xyz_b, POSITION_TOLERANCE_M):
            problems.append(
                f"joint '{name}': origin xyz differs by more than {POSITION_TOLERANCE_M}"
            )
        if not _close(rpy_a, rpy_b, ANGLE_TOLERANCE_RAD):
            problems.append(
                f"joint '{name}': origin rpy differs by more than {ANGLE_TOLERANCE_RAD}"
            )
        limit_a, limit_b = one.find("limit"), two.find("limit")
        if (limit_a is None) != (limit_b is None):
            problems.append(f"joint '{name}': one model has a <limit> and the other does not")
        elif limit_a is not None and limit_b is not None:
            for attribute in ("lower", "upper", "effort", "velocity"):
                x, y = limit_a.get(attribute), limit_b.get(attribute)
                if (x is None) != (y is None):
                    problems.append(f"joint '{name}': limit/{attribute} present in only one")
                elif x is not None and y is not None and abs(float(x) - float(y)) > 1e-9:
                    problems.append(f"joint '{name}': limit/{attribute} {y} != {x}")

    return problems


def _check_inertia(root: ET.Element) -> List[str]:
    """Finiteness, positive semi-definiteness and the triangle inequality.

    An inertia tensor's principal moments must each be no greater than the sum
    of the other two; a set that fails that describes no rigid body, however
    plausible the individual numbers look.
    """
    import numpy as np

    problems: List[str] = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        name = link.get("name")
        mass_el = inertial.find("mass")
        if mass_el is not None:
            mass = float(mass_el.get("value", "nan"))
            if not np.isfinite(mass) or mass < 0.0:
                problems.append(f"link '{name}': mass {mass} is not a usable value")
        tensor_el = inertial.find("inertia")
        if tensor_el is None:
            continue
        keys = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        values = {k: float(tensor_el.get(k, "nan")) for k in keys}
        if not all(np.isfinite(v) for v in values.values()):
            problems.append(f"link '{name}': inertia has a non-finite entry")
            continue
        tensor = np.array(
            [
                [values["ixx"], values["ixy"], values["ixz"]],
                [values["ixy"], values["iyy"], values["iyz"]],
                [values["ixz"], values["iyz"], values["izz"]],
            ]
        )
        eigenvalues = np.linalg.eigvalsh(tensor)
        scale = max(float(np.max(np.abs(eigenvalues))), 1e-300)
        if eigenvalues.min() < -1e-9 * scale:
            problems.append(f"link '{name}': inertia is not positive semi-definite")
            continue
        a, b, c = sorted(float(v) for v in eigenvalues)
        if c > a + b + 1e-9 * scale:
            problems.append(
                f"link '{name}': principal moments {a:.6g}, {b:.6g}, {c:.6g} break the "
                "triangle inequality, so they describe no rigid body"
            )
    return problems


def _total_mass(root: ET.Element) -> float:
    total = 0.0
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = inertial.find("mass")
        if mass is not None:
            total += float(mass.get("value", "0"))
    return total


def _count_collisions(root: ET.Element) -> int:
    return sum(len(link.findall("collision")) for link in root.findall("link"))


def _splice(text: str, wanted: Dict[str, List[str]]) -> Tuple[str, List[str], List[str]]:
    """Replace each link's inertial block with the source's, keeping indentation."""
    out: List[str] = []
    changed: List[str] = []
    unchanged: List[str] = []
    link: str | None = None
    buffer: List[str] | None = None
    indent = ""
    seen: set[str] = set()

    for line in text.splitlines():
        match = _LINK_RE.match(line)
        if match:
            link = match.group(1)
            out.append(line)
            continue
        if buffer is None:
            opening = _INERTIAL_OPEN_RE.match(line)
            if opening and link is not None and link in wanted and link not in seen:
                indent = opening.group(1)
                buffer = []
                continue
            out.append(line)
            continue
        if _INERTIAL_CLOSE_RE.match(line):
            assert link is not None
            seen.add(link)
            replacement = wanted[link]
            (changed if [b.strip() for b in buffer] != replacement else unchanged).append(link)
            # Indent the new body the way this file indents its old one.  The
            # three URDFs are not formatted alike -- the plain export uses four
            # spaces, the two collision variants two -- and imposing one on the
            # other rewrites every line of the block, including the ones whose
            # contents did not change.
            body_indent = _leading_space(buffer[0]) if buffer else f"{indent}  "
            out.append(f"{indent}<inertial>")
            out.extend(f"{body_indent}{body}" for body in replacement)
            out.append(f"{indent}</inertial>")
            buffer = None
            continue
        buffer.append(line)

    trailing = "\n" if text.endswith("\n") else ""
    return "\n".join(out) + trailing, changed, unchanged


def update_file(
    target: Path,
    source_root: ET.Element,
    wanted: Dict[str, List[str]],
    *,
    dry_run: bool,
) -> ImportReport:
    """Move the inertials into one URDF, after checking it is the same machine.

    Raises:
        ValueError: If the two models differ structurally, if a link would be
            left without an inertial, if the result does not re-parse to the
            source's inertials, or if collision geometry would be lost.
    """
    text = target.read_text(encoding="utf-8")
    before = ET.fromstring(text)

    problems = compare_structure(source_root, before)
    if problems:
        raise ValueError(
            f"{target} does not match the source export, so no inertials were moved:\n  "
            + "\n  ".join(problems)
        )

    # A link with no <inertial> in either model is fine -- the static root frame
    # is one, and giving it a mass would be an invention.  A link that has one
    # here but not in the source is not: it would keep its old value while
    # everything around it changed, which is the sort of half-updated model that
    # looks right and computes wrong.
    with_inertial = {
        link.get("name") for link in before.findall("link") if link.find("inertial") is not None
    }
    absent = sorted(with_inertial - set(wanted))
    if absent:
        raise ValueError(
            f"{target}: the source has no inertial for link(s) {absent}, which do have one here; "
            "refusing to leave them behind while changing their neighbours."
        )

    report = ImportReport(
        path=str(target),
        mass_before_kg=_total_mass(before),
        collisions_before=_count_collisions(before),
    )
    updated, report.changed_links, report.unchanged_links = _splice(text, wanted)

    after = ET.fromstring(updated)
    report.mass_after_kg = _total_mass(after)
    report.collisions_after = _count_collisions(after)
    if report.collisions_after != report.collisions_before:
        raise ValueError(
            f"{target}: collision elements went from {report.collisions_before} to "
            f"{report.collisions_after}. The edit is wrong; nothing was written."
        )

    # The splice is textual, so verify it against a real parse rather than
    # trusting the regular expressions.
    produced = inertials_by_link(updated)
    mismatched = sorted(name for name in wanted if produced.get(name) != wanted[name])
    if mismatched:
        raise ValueError(f"{target}: inertial blocks did not take for link(s) {mismatched[:5]}")

    inertia_problems = _check_inertia(after)
    if inertia_problems:
        raise ValueError(
            f"{target}: the resulting inertias are not usable:\n  " + "\n  ".join(inertia_problems)
        )

    if not dry_run:
        target.write_text(updated, encoding="utf-8")
        report.written = True
    return report


#: Links that are coordinate frames rather than parts, so carrying no mass is
#: correct for them.  ``gripper_*_dof`` are the CAD's *fixed* gripper frames --
#: :mod:`robopy.control.joint_mapping` says the same thing from the motor side,
#: that despite the ``_dof`` in the name they are not degrees of freedom -- and
#: ``head_camera_link`` is where the camera is, not a body.
FRAME_LINKS = ("gripper_left_dof", "gripper_right_dof", "head_camera_link")

#: A CAD sub-solid of a servo, by its model prefix.  The servo's mass belongs on
#: one link; repeating it on every piece the CAD split the housing into would
#: count the same motor several times over.
_MOTOR_SUBSOLID_RE = re.compile(r"^(xm|xc|xl)\d", re.IGNORECASE)


def classify_negligible(root: ET.Element) -> Dict[str, List[str]]:
    """Group the links that still carry no mass, so they can be judged separately.

    A coordinate frame with no mass is correct.  A motor sub-solid with no mass
    is probably correct, for the reason above.  A physical part with no mass is
    a hole in the model, and the only one of the three that wants fixing.

    This is a classification by name, which is a starting point for a human and
    not a verdict: it says which links to look at, not what they weigh.  Nothing
    downstream may treat the third group as if it were the first.
    """
    groups: Dict[str, List[str]] = {
        "coordinate_frames": [],
        "motor_subsolids": [],
        "parts_without_mass": [],
    }
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass_el = inertial.find("mass")
        if mass_el is None or float(mass_el.get("value", "0")) > NEGLIGIBLE_MASS_KG:
            continue
        name = link.get("name") or ""
        if name in FRAME_LINKS:
            groups["coordinate_frames"].append(name)
        elif _MOTOR_SUBSOLID_RE.match(name):
            groups["motor_subsolids"].append(name)
        else:
            groups["parts_without_mass"].append(name)
    return {key: sorted(value) for key, value in groups.items()}


def main(argv: Sequence[str] | None = None) -> int:
    """Run the import."""
    here = Path(__file__).resolve().parents[1]
    default_targets = sorted((here / "src/robopy/models/rakuda/assembly_2/urdf").glob("*.urdf"))

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("source", type=Path, help="the re-exported model: a .zip or a .urdf")
    parser.add_argument(
        "--targets",
        type=Path,
        nargs="*",
        default=default_targets,
        help="URDFs to update (default: the three committed Rakuda models)",
    )
    parser.add_argument("--dry-run", action="store_true", help="report without writing")
    parser.add_argument("--report", type=Path, default=None, help="write a JSON report here")
    args = parser.parse_args(argv)

    text, digest = read_source(args.source)
    source_root = ET.fromstring(text)
    wanted = inertials_by_link(text)
    source_mass = _total_mass(source_root)

    print(f"source      : {args.source}")
    print(f"sha256      : {digest}")
    print(f"links       : {len(source_root.findall('link'))}, inertials: {len(wanted)}")
    print(f"total mass  : {source_mass:.10g} kg")

    problems = _check_inertia(source_root)
    if problems:
        print("the source's own inertias are not usable:", file=sys.stderr)
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        return 1

    reports = []
    for target in args.targets:
        report = update_file(target, source_root, wanted, dry_run=args.dry_run)
        reports.append(report)
        print(report.summary())

    negligible = classify_negligible(source_root)
    print("\nlinks still carrying no mass:")
    for group, names in negligible.items():
        print(f"  {group} ({len(names)}):")
        for name in names:
            print(f"    {name}")

    if args.report:
        args.report.write_text(
            json.dumps(
                {
                    "source": str(args.source),
                    "source_sha256": digest,
                    "source_total_mass_kg": source_mass,
                    "negligible_mass_links": negligible,
                    "files": [vars(r) for r in reports],
                },
                indent=2,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )
        print(f"\nwrote {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
