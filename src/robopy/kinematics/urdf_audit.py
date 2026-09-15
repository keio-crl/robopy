"""Audit a URDF before trusting it as a kinematic or dynamic model.

This module uses only the standard library, so a model can be inspected without
installing the ``kinematics`` extra.  It answers the questions that decide
whether a file is usable at all:

* how many links and joints there are, and of which types;
* which joints are actually movable, and in what order they appear;
* which frames exist, so a TCP or camera frame can be located by name;
* which meshes are referenced and whether they resolve on disk;
* what the total mass is -- a CAD export frequently carries placeholder
  inertial data that is fine for geometry and useless for dynamics.

Run it directly::

    python -m robopy.kinematics.urdf_audit path/to/robot.urdf --package-dir /path/to/pkgs
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
import zipfile
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

__all__ = [
    "JointInfo",
    "UrdfAudit",
    "audit_urdf",
    "extract_model_archive",
    "resolve_package_path",
]

#: URDF joint types that contribute a degree of freedom.
MOVABLE_JOINT_TYPES: Tuple[str, ...] = ("revolute", "continuous", "prismatic", "planar", "floating")


@dataclass(frozen=True)
class JointInfo:
    """One joint as declared in the URDF.

    Attributes:
        name: Joint name.
        joint_type: URDF joint type string.
        parent: Parent link name.
        child: Child link name.
        axis: Joint axis as declared, or ``None`` for a fixed joint.
        lower: Lower limit in radians (or metres), or ``None``.
        upper: Upper limit, or ``None``.
        effort: Declared effort limit, or ``None``.
        velocity: Declared velocity limit, or ``None``.
    """

    name: str
    joint_type: str
    parent: str
    child: str
    axis: Tuple[float, float, float] | None
    lower: float | None
    upper: float | None
    effort: float | None
    velocity: float | None

    @property
    def is_movable(self) -> bool:
        """Whether this joint contributes at least one degree of freedom."""
        return self.joint_type in MOVABLE_JOINT_TYPES


@dataclass
class UrdfAudit:
    """Structural summary of a URDF file.

    Attributes:
        path: Path of the audited file.
        robot_name: ``<robot name=...>`` attribute.
        root_links: Links that never appear as a joint child.  A well-formed
            single-tree URDF has exactly one.
        link_names: Every link, in document order.
        joints: Every joint, in document order.
        joint_type_counts: Count of joints per type.
        movable_joint_names: Names of the movable joints, in document order.
        mesh_references: Every referenced mesh filename, in document order.
        unresolved_meshes: Mesh references that could not be resolved on disk.
        total_mass_kg: Sum of every ``<mass>`` value.
        links_without_inertial: Links that declare no ``<inertial>`` block.
        has_collision_geometry: Whether any link declares a ``<collision>``.
        ambiguous_names: Names used by both a joint and a link, which makes a
            frame lookup by that name ambiguous.
        warnings: Human-readable warnings about the model.
    """

    path: str
    robot_name: str
    root_links: List[str] = field(default_factory=list)
    link_names: List[str] = field(default_factory=list)
    joints: List[JointInfo] = field(default_factory=list)
    joint_type_counts: Dict[str, int] = field(default_factory=dict)
    movable_joint_names: List[str] = field(default_factory=list)
    mesh_references: List[str] = field(default_factory=list)
    unresolved_meshes: List[str] = field(default_factory=list)
    total_mass_kg: float = 0.0
    links_without_inertial: List[str] = field(default_factory=list)
    has_collision_geometry: bool = False
    ambiguous_names: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def n_links(self) -> int:
        """Number of links."""
        return len(self.link_names)

    @property
    def n_joints(self) -> int:
        """Number of joints."""
        return len(self.joints)

    @property
    def n_movable(self) -> int:
        """Number of movable joints."""
        return len(self.movable_joint_names)

    def joint(self, name: str) -> JointInfo:
        """Look up a joint by name."""
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(f"No joint named '{name}' in {self.path}.")

    def usable_for_dynamics(self, *, min_plausible_mass_kg: float = 0.5) -> bool:
        """Whether the inertial data is plausible enough for dynamics.

        A CAD export whose total mass is a few milligrams describes geometry
        correctly and dynamics not at all.  Returning ``False`` here is a reason
        to build a geometry-only model, never a reason to invent masses.
        """
        return self.total_mass_kg >= min_plausible_mass_kg and not self.links_without_inertial

    def to_json(self, indent: int = 2) -> str:
        """Serialise the audit as JSON."""
        payload = asdict(self)
        payload["n_links"] = self.n_links
        payload["n_joints"] = self.n_joints
        payload["n_movable"] = self.n_movable
        payload["usable_for_dynamics"] = self.usable_for_dynamics()
        return json.dumps(payload, indent=indent, ensure_ascii=False)

    def summary(self) -> str:
        """One-screen human-readable summary."""
        types = ", ".join(f"{k}={v}" for k, v in sorted(self.joint_type_counts.items()))
        lines = [
            f"URDF        : {self.path}",
            f"robot name  : {self.robot_name}",
            f"root link(s): {', '.join(self.root_links) or '(none found)'}",
            f"links/joints: {self.n_links} links, {self.n_joints} joints",
            f"joint types : {types}",
            f"movable DOF : {self.n_movable}",
            f"movable     : {', '.join(self.movable_joint_names)}",
            f"meshes      : {len(self.mesh_references)} referenced, "
            f"{len(self.unresolved_meshes)} unresolved",
            f"collision   : {'present' if self.has_collision_geometry else 'absent'}",
            f"total mass  : {self.total_mass_kg:.7g} kg "
            f"({'plausible' if self.usable_for_dynamics() else 'NOT usable for dynamics'})",
        ]
        if self.warnings:
            lines.append("warnings    :")
            lines.extend(f"  - {w}" for w in self.warnings)
        return "\n".join(lines)


def _parse_floats(text: str | None, count: int) -> Tuple[float, ...] | None:
    if text is None:
        return None
    parts = text.split()
    if len(parts) != count:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def _optional_float(value: str | None) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def resolve_package_path(uri: str, package_dirs: Sequence[Path]) -> Path | None:
    """Resolve a ``package://pkg/rest`` or plain relative mesh URI.

    Args:
        uri: The ``filename`` attribute of a ``<mesh>`` element.
        package_dirs: Directories that may contain the referenced packages.  For
            ``package://assembly_2/meshes/x.stl`` a directory is a match if it
            *is* named ``assembly_2`` or *contains* a directory of that name.

    Returns:
        The resolved path, or ``None`` when nothing matched.
    """
    if uri.startswith("package://"):
        remainder = uri[len("package://") :]
        package, _, rest = remainder.partition("/")
        for base in package_dirs:
            for candidate in (base / package / rest, base / rest):
                if candidate.exists():
                    return candidate
            if base.name == package and (base / rest).exists():
                return base / rest
        return None
    if uri.startswith("file://"):
        candidate = Path(uri[len("file://") :])
        return candidate if candidate.exists() else None
    for base in package_dirs:
        candidate = base / uri
        if candidate.exists():
            return candidate
    direct = Path(uri)
    return direct if direct.exists() else None


def audit_urdf(
    urdf_path: Path | str,
    *,
    package_dirs: Sequence[Path | str] = (),
) -> UrdfAudit:
    """Parse and summarise a URDF file.

    Args:
        urdf_path: Path to the ``.urdf`` file.
        package_dirs: Directories to search when resolving ``package://`` mesh
            references.  The URDF's own parent and grandparent directories are
            always searched as well.

    Returns:
        The :class:`UrdfAudit`.

    Raises:
        FileNotFoundError: If ``urdf_path`` does not exist.
        ValueError: If the file is not a ``<robot>`` document.
    """
    path = Path(urdf_path)
    if not path.exists():
        raise FileNotFoundError(f"URDF not found: {path}")

    tree = ET.parse(path)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError(f"{path} is not a URDF: root element is <{root.tag}>, expected <robot>.")

    search_dirs = [Path(d) for d in package_dirs]
    search_dirs.extend([path.parent, path.parent.parent, path.parent.parent.parent])

    audit = UrdfAudit(path=str(path), robot_name=root.get("name", ""))

    for link in root.findall("link"):
        name = link.get("name", "")
        audit.link_names.append(name)
        inertial = link.find("inertial")
        if inertial is None:
            audit.links_without_inertial.append(name)
        else:
            mass_el = inertial.find("mass")
            if mass_el is not None:
                audit.total_mass_kg += _optional_float(mass_el.get("value")) or 0.0
        if link.find("collision") is not None:
            audit.has_collision_geometry = True

    for joint in root.findall("joint"):
        joint_type = joint.get("type", "")
        axis_el = joint.find("axis")
        limit_el = joint.find("limit")
        parent_el = joint.find("parent")
        child_el = joint.find("child")
        axis = _parse_floats(axis_el.get("xyz") if axis_el is not None else None, 3)
        info = JointInfo(
            name=joint.get("name", ""),
            joint_type=joint_type,
            parent=parent_el.get("link", "") if parent_el is not None else "",
            child=child_el.get("link", "") if child_el is not None else "",
            axis=axis,  # type: ignore[arg-type]
            lower=_optional_float(limit_el.get("lower") if limit_el is not None else None),
            upper=_optional_float(limit_el.get("upper") if limit_el is not None else None),
            effort=_optional_float(limit_el.get("effort") if limit_el is not None else None),
            velocity=_optional_float(limit_el.get("velocity") if limit_el is not None else None),
        )
        audit.joints.append(info)
        audit.joint_type_counts[joint_type] = audit.joint_type_counts.get(joint_type, 0) + 1
        if info.is_movable:
            audit.movable_joint_names.append(info.name)

    children = {j.child for j in audit.joints}
    audit.root_links = [name for name in audit.link_names if name not in children]

    for mesh in root.iter("mesh"):
        uri = mesh.get("filename")
        if uri is None:
            continue
        audit.mesh_references.append(uri)
        if resolve_package_path(uri, search_dirs) is None:
            audit.unresolved_meshes.append(uri)

    # --- warnings ---------------------------------------------------------
    if len(audit.root_links) != 1:
        audit.warnings.append(
            f"Expected exactly one root link, found {len(audit.root_links)}: {audit.root_links}"
        )
    if not audit.usable_for_dynamics():
        audit.warnings.append(
            f"Total mass is {audit.total_mass_kg:.7g} kg. This model is usable for geometry only; "
            "do not use it as a dynamic model and do not edit the masses to make it look valid."
        )
    if audit.unresolved_meshes:
        audit.warnings.append(
            f"{len(audit.unresolved_meshes)} mesh reference(s) did not resolve; pass "
            "--package-dir so that package:// URIs can be found."
        )
    placeholder_limits = [
        j.name for j in audit.joints if j.is_movable and j.effort == 1.0 and j.velocity == 1.0
    ]
    if placeholder_limits:
        audit.warnings.append(
            f"Joint(s) {placeholder_limits} declare effort=1, velocity=1. These look like CAD "
            "exporter placeholders, not machine specifications; do not adopt them as limits."
        )
    continuous = [j.name for j in audit.joints if j.joint_type == "continuous"]
    if continuous:
        audit.warnings.append(
            f"Continuous joint(s) {continuous} have no URDF range. They need an explicit soft "
            "limit from the real machine (cable routing), otherwise shortest-angle motion will "
            "ignore a real constraint."
        )
    audit.ambiguous_names = sorted({j.name for j in audit.joints} & set(audit.link_names))
    shared_names = audit.ambiguous_names
    if shared_names:
        audit.warnings.append(
            f"Name(s) {shared_names} are used by both a joint and a link. Frame lookups by name "
            "are then ambiguous (Pinocchio holds a FIXED_JOINT and a BODY frame of that name), "
            "so attach a uniquely named operational frame instead of addressing these directly."
        )
    fixed_dof_named = [j.name for j in audit.joints if not j.is_movable and j.name.endswith("_dof")]
    if fixed_dof_named:
        audit.warnings.append(
            f"Joint(s) {fixed_dof_named} are named '*_dof' but are fixed. A name is not a degree "
            "of freedom; do not treat them as movable."
        )
    return audit


def extract_model_archive(archive: Path | str, destination: Path | str) -> Path:
    """Extract a robot-model ZIP, refusing entries that escape ``destination``.

    Args:
        archive: Path to the ``.zip`` file.
        destination: Directory to extract into; created if needed.

    Returns:
        The destination directory.

    Raises:
        ValueError: If an archive member would be written outside ``destination``.
    """
    dest = Path(destination).resolve()
    dest.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive) as zf:
        for member in zf.namelist():
            target = (dest / member).resolve()
            if not str(target).startswith(str(dest)):
                raise ValueError(f"Archive member '{member}' would escape {dest}.")
        zf.extractall(dest)
    return dest


def main(argv: Sequence[str] | None = None) -> int:
    """Command-line entry point for the URDF audit."""
    parser = argparse.ArgumentParser(description="Audit a URDF file before using it.")
    parser.add_argument("urdf", type=Path, help="Path to the .urdf file")
    parser.add_argument(
        "--package-dir",
        type=Path,
        action="append",
        default=[],
        dest="package_dirs",
        help="Directory to search when resolving package:// mesh URIs (repeatable)",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of a summary")
    args = parser.parse_args(argv)

    audit = audit_urdf(args.urdf, package_dirs=args.package_dirs)
    print(audit.to_json() if args.json else audit.summary())
    return 0 if not audit.warnings else 1


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    sys.exit(main())
