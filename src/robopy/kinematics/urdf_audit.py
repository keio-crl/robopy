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
import math
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

#: Mass at or below which a link is treated as carrying none.  CAD exports
#: write 1e-9 for a pure coordinate frame; a real part is grams at least.
NEGLIGIBLE_MASS_KG: float = 1e-8


def _principal_moments(
    ixx: float, ixy: float, ixz: float, iyy: float, iyz: float, izz: float
) -> Tuple[float, float, float]:
    """Eigenvalues of a symmetric 3x3 inertia tensor, in closed form.

    Written out rather than delegated to NumPy so this module keeps its promise
    of needing nothing but the standard library -- a URDF should be inspectable
    on a machine that has none of the simulation extras installed.  The formula
    is the standard trigonometric solution for the symmetric case, which is
    exact here and avoids an iterative solver.

    Returns:
        The three principal moments, ascending.
    """
    trace = (ixx + iyy + izz) / 3.0
    # Deviatoric part; its invariants give the eigenvalues directly.
    a, b, c = ixx - trace, iyy - trace, izz - trace
    p2 = a * a + b * b + c * c + 2.0 * (ixy * ixy + ixz * ixz + iyz * iyz)
    if p2 <= 0.0:
        return (trace, trace, trace)
    p = math.sqrt(p2 / 6.0)
    # determinant of the deviatoric tensor, divided by p**3
    det = a * (b * c - iyz * iyz) - ixy * (ixy * c - iyz * ixz) + ixz * (ixy * iyz - b * ixz)
    r = det / (2.0 * p * p * p)
    phi = math.acos(max(-1.0, min(1.0, r))) / 3.0
    first = trace + 2.0 * p * math.cos(phi)
    third = trace + 2.0 * p * math.cos(phi + 2.0 * math.pi / 3.0)
    second = 3.0 * trace - first - third
    return tuple(sorted((first, second, third)))  # type: ignore[return-value]


def _inertia_problem(name: str, values: Dict[str, float]) -> str | None:
    """Why this tensor describes no rigid body, or ``None`` if it does.

    Two conditions, both necessary: the tensor must be positive semi-definite,
    and its principal moments must satisfy the triangle inequality -- no single
    moment may exceed the sum of the other two.  Numbers can look entirely
    reasonable one at a time and still fail the second.
    """
    if not all(math.isfinite(v) for v in values.values()):
        return f"link '{name}': inertia has a non-finite entry"
    moments = _principal_moments(
        values["ixx"], values["ixy"], values["ixz"], values["iyy"], values["iyz"], values["izz"]
    )
    scale = max(abs(m) for m in moments) or 1.0
    if moments[0] < -1e-9 * scale:
        return f"link '{name}': inertia is not positive semi-definite (moments {moments})"
    if moments[2] > moments[0] + moments[1] + 1e-9 * scale:
        return (
            f"link '{name}': principal moments {moments[0]:.6g}, {moments[1]:.6g}, "
            f"{moments[2]:.6g} break the triangle inequality, so they describe no rigid body"
        )
    return None


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
        root_links_without_inertial: Of those, the ones that are root links.
            A static frame bolted to the world carries no mass by definition,
            and its absence is not a defect.
        massless_moving_links: Links inside a movable subtree that declare no
            inertial, or a negligible mass.  Coordinate frames and duplicated
            CAD sub-solids belong here legitimately, so this is a list to look
            through rather than a list of faults.
        massless_moving_parts: The subset of those that carry visual or
            collision geometry.  These *are* faults: something the model draws,
            hanging off a joint, that weighs nothing.  A frame has no geometry,
            which is how the two are told apart without guessing from names.
        link_masses_kg: Mass of every link that declares one.
        movable_subtree_masses_kg: Total mass carried below each movable joint.
            This is what a joint has to hold up, and the figure that says
            whether a model's masses are distributed plausibly -- a total that
            looks right can still have all of it in the base.
        invalid_inertias: Links whose inertia tensor describes no rigid body.
        hardware_validated: Whether these numbers were checked against the
            actual machine.  Never inferred: no combination of numerical checks
            can establish it, and it stays ``False`` until something that
            measured the robot says otherwise.
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
    root_links_without_inertial: List[str] = field(default_factory=list)
    massless_moving_links: List[str] = field(default_factory=list)
    massless_moving_parts: List[str] = field(default_factory=list)
    link_masses_kg: Dict[str, float] = field(default_factory=dict)
    movable_subtree_masses_kg: Dict[str, float] = field(default_factory=dict)
    invalid_inertias: List[str] = field(default_factory=list)
    has_collision_geometry: bool = False
    ambiguous_names: List[str] = field(default_factory=list)
    hardware_validated: bool = False
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

    def numerically_consistent(self, *, min_plausible_mass_kg: float = 0.5) -> bool:
        """Whether the numbers in the file describe a coherent set of rigid bodies.

        Three things, all of them properties of the file:

        * the total mass is not obviously placeholder data;
        * nothing that moves is massless -- a link on a root frame may be, a
          link swinging on a joint may not;
        * every inertia tensor describes a rigid body.

        This says the model is internally sound.  It says **nothing** about
        whether the model matches the machine: that is
        :attr:`hardware_validated`, which no amount of arithmetic can establish.
        Passing here is a reason to compute with the model, not a reason to put
        current through a motor.
        """
        return (
            self.total_mass_kg >= min_plausible_mass_kg
            and not self.massless_moving_parts
            and not self.invalid_inertias
        )

    def usable_for_dynamics(self, *, min_plausible_mass_kg: float = 0.5) -> bool:
        """Deprecated alias of :meth:`numerically_consistent`.

        The old name invited the reading it was given: that a model passing it
        was fit to drive hardware.  It never meant that, and the two questions
        now have two names.  Kept so existing callers and the recorded audit
        JSON keep working; prefer :meth:`numerically_consistent`, and
        :attr:`hardware_validated` when the question is about the real robot.
        """
        return self.numerically_consistent(min_plausible_mass_kg=min_plausible_mass_kg)

    def to_json(self, indent: int = 2) -> str:
        """Serialise the audit as JSON."""
        payload = asdict(self)
        payload["n_links"] = self.n_links
        payload["n_joints"] = self.n_joints
        payload["n_movable"] = self.n_movable
        payload["numerically_consistent"] = self.numerically_consistent()
        # Kept alongside the new name so a reader of an older report, and the
        # committed audit record, still find the key they expect.
        payload["usable_for_dynamics"] = payload["numerically_consistent"]
        return json.dumps(payload, indent=indent, ensure_ascii=False)

    def summary(self) -> str:
        """One-screen human-readable summary."""
        types = ", ".join(f"{k}={v}" for k, v in sorted(self.joint_type_counts.items()))
        consistency = (
            "internally consistent"
            if self.numerically_consistent()
            else "NOT consistent, geometry only"
        )
        validation = (
            "validated on the machine"
            if self.hardware_validated
            else "NOT validated on the machine"
        )
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
            f"total mass  : {self.total_mass_kg:.7g} kg",
            f"numbers     : {consistency}",
            f"hardware    : {validation}",
        ]
        if self.massless_moving_links:
            lines.append(
                f"massless     : {len(self.massless_moving_links)} moving link(s), of which "
                f"{len(self.massless_moving_parts)} draw geometry "
                f"(e.g. {', '.join(self.massless_moving_links[:3])})"
            )
        if self.movable_subtree_masses_kg:
            heaviest = sorted(self.movable_subtree_masses_kg.items(), key=lambda kv: -kv[1])[:5]
            lines.append("heaviest subtrees:")
            lines.extend(f"  {joint:28} {mass:8.4f} kg" for joint, mass in heaviest)
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
    links_with_geometry: set[str] = set()

    for link in root.findall("link"):
        name = link.get("name", "")
        audit.link_names.append(name)
        inertial = link.find("inertial")
        if inertial is None:
            audit.links_without_inertial.append(name)
        else:
            mass_el = inertial.find("mass")
            if mass_el is not None:
                mass = _optional_float(mass_el.get("value")) or 0.0
                audit.total_mass_kg += mass
                audit.link_masses_kg[name] = mass
            tensor_el = inertial.find("inertia")
            if tensor_el is not None:
                values = {
                    key: _optional_float(tensor_el.get(key)) or 0.0
                    for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
                }
                problem = _inertia_problem(name, values)
                if problem is not None:
                    audit.invalid_inertias.append(problem)
        if link.find("collision") is not None:
            audit.has_collision_geometry = True
        if link.find("visual") is not None or link.find("collision") is not None:
            links_with_geometry.add(name)

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

    # Where mass is allowed to be absent, and where it is not.  A root link is
    # the machine's attachment to the world; below a movable joint, everything
    # is a part that a torque has to move.
    audit.root_links_without_inertial = [
        name for name in audit.links_without_inertial if name in set(audit.root_links)
    ]
    children_by_link: Dict[str, List[JointInfo]] = {}
    for joint in audit.joints:
        children_by_link.setdefault(joint.parent, []).append(joint)

    def _subtree(link: str) -> List[str]:
        """Every link at or below ``link``, following the joint tree."""
        seen: List[str] = []
        stack = [link]
        visited = set()
        while stack:
            current = stack.pop()
            if current in visited:
                continue
            visited.add(current)
            seen.append(current)
            stack.extend(j.child for j in children_by_link.get(current, ()))
        return seen

    moving_links: set[str] = set()
    for joint in audit.joints:
        if not joint.is_movable:
            continue
        below = _subtree(joint.child)
        moving_links.update(below)
        audit.movable_subtree_masses_kg[joint.name] = sum(
            audit.link_masses_kg.get(name, 0.0) for name in below
        )

    audit.massless_moving_links = [
        name
        for name in audit.link_names
        if name in moving_links and audit.link_masses_kg.get(name, 0.0) <= NEGLIGIBLE_MASS_KG
    ]
    # Of those, the ones that are actually made of something.  A link with no
    # geometry is a coordinate frame and weighs nothing by construction; a link
    # that draws a part and weighs nothing is a hole in the model.  Telling them
    # apart structurally beats guessing from names, which in a CAD export are
    # whatever the assembly tree happened to be called.
    audit.massless_moving_parts = [
        name for name in audit.massless_moving_links if name in links_with_geometry
    ]

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
    if not audit.numerically_consistent():
        audit.warnings.append(
            f"Total mass is {audit.total_mass_kg:.7g} kg, with "
            f"{len(audit.massless_moving_parts)} massless moving part(s) and "
            f"{len(audit.invalid_inertias)} unusable inertia tensor(s). Treat this as geometry "
            "only; do not edit the masses to make it look valid."
        )
    elif audit.massless_moving_links:
        audit.warnings.append(
            f"{len(audit.massless_moving_links)} link(s) below a movable joint carry no mass: "
            f"{', '.join(audit.massless_moving_links[:5])}"
            f"{' ...' if len(audit.massless_moving_links) > 5 else ''}. None of them draws any "
            "geometry, so they are coordinate frames or duplicated CAD sub-solids rather than "
            "unweighed parts -- worth confirming against the assembly, not a fault in itself."
        )
    if not audit.hardware_validated:
        audit.warnings.append(
            "These numbers have not been checked against the actual machine. Consistent "
            "arithmetic is not a measurement; do not enable current output on this basis."
        )
    for problem in audit.invalid_inertias:
        audit.warnings.append(problem)
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
